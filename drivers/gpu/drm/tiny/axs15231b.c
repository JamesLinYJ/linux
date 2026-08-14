// SPDX-License-Identifier: GPL-2.0-only
/*
 * AXS Technology AXS15231B 172x640 (landscape 640x172) QSPI command-mode
 * LCD panel DRM driver
 *
 * The panel speaks a QSPI protocol with a 4-byte single-line command phase
 * [opcode][command][0x00][0x00] followed by a parameter or pixel phase; the
 * pixel phase runs on all four data lines. Protocol constants and the init
 * sequence live in axs15231b.h together with their provenance; see also
 * docs/clean-room.md.
 *
 * The Waveshare V2 board drives the panel from SPI3 with SPI mode 3 at
 * 40 MHz. Panel reset is the TCA9554 EXIO5 line (active-low), backlight is
 * the pwm-backlight node in the device tree.
 *
 * A DRM shadow plane holds a full-frame RGB565 buffer; updates stream the
 * merged damage rectangle into panel frame memory. The panel TE signal is
 * not used yet: tearing behaviour must be validated on device (M7 Gate).
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iosys-map.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/spi/spi.h>

#include <drm/drm_atomic_helper.h>
#include <drm/clients/drm_client_setup.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_modes.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include "axs15231b.h"

struct axs15231b {
	struct drm_device drm;
	struct spi_device *spi;
	struct gpio_desc *reset_gpio;
	struct backlight_device *backlight;
	struct drm_simple_display_pipe pipe;
	struct drm_connector connector;
	struct drm_display_mode mode;

	u8 header[AXS15231B_CMD_HEADER_LEN];
	struct spi_transfer *xfers; /* 1 header + one per row */
	struct spi_message msg;
};

static inline struct axs15231b *drm_to_axs15231b(struct drm_device *drm)
{
	return container_of(drm, struct axs15231b, drm);
}

static int axs15231b_write_cmd(struct axs15231b *panel, u8 cmd,
			       const u8 *param, size_t param_len)
{
	u8 buf[AXS15231B_CMD_HEADER_LEN + 32];

	if (param_len > 32)
		return -EINVAL;

	axs15231b_cmd_header(buf, AXS15231B_OP_WRITE_CMD, cmd);
	memcpy(buf + AXS15231B_CMD_HEADER_LEN, param, param_len);

	return spi_write(panel->spi, buf, AXS15231B_CMD_HEADER_LEN + param_len);
}

static int axs15231b_write_window(struct axs15231b *panel, u8 cmd,
				  u16 start, u16 end)
{
	u8 buf[AXS15231B_CMD_HEADER_LEN + AXS15231B_WINDOW_PARAM_LEN];

	axs15231b_cmd_header(buf, AXS15231B_OP_WRITE_CMD, cmd);
	buf[AXS15231B_CMD_HEADER_LEN + 0] = start >> 8;
	buf[AXS15231B_CMD_HEADER_LEN + 1] = start & 0xff;
	buf[AXS15231B_CMD_HEADER_LEN + 2] = end >> 8;
	buf[AXS15231B_CMD_HEADER_LEN + 3] = end & 0xff;

	return spi_write(panel->spi, buf, sizeof(buf));
}

static int axs15231b_panel_init(struct axs15231b *panel)
{
	const struct axs15231b_init_cmd *entry;
	size_t i;
	int ret;

	/*
	 * Reset timing follows the official component: deassert 10 ms,
	 * assert 10 ms, deassert and wait 120 ms. gpiod value semantics
	 * already account for the active-low reset line.
	 */
	gpiod_set_value_cansleep(panel->reset_gpio, 1);
	usleep_range(10000, 15000);
	gpiod_set_value_cansleep(panel->reset_gpio, 0);
	usleep_range(10000, 15000);
	gpiod_set_value_cansleep(panel->reset_gpio, 1);
	msleep(120);

	ret = axs15231b_write_cmd(panel, AXS15231B_CMD_SLPOUT, NULL, 0);
	if (ret)
		return ret;
	msleep(100);

	/* RGB element order, no mirroring; landscape is native in QSPI */
	{
		const u8 madctl = 0x00;

		ret = axs15231b_write_cmd(panel, AXS15231B_CMD_MADCTL,
					  &madctl, 1);
	}
	if (ret)
		return ret;

	{
		const u8 colmod = AXS15231B_COLMOD_RGB565;

		ret = axs15231b_write_cmd(panel, AXS15231B_CMD_COLMOD,
					  &colmod, 1);
	}
	if (ret)
		return ret;

	for (i = 0; i < AXS15231B_INIT_CMDS_COUNT; i++) {
		entry = &axs15231b_init_cmds[i];

		ret = axs15231b_write_cmd(panel, entry->cmd, entry->data,
					  entry->len);
		if (ret)
			return ret;
		if (entry->delay_ms)
			msleep(entry->delay_ms);
	}

	return 0;
}

/*
 * Stream one rectangle into panel frame memory. CASET/RASET select the
 * window; the pixel phase sends the color command header once and then one
 * quad transfer per row, chip select held across the whole message. Rows
 * use the panel's window pitch, which for partial rectangles differs from
 * the framebuffer pitch, hence row-wise transfers.
 */
static int axs15231b_write_rect(struct axs15231b *panel,
				struct iosys_map *map,
				u16 x1, u16 y1, u16 x2, u16 y2)
{
	size_t pitch = AXS15231B_WIDTH * AXS15231B_BYTES_PER_PIXEL;
	size_t row_bytes = (size_t)(x2 - x1 + 1) * AXS15231B_BYTES_PER_PIXEL;
	unsigned int rows = y2 - y1 + 1;
	unsigned int row;
	const u8 *pixels;
	int ret;

	if (WARN_ON_ONCE(map->is_iomem))
		return -EOPNOTSUPP;
	pixels = map->vaddr;
	if (!pixels)
		return -EINVAL;

	ret = axs15231b_write_window(panel, AXS15231B_CMD_CASET, x1, x2);
	if (ret)
		return ret;
	ret = axs15231b_write_window(panel, AXS15231B_CMD_RASET, y1, y2);
	if (ret)
		return ret;

	axs15231b_cmd_header(panel->header, AXS15231B_OP_WRITE_COLOR,
			     AXS15231B_CMD_RAMWR);

	spi_message_init(&panel->msg);
	panel->xfers[0].tx_buf = panel->header;
	panel->xfers[0].len = AXS15231B_CMD_HEADER_LEN;
	panel->xfers[0].tx_nbits = SPI_NBITS_SINGLE;
	panel->xfers[0].cs_change = false;
	spi_message_add_tail(&panel->xfers[0], &panel->msg);

	for (row = 0; row < rows; row++) {
		struct spi_transfer *xfer = &panel->xfers[row + 1];

		xfer->tx_buf = pixels + (y1 + row) * pitch + x1 *
			       AXS15231B_BYTES_PER_PIXEL;
		xfer->len = row_bytes;
		xfer->tx_nbits = SPI_NBITS_QUAD;
		xfer->cs_change = false;
		spi_message_add_tail(xfer, &panel->msg);
	}

	return spi_sync(panel->spi, &panel->msg);
}

static enum drm_mode_status
axs15231b_pipe_mode_valid(struct drm_simple_display_pipe *pipe,
			  const struct drm_display_mode *mode)
{
	struct axs15231b *panel = drm_to_axs15231b(pipe->crtc.dev);

	return drm_crtc_helper_mode_valid_fixed(&pipe->crtc, mode,
						&panel->mode);
}

static void axs15231b_pipe_enable(struct drm_simple_display_pipe *pipe,
				  struct drm_crtc_state *crtc_state,
				  struct drm_plane_state *plane_state)
{
	struct axs15231b *panel = drm_to_axs15231b(pipe->crtc.dev);
	int ret, idx;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	ret = axs15231b_panel_init(panel);
	if (ret)
		dev_err(pipe->crtc.dev->dev, "panel init failed: %d\n", ret);
	else if (panel->backlight)
		backlight_enable(panel->backlight);

	drm_dev_exit(idx);
}

static void axs15231b_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct axs15231b *panel = drm_to_axs15231b(pipe->crtc.dev);
	int ret;

	if (panel->backlight)
		backlight_disable(panel->backlight);

	ret = axs15231b_write_cmd(panel, AXS15231B_CMD_DISPOFF, NULL, 0);
	if (ret) {
		dev_err(pipe->crtc.dev->dev, "DISPOFF failed: %d\n", ret);
		return;
	}
	axs15231b_write_cmd(panel, AXS15231B_CMD_SLPIN, NULL, 0);
}

static void axs15231b_pipe_update(struct drm_simple_display_pipe *pipe,
				  struct drm_plane_state *old_state)
{
	struct axs15231b *panel = drm_to_axs15231b(pipe->crtc.dev);
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_shadow_plane_state *shadow_plane_state =
		to_drm_shadow_plane_state(state);
	struct drm_rect rect;
	int ret;

	if (!pipe->crtc.state->active)
		return;

	if (!drm_atomic_helper_damage_merged(old_state, state, &rect))
		return;

	ret = axs15231b_write_rect(panel, shadow_plane_state->data,
				   rect.x1, rect.y1, rect.x2, rect.y2);
	if (ret)
		dev_err_ratelimited(pipe->crtc.dev->dev,
				    "pixel update failed: %d\n", ret);
}

static const struct drm_simple_display_pipe_funcs axs15231b_pipe_funcs = {
	.mode_valid = axs15231b_pipe_mode_valid,
	.enable = axs15231b_pipe_enable,
	.disable = axs15231b_pipe_disable,
	.update = axs15231b_pipe_update,
	DRM_GEM_SIMPLE_DISPLAY_PIPE_SHADOW_PLANE_FUNCS,
};

static int axs15231b_connector_get_modes(struct drm_connector *connector)
{
	struct axs15231b *panel = drm_to_axs15231b(connector->dev);

	return drm_connector_helper_get_modes_fixed(connector, &panel->mode);
}

static const struct drm_connector_helper_funcs axs15231b_connector_hfuncs = {
	.get_modes = axs15231b_connector_get_modes,
};

static const struct drm_connector_funcs axs15231b_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_mode_config_funcs axs15231b_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static const u32 axs15231b_formats[] = {
	DRM_FORMAT_RGB565,
};

static const struct drm_display_mode axs15231b_mode = {
	DRM_SIMPLE_MODE(640, 172, 86, 23),
};

DEFINE_DRM_GEM_DMA_FOPS(axs15231b_fops);

static const struct drm_driver axs15231b_drm_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &axs15231b_fops,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	DRM_FBDEV_DMA_DRIVER_OPS,
	.name			= "axs15231b",
	.desc			= "AXS Technology AXS15231B QSPI panel",
	.major			= 1,
	.minor			= 0,
};

static int axs15231b_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct axs15231b *panel;
	struct drm_device *drm;
	int ret;

	if (!(spi->mode & SPI_CPOL) || !(spi->mode & SPI_CPHA))
		return dev_err_probe(dev, -EINVAL,
				     "panel requires SPI mode 3\n");

	panel = devm_drm_dev_alloc(dev, &axs15231b_drm_driver,
				   struct axs15231b, drm);
	if (IS_ERR(panel))
		return PTR_ERR(panel);

	drm = &panel->drm;
	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.funcs = &axs15231b_mode_config_funcs;

	panel->spi = spi;

	/* Optional reset line; deasserted at request time */
	panel->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						    GPIOD_OUT_HIGH);
	if (IS_ERR(panel->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(panel->reset_gpio),
				     "failed to get reset GPIO\n");

	panel->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(panel->backlight))
		return dev_err_probe(dev, PTR_ERR(panel->backlight),
				     "failed to get backlight\n");

	/* One header transfer plus one quad transfer per panel row */
	panel->xfers = devm_kcalloc(dev, 1 + AXS15231B_HEIGHT,
				    sizeof(*panel->xfers), GFP_KERNEL);
	if (!panel->xfers)
		return -ENOMEM;

	panel->mode = axs15231b_mode;
	drm->mode_config.min_width = panel->mode.hdisplay;
	drm->mode_config.max_width = panel->mode.hdisplay;
	drm->mode_config.min_height = panel->mode.vdisplay;
	drm->mode_config.max_height = panel->mode.vdisplay;

	drm_connector_helper_add(&panel->connector,
				 &axs15231b_connector_hfuncs);
	ret = drm_connector_init(drm, &panel->connector,
				 &axs15231b_connector_funcs,
				 DRM_MODE_CONNECTOR_DPI);
	if (ret)
		return ret;

	ret = drm_simple_display_pipe_init(drm, &panel->pipe,
					   &axs15231b_pipe_funcs,
					   axs15231b_formats,
					   ARRAY_SIZE(axs15231b_formats),
					   NULL, &panel->connector);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	spi_set_drvdata(spi, drm);

	drm_client_setup(drm, NULL);

	return 0;
}

static void axs15231b_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void axs15231b_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static const struct of_device_id axs15231b_of_match[] = {
	{ .compatible = "axs,axs15231b" },
	{ },
};
MODULE_DEVICE_TABLE(of, axs15231b_of_match);

static const struct spi_device_id axs15231b_id[] = {
	{ "axs15231b", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, axs15231b_id);

static struct spi_driver axs15231b_spi_driver = {
	.driver = {
		.name = "axs15231b",
		.of_match_table = axs15231b_of_match,
	},
	.id_table = axs15231b_id,
	.probe = axs15231b_probe,
	.remove = axs15231b_remove,
	.shutdown = axs15231b_shutdown,
};
module_spi_driver(axs15231b_spi_driver);

MODULE_DESCRIPTION("AXS Technology AXS15231B QSPI panel DRM driver");
MODULE_AUTHOR("ESP32-S3 Linux project");
MODULE_LICENSE("GPL");
