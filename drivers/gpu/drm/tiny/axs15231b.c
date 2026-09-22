// SPDX-License-Identifier: GPL-2.0-only
/*
 * AXS Technology AXS15231B 172x640 (landscape 640x172) QSPI command-mode
 * LCD panel DRM driver
 *
 * The panel speaks a QSPI protocol with a 4-byte single-line command phase
 * [opcode][0x00][command][0x00] followed by a parameter or pixel phase; the
 * pixel phase runs on all four data lines. Protocol constants and the init
 * sequence live in axs15231b.h together with their provenance; see also
 * docs/clean-room.md.
 *
 * The Waveshare V2 board drives the panel from SPI3 with SPI mode 3 at
 * 40 MHz. Panel reset is the TCA9554 EXIO5 line (active-low), backlight is
 * the pwm-backlight node in the device tree.
 *
 * A DRM shadow plane holds a full-frame RGB565 buffer; updates stream the
 * coalesced, rotated native frame into panel memory. The panel TE signal is
 * not used yet: tearing behaviour must be validated on device (M7 Gate).
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/iosys-map.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/spi/spi.h>

#include <drm/drm_atomic_helper.h>
#include <drm/clients/drm_client_setup.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_format_helper.h>
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
	u32 rotation;

	u8 command[AXS15231B_CMD_HEADER_LEN + 32];
	u8 *pixels;
	u8 *source_pixels;
	/* Serializes native-frame ownership, panel I/O and enable/disable. */
	struct mutex lock;
	struct delayed_work refresh;
	bool dirty;
	bool prepared;
	bool visible;
	u8 header[AXS15231B_CMD_HEADER_LEN];
	struct spi_transfer *xfers; /* header plus bounded native-frame chunks */
	struct spi_message msg;
};

static inline struct axs15231b *drm_to_axs15231b(struct drm_device *drm)
{
	return container_of(drm, struct axs15231b, drm);
}

static int axs15231b_write_cmd(struct axs15231b *panel, u8 cmd,
			       const u8 *param, size_t param_len)
{
	struct spi_transfer xfers[2] = {};
	u8 *buf = panel->command;

	if (param_len > 32)
		return -EINVAL;
	axs15231b_cmd_header(buf, AXS15231B_OP_WRITE_CMD, cmd);
	if (param_len)
		memcpy(buf + AXS15231B_CMD_HEADER_LEN, param, param_len);
	xfers[0].tx_buf = buf;
	xfers[0].len = AXS15231B_CMD_HEADER_LEN;
	xfers[0].tx_nbits = SPI_NBITS_SINGLE;
	xfers[1].tx_buf = buf + AXS15231B_CMD_HEADER_LEN;
	xfers[1].len = param_len;
	/* Opcode 0x02 keeps parameters single-line; only 0x32 is quad. */
	xfers[1].tx_nbits = SPI_NBITS_SINGLE;
	return spi_sync_transfer(panel->spi, xfers, param_len ? 2 : 1);
}

static int axs15231b_panel_init(struct axs15231b *panel)
{
	unsigned int i;
	int ret;

	/* Waveshare V2 reset and short init; panel retains factory calibration. */
	gpiod_set_value_cansleep(panel->reset_gpio, 0);
	msleep(30);
	gpiod_set_value_cansleep(panel->reset_gpio, 1);
	msleep(250);
	gpiod_set_value_cansleep(panel->reset_gpio, 0);
	msleep(30);
	for (i = 0; i < AXS15231B_INIT_CMDS_COUNT; i++) {
		const struct axs15231b_init_cmd *entry = &axs15231b_init_cmds[i];

		ret = axs15231b_write_cmd(panel, entry->cmd, entry->data, entry->len);
		if (ret)
			return ret;
		if (entry->delay_ms)
			msleep(entry->delay_ms);
	}
	return 0;
}

/* Stream a full native frame from RAMWR's origin, as in the vendor driver. */
static int axs15231b_upload_frame(struct axs15231b *panel)
{
	const u8 columns[] = { 0, 0, (AXS15231B_WIDTH - 1) >> 8,
			      (AXS15231B_WIDTH - 1) & 0xff };
	size_t total = AXS15231B_WIDTH * AXS15231B_HEIGHT * 2;
	size_t limit = min_t(size_t, spi_max_transfer_size(panel->spi), 32768);
	size_t offset = 0;
	unsigned int index = 1;
	int ret;

	if (!limit)
		return -EINVAL;
	ret = axs15231b_write_cmd(panel, AXS15231B_CMD_CASET, columns, sizeof(columns));
	if (ret)
		return ret;
	axs15231b_cmd_header(panel->header, AXS15231B_OP_WRITE_COLOR,
			     AXS15231B_CMD_RAMWR);
	spi_message_init(&panel->msg);
	panel->xfers[0].tx_buf = panel->header;
	panel->xfers[0].len = AXS15231B_CMD_HEADER_LEN;
	panel->xfers[0].tx_nbits = SPI_NBITS_SINGLE;
	spi_message_add_tail(&panel->xfers[0], &panel->msg);
	while (offset < total) {
		struct spi_transfer *xfer = &panel->xfers[index++];

		xfer->tx_buf = panel->pixels + offset;
		xfer->len = min(limit, total - offset);
		xfer->tx_nbits = SPI_NBITS_QUAD;
		xfer->cs_change = false;
		spi_message_add_tail(xfer, &panel->msg);
		offset += xfer->len;
	}
	return spi_sync(panel->spi, &panel->msg);
}

/* Coalesce DRM damage without retaining a framebuffer mapping past update. */
static void axs15231b_refresh_work(struct work_struct *work)
{
	struct axs15231b *panel = container_of(to_delayed_work(work),
					     struct axs15231b, refresh);
	int ret, idx;

	if (!drm_dev_enter(&panel->drm, &idx))
		return;
	mutex_lock(&panel->lock);
	if (!panel->prepared || !panel->dirty)
		goto out;
	ret = axs15231b_upload_frame(panel);
	if (ret)
		goto error;
	if (!panel->visible && panel->backlight) {
		ret = backlight_enable(panel->backlight);
		if (ret)
			goto error;
	}
	if (!panel->visible)
		dev_info(panel->drm.dev, "first native frame uploaded\n");
	panel->visible = true;
	panel->dirty = false;
	goto out;
error:
	/* Break the error -> DRM console damage -> upload error feedback loop. */
	panel->prepared = false;
	dev_err_ratelimited(panel->drm.dev, "frame upload failed: %d\n", ret);
out:
	mutex_unlock(&panel->lock);
	drm_dev_exit(idx);
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

	mutex_lock(&panel->lock);
	ret = axs15231b_panel_init(panel);
	if (ret)
		dev_err(pipe->crtc.dev->dev, "panel init failed: %d\n", ret);
	panel->prepared = !ret;
	panel->visible = false;
	mutex_unlock(&panel->lock);

	drm_dev_exit(idx);
}

static void axs15231b_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct axs15231b *panel = drm_to_axs15231b(pipe->crtc.dev);
	int ret;

	cancel_delayed_work_sync(&panel->refresh);
	mutex_lock(&panel->lock);
	panel->dirty = false;
	panel->prepared = false;
	panel->visible = false;
	if (panel->backlight)
		backlight_disable(panel->backlight);

	ret = axs15231b_write_cmd(panel, AXS15231B_CMD_DISPOFF, NULL, 0);
	if (ret) {
		dev_err(pipe->crtc.dev->dev, "DISPOFF failed: %d\n", ret);
		goto out;
	}
	axs15231b_write_cmd(panel, AXS15231B_CMD_SLPIN, NULL, 0);
out:
	mutex_unlock(&panel->lock);
}

static void axs15231b_pipe_update(struct drm_simple_display_pipe *pipe,
				  struct drm_plane_state *old_state)
{
	struct axs15231b *panel = drm_to_axs15231b(pipe->crtc.dev);
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_shadow_plane_state *shadow = to_drm_shadow_plane_state(state);
	unsigned int pitch = panel->mode.hdisplay * AXS15231B_BYTES_PER_PIXEL;
	unsigned int x, y;
	struct iosys_map dst;
	struct drm_rect rect;
	int ret, idx;

	if (!pipe->crtc.state->active)
		return;
	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;
	mutex_lock(&panel->lock);
	if (!panel->prepared)
		goto out;
	if (!drm_atomic_helper_damage_merged(old_state, state, &rect))
		goto out;
	if (!panel->visible)
		drm_rect_init(&rect, 0, 0, panel->mode.hdisplay, panel->mode.vdisplay);

	ret = drm_gem_fb_begin_cpu_access(state->fb, DMA_FROM_DEVICE);
	if (ret)
		goto error;
	iosys_map_set_vaddr(&dst, panel->source_pixels + rect.y1 * pitch +
			   rect.x1 * AXS15231B_BYTES_PER_PIXEL);
	switch (state->fb->format->format) {
	case DRM_FORMAT_XRGB8888:
		drm_fb_xrgb8888_to_rgb565be(&dst, &pitch, shadow->data,
					    state->fb, &rect, &shadow->fmtcnv_state);
		break;
	case DRM_FORMAT_RGB565:
		drm_fb_swab(&dst, &pitch, shadow->data, state->fb, &rect,
			    true, &shadow->fmtcnv_state);
		break;
	default:
		drm_fb_memcpy(&dst, &pitch, shadow->data, state->fb, &rect);
		break;
	}
	drm_gem_fb_end_cpu_access(state->fb, DMA_FROM_DEVICE);

	/* Rotate the changed logical pixels into the persistent native frame. */
	for (y = rect.y1; y < rect.y2; y++) {
		for (x = rect.x1; x < rect.x2; x++) {
			size_t src = y * pitch + x * 2;
			size_t dst_offset = axs15231b_native_index(x, y, panel->rotation) * 2;

			memcpy(panel->pixels + dst_offset, panel->source_pixels + src, 2);
		}
	}
	panel->dirty = true;
	queue_delayed_work(system_long_wq, &panel->refresh, msecs_to_jiffies(20));
	goto out;
error:
	dev_err_ratelimited(pipe->crtc.dev->dev, "pixel update failed: %d\n", ret);
out:
	mutex_unlock(&panel->lock);
	drm_dev_exit(idx);
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
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_RGB565,
};

static const struct drm_display_mode axs15231b_modes[] = {
	{ DRM_SIMPLE_MODE(172, 640, 23, 86) },
	{ DRM_SIMPLE_MODE(640, 172, 86, 23) },
};

DEFINE_DRM_GEM_DMA_FOPS(axs15231b_fops);

/* Cached PSRAM backs the shadow; SPI owns streaming DMA/cache transitions. */
static struct drm_gem_object *
axs15231b_gem_create_object(struct drm_device *drm, size_t size)
{
	struct drm_gem_dma_object *obj = kzalloc_obj(*obj);

	if (!obj)
		return ERR_PTR(-ENOMEM);
	obj->map_noncoherent = true;
	return &obj->base;
}

static const struct drm_driver axs15231b_drm_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &axs15231b_fops,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	.gem_create_object = axs15231b_gem_create_object,
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
	drm_dev_set_dma_dev(drm, spi->controller->dev.parent);
	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.funcs = &axs15231b_mode_config_funcs;

	mutex_init(&panel->lock);
	INIT_DELAYED_WORK(&panel->refresh, axs15231b_refresh_work);
	panel->spi = spi;
	panel->pixels = devm_kzalloc(dev, AXS15231B_WIDTH * AXS15231B_HEIGHT *
				     AXS15231B_BYTES_PER_PIXEL, GFP_KERNEL);
	panel->source_pixels = devm_kzalloc(dev, AXS15231B_WIDTH * AXS15231B_HEIGHT *
					    AXS15231B_BYTES_PER_PIXEL, GFP_KERNEL);
	if (!panel->pixels || !panel->source_pixels)
		return -ENOMEM;

	/* Optional reset line; deasserted at request time */
	panel->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						    GPIOD_OUT_LOW);
	if (IS_ERR(panel->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(panel->reset_gpio),
				     "failed to get reset GPIO\n");

	panel->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(panel->backlight))
		return dev_err_probe(dev, PTR_ERR(panel->backlight),
				     "failed to get backlight\n");

	if (!spi_max_transfer_size(spi))
		return -EINVAL;
	/* One header plus chunks bounded by the SPI controller transfer limit. */
	panel->xfers = devm_kcalloc(dev, 1 + DIV_ROUND_UP(AXS15231B_WIDTH *
				    AXS15231B_HEIGHT * 2, min_t(size_t,
				    spi_max_transfer_size(spi), 32768)),
				    sizeof(*panel->xfers), GFP_KERNEL);
	if (!panel->xfers)
		return -ENOMEM;

	device_property_read_u32(dev, "rotation", &panel->rotation);
	if (panel->rotation != 0 && panel->rotation != 90 &&
	    panel->rotation != 180 && panel->rotation != 270)
		return dev_err_probe(dev, -EINVAL, "unsupported rotation\n");
	panel->mode = axs15231b_modes[(panel->rotation / 90) % 2];
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

	drm_plane_enable_fb_damage_clips(&panel->pipe.plane);
	drm->mode_config.preferred_depth = 16;
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
