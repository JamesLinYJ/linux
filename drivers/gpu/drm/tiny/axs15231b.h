/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * AXS Technology AXS15231B 172x640 QSPI command-mode panel: protocol helpers
 *
 * The wire format and the init sequence are documented by the AXS15231B
 * datasheet V0.5 (locked in sources.lock as "axs15231b-datasheet") and the
 * official Espressif component espressif/esp_lcd_axs15231b v1.0.1~1
 * (Apache-2.0, component hash
 * e614bd75827e95800e39df1ba9474166060ab26c3807d06df9cc312a27021cda, locked in
 * sources.lock as "esp-lcd-axs15231b", upstream commit 59a708e5356f7922 in
 * espressif/esp-iot-solution). The short V2 initialization follows the Waveshare board demo at
 * commit 1c157e6e8e68b89fd4dc400f46bf1724cb64a57e; see docs/clean-room.md
 * for the provenance record.
 *
 * QSPI write format (datasheet V0.5 section 4.4): the command phase is a
 * 4-byte single-line stream [opcode][0x00][command][0x00] with
 * AXS15231B_OP_WRITE_CMD for parameters and AXS15231B_OP_WRITE_COLOR for
 * pixel data, followed by the data phase which for pixels uses all four
 * lines. The official Waveshare V2 demo (ESP-IDF esp_lcd_panel_io_spi with
 * lcd_cmd_bits=32, quad_mode) produces exactly this byte order on the wire.
 */
#ifndef _AXS15231B_H
#define _AXS15231B_H

#include <linux/errno.h>
#include <linux/limits.h>
#include <linux/overflow.h>
#include <linux/types.h>

/* QSPI command header opcodes (datasheet V0.5 section 4.4.1) */
#define AXS15231B_OP_WRITE_CMD	0x02
#define AXS15231B_OP_READ_CMD	0x0b
#define AXS15231B_OP_WRITE_COLOR	0x32

/* Waveshare V2 native panel memory geometry. */
#define AXS15231B_WIDTH	172
#define AXS15231B_HEIGHT	640
#define AXS15231B_BYTES_PER_PIXEL	2 /* RGB565 */

/* DCS command bytes used by this driver */
#define AXS15231B_CMD_SLPIN	0x10
#define AXS15231B_CMD_SLPOUT	0x11
#define AXS15231B_CMD_DISPOFF	0x28
#define AXS15231B_CMD_DISPON	0x29
#define AXS15231B_CMD_CASET	0x2a
#define AXS15231B_CMD_RASET	0x2b
#define AXS15231B_CMD_RAMWR	0x2c
#define AXS15231B_CMD_RAMWRC	0x3c
#define AXS15231B_CMD_MADCTL	0x36
#define AXS15231B_CMD_COLMOD	0x3a

#define AXS15231B_COLMOD_RGB565	0x55

/* Command header is 4 bytes; CASET/RASET carry 4 parameter bytes */
#define AXS15231B_CMD_HEADER_LEN	4
#define AXS15231B_WINDOW_PARAM_LEN	4

struct axs15231b_init_cmd {
	u8 cmd;
	u8 data[32];
	size_t len;
	unsigned int delay_ms;
};

/* Waveshare V2 short init plus the official component's pixel-format setup. */
static const struct axs15231b_init_cmd axs15231b_init_cmds[] = {
	{ AXS15231B_CMD_SLPOUT, { 0 }, 0, 100 },
	{ AXS15231B_CMD_MADCTL, { 0 }, 1, 0 },
	{ AXS15231B_CMD_COLMOD, { AXS15231B_COLMOD_RGB565 }, 1, 0 },
	{ AXS15231B_CMD_DISPON, { 0 }, 0, 100 },
};

#define AXS15231B_INIT_CMDS_COUNT ARRAY_SIZE(axs15231b_init_cmds)

/* Build the 4-byte single-line QSPI command header */
static inline void axs15231b_cmd_header(u8 *buf, u8 opcode, u8 cmd)
{
	buf[0] = opcode;
	buf[1] = 0x00;
	buf[2] = cmd;
	buf[3] = 0x00;
}

/* Logical coordinates to the native 172x640 pixel stream. */
static inline size_t axs15231b_native_index(u16 x, u16 y, u32 rotation)
{
	switch (rotation) {
	case 90:
		return (size_t)x * AXS15231B_WIDTH + AXS15231B_WIDTH - 1 - y;
	case 180:
		return ((size_t)AXS15231B_HEIGHT - 1 - y) * AXS15231B_WIDTH +
			AXS15231B_WIDTH - 1 - x;
	case 270:
		return ((size_t)AXS15231B_HEIGHT - 1 - x) * AXS15231B_WIDTH + y;
	default:
		return (size_t)y * AXS15231B_WIDTH + x;
	}
}

/*
 * Size in bytes of a DRM-style pixel rectangle. The lower-right coordinates
 * are exclusive, matching struct drm_rect and the official panel component.
 * Reject dimensions that exceed the panel or overflow the multiplication.
 */
static inline int axs15231b_rect_bytes(u16 x1, u16 y1, u16 x2, u16 y2,
				       size_t *bytes)
{
	size_t width, height, total, byte_count;
	int ret;

	if (x1 >= x2 || y1 >= y2 || x2 > AXS15231B_WIDTH ||
	    y2 > AXS15231B_HEIGHT)
		return -EINVAL;

	width = (size_t)x2 - x1;
	height = (size_t)y2 - y1;
	if (check_mul_overflow(width, height, &total))
		return -EOVERFLOW;

	ret = check_mul_overflow(total, AXS15231B_BYTES_PER_PIXEL,
				 &byte_count);
	if (ret)
		return -EOVERFLOW;
	if (bytes)
		*bytes = byte_count;

	return 0;
}

/*
 * Validate an init table structure: entry sizes within the storage array
 * and delays bounded.
 */
static inline bool
axs15231b_init_table_valid(const struct axs15231b_init_cmd *table,
			   size_t count)
{
	size_t i;

	for (i = 0; i < count; i++) {
		const struct axs15231b_init_cmd *entry = &table[i];

		if (entry->len > ARRAY_SIZE(entry->data))
			return false;
		if (entry->delay_ms > 1000)
			return false;
	}
	return true;
}

#endif /* _AXS15231B_H */
