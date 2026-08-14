/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ESP32S3_GDMA_H
#define _ESP32S3_GDMA_H

#include <linux/bitfield.h>
#include <linux/dma-mapping.h>
#include <linux/overflow.h>

#define ESP32S3_GDMA_DESC_SIZE		GENMASK(11, 0)
#define ESP32S3_GDMA_DESC_LENGTH	GENMASK(23, 12)
#define ESP32S3_GDMA_DESC_EOF		BIT(30)
#define ESP32S3_GDMA_DESC_OWNER		BIT(31)
#define ESP32S3_GDMA_DESC_MAX_LEN	4092

/* Link base registers omit the fixed 0x3fc upper internal-SRAM bits. */
#define ESP32S3_GDMA_DESC_ADDR_PREFIX	0x3fc00000
#define ESP32S3_GDMA_DESC_ADDR_MASK	GENMASK(19, 0)

static inline bool
esp32s3_gdma_desc_addr_valid(dma_addr_t addr, size_t size,
			     phys_addr_t pool_start, phys_addr_t pool_end)
{
	phys_addr_t end;

	if (!size || check_add_overflow((phys_addr_t)addr, size - 1, &end))
		return false;
	if ((addr & ~ESP32S3_GDMA_DESC_ADDR_MASK) !=
	    ESP32S3_GDMA_DESC_ADDR_PREFIX)
		return false;

	return addr >= pool_start && end <= pool_end;
}

static inline bool esp32s3_gdma_data_addr_valid(dma_addr_t addr, size_t len)
{
	dma_addr_t end;

	if (!len || !IS_ALIGNED(addr, 4) || !IS_ALIGNED(len, 4))
		return false;
	if (check_add_overflow(addr, len - 1, &end))
		return false;

	return !upper_32_bits(addr) && !upper_32_bits(end);
}

static inline u32 esp32s3_gdma_desc_flags(size_t length, bool tx, bool last)
{
	u32 value = FIELD_PREP(ESP32S3_GDMA_DESC_SIZE, length) |
		ESP32S3_GDMA_DESC_OWNER;

	if (tx)
		value |= FIELD_PREP(ESP32S3_GDMA_DESC_LENGTH, length);
	if (tx && last)
		value |= ESP32S3_GDMA_DESC_EOF;

	return value;
}

#endif /* _ESP32S3_GDMA_H */
