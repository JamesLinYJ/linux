/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Everest ES7210 4-ch ADC codec register definitions and clock table.
 *
 * Register facts are verified against the official Espressif component
 * espressif/esp_codec_dev v1.3.6 (Apache-2.0, component hash
 * 420a8a931f8bdfc74ae89c4d2ce634823d10e1865b1e9bdb8428bfe4a1060def,
 * locked in sources.lock as "esp-codec-dev"); the clock coefficient
 * table is mechanically transcribed from coeff_div[] in
 * device/es7210/es7210.c and pinned by a checksum. See
 * docs/clean-room.md.
 */
#ifndef _ES7210_H
#define _ES7210_H

#include <linux/errno.h>
#include <linux/types.h>

#define ES7210_RESET_REG		0x00
#define ES7210_CLOCK_OFF_REG		0x01
#define ES7210_MAINCLK_REG		0x02
#define ES7210_MASTER_CLK_REG		0x03
#define ES7210_LRCK_DIVH_REG		0x04
#define ES7210_LRCK_DIVL_REG		0x05
#define ES7210_POWER_DOWN_REG		0x06
#define ES7210_OSR_REG			0x07
#define ES7210_MODE_CONFIG_REG		0x08
#define ES7210_SDP_INTERFACE1_REG	0x11
#define ES7210_SDP_INTERFACE2_REG	0x12
#define ES7210_ANALOG_REG		0x40
#define ES7210_MIC1_POWER_REG		0x47
#define ES7210_MIC2_POWER_REG		0x48
#define ES7210_MIC3_POWER_REG		0x49
#define ES7210_MIC4_POWER_REG		0x4a

/* SDP_INTERFACE1 format bits (config_fmt): I2S normal = 0x00 */
#define ES7210_SDP_FMT_MASK		0x03
#define ES7210_SDP_FMT_I2S		0x00

/* SDP_INTERFACE1 sample width bits (set_bits) */
#define ES7210_SDP_BITS_MASK		0xe0
#define ES7210_SDP_BITS_16		0x60
#define ES7210_SDP_BITS_24		0x00
#define ES7210_SDP_BITS_32		0x80

struct es7210_coeff {
	u32 mclk;
	u32 lrck;
	u8 adc_div;
	u8 dll;
	u8 doubler;
	u8 osr;
	u8 mclk_src;
	u8 lrck_h;
	u8 lrck_l;
};

/*
 * Clock coefficient table transcribed from coeff_div[] in the official
 * esp_codec_dev v1.3.6 component (mclk, lrck, adc_div, dll, doubler,
 * osr, mclk_src, lrck_h, lrck_l; the original ss_ds column is dropped
 * as the Linux driver does not use it).
 */
static const struct es7210_coeff es7210_coeff_table[] = {
	{ 12288000,  8000, 0x03, 1, 0, 0x20, 0x00, 0x06, 0x00 },
	{ 16384000,  8000, 0x04, 1, 0, 0x20, 0x00, 0x08, 0x00 },
	{ 19200000,  8000, 0x1e, 0, 1, 0x28, 0x00, 0x09, 0x60 },
	{  4096000,  8000, 0x01, 1, 0, 0x20, 0x00, 0x02, 0x00 },
	{ 11289600, 11025, 0x02, 1, 0, 0x20, 0x00, 0x01, 0x00 },
	{ 12288000, 12000, 0x02, 1, 0, 0x20, 0x00, 0x04, 0x00 },
	{ 19200000, 12000, 0x14, 0, 1, 0x28, 0x00, 0x06, 0x40 },
	{  4096000, 16000, 0x01, 1, 1, 0x20, 0x00, 0x01, 0x00 },
	{ 19200000, 16000, 0x0a, 0, 0, 0x1e, 0x00, 0x04, 0x80 },
	{ 16384000, 16000, 0x02, 1, 0, 0x20, 0x00, 0x04, 0x00 },
	{ 12288000, 16000, 0x03, 1, 1, 0x20, 0x00, 0x03, 0x00 },
	{ 11289600, 22050, 0x01, 1, 0, 0x20, 0x00, 0x02, 0x00 },
	{ 12288000, 24000, 0x01, 1, 0, 0x20, 0x00, 0x02, 0x00 },
	{ 19200000, 24000, 0x0a, 0, 1, 0x28, 0x00, 0x03, 0x20 },
	{  8192000, 32000, 0x01, 1, 1, 0x20, 0x00, 0x01, 0x00 },
	{ 12288000, 32000, 0x03, 0, 0, 0x20, 0x00, 0x01, 0x80 },
	{ 16384000, 32000, 0x01, 1, 0, 0x20, 0x00, 0x02, 0x00 },
	{ 19200000, 32000, 0x05, 0, 0, 0x1e, 0x00, 0x02, 0x58 },
	{ 11289600, 44100, 0x01, 1, 1, 0x20, 0x00, 0x01, 0x00 },
	{ 12288000, 48000, 0x01, 1, 1, 0x20, 0x00, 0x01, 0x00 },
	{ 19200000, 48000, 0x05, 0, 1, 0x28, 0x00, 0x01, 0x90 },
	{ 16384000, 64000, 0x01, 1, 0, 0x20, 0x00, 0x01, 0x00 },
	{ 19200000, 64000, 0x05, 0, 1, 0x1e, 0x00, 0x01, 0x2c },
	{ 11289600, 88200, 0x01, 1, 1, 0x20, 0x00, 0x00, 0x80 },
	{ 12288000, 96000, 0x01, 1, 1, 0x20, 0x00, 0x00, 0x80 },
	{ 19200000, 96000, 0x05, 0, 1, 0x28, 0x00, 0x00, 0xc8 },
};

#define ES7210_COEFF_COUNT ARRAY_SIZE(es7210_coeff_table)

/* Golden checksum pinning the transcription to the official component */
#define ES7210_COEFF_GOLDEN_CHECKSUM 0x15e8bc15

static inline u32 es7210_coeff_checksum(void)
{
	u32 sum = 0;
	size_t i;

	for (i = 0; i < ES7210_COEFF_COUNT; i++) {
		const struct es7210_coeff *c = &es7210_coeff_table[i];

		sum += c->mclk;
		sum += c->lrck;
		sum += c->adc_div;
		sum += c->dll;
		sum += c->doubler;
		sum += c->osr;
		sum += c->mclk_src;
		sum += c->lrck_h;
		sum += c->lrck_l;
	}
	return sum;
}

/* Find the coefficient row for a given MCLK/sample rate pair */
static inline const struct es7210_coeff *
es7210_coeff_find(u32 mclk, u32 rate)
{
	size_t i;

	for (i = 0; i < ES7210_COEFF_COUNT; i++)
		if (es7210_coeff_table[i].mclk == mclk &&
		    es7210_coeff_table[i].lrck == rate)
			return &es7210_coeff_table[i];

	return NULL;
}

#endif /* _ES7210_H */
