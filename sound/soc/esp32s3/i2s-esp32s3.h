/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ESP32-S3 I2S controller register definitions and divider math.
 *
 * Register facts are verified against the locked ESP-IDF v5.5.3
 * (sources.lock "esp-idf-xtensa-config" commit 2c211b236707):
 *  - offsets: soc/esp32s3/register/soc/i2s_reg.h
 *  - bit layouts: soc/esp32s3/register/soc/i2s_struct.h (rx_conf/tx_conf/
 *    rx_conf1/tx_conf1/rx_clkm_conf/tx_clkm_conf unions)
 *  - clock source encodings: hal/esp32s3/include/hal/i2s_ll.h
 *    (0 XTAL, 1 PLL_240M, 2 PLL_160M, 3 external MCLK)
 *  - divider coefficient encoding: i2s_ll_tx_set_mclk()
 */
#ifndef _I2S_ESP32S3_H
#define _I2S_ESP32S3_H

#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/math64.h>
#include <linux/types.h>

#define I2S_ESP32S3_FIFO_WR		0x0000
#define I2S_ESP32S3_FIFO_RD		0x0004
#define I2S_ESP32S3_INT_ENA		0x0014
#define I2S_ESP32S3_INT_CLR		0x0018
#define I2S_ESP32S3_RX_CONF		0x0020
#define I2S_ESP32S3_TX_CONF		0x0024
#define I2S_ESP32S3_RX_CONF1		0x0028
#define I2S_ESP32S3_TX_CONF1		0x002c
#define I2S_ESP32S3_RX_CLKM_CONF	0x0030
#define I2S_ESP32S3_TX_CLKM_CONF	0x0034
#define I2S_ESP32S3_RX_CLKM_DIV_CONF	0x0038
#define I2S_ESP32S3_TX_CLKM_DIV_CONF	0x003c

/* rx_conf / tx_conf */
#define I2S_ESP32S3_CONF_FIFO_RESET	BIT(1)
#define I2S_ESP32S3_CONF_START		BIT(2)
#define I2S_ESP32S3_CONF_SLAVE_MOD	BIT(3)
#define I2S_ESP32S3_CONF_MONO		BIT(5)
#define I2S_ESP32S3_CONF_BIG_ENDIAN	BIT(7)
#define I2S_ESP32S3_CONF_UPDATE		BIT(8)

/* rx_conf1 / tx_conf1: bits_mod = valid data bits per channel - 1 */
#define I2S_ESP32S3_CONF1_BCK_DIV_SHIFT	7
#define I2S_ESP32S3_CONF1_BCK_DIV_MASK	GENMASK(12, 7)
#define I2S_ESP32S3_CONF1_BITS_MOD_SHIFT	13
#define I2S_ESP32S3_CONF1_BITS_MOD_MASK	GENMASK(17, 13)
#define I2S_ESP32S3_CONF1_MSB_SHIFT	BIT(30)

/* rx_clkm_conf / tx_clkm_conf */
#define I2S_ESP32S3_CLKM_DIV_SHIFT	0
#define I2S_ESP32S3_CLKM_DIV_MASK	GENMASK(7, 0)
#define I2S_ESP32S3_CLKM_ACTIVE		BIT(26)
#define I2S_ESP32S3_CLKM_SEL_SHIFT	27
#define I2S_ESP32S3_CLKM_SEL_MASK	GENMASK(28, 27)

/* rx_clkm_div_conf / tx_clkm_div_conf */
#define I2S_ESP32S3_CLKM_DIV_Y_SHIFT	9
#define I2S_ESP32S3_CLKM_DIV_X_SHIFT	18
#define I2S_ESP32S3_CLKM_DIV_YN1_SHIFT	27

/* Clock source selection (i2s_ll.h encodings) */
#define I2S_ESP32S3_CLK_SRC_PLL_160M	2

/* Rational approximation denominator for the fractional divider */
#define I2S_ESP32S3_DIV_DENOM		1000

/* Codec MCLK multiple: 256x the sample rate (ES8311/ES7210 tables) */
#define I2S_ESP32S3_MCLK_MULTIPLE	256

struct i2s_esp32s3_div {
	unsigned int integer;
	unsigned int numerator;
	unsigned int denominator;
	unsigned int x;
	unsigned int y;
	unsigned int z;
	unsigned int yn1;
};

/*
 * f_mclk = f_in / (N + num/den); the board codecs (ES8311/ES7210) run
 * from a 256x MCLK. The fractional part num/den is encoded into the
 * hardware coefficients (x, y, z, yn1) exactly as the official
 * i2s_ll_tx_set_mclk() does. num is rounded to the nearest 1/1000,
 * which keeps the MCLK within 0.05% for the supported rates.
 */
static inline int i2s_esp32s3_calc_div(unsigned long source_rate,
				       unsigned int rate,
				       unsigned int mclk_multiple,
				       struct i2s_esp32s3_div *div)
{
	unsigned long long total;
	unsigned int integer, num, den = I2S_ESP32S3_DIV_DENOM;
	unsigned int mclk;

	if (!rate || !mclk_multiple)
		return -EINVAL;

	mclk = rate * mclk_multiple;
	if (source_rate < mclk)
		return -EINVAL;

	/* total = source_rate * den / mclk */
	total = (unsigned long long)source_rate * den;
	total = DIV_ROUND_CLOSEST_ULL(total, mclk);
	if (total < den || total > 0x100 * den)
		return -EINVAL;

	/* Xtensa has no 64-bit hardware division: use the kernel helpers */
	integer = div_u64(total, den);
	num = (unsigned int)(total - (unsigned long long)integer * den);
	if (num == 0) {
		/* Exact integer division: hardware coefficients are zero */
		div->integer = integer;
		div->numerator = 0;
		div->denominator = den;
		div->x = 0;
		div->y = 0;
		div->z = 0;
		div->yn1 = 0;
		return 0;
	}

	/* Official encoding from i2s_ll_tx_set_mclk() */
	div->integer = integer;
	div->numerator = num;
	div->denominator = den;
	div->yn1 = num * 2 > den;
	div->z = div->yn1 ? den - num : num;
	div->x = den / div->z - 1;
	div->y = den % div->z;

	return 0;
}

#endif /* _I2S_ESP32S3_H */
