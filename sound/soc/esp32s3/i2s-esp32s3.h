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
#include <linux/overflow.h>
#include <linux/rational.h>
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
#define I2S_ESP32S3_RX_EOF_NUM		0x0064

/* rx_conf / tx_conf */
#define I2S_ESP32S3_CONF_RESET		BIT(0)
#define I2S_ESP32S3_CONF_FIFO_RESET	BIT(1)
#define I2S_ESP32S3_CONF_START		BIT(2)
#define I2S_ESP32S3_CONF_SLAVE_MOD	BIT(3)
#define I2S_ESP32S3_CONF_MONO		BIT(5)
#define I2S_ESP32S3_CONF_BIG_ENDIAN	BIT(7)
#define I2S_ESP32S3_CONF_UPDATE		BIT(8)
#define I2S_ESP32S3_TX_CONF_SIG_LOOPBACK BIT(27)

/* rx_conf1 / tx_conf1: all widths are encoded as bits per slot minus one */
#define I2S_ESP32S3_CONF1_WS_WIDTH_SHIFT 0
#define I2S_ESP32S3_CONF1_WS_WIDTH_MASK	GENMASK(6, 0)
#define I2S_ESP32S3_CONF1_BCK_DIV_SHIFT	7
#define I2S_ESP32S3_CONF1_BCK_DIV_MASK	GENMASK(12, 7)
#define I2S_ESP32S3_CONF1_BITS_MOD_SHIFT	13
#define I2S_ESP32S3_CONF1_BITS_MOD_MASK	GENMASK(17, 13)
#define I2S_ESP32S3_CONF1_HALF_SAMPLE_SHIFT 18
#define I2S_ESP32S3_CONF1_HALF_SAMPLE_MASK GENMASK(23, 18)
#define I2S_ESP32S3_CONF1_CHAN_BITS_SHIFT 24
#define I2S_ESP32S3_CONF1_CHAN_BITS_MASK	GENMASK(28, 24)
#define I2S_ESP32S3_CONF1_MSB_SHIFT	BIT(29)

/* rx_clkm_conf / tx_clkm_conf */
#define I2S_ESP32S3_CLKM_DIV_SHIFT	0
#define I2S_ESP32S3_CLKM_DIV_MASK	GENMASK(7, 0)
#define I2S_ESP32S3_CLKM_ACTIVE		BIT(26)
#define I2S_ESP32S3_CLKM_SEL_SHIFT	27
#define I2S_ESP32S3_CLKM_SEL_MASK	GENMASK(28, 27)
#define I2S_ESP32S3_CLKM_MCLK_SEL	BIT(29)

/* rx_clkm_div_conf / tx_clkm_div_conf */
#define I2S_ESP32S3_CLKM_DIV_Y_SHIFT	9
#define I2S_ESP32S3_CLKM_DIV_X_SHIFT	18
#define I2S_ESP32S3_CLKM_DIV_YN1_SHIFT	27

#define I2S_ESP32S3_RX_EOF_NUM_MASK	GENMASK(11, 0)

/* Clock source selection (i2s_ll.h encodings) */
#define I2S_ESP32S3_CLK_SRC_PLL_160M	2

#define I2S_ESP32S3_CLKM_DIV_MAX	255
#define I2S_ESP32S3_FRAC_DIV_MAX	511

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
 * i2s_ll_tx_set_mclk() does. The fractional part uses the best rational
 * approximation that fits the hardware's 9-bit coefficient fields.
 */
static inline int i2s_esp32s3_calc_div(unsigned long source_rate,
				       unsigned int rate,
				       unsigned int mclk_multiple,
				       struct i2s_esp32s3_div *div)
{
	unsigned long best_num;
	unsigned long best_den;
	unsigned long remainder;
	unsigned int integer;
	unsigned int mclk;

	if (!rate || !mclk_multiple)
		return -EINVAL;

	if (check_mul_overflow(rate, mclk_multiple, &mclk))
		return -EOVERFLOW;
	if (source_rate < mclk)
		return -EINVAL;

	integer = source_rate / mclk;
	if (!integer || integer > I2S_ESP32S3_CLKM_DIV_MAX)
		return -EINVAL;
	remainder = source_rate % mclk;
	if (!remainder) {
		/* Exact integer division: hardware coefficients are zero */
		div->integer = integer;
		div->numerator = 0;
		div->denominator = 1;
		div->x = 0;
		div->y = 0;
		div->z = 0;
		div->yn1 = 0;
		return 0;
	}

	rational_best_approximation(remainder, mclk,
				    I2S_ESP32S3_FRAC_DIV_MAX,
				    I2S_ESP32S3_FRAC_DIV_MAX,
				    &best_num, &best_den);
	if (!best_num || !best_den)
		return -EINVAL;

	/* Official encoding from i2s_ll_tx_set_mclk() */
	div->integer = integer;
	div->numerator = best_num;
	div->denominator = best_den;
	div->yn1 = best_num * 2 > best_den;
	div->z = div->yn1 ? best_den - best_num : best_num;
	div->x = best_den / div->z - 1;
	div->y = best_den % div->z;

	return 0;
}

static inline u32 i2s_esp32s3_conf1_value(unsigned int bits_per_sample,
					  unsigned int bck_div)
{
	return ((bits_per_sample - 1) << I2S_ESP32S3_CONF1_WS_WIDTH_SHIFT) |
	       ((bck_div - 1) << I2S_ESP32S3_CONF1_BCK_DIV_SHIFT) |
	       ((bits_per_sample - 1) << I2S_ESP32S3_CONF1_BITS_MOD_SHIFT) |
	       ((bits_per_sample - 1) << I2S_ESP32S3_CONF1_HALF_SAMPLE_SHIFT) |
	       ((bits_per_sample - 1) << I2S_ESP32S3_CONF1_CHAN_BITS_SHIFT) |
	       I2S_ESP32S3_CONF1_MSB_SHIFT;
}

#endif /* _I2S_ESP32S3_H */
