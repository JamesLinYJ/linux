// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the ESP32-S3 I2S clock divider math. Pure logic only:
 * no register access and no I2S hardware are exercised.
 */

#include <kunit/test.h>
#include <linux/bitfield.h>

#include "i2s-esp32s3.h"

static void i2s_esp32s3_div_exact_test(struct kunit *test)
{
	struct i2s_esp32s3_div div;
	int ret;

	/* 160 MHz / (62.5 kHz * 256) = 10 exactly */
	ret = i2s_esp32s3_calc_div(160000000, 62500,
				   I2S_ESP32S3_MCLK_MULTIPLE, &div);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, div.integer, 10);
	KUNIT_EXPECT_EQ(test, div.numerator, 0);
	KUNIT_EXPECT_EQ(test, div.x, 0);
	KUNIT_EXPECT_EQ(test, div.y, 0);
	KUNIT_EXPECT_EQ(test, div.z, 0);
	KUNIT_EXPECT_EQ(test, div.yn1, 0);
}

static void i2s_esp32s3_div_fraction_test(struct kunit *test)
{
	struct i2s_esp32s3_div div;
	int ret;

	/*
	 * 160 MHz / (48 kHz * 256) = 13.0208... -> integer 13,
	 * fractional part 1/48. The hardware coefficients must reconstruct
	 * the same fraction per the official encoding:
	 *   yn1 = 0, z = 1, x = 48/1 - 1 = 47, y = 0.
	 */
	ret = i2s_esp32s3_calc_div(160000000, 48000,
				   I2S_ESP32S3_MCLK_MULTIPLE, &div);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, div.integer, 13);
	KUNIT_EXPECT_EQ(test, div.numerator, 1);
	KUNIT_EXPECT_EQ(test, div.denominator, 48);
	KUNIT_EXPECT_EQ(test, div.yn1, 0);
	KUNIT_EXPECT_EQ(test, div.z, 1);
	KUNIT_EXPECT_EQ(test, div.x, 47);
	KUNIT_EXPECT_EQ(test, div.y, 0);
}

static void i2s_esp32s3_div_large_fraction_test(struct kunit *test)
{
	struct i2s_esp32s3_div div;
	int ret;

	/*
	 * 160 MHz / (44.1 kHz * 256) = 14.1723...; the exact
	 * fractional part is 76/441 and fits the 9-bit fields.
	 */
	ret = i2s_esp32s3_calc_div(160000000, 44100,
				   I2S_ESP32S3_MCLK_MULTIPLE, &div);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, div.integer, 14);
	KUNIT_EXPECT_EQ(test, div.numerator, 76);
	KUNIT_EXPECT_EQ(test, div.denominator, 441);
	KUNIT_EXPECT_EQ(test, div.yn1, 0);
	KUNIT_EXPECT_EQ(test, div.z, 76);
	KUNIT_EXPECT_EQ(test, div.x, 4);
	KUNIT_EXPECT_EQ(test, div.y, 61);
}

static void i2s_esp32s3_div_invalid_test(struct kunit *test)
{
	struct i2s_esp32s3_div div;

	KUNIT_EXPECT_EQ(test, i2s_esp32s3_calc_div(160000000, 0,
			I2S_ESP32S3_MCLK_MULTIPLE, &div), -EINVAL);
	KUNIT_EXPECT_EQ(test, i2s_esp32s3_calc_div(160000000, 48000, 0,
			&div), -EINVAL);
	/* Source slower than the MCLK cannot divide down */
	KUNIT_EXPECT_EQ(test, i2s_esp32s3_calc_div(1000000, 48000,
			I2S_ESP32S3_MCLK_MULTIPLE, &div), -EINVAL);
}

static void i2s_esp32s3_conf1_layout_test(struct kunit *test)
{
	u32 value = i2s_esp32s3_conf1_value(32, 4);

	KUNIT_EXPECT_EQ(test, FIELD_GET(I2S_ESP32S3_CONF1_WS_WIDTH_MASK, value),
			31U);
	KUNIT_EXPECT_EQ(test, FIELD_GET(I2S_ESP32S3_CONF1_BCK_DIV_MASK, value),
			3U);
	KUNIT_EXPECT_EQ(test, FIELD_GET(I2S_ESP32S3_CONF1_BITS_MOD_MASK, value),
			31U);
	KUNIT_EXPECT_EQ(test,
			FIELD_GET(I2S_ESP32S3_CONF1_HALF_SAMPLE_MASK, value),
			31U);
	KUNIT_EXPECT_EQ(test,
			FIELD_GET(I2S_ESP32S3_CONF1_CHAN_BITS_MASK, value), 31U);
	KUNIT_EXPECT_TRUE(test, value & I2S_ESP32S3_CONF1_MSB_SHIFT);
	KUNIT_EXPECT_EQ(test, I2S_ESP32S3_CONF1_MSB_SHIFT, BIT(29));
	KUNIT_EXPECT_EQ(test, I2S_ESP32S3_CLKM_MCLK_SEL, BIT(29));
}

static struct kunit_case i2s_esp32s3_test_cases[] = {
	KUNIT_CASE(i2s_esp32s3_div_exact_test),
	KUNIT_CASE(i2s_esp32s3_div_fraction_test),
	KUNIT_CASE(i2s_esp32s3_div_large_fraction_test),
	KUNIT_CASE(i2s_esp32s3_div_invalid_test),
	KUNIT_CASE(i2s_esp32s3_conf1_layout_test),
	{ }
};

static struct kunit_suite i2s_esp32s3_test_suite = {
	.name = "esp32s3-i2s",
	.test_cases = i2s_esp32s3_test_cases,
};

kunit_test_suite(i2s_esp32s3_test_suite);

MODULE_LICENSE("GPL");
