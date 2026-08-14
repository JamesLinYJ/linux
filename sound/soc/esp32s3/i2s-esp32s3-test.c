// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the ESP32-S3 I2S clock divider math. Pure logic only:
 * no register access and no I2S hardware are exercised.
 */

#include <kunit/test.h>

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
	 * numerator 21/1000. The hardware coefficients must reconstruct
	 * the same fraction per the official encoding:
	 *   yn1 = (21*2 > 1000) = 0, z = 21,
	 *   x = 1000/21 - 1 = 46, y = 1000 % 21 = 13.
	 */
	ret = i2s_esp32s3_calc_div(160000000, 48000,
				   I2S_ESP32S3_MCLK_MULTIPLE, &div);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, div.integer, 13);
	KUNIT_EXPECT_EQ(test, div.numerator, 21);
	KUNIT_EXPECT_EQ(test, div.denominator, 1000);
	KUNIT_EXPECT_EQ(test, div.yn1, 0);
	KUNIT_EXPECT_EQ(test, div.z, 21);
	KUNIT_EXPECT_EQ(test, div.x, 46);
	KUNIT_EXPECT_EQ(test, div.y, 13);
}

static void i2s_esp32s3_div_large_fraction_test(struct kunit *test)
{
	struct i2s_esp32s3_div div;
	int ret;

	/*
	 * 160 MHz / (44.1 kHz * 256) = 14.1723... -> integer 14,
	 * numerator 172/1000; num*2 <= den so yn1 = 0.
	 */
	ret = i2s_esp32s3_calc_div(160000000, 44100,
				   I2S_ESP32S3_MCLK_MULTIPLE, &div);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, div.integer, 14);
	KUNIT_EXPECT_EQ(test, div.numerator, 172);
	KUNIT_EXPECT_EQ(test, div.yn1, 0);
	KUNIT_EXPECT_EQ(test, div.z, 172);
	KUNIT_EXPECT_EQ(test, div.x, 4);
	KUNIT_EXPECT_EQ(test, div.y, 140);
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

static struct kunit_case i2s_esp32s3_test_cases[] = {
	KUNIT_CASE(i2s_esp32s3_div_exact_test),
	KUNIT_CASE(i2s_esp32s3_div_fraction_test),
	KUNIT_CASE(i2s_esp32s3_div_large_fraction_test),
	KUNIT_CASE(i2s_esp32s3_div_invalid_test),
	{ }
};

static struct kunit_suite i2s_esp32s3_test_suite = {
	.name = "esp32s3-i2s",
	.test_cases = i2s_esp32s3_test_cases,
};

kunit_test_suite(i2s_esp32s3_test_suite);

MODULE_LICENSE("GPL");
