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

	/* 160 MHz / (50 kHz * 64) = 50 exactly */
	ret = i2s_esp32s3_calc_div(160000000, 50000, 64, &div);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, div.integer, 50);
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
	 * 160 MHz / (48 kHz * 64) = 52.0833... -> integer 52,
	 * numerator 83/1000. The hardware coefficients must reconstruct
	 * the same fraction per the official encoding:
	 *   yn1 = (83*2 > 1000) = 0, z = 83,
	 *   x = 1000/83 - 1 = 11, y = 1000 % 83 = 4.
	 */
	ret = i2s_esp32s3_calc_div(160000000, 48000, 64, &div);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, div.integer, 52);
	KUNIT_EXPECT_EQ(test, div.numerator, 83);
	KUNIT_EXPECT_EQ(test, div.denominator, 1000);
	KUNIT_EXPECT_EQ(test, div.yn1, 0);
	KUNIT_EXPECT_EQ(test, div.z, 83);
	KUNIT_EXPECT_EQ(test, div.x, 11);
	KUNIT_EXPECT_EQ(test, div.y, 4);
}

static void i2s_esp32s3_div_large_fraction_test(struct kunit *test)
{
	struct i2s_esp32s3_div div;
	int ret;

	/* 160 MHz / (44.1 kHz * 64) = 56.689... -> integer 56, 689/1000 */
	ret = i2s_esp32s3_calc_div(160000000, 44100, 64, &div);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, div.integer, 56);
	KUNIT_EXPECT_EQ(test, div.numerator, 689);
	/* num*2 > den: yn1 = 1, z = den - num, x = den/z - 1, y = den%z */
	KUNIT_EXPECT_EQ(test, div.yn1, 1);
	KUNIT_EXPECT_EQ(test, div.z, 311);
	KUNIT_EXPECT_EQ(test, div.x, 2);
	KUNIT_EXPECT_EQ(test, div.y, 67);
}

static void i2s_esp32s3_div_invalid_test(struct kunit *test)
{
	struct i2s_esp32s3_div div;

	KUNIT_EXPECT_EQ(test, i2s_esp32s3_calc_div(160000000, 0, 64, &div),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, i2s_esp32s3_calc_div(160000000, 48000, 0, &div),
			-EINVAL);
	/* Source slower than the bit clock cannot divide down */
	KUNIT_EXPECT_EQ(test, i2s_esp32s3_calc_div(1000000, 48000, 64, &div),
			-EINVAL);
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
