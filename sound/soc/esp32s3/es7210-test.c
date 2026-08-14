// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the ES7210 clock coefficient table. Pure logic only:
 * no I2C transfers and no codec hardware are exercised.
 */

#include <kunit/test.h>

#include "es7210.h"

static void es7210_table_checksum_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, ES7210_COEFF_COUNT, 26);
	KUNIT_EXPECT_EQ(test, es7210_coeff_checksum(),
			ES7210_COEFF_GOLDEN_CHECKSUM);
}

static void es7210_table_lookup_test(struct kunit *test)
{
	const struct es7210_coeff *coeff;

	/* Standard rates present in the official table */
	coeff = es7210_coeff_find(12288000, 48000);
	KUNIT_EXPECT_NOT_NULL(test, coeff);
	coeff = es7210_coeff_find(19200000, 48000);
	KUNIT_EXPECT_NOT_NULL(test, coeff);
	coeff = es7210_coeff_find(11289600, 44100);
	KUNIT_EXPECT_NOT_NULL(test, coeff);

	/* Unsupported combinations are rejected */
	KUNIT_EXPECT_NULL(test, es7210_coeff_find(12288000, 44100));
	KUNIT_EXPECT_NULL(test, es7210_coeff_find(24576000, 48000));
	KUNIT_EXPECT_NULL(test, es7210_coeff_find(0, 48000));
}

static struct kunit_case es7210_test_cases[] = {
	KUNIT_CASE(es7210_table_checksum_test),
	KUNIT_CASE(es7210_table_lookup_test),
	{ }
};

static struct kunit_suite es7210_test_suite = {
	.name = "esp32s3-es7210",
	.test_cases = es7210_test_cases,
};

kunit_test_suite(es7210_test_suite);

MODULE_LICENSE("GPL");
