// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the ESP32-S3 UART divider math. Pure logic only: no
 * register access and no UART hardware are exercised.
 */

#include <kunit/test.h>

#include "esp32s3-uart.h"

static void esp32s3_uart_div_test(struct kunit *test)
{
	u32 div;
	int ret;

	/* 80 MHz / 115200 * 16 = 11111.11 -> rounded 11111 */
	ret = esp32s3_uart_clk_div(80000000, 115200, &div);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, div, 11111);
	KUNIT_EXPECT_EQ(test, div >> 4, 694);
	KUNIT_EXPECT_EQ(test, div & 0xf, 7);

	/* Exact: 80 MHz / 250000 * 16 = 5120 */
	ret = esp32s3_uart_clk_div(80000000, 250000, &div);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, div, 5120);
	KUNIT_EXPECT_EQ(test, div & 0xf, 0);

	/* Invalid inputs */
	KUNIT_EXPECT_EQ(test, esp32s3_uart_clk_div(80000000, 0, &div),
			-EINVAL);
	/* Baud too high for the source clock */
	KUNIT_EXPECT_EQ(test, esp32s3_uart_clk_div(80000000, 4000000, &div),
			0);
	KUNIT_EXPECT_GE(test, div, 16);
}

static struct kunit_case esp32s3_uart_test_cases[] = {
	KUNIT_CASE(esp32s3_uart_div_test),
	{ }
};

static struct kunit_suite esp32s3_uart_test_suite = {
	.name = "esp32s3-uart",
	.test_cases = esp32s3_uart_test_cases,
};

kunit_test_suite(esp32s3_uart_test_suite);

MODULE_LICENSE("GPL");
