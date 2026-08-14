// SPDX-License-Identifier: GPL-2.0-only

#include <kunit/test.h>

#include "pwm-esp32s3.h"

static void esp32s3_ledc_timer_config_test(struct kunit *test)
{
	struct esp32s3_ledc_timer_config config;
	int ret;

	ret = esp32s3_ledc_calc_timer(40000000, 50000, &config);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, config.resolution, (u8)10);
	KUNIT_EXPECT_EQ(test, config.divider, 500U);
	KUNIT_EXPECT_EQ(test, config.period_ns, 50000ULL);

	KUNIT_EXPECT_EQ(test, esp32s3_ledc_calc_timer(0, 50000, &config),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, esp32s3_ledc_calc_timer(40000000, 1, &config),
			-ERANGE);
}

static void esp32s3_ledc_duty_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test,
			esp32s3_ledc_duty_ticks(25000, 50000, 10, false),
			512U);
	KUNIT_EXPECT_EQ(test,
			esp32s3_ledc_duty_ticks(12500, 50000, 10, true),
			768U);
	KUNIT_EXPECT_EQ(test,
			esp32s3_ledc_duty_ticks(0, 50000, 10, true),
			1024U);
}

static struct kunit_case esp32s3_ledc_test_cases[] = {
	KUNIT_CASE(esp32s3_ledc_timer_config_test),
	KUNIT_CASE(esp32s3_ledc_duty_test),
	{}
};

static struct kunit_suite esp32s3_ledc_test_suite = {
	.name = "esp32s3-ledc",
	.test_cases = esp32s3_ledc_test_cases,
};

kunit_test_suite(esp32s3_ledc_test_suite);

MODULE_LICENSE("GPL");
