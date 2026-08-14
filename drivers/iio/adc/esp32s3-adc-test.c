// SPDX-License-Identifier: GPL-2.0-only

#include <kunit/test.h>

#include "esp32s3-adc.h"

static void esp32s3_adc_channel_encoding_test(struct kunit *test)
{
	u32 atten_value;
	u32 atten_mask;
	u32 pad;

	KUNIT_ASSERT_EQ(test,
			esp32s3_adc_encode_channel(3, 2, &pad, &atten_mask,
						   &atten_value), 0);
	KUNIT_EXPECT_EQ(test, pad, BIT(3));
	KUNIT_EXPECT_EQ(test, atten_mask, GENMASK(7, 6));
	KUNIT_EXPECT_EQ(test, atten_value, 2U << 6);
}

static void esp32s3_adc_invalid_config_test(struct kunit *test)
{
	u32 atten_value;
	u32 atten_mask;
	u32 pad;

	KUNIT_EXPECT_EQ(test,
			esp32s3_adc_encode_channel(10, 2, &pad, &atten_mask,
						   &atten_value), -EINVAL);
	KUNIT_EXPECT_EQ(test,
			esp32s3_adc_encode_channel(3, 4, &pad, &atten_mask,
						   &atten_value), -EINVAL);
	KUNIT_EXPECT_EQ(test,
			esp32s3_adc_encode_channel(3, 2, NULL, &atten_mask,
						   &atten_value), -EINVAL);
}

static struct kunit_case esp32s3_adc_test_cases[] = {
	KUNIT_CASE(esp32s3_adc_channel_encoding_test),
	KUNIT_CASE(esp32s3_adc_invalid_config_test),
	{}
};

static struct kunit_suite esp32s3_adc_test_suite = {
	.name = "esp32s3-adc",
	.test_cases = esp32s3_adc_test_cases,
};

kunit_test_suite(esp32s3_adc_test_suite);

MODULE_LICENSE("GPL");
