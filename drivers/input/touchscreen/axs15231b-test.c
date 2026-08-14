// SPDX-License-Identifier: GPL-2.0-only

#include <kunit/test.h>

#include "axs15231b.h"

static void axs15231b_parse_touch_test(struct kunit *test)
{
	const u8 report[AXS15231B_REPORT_LEN] = {
		0x00, 0x01, 0x82, 0x34, 0x01, 0x56, 0x00, 0x00,
	};
	struct axs15231b_point point;
	int ret;

	ret = axs15231b_parse_report(report, sizeof(report), &point);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_TRUE(test, point.active);
	KUNIT_EXPECT_EQ(test, point.x, 0x234U);
	KUNIT_EXPECT_EQ(test, point.y, 0x156U);
}

static void axs15231b_parse_release_test(struct kunit *test)
{
	const u8 report[AXS15231B_REPORT_LEN] = { 0 };
	struct axs15231b_point point = {
		.x = 123,
		.y = 456,
		.active = true,
	};
	int ret;

	ret = axs15231b_parse_report(report, sizeof(report), &point);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_FALSE(test, point.active);
	KUNIT_EXPECT_EQ(test, point.x, 0U);
	KUNIT_EXPECT_EQ(test, point.y, 0U);
}

static void axs15231b_parse_rejects_invalid_test(struct kunit *test)
{
	u8 report[AXS15231B_REPORT_LEN] = { 0 };
	struct axs15231b_point point;
	int ret;

	report[1] = 2;
	ret = axs15231b_parse_report(report, sizeof(report), &point);
	KUNIT_EXPECT_EQ(test, ret, -EPROTO);

	ret = axs15231b_parse_report(report, sizeof(report) - 1, &point);
	KUNIT_EXPECT_EQ(test, ret, -EMSGSIZE);

	ret = axs15231b_parse_report(NULL, sizeof(report), &point);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);

	ret = axs15231b_parse_report(report, sizeof(report), NULL);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

static struct kunit_case axs15231b_test_cases[] = {
	KUNIT_CASE(axs15231b_parse_touch_test),
	KUNIT_CASE(axs15231b_parse_release_test),
	KUNIT_CASE(axs15231b_parse_rejects_invalid_test),
	{ }
};

static struct kunit_suite axs15231b_test_suite = {
	.name = "esp32s3-axs15231b",
	.test_cases = axs15231b_test_cases,
};

kunit_test_suite(axs15231b_test_suite);

MODULE_LICENSE("GPL");
