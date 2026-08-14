// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the AXS15231B panel protocol helpers. Pure logic only:
 * no SPI transfers and no panel hardware are exercised.
 */

#include <kunit/test.h>

#include "axs15231b.h"

static void axs15231b_cmd_header_test(struct kunit *test)
{
	const u8 expect_write_cmd[AXS15231B_CMD_HEADER_LEN] = {
		0x02, 0x2a, 0x00, 0x00
	};
	const u8 expect_write_color[AXS15231B_CMD_HEADER_LEN] = {
		0x32, 0x2c, 0x00, 0x00
	};
	u8 buf[AXS15231B_CMD_HEADER_LEN];

	axs15231b_cmd_header(buf, AXS15231B_OP_WRITE_CMD, 0x2a);
	KUNIT_EXPECT_MEMEQ(test, buf, expect_write_cmd,
			   AXS15231B_CMD_HEADER_LEN);

	axs15231b_cmd_header(buf, AXS15231B_OP_WRITE_COLOR, 0x2c);
	KUNIT_EXPECT_MEMEQ(test, buf, expect_write_color,
			   AXS15231B_CMD_HEADER_LEN);
}

static void axs15231b_rect_bytes_test(struct kunit *test)
{
	size_t bytes;
	int ret;

	/* Full frame: 640 * 172 * 2 */
	ret = axs15231b_rect_bytes(0, 0, AXS15231B_WIDTH - 1,
				   AXS15231B_HEIGHT - 1, &bytes);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, bytes,
			(size_t)AXS15231B_WIDTH * AXS15231B_HEIGHT *
			AXS15231B_BYTES_PER_PIXEL);

	/* Single pixel */
	ret = axs15231b_rect_bytes(7, 9, 7, 9, &bytes);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, bytes, 2);

	/* Inverted window */
	ret = axs15231b_rect_bytes(10, 0, 5, 5, &bytes);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);

	/* Beyond panel width */
	ret = axs15231b_rect_bytes(0, 0, AXS15231B_WIDTH, 0, &bytes);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);

	/* Beyond panel height */
	ret = axs15231b_rect_bytes(0, 0, 0, AXS15231B_HEIGHT, &bytes);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);

	/* NULL output */
	ret = axs15231b_rect_bytes(0, 0, 1, 1, NULL);
	KUNIT_EXPECT_EQ(test, ret, 0);
}

/*
 * Pin the init table transcription to the official espressif component
 * esp_lcd_axs15231b v1.0.1~1: checksum of all (cmd, data bytes, len,
 * delay_ms) fields of vendor_specific_init_default[], modulo 2^32.
 */
#define AXS15231B_INIT_GOLDEN_CHECKSUM 0x000080af

static u32 axs15231b_init_checksum(void)
{
	u32 sum = 0;
	size_t i, j;

	for (i = 0; i < AXS15231B_INIT_CMDS_COUNT; i++) {
		const struct axs15231b_init_cmd *entry = &axs15231b_init_cmds[i];

		sum += entry->cmd;
		sum += entry->len;
		sum += entry->delay_ms;
		for (j = 0; j < entry->len; j++)
			sum += entry->data[j];
	}
	return sum;
}

static void axs15231b_init_table_test(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test,
			  axs15231b_init_table_valid(axs15231b_init_cmds,
						     AXS15231B_INIT_CMDS_COUNT));
	KUNIT_EXPECT_EQ(test, AXS15231B_INIT_CMDS_COUNT, 32);
	KUNIT_EXPECT_EQ(test, axs15231b_init_checksum(),
			AXS15231B_INIT_GOLDEN_CHECKSUM);
}

static void axs15231b_init_table_valid_test(struct kunit *test)
{
	struct axs15231b_init_cmd table[2] = { { 0 } };

	KUNIT_EXPECT_TRUE(test, axs15231b_init_table_valid(table, 1));

	table[0].len = ARRAY_SIZE(table[0].data) + 1;
	KUNIT_EXPECT_FALSE(test, axs15231b_init_table_valid(table, 1));

	table[0].len = 0;
	table[0].delay_ms = 1001;
	KUNIT_EXPECT_FALSE(test, axs15231b_init_table_valid(table, 1));
}

static struct kunit_case axs15231b_test_cases[] = {
	KUNIT_CASE(axs15231b_cmd_header_test),
	KUNIT_CASE(axs15231b_rect_bytes_test),
	KUNIT_CASE(axs15231b_init_table_test),
	KUNIT_CASE(axs15231b_init_table_valid_test),
	{ }
};

static struct kunit_suite axs15231b_test_suite = {
	.name = "esp32s3-axs15231b-panel",
	.test_cases = axs15231b_test_cases,
};

kunit_test_suite(axs15231b_test_suite);

MODULE_LICENSE("GPL");
