// SPDX-License-Identifier: GPL-2.0-only

#include <kunit/test.h>

#include "esp32s3-gdma.h"

static void esp32s3_gdma_data_address_test(struct kunit *test)
{
	bool valid;

	KUNIT_EXPECT_TRUE(test, esp32s3_gdma_data_addr_valid(0x3c000000, 4));
	KUNIT_EXPECT_TRUE(test, esp32s3_gdma_data_addr_valid(0xfffffffc, 4));
	KUNIT_EXPECT_FALSE(test, esp32s3_gdma_data_addr_valid(0x3c000001, 4));
	KUNIT_EXPECT_FALSE(test, esp32s3_gdma_data_addr_valid(0x3c000000, 2));
	KUNIT_EXPECT_FALSE(test, esp32s3_gdma_data_addr_valid(0x3c000000, 0));
	valid = esp32s3_gdma_data_addr_valid(0xfffffffc, 8);
	KUNIT_EXPECT_FALSE(test, valid);
}

static void esp32s3_gdma_descriptor_address_test(struct kunit *test)
{
	const phys_addr_t start = 0x3fc88000;
	const phys_addr_t end = 0x3fc8ffff;
	bool valid;

	valid = esp32s3_gdma_desc_addr_valid(start, 12, start, end);
	KUNIT_EXPECT_TRUE(test, valid);
	valid = esp32s3_gdma_desc_addr_valid(end - 11, 12, start, end);
	KUNIT_EXPECT_TRUE(test, valid);
	valid = esp32s3_gdma_desc_addr_valid(start - 4, 12, start, end);
	KUNIT_EXPECT_FALSE(test, valid);
	valid = esp32s3_gdma_desc_addr_valid(end - 7, 12, start, end);
	KUNIT_EXPECT_FALSE(test, valid);
	valid = esp32s3_gdma_desc_addr_valid(0x3fd00000, 12, start, end);
	KUNIT_EXPECT_FALSE(test, valid);
	valid = esp32s3_gdma_desc_addr_valid(start, 0, start, end);
	KUNIT_EXPECT_FALSE(test, valid);
}

static void esp32s3_gdma_descriptor_flags_test(struct kunit *test)
{
	u32 flags;

	flags = esp32s3_gdma_desc_flags(ESP32S3_GDMA_DESC_MAX_LEN,
					false, true);
	KUNIT_EXPECT_EQ(test, FIELD_GET(ESP32S3_GDMA_DESC_SIZE, flags),
			(u32)ESP32S3_GDMA_DESC_MAX_LEN);
	KUNIT_EXPECT_EQ(test, FIELD_GET(ESP32S3_GDMA_DESC_LENGTH, flags), 0U);
	KUNIT_EXPECT_TRUE(test, flags & ESP32S3_GDMA_DESC_OWNER);
	KUNIT_EXPECT_FALSE(test, flags & ESP32S3_GDMA_DESC_EOF);

	flags = esp32s3_gdma_desc_flags(64, true, false);
	KUNIT_EXPECT_EQ(test, FIELD_GET(ESP32S3_GDMA_DESC_LENGTH, flags), 64U);
	KUNIT_EXPECT_FALSE(test, flags & ESP32S3_GDMA_DESC_EOF);

	flags = esp32s3_gdma_desc_flags(64, true, true);
	KUNIT_EXPECT_TRUE(test, flags & ESP32S3_GDMA_DESC_EOF);
}

static void esp32s3_gdma_cyclic_validation_test(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test,
			  esp32s3_gdma_cyclic_valid(0x3c000000, 16368, 4092));
	KUNIT_EXPECT_FALSE(test,
			   esp32s3_gdma_cyclic_valid(0x3c000000, 16384, 0));
	KUNIT_EXPECT_FALSE(test,
			   esp32s3_gdma_cyclic_valid(0x3c000000, 16384, 4097));
	KUNIT_EXPECT_FALSE(test,
			   esp32s3_gdma_cyclic_valid(0x3c000000, 16384, 4090));
	KUNIT_EXPECT_FALSE(test,
			   esp32s3_gdma_cyclic_valid(0x3c000002, 16368, 4092));
}

static struct kunit_case esp32s3_gdma_test_cases[] = {
	KUNIT_CASE(esp32s3_gdma_data_address_test),
	KUNIT_CASE(esp32s3_gdma_descriptor_address_test),
	KUNIT_CASE(esp32s3_gdma_descriptor_flags_test),
	KUNIT_CASE(esp32s3_gdma_cyclic_validation_test),
	{}
};

static struct kunit_suite esp32s3_gdma_test_suite = {
	.name = "esp32s3-gdma",
	.test_cases = esp32s3_gdma_test_cases,
};

kunit_test_suite(esp32s3_gdma_test_suite);

MODULE_LICENSE("GPL");
