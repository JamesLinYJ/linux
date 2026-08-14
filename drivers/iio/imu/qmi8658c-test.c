// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the QMI8658C register/scale helpers. Pure logic only:
 * no I2C transfers and no sensor hardware are exercised.
 */

#include <kunit/test.h>

#include "qmi8658c.h"

static void qmi8658c_decode_test(struct kunit *test)
{
	/* Little-endian burst: AX=0x1234, AY=-1, AZ=0, GX=1, GY=-32768, GZ=32767 */
	const u8 burst[QMI8658C_DATA_BURST_LEN] = {
		0x34, 0x12, 0xff, 0xff, 0x00, 0x00,
		0x01, 0x00, 0x00, 0x80, 0xff, 0x7f,
	};
	s16 sample[6];

	qmi8658c_decode_burst(burst, sample);
	KUNIT_EXPECT_EQ(test, sample[0], (s16)0x1234);
	KUNIT_EXPECT_EQ(test, sample[1], (s16)-1);
	KUNIT_EXPECT_EQ(test, sample[2], (s16)0);
	KUNIT_EXPECT_EQ(test, sample[3], (s16)1);
	KUNIT_EXPECT_EQ(test, sample[4], (s16)-32768);
	KUNIT_EXPECT_EQ(test, sample[5], (s16)32767);
}

static void qmi8658c_scale_test(struct kunit *test)
{
	/*
	 * +-8 g: 2*8*9.80665/65536 m/s^2 = 2.3942027e-3 -> 2394202e-9.
	 * +-256 dps: 2*256*pi/180/65536 rad/s = 1.3635409e-4 -> 136354e-9.
	 */
	KUNIT_EXPECT_EQ(test,
			qmi8658c_accel_scale_nano(QMI8658C_CTRL2_AFS_8G),
			2394202);
	KUNIT_EXPECT_EQ(test,
			qmi8658c_gyro_scale_nano(QMI8658C_CTRL3_GFS_256DPS),
			136354);

	/* Smallest scales stay nonzero */
	KUNIT_EXPECT_GT(test,
			qmi8658c_accel_scale_nano(QMI8658C_CTRL2_AFS_2G), 0);
	KUNIT_EXPECT_GT(test,
			qmi8658c_gyro_scale_nano(QMI8658C_CTRL3_GFS_16DPS), 0);

	/* Largest gyro ranges must not truncate (regression: u8 overflow
	 * of the 16<<n encoding) and must scale monotonically.
	 */
	KUNIT_EXPECT_GT(test,
			qmi8658c_gyro_scale_nano(QMI8658C_CTRL3_GFS_2048DPS),
			qmi8658c_gyro_scale_nano(QMI8658C_CTRL3_GFS_512DPS));
	KUNIT_EXPECT_GT(test,
			qmi8658c_gyro_scale_nano(QMI8658C_CTRL3_GFS_512DPS),
			qmi8658c_gyro_scale_nano(QMI8658C_CTRL3_GFS_16DPS));
}

static void qmi8658c_ctrl_value_test(struct kunit *test)
{
	u8 value;
	int ret;

	ret = qmi8658c_ctrl2_value(QMI8658C_CTRL2_AFS_8G, &value);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, value,
			QMI8658C_CTRL2_AODR_250HZ | QMI8658C_CTRL2_AFS_8G);

	ret = qmi8658c_ctrl3_value(QMI8658C_CTRL3_GFS_256DPS, &value);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, value,
			QMI8658C_CTRL3_GODR_235HZ | QMI8658C_CTRL3_GFS_256DPS);

	/* Unknown scale encodings are rejected */
	ret = qmi8658c_ctrl2_value(QMI8658C_CTRL2_AFS_16G + 0x10, &value);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = qmi8658c_ctrl3_value(QMI8658C_CTRL3_GFS_2048DPS + 0x10, &value);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

static struct kunit_case qmi8658c_test_cases[] = {
	KUNIT_CASE(qmi8658c_decode_test),
	KUNIT_CASE(qmi8658c_scale_test),
	KUNIT_CASE(qmi8658c_ctrl_value_test),
	{ }
};

static struct kunit_suite qmi8658c_test_suite = {
	.name = "esp32s3-qmi8658c",
	.test_cases = qmi8658c_test_cases,
};

kunit_test_suite(qmi8658c_test_suite);

MODULE_LICENSE("GPL");
