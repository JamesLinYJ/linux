/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * QST QMI8658C 6-axis IMU register definitions and pure logic helpers.
 *
 * Register facts are verified against the QMI8658C datasheet rev 0.9
 * (locked in sources.lock as "qmi8658c-datasheet"): register map
 * Table 26/27/28, I2C slave address section 12.2 (SA0 pulled up ->
 * 0x6A), ODR and full-scale encodings Table 26.
 *
 * Endianness: Table 26 documents CTRL1.BE default as big-endian, but the
 * Waveshare V2 example's official SensorQMI8658 library (MIT, locked
 * waveshare-v2-schematic commit 1c157e6e) leaves the default untouched
 * and assembles samples little-endian (buffer[1]<<8 | buffer[0]) and its
 * demo reads correct values on this board. The driver therefore writes
 * CTRL1 = 0x40 explicitly (little-endian reads + address auto-increment)
 * to remove the ambiguity. See docs/clean-room.md.
 */
#ifndef _QMI8658C_H
#define _QMI8658C_H

#include <linux/errno.h>
#include <linux/math64.h>
#include <linux/types.h>

#define QMI8658C_WHO_AM_I	0x00
#define QMI8658C_REVISION_ID	0x01
#define QMI8658C_CTRL1		0x02
#define QMI8658C_CTRL2		0x03
#define QMI8658C_CTRL3		0x04
#define QMI8658C_CTRL5		0x06
#define QMI8658C_CTRL7		0x08
#define QMI8658C_CTRL9		0x0a
#define QMI8658C_FIFO_DATA	0x17
#define QMI8658C_STATUSINT	0x2d
#define QMI8658C_STATUS0	0x2e
#define QMI8658C_STATUS1	0x2f
#define QMI8658C_TEMP_L		0x33
#define QMI8658C_AX_L		0x35
#define QMI8658C_GZ_H		0x40

#define QMI8658C_CHIP_ID	0x05

/* CTRL1: little-endian reads (BE=0) plus register address auto-increment */
#define QMI8658C_CTRL1_ADDR_AI		BIT(6)
#define QMI8658C_CTRL1_INIT		(QMI8658C_CTRL1_ADDR_AI)

/* CTRL2: accel full-scale and output data rate (Table 26) */
#define QMI8658C_CTRL2_AST		BIT(7)
#define QMI8658C_CTRL2_AFS_MASK		GENMASK(6, 4)
#define QMI8658C_CTRL2_AFS_2G		0x00
#define QMI8658C_CTRL2_AFS_4G		0x10
#define QMI8658C_CTRL2_AFS_8G		0x20
#define QMI8658C_CTRL2_AFS_16G		0x30
#define QMI8658C_CTRL2_AODR_MASK	GENMASK(3, 0)
#define QMI8658C_CTRL2_AODR_250HZ	0x05

/* CTRL3: gyro full-scale and output data rate (Table 26) */
#define QMI8658C_CTRL3_GST		BIT(7)
#define QMI8658C_CTRL3_GFS_MASK		GENMASK(6, 4)
#define QMI8658C_CTRL3_GFS_16DPS	0x00
#define QMI8658C_CTRL3_GFS_32DPS	0x10
#define QMI8658C_CTRL3_GFS_64DPS	0x20
#define QMI8658C_CTRL3_GFS_128DPS	0x30
#define QMI8658C_CTRL3_GFS_256DPS	0x40
#define QMI8658C_CTRL3_GFS_512DPS	0x50
#define QMI8658C_CTRL3_GFS_1024DPS	0x60
#define QMI8658C_CTRL3_GFS_2048DPS	0x70
#define QMI8658C_CTRL3_GODR_MASK	GENMASK(3, 0)
#define QMI8658C_CTRL3_GODR_235HZ	0x05

/* CTRL7: sensor enables (Table 26) */
#define QMI8658C_CTRL7_AEN		BIT(0)
#define QMI8658C_CTRL7_GEN		BIT(1)
#define QMI8658C_CTRL7_SEN		BIT(3)

/* STATUS0: new-data flags, cleared by read (Table 28) */
#define QMI8658C_STATUS0_ADA		BIT(0)
#define QMI8658C_STATUS0_GDA		BIT(1)

/* 6DOF output data rate in Hz for the fixed default configuration */
#define QMI8658C_DEFAULT_ODR_HZ	235

/*
 * With CTRL1 configured for little-endian reads, each 16-bit data
 * register is presented low byte first. A 12-byte burst from AX_L
 * therefore arrives as [AX_L AX_H AY_L AY_H ... GZ_L GZ_H].
 */
#define QMI8658C_DATA_BURST_LEN	12

static inline s16 qmi8658c_decode_le(const u8 *pair)
{
	return (s16)le16_to_cpup((const __le16 *)pair);
}

/*
 * Decode a 12-byte little-endian data burst into six signed 16-bit
 * samples: accel X/Y/Z followed by gyro X/Y/Z.
 */
static inline void qmi8658c_decode_burst(const u8 *burst, s16 *out)
{
	int i;

	for (i = 0; i < 6; i++)
		out[i] = qmi8658c_decode_le(&burst[i * 2]);
}

/* Accelerometer scale in m/s^2 per LSB, nano part */
static inline s32 qmi8658c_accel_scale_nano(u8 afs_encoding)
{
	unsigned int fs_g = 2U << (afs_encoding >> 4);
	u64 nano = DIV_ROUND_CLOSEST_ULL(2ULL * fs_g * 9806650ULL * 1000,
					 65536);

	return (s32)nano;
}

/* Gyroscope scale in rad/s per LSB, nano part */
static inline s32 qmi8658c_gyro_scale_nano(u8 gfs_encoding)
{
	unsigned int fs_dps = 16U << (gfs_encoding >> 4);
	u64 nano = DIV_ROUND_CLOSEST_ULL(2ULL * fs_dps * 17453293ULL,
					 65536);

	return (s32)nano;
}

/*
 * Build CTRL2/CTRL3 register values from the scale encodings above plus
 * the fixed default ODR; rejects unknown encodings.
 */
static inline int qmi8658c_ctrl2_value(u8 afs, u8 *value)
{
	if (afs > QMI8658C_CTRL2_AFS_16G)
		return -EINVAL;
	*value = QMI8658C_CTRL2_AODR_250HZ | afs;
	return 0;
}

static inline int qmi8658c_ctrl3_value(u8 gfs, u8 *value)
{
	if (gfs > QMI8658C_CTRL3_GFS_2048DPS)
		return -EINVAL;
	*value = QMI8658C_CTRL3_GODR_235HZ | gfs;
	return 0;
}

#endif /* _QMI8658C_H */
