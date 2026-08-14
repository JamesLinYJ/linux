// SPDX-License-Identifier: GPL-2.0-only
/*
 * QST QMI8658C 6-axis inertial measurement unit (I2C)
 *
 * Datasheet: QMI8658C_datasheet_rev_0.9 (locked in sources.lock as
 * "qmi8658c-datasheet"). Register facts are listed in qmi8658c.h.
 *
 * Current scope: IIO direct mode with raw acceleration (m/s^2) and
 * angular velocity (rad/s), per-axis scale, and a fixed 235 Hz 6DOF
 * output data rate. The data-ready interrupt pins (INT1/INT2, wired to
 * TCA9554 EXIO2/EXIO3 on the Waveshare V2) and the FIFO are described
 * in the device tree but not used yet; an IIO triggered buffer on the
 * INT2 signal is the planned follow-up (M9 item remains open until the
 * device is exercised on hardware).
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/units.h>

#include "qmi8658c.h"

#define QMI8658C_STATUS_POLL_US	1000

struct qmi8658c_data {
	struct regmap *regmap;
	struct i2c_client *client;
	u8 afs;
	u8 gfs;
};

static const struct regmap_config qmi8658c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = QMI8658C_GZ_H,
};

enum qmi8658c_scan {
	QMI8658C_SCAN_ACCEL_X,
	QMI8658C_SCAN_ACCEL_Y,
	QMI8658C_SCAN_ACCEL_Z,
	QMI8658C_SCAN_GYRO_X,
	QMI8658C_SCAN_GYRO_Y,
	QMI8658C_SCAN_GYRO_Z,
	QMI8658C_SCAN_TIMESTAMP,
};

#define QMI8658C_CHANNEL(_type, _index, _scan_index)			\
	{								\
		.type = _type,						\
		.modified = 1,						\
		.channel2 = IIO_MOD_##_index,				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				      BIT(IIO_CHAN_INFO_SCALE),		\
		.scan_index = _scan_index,				\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 16,					\
			.storagebits = 16,				\
			.endianness = IIO_CPU,				\
		},							\
	}

static const struct iio_chan_spec qmi8658c_channels[] = {
	QMI8658C_CHANNEL(IIO_ACCEL, X, QMI8658C_SCAN_ACCEL_X),
	QMI8658C_CHANNEL(IIO_ACCEL, Y, QMI8658C_SCAN_ACCEL_Y),
	QMI8658C_CHANNEL(IIO_ACCEL, Z, QMI8658C_SCAN_ACCEL_Z),
	QMI8658C_CHANNEL(IIO_ANGL_VEL, X, QMI8658C_SCAN_GYRO_X),
	QMI8658C_CHANNEL(IIO_ANGL_VEL, Y, QMI8658C_SCAN_GYRO_Y),
	QMI8658C_CHANNEL(IIO_ANGL_VEL, Z, QMI8658C_SCAN_GYRO_Z),
	IIO_CHAN_SOFT_TIMESTAMP(QMI8658C_SCAN_TIMESTAMP),
};

/*
 * Wait until new sensor data is flagged in STATUS0. The flags are
 * cleared by reading the data registers, so this must run before the
 * burst read. Bounded poll; timeouts return an error to the caller.
 */
static int qmi8658c_wait_data_ready(struct qmi8658c_data *data)
{
	unsigned int status;
	unsigned int attempts;

	for (attempts = 0; attempts < QMI8658C_STATUS_POLL_US / 100;
	     attempts++) {
		int ret;

		ret = regmap_read(data->regmap, QMI8658C_STATUS0, &status);
		if (ret)
			return ret;
		if (status & (QMI8658C_STATUS0_ADA | QMI8658C_STATUS0_GDA))
			return 0;
		usleep_range(90, 110);
	}

	return -ETIMEDOUT;
}

static int qmi8658c_read_sample(struct qmi8658c_data *data, s16 *sample)
{
	u8 burst[QMI8658C_DATA_BURST_LEN];
	int ret;

	ret = qmi8658c_wait_data_ready(data);
	if (ret)
		return ret;

	ret = regmap_bulk_read(data->regmap, QMI8658C_AX_L, burst,
			       sizeof(burst));
	if (ret)
		return ret;

	qmi8658c_decode_burst(burst, sample);

	return 0;
}

static int qmi8658c_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct qmi8658c_data *data = iio_priv(indio_dev);
	s16 sample[6];
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (!iio_device_claim_direct(indio_dev))
			return -EBUSY;
		ret = qmi8658c_read_sample(data, sample);
		iio_device_release_direct(indio_dev);
		if (ret)
			return ret;

		*val = sample[chan->scan_index];
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_ACCEL) {
			*val = 0;
			*val2 = qmi8658c_accel_scale_nano(data->afs);
		} else {
			*val = 0;
			*val2 = qmi8658c_gyro_scale_nano(data->gfs);
		}
		return IIO_VAL_INT_PLUS_NANO;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = QMI8658C_DEFAULT_ODR_HZ;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static const struct iio_info qmi8658c_info = {
	.read_raw = qmi8658c_read_raw,
};

static int qmi8658c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct qmi8658c_data *data;
	struct iio_dev *indio_dev;
	unsigned int id;
	u8 ctrl;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	data->regmap = devm_regmap_init_i2c(client, &qmi8658c_regmap_config);
	if (IS_ERR(data->regmap))
		return dev_err_probe(dev, PTR_ERR(data->regmap),
				     "failed to initialize regmap\n");

	ret = regmap_read(data->regmap, QMI8658C_WHO_AM_I, &id);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read WHO_AM_I\n");
	if (id != QMI8658C_CHIP_ID)
		return dev_err_probe(dev, -ENODEV,
				     "unexpected WHO_AM_I 0x%02x\n", id);

	/* Default full-scale: accel +-8 g, gyro +-256 dps */
	data->afs = QMI8658C_CTRL2_AFS_8G;
	data->gfs = QMI8658C_CTRL3_GFS_256DPS;

	ret = qmi8658c_ctrl2_value(data->afs, &ctrl);
	if (ret)
		return ret;
	ret = regmap_write(data->regmap, QMI8658C_CTRL2, ctrl);
	if (ret)
		return dev_err_probe(dev, ret, "failed to configure accel\n");

	ret = qmi8658c_ctrl3_value(data->gfs, &ctrl);
	if (ret)
		return ret;
	ret = regmap_write(data->regmap, QMI8658C_CTRL3, ctrl);
	if (ret)
		return dev_err_probe(dev, ret, "failed to configure gyro\n");

	/*
	 * Little-endian reads plus address auto-increment; the explicit
	 * write resolves the datasheet/vendor-library disagreement about
	 * the BE reset value, see qmi8658c.h.
	 */
	ret = regmap_write(data->regmap, QMI8658C_CTRL1, QMI8658C_CTRL1_INIT);
	if (ret)
		return dev_err_probe(dev, ret, "failed to configure CTRL1\n");

	/* Enable accelerometer and gyroscope */
	ret = regmap_write(data->regmap, QMI8658C_CTRL7,
			   QMI8658C_CTRL7_AEN | QMI8658C_CTRL7_GEN);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable sensors\n");

	indio_dev->name = "qmi8658c";
	indio_dev->info = &qmi8658c_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = qmi8658c_channels;
	indio_dev->num_channels = ARRAY_SIZE(qmi8658c_channels);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id qmi8658c_of_match[] = {
	{ .compatible = "qst,qmi8658c" },
	{ }
};
MODULE_DEVICE_TABLE(of, qmi8658c_of_match);

static const struct i2c_device_id qmi8658c_id[] = {
	{ "qmi8658c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, qmi8658c_id);

static struct i2c_driver qmi8658c_driver = {
	.driver = {
		.name = "qmi8658c",
		.of_match_table = qmi8658c_of_match,
	},
	.probe = qmi8658c_probe,
	.id_table = qmi8658c_id,
};
module_i2c_driver(qmi8658c_driver);

MODULE_DESCRIPTION("QST QMI8658C 6-axis IMU driver");
MODULE_AUTHOR("ESP32-S3 Linux project");
MODULE_LICENSE("GPL");
