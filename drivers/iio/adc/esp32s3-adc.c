// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif ESP32-S3 ADC1 one-shot IIO driver
 */

#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#include <linux/iio/iio.h>

#include "esp32s3-adc.h"

#define ESP32S3_ADC_READER1_CTRL	0x000
#define ESP32S3_ADC_READER1_DATA_INV	BIT(28)
#define ESP32S3_ADC_READER1_CLK_DIV	GENMASK(7, 0)
#define ESP32S3_ADC_READER1_CLK_DIV_DEFAULT 1

#define ESP32S3_ADC_MEAS1_CTRL2		0x00c
#define ESP32S3_ADC_MEAS1_EN_PAD_FORCE	BIT(31)
#define ESP32S3_ADC_MEAS1_EN_PAD	GENMASK(30, 19)
#define ESP32S3_ADC_MEAS1_START_FORCE	BIT(18)
#define ESP32S3_ADC_MEAS1_START		BIT(17)
#define ESP32S3_ADC_MEAS1_DONE		BIT(16)
#define ESP32S3_ADC_MEAS1_DATA		GENMASK(15, 0)

#define ESP32S3_ADC_MEAS1_MUX		0x010
#define ESP32S3_ADC_MEAS1_DIG_FORCE	BIT(31)
#define ESP32S3_ADC_ATTEN1		0x014
#define ESP32S3_ADC_POWER		0x03c
#define ESP32S3_ADC_POWER_FORCE		GENMASK(30, 29)
#define ESP32S3_ADC_POWER_FSM		0
#define ESP32S3_ADC_POWER_ON		3
#define ESP32S3_ADC_SLAVE_ADDR1		0x040
#define ESP32S3_ADC_MEAS_STATUS		GENMASK(29, 22)
#define ESP32S3_ADC_CLK_GATE		0x104
#define ESP32S3_ADC_CLK_ENABLE		BIT(30)

#define ESP32S3_ANALOG_CONFIG		0x044
#define ESP32S3_ANALOG_I2C_SAR_DISABLE	BIT(18)
#define ESP32S3_ANALOG_CONFIG2		0x048
#define ESP32S3_ANALOG_SAR_ENABLE	BIT(16)

#define ESP32S3_ADC_RAW_MASK		GENMASK(11, 0)
#define ESP32S3_ADC_TIMEOUT_US		1000

struct esp32s3_adc {
	void __iomem *sens;
	void __iomem *analog;
	/* Serializes shared controller register access and conversions. */
	struct mutex lock;
	struct iio_chan_spec *channels;
	u8 attenuation[ESP32S3_ADC_MAX_CHANNEL + 1];
};

static void esp32s3_adc_update_bits(void __iomem *reg, u32 mask, u32 value)
{
	u32 val = readl(reg);

	writel((val & ~mask) | (value & mask), reg);
}

static void esp32s3_adc_power_on(struct esp32s3_adc *adc)
{
	/* Enable the internal analog bus before forcing the SAR block on. */
	esp32s3_adc_update_bits(adc->analog + ESP32S3_ANALOG_CONFIG,
				ESP32S3_ANALOG_I2C_SAR_DISABLE, 0);
	esp32s3_adc_update_bits(adc->analog + ESP32S3_ANALOG_CONFIG2,
				ESP32S3_ANALOG_SAR_ENABLE,
				ESP32S3_ANALOG_SAR_ENABLE);
	esp32s3_adc_update_bits(adc->sens + ESP32S3_ADC_CLK_GATE,
				ESP32S3_ADC_CLK_ENABLE,
				ESP32S3_ADC_CLK_ENABLE);
	esp32s3_adc_update_bits(adc->sens + ESP32S3_ADC_POWER,
				ESP32S3_ADC_POWER_FORCE,
				FIELD_PREP(ESP32S3_ADC_POWER_FORCE,
					   ESP32S3_ADC_POWER_ON));
}

static void esp32s3_adc_power_off(struct esp32s3_adc *adc)
{
	/* Return power control to the hardware FSM after each direct read. */
	esp32s3_adc_update_bits(adc->sens + ESP32S3_ADC_POWER,
				ESP32S3_ADC_POWER_FORCE,
				FIELD_PREP(ESP32S3_ADC_POWER_FORCE,
					   ESP32S3_ADC_POWER_FSM));
	esp32s3_adc_update_bits(adc->analog + ESP32S3_ANALOG_CONFIG2,
				ESP32S3_ANALOG_SAR_ENABLE, 0);
}

static int esp32s3_adc_convert(struct esp32s3_adc *adc,
			       unsigned int channel, int *value)
{
	u32 atten_mask;
	u32 atten_value;
	u32 pad;
	u32 reg;
	int ret;

	ret = esp32s3_adc_encode_channel(channel, adc->attenuation[channel],
					 &pad, &atten_mask, &atten_value);
	if (ret)
		return ret;

	esp32s3_adc_power_on(adc);
	ret = readl_poll_timeout(adc->sens + ESP32S3_ADC_SLAVE_ADDR1, reg,
				 !(reg & ESP32S3_ADC_MEAS_STATUS), 1,
				 ESP32S3_ADC_TIMEOUT_US);
	if (ret)
		goto out_power_off;

	esp32s3_adc_update_bits(adc->sens + ESP32S3_ADC_READER1_CTRL,
				ESP32S3_ADC_READER1_DATA_INV |
				ESP32S3_ADC_READER1_CLK_DIV,
				FIELD_PREP(ESP32S3_ADC_READER1_CLK_DIV,
					   ESP32S3_ADC_READER1_CLK_DIV_DEFAULT));
	esp32s3_adc_update_bits(adc->sens + ESP32S3_ADC_MEAS1_MUX,
				ESP32S3_ADC_MEAS1_DIG_FORCE, 0);
	esp32s3_adc_update_bits(adc->sens + ESP32S3_ADC_ATTEN1,
				atten_mask, atten_value);
	esp32s3_adc_update_bits(adc->sens + ESP32S3_ADC_MEAS1_CTRL2,
				ESP32S3_ADC_MEAS1_EN_PAD_FORCE |
				ESP32S3_ADC_MEAS1_EN_PAD |
				ESP32S3_ADC_MEAS1_START_FORCE |
				ESP32S3_ADC_MEAS1_START,
				ESP32S3_ADC_MEAS1_EN_PAD_FORCE |
				FIELD_PREP(ESP32S3_ADC_MEAS1_EN_PAD, pad) |
				ESP32S3_ADC_MEAS1_START_FORCE);
	esp32s3_adc_update_bits(adc->sens + ESP32S3_ADC_MEAS1_CTRL2,
				ESP32S3_ADC_MEAS1_START,
				ESP32S3_ADC_MEAS1_START);

	ret = readl_poll_timeout(adc->sens + ESP32S3_ADC_MEAS1_CTRL2, reg,
				 reg & ESP32S3_ADC_MEAS1_DONE, 1,
				 ESP32S3_ADC_TIMEOUT_US);
	if (!ret)
		*value = FIELD_GET(ESP32S3_ADC_MEAS1_DATA, reg) &
			 ESP32S3_ADC_RAW_MASK;

	esp32s3_adc_update_bits(adc->sens + ESP32S3_ADC_MEAS1_CTRL2,
				ESP32S3_ADC_MEAS1_EN_PAD |
				ESP32S3_ADC_MEAS1_START, 0);
out_power_off:
	esp32s3_adc_power_off(adc);
	return ret;
}

static int esp32s3_adc_read_raw(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				int *value, int *value2, long mask)
{
	struct esp32s3_adc *adc = iio_priv(indio_dev);
	int ret;

	if (mask != IIO_CHAN_INFO_RAW)
		return -EINVAL;
	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;

	mutex_lock(&adc->lock);
	ret = esp32s3_adc_convert(adc, chan->channel, value);
	mutex_unlock(&adc->lock);
	iio_device_release_direct(indio_dev);

	return ret ? ret : IIO_VAL_INT;
}

static const struct iio_info esp32s3_adc_info = {
	.read_raw = esp32s3_adc_read_raw,
};

static int esp32s3_adc_parse_channels(struct device *dev,
				      struct iio_dev *indio_dev)
{
	struct esp32s3_adc *adc = iio_priv(indio_dev);
	unsigned long seen = 0;
	unsigned int count = 0;
	const char *label;
	u32 attenuation;
	u32 channel;
	int ret;

	adc->channels = devm_kcalloc(dev, ESP32S3_ADC_MAX_CHANNEL + 1,
				     sizeof(*adc->channels), GFP_KERNEL);
	if (!adc->channels)
		return -ENOMEM;

	fwnode_for_each_available_child_node_scoped(dev_fwnode(dev), child) {
		ret = fwnode_property_read_u32(child, "reg", &channel);
		if (ret)
			return dev_err_probe(dev, ret,
					     "channel is missing reg\n");
		if (channel > ESP32S3_ADC_MAX_CHANNEL ||
		    test_and_set_bit(channel, &seen))
			return dev_err_probe(dev, -EINVAL,
					     "invalid or duplicate channel %u\n",
					     channel);

		ret = fwnode_property_read_u32(child, "esp,attenuation",
					       &attenuation);
		if (ret)
			return dev_err_probe(dev, ret,
					     "channel %u is missing attenuation\n",
					     channel);
		if (attenuation > ESP32S3_ADC_MAX_ATTENUATION)
			return dev_err_probe(dev, -EINVAL,
					     "invalid attenuation %u on channel %u\n",
					     attenuation, channel);

		adc->attenuation[channel] = attenuation;
		adc->channels[count].type = IIO_VOLTAGE;
		adc->channels[count].indexed = 1;
		adc->channels[count].channel = channel;
		adc->channels[count].info_mask_separate =
			BIT(IIO_CHAN_INFO_RAW);
		adc->channels[count].scan_index = -1;
		ret = fwnode_property_read_string(child, "label", &label);
		if (!ret)
			adc->channels[count].datasheet_name = label;
		else if (ret != -EINVAL)
			return dev_err_probe(dev, ret,
					     "invalid channel %u label\n",
					     channel);
		count++;
	}

	if (!count)
		return dev_err_probe(dev, -EINVAL, "no ADC1 channels configured\n");

	indio_dev->channels = adc->channels;
	indio_dev->num_channels = count;
	return 0;
}

static int esp32s3_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32s3_adc *adc;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;
	adc = iio_priv(indio_dev);
	mutex_init(&adc->lock);

	adc->sens = devm_platform_ioremap_resource_byname(pdev, "sens");
	if (IS_ERR(adc->sens))
		return PTR_ERR(adc->sens);
	adc->analog = devm_platform_ioremap_resource_byname(pdev, "analog");
	if (IS_ERR(adc->analog))
		return PTR_ERR(adc->analog);

	ret = esp32s3_adc_parse_channels(dev, indio_dev);
	if (ret)
		return ret;

	indio_dev->name = "esp32s3-adc1";
	indio_dev->info = &esp32s3_adc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id esp32s3_adc_of_match[] = {
	{ .compatible = "esp,esp32s3-adc" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s3_adc_of_match);

static struct platform_driver esp32s3_adc_driver = {
	.probe = esp32s3_adc_probe,
	.driver = {
		.name = "esp32s3-adc",
		.of_match_table = esp32s3_adc_of_match,
	},
};
module_platform_driver(esp32s3_adc_driver);

MODULE_DESCRIPTION("Espressif ESP32-S3 ADC1 IIO driver");
MODULE_LICENSE("GPL");
