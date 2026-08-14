/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ESP32S3_ADC_H
#define _ESP32S3_ADC_H

#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/types.h>

#define ESP32S3_ADC_MAX_CHANNEL		9
#define ESP32S3_ADC_MAX_ATTENUATION	3

static inline int esp32s3_adc_encode_channel(unsigned int channel,
					     unsigned int attenuation,
					      u32 *pad, u32 *atten_mask,
					      u32 *atten_value)
{
	unsigned int shift;

	if (channel > ESP32S3_ADC_MAX_CHANNEL ||
	    attenuation > ESP32S3_ADC_MAX_ATTENUATION ||
	    !pad || !atten_mask || !atten_value)
		return -EINVAL;

	shift = channel * 2;
	*pad = BIT(channel);
	*atten_mask = GENMASK(shift + 1, shift);
	*atten_value = attenuation << shift;

	return 0;
}

#endif /* _ESP32S3_ADC_H */
