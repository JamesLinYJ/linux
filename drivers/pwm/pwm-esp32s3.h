/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _PWM_ESP32S3_H
#define _PWM_ESP32S3_H

#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/time.h>
#include <linux/types.h>

#define ESP32S3_LEDC_MAX_RESOLUTION	14
#define ESP32S3_LEDC_DIV_MIN		256
#define ESP32S3_LEDC_DIV_MAX		GENMASK(17, 0)

struct esp32s3_ledc_timer_config {
	u32 divider;
	u8 resolution;
	u64 period_ns;
};

static inline int
esp32s3_ledc_calc_timer(unsigned long rate, u64 period_ns,
			struct esp32s3_ledc_timer_config *config)
{
	u64 denominator;
	u64 divider;
	u64 actual;
	u64 scaled_rate;
	u64 scaled_ticks;
	int resolution;

	if (!rate || !period_ns || !config)
		return -EINVAL;
	scaled_rate = (u64)rate << 8;

	for (resolution = ESP32S3_LEDC_MAX_RESOLUTION;
	     resolution >= 1; resolution--) {
		denominator = (u64)NSEC_PER_SEC << resolution;
		divider = mul_u64_u64_div_u64(scaled_rate, period_ns, denominator);
		if (divider < ESP32S3_LEDC_DIV_MIN ||
		    divider > ESP32S3_LEDC_DIV_MAX)
			continue;

		scaled_ticks = divider << resolution;
		actual = mul_u64_u64_div_u64(scaled_ticks, NSEC_PER_SEC, scaled_rate);
		config->divider = divider;
		config->resolution = resolution;
		config->period_ns = actual;
		return 0;
	}

	return -ERANGE;
}

static inline u32 esp32s3_ledc_duty_ticks(u64 duty_ns, u64 period_ns,
					  u8 resolution, bool inverted)
{
	u32 steps = BIT(resolution);
	u32 ticks;

	if (!period_ns)
		return 0;
	ticks = mul_u64_u64_div_u64(duty_ns, steps, period_ns);
	ticks = min(ticks, steps);

	return inverted ? steps - ticks : ticks;
}

#endif /* _PWM_ESP32S3_H */
