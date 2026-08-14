// SPDX-License-Identifier: GPL-2.0-only
/* Espressif ESP32-S3 LED PWM controller driver */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/reset.h>

#include "pwm-esp32s3.h"

#define ESP32S3_LEDC_CHANNELS		8
#define ESP32S3_LEDC_TIMERS		4

#define ESP32S3_LEDC_CH_STRIDE		0x14
#define ESP32S3_LEDC_CH_CONF0		0x00
#define ESP32S3_LEDC_CH_HPOINT		0x04
#define ESP32S3_LEDC_CH_DUTY		0x08
#define ESP32S3_LEDC_CH_CONF1		0x0c
#define ESP32S3_LEDC_TIMER_BASE		0xa0
#define ESP32S3_LEDC_TIMER_STRIDE	0x08
#define ESP32S3_LEDC_GLOBAL_CONF		0xd0

#define ESP32S3_LEDC_CH_TIMER		GENMASK(1, 0)
#define ESP32S3_LEDC_CH_SIG_OUT_EN	BIT(2)
#define ESP32S3_LEDC_CH_IDLE_LEVEL	BIT(3)
#define ESP32S3_LEDC_CH_UPDATE		BIT(4)
#define ESP32S3_LEDC_CH_DUTY_MASK	GENMASK(18, 0)
#define ESP32S3_LEDC_CH_DUTY_SHIFT	4
#define ESP32S3_LEDC_DUTY_SCALE		GENMASK(9, 0)
#define ESP32S3_LEDC_DUTY_CYCLE		GENMASK(19, 10)
#define ESP32S3_LEDC_DUTY_NUM		GENMASK(29, 20)
#define ESP32S3_LEDC_DUTY_INC		BIT(30)
#define ESP32S3_LEDC_DUTY_START		BIT(31)

#define ESP32S3_LEDC_TIMER_RESOLUTION	GENMASK(3, 0)
#define ESP32S3_LEDC_TIMER_DIVIDER	GENMASK(21, 4)
#define ESP32S3_LEDC_TIMER_UPDATE	BIT(25)

#define ESP32S3_LEDC_GLOBAL_CLK_SEL	GENMASK(1, 0)
#define ESP32S3_LEDC_GLOBAL_CLK_XTAL	3
#define ESP32S3_LEDC_GLOBAL_CLK_EN	BIT(31)

struct esp32s3_ledc_timer {
	struct esp32s3_ledc_timer_config config;
	unsigned long channels;
};

struct esp32s3_ledc_channel {
	struct pwm_state state;
	int timer;
};

struct esp32s3_ledc {
	void __iomem *base;
	unsigned long rate;
	struct esp32s3_ledc_timer timers[ESP32S3_LEDC_TIMERS];
	struct esp32s3_ledc_channel channels[ESP32S3_LEDC_CHANNELS];
};

static void __iomem *
esp32s3_ledc_channel_base(struct esp32s3_ledc *ledc, unsigned int channel)
{
	return ledc->base + channel * ESP32S3_LEDC_CH_STRIDE;
}

static void __iomem *
esp32s3_ledc_timer_reg(struct esp32s3_ledc *ledc, unsigned int timer)
{
	return ledc->base + ESP32S3_LEDC_TIMER_BASE +
		timer * ESP32S3_LEDC_TIMER_STRIDE;
}

static bool
esp32s3_ledc_timer_matches(const struct esp32s3_ledc_timer *timer,
			   const struct esp32s3_ledc_timer_config *config)
{
	return timer->config.divider == config->divider &&
		timer->config.resolution == config->resolution;
}

static int
esp32s3_ledc_find_timer(struct esp32s3_ledc *ledc,
			const struct esp32s3_ledc_timer_config *config)
{
	unsigned int i;

	for (i = 0; i < ESP32S3_LEDC_TIMERS; i++)
		if (ledc->timers[i].channels &&
		    esp32s3_ledc_timer_matches(&ledc->timers[i], config))
			return i;
	for (i = 0; i < ESP32S3_LEDC_TIMERS; i++)
		if (!ledc->timers[i].channels)
			return i;

	return -EBUSY;
}

static void
esp32s3_ledc_program_timer(struct esp32s3_ledc *ledc, unsigned int timer,
			   const struct esp32s3_ledc_timer_config *config)
{
	u32 value = FIELD_PREP(ESP32S3_LEDC_TIMER_RESOLUTION,
			       config->resolution) |
		FIELD_PREP(ESP32S3_LEDC_TIMER_DIVIDER, config->divider) |
		ESP32S3_LEDC_TIMER_UPDATE;

	writel(value, esp32s3_ledc_timer_reg(ledc, timer));
	ledc->timers[timer].config = *config;
}

static void esp32s3_ledc_unbind_channel(struct esp32s3_ledc *ledc,
					unsigned int channel)
{
	int timer = ledc->channels[channel].timer;

	if (timer >= 0)
		__clear_bit(channel, &ledc->timers[timer].channels);
	ledc->channels[channel].timer = -1;
}

static void esp32s3_ledc_set_constant(struct esp32s3_ledc *ledc,
				      unsigned int channel, bool level)
{
	void __iomem *base = esp32s3_ledc_channel_base(ledc, channel);
	u32 conf0 = ESP32S3_LEDC_CH_UPDATE;

	if (level)
		conf0 |= ESP32S3_LEDC_CH_IDLE_LEVEL;
	writel(0, base + ESP32S3_LEDC_CH_CONF1);
	writel(0, base + ESP32S3_LEDC_CH_DUTY);
	writel(conf0, base + ESP32S3_LEDC_CH_CONF0);
}

static void esp32s3_ledc_program_channel(struct esp32s3_ledc *ledc,
					 unsigned int channel,
					 unsigned int timer, u32 duty,
					 bool inverted)
{
	void __iomem *base = esp32s3_ledc_channel_base(ledc, channel);
	u32 conf0 = FIELD_PREP(ESP32S3_LEDC_CH_TIMER, timer) |
		ESP32S3_LEDC_CH_SIG_OUT_EN | ESP32S3_LEDC_CH_UPDATE;
	u32 conf1 = FIELD_PREP(ESP32S3_LEDC_DUTY_CYCLE, 1) |
		FIELD_PREP(ESP32S3_LEDC_DUTY_NUM, 1) |
		ESP32S3_LEDC_DUTY_INC | ESP32S3_LEDC_DUTY_START;
	u32 duty_value = duty << ESP32S3_LEDC_CH_DUTY_SHIFT;

	if (inverted)
		conf0 |= ESP32S3_LEDC_CH_IDLE_LEVEL;
	writel(0, base + ESP32S3_LEDC_CH_HPOINT);
	writel(FIELD_PREP(ESP32S3_LEDC_CH_DUTY_MASK, duty_value),
	       base + ESP32S3_LEDC_CH_DUTY);
	writel(conf1, base + ESP32S3_LEDC_CH_CONF1);
	writel(conf0, base + ESP32S3_LEDC_CH_CONF0);
}

static int esp32s3_ledc_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			      const struct pwm_state *state)
{
	struct esp32s3_ledc *ledc = pwmchip_get_drvdata(chip);
	struct esp32s3_ledc_timer_config config;
	unsigned int channel = pwm->hwpwm;
	bool inverted = state->polarity == PWM_POLARITY_INVERSED;
	u64 duty_ns = state->duty_cycle;
	u64 period_ns = state->period;
	u32 duty;
	int old_timer;
	int timer;
	int ret;

	if (!state->period || state->duty_cycle > state->period)
		return -EINVAL;

	if (!state->enabled || !state->duty_cycle ||
	    state->duty_cycle == state->period) {
		bool level = inverted;

		if (state->enabled && state->duty_cycle == state->period)
			level = !level;
		esp32s3_ledc_unbind_channel(ledc, channel);
		esp32s3_ledc_set_constant(ledc, channel, level);
		ledc->channels[channel].state = *state;
		return 0;
	}

	ret = esp32s3_ledc_calc_timer(ledc->rate, state->period, &config);
	if (ret)
		return ret;

	old_timer = ledc->channels[channel].timer;
	if (old_timer >= 0)
		__clear_bit(channel, &ledc->timers[old_timer].channels);
	timer = esp32s3_ledc_find_timer(ledc, &config);
	if (timer < 0) {
		if (old_timer >= 0)
			__set_bit(channel, &ledc->timers[old_timer].channels);
		return timer;
	}
	if (!esp32s3_ledc_timer_matches(&ledc->timers[timer], &config))
		esp32s3_ledc_program_timer(ledc, timer, &config);

	duty = esp32s3_ledc_duty_ticks(duty_ns, period_ns, config.resolution, inverted);
	duty = clamp(duty, 1U, BIT(config.resolution) - 1);
	esp32s3_ledc_program_channel(ledc, channel, timer, duty, inverted);
	__set_bit(channel, &ledc->timers[timer].channels);
	ledc->channels[channel].timer = timer;
	ledc->channels[channel].state = *state;

	return 0;
}

static int esp32s3_ledc_get_state(struct pwm_chip *chip,
				  struct pwm_device *pwm,
				  struct pwm_state *state)
{
	struct esp32s3_ledc *ledc = pwmchip_get_drvdata(chip);

	*state = ledc->channels[pwm->hwpwm].state;
	return 0;
}

static const struct pwm_ops esp32s3_ledc_ops = {
	.apply = esp32s3_ledc_apply,
	.get_state = esp32s3_ledc_get_state,
};

static int esp32s3_ledc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32s3_ledc *ledc;
	struct pwm_chip *chip;
	struct clk *clk;
	struct reset_control *rst;
	unsigned int i;
	int ret;

	chip = devm_pwmchip_alloc(dev, ESP32S3_LEDC_CHANNELS, sizeof(*ledc));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	ledc = pwmchip_get_drvdata(chip);

	ledc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ledc->base))
		return PTR_ERR(ledc->base);
	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "failed to enable clock\n");
	rst = devm_reset_control_get_exclusive_deasserted(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst),
				     "failed to deassert reset\n");

	ledc->rate = clk_get_rate(clk);
	if (!ledc->rate)
		return dev_err_probe(dev, -EINVAL, "invalid source clock\n");
	writel(ESP32S3_LEDC_GLOBAL_CLK_EN |
	       FIELD_PREP(ESP32S3_LEDC_GLOBAL_CLK_SEL,
			  ESP32S3_LEDC_GLOBAL_CLK_XTAL),
	       ledc->base + ESP32S3_LEDC_GLOBAL_CONF);
	for (i = 0; i < ESP32S3_LEDC_CHANNELS; i++) {
		ledc->channels[i].timer = -1;
		esp32s3_ledc_set_constant(ledc, i, false);
	}

	chip->ops = &esp32s3_ledc_ops;
	ret = devm_pwmchip_add(dev, chip);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register PWM controller\n");

	return 0;
}

static const struct of_device_id esp32s3_ledc_of_match[] = {
	{ .compatible = "esp,esp32s3-ledc" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s3_ledc_of_match);

static struct platform_driver esp32s3_ledc_driver = {
	.probe = esp32s3_ledc_probe,
	.driver = {
		.name = "esp32s3-ledc",
		.of_match_table = esp32s3_ledc_of_match,
	},
};
module_platform_driver(esp32s3_ledc_driver);

MODULE_DESCRIPTION("Espressif ESP32-S3 LED PWM controller");
MODULE_LICENSE("GPL");
