// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif ESP32-S3 pin control, GPIO, and GPIO interrupt driver
 */

#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/bits.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>

#include "core.h"
#include "pinctrl-utils.h"
#include "pinmux.h"

#define ESP32S3_GPIO_NR_PINS		49

#define ESP32S3_GPIO_OUT			0x004
#define ESP32S3_GPIO_OUT_W1TS		0x008
#define ESP32S3_GPIO_OUT_W1TC		0x00c
#define ESP32S3_GPIO_OUT1		0x010
#define ESP32S3_GPIO_OUT1_W1TS		0x014
#define ESP32S3_GPIO_OUT1_W1TC		0x018
#define ESP32S3_GPIO_ENABLE		0x020
#define ESP32S3_GPIO_ENABLE_W1TS	0x024
#define ESP32S3_GPIO_ENABLE_W1TC	0x028
#define ESP32S3_GPIO_ENABLE1		0x02c
#define ESP32S3_GPIO_ENABLE1_W1TS	0x030
#define ESP32S3_GPIO_ENABLE1_W1TC	0x034
#define ESP32S3_GPIO_IN			0x03c
#define ESP32S3_GPIO_IN1			0x040
#define ESP32S3_GPIO_STATUS_W1TC	0x04c
#define ESP32S3_GPIO_STATUS1_W1TC	0x058
#define ESP32S3_GPIO_PCPU_INT		0x05c
#define ESP32S3_GPIO_PCPU_INT1		0x068
#define ESP32S3_GPIO_PIN_BASE		0x074
#define ESP32S3_GPIO_OUT_SEL_BASE	0x554

#define ESP32S3_GPIO_PIN_INT_TYPE	GENMASK(9, 7)
#define ESP32S3_GPIO_PIN_INT_ENA	GENMASK(17, 13)
#define ESP32S3_GPIO_PIN_PAD_DRIVER	BIT(2)
#define ESP32S3_GPIO_PIN_INT_CPU0	FIELD_PREP(ESP32S3_GPIO_PIN_INT_ENA, 1)

#define ESP32S3_GPIO_INTR_DISABLE	0
#define ESP32S3_GPIO_INTR_POSEDGE	1
#define ESP32S3_GPIO_INTR_NEGEDGE	2
#define ESP32S3_GPIO_INTR_ANYEDGE	3
#define ESP32S3_GPIO_INTR_LOW_LEVEL	4
#define ESP32S3_GPIO_INTR_HIGH_LEVEL	5

#define ESP32S3_GPIO_OUT_SEL		GENMASK(8, 0)
#define ESP32S3_GPIO_MATRIX_GPIO_OUT	256
#define ESP32S3_GPIO_IN_SEL_BASE		0x154
#define ESP32S3_GPIO_IN_SEL		GENMASK(5, 0)
#define ESP32S3_GPIO_IN_SEL_MATRIX	BIT(7)

#define ESP32S3_IOMUX_PIN_BASE		0x004
#define ESP32S3_IOMUX_PIN_FUNC		GENMASK(14, 12)
#define ESP32S3_IOMUX_PIN_FUNC_GPIO	1
#define ESP32S3_IOMUX_PIN_PULL_DOWN	BIT(7)
#define ESP32S3_IOMUX_PIN_PULL_UP	BIT(8)
#define ESP32S3_IOMUX_PIN_INPUT_ENABLE	BIT(9)

#define ESP32S3_RTCIO_ENABLE_W1TC	0x014
#define ESP32S3_RTCIO_TOUCH_PAD_BASE	0x084
#define ESP32S3_RTCIO_TOUCH_PAD_MUX	BIT(19)
#define ESP32S3_RTCIO_TOUCH_PAD_FUNC	GENMASK(18, 17)
#define ESP32S3_RTCIO_TOUCH_PAD_INPUT	BIT(13)
#define ESP32S3_RTCIO_TOUCH_PAD_PULL_DOWN BIT(28)
#define ESP32S3_RTCIO_TOUCH_PAD_PULL_UP	BIT(27)
#define ESP32S3_RTCIO_OUTPUT_SHIFT	10
#define ESP32S3_RTCIO_TOUCH_PAD_LAST	14

struct esp32s3_pinctrl {
	struct device *dev;
	void __iomem *gpio_base;
	void __iomem *iomux_base;
	void __iomem *rtcio_base;
	raw_spinlock_t lock;
	struct pinctrl_dev *pctldev;
	struct pinctrl_desc pctldesc;
	struct gpio_chip gpio;
};

struct esp32s3_pin_function {
	const char *name;
	u16 signal;
	bool input_enable;
	bool open_drain;
	bool analog;
};

static const struct esp32s3_pin_function esp32s3_pin_functions[] = {
	{ "gpio", ESP32S3_GPIO_MATRIX_GPIO_OUT },
	{ "analog", ESP32S3_GPIO_MATRIX_GPIO_OUT, .analog = true },
	{ "i2c0-scl", 89, true, true },
	{ "i2c0-sda", 90, true, true },
	{ "i2c1-scl", 91, true, true },
	{ "i2c1-sda", 92, true, true },
	{ "spi2-clk", 101 },
	{ "spi2-q", 102, true },
	{ "spi2-d", 103, true },
	{ "spi2-cs0", 110 },
	{ "spi3-clk", 66 },
	{ "spi3-q", 67, true },
	{ "spi3-d", 68, true },
	{ "spi3-hd", 69, true },
	{ "spi3-wp", 70, true },
	{ "spi3-cs0", 71 },
	{ "ledc-ls0", 73 },
	{ "ledc-ls1", 74 },
	{ "ledc-ls2", 75 },
	{ "ledc-ls3", 76 },
	{ "ledc-ls4", 77 },
	{ "ledc-ls5", 78 },
	{ "ledc-ls6", 79 },
	{ "ledc-ls7", 80 },
};

static bool esp32s3_gpio_pin_valid(unsigned int pin)
{
	return pin < ESP32S3_GPIO_NR_PINS && (pin < 22 || pin > 25);
}

static void __iomem *esp32s3_gpio_pin_reg(struct esp32s3_pinctrl *pctl,
					  unsigned int pin)
{
	return pctl->gpio_base + ESP32S3_GPIO_PIN_BASE + pin * sizeof(u32);
}

static void __iomem *esp32s3_iomux_pin_reg(struct esp32s3_pinctrl *pctl,
					   unsigned int pin)
{
	return pctl->iomux_base + ESP32S3_IOMUX_PIN_BASE + pin * sizeof(u32);
}

static void esp32s3_update_bits(struct esp32s3_pinctrl *pctl,
				void __iomem *reg, u32 mask, u32 value)
{
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&pctl->lock, flags);
	val = readl_relaxed(reg);
	val = (val & ~mask) | (value & mask);
	writel_relaxed(val, reg);
	raw_spin_unlock_irqrestore(&pctl->lock, flags);
}

static void esp32s3_gpio_set_value(struct esp32s3_pinctrl *pctl,
				   unsigned int pin, bool value)
{
	void __iomem *reg;
	u32 bit;

	if (pin < 32) {
		reg = pctl->gpio_base + (value ? ESP32S3_GPIO_OUT_W1TS :
						 ESP32S3_GPIO_OUT_W1TC);
		bit = BIT(pin);
	} else {
		reg = pctl->gpio_base + (value ? ESP32S3_GPIO_OUT1_W1TS :
						 ESP32S3_GPIO_OUT1_W1TC);
		bit = BIT(pin - 32);
	}
	writel_relaxed(bit, reg);
}

static void esp32s3_gpio_set_output_enable(struct esp32s3_pinctrl *pctl,
					   unsigned int pin, bool enable)
{
	void __iomem *reg;
	u32 bit;

	if (pin < 32) {
		reg = pctl->gpio_base + (enable ? ESP32S3_GPIO_ENABLE_W1TS :
						 ESP32S3_GPIO_ENABLE_W1TC);
		bit = BIT(pin);
	} else {
		reg = pctl->gpio_base + (enable ? ESP32S3_GPIO_ENABLE1_W1TS :
						 ESP32S3_GPIO_ENABLE1_W1TC);
		bit = BIT(pin - 32);
	}
	writel_relaxed(bit, reg);
}

static void esp32s3_select_gpio(struct esp32s3_pinctrl *pctl,
				unsigned int pin)
{
	esp32s3_update_bits(pctl, esp32s3_iomux_pin_reg(pctl, pin),
			    ESP32S3_IOMUX_PIN_FUNC,
			    FIELD_PREP(ESP32S3_IOMUX_PIN_FUNC,
				       ESP32S3_IOMUX_PIN_FUNC_GPIO));
	esp32s3_update_bits(pctl,
			    pctl->gpio_base + ESP32S3_GPIO_OUT_SEL_BASE +
			    pin * sizeof(u32), ESP32S3_GPIO_OUT_SEL,
			    ESP32S3_GPIO_MATRIX_GPIO_OUT);
}

static int esp32s3_select_analog(struct esp32s3_pinctrl *pctl,
				 unsigned int pin)
{
	void __iomem *rtcio_reg;
	u32 mask;

	if (pin > ESP32S3_RTCIO_TOUCH_PAD_LAST)
		return -EINVAL;

	esp32s3_select_gpio(pctl, pin);
	esp32s3_gpio_set_output_enable(pctl, pin, false);
	esp32s3_update_bits(pctl, esp32s3_iomux_pin_reg(pctl, pin),
			    ESP32S3_IOMUX_PIN_INPUT_ENABLE |
			    ESP32S3_IOMUX_PIN_PULL_UP |
			    ESP32S3_IOMUX_PIN_PULL_DOWN, 0);

	/*
	 * ADC pads 0..14 have a second RTC I/O mux.  Keep the pad on the
	 * digital GPIO path, but disable both digital and RTC input/output
	 * buffers and pulls so the external voltage reaches the SAR ADC.
	 */
	rtcio_reg = pctl->rtcio_base + ESP32S3_RTCIO_TOUCH_PAD_BASE +
		     pin * sizeof(u32);
	mask = ESP32S3_RTCIO_TOUCH_PAD_MUX |
	       ESP32S3_RTCIO_TOUCH_PAD_FUNC |
	       ESP32S3_RTCIO_TOUCH_PAD_INPUT |
	       ESP32S3_RTCIO_TOUCH_PAD_PULL_UP |
	       ESP32S3_RTCIO_TOUCH_PAD_PULL_DOWN;
	esp32s3_update_bits(pctl, rtcio_reg, mask, 0);
	writel_relaxed(BIT(pin + ESP32S3_RTCIO_OUTPUT_SHIFT),
		       pctl->rtcio_base + ESP32S3_RTCIO_ENABLE_W1TC);

	return 0;
}

static void esp32s3_select_matrix_signal(struct esp32s3_pinctrl *pctl,
					 unsigned int pin,
					 const struct esp32s3_pin_function *func)
{
	u32 iomux_value = FIELD_PREP(ESP32S3_IOMUX_PIN_FUNC,
				     ESP32S3_IOMUX_PIN_FUNC_GPIO);

	if (func->input_enable)
		iomux_value |= ESP32S3_IOMUX_PIN_INPUT_ENABLE;
	esp32s3_gpio_set_value(pctl, pin, true);
	esp32s3_update_bits(pctl, esp32s3_iomux_pin_reg(pctl, pin),
			    ESP32S3_IOMUX_PIN_FUNC |
			    ESP32S3_IOMUX_PIN_INPUT_ENABLE,
			    iomux_value);
	esp32s3_update_bits(pctl, esp32s3_gpio_pin_reg(pctl, pin),
			    ESP32S3_GPIO_PIN_PAD_DRIVER,
			    func->open_drain ? ESP32S3_GPIO_PIN_PAD_DRIVER : 0);
	esp32s3_update_bits(pctl,
			    pctl->gpio_base + ESP32S3_GPIO_OUT_SEL_BASE +
			    pin * sizeof(u32), GENMASK(11, 0), func->signal);
	if (func->input_enable) {
		void __iomem *input_reg = pctl->gpio_base +
			ESP32S3_GPIO_IN_SEL_BASE + func->signal * sizeof(u32);

		esp32s3_update_bits(pctl, input_reg,
				    ESP32S3_GPIO_IN_SEL |
				    ESP32S3_GPIO_IN_SEL_MATRIX,
				    FIELD_PREP(ESP32S3_GPIO_IN_SEL, pin) |
				    ESP32S3_GPIO_IN_SEL_MATRIX);
	}
	esp32s3_gpio_set_output_enable(pctl, pin, true);
}

static const struct pinctrl_ops esp32s3_pinctrl_ops = {
	.get_groups_count = pinctrl_generic_get_group_count,
	.get_group_name = pinctrl_generic_get_group_name,
	.get_group_pins = pinctrl_generic_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_all,
	.dt_free_map = pinctrl_utils_free_map,
};

static int esp32s3_pinmux_set(struct pinctrl_dev *pctldev,
			      unsigned int function, unsigned int group)
{
	struct esp32s3_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	if (function >= ARRAY_SIZE(esp32s3_pin_functions) ||
	    !esp32s3_gpio_pin_valid(group))
		return -EINVAL;
	if (esp32s3_pin_functions[function].analog)
		return esp32s3_select_analog(pctl, group);
	if (!function)
		esp32s3_select_gpio(pctl, group);
	else
		esp32s3_select_matrix_signal(pctl, group,
					     &esp32s3_pin_functions[function]);
	return 0;
}

static int esp32s3_gpio_request_enable(struct pinctrl_dev *pctldev,
				       struct pinctrl_gpio_range *range,
				       unsigned int pin)
{
	struct esp32s3_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

	if (!esp32s3_gpio_pin_valid(pin))
		return -EINVAL;
	esp32s3_select_gpio(pctl, pin);
	return 0;
}

static const struct pinmux_ops esp32s3_pinmux_ops = {
	.get_functions_count = pinmux_generic_get_function_count,
	.get_function_name = pinmux_generic_get_function_name,
	.get_function_groups = pinmux_generic_get_function_groups,
	.set_mux = esp32s3_pinmux_set,
	.gpio_request_enable = esp32s3_gpio_request_enable,
	.strict = true,
};

static int esp32s3_pinconf_get(struct pinctrl_dev *pctldev, unsigned int pin,
			       unsigned long *config)
{
	struct esp32s3_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param = pinconf_to_config_param(*config);
	u32 gpio = readl_relaxed(esp32s3_gpio_pin_reg(pctl, pin));
	u32 iomux = readl_relaxed(esp32s3_iomux_pin_reg(pctl, pin));
	u32 arg = pinconf_to_config_argument(*config);
	bool enabled;

	if (!esp32s3_gpio_pin_valid(pin))
		return -EINVAL;
	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		enabled = !(iomux & (ESP32S3_IOMUX_PIN_PULL_UP |
				     ESP32S3_IOMUX_PIN_PULL_DOWN));
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		enabled = iomux & ESP32S3_IOMUX_PIN_PULL_UP;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		enabled = iomux & ESP32S3_IOMUX_PIN_PULL_DOWN;
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		enabled = gpio & ESP32S3_GPIO_PIN_PAD_DRIVER;
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		enabled = !(gpio & ESP32S3_GPIO_PIN_PAD_DRIVER);
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		enabled = !!(iomux & ESP32S3_IOMUX_PIN_INPUT_ENABLE) == !!arg;
		break;
	default:
		return -ENOTSUPP;
	}
	if (!enabled)
		return -EINVAL;
	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static int esp32s3_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
			       unsigned long *configs,
			       unsigned int num_configs)
{
	struct esp32s3_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	unsigned int i;

	if (!esp32s3_gpio_pin_valid(pin))
		return -EINVAL;
	for (i = 0; i < num_configs; i++) {
		enum pin_config_param param =
			pinconf_to_config_param(configs[i]);
		u32 mask;
		u32 value;

		switch (param) {
		case PIN_CONFIG_BIAS_DISABLE:
			mask = ESP32S3_IOMUX_PIN_PULL_UP |
			       ESP32S3_IOMUX_PIN_PULL_DOWN;
			value = 0;
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			mask = ESP32S3_IOMUX_PIN_PULL_UP |
			       ESP32S3_IOMUX_PIN_PULL_DOWN;
			value = ESP32S3_IOMUX_PIN_PULL_UP;
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			mask = ESP32S3_IOMUX_PIN_PULL_UP |
			       ESP32S3_IOMUX_PIN_PULL_DOWN;
			value = ESP32S3_IOMUX_PIN_PULL_DOWN;
			break;
		case PIN_CONFIG_INPUT_ENABLE:
			mask = ESP32S3_IOMUX_PIN_INPUT_ENABLE;
			value = pinconf_to_config_argument(configs[i]) ? mask : 0;
			break;
		case PIN_CONFIG_DRIVE_OPEN_DRAIN:
			esp32s3_update_bits(pctl,
					    esp32s3_gpio_pin_reg(pctl, pin),
					    ESP32S3_GPIO_PIN_PAD_DRIVER,
					    ESP32S3_GPIO_PIN_PAD_DRIVER);
			continue;
		case PIN_CONFIG_DRIVE_PUSH_PULL:
			esp32s3_update_bits(pctl,
					    esp32s3_gpio_pin_reg(pctl, pin),
					    ESP32S3_GPIO_PIN_PAD_DRIVER, 0);
			continue;
		default:
			return -ENOTSUPP;
		}
		esp32s3_update_bits(pctl, esp32s3_iomux_pin_reg(pctl, pin),
				    mask, value);
	}
	return 0;
}

static const struct pinconf_ops esp32s3_pinconf_ops = {
	.pin_config_get = esp32s3_pinconf_get,
	.pin_config_set = esp32s3_pinconf_set,
	.is_generic = true,
};

static int esp32s3_gpio_get_direction(struct gpio_chip *gpio,
				      unsigned int pin)
{
	struct esp32s3_pinctrl *pctl = gpiochip_get_data(gpio);
	u32 value;

	if (pin < 32)
		value = readl_relaxed(pctl->gpio_base + ESP32S3_GPIO_ENABLE);
	else
		value = readl_relaxed(pctl->gpio_base + ESP32S3_GPIO_ENABLE1);
	return value & BIT(pin % 32) ? GPIO_LINE_DIRECTION_OUT :
					GPIO_LINE_DIRECTION_IN;
}

static int esp32s3_gpio_direction_input(struct gpio_chip *gpio,
					unsigned int pin)
{
	struct esp32s3_pinctrl *pctl = gpiochip_get_data(gpio);

	esp32s3_gpio_set_output_enable(pctl, pin, false);
	esp32s3_update_bits(pctl, esp32s3_iomux_pin_reg(pctl, pin),
			    ESP32S3_IOMUX_PIN_INPUT_ENABLE,
			    ESP32S3_IOMUX_PIN_INPUT_ENABLE);
	return 0;
}

static int esp32s3_gpio_direction_output(struct gpio_chip *gpio,
					 unsigned int pin, int value)
{
	struct esp32s3_pinctrl *pctl = gpiochip_get_data(gpio);

	esp32s3_gpio_set_value(pctl, pin, value);
	esp32s3_gpio_set_output_enable(pctl, pin, true);
	return 0;
}

static int esp32s3_gpio_get(struct gpio_chip *gpio, unsigned int pin)
{
	struct esp32s3_pinctrl *pctl = gpiochip_get_data(gpio);
	u32 value;
	bool output = esp32s3_gpio_get_direction(gpio, pin) ==
		      GPIO_LINE_DIRECTION_OUT;

	if (pin < 32)
		value = readl_relaxed(pctl->gpio_base +
				      (output ? ESP32S3_GPIO_OUT :
						ESP32S3_GPIO_IN));
	else
		value = readl_relaxed(pctl->gpio_base +
				      (output ? ESP32S3_GPIO_OUT1 :
						ESP32S3_GPIO_IN1));
	return !!(value & BIT(pin % 32));
}

static int esp32s3_gpio_set(struct gpio_chip *gpio, unsigned int pin,
			    int value)
{
	struct esp32s3_pinctrl *pctl = gpiochip_get_data(gpio);

	esp32s3_gpio_set_value(pctl, pin, value);
	return 0;
}

static int esp32s3_gpio_init_valid_mask(struct gpio_chip *gpio,
					unsigned long *valid_mask,
					unsigned int ngpios)
{
	bitmap_clear(valid_mask, 22, 4);
	return 0;
}

static void esp32s3_gpio_irq_ack(struct irq_data *data)
{
	struct gpio_chip *gpio = irq_data_get_irq_chip_data(data);
	struct esp32s3_pinctrl *pctl = gpiochip_get_data(gpio);

	if (data->hwirq < 32)
		writel_relaxed(BIT(data->hwirq),
			       pctl->gpio_base + ESP32S3_GPIO_STATUS_W1TC);
	else
		writel_relaxed(BIT(data->hwirq - 32),
			       pctl->gpio_base + ESP32S3_GPIO_STATUS1_W1TC);
}

static void esp32s3_gpio_irq_mask(struct irq_data *data)
{
	struct gpio_chip *gpio = irq_data_get_irq_chip_data(data);
	struct esp32s3_pinctrl *pctl = gpiochip_get_data(gpio);

	esp32s3_update_bits(pctl, esp32s3_gpio_pin_reg(pctl, data->hwirq),
			    ESP32S3_GPIO_PIN_INT_ENA, 0);
	gpiochip_disable_irq(gpio, data->hwirq);
}

static void esp32s3_gpio_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *gpio = irq_data_get_irq_chip_data(data);
	struct esp32s3_pinctrl *pctl = gpiochip_get_data(gpio);

	gpiochip_enable_irq(gpio, data->hwirq);
	esp32s3_update_bits(pctl, esp32s3_gpio_pin_reg(pctl, data->hwirq),
			    ESP32S3_GPIO_PIN_INT_ENA,
			    ESP32S3_GPIO_PIN_INT_CPU0);
}

static int esp32s3_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct gpio_chip *gpio = irq_data_get_irq_chip_data(data);
	struct esp32s3_pinctrl *pctl = gpiochip_get_data(gpio);
	u32 hw_type;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		hw_type = ESP32S3_GPIO_INTR_POSEDGE;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		hw_type = ESP32S3_GPIO_INTR_NEGEDGE;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		hw_type = ESP32S3_GPIO_INTR_ANYEDGE;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		hw_type = ESP32S3_GPIO_INTR_LOW_LEVEL;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		hw_type = ESP32S3_GPIO_INTR_HIGH_LEVEL;
		break;
	default:
		return -EINVAL;
	}
	esp32s3_update_bits(pctl, esp32s3_gpio_pin_reg(pctl, data->hwirq),
			    ESP32S3_GPIO_PIN_INT_TYPE,
			    FIELD_PREP(ESP32S3_GPIO_PIN_INT_TYPE, hw_type));
	if (type & IRQ_TYPE_LEVEL_MASK)
		irq_set_handler_locked(data, handle_level_irq);
	else
		irq_set_handler_locked(data, handle_edge_irq);
	return 0;
}

static const struct irq_chip esp32s3_gpio_irqchip = {
	.name = "esp32s3-gpio",
	.irq_ack = esp32s3_gpio_irq_ack,
	.irq_mask = esp32s3_gpio_irq_mask,
	.irq_unmask = esp32s3_gpio_irq_unmask,
	.irq_set_type = esp32s3_gpio_irq_set_type,
	.flags = IRQCHIP_IMMUTABLE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static void esp32s3_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gpio = irq_desc_get_handler_data(desc);
	struct esp32s3_pinctrl *pctl = gpiochip_get_data(gpio);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long pending;
	unsigned int bit;

	chained_irq_enter(chip, desc);
	pending = readl_relaxed(pctl->gpio_base + ESP32S3_GPIO_PCPU_INT);
	for_each_set_bit(bit, &pending, 32)
		generic_handle_domain_irq(gpio->irq.domain, bit);

	pending = readl_relaxed(pctl->gpio_base + ESP32S3_GPIO_PCPU_INT1) &
		  GENMASK(16, 0);
	for_each_set_bit(bit, &pending, 17)
		generic_handle_domain_irq(gpio->irq.domain, bit + 32);
	chained_irq_exit(chip, desc);
}

static int esp32s3_gpio_register(struct platform_device *pdev,
				 struct esp32s3_pinctrl *pctl)
{
	struct gpio_irq_chip *girq = &pctl->gpio.irq;
	int irq;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	pctl->gpio.label = dev_name(&pdev->dev);
	pctl->gpio.parent = &pdev->dev;
	pctl->gpio.owner = THIS_MODULE;
	pctl->gpio.request = gpiochip_generic_request;
	pctl->gpio.free = gpiochip_generic_free;
	pctl->gpio.get_direction = esp32s3_gpio_get_direction;
	pctl->gpio.direction_input = esp32s3_gpio_direction_input;
	pctl->gpio.direction_output = esp32s3_gpio_direction_output;
	pctl->gpio.get = esp32s3_gpio_get;
	pctl->gpio.set = esp32s3_gpio_set;
	pctl->gpio.set_config = gpiochip_generic_config;
	pctl->gpio.init_valid_mask = esp32s3_gpio_init_valid_mask;
	pctl->gpio.base = -1;
	pctl->gpio.ngpio = ESP32S3_GPIO_NR_PINS;
	pctl->gpio.can_sleep = false;

	gpio_irq_chip_set_chip(girq, &esp32s3_gpio_irqchip);
	girq->parent_handler = esp32s3_gpio_irq_handler;
	girq->num_parents = 1;
	girq->parents = devm_kcalloc(&pdev->dev, 1, sizeof(*girq->parents),
				     GFP_KERNEL);
	if (!girq->parents)
		return -ENOMEM;
	girq->parents[0] = irq;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_bad_irq;

	return devm_gpiochip_add_data(&pdev->dev, &pctl->gpio, pctl);
}

static int esp32s3_pinctrl_probe(struct platform_device *pdev)
{
	struct esp32s3_pinctrl *pctl;
	struct pinctrl_pin_desc *pins;
	const char **group_names;
	unsigned int *pin_numbers;
	unsigned int pin;
	int ret;

	pctl = devm_kzalloc(&pdev->dev, sizeof(*pctl), GFP_KERNEL);
	if (!pctl)
		return -ENOMEM;
	pctl->dev = &pdev->dev;
	raw_spin_lock_init(&pctl->lock);

	pctl->gpio_base = devm_platform_ioremap_resource_byname(pdev, "gpio");
	if (IS_ERR(pctl->gpio_base))
		return PTR_ERR(pctl->gpio_base);
	pctl->iomux_base = devm_platform_ioremap_resource_byname(pdev, "iomux");
	if (IS_ERR(pctl->iomux_base))
		return PTR_ERR(pctl->iomux_base);
	pctl->rtcio_base = devm_platform_ioremap_resource_byname(pdev, "rtcio");
	if (IS_ERR(pctl->rtcio_base))
		return PTR_ERR(pctl->rtcio_base);

	pins = devm_kcalloc(&pdev->dev, ESP32S3_GPIO_NR_PINS, sizeof(*pins),
			    GFP_KERNEL);
	group_names = devm_kcalloc(&pdev->dev, ESP32S3_GPIO_NR_PINS,
				   sizeof(*group_names), GFP_KERNEL);
	pin_numbers = devm_kcalloc(&pdev->dev, ESP32S3_GPIO_NR_PINS,
				   sizeof(*pin_numbers), GFP_KERNEL);
	if (!pins || !group_names || !pin_numbers)
		return -ENOMEM;

	for (pin = 0; pin < ESP32S3_GPIO_NR_PINS; pin++) {
		pins[pin].number = pin;
		pins[pin].name = devm_kasprintf(&pdev->dev, GFP_KERNEL,
						"gpio%u", pin);
		if (!pins[pin].name)
			return -ENOMEM;
		group_names[pin] = pins[pin].name;
		pin_numbers[pin] = pin;
	}

	pctl->pctldesc.name = dev_name(&pdev->dev);
	pctl->pctldesc.owner = THIS_MODULE;
	pctl->pctldesc.pins = pins;
	pctl->pctldesc.npins = ESP32S3_GPIO_NR_PINS;
	pctl->pctldesc.pctlops = &esp32s3_pinctrl_ops;
	pctl->pctldesc.pmxops = &esp32s3_pinmux_ops;
	pctl->pctldesc.confops = &esp32s3_pinconf_ops;
	pctl->pctldev = devm_pinctrl_register(&pdev->dev, &pctl->pctldesc,
					      pctl);
	if (IS_ERR(pctl->pctldev))
		return dev_err_probe(&pdev->dev, PTR_ERR(pctl->pctldev),
				     "failed to register pin controller\n");

	for (pin = 0; pin < ESP32S3_GPIO_NR_PINS; pin++) {
		ret = pinctrl_generic_add_group(pctl->pctldev, group_names[pin],
						pin_numbers + pin, 1, pctl);
		if (ret < 0)
			return dev_err_probe(&pdev->dev, ret,
					     "failed to add pin group\n");
	}
	for (pin = 0; pin < ARRAY_SIZE(esp32s3_pin_functions); pin++) {
		ret = pinmux_generic_add_function(pctl->pctldev,
						  esp32s3_pin_functions[pin].name,
						  group_names,
						  ESP32S3_GPIO_NR_PINS, pctl);
		if (ret < 0)
			return dev_err_probe(&pdev->dev, ret,
					     "failed to add pinmux function\n");
	}

	ret = esp32s3_gpio_register(pdev, pctl);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register GPIO controller\n");
	platform_set_drvdata(pdev, pctl);
	return 0;
}

static const struct of_device_id esp32s3_pinctrl_of_match[] = {
	{ .compatible = "esp,esp32s3-pinctrl" },
	{ }
};

static struct platform_driver esp32s3_pinctrl_driver = {
	.probe = esp32s3_pinctrl_probe,
	.driver = {
		.name = "esp32s3-pinctrl",
		.of_match_table = esp32s3_pinctrl_of_match,
	},
};
builtin_platform_driver(esp32s3_pinctrl_driver);

MODULE_DESCRIPTION("Espressif ESP32-S3 pin control and GPIO driver");
MODULE_LICENSE("GPL");
