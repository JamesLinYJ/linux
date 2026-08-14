// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif ESP32-S3 peripheral clock and reset controller
 */

#include <dt-bindings/clock/esp32s3.h>
#include <dt-bindings/reset/esp32s3.h>

#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/spinlock.h>

#define ESP32S3_SYS_PERIP_CLK_EN0	0x18
#define ESP32S3_SYS_PERIP_CLK_EN1	0x1c
#define ESP32S3_SYS_PERIP_RST_EN0	0x20
#define ESP32S3_SYS_PERIP_RST_EN1	0x24

struct esp32s3_peripheral_gate {
	u8 bank;
	u8 bit;
};

struct esp32s3_syscon {
	void __iomem *base;
	/* Protects shared clock-gate and reset registers. */
	spinlock_t lock;
	struct clk_hw_onecell_data *clk_data;
	struct reset_controller_dev reset;
};

static const struct esp32s3_peripheral_gate esp32s3_peripheral_gates[] = {
	[ESP32S3_RST_I2C0] = { .bit = 7 },
	[ESP32S3_RST_I2C1] = { .bit = 18 },
	[ESP32S3_RST_SPI3] = { .bit = 16 },
	[ESP32S3_RST_GDMA] = { .bank = 1, .bit = 6 },
	[ESP32S3_RST_LEDC] = { .bit = 11 },
	[ESP32S3_RST_SPI2] = { .bit = 6 },
	[ESP32S3_RST_I2S0] = { .bit = 4 },
};

static void __iomem *esp32s3_syscon_reg(struct esp32s3_syscon *syscon,
					unsigned int bank, bool reset)
{
	unsigned int offset;

	if (reset)
		offset = bank ? ESP32S3_SYS_PERIP_RST_EN1 :
				ESP32S3_SYS_PERIP_RST_EN0;
	else
		offset = bank ? ESP32S3_SYS_PERIP_CLK_EN1 :
				ESP32S3_SYS_PERIP_CLK_EN0;

	return syscon->base + offset;
}

static int esp32s3_reset_update(struct reset_controller_dev *rcdev,
				unsigned long id, bool assert)
{
	struct esp32s3_syscon *syscon =
		container_of(rcdev, struct esp32s3_syscon, reset);
	const struct esp32s3_peripheral_gate *gate;
	unsigned long flags;
	void __iomem *reg;
	u32 value;

	if (id >= ARRAY_SIZE(esp32s3_peripheral_gates))
		return -EINVAL;
	gate = &esp32s3_peripheral_gates[id];
	reg = esp32s3_syscon_reg(syscon, gate->bank, true);

	spin_lock_irqsave(&syscon->lock, flags);
	value = readl_relaxed(reg);
	if (assert)
		value |= BIT(gate->bit);
	else
		value &= ~BIT(gate->bit);
	writel_relaxed(value, reg);
	readl_relaxed(reg);
	spin_unlock_irqrestore(&syscon->lock, flags);
	return 0;
}

static int esp32s3_reset_assert(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	return esp32s3_reset_update(rcdev, id, true);
}

static int esp32s3_reset_deassert(struct reset_controller_dev *rcdev,
				  unsigned long id)
{
	return esp32s3_reset_update(rcdev, id, false);
}

static int esp32s3_reset_status(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	struct esp32s3_syscon *syscon =
		container_of(rcdev, struct esp32s3_syscon, reset);
	const struct esp32s3_peripheral_gate *gate;
	void __iomem *reg;

	if (id >= ARRAY_SIZE(esp32s3_peripheral_gates))
		return -EINVAL;
	gate = &esp32s3_peripheral_gates[id];
	reg = esp32s3_syscon_reg(syscon, gate->bank, true);
	return !!(readl_relaxed(reg) & BIT(gate->bit));
}

static const struct reset_control_ops esp32s3_reset_ops = {
	.assert = esp32s3_reset_assert,
	.deassert = esp32s3_reset_deassert,
	.status = esp32s3_reset_status,
};

static struct clk_hw *esp32s3_register_gate(struct device *dev,
					    const char *name,
					    const struct clk_parent_data *parent,
					    void __iomem *reg, u8 bit,
					    spinlock_t *lock)
{
	return devm_clk_hw_register_gate_parent_data(dev, name, parent, 0, reg,
						     bit, 0, lock);
}

static struct clk_hw *
esp32s3_register_peripheral_gate(struct device *dev,
				 struct esp32s3_syscon *syscon,
				 const char *name, unsigned int id,
				 struct clk_hw *pll80,
				 const struct clk_parent_data *parent)
{
	const struct esp32s3_peripheral_gate *gate =
		&esp32s3_peripheral_gates[id];
	void __iomem *reg = esp32s3_syscon_reg(syscon, gate->bank, false);

	if (id == ESP32S3_CLK_SPI3 || id == ESP32S3_CLK_SPI2)
		return devm_clk_hw_register_gate_parent_hw(dev, name, pll80, 0,
							reg, gate->bit, 0,
							&syscon->lock);
	return esp32s3_register_gate(dev, name, parent, reg, gate->bit,
				     &syscon->lock);
}

static int esp32s3_syscon_probe(struct platform_device *pdev)
{
	static const char * const names[] = {
		"i2c0", "i2c1", "spi3", "gdma", "ledc", "spi2", "i2s0"
	};
	struct device *dev = &pdev->dev;
	struct esp32s3_syscon *syscon;
	struct clk_parent_data parent = { .index = 0 };
	struct clk_hw *pll80;
	struct clk_hw *hw;
	size_t clk_data_size;
	unsigned int i;
	int ret;

	syscon = devm_kzalloc(dev, sizeof(*syscon), GFP_KERNEL);
	if (!syscon)
		return -ENOMEM;
	syscon->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(syscon->base))
		return PTR_ERR(syscon->base);
	spin_lock_init(&syscon->lock);

	clk_data_size = struct_size(syscon->clk_data, hws, ESP32S3_CLK_NUM);
	syscon->clk_data = devm_kzalloc(dev, clk_data_size, GFP_KERNEL);
	if (!syscon->clk_data)
		return -ENOMEM;
	syscon->clk_data->num = ESP32S3_CLK_NUM;
	pll80 = devm_clk_hw_register_fixed_factor_index(dev, "pll80", 0, 0,
							2, 1);
	if (IS_ERR(pll80))
		return dev_err_probe(dev, PTR_ERR(pll80),
				     "failed to register PLL80 clock\n");

	for (i = 0; i < ESP32S3_CLK_NUM; i++) {
		hw = esp32s3_register_peripheral_gate(dev, syscon, names[i], i,
						      pll80, &parent);
		if (IS_ERR(hw))
			return dev_err_probe(dev,
				PTR_ERR(hw),
				"failed to register clock %s\n", names[i]);
		syscon->clk_data->hws[i] = hw;
	}

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get,
					  syscon->clk_data);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register clock provider\n");

	syscon->reset.owner = THIS_MODULE;
	syscon->reset.ops = &esp32s3_reset_ops;
	syscon->reset.of_node = dev->of_node;
	syscon->reset.nr_resets = ESP32S3_RST_NUM;
	ret = devm_reset_controller_register(dev, &syscon->reset);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register reset controller\n");

	platform_set_drvdata(pdev, syscon);
	return 0;
}

static const struct of_device_id esp32s3_syscon_of_match[] = {
	{ .compatible = "esp,esp32s3-system-controller" },
	{ }
};

static struct platform_driver esp32s3_syscon_driver = {
	.probe = esp32s3_syscon_probe,
	.driver = {
		.name = "esp32s3-system-controller",
		.of_match_table = esp32s3_syscon_of_match,
	},
};
builtin_platform_driver(esp32s3_syscon_driver);

MODULE_DESCRIPTION("Espressif ESP32-S3 clock and reset controller");
MODULE_LICENSE("GPL");
