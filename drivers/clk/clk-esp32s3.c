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
#define ESP32S3_SYS_PERIP_RST_EN0	0x20

struct esp32s3_syscon {
	void __iomem *base;
	/* Protects shared clock-gate and reset registers. */
	spinlock_t lock;
	struct clk_hw_onecell_data *clk_data;
	struct reset_controller_dev reset;
};

static const u8 esp32s3_peripheral_bits[] = {
	[ESP32S3_RST_I2C0] = 7,
	[ESP32S3_RST_I2C1] = 18,
	[ESP32S3_RST_SPI3] = 16,
};

static int esp32s3_reset_update(struct reset_controller_dev *rcdev,
				unsigned long id, bool assert)
{
	struct esp32s3_syscon *syscon =
		container_of(rcdev, struct esp32s3_syscon, reset);
	unsigned long flags;
	u32 value;

	if (id >= ARRAY_SIZE(esp32s3_peripheral_bits))
		return -EINVAL;

	spin_lock_irqsave(&syscon->lock, flags);
	value = readl_relaxed(syscon->base + ESP32S3_SYS_PERIP_RST_EN0);
	if (assert)
		value |= BIT(esp32s3_peripheral_bits[id]);
	else
		value &= ~BIT(esp32s3_peripheral_bits[id]);
	writel_relaxed(value, syscon->base + ESP32S3_SYS_PERIP_RST_EN0);
	readl_relaxed(syscon->base + ESP32S3_SYS_PERIP_RST_EN0);
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

	if (id >= ARRAY_SIZE(esp32s3_peripheral_bits))
		return -EINVAL;
	return !!(readl_relaxed(syscon->base + ESP32S3_SYS_PERIP_RST_EN0) &
		  BIT(esp32s3_peripheral_bits[id]));
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
	void __iomem *reg = syscon->base + ESP32S3_SYS_PERIP_CLK_EN0;
	u8 bit = esp32s3_peripheral_bits[id];

	if (id == ESP32S3_CLK_SPI3)
		return devm_clk_hw_register_gate_parent_hw(dev, name, pll80, 0,
							reg, bit, 0,
							&syscon->lock);
	return esp32s3_register_gate(dev, name, parent, reg, bit, &syscon->lock);
}

static int esp32s3_syscon_probe(struct platform_device *pdev)
{
	static const char * const names[] = { "i2c0", "i2c1", "spi3" };
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
