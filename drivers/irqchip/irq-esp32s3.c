// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif ESP32-S3 peripheral interrupt matrix
 *
 * The Xtensa PIC remains the root controller. This irqchip routes all
 * enabled peripheral sources to one level-triggered CPU0 parent interrupt
 * and demultiplexes them through the matrix status registers.
 */

#include <linux/bitmap.h>
#include <linux/bits.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>

#define ESP32S3_INTC_NR_SOURCES		99
#define ESP32S3_INTC_STATUS_WORDS	4
#define ESP32S3_INTC_CORE1_OFFSET	0x800
#define ESP32S3_INTC_STATUS_BASE	0x18c
#define ESP32S3_INTC_CLOCK_GATE		0x19c
#define ESP32S3_INTC_CLOCK_GATE_EN	BIT(0)
#define ESP32S3_INTC_ROUTE_MASK		GENMASK(4, 0)
#define ESP32S3_INTC_DISABLED_CPU_IRQ	6
#define ESP32S3_INTC_PARENT_CPU_IRQ	2
#define ESP32S3_INTC_DRAIN_LIMIT	4

struct esp32s3_intc {
	void __iomem *base;
	struct irq_domain *domain;
	DECLARE_BITMAP(enabled, ESP32S3_INTC_NR_SOURCES);
};

static void esp32s3_intc_route(struct esp32s3_intc *intc,
			       unsigned int source, unsigned int cpu_irq)
{
	void __iomem *route = intc->base + source * sizeof(u32);

	writel(cpu_irq & ESP32S3_INTC_ROUTE_MASK, route);
	readl(route);
}

static void esp32s3_intc_mask(struct irq_data *data)
{
	struct esp32s3_intc *intc = irq_data_get_irq_chip_data(data);

	esp32s3_intc_route(intc, data->hwirq, ESP32S3_INTC_DISABLED_CPU_IRQ);
	clear_bit(data->hwirq, intc->enabled);
}

static void esp32s3_intc_unmask(struct irq_data *data)
{
	struct esp32s3_intc *intc = irq_data_get_irq_chip_data(data);

	set_bit(data->hwirq, intc->enabled);
	esp32s3_intc_route(intc, data->hwirq, ESP32S3_INTC_PARENT_CPU_IRQ);
}

static int esp32s3_intc_set_type(struct irq_data *data, unsigned int type)
{
	if (type != IRQ_TYPE_LEVEL_HIGH)
		return -EINVAL;

	return 0;
}

static struct irq_chip esp32s3_intc_chip = {
	.name		= "esp32s3-intc",
	.irq_mask	= esp32s3_intc_mask,
	.irq_unmask	= esp32s3_intc_unmask,
	.irq_set_type	= esp32s3_intc_set_type,
};

static int esp32s3_intc_map(struct irq_domain *domain, unsigned int irq,
			    irq_hw_number_t hwirq)
{
	if (hwirq >= ESP32S3_INTC_NR_SOURCES)
		return -EINVAL;

	irq_set_chip_data(irq, domain->host_data);
	irq_set_chip_and_handler(irq, &esp32s3_intc_chip, handle_level_irq);
	irq_set_status_flags(irq, IRQ_LEVEL);
	irq_set_noprobe(irq);
	return 0;
}

static const struct irq_domain_ops esp32s3_intc_domain_ops = {
	.map	= esp32s3_intc_map,
	.xlate	= irq_domain_xlate_twocell,
};

static void esp32s3_intc_handle(struct irq_desc *desc)
{
	struct esp32s3_intc *intc = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	bool handled = false;
	unsigned int pass;

	chained_irq_enter(chip, desc);

	for (pass = 0; pass < ESP32S3_INTC_DRAIN_LIMIT; pass++) {
		bool pass_handled = false;
		unsigned int word;

		for (word = 0; word < ESP32S3_INTC_STATUS_WORDS; word++) {
			u32 pending;

			pending = readl(intc->base + ESP32S3_INTC_STATUS_BASE +
					word * sizeof(u32));
			if (word == ESP32S3_INTC_STATUS_WORDS - 1)
				pending &= GENMASK(2, 0);

			while (pending) {
				unsigned int bit = __ffs(pending);
				unsigned int source = word * 32 + bit;

				pending &= ~BIT(bit);
				if (!test_bit(source, intc->enabled))
					continue;
				pass_handled = true;
				handled = true;
				generic_handle_domain_irq(intc->domain, source);
			}
		}
		if (!pass_handled)
			break;
	}

	if (!handled)
		handle_bad_irq(desc);

	/* Order device acknowledgements before the parent is unmasked. */
	wmb();
	chained_irq_exit(chip, desc);
}

static void esp32s3_intc_disable_all(struct esp32s3_intc *intc)
{
	unsigned int source;

	writel(ESP32S3_INTC_CLOCK_GATE_EN,
	       intc->base + ESP32S3_INTC_CLOCK_GATE);
	writel(ESP32S3_INTC_CLOCK_GATE_EN,
	       intc->base + ESP32S3_INTC_CORE1_OFFSET +
	       ESP32S3_INTC_CLOCK_GATE);

	for (source = 0; source < ESP32S3_INTC_NR_SOURCES; source++) {
		writel(ESP32S3_INTC_DISABLED_CPU_IRQ,
		       intc->base + source * sizeof(u32));
		writel(ESP32S3_INTC_DISABLED_CPU_IRQ,
		       intc->base + ESP32S3_INTC_CORE1_OFFSET +
		       source * sizeof(u32));
	}
	readl(intc->base + (ESP32S3_INTC_NR_SOURCES - 1) * sizeof(u32));
	readl(intc->base + ESP32S3_INTC_CORE1_OFFSET +
	      (ESP32S3_INTC_NR_SOURCES - 1) * sizeof(u32));
	bitmap_zero(intc->enabled, ESP32S3_INTC_NR_SOURCES);
}

static int __init esp32s3_intc_of_init(struct device_node *node,
				       struct device_node *parent)
{
	struct esp32s3_intc *intc;
	struct irq_data *parent_data;
	unsigned int parent_irq;

	intc = kzalloc_obj(*intc);
	if (!intc)
		return -ENOMEM;

	intc->base = of_iomap(node, 0);
	if (!intc->base)
		goto err_free;

	parent_irq = irq_of_parse_and_map(node, 0);
	if (!parent_irq)
		goto err_unmap;
	parent_data = irq_get_irq_data(parent_irq);
	if (!parent_data ||
	    irqd_to_hwirq(parent_data) != ESP32S3_INTC_PARENT_CPU_IRQ)
		goto err_dispose;

	esp32s3_intc_disable_all(intc);
	intc->domain = irq_domain_create_linear(of_fwnode_handle(node),
						ESP32S3_INTC_NR_SOURCES,
						&esp32s3_intc_domain_ops,
						intc);
	if (!intc->domain)
		goto err_dispose;

	irq_set_chained_handler_and_data(parent_irq, esp32s3_intc_handle, intc);
	pr_info("ESP32-S3 interrupt matrix: 99 sources on CPU0 IRQ2\n");
	return 0;

err_dispose:
	irq_dispose_mapping(parent_irq);
err_unmap:
	iounmap(intc->base);
err_free:
	kfree(intc);
	return -EINVAL;
}

IRQCHIP_DECLARE(esp32s3_intc, "esp,esp32s3-intc",
		esp32s3_intc_of_init);
