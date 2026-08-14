// SPDX-License-Identifier: GPL-2.0-only
/* Espressif ESP32-S3 general-purpose DMA controller driver */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_dma.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/scatterlist.h>

#include <dt-bindings/dma/esp32s3-gdma.h>

#include "esp32s3-gdma.h"
#include "virt-dma.h"

#define ESP32S3_GDMA_PAIRS		5
#define ESP32S3_GDMA_CHANNELS		(ESP32S3_GDMA_PAIRS * 2)
#define ESP32S3_GDMA_PAIR_STRIDE		0xc0
#define ESP32S3_GDMA_TX_OFFSET		0x60

#define ESP32S3_GDMA_CONF0		0x00
#define ESP32S3_GDMA_CONF1		0x04
#define ESP32S3_GDMA_INT_ST		0x0c
#define ESP32S3_GDMA_INT_ENA		0x10
#define ESP32S3_GDMA_INT_CLR		0x14
#define ESP32S3_GDMA_LINK		0x20
#define ESP32S3_GDMA_PERI_SEL		0x48
#define ESP32S3_GDMA_MISC_CONF		0x3c8

#define ESP32S3_GDMA_CONF0_RST		BIT(0)
#define ESP32S3_GDMA_TX_AUTO_WRBACK	BIT(2)
#define ESP32S3_GDMA_TX_EOF_MODE		BIT(3)
#define ESP32S3_GDMA_CONF1_CHECK_OWNER	BIT(12)
#define ESP32S3_GDMA_MISC_CLK_EN		BIT(4)
#define ESP32S3_GDMA_PERI_SEL_MASK	GENMASK(5, 0)

#define ESP32S3_GDMA_LINK_ADDR		GENMASK(19, 0)
#define ESP32S3_GDMA_RX_LINK_AUTO_RET	BIT(20)
#define ESP32S3_GDMA_RX_LINK_STOP	BIT(21)
#define ESP32S3_GDMA_RX_LINK_START	BIT(22)
#define ESP32S3_GDMA_TX_LINK_STOP	BIT(20)
#define ESP32S3_GDMA_TX_LINK_START	BIT(21)

#define ESP32S3_GDMA_RX_DONE		BIT(1)
#define ESP32S3_GDMA_RX_ERRORS		(BIT(2) | BIT(3) | BIT(4) | \
					 BIT(6) | BIT(7) | BIT(8) | BIT(9))
#define ESP32S3_GDMA_TX_DONE		BIT(3)
#define ESP32S3_GDMA_TX_ERRORS		(BIT(2) | BIT(4) | BIT(5) | \
					 BIT(6) | BIT(7))

struct esp32s3_gdma_hw_desc {
	__le32 flags;
	__le32 buffer;
	__le32 next;
};

static_assert(sizeof(struct esp32s3_gdma_hw_desc) == 12);

struct esp32s3_gdma;

struct esp32s3_gdma_desc {
	struct virt_dma_desc vd;
	struct esp32s3_gdma *gdma;
	struct esp32s3_gdma_hw_desc *hw;
	dma_addr_t hw_dma;
	size_t hw_size;
};

struct esp32s3_gdma_chan {
	struct virt_dma_chan vc;
	struct esp32s3_gdma *gdma;
	struct esp32s3_gdma_desc *active;
	dma_cookie_t error_cookie;
	unsigned int pair;
	int trigger;
	int irq;
	bool tx;
};

struct esp32s3_gdma {
	struct dma_device dma_dev;
	struct device *dev;
	void __iomem *base;
	struct resource desc_pool;
	struct esp32s3_gdma_chan chans[ESP32S3_GDMA_CHANNELS];
};

static inline struct esp32s3_gdma_chan *
to_esp32s3_gdma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct esp32s3_gdma_chan, vc.chan);
}

static inline struct esp32s3_gdma_desc *
to_esp32s3_gdma_desc(struct virt_dma_desc *vd)
{
	return container_of(vd, struct esp32s3_gdma_desc, vd);
}

static void __iomem *esp32s3_gdma_chan_base(struct esp32s3_gdma_chan *chan)
{
	return chan->gdma->base + chan->pair * ESP32S3_GDMA_PAIR_STRIDE +
		(chan->tx ? ESP32S3_GDMA_TX_OFFSET : 0);
}

static u32 esp32s3_gdma_irq_mask(struct esp32s3_gdma_chan *chan)
{
	return chan->tx ? ESP32S3_GDMA_TX_DONE | ESP32S3_GDMA_TX_ERRORS :
		ESP32S3_GDMA_RX_DONE | ESP32S3_GDMA_RX_ERRORS;
}

static void esp32s3_gdma_stop(struct esp32s3_gdma_chan *chan)
{
	void __iomem *base = esp32s3_gdma_chan_base(chan);
	u32 stop = chan->tx ? ESP32S3_GDMA_TX_LINK_STOP :
		ESP32S3_GDMA_RX_LINK_STOP;

	writel(0, base + ESP32S3_GDMA_INT_ENA);
	writel(stop, base + ESP32S3_GDMA_LINK);
	writel(esp32s3_gdma_irq_mask(chan), base + ESP32S3_GDMA_INT_CLR);
	writel(ESP32S3_GDMA_CONF0_RST, base + ESP32S3_GDMA_CONF0);
	writel(0, base + ESP32S3_GDMA_CONF0);
}

static void esp32s3_gdma_desc_free(struct virt_dma_desc *vd)
{
	struct esp32s3_gdma_desc *desc = to_esp32s3_gdma_desc(vd);

	dma_free_coherent(desc->gdma->dev, desc->hw_size, desc->hw,
			  desc->hw_dma);
	kfree(desc);
}

static struct dma_async_tx_descriptor *
esp32s3_gdma_prep_slave_sg(struct dma_chan *dma_chan,
			   struct scatterlist *sgl, unsigned int sg_len,
			   enum dma_transfer_direction direction,
			   unsigned long flags, void *context)
{
	struct esp32s3_gdma_chan *chan = to_esp32s3_gdma_chan(dma_chan);
	struct esp32s3_gdma *gdma = chan->gdma;
	struct esp32s3_gdma_desc *desc;
	struct scatterlist *sg;
	unsigned int count = 0;
	unsigned int index = 0;
	unsigned int i;
	size_t hw_size;

	if (!sgl || !sg_len || chan->trigger < 0)
		return NULL;
	if ((chan->tx && direction != DMA_MEM_TO_DEV) ||
	    (!chan->tx && direction != DMA_DEV_TO_MEM))
		return NULL;

	for_each_sg(sgl, sg, sg_len, i) {
		unsigned int chunks;

		if (!esp32s3_gdma_data_addr_valid(sg_dma_address(sg),
						  sg_dma_len(sg)))
			return NULL;
		chunks = DIV_ROUND_UP(sg_dma_len(sg),
				      ESP32S3_GDMA_DESC_MAX_LEN);
		if (check_add_overflow(count, chunks, &count))
			return NULL;
	}

	if (check_mul_overflow((size_t)count, sizeof(*desc->hw), &hw_size))
		return NULL;

	desc = kzalloc_obj(*desc, GFP_NOWAIT);
	if (!desc)
		return NULL;
	desc->gdma = gdma;
	desc->hw_size = hw_size;
	desc->hw = dma_alloc_coherent(gdma->dev, hw_size,
				      &desc->hw_dma, GFP_NOWAIT);
	if (!desc->hw)
		goto err_free_desc;
	if (!esp32s3_gdma_desc_addr_valid(desc->hw_dma, hw_size,
					  gdma->desc_pool.start,
					  gdma->desc_pool.end))
		goto err_free_hw;

	for_each_sg(sgl, sg, sg_len, i) {
		dma_addr_t address = sg_dma_address(sg);
		size_t remaining = sg_dma_len(sg);

		while (remaining) {
			struct esp32s3_gdma_hw_desc *hw = &desc->hw[index];
			size_t length = min_t(size_t, remaining,
					      ESP32S3_GDMA_DESC_MAX_LEN);
			u32 value = esp32s3_gdma_desc_flags(length, chan->tx,
							 index == count - 1);
			hw->flags = cpu_to_le32(value);
			hw->buffer = cpu_to_le32(lower_32_bits(address));
			hw->next = cpu_to_le32(index == count - 1 ? 0 :
				lower_32_bits(desc->hw_dma +
					      (index + 1) * sizeof(*hw)));
			address += length;
			remaining -= length;
			index++;
		}
	}

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);

err_free_hw:
	dma_free_coherent(gdma->dev, hw_size, desc->hw, desc->hw_dma);
err_free_desc:
	kfree(desc);
	return NULL;
}

static void esp32s3_gdma_start(struct esp32s3_gdma_chan *chan)
{
	struct virt_dma_desc *vd = vchan_next_desc(&chan->vc);
	void __iomem *base;
	u32 conf0;
	u32 link;

	lockdep_assert_held(&chan->vc.lock);
	if (!vd || chan->active)
		return;

	list_del(&vd->node);
	chan->active = to_esp32s3_gdma_desc(vd);
	base = esp32s3_gdma_chan_base(chan);

	writel(ESP32S3_GDMA_CONF0_RST, base + ESP32S3_GDMA_CONF0);
	conf0 = chan->tx ? ESP32S3_GDMA_TX_AUTO_WRBACK |
		ESP32S3_GDMA_TX_EOF_MODE : 0;
	writel(conf0, base + ESP32S3_GDMA_CONF0);
	writel(ESP32S3_GDMA_CONF1_CHECK_OWNER,
	       base + ESP32S3_GDMA_CONF1);
	writel(FIELD_PREP(ESP32S3_GDMA_PERI_SEL_MASK, chan->trigger),
	       base + ESP32S3_GDMA_PERI_SEL);
	writel(esp32s3_gdma_irq_mask(chan), base + ESP32S3_GDMA_INT_CLR);
	writel(esp32s3_gdma_irq_mask(chan), base + ESP32S3_GDMA_INT_ENA);

	dma_wmb();
	link = FIELD_PREP(ESP32S3_GDMA_LINK_ADDR, chan->active->hw_dma);
	link |= chan->tx ? ESP32S3_GDMA_TX_LINK_START :
		ESP32S3_GDMA_RX_LINK_AUTO_RET | ESP32S3_GDMA_RX_LINK_START;
	writel(link, base + ESP32S3_GDMA_LINK);
}

static irqreturn_t esp32s3_gdma_irq(int irq, void *data)
{
	struct esp32s3_gdma_chan *chan = data;
	void __iomem *base = esp32s3_gdma_chan_base(chan);
	u32 status = readl(base + ESP32S3_GDMA_INT_ST);
	u32 errors = chan->tx ? ESP32S3_GDMA_TX_ERRORS :
		ESP32S3_GDMA_RX_ERRORS;
	u32 done = chan->tx ? ESP32S3_GDMA_TX_DONE : ESP32S3_GDMA_RX_DONE;
	unsigned long flags;

	if (!(status & esp32s3_gdma_irq_mask(chan)))
		return IRQ_NONE;

	writel(status, base + ESP32S3_GDMA_INT_CLR);
	spin_lock_irqsave(&chan->vc.lock, flags);
	if (!chan->active) {
		writel(0, base + ESP32S3_GDMA_INT_ENA);
		spin_unlock_irqrestore(&chan->vc.lock, flags);
		return IRQ_HANDLED;
	}

	if (status & errors) {
		chan->active->vd.tx_result.result = chan->tx ?
			DMA_TRANS_READ_FAILED : DMA_TRANS_WRITE_FAILED;
		chan->error_cookie = chan->active->vd.tx.cookie;
		esp32s3_gdma_stop(chan);
	}

	if (status & (done | errors)) {
		struct esp32s3_gdma_desc *desc = chan->active;

		writel(0, base + ESP32S3_GDMA_INT_ENA);
		chan->active = NULL;
		vchan_cookie_complete(&desc->vd);
		esp32s3_gdma_start(chan);
	}
	spin_unlock_irqrestore(&chan->vc.lock, flags);

	return IRQ_HANDLED;
}

static void esp32s3_gdma_issue_pending(struct dma_chan *dma_chan)
{
	struct esp32s3_gdma_chan *chan = to_esp32s3_gdma_chan(dma_chan);
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);
	if (vchan_issue_pending(&chan->vc) && !chan->active)
		esp32s3_gdma_start(chan);
	spin_unlock_irqrestore(&chan->vc.lock, flags);
}

static int esp32s3_gdma_terminate_all(struct dma_chan *dma_chan)
{
	struct esp32s3_gdma_chan *chan = to_esp32s3_gdma_chan(dma_chan);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&chan->vc.lock, flags);
	esp32s3_gdma_stop(chan);
	if (chan->active) {
		vchan_terminate_vdesc(&chan->active->vd);
		chan->active = NULL;
	}
	chan->error_cookie = 0;
	vchan_get_all_descriptors(&chan->vc, &head);
	spin_unlock_irqrestore(&chan->vc.lock, flags);

	vchan_dma_desc_free_list(&chan->vc, &head);
	return 0;
}

static void esp32s3_gdma_synchronize(struct dma_chan *dma_chan)
{
	struct esp32s3_gdma_chan *chan = to_esp32s3_gdma_chan(dma_chan);

	vchan_synchronize(&chan->vc);
}

static int esp32s3_gdma_config(struct dma_chan *dma_chan,
			       struct dma_slave_config *config)
{
	struct esp32s3_gdma_chan *chan = to_esp32s3_gdma_chan(dma_chan);
	enum dma_slave_buswidth width;

	if ((chan->tx && config->direction != DMA_MEM_TO_DEV) ||
	    (!chan->tx && config->direction != DMA_DEV_TO_MEM))
		return -EINVAL;

	width = chan->tx ? config->dst_addr_width : config->src_addr_width;
	if (width != DMA_SLAVE_BUSWIDTH_UNDEFINED &&
	    width != DMA_SLAVE_BUSWIDTH_1_BYTE &&
	    width != DMA_SLAVE_BUSWIDTH_2_BYTES &&
	    width != DMA_SLAVE_BUSWIDTH_4_BYTES)
		return -EINVAL;

	return 0;
}

static enum dma_status esp32s3_gdma_tx_status(struct dma_chan *dma_chan,
					      dma_cookie_t cookie,
					      struct dma_tx_state *state)
{
	struct esp32s3_gdma_chan *chan = to_esp32s3_gdma_chan(dma_chan);
	unsigned long flags;
	enum dma_status status;

	spin_lock_irqsave(&chan->vc.lock, flags);
	status = cookie == chan->error_cookie ? DMA_ERROR :
		dma_cookie_status(dma_chan, cookie, state);
	spin_unlock_irqrestore(&chan->vc.lock, flags);

	return status;
}

static int esp32s3_gdma_alloc_chan_resources(struct dma_chan *dma_chan)
{
	struct esp32s3_gdma_chan *chan = to_esp32s3_gdma_chan(dma_chan);

	return chan->trigger < 0 ? -EINVAL : 0;
}

static void esp32s3_gdma_free_chan_resources(struct dma_chan *dma_chan)
{
	struct esp32s3_gdma_chan *chan = to_esp32s3_gdma_chan(dma_chan);

	esp32s3_gdma_terminate_all(dma_chan);
	vchan_free_chan_resources(&chan->vc);
	chan->trigger = -1;
}

static struct dma_chan *
esp32s3_gdma_of_xlate(struct of_phandle_args *dma_spec, struct of_dma *ofdma)
{
	struct esp32s3_gdma *gdma = ofdma->of_dma_data;
	unsigned int trigger;
	unsigned int direction;
	unsigned int first;
	unsigned int i;

	if (dma_spec->args_count != 2)
		return NULL;
	trigger = dma_spec->args[0];
	direction = dma_spec->args[1];
	if (trigger > ESP32S3_GDMA_TRIG_RMT ||
	    direction > ESP32S3_GDMA_DIR_TX)
		return NULL;

	first = direction == ESP32S3_GDMA_DIR_TX ? ESP32S3_GDMA_PAIRS : 0;
	for (i = first; i < first + ESP32S3_GDMA_PAIRS; i++) {
		struct esp32s3_gdma_chan *chan = &gdma->chans[i];
		struct dma_chan *dma_chan;

		chan->trigger = trigger;
		dma_chan = dma_get_slave_channel(&chan->vc.chan);
		if (dma_chan)
			return dma_chan;
		chan->trigger = -1;
	}

	return NULL;
}

static void esp32s3_gdma_release_reserved_mem(void *data)
{
	of_reserved_mem_device_release(data);
}

static void esp32s3_gdma_kill_tasklets(void *data)
{
	struct esp32s3_gdma *gdma = data;
	unsigned int i;

	for (i = 0; i < ESP32S3_GDMA_CHANNELS; i++)
		tasklet_kill(&gdma->chans[i].vc.task);
}

static int esp32s3_gdma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32s3_gdma *gdma;
	struct dma_device *dma_dev;
	struct reset_control *rst;
	struct clk *clk;
	unsigned int i;
	int ret;

	gdma = devm_kzalloc(dev, sizeof(*gdma), GFP_KERNEL);
	if (!gdma)
		return -ENOMEM;
	gdma->dev = dev;
	gdma->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gdma->base))
		return PTR_ERR(gdma->base);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DMA mask\n");
	ret = of_reserved_mem_region_to_resource(dev->of_node, 0,
						 &gdma->desc_pool);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to find descriptor SRAM pool\n");
	if ((gdma->desc_pool.start & ~ESP32S3_GDMA_DESC_ADDR_MASK) !=
	    ESP32S3_GDMA_DESC_ADDR_PREFIX ||
	    (gdma->desc_pool.end & ~ESP32S3_GDMA_DESC_ADDR_MASK) !=
	    ESP32S3_GDMA_DESC_ADDR_PREFIX)
		return dev_err_probe(dev, -EINVAL,
				     "descriptor pool is outside internal SRAM\n");
	ret = of_reserved_mem_device_init(dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to assign descriptor SRAM pool\n");
	ret = devm_add_action_or_reset(dev, esp32s3_gdma_release_reserved_mem,
				       dev);
	if (ret)
		return ret;

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "failed to enable clock\n");
	rst = devm_reset_control_get_exclusive_deasserted(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst),
				     "failed to deassert reset\n");

	writel(ESP32S3_GDMA_MISC_CLK_EN, gdma->base + ESP32S3_GDMA_MISC_CONF);
	dma_dev = &gdma->dma_dev;
	dma_cap_set(DMA_SLAVE, dma_dev->cap_mask);
	dma_dev->dev = dev;
	dma_dev->device_alloc_chan_resources =
		esp32s3_gdma_alloc_chan_resources;
	dma_dev->device_free_chan_resources =
		esp32s3_gdma_free_chan_resources;
	dma_dev->device_prep_slave_sg = esp32s3_gdma_prep_slave_sg;
	dma_dev->device_config = esp32s3_gdma_config;
	dma_dev->device_terminate_all = esp32s3_gdma_terminate_all;
	dma_dev->device_synchronize = esp32s3_gdma_synchronize;
	dma_dev->device_tx_status = esp32s3_gdma_tx_status;
	dma_dev->device_issue_pending = esp32s3_gdma_issue_pending;
	dma_dev->src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
		BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
		BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	dma_dev->dst_addr_widths = dma_dev->src_addr_widths;
	dma_dev->directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);
	dma_dev->residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;
	INIT_LIST_HEAD(&dma_dev->channels);

	for (i = 0; i < ESP32S3_GDMA_CHANNELS; i++) {
		struct esp32s3_gdma_chan *chan = &gdma->chans[i];
		const char *irq_name;

		chan->gdma = gdma;
		chan->pair = i % ESP32S3_GDMA_PAIRS;
		chan->tx = i >= ESP32S3_GDMA_PAIRS;
		chan->trigger = -1;
		vchan_init(&chan->vc, dma_dev);
		chan->vc.desc_free = esp32s3_gdma_desc_free;
		esp32s3_gdma_stop(chan);
		irq_name = devm_kasprintf(dev, GFP_KERNEL, "%s%u",
					  chan->tx ? "tx" : "rx", chan->pair);
		if (!irq_name)
			return -ENOMEM;
		chan->irq = platform_get_irq_byname(pdev, irq_name);
		if (chan->irq < 0)
			return chan->irq;
		ret = devm_request_irq(dev, chan->irq, esp32s3_gdma_irq, 0,
				       irq_name, chan);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to request %s IRQ\n", irq_name);
	}

	platform_set_drvdata(pdev, gdma);
	ret = dmaenginem_async_device_register(dma_dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register DMA engine\n");
	ret = devm_add_action_or_reset(dev, esp32s3_gdma_kill_tasklets, gdma);
	if (ret)
		return ret;
	ret = devm_of_dma_controller_register(dev, dev->of_node,
					      esp32s3_gdma_of_xlate, gdma);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register OF DMA controller\n");

	return 0;
}

static const struct of_device_id esp32s3_gdma_of_match[] = {
	{ .compatible = "esp,esp32s3-gdma" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s3_gdma_of_match);

static struct platform_driver esp32s3_gdma_driver = {
	.probe = esp32s3_gdma_probe,
	.driver = {
		.name = "esp32s3-gdma",
		.of_match_table = esp32s3_gdma_of_match,
	},
};
module_platform_driver(esp32s3_gdma_driver);

MODULE_DESCRIPTION("Espressif ESP32-S3 general-purpose DMA controller");
MODULE_LICENSE("GPL");
