// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S3 I2S controller ASoC CPU DAI
 *
 * Register facts and the divider encoding are documented in
 * i2s-esp32s3.h (locked ESP-IDF v5.5.3 sources). PCM transfers use the
 * project GDMA dmaengine driver; the peripheral FIFO address (FIFO_WR /
 * FIFO_RD) is passed through dmaengine_slave_config.
 *
 * Current scope: master mode, I2S (Phillips) format, 2 channels, sample
 * widths 16/24/32, playback and capture through dmaengine_pcm. The
 * controller interrupt is not used; FIFO flow control is handled by the
 * DMA request mechanism. No device has exercised this driver yet.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "i2s-esp32s3.h"

#define I2S_ESP32S3_FIFO_DEPTH	64

struct i2s_esp32s3 {
	struct device *dev;
	void __iomem *regs;
	phys_addr_t phys_base;
	struct clk *gate;
	struct clk *source;
	unsigned long source_rate;
	unsigned long mclk_rate;
	struct clk_hw mclk_hw;
};

static inline void i2s_esp32s3_writel(struct i2s_esp32s3 *i2s,
				      unsigned int offset, u32 value)
{
	writel(value, i2s->regs + offset);
}

static inline u32 i2s_esp32s3_readl(struct i2s_esp32s3 *i2s,
				    unsigned int offset)
{
	return readl(i2s->regs + offset);
}

static void i2s_esp32s3_write_mask(struct i2s_esp32s3 *i2s,
				   unsigned int offset, u32 mask, u32 value)
{
	u32 reg = i2s_esp32s3_readl(i2s, offset);

	reg = (reg & ~mask) | (value & mask);
	i2s_esp32s3_writel(i2s, offset, reg);
}

/* Poll the self-clearing update bit; bounded per datasheet handshake */
static int i2s_esp32s3_update(struct i2s_esp32s3 *i2s, unsigned int conf)
{
	unsigned int attempts;

	i2s_esp32s3_writel(i2s, conf,
			   i2s_esp32s3_readl(i2s, conf) |
			   I2S_ESP32S3_CONF_UPDATE);
	for (attempts = 0; attempts < 1000; attempts++) {
		if (!(i2s_esp32s3_readl(i2s, conf) & I2S_ESP32S3_CONF_UPDATE))
			return 0;
		cpu_relax();
	}

	return -ETIMEDOUT;
}

static void i2s_esp32s3_conf1_configure(struct i2s_esp32s3 *i2s,
					unsigned int conf1,
					unsigned int bits_per_sample,
					unsigned int bck_div)
{
	u32 reg = 0;

	reg |= (bck_div - 1) << I2S_ESP32S3_CONF1_BCK_DIV_SHIFT;
	reg |= (bits_per_sample - 1) << I2S_ESP32S3_CONF1_BITS_MOD_SHIFT;
	reg |= I2S_ESP32S3_CONF1_MSB_SHIFT; /* Phillips standard */
	i2s_esp32s3_writel(i2s, conf1, reg);
}

static int i2s_esp32s3_configure_clkm(struct i2s_esp32s3 *i2s,
				      unsigned int conf, unsigned int div_conf,
				      const struct i2s_esp32s3_div *div)
{
	u32 reg;

	/* Official workaround order: small division first, then target */
	i2s_esp32s3_write_mask(i2s, conf, I2S_ESP32S3_CLKM_DIV_MASK, 2);
	i2s_esp32s3_writel(i2s, div_conf, 1U << I2S_ESP32S3_CLKM_DIV_Y_SHIFT);

	reg = (div->z) | (div->y << I2S_ESP32S3_CLKM_DIV_Y_SHIFT) |
	      (div->x << I2S_ESP32S3_CLKM_DIV_X_SHIFT) |
	      (div->yn1 << I2S_ESP32S3_CLKM_DIV_YN1_SHIFT);
	i2s_esp32s3_writel(i2s, div_conf, reg);
	i2s_esp32s3_write_mask(i2s, conf, I2S_ESP32S3_CLKM_DIV_MASK,
			       div->integer << I2S_ESP32S3_CLKM_DIV_SHIFT);

	/* Select PLL_160M as the module clock and enable it */
	i2s_esp32s3_write_mask(i2s, conf, I2S_ESP32S3_CLKM_SEL_MASK,
			       I2S_ESP32S3_CLK_SRC_PLL_160M <<
			       I2S_ESP32S3_CLKM_SEL_SHIFT);
	i2s_esp32s3_writel(i2s, conf,
			   i2s_esp32s3_readl(i2s, conf) |
			   I2S_ESP32S3_CLKM_ACTIVE);

	return 0;
}

/* The codec MCLK output rate, exposed as a clock provider for the
 * ES8311/ES7210 codecs (see the device tree "mclk" phandle).
 */
static unsigned long i2s_esp32s3_mclk_recalc(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct i2s_esp32s3 *i2s = container_of(hw, struct i2s_esp32s3, mclk_hw);

	return i2s->mclk_rate;
}

static const struct clk_ops i2s_esp32s3_mclk_ops = {
	.recalc_rate = i2s_esp32s3_mclk_recalc,
};

static int i2s_esp32s3_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct i2s_esp32s3 *i2s = snd_soc_dai_get_drvdata(dai);
	struct i2s_esp32s3_div div;
	unsigned int bits = params_physical_width(params);
	unsigned int frame_bits = params_channels(params) * bits;
	unsigned int bck_div;
	unsigned int conf, conf1, clkm_conf, clkm_div_conf;
	int ret;

	/* MCLK = 256x the sample rate; BCLK = MCLK / (channels * bits) */
	if (I2S_ESP32S3_MCLK_MULTIPLE % frame_bits)
		return -EINVAL;
	bck_div = I2S_ESP32S3_MCLK_MULTIPLE / frame_bits;
	if (bck_div < 2 || bck_div >
	    (I2S_ESP32S3_CONF1_BCK_DIV_MASK >> I2S_ESP32S3_CONF1_BCK_DIV_SHIFT) + 1)
		return -EINVAL;

	ret = i2s_esp32s3_calc_div(i2s->source_rate, params_rate(params),
				   I2S_ESP32S3_MCLK_MULTIPLE, &div);
	if (ret)
		return ret;

	i2s->mclk_rate = params_rate(params) * I2S_ESP32S3_MCLK_MULTIPLE;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		conf = I2S_ESP32S3_TX_CONF;
		conf1 = I2S_ESP32S3_TX_CONF1;
		clkm_conf = I2S_ESP32S3_TX_CLKM_CONF;
		clkm_div_conf = I2S_ESP32S3_TX_CLKM_DIV_CONF;
	} else {
		conf = I2S_ESP32S3_RX_CONF;
		conf1 = I2S_ESP32S3_RX_CONF1;
		clkm_conf = I2S_ESP32S3_RX_CLKM_CONF;
		clkm_div_conf = I2S_ESP32S3_RX_CLKM_DIV_CONF;
	}

	i2s_esp32s3_conf1_configure(i2s, conf1, bits, bck_div);
	i2s_esp32s3_configure_clkm(i2s, clkm_conf, clkm_div_conf, &div);

	/* MCLK_OUT follows the TX module clock (mclk_sel = 0) */
	i2s_esp32s3_write_mask(i2s, I2S_ESP32S3_RX_CLKM_CONF, BIT(30), 0);

	/* Little-endian data, master mode, fifo reset */
	i2s_esp32s3_write_mask(i2s, conf, I2S_ESP32S3_CONF_BIG_ENDIAN |
			       I2S_ESP32S3_CONF_SLAVE_MOD |
			       I2S_ESP32S3_CONF_MONO, 0);
	i2s_esp32s3_writel(i2s, conf,
			   i2s_esp32s3_readl(i2s, conf) |
			   I2S_ESP32S3_CONF_FIFO_RESET);

	return i2s_esp32s3_update(i2s, conf);
}

static int i2s_esp32s3_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	default:
		dev_err(dai->dev, "unsupported DAI format 0x%x\n", fmt);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_CBP_CFC:
		break;
	default:
		dev_err(dai->dev, "only I2S master mode is supported\n");
		return -EINVAL;
	}

	return 0;
}

static int i2s_esp32s3_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct i2s_esp32s3 *i2s = snd_soc_dai_get_drvdata(dai);
	bool playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	unsigned int conf = playback ? I2S_ESP32S3_TX_CONF :
				       I2S_ESP32S3_RX_CONF;
	u32 reg;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/* Clear FIFO and start the direction */
		i2s_esp32s3_writel(i2s, conf,
				   i2s_esp32s3_readl(i2s, conf) |
				   I2S_ESP32S3_CONF_FIFO_RESET);
		reg = i2s_esp32s3_readl(i2s, conf) | I2S_ESP32S3_CONF_START;
		i2s_esp32s3_writel(i2s, conf, reg);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		reg = i2s_esp32s3_readl(i2s, conf) & ~I2S_ESP32S3_CONF_START;
		i2s_esp32s3_writel(i2s, conf, reg);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dai_ops i2s_esp32s3_dai_ops = {
	.hw_params = i2s_esp32s3_hw_params,
	.set_fmt = i2s_esp32s3_set_fmt,
	.trigger = i2s_esp32s3_trigger,
};

static struct snd_soc_dai_driver i2s_esp32s3_dai = {
	.name = "esp32s3-i2s0",
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &i2s_esp32s3_dai_ops,
};

static int i2s_esp32s3_prepare_slave_config(struct snd_pcm_substream *substream,
					    struct snd_pcm_hw_params *params,
					    struct dma_slave_config *config)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct i2s_esp32s3 *i2s = snd_soc_dai_get_drvdata(snd_soc_rtd_to_cpu(rtd, 0));

	/* Per-direction peripheral FIFO address */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		config->dst_addr = i2s->phys_base + I2S_ESP32S3_FIFO_WR;
		config->dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		config->dst_maxburst = I2S_ESP32S3_FIFO_DEPTH;
	} else {
		config->src_addr = i2s->phys_base + I2S_ESP32S3_FIFO_RD;
		config->src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		config->src_maxburst = I2S_ESP32S3_FIFO_DEPTH;
	}

	return 0;
}

static const struct snd_dmaengine_pcm_config i2s_esp32s3_dmaengine_pcm = {
	.prepare_slave_config = i2s_esp32s3_prepare_slave_config,
};

static int i2s_esp32s3_component_probe(struct snd_soc_component *component)
{
	return 0;
}

static const struct snd_soc_component_driver i2s_esp32s3_component = {
	.name = "esp32s3-i2s0",
	.probe = i2s_esp32s3_component_probe,
	.legacy_dai_naming = 1,
};

static int i2s_esp32s3_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct i2s_esp32s3 *i2s;
	struct reset_control *reset;
	int ret;

	i2s = devm_kzalloc(dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;

	i2s->dev = dev;
	i2s->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(i2s->regs))
		return PTR_ERR(i2s->regs);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;
	i2s->phys_base = res->start;

	reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset),
				     "failed to get reset control\n");
	reset_control_reset(reset);

	i2s->gate = devm_clk_get(dev, "gate");
	if (IS_ERR(i2s->gate))
		return dev_err_probe(dev, PTR_ERR(i2s->gate),
				     "failed to get gate clock\n");
	ret = clk_prepare_enable(i2s->gate);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable gate clock\n");

	i2s->source = devm_clk_get(dev, "source");
	if (IS_ERR(i2s->source))
		return dev_err_probe(dev, PTR_ERR(i2s->source),
				     "failed to get source clock\n");
	i2s->source_rate = clk_get_rate(i2s->source);
	if (!i2s->source_rate)
		return dev_err_probe(dev, -EINVAL,
				     "invalid I2S source clock rate\n");

	dev_set_drvdata(dev, i2s);

	i2s->mclk_hw.init = &(struct clk_init_data){
		.name = "mclk",
		.ops = &i2s_esp32s3_mclk_ops,
	};
	ret = devm_clk_hw_register(dev, &i2s->mclk_hw);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register MCLK clock\n");
	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get,
					  &i2s->mclk_hw);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register MCLK provider\n");

	ret = devm_snd_soc_register_component(dev, &i2s_esp32s3_component,
					      &i2s_esp32s3_dai, 1);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register component\n");

	return devm_snd_dmaengine_pcm_register(dev,
					       &i2s_esp32s3_dmaengine_pcm,
					       SND_DMAENGINE_PCM_FLAG_NO_DT);
}

static const struct of_device_id i2s_esp32s3_of_match[] = {
	{ .compatible = "esp,esp32s3-i2s" },
	{ }
};
MODULE_DEVICE_TABLE(of, i2s_esp32s3_of_match);

static struct platform_driver i2s_esp32s3_driver = {
	.driver = {
		.name = "esp32s3-i2s",
		.of_match_table = i2s_esp32s3_of_match,
	},
	.probe = i2s_esp32s3_probe,
};
module_platform_driver(i2s_esp32s3_driver);

MODULE_DESCRIPTION("ESP32-S3 I2S ASoC CPU DAI driver");
MODULE_AUTHOR("ESP32-S3 Linux project");
MODULE_LICENSE("GPL");
