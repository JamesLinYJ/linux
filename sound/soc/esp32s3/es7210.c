// SPDX-License-Identifier: GPL-2.0-only
/*
 * Everest ES7210 4-ch ADC codec driver (ASoC)
 *
 * Register facts and the clock coefficient table come from the official
 * espressif/esp_codec_dev v1.3.6 component (see es7210.h and
 * docs/clean-room.md). The Waveshare V2 wires the codec to I2S0 (slave,
 * MCLK from the SoC) and I2C0 address 0x40 (AD1/AD0 = 00).
 *
 * Current scope: capture-only DAI in I2S (Phillips) format, 16/24/32
 * bit samples, sample rates from the official coefficient table. The
 * MCLK rate is taken from the codec's reference clock (the machine
 * driver supplies the I2S-generated MCLK via clock-names "mclk").
 * No device has exercised this driver yet.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/tlv.h>

#include "es7210.h"

struct es7210_priv {
	struct regmap *regmap;
	struct clk *mclk;
};

static const struct reg_sequence es7210_power_up_sequence[] = {
	{ ES7210_CLOCK_OFF_REG, 0x00 },
	{ ES7210_POWER_DOWN_REG, 0x00 },
	{ ES7210_ANALOG_REG, 0x43 },
	{ ES7210_MIC1_POWER_REG, 0x08 },
	{ ES7210_MIC2_POWER_REG, 0x08 },
	{ ES7210_MIC3_POWER_REG, 0x08 },
	{ ES7210_MIC4_POWER_REG, 0x08 },
	{ ES7210_ANALOG_REG, 0x43 },
	{ ES7210_RESET_REG, 0x71 },
	{ ES7210_RESET_REG, 0x41 },
};

static const struct reg_sequence es7210_power_down_sequence[] = {
	{ ES7210_MIC1_POWER_REG, 0xff },
	{ ES7210_MIC2_POWER_REG, 0xff },
	{ ES7210_MIC3_POWER_REG, 0xff },
	{ ES7210_MIC4_POWER_REG, 0xff },
	{ ES7210_ANALOG_REG, 0x00 },
	{ ES7210_RESET_REG, 0x00 },
};

static const struct regmap_config es7210_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x4c,
};

static int es7210_power_up(struct es7210_priv *es7210)
{
	/* Official es7210_start() sequence (clock_reg_value = 0) */
	return regmap_multi_reg_write(es7210->regmap,
				      es7210_power_up_sequence,
				      ARRAY_SIZE(es7210_power_up_sequence));
}

static int es7210_power_down(struct es7210_priv *es7210)
{
	/* Official es7210_stop() sequence */
	return regmap_multi_reg_write(es7210->regmap,
				      es7210_power_down_sequence,
				      ARRAY_SIZE(es7210_power_down_sequence));
}

static int es7210_configure_clock(struct es7210_priv *es7210,
				  const struct es7210_coeff *coeff)
{
	const struct reg_sequence sequence[] = {
		{
			ES7210_MAINCLK_REG,
			coeff->adc_div | (coeff->doubler << 6) |
			(coeff->dll << 7),
		},
		{ ES7210_OSR_REG, coeff->osr },
		{ ES7210_LRCK_DIVH_REG, coeff->lrck_h },
		{ ES7210_LRCK_DIVL_REG, coeff->lrck_l },
		{ ES7210_MASTER_CLK_REG, coeff->mclk_src },
	};

	return regmap_multi_reg_write(es7210->regmap, sequence,
				      ARRAY_SIZE(sequence));
}

static int es7210_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct es7210_priv *es7210 = snd_soc_component_get_drvdata(dai->component);
	const struct es7210_coeff *coeff;
	unsigned int bits = params_physical_width(params);
	unsigned long mclk;
	unsigned int sdp;
	int ret;

	mclk = clk_get_rate(es7210->mclk);
	coeff = es7210_coeff_find(mclk, params_rate(params));
	if (!coeff)
		return dev_err_probe(dai->dev, -EINVAL,
				     "unsupported MCLK %lu with rate %d\n",
				     mclk, params_rate(params));

	/* Official es7210_config_sample() register writes */
	ret = es7210_configure_clock(es7210, coeff);
	if (ret)
		return ret;

	/* SDP interface: I2S format plus the sample width */
	switch (bits) {
	case 16:
		sdp = ES7210_SDP_BITS_16;
		break;
	case 24:
		sdp = ES7210_SDP_BITS_24;
		break;
	case 32:
		sdp = ES7210_SDP_BITS_32;
		break;
	default:
		return -EINVAL;
	}
	return regmap_update_bits(es7210->regmap, ES7210_SDP_INTERFACE1_REG,
				  ES7210_SDP_BITS_MASK | ES7210_SDP_FMT_MASK,
				  sdp | ES7210_SDP_FMT_I2S);
}

static int es7210_set_sysclk(struct snd_soc_dai *dai, int clk_id,
			     unsigned int freq, int dir)
{
	struct es7210_priv *es7210 =
		snd_soc_component_get_drvdata(dai->component);

	if (clk_id || dir != SND_SOC_CLOCK_IN || !freq)
		return -EINVAL;

	return clk_set_rate(es7210->mclk, freq);
}

static int es7210_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S) {
		dev_err(dai->dev, "unsupported DAI format 0x%x\n", fmt);
		return -EINVAL;
	}
	if ((fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_CBC_CFC) {
		dev_err(dai->dev, "codec must be clock consumer\n");
		return -EINVAL;
	}

	return 0;
}

static int es7210_set_bias_level(struct snd_soc_component *component,
				 enum snd_soc_bias_level level)
{
	struct es7210_priv *es7210 =
		snd_soc_component_get_drvdata(component);
	struct snd_soc_dapm_context *dapm =
		snd_soc_component_to_dapm(component);
	int ret;

	switch (level) {
	case SND_SOC_BIAS_OFF:
		if (snd_soc_dapm_get_bias_level(dapm) == SND_SOC_BIAS_OFF)
			return 0;
		ret = es7210_power_down(es7210);
		clk_disable_unprepare(es7210->mclk);
		return ret;
	case SND_SOC_BIAS_STANDBY:
		if (snd_soc_dapm_get_bias_level(dapm) != SND_SOC_BIAS_OFF)
			return 0;
		ret = clk_prepare_enable(es7210->mclk);
		if (ret)
			return ret;
		ret = es7210_power_up(es7210);
		if (ret)
			clk_disable_unprepare(es7210->mclk);
		return ret;
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	default:
		return 0;
	}
}

static const struct snd_soc_dai_ops es7210_dai_ops = {
	.hw_params = es7210_hw_params,
	.set_sysclk = es7210_set_sysclk,
	.set_fmt = es7210_set_fmt,
};

static struct snd_soc_dai_driver es7210_dai = {
	.name = "es7210",
	.capture = {
		.channels_min = 1,
		.channels_max = 4,
		/* Rates represented in the official table at 256x MCLK. */
		.rates = SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_32000 |
			 SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
			 SNDRV_PCM_RATE_64000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &es7210_dai_ops,
};

static const struct snd_soc_component_driver es7210_component = {
	.set_bias_level = es7210_set_bias_level,
	.legacy_dai_naming = 1,
};

static int es7210_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct es7210_priv *es7210;

	es7210 = devm_kzalloc(dev, sizeof(*es7210), GFP_KERNEL);
	if (!es7210)
		return -ENOMEM;

	es7210->regmap = devm_regmap_init_i2c(client, &es7210_regmap_config);
	if (IS_ERR(es7210->regmap))
		return dev_err_probe(dev, PTR_ERR(es7210->regmap),
				     "failed to initialize regmap\n");

	es7210->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(es7210->mclk))
		return dev_err_probe(dev, PTR_ERR(es7210->mclk),
				     "failed to get MCLK\n");

	i2c_set_clientdata(client, es7210);

	return devm_snd_soc_register_component(dev, &es7210_component,
					       &es7210_dai, 1);
}

static const struct of_device_id es7210_of_match[] = {
	{ .compatible = "everest,es7210" },
	{ }
};
MODULE_DEVICE_TABLE(of, es7210_of_match);

static const struct i2c_device_id es7210_i2c_id[] = {
	{ "es7210", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, es7210_i2c_id);

static struct i2c_driver es7210_i2c_driver = {
	.driver = {
		.name = "es7210",
		.of_match_table = es7210_of_match,
	},
	.probe = es7210_probe,
	.id_table = es7210_i2c_id,
};
module_i2c_driver(es7210_i2c_driver);

MODULE_DESCRIPTION("Everest ES7210 ADC codec driver");
MODULE_AUTHOR("ESP32-S3 Linux project");
MODULE_LICENSE("GPL");
