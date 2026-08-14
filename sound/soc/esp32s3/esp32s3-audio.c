// SPDX-License-Identifier: GPL-2.0-only
/*
 * Waveshare ESP32-S3-Touch-LCD-3.49 V2 ASoC machine driver
 *
 * Links the ESP32-S3 I2S0 CPU DAI with the ES8311 codec (playback,
 * mainline driver) and the ES7210 ADC (capture, fork driver), and
 * drives the NS4150B amplifier enable line on TCA9554 EXIO7.
 *
 * The I2S0 controller does not generate MCLK yet, so the sound card
 * registers with the codec links in place but the card is only fully
 * functional once the I2S driver gains MCLK output; see the I2S driver
 * TODO and PLAN.md M9.
 */

#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <sound/pcm.h>
#include <sound/soc.h>

struct esp32s3_audio_priv {
	struct gpio_desc *pa_gpio; /* NS4150B enable, TCA9554 EXIO7 */
};

SND_SOC_DAILINK_DEFS(playback,
	DAILINK_COMP_ARRAY(COMP_CPU("esp32s3-i2s0")),
	DAILINK_COMP_ARRAY(COMP_CODEC("es8311.0-0018", "es8311")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("esp32s3-i2s0")));

SND_SOC_DAILINK_DEFS(capture,
	DAILINK_COMP_ARRAY(COMP_CPU("esp32s3-i2s0")),
	DAILINK_COMP_ARRAY(COMP_CODEC("es7210.0-0040", "es7210")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("esp32s3-i2s0")));

static int esp32s3_audio_init(struct snd_soc_pcm_runtime *rtd)
{
	struct esp32s3_audio_priv *priv = snd_soc_card_get_drvdata(rtd->card);

	/* NS4150B mode/enable: EXIO7 high selects BTL mode per schematic */
	if (priv->pa_gpio)
		gpiod_set_value_cansleep(priv->pa_gpio, 1);

	return 0;
}

static struct snd_soc_dai_link esp32s3_audio_dai_links[] = {
	{
		.name = "playback",
		.stream_name = "Playback",
		.init = esp32s3_audio_init,
		SND_SOC_DAILINK_REG(playback),
	},
	{
		.name = "capture",
		.stream_name = "Capture",
		SND_SOC_DAILINK_REG(capture),
	},
};

static struct snd_soc_card esp32s3_audio_card = {
	.name = "esp32s3-audio",
	.owner = THIS_MODULE,
	.dai_link = esp32s3_audio_dai_links,
	.num_links = ARRAY_SIZE(esp32s3_audio_dai_links),
};

static int esp32s3_audio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32s3_audio_priv *priv;
	struct snd_soc_card *card = &esp32s3_audio_card;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* Optional NS4150B amplifier enable (TCA9554 EXIO7) */
	priv->pa_gpio = devm_gpiod_get_optional(dev, "pa", GPIOD_OUT_LOW);
	if (IS_ERR(priv->pa_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->pa_gpio),
				     "failed to get PA GPIO\n");

	card->dev = dev;
	snd_soc_card_set_drvdata(card, priv);

	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id esp32s3_audio_of_match[] = {
	{ .compatible = "waveshare,esp32s3-audio" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s3_audio_of_match);

static struct platform_driver esp32s3_audio_driver = {
	.driver = {
		.name = "esp32s3-audio",
		.of_match_table = esp32s3_audio_of_match,
	},
	.probe = esp32s3_audio_probe,
};
module_platform_driver(esp32s3_audio_driver);

MODULE_DESCRIPTION("Waveshare ESP32-S3-Touch-LCD-3.49 V2 ASoC machine driver");
MODULE_AUTHOR("ESP32-S3 Linux project");
MODULE_LICENSE("GPL");
