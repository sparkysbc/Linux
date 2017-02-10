/*
 * ALSA ASoC Machine Driver for Allo Piano DAC
 *
 * Author:      Baswaraj K <jaikumar@cem-solutions.net>
 *	       Copyright 2016
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <sound/soc.h>
#include <mach/hardware.h>
#include <linux/time.h>
#include <linux/delay.h>
#include "sndrv-owl.h"
#include "../codecs/pcm512x.h"

#define GPIO_NAME_PCM512X_DAC_MUTE	"dac_mute"
#define PCM512X_DAC_MUTE	OWL_GPIO_PORTB(18)

#define GPIO_NAME_PCM512X_HEAD_SHUT	"dac_shut"
#define PCM512X_HEAD_SHUT	OWL_GPIO_PORTB(13)

#define GPIO_NAME_PCM512X_AMP_SHUT	"amp_shut"
#define PCM512X_AMP_SHUT	OWL_GPIO_PORTB(31)

#define GPIO_NAME_PCM512X_AMP_MUTE	"amp_mute"
#define PCM512X_AMP_MUTE	OWL_GPIO_PORTB(17)

static bool digital_gain_0db_limit = true;
module_param(digital_gain_0db_limit, bool, 0);
MODULE_PARM_DESC(digital_gain_0db_limit, "Set the max volume");

static bool glb_mclk = 1;
module_param(glb_mclk, bool, 0);
MODULE_PARM_DESC(glb_mclk, "Used to disable the MCLK clock");

static int snd_allo_piano_dac_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int dac = 0, val = 0;

	if (!glb_mclk) {
		snd_soc_write(rtd->codec, PCM512x_PLL_EN, 0x01);
		snd_soc_write(rtd->codec, PCM512x_PLL_REF, (1 << 4));

		dev_info(rtd->codec->dev,
				"Force Set BCLK as input clock & Enable PLL\n");
	} else {
		val = snd_soc_read(rtd->codec, PCM512x_RATE_DET_4);
		if(val & 0x40) {
			snd_soc_write(rtd->codec, PCM512x_PLL_EN, 0x01);
			snd_soc_write(rtd->codec, PCM512x_PLL_REF, (1 << 4));
			dev_info(rtd->codec->dev,
					"Setting BCLK as input clock & Enable PLL\n");
		} else {
			snd_soc_write(rtd->codec, PCM512x_PLL_EN, 0x00);
			snd_soc_write(rtd->codec, PCM512x_PLL_REF, 0x00);
			dev_info(rtd->codec->dev,
					"Setting SCLK as input clock & disabled PLL\n");
		}
	}

	return 0;
}

static struct snd_soc_ops allo_pianodac_ops = {
	.hw_params = snd_allo_piano_dac_hw_params,
};

/*
 * Logic for a link as connected on a atm7059 board.
 */
static int snd_allo_piano_dac_init(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;

	if (digital_gain_0db_limit) {
		struct snd_soc_codec *codec = rtd->codec;

		ret = snd_soc_limit_volume(codec, "Digital Playback Volume",
				207);
		if (ret < 0)
			dev_warn(codec->dev,
				"Failed to set volume limit: %d\n", ret);
	}

	return 0;
}

static struct snd_soc_dai_link allo_piano_dac_link_dai = {
	.name = "PianoDAC",
	.stream_name = "PianoDAC",
	.cpu_dai_name = "owl-audio-i2s",
	.codec_dai_name = "pcm512x-hifi",
	.init = snd_allo_piano_dac_init,
	.platform_name = "atm7059-pcm-audio",
	.codec_name = "pcm512x.2-004c",
	.ops = &allo_pianodac_ops,
};
static struct snd_soc_card snd_soc_allo_pianodac = {
	.name = "PianoDAC",
	.owner = THIS_MODULE,
	.dai_link = &allo_piano_dac_link_dai,
	.num_links = 1,
};

static struct platform_device *snd_allo_piano_link_device_pcm512x;

static int __init snd_allo_piano_link_init(void)
{
	int ret = 0;

	snd_allo_piano_link_device_pcm512x =
				platform_device_alloc("soc-audio", 1);
	if (!snd_allo_piano_link_device_pcm512x) {
		snd_err("ASoC: Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(snd_allo_piano_link_device_pcm512x,
			&snd_soc_allo_pianodac);
	ret = platform_device_add(snd_allo_piano_link_device_pcm512x);
	if (ret) {
		snd_err("ASoC: Platform device allocation failed\n");
		goto platform_device_add_failed;
	}

	/** Configure Mute and Shutdown for DAC and Amplifier board **/
	act_setl(act_readl(MFP_CTL1) | (0x1 << 22), MFP_CTL1); /* GPIOB pins */

	/** Configure Mute pin **/
	ret = gpio_request(PCM512X_DAC_MUTE, GPIO_NAME_PCM512X_DAC_MUTE);
	if (ret < 0) {
		pr_err("DAC mute control %d request failed!\n",
				PCM512X_DAC_MUTE);
		goto platform_device_add_failed;
	}
	gpio_direction_output(PCM512X_DAC_MUTE, 1);

	/** configure Headphone Shutdown pin *****/
	if (gpio_request(PCM512X_HEAD_SHUT, GPIO_NAME_PCM512X_HEAD_SHUT) < 0) {
		pr_err("Headphone shutdown control %d request failed!\n",
				PCM512X_HEAD_SHUT);
	}
	gpio_direction_output(PCM512X_HEAD_SHUT, 1);

	/** configure Amp Shutdown pin **/
	if (gpio_request(PCM512X_AMP_SHUT, GPIO_NAME_PCM512X_AMP_SHUT) < 0) {
		pr_err("AMP shutdown control %d request failed!\n",
				PCM512X_AMP_SHUT);
	}
	gpio_direction_output(PCM512X_AMP_SHUT, 1);

	/** configure Amp Mute pin **/
	if (gpio_request(PCM512X_AMP_MUTE, GPIO_NAME_PCM512X_AMP_MUTE) < 0) {
		pr_err("AMP Mute control %d request failed!\n",
				PCM512X_AMP_MUTE);
	}
	gpio_direction_output(PCM512X_AMP_MUTE, 0);

	return 0;

platform_device_add_failed:
	platform_device_put(snd_allo_piano_link_device_pcm512x);

	return ret;
}

static void __exit snd_allo_piano_link_exit(void)
{
	platform_device_unregister(snd_allo_piano_link_device_pcm512x);
}

module_init(snd_allo_piano_link_init);
module_exit(snd_allo_piano_link_exit);

/* Module information */
MODULE_AUTHOR("Baswaraj K <jaikumar@cem-solutions.net>");
MODULE_DESCRIPTION("ALSA ASoC Machine Driver for Allo Piano DAC Plus");
MODULE_LICENSE("GPL");
