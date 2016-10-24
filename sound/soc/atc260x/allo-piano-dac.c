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

 /****** Mute and Shutdown enable **********/
#define GPIO_NAME_PCM512X_DAC_MUTE	"dac_mute"
#define PCM512X_DAC_MUTE	OWL_GPIO_PORTB(18)

#define GPIO_NAME_PCM512X_DAC_SHUT	"dac_shut"
#define PCM512X_DAC_SHUT	OWL_GPIO_PORTB(13)

#define GPIO_NAME_PCM512X_AMP_SHUT	"amp_shut"
#define PCM512X_AMP_SHUT	OWL_GPIO_PORTB(31)


static int snd_allo_piano_dac_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{



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
	snd_dbg("snd_allo_piano_dac_init() called\n");

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
/*	.num_links = ARRAY_SIZE(allo_piano_dac_link_dai),*/
	.num_links = 1,
};

static struct platform_device *snd_allo_piano_link_device_pcm512x;

static int __init snd_allo_piano_link_init(void)
{
	int ret = 0;
	int gpio_ret = 0;

	snd_allo_piano_link_device_pcm512x =
				platform_device_alloc("soc-audio", 1);
	if (!snd_allo_piano_link_device_pcm512x) {
		snd_err("ASoC: Platform device allocation failed\n");
		ret = -ENOMEM;
	}

	platform_set_drvdata(snd_allo_piano_link_device_pcm512x,
				&snd_soc_allo_pianodac);
	ret = platform_device_add(snd_allo_piano_link_device_pcm512x);
	if (ret) {
		snd_err("ASoC: Platform device allocation failed\n");
		goto platform_device_add_failed;
	}

	/****** Configure Mute and Shutdown for DAC and Amplifier board ******/
	/*** configure the GPIOB pins ***/
	act_setl(act_readl(MFP_CTL1) | (0x1 << 22), MFP_CTL1);

	/********** Configure the Mute pins ********/
	gpio_ret = gpio_request(PCM512X_DAC_MUTE, GPIO_NAME_PCM512X_DAC_MUTE);

	if (gpio_ret < 0) {
		pr_err("%s: DAC mute control %d request failed!\n",
				__func__, PCM512X_DAC_MUTE);
		return 0;
	}
	gpio_direction_output(PCM512X_DAC_MUTE, 1);

	/************ configure the Shutdown pins *****/
	gpio_ret = gpio_request(PCM512X_DAC_SHUT, GPIO_NAME_PCM512X_DAC_SHUT);

	if (gpio_ret < 0) {
		pr_err("%s: DAC shutdown control %d request failed!\n",
				__func__, PCM512X_DAC_SHUT);
		return 0;
	}
	gpio_direction_output(PCM512X_DAC_SHUT, 1);

	gpio_ret = 0;

	gpio_ret = gpio_request(PCM512X_AMP_SHUT, GPIO_NAME_PCM512X_AMP_SHUT);

	if (gpio_ret < 0) {
		pr_err("%s: AMP shutdown control %d request failed!\n",
				__func__, PCM512X_AMP_SHUT);
		return 0;
	}
	gpio_direction_output(PCM512X_AMP_SHUT, 1);

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
