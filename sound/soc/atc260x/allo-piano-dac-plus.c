/*
 * ALSA ASoC Machine Driver for Allo Piano DAC Plus Subwoofer
 *
 * Author:	Baswaraj K <jaikumar@cem-solutions.net>
 *		Copyright 2016
 *		based on code by Daniel Matuschek <info@crazy-audio.com>
 *		based on code by Florian Meier <florian.meier@koalo.de>
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
#include <linux/firmware.h>
#include <mach/hardware.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <sound/tlv.h>
#include <linux/slab.h>
#include "sndrv-owl.h"
#include "../codecs/pcm512x.h"

#define NUM_CODECS	2

#define GPIO_NAME_PCM5142_DAC_ONE_MUTE	"dac_one_mute"
#define PCM5142_DAC_ONE_MUTE	OWL_GPIO_PORTB(18)

#define GPIO_NAME_PCM5142_DAC_TWO_MUTE	"dac_two_mute"
#define PCM5142_DAC_TWO_MUTE	OWL_GPIO_PORTB(12)

#define GPIO_NAME_PCM512X_AMP_SHUT	"amp_shut"
#define PCM512X_AMP_SHUT	OWL_GPIO_PORTB(31)

#define GPIO_NAME_PCM512X_AMP_MUTE      "amp_mute"
#define PCM512X_AMP_MUTE        OWL_GPIO_PORTB(17)

static bool digital_gain_0db_limit = true;
module_param(digital_gain_0db_limit, bool, 0);
MODULE_PARM_DESC(digital_gain_0db_limit, "Set the max volume");


struct dsp_code {
	char i2c_addr;
	char offset;
	char val;
};

struct glb_pool {
        struct mutex lock;
        unsigned int set_lowpass;
        unsigned int set_mode;
        unsigned int set_rate;
        unsigned int dsp_page_number;
};

static const char *const allo_piano_mode_texts[] = {
	"2.0",
	"2.1",
	"2.2",
};

static const SOC_ENUM_SINGLE_DECL(allo_piano_mode_enum,
					0, 0, allo_piano_mode_texts);

static const char * const allo_piano_dsp_low_pass_texts[] = {
	"60",
	"70",
	"80",
	"90",
	"100",
	"110",
	"120",
	"130",
	"140",
	"150",
	"160",
	"170",
	"180",
	"190",
	"200",
};

static const SOC_ENUM_SINGLE_DECL(allo_piano_enum,
		0, 0, allo_piano_dsp_low_pass_texts);

static int __snd_allo_piano_dsp_program(struct snd_soc_pcm_runtime *rtd,
		unsigned int mode, unsigned int rate, unsigned int lowpass)
{
	const struct firmware *fw;
	char firmware_name[60];
	int ret = 0, dac = 0;
	struct snd_soc_card *card = rtd->card;
	struct glb_pool *glb_ptr = card->drvdata;

	if (rate <= 46000)
		rate = 44100;
	else if (rate <= 68000)
		rate = 48000;
	else if (rate <= 92000)
		rate = 88200;
	else if (rate <= 136000)
		rate = 96000;
	else if (rate <= 184000)
		rate = 176400;
	else
		rate = 192000;

	/* same configuration loaded */
	if ((rate == glb_ptr->set_rate) && (lowpass == glb_ptr->set_lowpass)
			&& (mode == glb_ptr->set_mode))
		return 0;

	glb_ptr->set_rate = rate;
	glb_ptr->set_mode = mode;

	if (mode == 0) { /* 2.0 */
		ret = pcm512x_set_reg(1, PCM512x_MUTE, 0x11);
		return 1;
	} else {
		ret = pcm512x_set_reg(1, PCM512x_MUTE, 0x00);
	}

	glb_ptr->set_lowpass = lowpass;

	for (dac = 0; dac < NUM_CODECS; dac++) {
		struct dsp_code *dsp_code_read;
		int i = 1;
		struct snd_soc_codec *codec = rtd->codec;

		if (dac == 0) { /* high */
			sprintf(firmware_name,
				"allo/piano/2.2/allo-piano-dsp-%d-%d-%d.bin",
				rate, ((glb_ptr->set_lowpass * 10) + 60), dac);
		} else { /* low */
			sprintf(firmware_name,
				"allo/piano/2.%d/allo-piano-dsp-%d-%d-%d.bin",
				glb_ptr->set_mode, rate,
				((glb_ptr->set_lowpass * 10) + 60), dac);
		}

		dev_info(codec->dev, "Dsp Firmware File Name: %s\n",
			firmware_name);

		ret = request_firmware(&fw, firmware_name, codec->dev);
		if (ret < 0) {
			dev_err(codec->dev,
				"Error: AlloPiano Firmware %s missing. %d\n",
				firmware_name, ret);
			continue;
		}

		while (i < (fw->size - 1)) {
			dsp_code_read = (struct dsp_code *)&fw->data[i];
			ndelay(700000);
			if (dsp_code_read->offset == 0) {
				glb_ptr->dsp_page_number = dsp_code_read->val;
				ret = pcm512x_set_reg(dac,
                                        PCM512x_PAGE_BASE(0),
                                        dsp_code_read->val);
					
			} else if (dsp_code_read->offset != 0) {
				ret = pcm512x_set_reg(dac,
                                        (PCM512x_PAGE_BASE(glb_ptr->dsp_page_number) +
                                        dsp_code_read->offset),
                                        dsp_code_read->val);
			}
			if (ret < 0) {
				dev_err(codec->dev,
					"Failed to write Register: %d\n", ret);
				break;
			}
			i = i + 3;
		}
		release_firmware(fw);
	}
	return 1;
}

static int snd_allo_piano_dsp_program(struct snd_soc_pcm_runtime *rtd,
		unsigned int mode, unsigned int rate, unsigned int lowpass)
{
	struct snd_soc_card *card = rtd->card;
	struct glb_pool *glb_ptr = card->drvdata;
	int ret = 0;

	mutex_lock(&glb_ptr->lock);

	ret = __snd_allo_piano_dsp_program(rtd,
				mode, rate, lowpass);
	mutex_unlock(&glb_ptr->lock);

	return ret;
}

static int snd_allo_piano_mode_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct glb_pool *glb_ptr = card->drvdata;

	ucontrol->value.integer.value[0] = glb_ptr->set_mode;
	return 0;
}

static int snd_allo_piano_mode_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct snd_soc_pcm_runtime *rtd;
	struct glb_pool *glb_ptr = card->drvdata;

	rtd = snd_soc_get_pcm_runtime(card, card->dai_link[0].name);
	return(snd_allo_piano_dsp_program(rtd,
				ucontrol->value.integer.value[0],
				glb_ptr->set_rate, glb_ptr->set_lowpass));
}

static int snd_allo_piano_lowpass_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct glb_pool *glb_ptr = card->drvdata;

	ucontrol->value.integer.value[0] = glb_ptr->set_lowpass;
	return 0;
}

static int snd_allo_piano_lowpass_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct snd_soc_pcm_runtime *rtd;
	struct glb_pool *glb_ptr = card->drvdata;

	rtd = snd_soc_get_pcm_runtime(card, card->dai_link[0].name);
	return(snd_allo_piano_dsp_program(rtd,
				glb_ptr->set_mode, glb_ptr->set_rate,
				ucontrol->value.integer.value[0]));
}

static int pcm512x_get_reg_sub(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int left_val = 0, right_val = 0, ret = 0;

	ret = pcm512x_get_reg(1,
			PCM512x_DIGITAL_VOLUME_2, &left_val);
	if (ret)
		return ret;

	ret = pcm512x_get_reg(1,
			PCM512x_DIGITAL_VOLUME_3, &right_val);
	if (ret)
		return ret;

	ucontrol->value.integer.value[0] =
			(~(left_val >> mc->shift)) & mc->max;
	ucontrol->value.integer.value[1] =
			(~(right_val >> mc->shift)) & mc->max;

	return 0;
}

static int pcm512x_set_reg_sub(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int left_val =
			(ucontrol->value.integer.value[0] & mc->max);
	unsigned int right_val =
			(ucontrol->value.integer.value[1] & mc->max);
	int ret = 0;

	ret = pcm512x_set_reg(1,
			PCM512x_DIGITAL_VOLUME_2, (~left_val));
	if (ret < 0)
		return ret;

	ret = pcm512x_set_reg(1,
			PCM512x_DIGITAL_VOLUME_3, (~right_val));
	if (ret < 0)
		return ret;

	return 1;
}
static int pcm512x_get_reg_sub_switch(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int val = 0;
	int ret = 0;

	ret = pcm512x_get_reg(1, PCM512x_MUTE, &val);
	if (ret < 0)
		return ret;

	ucontrol->value.integer.value[0] = (val & 0x10) ? 0 : 1;
	ucontrol->value.integer.value[1] = (val & 0x01) ? 0 : 1;

	return val;
}

static int pcm512x_set_reg_sub_switch(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	unsigned int left_val = (ucontrol->value.integer.value[0]);
	unsigned int right_val = (ucontrol->value.integer.value[1]);
	int ret = 0;

	ret = pcm512x_set_reg(1, PCM512x_MUTE,
			~((left_val & 0x01) << 4 | (right_val & 0x01)));
	if (ret < 0)
		return ret;

	return 1;
}

static const DECLARE_TLV_DB_SCALE(digital_tlv_sub, -10350, 50, 1);

static const struct snd_kcontrol_new allo_piano_controls[] = {
	SOC_ENUM_EXT("Subwoofer mode Route",
			allo_piano_mode_enum,
			snd_allo_piano_mode_get,
			snd_allo_piano_mode_put),

	SOC_ENUM_EXT("Lowpass Route", allo_piano_enum,
			snd_allo_piano_lowpass_get,
			snd_allo_piano_lowpass_put),

	SOC_DOUBLE_R_EXT_TLV("Subwoofer Playback Volume",
			PCM512x_DIGITAL_VOLUME_2,
			PCM512x_DIGITAL_VOLUME_3, 0, 255, 1,
			pcm512x_get_reg_sub,
			pcm512x_set_reg_sub,
			digital_tlv_sub),

	SOC_DOUBLE_EXT("Subwoofer Playback Switch",
			PCM512x_MUTE,
			PCM512x_RQML_SHIFT,
			PCM512x_RQMR_SHIFT, 1, 1,
			pcm512x_get_reg_sub_switch,
			pcm512x_set_reg_sub_switch),
};

static int snd_allo_piano_dac_init(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;
	struct snd_soc_card *card = rtd->card;
        struct glb_pool *glb_ptr;

        glb_ptr = kzmalloc(sizeof(struct glb_pool), GFP_KERNEL);
        if (!glb_ptr)
                return -ENOMEM;

        card->drvdata = glb_ptr;

	mutex_init(&glb_ptr->lock);

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

static int snd_allo_piano_dac_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	unsigned int rate = params_rate(params);
	struct snd_soc_card *card = rtd->card;
	struct glb_pool *glb_ptr = card->drvdata;

	if (digital_gain_0db_limit) {
		ret = snd_soc_limit_volume(codec,
				"Subwoofer Playback Volume", 207);
		if (ret < 0)
			dev_warn(codec->dev,
				"Failed to set volume limit: %d\n", ret);
	}

	ret = snd_allo_piano_dsp_program(rtd, glb_ptr->set_mode, rate, glb_ptr->set_lowpass);
	if (ret < 0)
		return ret;

	return ret;
}

static struct snd_soc_ops allo_pianodac_ops = {
	.hw_params = snd_allo_piano_dac_hw_params,
};

static struct snd_soc_dai_link allo_piano_dac_link_dai[] = {
		{
			.name = "PianoDACPlus",
			.stream_name = "PianoDACPlus",
			.cpu_dai_name = "owl-audio-i2s",
			.codec_dai_name = "pcm512x-hifi",
			.init = snd_allo_piano_dac_init,
			.platform_name = "atm7059-pcm-audio",
			.codec_name = "pcm512x.2-004c",
			.ops = &allo_pianodac_ops,
			.init	= snd_allo_piano_dac_init,
		},
	};

static struct snd_soc_card snd_soc_allo_pianodac = {
	.name = "PianoDACPlus",
	.owner = THIS_MODULE,
	.dai_link = allo_piano_dac_link_dai,
	.num_links = ARRAY_SIZE(allo_piano_dac_link_dai),
	.controls = allo_piano_controls,
	.num_controls = ARRAY_SIZE(allo_piano_controls),
};

static struct platform_device *atm7059_link_snd_device_pcm512x;

static int __init atm7059_link_init(void)
{
	int ret = 0;

	atm7059_link_snd_device_pcm512x =
				platform_device_alloc("soc-audio", 1);
	if (!atm7059_link_snd_device_pcm512x) {
		snd_err("ASoC: Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(atm7059_link_snd_device_pcm512x,
				&snd_soc_allo_pianodac);
	ret = platform_device_add(atm7059_link_snd_device_pcm512x);
	if (ret) {
		snd_err("ASoC: Platform device allocation failed\n");
		goto platform_device_add_failed;
	}

	/** Configure Mute and Shutdown for DAC and Amplifier board **/
	act_setl(act_readl(MFP_CTL1) | (0x1 << 22), MFP_CTL1); /* GPIOB pins */

	/** Configure First DAC Mute pin **/
	ret = gpio_request(PCM5142_DAC_ONE_MUTE,
				GPIO_NAME_PCM5142_DAC_ONE_MUTE);
	if (ret < 0) {
		pr_err("First DAC mute control %d request failed!\n",
				PCM5142_DAC_ONE_MUTE);
		goto platform_device_add_failed;
	}
	gpio_direction_output(PCM5142_DAC_ONE_MUTE, 1);

	/** Configure Second DAC Mute pin **/
	ret = gpio_request(PCM5142_DAC_TWO_MUTE,
				GPIO_NAME_PCM5142_DAC_TWO_MUTE);
	if (ret < 0) {
		pr_err("Second DAC mute control %d request failed!\n",
				PCM5142_DAC_TWO_MUTE);
		goto platform_device_add_failed;
	}
	gpio_direction_output(PCM5142_DAC_TWO_MUTE, 1);

	/** Configure AMP Shutdown pins **/
	if (gpio_request(PCM512X_AMP_SHUT, GPIO_NAME_PCM512X_AMP_SHUT) < 0) {
		pr_err("AMP shutdown control %d request failed!\n",
				PCM512X_AMP_SHUT);
	}
	gpio_direction_output(PCM512X_AMP_SHUT, 1);

	/** configure the Amp Mute pins **/
	if (gpio_request(PCM512X_AMP_MUTE, GPIO_NAME_PCM512X_AMP_MUTE) < 0) {
		pr_err("AMP Mute control %d request failed!\n",
				PCM512X_AMP_MUTE);
	}
	gpio_direction_output(PCM512X_AMP_MUTE, 0);
	return 0;

platform_device_add_failed:
	platform_device_put(atm7059_link_snd_device_pcm512x);

	return ret;
}

static void __exit atm7059_link_exit(void)
{
        kfree (glb_ptr);
	platform_device_unregister(atm7059_link_snd_device_pcm512x);
}

module_init(atm7059_link_init);
module_exit(atm7059_link_exit);

/* Module information */
MODULE_AUTHOR("Baswaraj K <jaikumar@cem-solutions.net>");
MODULE_DESCRIPTION("ALSA ASoC Machine Driver for Allo Piano DAC Plus");
MODULE_LICENSE("GPL");
