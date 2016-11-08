/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2009 Actions Semi Inc.
 */

#ifndef __ATM7059_SNDRV_H__
#define __ATM7059_SNDRV_H__
#include <sound/pcm.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/control.h>
#include <linux/atomic.h>
#include <mach/hdmac-owl.h>
#include <linux/dmaengine.h>
#include <linux/pinctrl/consumer.h>

#define PMU_NOT_USED	-1

#define COMPILETEST 0

#define ATM7059_AIF_I2S 0
#define ATM7059_AIF_HDMI 1
#define ATM7059_AIF_SPDIF 2
static int error_switch = 1;
static int debug_switch = 1;

#define SND_DEBUG
#ifdef SND_DEBUG
#define snd_err(fmt, args...) \
	if (error_switch) \
		printk(KERN_ERR"[SNDRV]:[%s] "fmt"\n", __func__, ##args)

#define snd_dbg(fmt, args...) \
	if (debug_switch) \
		printk(KERN_DEBUG"[SNDRV]:[%s] "fmt"\n", __func__, ##args)
#endif

enum {
	O_MODE_I2S,
	O_MODE_HDMI,
	O_MODE_SPDIF
};

enum {
	SPEAKER_ON = 0,
	HEADSET_MIC = 1,
	HEADSET_NO_MIC = 2,
};





//////////////////////////////////////////////
enum {
	SAMPLE_RATE_32000= 1,

	SAMPLE_RATE_44100=2,
	DEFAULT_SAMPLE_RATE=SAMPLE_RATE_44100,

	SAMPLE_RATE_48000=3,
	SAMPLE_RATE_88200,
	SAMPLE_RATE_96000,
	SAMPLE_RATE_176400,
	SAMPLE_RATE_192000,
	SAMPLE_RATE_352800,
	SAMPLE_RATE_384000,
};
enum {
	USER_TYPE_Audio60958_3 = 0,
	USER_TYPE_Audio60958_4,
	USER_TYPE_Audio61937,
	USER_TYPE_AudioSMPTE_337M_AND_OTHERS,
	USER_TYPE_TSTAMP_LAST = USER_TYPE_AudioSMPTE_337M_AND_OTHERS,
};
//////////////////////////////////////////////

typedef struct {
	short sample_rate;	/* 真实采样率除以1000 */
	char index[2];		/* 对应硬件寄存器的索引值 */
} fs_t;

typedef struct {
	unsigned int earphone_gpios;
	unsigned int speaker_gpios;
	unsigned int earphone_output_mode;
	unsigned int mic_num;
	unsigned int mic0_gain[2];
	unsigned int speaker_gain[2];
	unsigned int earphone_gain[2];
	unsigned int speaker_volume;
	unsigned int earphone_volume;
	unsigned int earphone_detect_mode;
	unsigned int mic_mode;
	unsigned int earphone_detect_method;
	unsigned int adc_plugin_threshold;
	unsigned int adc_level;
} audio_hw_cfg_t;

//extern audio_hw_cfg_t audio_hw_cfg;

struct atm7059_pcm_priv {
	int output_mode;
	volatile int userType;
	struct pinctrl *pc;
	struct pinctrl_state *ps;
};

enum{
I2S_SPDIF_NUM = 0,
GPIO_MFP_NUM,
HDMI_NUM,
MAX_RES_NUM
};

void set_dai_reg_base(int num);
u32 snd_dai_readl(u32 reg);
void snd_dai_writel(u32 val, u32 reg);

extern void hdmihw_write_reg(u32 val, const u16 idx);
extern int hdmihw_read_reg(const u16 idx);
#ifdef CONFIG_SND_UBUNTU
int audio_set_output_mode(struct snd_pcm_substream *substream, int value);
#endif
#endif /* ifndef __ATM7059_SNDRV_H__ */
