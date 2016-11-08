#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/clk.h>			/* clk_enable */
#include <mach/hardware.h>
#include "sndrv-owl.h"
#include <mach/module-owl.h>
//#include "common-regs-owl.h"


static int audio_clk_enable;

#define atm7059_spdif_RATES SNDRV_PCM_RATE_8000_192000
#define atm7059_spdif_FORMATS (SNDRV_PCM_FMTBIT_S16_LE \
| SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S32_LE |\
		SNDRV_PCM_FMTBIT_S24_LE)

static ssize_t error_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int cnt;

	cnt = sprintf(buf, "%d\n(Note: 1: open, 0:close)\n", error_switch);
	return cnt;
}

static ssize_t error_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int cnt, tmp;
	cnt = sscanf(buf, "%d", &tmp);
	switch (tmp) {
	case 0:
	case 1:
		error_switch = tmp;
		break;
	default:
		printk(KERN_EMERG"invalid input\n");
		break;
	}
	return count;
}

static ssize_t debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int cnt;

	cnt = sprintf(buf, "%d\n(Note: 1: open, 0:close)\n", debug_switch);
	return cnt;
}

static ssize_t debug_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int cnt, tmp;
	cnt = sscanf(buf, "%d", &tmp);
	switch (tmp) {
	case 0:
	case 1:
		debug_switch = tmp;
		break;
	default:
		printk(KERN_INFO"invalid input\n");
		break;
	}
	return count;
}

static void spdif_audio_enable(void)
{
	    printk("%s %d\n",__FUNCTION__,__LINE__);
	u32 val = 0;
	//1, CONFIG MFP SPDIF
		//BIT[21-19] = B[100]
	val = act_readl(MFP_CTL3);
	printk(KERN_ERR "%s: MFP_CTL3=0x%x\n", __func__, val);

	val &= 0xFFc7FFFF; //clear bit[21-19]
	val |= 0x00200000;//set bit21
	act_writel(val,MFP_CTL3);
		
	val = act_readl(MFP_CTL3);
	printk(KERN_ERR "%s: new MFP_CTL3=0x%x\n", __func__, val);
	//2, CONFIG CMU_AUDIOPLL SPDIF
	    //CMU_AUDIOPLL,SPDIF CLK=5.644MHZ,AUDIPLL ENABLE,44100
	act_writel(0x50000010,CMU_AUDIOPLL);

	//3, CONFIG SPDIF_HDMI_CTL
		//spdif en, spdif fifo en, spdif fifo drq en
	act_writel(0x411,SPDIF_HDMI_CTL);
		
	//4, CONFIG SPDIF_CLSTAT  SPDIF_CHSTAT 
		//set the sample rate:byte3 0x12-->48k; 0x13-->32k;0x10-->44.1kHz
	act_writel(0x10550100,SPDIF_CLSTAT);
	act_writel(0x10550100,SPDIF_CHSTAT);
	return;
}

static void spdif_audio_disable(void)
{
	u32 val = 0;
	//3, DISABLE SPDIF_HDMI_CTL, ENABLE SPDIF  SPDIF_FIFO
	act_writel(0x0,SPDIF_HDMI_CTL);
	
	//2, DISABLE CMU_AUDIOPLL SPDIF, ENABLE I2S
	act_writel(0x00330010,CMU_AUDIOPLL);

	//1, DISABLE MFP SPDIF
	val = act_readl(MFP_CTL3);
	val &= 0xFFc7FFFF; //clear bit[21-19]
	val |= 0x00100000;//set bit20 set uart0_tx pin as uart tx
	act_writel(val,MFP_CTL3);
	return;
}

static struct device_attribute spdif_attr[] = {
	__ATTR(error, S_IRUSR | S_IWUSR, error_show, error_store),
	__ATTR(debug, S_IRUSR | S_IWUSR, debug_show, debug_store),
};

static int spdif_audio_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{	
    printk("%s %d\n",__FUNCTION__,__LINE__);
	if (audio_clk_enable == 0) {
		module_clk_enable(MOD_ID_SPDIF);
		spdif_audio_enable();
		audio_clk_enable = 1;
	}
#ifdef CONFIG_SND_UBUNTU
//      audio_set_output_mode(substream, O_MODE_SPDIF); 
  /*     commented above line & below lines added for building as module sud */
               struct snd_soc_pcm_runtime *rtd = substream->private_data;
               struct snd_soc_platform *platform = rtd->platform;
               struct atm7059_pcm_priv *pcm_priv = snd_soc_platform_get_drvdata(platform);
               pcm_priv->output_mode = O_MODE_SPDIF;
#endif
	return 0;
}

static int spdif_dai_hw_free(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	if (audio_clk_enable == 1) {
		module_clk_disable(MOD_ID_SPDIF);
		spdif_audio_disable();
		audio_clk_enable = 0;
	}
	return 0;
}

static int spdif_audio_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	/* nothing should to do here now */
	return 0;
}

static int spdif_audio_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	/* nothing should to do here now */
	return 0;
}

static int spdif_audio_set_dai_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	/* nothing should to do here now */
	return 0;
}

struct snd_soc_dai_ops spdif_aif_dai_ops = {
	.hw_params = spdif_audio_hw_params,
	.prepare = spdif_audio_prepare,
	.set_fmt = spdif_audio_set_dai_fmt,
	.set_sysclk = spdif_audio_set_dai_sysclk,
	.hw_free = spdif_dai_hw_free,
};

struct snd_soc_dai_driver codec_spdif_dai[] = {
	{
		.name = "atm7059-spdif-dai",
		.id = ATM7059_AIF_SPDIF,
		.playback = {
			.stream_name = "atm7059 spdif Playback",
			.channels_min = 1,
			.channels_max = 8,
			.rates = atm7059_spdif_RATES,
			.formats = atm7059_spdif_FORMATS,
		},
		.ops = &spdif_aif_dai_ops,
	},
};

static int codec_spdif_probe(struct snd_soc_codec *codec)
{
	/* nothing should to do here now */
	return 0;
}

static int codec_spdif_remove(struct snd_soc_codec *codec)
{
	/* nothing should to do here now */
	return 0;
}

static struct snd_soc_codec_driver soc_codec_spdif = {
	.probe = codec_spdif_probe,
	.remove = codec_spdif_remove,
};

static  int atm7059_spdif_probe(struct platform_device *pdev)
{
	printk("%s %d\n",__FUNCTION__,__LINE__);
	return snd_soc_register_codec(&pdev->dev, &soc_codec_spdif,
			codec_spdif_dai, ARRAY_SIZE(codec_spdif_dai));
}

static int  atm7059_spdif_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver atm7059_spdif_driver = {
	.driver = {
			.name = "atm7059-spdif-audio",
			.owner = THIS_MODULE,
	},

	.probe = atm7059_spdif_probe,
	.remove = atm7059_spdif_remove,
};

static struct platform_device *atm7059_spdif_device;

static int __init atm7059_spdif_init(void)
{
	int ret;
	int i = 0;
	atm7059_spdif_device = platform_device_alloc("atm7059-spdif-audio", -1);
	if (!atm7059_spdif_device) {
		snd_err("ASoC: Platform device atm7059-spdif-audio alloc failed\n");
		ret = -ENOMEM;
		goto err;
	}
	ret = platform_device_add(atm7059_spdif_device);
	if (ret) {
		snd_err("ASoC: Platform device atm7059-spdif-audio add failed\n");
		goto err_device_add;
	}
	ret = platform_driver_register(&atm7059_spdif_driver);
	if (ret) {
		snd_err("ASoC: Platform driver atm7059-spdif-audio register failed\n");
		goto err_driver_register;
	}
	for (i = 0; i < ARRAY_SIZE(spdif_attr); i++) {
		ret = device_create_file(
			&atm7059_spdif_device->dev, &spdif_attr[i]);
		if (ret) {
			snd_err("Add device file failed");
			goto device_create_file_failed;
		}
	}
	return 0;

device_create_file_failed:
err_driver_register:
	platform_device_unregister(atm7059_spdif_device);
err_device_add:
	platform_device_put(atm7059_spdif_device);
err:
	return ret;
}
static void __exit atm7059_spdif_exit(void)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(spdif_attr); i++) {
		device_remove_file(&atm7059_spdif_device->dev, &spdif_attr[i]);
	}

	platform_driver_unregister(&atm7059_spdif_driver);
	platform_device_unregister(atm7059_spdif_device);
	atm7059_spdif_device = NULL;
}

module_init(atm7059_spdif_init);
module_exit(atm7059_spdif_exit);

MODULE_AUTHOR("sall.xie <sall.xie@actions-semi.com>");
MODULE_DESCRIPTION("atm7059 SPDIF AUDIO module");
MODULE_LICENSE("GPL");
