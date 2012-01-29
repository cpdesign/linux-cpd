/*
 *
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <mach/hardware.h>

#include "pcm1774.h"

struct pcm1774_priv {
	int master;
	int fmt;

	unsigned int sysclk;

	enum snd_soc_control_type control_type;
	void *control_data;

	struct snd_pcm_substream *master_substream;
	struct snd_pcm_substream *slave_substream;
};

static int all_reg[] = {
	PCM1774_VOL_HPA_L,
	PCM1774_VOL_HPA_R,
	PCM1774_DAC_MUTE_L,
	PCM1774_DAC_MUTE_R,
	PCM1774_DAC_SAMPLE,
	PCM1774_AMIX_PWR,
	PCM1774_DAC_PWR,
	PCM1774_AOUT_CFG,
	PCM1774_HPA_DETECT,
	PCM1774_STATUS,
	PCM1774_PG_PWR,
	PCM1774_MASTER_MODE,
	PCM1774_SYS_RESET,
	PCM1774_CLK_RATE,
	PCM1774_AIN_SELECT,
	PCM1774_AMIX_SELECT,
	PCM1774_APATH_GAIN,
	PCM1774_MIC_BOOST,
	PCM1774_BASS_GAIN,
	PCM1774_MID_GAIN,
	PCM1774_TREB_GAIN,
	PCM1774_3D_SOUND,
	PCM1774_MONO_MIX,
	PCM1774_PG_GAIN,
	PCM1774_PWR_TIME_CTRL,
};

static unsigned int cache_index_of_reg(unsigned int reg)
{
	int ii;
	for(ii=0; ii< ARRAY_SIZE(all_reg); ++ii){
		if(reg == all_reg[ii])
			return ii;
	}
	return ii;
}

static int pcm1774_set_bias_level(struct snd_soc_codec *codec,
				   enum snd_soc_bias_level level);

#define PCM1774_MAX_CACHED_REG ARRAY_SIZE(all_reg)
static u8 pcm1774_regs[PCM1774_MAX_CACHED_REG];


static unsigned int pcm1774_read_reg_cache(struct snd_soc_codec *codec,
					    unsigned int reg)
{
	u8 *cache = codec->reg_cache;
	int offset = cache_index_of_reg(reg);
	if (offset >= ARRAY_SIZE(pcm1774_regs))
		return -EINVAL;
	pr_debug("r r:%02x,v:%02x\n", reg, cache[offset]);
	return cache[offset];
}

static unsigned int pcm1774_hw_read(struct snd_soc_codec *codec,
				     unsigned int reg)
{
	struct i2c_client *client = codec->control_data;
	int i2c_ret;
	u16 value;
	u8 buf0[1], buf1[1];
	u16 addr = client->addr;
	u16 flags = client->flags;
	struct i2c_msg msg[2] = {
		{addr, flags, 1, buf0},
		{addr, flags | I2C_M_RD, 1, buf1},
	};

	buf0[0] = reg & 0xff;
	i2c_ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (i2c_ret < 0) {
		pr_err("%s: read reg error : Reg 0x%02x\n", __func__, reg);
		return 0;
	}

	value = buf1[0];

	pr_debug("r r:%02x,v:%02x\n", reg, value);
	return value;
}

static unsigned int pcm1774_read(struct snd_soc_codec *codec, unsigned int reg)
{
	if (reg > PCM1774_MAX_CACHED_REG)
		return pcm1774_hw_read(codec, reg);
	else
		return pcm1774_read_reg_cache(codec, reg);
}

static inline void pcm1774_write_reg_cache(struct snd_soc_codec *codec,
					    u16 reg, unsigned int value)
{
	u8 *cache = codec->reg_cache;
	unsigned int offset = cache_index_of_reg(reg);
	if (offset < ARRAY_SIZE(pcm1774_regs))
		cache[offset] = value;
}

static int pcm1774_write(struct snd_soc_codec *codec, unsigned int reg,
			  unsigned int value)
{
	struct i2c_client *client = codec->control_data;
	u16 addr = client->addr;
	u16 flags = client->flags;
	u8 buf[2];
	int i2c_ret;
	struct i2c_msg msg = { addr, flags, 2, buf };

	pr_debug("w r:%02x,v:%04x\n", reg, value);
	buf[0] = reg & 0xff;
	buf[1] = value & 0xff;

	i2c_ret = i2c_transfer(client->adapter, &msg, 1);
	if (i2c_ret < 0) {
		pr_err("%s: write reg error : Reg 0x%02x = 0x%04x\n",
		       __func__, reg, value);
		return -EIO;
	}

	pcm1774_write_reg_cache(codec, reg, value);
	return i2c_ret;
}

static void pcm1774_sync_reg_cache(struct snd_soc_codec *codec)
{
	int reg;
	for (reg = 0; reg <= PCM1774_MAX_CACHED_REG; ++reg)
		pcm1774_write_reg_cache(codec, reg,
					 pcm1774_hw_read(codec, reg));
}

#if 0
static int pcm1774_restore_reg(struct snd_soc_codec *codec, unsigned int reg)
{
	unsigned int cached_val, hw_val;

	cached_val = pcm1774_read_reg_cache(codec, reg);
	hw_val = pcm1774_hw_read(codec, reg);

	if (hw_val != cached_val)
		return pcm1774_write(codec, reg, cached_val);

	return 0;
}
#endif


#ifdef DEBUG
static void dump_reg(struct snd_soc_codec *codec)
{
	int i, reg;
	for (i = 0; i < ARRAY_SIZE(all_reg); i++) {
		reg = pcm1774_read(codec, all_reg[i]);
		printk(KERN_INFO "d r %02x, v %02x\n", all_reg[i], reg);
	}
}
#else
static void dump_reg(struct snd_soc_codec *codec)
{
}
#endif


static const struct snd_soc_dapm_widget pcm1774_dapm_widgets[] = {
	/* pins */
	SND_SOC_DAPM_OUTPUT("HP_L"),
	SND_SOC_DAPM_OUTPUT("HP_R"),

	/* output */
//	SND_SOC_DAPM_DAC("DACL", "Left Playback", PCM1774_DAC_PWR, 5, 0),
//	SND_SOC_DAPM_DAC("DACR", "Right Playback", PCM1774_DAC_PWR, 6, 0),
};

static const struct snd_soc_dapm_route audio_map[] = {
//	{"HP_L", NULL, "DACL"},
//	{"HP_R", NULL, "DACR"},
};



static int pcm1774_add_widgets(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;

	pr_info("%s\n" ,__func__);

	ret = snd_soc_dapm_new_controls(dapm,
			pcm1774_dapm_widgets,
			ARRAY_SIZE(pcm1774_dapm_widgets));
	if (ret != 0) {
		dev_err(codec->dev, "dapm control register failed\n");
		return ret;
	}

	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));
	if (ret != 0) {
		dev_err(codec->dev, "DAPM add routes failed\n");
		return ret;
	}

	return 0;
}

static int dac_info_volsw(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0x3f;
	return 0;
}

static int dac_get_volsw(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int l, r;

	l = snd_soc_read(codec, PCM1774_VOL_HPA_L) & 0x3f;
	r = snd_soc_read(codec, PCM1774_VOL_HPA_R) & 0x3f;

	ucontrol->value.integer.value[0] = l;
	ucontrol->value.integer.value[1] = r;

	return 0;
}

static int dac_put_volsw(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg, l, r;

	l = ucontrol->value.integer.value[0];
	r = ucontrol->value.integer.value[1];

	if (l <= 0 )
		l = 0;
	if (l > 0x3f)
		l = 0x3f;

	if (r <= 0)
		r = 0;
	if (r > 0x3f)
		r = 0x3f;

	reg = snd_soc_read(codec, PCM1774_VOL_HPA_L);
	snd_soc_write(codec, PCM1774_VOL_HPA_L, l | (reg & 0x3f));

	reg = snd_soc_read(codec, PCM1774_VOL_HPA_R);
	snd_soc_write(codec, PCM1774_VOL_HPA_R, r | (reg & 0x3f));

	return 0;
}


static const struct snd_kcontrol_new pcm1774_snd_controls[] = {
	{.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .name = "Playback Volume",
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
	 SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = dac_info_volsw,
	 .get = dac_get_volsw,
	 .put = dac_put_volsw,
	 },
};


static int pcm1774_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	int l, r;

	l = snd_soc_read(codec, PCM1774_VOL_HPA_L);
	r = snd_soc_read(codec, PCM1774_VOL_HPA_R);

	if (mute) {
		l |= 0x40;
		r |= 0x40;
	} else {
		l &= ~(0x40);
		r &= ~(0x40);
	}

	snd_soc_write(codec, PCM1774_VOL_HPA_L, l);
	snd_soc_write(codec, PCM1774_VOL_HPA_R, r);

	return 0;
}


static int pcm1774_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct pcm1774_priv *pcm1774 = snd_soc_codec_get_drvdata(codec);

	int regsample, regmaster;
	u8 mstr = 0;
	u8 pfm = 0;

	pr_info("%s: fmt=%08x\n", __func__, fmt);
	pcm1774->master = 0;
	
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		pcm1774->master = 1;
		mstr = 0x1;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
	case SND_SOC_DAIFMT_CBS_CFM:
		return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		pfm = 0x3;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		break;
	case SND_SOC_DAIFMT_I2S:
		pfm = 0x0;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		pfm = 0x1;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		pfm = 0x2;
		break;
	default:
		return -EINVAL;
	}
	pcm1774->fmt = fmt & SND_SOC_DAIFMT_FORMAT_MASK;

	/* Clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_NB_IF:
	case SND_SOC_DAIFMT_IB_IF:
	case SND_SOC_DAIFMT_IB_NF:
		pr_warning("unsupported clock inversion\n");
		break;
	default:
		return -EINVAL;
	}

	pr_info("%s pfm: %d\n",__func__, pfm);

	regsample = snd_soc_read(codec, PCM1774_DAC_SAMPLE);
	regsample &= (~(0x03)) << 4;
	regsample |= (pfm << 4);

	regmaster = snd_soc_read(codec, PCM1774_MASTER_MODE);
	regmaster &= (~(0x1)) << 2;
	regmaster |= (mstr << 2);

	regmaster |= 0x01;

	snd_soc_write(codec, PCM1774_DAC_SAMPLE, regsample);
	snd_soc_write(codec, PCM1774_MASTER_MODE, regmaster);

	dump_reg(codec);

	return 0;
}

static int pcm1774_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				   int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct pcm1774_priv *pcm1774 = snd_soc_codec_get_drvdata(codec);

	pcm1774->sysclk = freq;
	return 0;

#if 0
	switch (clk_id) {
	default:
		return -EINVAL;
	}
#endif
	return 0;
}


static int pcm1774_pcm_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{

#if 0
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;

	snd_soc_write(codec, PCM1774_DAC_MUTE_L, 0x3f);
	snd_soc_write(codec, PCM1774_DAC_MUTE_R, 0x3f);

	snd_soc_write(codec, PCM1774_DAC_SAMPLE, 0x81);
	snd_soc_write(codec, PCM1774_DAC_PWR, 0xe0);
	snd_soc_write(codec, PCM1774_AMIX_PWR, 0x03);
	snd_soc_write(codec, PCM1774_AMIX_SELECT, 0x11 );
	snd_soc_write(codec, PCM1774_DAC_PWR, 0xeC);


	struct pcm1774_priv *pcm1774 = codec->private_data;
	int reg;


	reg = sgtl5000_read(codec, SGTL5000_CHIP_DIG_POWER);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		reg |= SGTL5000_I2S_IN_POWERUP;
	else
		reg |= SGTL5000_I2S_OUT_POWERUP;
	sgtl5000_write(codec, SGTL5000_CHIP_DIG_POWER, reg);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		reg = sgtl5000_read(codec, SGTL5000_CHIP_ANA_POWER);
		reg |= SGTL5000_ADC_POWERUP;
		if (sgtl5000->capture_channels == 1)
			reg &= ~SGTL5000_ADC_STEREO;
		else
			reg |= SGTL5000_ADC_STEREO;
		sgtl5000_write(codec, SGTL5000_CHIP_ANA_POWER, reg);
	}
#endif
	return 0;
}


static int pcm1774_pcm_startup(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
#if 0
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct pcm1774_priv *pcm1774 = codec->private_data;
	struct snd_pcm_runtime *master_runtime;

	if (pcm1774->master_substream) {
		master_runtime = pcm1774->master_substream->runtime;

		pr_debug("Constraining to %d bits\n",
			 master_runtime->sample_bits);

		snd_pcm_hw_constraint_minmax(substream->runtime,
					     SNDRV_PCM_HW_PARAM_SAMPLE_BITS,
					     master_runtime->sample_bits,
					     master_runtime->sample_bits);

		pcm1774->slave_substream = substream;
	} else
		pcm1774->master_substream = substream;
#endif
	return 0;
}

static void pcm1774_pcm_shutdown(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
#if 0
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct pcm1774_priv *pcm1774 = codec->private_data;

	if (pcm1774->master_substream == substream)
		pcm1774->master_substream = substream;

	pcm1774->slave_substream = NULL;
#endif
}

/*
 * Set PCM DAI bit size and sample rate.
 * input: params_rate, params_fmt
 */
static int pcm1774_pcm_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct pcm1774_priv *pcm1774 = snd_soc_codec_get_drvdata(codec);
	int channels = params_channels(params);
	int blah = 0;

	if (substream == pcm1774->slave_substream) {
		pr_info("%s ignoring params for slave substream\n", __func__);
		return 0;
	}

	pr_info("%s channels=%d, params_f: %d\n", __func__, channels, params_format(params));
	pr_info("%s params_rate: %d\n", __func__, params_rate(params));
	pr_info("%s sysclk: %d\n", __func__, pcm1774->sysclk);

	blah = pcm1774->sysclk /params_rate(params);

	pr_info("blah is %d\n",blah);


	snd_soc_write(codec, PCM1774_CLK_RATE, 0x2 << 4);
	snd_soc_write(codec, PCM1774_SYS_RESET, 0x3);

	snd_soc_write(codec, PCM1774_AOUT_CFG, 0x01);

	dump_reg(codec);

	return 0;
}

static int pcm1774_set_bias_level(struct snd_soc_codec *codec,
				   enum snd_soc_bias_level level)
{
	pr_info("dapm level %d\n", level);
	switch (level) {
	case SND_SOC_BIAS_ON:		/* full On */
		break;

	case SND_SOC_BIAS_PREPARE:	/* partial On */
		pcm1774_sync_reg_cache(codec);

		break;

	case SND_SOC_BIAS_STANDBY:	/* Off, with power */
		/* soc doesn't do PREPARE state after record so make sure
		   that anything that needs to be turned OFF gets turned off. */
		break;

	case SND_SOC_BIAS_OFF:	/* Off, without power */
		/* must power down hp/line out after vag & dac to
		   avoid pops. */
		break;
	}
	codec->dapm.bias_level = level;
	return 0;
}

#define PCM1774_RATES (SNDRV_PCM_RATE_8000 |\
		      SNDRV_PCM_RATE_11025 |\
		      SNDRV_PCM_RATE_16000 |\
		      SNDRV_PCM_RATE_22050 |\
		      SNDRV_PCM_RATE_32000 |\
		      SNDRV_PCM_RATE_44100 |\
		      SNDRV_PCM_RATE_48000 |\
		      SNDRV_PCM_RATE_96000)

//todo: revoew
#if 0
#define PCM1774_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE)
#endif

#define PCM1774_FORMATS (SNDRV_PCM_FMTBIT_S16_LE)

static struct snd_soc_dai_ops pcm1774_ops = {
	.prepare = pcm1774_pcm_prepare,
	.startup = pcm1774_pcm_startup,
	.shutdown = pcm1774_pcm_shutdown,
	.hw_params = pcm1774_pcm_hw_params,
	.digital_mute = pcm1774_digital_mute,
	.set_fmt =  pcm1774_set_dai_fmt,
	.set_sysclk = pcm1774_set_dai_sysclk
};

static struct snd_soc_dai_driver pcm1774_dai = {
	.name = "pcm1774-hifi",
	.playback = {
		     .stream_name = "Playback",
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = PCM1774_RATES,
		     .formats = PCM1774_FORMATS,
		     },
	.ops = &pcm1774_ops,
	.symmetric_rates = 1,
};
EXPORT_SYMBOL_GPL(pcm1774_dai);

static int pcm1774_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	pcm1774_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int pcm1774_resume(struct snd_soc_codec *codec)
{
	pcm1774_set_bias_level(codec, SND_SOC_BIAS_ON);

	return 0;
}


/*
 */
static int pcm1774_probe(struct snd_soc_codec *codec)
{
	struct pcm1774_priv *pcm1774 = snd_soc_codec_get_drvdata(codec);

	codec->hw_write = (hw_write_t)i2c_master_send;
	codec->control_data = pcm1774->control_data;

	pcm1774_sync_reg_cache(codec);
#if 0
	ret = snd_soc_codec_set_cache_io(codec, 8, 8, pcm1774->control_type);
	if (ret < 0 ) {
		dev_err(codec->dev, "Failed while setting I/O cache\n");
		return ret;
	}
#endif

	snd_soc_add_controls(codec, pcm1774_snd_controls,
			     ARRAY_SIZE(pcm1774_snd_controls));

	pcm1774_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	snd_soc_write(codec, PCM1774_DAC_MUTE_L, 0x3f);
	snd_soc_write(codec, PCM1774_DAC_MUTE_R, 0x3f);

	snd_soc_write(codec, PCM1774_DAC_SAMPLE, 0x81);
	snd_soc_write(codec, PCM1774_DAC_PWR, 0xe0);
	snd_soc_write(codec, PCM1774_AMIX_PWR, 0x03);
	snd_soc_write(codec, PCM1774_AMIX_SELECT, 0x11 );
	snd_soc_write(codec, PCM1774_DAC_PWR, 0xeC);

	pr_info("%s\n", __func__);

	return 0;
}

/* power down chip */
static int pcm1774_remove(struct snd_soc_codec *codec)
{
	struct pcm1774_priv *pcm1774 = snd_soc_codec_get_drvdata(codec);

	pcm1774_set_bias_level(codec, SND_SOC_BIAS_OFF);

	kfree(pcm1774);

	return 0;
}

#if 0
struct snd_soc_codec_device soc_codec_dev_pcm1774 = {
	.probe = pcm1774_probe,
	.remove = pcm1774_remove,
	.suspend = pcm1774_suspend,
	.resume = pcm1774_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_pcm1774);
#endif

struct snd_soc_codec_driver soc_codec_dev_pcm1774 = {
	.set_bias_level = pcm1774_set_bias_level,
	.reg_cache_size = sizeof(pcm1774_regs),
	.reg_word_size = sizeof(u8),
	.reg_cache_default = pcm1774_regs,
	.probe = pcm1774_probe,
	.remove = pcm1774_remove,
	.suspend = pcm1774_suspend,
	.resume = pcm1774_resume,
	.read = pcm1774_read,
	.write = pcm1774_write,
};


static __devinit int pcm1774_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct pcm1774_priv *pcm1774;
	int ret = 0;

	pr_info("%s\n", __func__);

	pcm1774 = kzalloc(sizeof(struct pcm1774_priv), GFP_KERNEL);
	if (pcm1774 == NULL) {
		dev_err(&client->dev, "Couldn't alloc for pcm1774\n");
		return -ENOMEM;
	}

	pr_info("set client data %s\n", __func__);
	i2c_set_clientdata(client, pcm1774);
	pcm1774->control_data = client;
	pcm1774->control_type = SND_SOC_I2C;

	pr_info("register client %s %p\n", __func__, &client->dev);
	ret = snd_soc_register_codec(&client->dev,
			&soc_codec_dev_pcm1774, &pcm1774_dai, 1);

	if (ret != 0) {
		dev_err(&client->dev, "Failed to register codec: %d\n", ret);
		goto err_codec_reg;
	}

	pr_info("%s\n", __func__);

	return 0;

err_codec_reg:
	kfree(pcm1774);
	return ret;
}

static __devexit int pcm1774_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));

	return 0;
}

static const struct i2c_device_id pcm1774_id[] = {
	{"pcm1774", 0x47},
	{},
};

MODULE_DEVICE_TABLE(i2c, pcm1774_id);

static struct i2c_driver pcm1774_i2c_driver = {
	.driver = {
		   .name = "pcm1774-codec",
		   .owner = THIS_MODULE,
		   },
	.probe = pcm1774_i2c_probe,
	.remove = __devexit_p(pcm1774_i2c_remove),
	.id_table = pcm1774_id,
};

static int __init pcm1774_modinit(void)
{
	return i2c_add_driver(&pcm1774_i2c_driver);
}
module_init(pcm1774_modinit);

static void __exit pcm1774_exit(void)
{
	i2c_del_driver(&pcm1774_i2c_driver);
}
module_exit(pcm1774_exit);

MODULE_DESCRIPTION("ASoC PCM1774 driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
