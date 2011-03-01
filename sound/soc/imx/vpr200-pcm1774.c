/*
 * imx-3stack-sgtl5000.c  --  i.MX 3Stack Driver for Freescale SGTL5000 Codec
 *
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    21th Oct 2008   Initial version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include <mach/dma.h>
#include <mach/clock.h>

#include "../codecs/pcm1774.h"
#include "imx-ssi.h"


/* SSI BCLK and LRC master */
#define PCM1774_SSI_MASTER	1

struct imx_vpr200_priv {
	int sysclk;
	int hw;
	struct platform_device *pdev;
};

static struct imx_vpr200_priv card_priv;

static int imx_vpr200_audio_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct imx_vpr200_priv *priv = &card_priv;
	unsigned int rate = params_rate(params);
//	struct imx_ssi *ssi_mode = (struct imx_ssi *)cpu_dai->private_data;
	int ret = 0;

	unsigned int channels = params_channels(params);
	u32 dai_format;

	/* only need to do this once as capture and playback are sync */
	if (priv->hw)
		return 0;
	priv->hw = 1;

	priv->sysclk = 12000000;

	snd_soc_dai_set_sysclk(codec_dai, 0, priv->sysclk, 0);

#if PCM1774_SSI_MASTER
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBM_CFM;
#else
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBS_CFS;
#endif
/*
	ssi_mode->sync_mode = 1;
	if (channels == 1)
		ssi_mode->network_mode = 0;
	else
		ssi_mode->network_mode = 1;
*/

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 2,
				 0);

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set the SSI system clock as input (unused) */
	snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0, SND_SOC_CLOCK_IN);

	return 0;
}

static int imx_vpr200_startup(struct snd_pcm_substream *substream)
{

	return 0;
}

static void imx_vpr200_shutdown(struct snd_pcm_substream *substream)
{
	struct imx_vpr200_priv *priv = &card_priv;



	priv->hw = 0;
}

/*
 * imx_3stack SGTL5000 audio DAI opserations.
 */
static struct snd_soc_ops imx_vpr200_ops = {
	.startup = imx_vpr200_startup,
	.shutdown = imx_vpr200_shutdown,
	.hw_params = imx_vpr200_audio_hw_params,
};

#if 0
static void imx_vpr200_init_dam(int ssi_port, int dai_port)
{
	unsigned int ssi_ptcr = 0;
	unsigned int dai_ptcr = 0;
	unsigned int ssi_pdcr = 0;
	unsigned int dai_pdcr = 0;
	/* SGTL5000 uses SSI1 or SSI2 via AUDMUX port dai_port for audio */

	/* reset port ssi_port & dai_port */
	__raw_writel(0, DAM_PTCR(ssi_port));
	__raw_writel(0, DAM_PTCR(dai_port));
	__raw_writel(0, DAM_PDCR(ssi_port));
	__raw_writel(0, DAM_PDCR(dai_port));

	pr_info("ssi_port: %d, dai_port: %d\n", ssi_port, dai_port);

	/* set to synchronous */
	ssi_ptcr |= AUDMUX_PTCR_SYN;
	dai_ptcr |= AUDMUX_PTCR_SYN;

#if PCM1774_SSI_MASTER
	/* set Rx sources ssi_port <--> dai_port */
	ssi_pdcr |= AUDMUX_PDCR_RXDSEL(dai_port);
	dai_pdcr |= AUDMUX_PDCR_RXDSEL(ssi_port);

	/* set Tx frame direction and source  dai_port--> ssi_port output */
	ssi_ptcr |= AUDMUX_PTCR_TFSDIR;
	ssi_ptcr |= AUDMUX_PTCR_TFSSEL(AUDMUX_FROM_TXFS, dai_port);

	/* set Tx Clock direction and source dai_port--> ssi_port output */
	ssi_ptcr |= AUDMUX_PTCR_TCLKDIR;
	ssi_ptcr |= AUDMUX_PTCR_TCSEL(AUDMUX_FROM_TXFS, dai_port);
#else
	/* set Rx sources ssi_port <--> dai_port */
	ssi_pdcr |= AUDMUX_PDCR_RXDSEL(dai_port);
	dai_pdcr |= AUDMUX_PDCR_RXDSEL(ssi_port);

	/* set Tx frame direction and source  ssi_port --> dai_port output */
	dai_ptcr |= AUDMUX_PTCR_TFSDIR;
	dai_ptcr |= AUDMUX_PTCR_TFSSEL(AUDMUX_FROM_TXFS, ssi_port);

	/* set Tx Clock direction and source ssi_port--> dai_port output */
	dai_ptcr |= AUDMUX_PTCR_TCLKDIR;
	dai_ptcr |= AUDMUX_PTCR_TCSEL(AUDMUX_FROM_TXFS, ssi_port);
#endif

	pr_info("ssi_ptcr: 0x%08x\n", ssi_ptcr);
	pr_info("ssi_pdcr: 0x%08x\n", ssi_pdcr);
	pr_info("dai_ptcr: 0x%08x\n", dai_ptcr);
	pr_info("dai_pdcr: 0x%08x\n", dai_pdcr);

	__raw_writel(ssi_ptcr, DAM_PTCR(ssi_port));
	__raw_writel(dai_ptcr, DAM_PTCR(dai_port));
	__raw_writel(ssi_pdcr, DAM_PDCR(ssi_port));
	__raw_writel(dai_pdcr, DAM_PDCR(dai_port));
}

#endif

/* imx_3stack machine connections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
	{"HP_R", NULL, "DACR"},

};



static int imx_vpr200_pcm1774_init(struct snd_soc_pcm_runtime *rtd)
{
	int i, ret;

	pr_info("%s\n", __func__);
#if 0
	/* Add imx_3stack specific controls */
	for (i = 0; i < ARRAY_SIZE(pcm1774_machine_controls); i++) {
		ret = snd_ctl_add(codec->card,
				  snd_soc_cnew(&pcm1774_machine_controls[i],
					       codec, NULL));
		if (ret < 0)
			return ret;
	}

	/* Add imx_3stack specific widgets */
	snd_soc_dapm_new_controls(codec, imx_vpr200_dapm_widgets,
				  ARRAY_SIZE(imx_vpr200_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_disable_pin(codec, "Line In Jack");

	snd_soc_dapm_sync(codec);

#endif
	return 0;
}

/* imx_3stack digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link imx_vpr200_dai = {
	.name = "pcm1774",
	.stream_name = "PCM1774",
	.codec_dai_name = "pcm1774-hifi",
	.platform_name = "imx-fiq-pcm-audio.0",
	.codec_name = "pcm1774-codec.1-0047",
	.cpu_dai_name = "imx-ssi.0",
	.init = imx_vpr200_pcm1774_init,
	.ops = &imx_vpr200_ops,
};

static struct snd_soc_card snd_soc_card_imx_vpr200 = {
	.name = "imx-vpr200",
	.dai_link = &imx_vpr200_dai,
	.num_links = 1,
};

#if 0
static int __devinit imx_vpr200_pcm1774_probe(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	struct imx_vpr200_priv *priv = &card_priv;
	struct snd_soc_dai *pcm1774_cpu_dai;
	int ret = 0;

	priv->pdev = pdev;

	gpio_activate_audio_ports();
	imx_vpr200_init_dam(plat->src_port, plat->ext_port);

	if (plat->src_port == 2)
		pcm1774_cpu_dai = imx_ssi_dai[2];
	else
		pcm1774_cpu_dai = imx_ssi_dai[0];

	imx_vpr200_dai.cpu_dai = pcm1774_cpu_dai;

	ret = -EINVAL;
	if (plat->init && plat->init())
		goto err_plat_init;

	priv->sysclk = plat->sysclk;

	return 0;

err_card_reg:
	if (plat->finit)
		plat->finit();
err_plat_init:
sysfs_err:
	return ret;
}

static int imx_vpr200_pcm1774_remove(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	struct imx_vpr200_priv *priv = &card_priv;

	if (plat->finit)
		plat->finit();


	return 0;
}

static struct platform_driver imx_vpr200_pcm1774_audio_driver = {
	.probe = imx_vpr200_pcm1774_probe,
	.remove = imx_vpr200_pcm1774_remove,
	.driver = {
		   .name = "imx-vpr200-pcm1774",
		   },
};
#endif


static struct platform_device *imx_vpr200_snd_device;

static int __init imx_vpr200_init(void)
{
	int ret;

	pr_info("%s\n", __func__ );

	imx_vpr200_snd_device = platform_device_alloc("soc-audio", 2);
	if (!imx_vpr200_snd_device)
		return -ENOMEM;

	platform_set_drvdata(imx_vpr200_snd_device, &snd_soc_card_imx_vpr200);
	ret = platform_device_add(imx_vpr200_snd_device);

	if (ret) {
		printk(KERN_ERR "ASoC: vpr200 device alloc failed\n");
		platform_device_put(imx_vpr200_snd_device);
	}

	pr_info("ret : %d\n", ret);

	return ret;
}

static void __exit imx_vpr200_exit(void)
{
	platform_device_unregister(imx_vpr200_snd_device);
}

module_init(imx_vpr200_init);
module_exit(imx_vpr200_exit);

MODULE_AUTHOR("Creative Product Design");
MODULE_DESCRIPTION("PCM1774 Driver for VPR200");
MODULE_LICENSE("GPL");
