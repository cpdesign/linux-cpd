/*
 *  pcm1774.h - PCM1774 audio codec interface
 *
 *  Copyright (C) 2010 Creative Poduct Design.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef _PCM1774_H
#define _PCM1774_H

#include <linux/i2c.h>


#define PCM1774_VOL_HPA_L	0x40
#define PCM1774_VOL_HPA_R	0x41
#define PCM1774_DAC_MUTE_L	0x44
#define PCM1774_DAC_MUTE_R	0x45
#define PCM1774_DAC_SAMPLE	0x46
#define PCM1774_AMIX_PWR	0x48
#define PCM1774_DAC_PWR		0x49
#define PCM1774_AOUT_CFG	0x4A
#define PCM1774_HPA_DETECT	0x4B
#define PCM1774_STATUS		0x4D
#define PCM1774_PG_PWR		0x52
#define PCM1774_MASTER_MODE	0x54
#define PCM1774_SYS_RESET	0x55
#define PCM1774_CLK_RATE	0x56
#define PCM1774_AIN_SELECT	0x57
#define PCM1774_AMIX_SELECT	0x58
#define PCM1774_APATH_GAIN	0x59
#define PCM1774_MIC_BOOST	0x5A
#define PCM1774_BASS_GAIN	0x5C
#define PCM1774_MID_GAIN	0x5D
#define PCM1774_TREB_GAIN	0x5E
#define PCM1774_3D_SOUND	0x5F
#define PCM1774_MONO_MIX	0x60
#define PCM1774_PG_GAIN		0x7C
#define PCM1774_PWR_TIME_CTRL	0x7D

#endif
