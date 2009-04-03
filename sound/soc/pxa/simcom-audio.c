/*
 * linux/sound/soc/pxa/simcom.c
 *
 * Copyright (C) 2009 Combitech AB
 * Tobias Knutsson <tobias.knutsson(at)combitech.se>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/audio.h>

#include "../codecs/tlv320aic3x.h"
#include "pxa2xx-pcm.h"
#include "pxa2xx-i2s.h"

struct aic3x_setup_data aic3x_data = {
	.i2c_bus = 0,
	.i2c_address = 0x18,
	.gpio_func = { 0 , 0 },
};

static struct snd_soc_dai_link simcom_dai = {
		.name = "I2S",
		.stream_name = "I2S",
		.cpu_dai = &pxa_i2s_dai,
		.codec_dai = &aic3x_dai,
};

static struct snd_soc_card simcom_audio = {
	.name = "I2S",
	.platform = &pxa2xx_soc_platform,
	.dai_link = &simcom_dai,
	.num_links = 1,
};

static struct snd_soc_device simcom_snd_devdata = {
	.card = &simcom_audio,
	.codec_dev = &soc_codec_dev_aic3x,
	.codec_data = &aic3x_data,
};

static struct platform_device *simcom_snd_device;

static int __init simcom_audio_init(void)
{
	int ret;

	if (!machine_is_simcom())
		return -ENODEV;

	simcom_snd_device = platform_device_alloc("soc-audio", -1);
	if (!simcom_snd_device)
		return -ENOMEM;

	platform_set_drvdata(simcom_snd_device, &simcom_snd_devdata);
	simcom_snd_devdata.dev = &simcom_snd_device->dev;
	ret = platform_device_add(simcom_snd_device);

	if (ret)
		platform_device_put(simcom_snd_device);

	return ret;
}

static void __exit simcom_audio_exit(void)
{
	platform_device_unregister(simcom_snd_device);
}

module_init(simcom_audio_init);
module_exit(simcom_audio_exit);

MODULE_AUTHOR("Tobias Knutsson");
MODULE_DESCRIPTION("ALSA SoC SimCom");
MODULE_LICENSE("GPL");
