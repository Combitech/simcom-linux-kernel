/*
 * linux/drivers/video/ssd1322.c -- FB driver for Arc monochrome LCD board
 *
 * Copyright (C) 2005, Jaya Kumar <jayalk@intworks.biz>
 * http://www.intworks.biz/arclcd
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/ssd1322.h>
#include <linux/gpio.h>

#include <linux/fb.h>
#include <linux/init.h>
#include <linux/uaccess.h>




#define ENABLE_GRAYSCALE_TABLE		0x00
#define SET_COLUMN_ADDRESS			0x15
#define WRITE_RAM_COMMAND			0x5c
#define READ_RAM_COMMAND			0x5d
#define SET_ROW_ADDRESS				0x75
#define SET_DISPLAY_START_LINE		0xa1
#define SET_DISPLAY_OFFSET			0xa2
#define SET_DISPLAY_MODE_OFF		0xa4
#define SET_DISPLAY_MODE_ON			0xa5
#define SET_DISPLAY_MODE_NORMAL		0xa6
#define SET_DISPLAY_MODE_INVERSE	0xa7
#define ENABLE_PARTIAL_DISPLAY		0xa8
#define EXIT_PARTIAL_DISPLAY		0xa9
#define FUNCTION_SELECTION			0xab
#define SLEEP_MODE_ON				0xae
#define SLEEP_MODE_OFF				0xaf
#define SET_PHASE_LENGTH			0xb1
#define SET_FRONT_CLOCK_DIVIDER		0xb3
#define SET_GPIO					0xb5
#define SET_SECOND_PRCHARGE_PERIOD	0xb6
#define SET_GRAY_SCALE_TABLE		0xb8
#define SELECT_DEFAULT_GRAYSCALE	0xb9
#define SET_PRECHARGE_VOLTAGE		0xbb
#define SET_VCOMH					0xbe
#define SET_CONTRAST_CURRENT		0xc1
#define MASTER_CONTRAST_CURRENT		0xc7
#define SET_MUX_RATIO				0xca
#define SET_COMMAND_LOCK			0xfd



static struct fb_fix_screeninfo ssd1322fb_fix __devinitdata = {
	.id =		"ssd1322fb",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.xpanstep =	0,
	.ypanstep =	1,
	.ywrapstep =	0,
	.accel =	FB_ACCEL_NONE,
};

static struct fb_var_screeninfo ssd1322fb_var __devinitdata = {
	.xres			= 256,
	.yres			= 64,
	.xres_virtual	= 256,
	.yres_virtual	= 64,
	.bits_per_pixel	= 4,
	.nonstd			= 1,
	.grayscale 		= 1,
};

struct ssd1322_par {
	struct fb_info *info;
	int reg_iopin;
	int reset_iopin;
	struct spi_device *spi_dev;
};


static void ssd1322_write_data(struct fb_info *info, u8 *data, int len)
{
	struct ssd1322_par *par = info->par;
	gpio_set_value(par->reg_iopin, 1);
	spi_write(par->spi_dev, data, len);
}

static void ssd1322_read_data(struct fb_info *info, u8 *data, int len)
{
	struct ssd1322_par *par = info->par;
	gpio_set_value(par->reg_iopin, 1);
	spi_read(par->spi_dev, data, len);
}


static void ssd1322_write_command(struct fb_info *info, u8 cmd)
{
	struct ssd1322_par *par = info->par;
	gpio_set_value(par->reg_iopin, 0);
	spi_write(par->spi_dev, &cmd, 1);
}

static void ssd1322_set_draw_area(struct fb_info *info, u8 x, u8 y, u8 width, u8 height)
{


}


static int ssd1322_open(struct fb_info *info, int user)
{
	u8 data;
	printk("ssd1322_open\n");
	ssd1322_write_command(info, SET_COMMAND_LOCK);
	data = 0x12;
	ssd1322_write_data(info, &data, 1);

	ssd1322_write_command(info, SLEEP_MODE_ON);

	ssd1322_write_command(info, SET_FRONT_CLOCK_DIVIDER);
	data = 0x91;
	ssd1322_write_data(info, &data, 1);

	ssd1322_write_command(info, SET_MUX_RATIO);
	data = 0x3f;
	ssd1322_write_data(info, &data, 1);

	ssd1322_write_command(info, SET_DISPLAY_OFFSET);
	data = 0x00;
	ssd1322_write_data(info, &data, 1);

	ssd1322_write_command(info, SET_DISPLAY_START_LINE);
	data = 0x00;
	ssd1322_write_data(info, &data, 1);

	ssd1322_write_command(info, 0xa0);
	data = 0x14;
	ssd1322_write_data(info, &data, 1);
	data = 0x11;
	ssd1322_write_data(info, &data, 1);

	ssd1322_write_command(info, SET_GPIO);
	data = 0x00;
	ssd1322_write_data(info, &data, 1);

	ssd1322_write_command(info, FUNCTION_SELECTION);
	data = 0x01;
	ssd1322_write_data(info, &data, 1);

	ssd1322_write_command(info, 0xb4);
	data = 0xa0;
	ssd1322_write_data(info, &data, 1);
	data = 0xfd;
	ssd1322_write_data(info, &data, 1);

	ssd1322_write_command(info, SET_CONTRAST_CURRENT);
	data = 0x9f;
	ssd1322_write_data(info, &data, 1);

	ssd1322_write_command(info, MASTER_CONTRAST_CURRENT);
	data = 0x0f;
	ssd1322_write_data(info, &data, 1);

	ssd1322_write_command(info, SELECT_DEFAULT_GRAYSCALE);

	ssd1322_write_command(info, SET_PHASE_LENGTH);
	data = 0xe2;
	ssd1322_write_data(info, &data, 1);

	ssd1322_write_command(info, 0xd1);
	data = 0x82;
	ssd1322_write_data(info, &data, 1);
	data = 0x20;
	ssd1322_write_data(info, &data, 1);

	ssd1322_write_command(info, SET_PRECHARGE_VOLTAGE);
	data = 0x1f;
	ssd1322_write_data(info, &data, 1);

	ssd1322_write_command(info, SET_SECOND_PRCHARGE_PERIOD);
	data = 0x08;
	ssd1322_write_data(info, &data, 1);

	ssd1322_write_command(info, SET_VCOMH);
	data = 0x07;
	ssd1322_write_data(info, &data, 1);

	ssd1322_write_command(info, SET_DISPLAY_MODE_NORMAL);

	ssd1322_write_command(info, SLEEP_MODE_OFF);


	return 0;
}

static int ssd1322_release(struct fb_info *info, int user)
{
	return 0;
}



static void ssd1322_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	/* Setup drawing area and fill */
	ssd1322_set_draw_area(info, rect->dx, rect->dy, rect->width, rect->height);
	ssd1322_write_command(info, WRITE_RAM_COMMAND);
	ssd1322_write_data(info, (u8 *)&rect->color, rect->width*rect->height);
}

static void ssd1322_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{


}

static void ssd1322_imageblit(struct fb_info *info, const struct fb_image *image)
{
	/* Setup drawing area and fill */
	ssd1322_set_draw_area(info, image->dx, image->dy, image->width, image->height);
	ssd1322_write_command(info, WRITE_RAM_COMMAND);
	ssd1322_write_data(info, (u8 *)image->data, image->width*image->height);
}





static struct fb_ops ssd1322_ops = {
	.owner			= THIS_MODULE,
	.fb_open		= ssd1322_open,
	.fb_release		= ssd1322_release,
	.fb_fillrect	= ssd1322_fillrect,
	.fb_copyarea	= ssd1322_copyarea,
	.fb_imageblit	= ssd1322_imageblit,
};

static int __devinit ssd1322_probe(struct spi_device *spi)
{
	struct fb_info *info;
	int retval = -ENOMEM;
	int videomemorysize;
	unsigned char *videomemory;
	struct ssd1322_par *par;
	struct ssd1322_spi_platform_data *pdata = spi->dev.platform_data;

	videomemorysize = 256*64 / 2;

	if (!(videomemory = vmalloc(videomemorysize)))
		return retval;

	printk("Probing ssd1322 display\n");

	memset(videomemory, 0, videomemorysize);

	info = framebuffer_alloc(sizeof(struct ssd1322_par), &spi->dev);
	if (!info)
		goto err;

	//framebuffer_priv(info)
	info->screen_base = (char __iomem *)videomemory;
	info->fbops = &ssd1322_ops;
	info->fix = ssd1322fb_fix;
	info->var = ssd1322fb_var;
	info->flags = FBINFO_FLAG_DEFAULT;

	par = info->par;
	par->info = info;
	par->reg_iopin = pdata->reg_iopin;
	par->reset_iopin = pdata->reset_iopin;
	par->spi_dev = spi;

	gpio_request(par->reg_iopin, "cmd_data_pin");
	gpio_request(par->reset_iopin, "reset_pin");
	gpio_direction_output(par->reg_iopin, 0);
	gpio_direction_output(par->reset_iopin, 1);
	gpio_set_value(par->reg_iopin, 0);
	gpio_set_value(par->reset_iopin, 1);

	retval = register_framebuffer(info);
	dev_set_drvdata(&spi->dev, info);

	return 0;
err:
	framebuffer_release(info);
	vfree(videomemory);
	return retval;
}


static int __devexit ssd1322_remove(struct spi_device *spi)
{

	return 0;
}

static struct spi_driver ssd1322_driver = {
	.probe	= ssd1322_probe,
	.remove	= ssd1322_remove,
	.driver = {
		.name	= "ssd1322fb",
		.owner	= THIS_MODULE,
	},
};

static __init int ssd1322_init_module(void)
{
	return spi_register_driver(&ssd1322_driver);
}

static __exit void ssd1322_cleanup_module(void)
{
	spi_unregister_driver(&ssd1322_driver);
}

module_init(ssd1322_init_module);
module_exit(ssd1322_cleanup_module);

MODULE_DESCRIPTION("fbdev driver for SSD1322 OLED Display");
MODULE_AUTHOR("David Kiland Combitech AB <david.kiland@combitech.se>");
MODULE_LICENSE("GPL");

