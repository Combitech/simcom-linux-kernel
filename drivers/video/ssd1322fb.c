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

static const unsigned char boot_img[] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};


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
	.visual =	FB_VISUAL_PSEUDOCOLOR,
	.xpanstep =	0,
	.ypanstep =	1,
	.ywrapstep =	0,
	.line_length	= 128,
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
	unsigned char *frame;
	struct work_struct redraw_work;
	int is_init;
	int init_counter;
};


static void ssd1322_write_data(struct fb_info *info, u8 *data, int len)
{
	struct ssd1322_par *par = info->par;
	gpio_set_value(par->reg_iopin, 1);
	spi_write(par->spi_dev, data, len);
}



static void ssd1322_write_command(struct fb_info *info, u8 cmd)
{
	struct ssd1322_par *par = info->par;
	gpio_set_value(par->reg_iopin, 0);
	spi_write(par->spi_dev, &cmd, 1);
}


static void ssd1322_sleep(struct fb_info *info)
{
	ssd1322_write_command(info, SLEEP_MODE_ON);
}

static void ssd1322_unsleep(struct fb_info *info)
{
	ssd1322_write_command(info, SLEEP_MODE_OFF);
}


static void ssd1322_update_display(struct work_struct *work)
{
	struct ssd1322_par *par =	container_of(work, struct ssd1322_par, redraw_work);
	int i;

	ssd1322_write_command(par->info, WRITE_RAM_COMMAND);
		for(i=0; i<(128*64); i++) {
			par->frame[i] = (par->info->screen_base[i]<<4) | (par->info->screen_base[i]>>4);
		}
		ssd1322_write_data(par->info, par->frame, 128*64);

}


static ssize_t ssd1322_write(struct fb_info *info, const char __user *buf,
			   size_t count, loff_t *ppos)
{
	unsigned long p;
	int err=-EINVAL;
	unsigned int fbmemlength;
	struct ssd1322_par *par;
	unsigned int xres;

	p = *ppos;
	par = info->par;
	par->is_init = 1;
	xres = info->var.xres;
	fbmemlength = (xres * info->var.yres)/2;

	if (p > fbmemlength)
		return -ENOSPC;

	err = 0;
	if ((count + p) > fbmemlength) {
		count = fbmemlength - p;
		err = -ENOSPC;
	}

	if (count) {
		char *base_addr;
		base_addr = (char __force *)info->screen_base;
		count -= copy_from_user(base_addr + p, buf, count);
		err = -EFAULT;
	}

	schedule_work(&par->redraw_work);

	if (count)
		return count;
	return err;
}


static int ssd1322_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct ssd1322_par *par = info->par;
	unsigned long flags;

	switch (cmd) {
		case FBIOPUT_MODEINFO:
		{
			if(arg) {
				ssd1322_sleep(info);
			}
			else {
				ssd1322_unsleep(info);
			}
			return 0;
		}
		default:
			return -EINVAL;
	}
}

static int ssd1322_open(struct fb_info *info, int user)
{
	u8 data[3];
	struct fb_image image;
	int i;
	struct ssd1322_par *par = info->par;

	ssd1322_write_command(info, SET_COMMAND_LOCK);
	data[0] = 0x12;
	ssd1322_write_data(info, &data[0], 1);

	ssd1322_write_command(info, SLEEP_MODE_ON);

	ssd1322_write_command(info, SET_FRONT_CLOCK_DIVIDER);
	data[0] = 0x91;
	ssd1322_write_data(info, &data[0], 1);

	ssd1322_write_command(info, SET_MUX_RATIO);
	data[0] = 0x3f;
	ssd1322_write_data(info, &data[0], 1);

	ssd1322_write_command(info, SET_DISPLAY_OFFSET);
	data[0] = 0x00;
	ssd1322_write_data(info, &data[0], 1);

	ssd1322_write_command(info, SET_DISPLAY_START_LINE);
	data[0] = 0x00;
	ssd1322_write_data(info, &data[0], 1);

	ssd1322_write_command(info, 0xa0);
	data[0] = 0x14;
	data[1] = 0x11;
	ssd1322_write_data(info, &data[0], 2);

	ssd1322_write_command(info, SET_GPIO);
	data[0] = 0x00;
	ssd1322_write_data(info, &data[0], 1);

	ssd1322_write_command(info, FUNCTION_SELECTION);
	data[0] = 0x01;
	ssd1322_write_data(info, &data[0], 1);

	ssd1322_write_command(info, 0xb4);
	data[0] = 0xa0;
	data[1] = 0xfd;
	ssd1322_write_data(info, &data[0], 2);

	ssd1322_write_command(info, SET_CONTRAST_CURRENT);
	data[0] = 0x9f;
	ssd1322_write_data(info, &data[0], 1);

	ssd1322_write_command(info, MASTER_CONTRAST_CURRENT);
	data[0] = 0x0f;
	ssd1322_write_data(info, &data[0], 1);

	ssd1322_write_command(info, SELECT_DEFAULT_GRAYSCALE);

	ssd1322_write_command(info, SET_PHASE_LENGTH);
	data[0] = 0xe2;
	ssd1322_write_data(info, &data[0], 1);

	ssd1322_write_command(info, 0xd1);
	data[0] = 0x82;
	data[1] = 0x20;
	ssd1322_write_data(info, &data[0], 2);

	ssd1322_write_command(info, SET_PRECHARGE_VOLTAGE);
	data[0] = 0x1f;
	ssd1322_write_data(info, &data[0], 1);

	ssd1322_write_command(info, SET_SECOND_PRCHARGE_PERIOD);
	data[0] = 0x08;
	ssd1322_write_data(info, &data[0], 1);

	ssd1322_write_command(info, SET_VCOMH);
	data[0] = 0x07;
	ssd1322_write_data(info, &data[0], 1);

	ssd1322_write_command(info, SET_DISPLAY_MODE_NORMAL);

	ssd1322_write_command(info, SLEEP_MODE_OFF);


	//! Set start line
	ssd1322_write_command(info, SET_ROW_ADDRESS);
	data[0] = 0;
	data[1] = 0x3f;
	ssd1322_write_data(info, &data[0], 2);

	//! Set the column
	ssd1322_write_command(info, SET_COLUMN_ADDRESS);
	data[0] = 0x1c;
	data[1] = 0x5b;
	ssd1322_write_data(info, &data[0], 2);

	image.data = &boot_img;
	image.depth = 8;
	image.dx = 124;
	image.dy = 28;
	image.height = 8;
	image.width = 8;
	sys_imageblit(info, &image);

	schedule_work(&par->redraw_work);

	return 0;
}

static int ssd1322_release(struct fb_info *info, int user)
{
	return 0;
}



static void ssd1322_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{

	struct ssd1322_par *par = info->par;
	//sys_fillrect(info, rect);
	//schedule_work(&par->redraw_work);
}

static void ssd1322_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{

	struct ssd1322_par *par = info->par;
	//sys_copyarea(info, area);
	//schedule_work(&par->redraw_work);
}

static void ssd1322_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct ssd1322_par *par = info->par;
	//sys_imageblit(info, image);
	//schedule_work(&par->redraw_work);
}


static struct fb_ops ssd1322_ops = {
	.owner			= THIS_MODULE,
	.fb_open		= ssd1322_open,
	.fb_release		= ssd1322_release,
	.fb_write		= ssd1322_write,
	.fb_fillrect	= ssd1322_fillrect,
	.fb_copyarea	= ssd1322_copyarea,
	.fb_imageblit	= ssd1322_imageblit,
	.fb_ioctl		= ssd1322_ioctl,
};


static int __devinit ssd1322_probe(struct spi_device *spi)
{
	struct fb_info *info;
	int retval = -ENOMEM;
	struct ssd1322_par *par;
	struct ssd1322_spi_platform_data *pdata = spi->dev.platform_data;


	printk("Probing ssd1322 display\n");

	info = framebuffer_alloc(sizeof(struct ssd1322_par), &spi->dev);
	if (!info)
		goto err;


	info->screen_base = kmalloc(256*64, GFP_KERNEL);
	memset(info->screen_base, 0, 256*64);
	info->fbops = &ssd1322_ops;
	info->fix = ssd1322fb_fix;
	info->var = ssd1322fb_var;
	info->flags = FBINFO_FLAG_DEFAULT;

	par = info->par;
	par->info = info;
	par->reg_iopin = pdata->reg_gpio;
	par->reset_iopin = pdata->reset_gpio;
	par->frame = kmalloc(256*64, GFP_KERNEL);
	memset(par->frame, 0, 256*64);
	par->spi_dev = spi;
	par->is_init = 0;

	INIT_WORK(&par->redraw_work, ssd1322_update_display);


	gpio_request(par->reg_iopin, "ssd1322fb_cmd_data");
	gpio_direction_output(par->reg_iopin, 0);
	gpio_set_value(par->reg_iopin, 0);

	retval = register_framebuffer(info);
	dev_set_drvdata(&spi->dev, info);

	return 0;
err:
	framebuffer_release(info);
	kfree(info->screen_base);
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

