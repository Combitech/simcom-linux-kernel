
/*
 *  pic_pwm.c
 *
 *  Copyright (C) 2010 Combitech AB
 *  Marcus Folkesson <marcus.folkesson@combitech.se>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <mach/ssp.h>
#include <mach/regs-ssp.h>
#include <linux/platform_device.h>


MODULE_DESCRIPTION("Driver for a PIC Microcontroller connected to Nacelle used for PWM measurement");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marcus Folkesson <marcus.folkesson@combitech.se>");
MODULE_ALIAS("spi:pic_pwm");

/* Command to recieve measures */
#define	CMD_GET 0xAB


struct pic_priv {
	struct ssp_dev *spi_dev;
	int cs_gpio;
	struct cdev cdev;
	struct device *device;
	dev_t dev;
};



struct class *pic_class;

static int spi_read_byte(struct ssp_dev *dev, u8 output, u8 *data)
{
	u32 d = 0;

	ssp_write_word(dev, output);
	ssp_read_word(dev, &d);
	ssp_flush(dev);
	*data = d;
	return 0;
}

static int spi_write_byte(struct ssp_dev *dev, u8 data)
{
	ssp_write_word(dev, data);
	ssp_flush(dev);
	return 0;
}



static ssize_t pic_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{

	u8 rx[5];
	u8 tx[1];

	struct pic_priv *priv = file->private_data;

	if(count < 4) {
		return 0;
	}


	gpio_set_value(priv->cs_gpio, 0);
	spi_write_byte(&priv->spi_dev, CMD_GET);
	//spi_read_byte(&priv->spi_dev, CMD_GET, &rx[0]);
	spi_read_byte(&priv->spi_dev, 0xff, &rx[0]);	/* The pic isnt fast enough? */
	spi_read_byte(&priv->spi_dev, 0xff, &rx[0]);
	spi_read_byte(&priv->spi_dev, 0xff, &rx[1]);
	spi_read_byte(&priv->spi_dev, 0xff, &rx[2]);
	spi_read_byte(&priv->spi_dev, 0xff, &rx[3]);
	spi_read_byte(&priv->spi_dev, 0xff, &rx[4]);

	gpio_set_value(priv->cs_gpio, 1);

	printk("Read from pic: 0x%x 0x%x 0x%x 0x%x 0x%x\n", rx[0],rx[1],rx[2],rx[3],rx[4]);

	if(copy_to_user(buf, rx, 5)) {
		return 0;
	}

	return 5;
}



static ssize_t pic_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	struct pic_priv *priv = file->private_data;




	return count;
}


static int pic_open(struct inode *inode, struct file *file)
{
	struct pic_priv *priv = container_of(inode->i_cdev, struct pic_priv, cdev);

	file->private_data = priv;

	return 0;
}

static int pic_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations pic_ops = {
	.owner		= THIS_MODULE,
	.read		= pic_read,
	.write		= pic_write,
	.open		= pic_open,
	.release	= pic_release,
};

static int __devinit pic_probe(struct platform_device *pdev)
{
	struct pic_priv *priv;
	int err;
	struct resource *r;

	dev_info(&pdev->dev, "Probing pic..\n");

	priv = kzalloc(sizeof(struct pic_priv), GFP_KERNEL);

	if (!priv) {
		err = -ENOMEM;
		goto exit_free;
	}

	/* Initialize spi and read device id */
	dev_set_drvdata(&pdev->dev, priv);

	if(ssp_init(&priv->spi_dev, 3, SSP_NO_IRQ) == -ENODEV) {
		printk("Could not allocate device\n");
		goto exit_free;
	}

	ssp_disable(&priv->spi_dev);

	ssp_config(&priv->spi_dev, 	SSCR0_DataSize(8) | SSCR0_Motorola,
			SSCR1_TxTresh(1) | SSCR1_RxTresh(1) | SSCR1_SPO | SSCR1_SPH,
			0,
			SSCR0_SCR & SSCR0_SerClkDiv(128));

	ssp_enable(&priv->spi_dev);


	/* Get chip select signal from platform resource */
	r = platform_get_resource(pdev, IORESOURCE_IO, 0);
	priv->cs_gpio = r->start;
	printk("Chipselect on gpio%i\n", priv->cs_gpio);

	if(gpio_request(priv->cs_gpio, "PIC chipselect") < 0) {
		printk("Could not request chipselect pin for PIC\n");
		goto exit_free;
	}

	gpio_direction_output(priv->cs_gpio, 1);
	gpio_set_value(priv->cs_gpio, 1);



	alloc_chrdev_region(&priv->dev, 0, 1, "pic");
	cdev_init(&priv->cdev, &pic_ops);
	priv->cdev.owner = THIS_MODULE;
	priv->cdev.ops = &pic_ops;

	cdev_add(&priv->cdev, priv->dev, 1);
	priv->device = device_create(pic_class, NULL, priv->dev, NULL, "pic");

	dev_info(&pdev->dev, "pic device registered\n");

	return 0;

exit_free:
	kzfree(priv);
	return err;
}




static int __devexit pic_remove(struct platform_device *pdev)
{
	struct pic_priv *priv = dev_get_drvdata(&pdev->dev);

	device_del(priv->device);
	cdev_del(&priv->cdev);
	unregister_chrdev_region(priv->dev, 1);

	return 0;
}

static struct platform_driver pic_driver = {
	.probe	= pic_probe,
	.remove	= pic_remove,
	.driver = {
		.name	= "pic_pwm",
		.owner	= THIS_MODULE,
	},
};

static __init int pic_init_module(void)
{
	pic_class = class_create(THIS_MODULE, "pic");

	if(pic_class == NULL) {
		printk("Could not create class pic\n");
		return -ENOMEM;
	}

	return platform_driver_register(&pic_driver);
}

static __exit void pic_cleanup_module(void)
{
	platform_driver_unregister(&pic_driver);
}

module_init(pic_init_module);
module_exit(pic_cleanup_module);

MODULE_ALIAS("spi:pic");

