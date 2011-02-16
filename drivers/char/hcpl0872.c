/*
 *  hcpl0872.c - Agilent HCPL-0872 Digital Interface IC
 *
 *  Copyright (C) 2010 Combitech AB
 *  Marcus Folkesson <marcus.folkesson@combitech.se>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 */



#include <linux/module.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <mach/ssp.h>
#include <mach/regs-ssp.h>
#include <linux/platform_device.h>

MODULE_DESCRIPTION("Agilent HCPL-0872 Digital Interface IC");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marcus Folkesson <marcus.folkesson@combitech.se>");
MODULE_ALIAS("spi:hcpl0872");


struct hcpl_priv {
	struct ssp_dev *spi_dev;
	int cs_gpio;
	struct cdev cdev;
	struct device *device;
	dev_t dev;
	int channel_gpio;
	struct hcpl0872_spi_platform_data *pdata;
};



struct class *hcpl_class;

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



static ssize_t hcpl_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct hcpl_priv *priv = file->private_data;
	u8 rx[4];

	gpio_set_value(priv->cs_gpio, 0);

	/* Read channel 0 */
	gpio_set_value(priv->channel_gpio, 0);
	spi_read_byte(priv->spi_dev, 0xff, &rx[0]);
	spi_read_byte(priv->spi_dev, 0xff, &rx[1]);

	/* Read channel 1 */
	gpio_set_value(priv->channel_gpio, 1);
	spi_read_byte(priv->spi_dev, 0xff, &rx[2]);
	spi_read_byte(priv->spi_dev, 0xff, &rx[3]);

	gpio_set_value(priv->cs_gpio, 1);


	printk("Read 0x%02x 0x%02x 0x%02x 0x%02x \n", rx[0], rx[1],rx[2],rx[3]);
	return 0;
	copy_to_user(buf, rx, 4);

	return 4;
}



static ssize_t hcpl_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	struct hcpl_priv *priv = file->private_data;

	return count;
}


static int hcpl_open(struct inode *inode, struct file *file)
{
	struct hcpl_priv *priv = container_of(inode->i_cdev, struct hcpl_priv, cdev);

	file->private_data = priv;

	return 0;
}

static int hcpl_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations hcpl_ops = {
	.owner		= THIS_MODULE,
	.read		= hcpl_read,
	.write		= hcpl_write,
	.open		= hcpl_open,
	.release	= hcpl_release,
};

static int __devinit hcpl_probe(struct platform_device *pdev)
{
	struct hcpl_priv *priv;
	int err;
	struct resource *r;

	dev_info(&pdev->dev, "Probing hcpl..\n");

	priv = kzalloc(sizeof(struct hcpl_priv), GFP_KERNEL);

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

	if(gpio_request(priv->cs_gpio, "HCPL chipselect") < 0) {
		printk("Could not request chipselect pin for HCPL\n");
		goto exit_free;
	}

	gpio_direction_output(priv->cs_gpio, 1);
	gpio_set_value(priv->cs_gpio, 1);


	/* Get channel select signal from platform resource */
	r = platform_get_resource(pdev, IORESOURCE_IO, 1);
	priv->channel_gpio = r->start;
	printk("Channel select on gpio%i\n", priv->channel_gpio);

	if(gpio_request(priv->channel_gpio, "HCPL Channel select") < 0) {
		printk("Could not request channel select pin for HCPL\n");
		goto exit_free;
	}

	gpio_direction_output(priv->channel_gpio, 1);
	gpio_set_value(priv->channel_gpio, 1);




	alloc_chrdev_region(&priv->dev, 0, 1, "hcpl");
	cdev_init(&priv->cdev, &hcpl_ops);
	priv->cdev.owner = THIS_MODULE;
	priv->cdev.ops = &hcpl_ops;

	cdev_add(&priv->cdev, priv->dev, 1);
	priv->device = device_create(hcpl_class, NULL, priv->dev, NULL, "hcpl");

	dev_info(&pdev->dev, "hcpl device registered\n");

	return 0;

exit_free:
	kzfree(priv);
	return err;
}




static int __devexit hcpl_remove(struct platform_device *pdev)
{
	struct hcpl_priv *priv = dev_get_drvdata(&pdev->dev);

	device_del(priv->device);
	cdev_del(&priv->cdev);
	unregister_chrdev_region(priv->dev, 1);

	return 0;
}

static struct platform_driver hcpl_driver = {
	.probe	= hcpl_probe,
	.remove	= hcpl_remove,
	.driver = {
		.name	= "hcpl0872",
		.owner	= THIS_MODULE,
	},
};

static __init int hcpl_init_module(void)
{
	hcpl_class = class_create(THIS_MODULE, "hcpl");

	if(hcpl_class == NULL) {
		printk("Could not create class hcpl\n");
		return -ENOMEM;
	}

	return platform_driver_register(&hcpl_driver);
}

static __exit void hcpl_cleanup_module(void)
{
	platform_driver_unregister(&hcpl_driver);
}

module_init(hcpl_init_module);
module_exit(hcpl_cleanup_module);

MODULE_ALIAS("spi:hcpl");

