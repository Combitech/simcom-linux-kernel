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
#include <linux/spi/hcpl0872.h>

MODULE_DESCRIPTION("Agilent HCPL-0872 Digital Interface IC");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marcus Folkesson <marcus.folkesson@combitech.se>");
MODULE_ALIAS("spi:hcpl0872");


struct hcpl0872_priv {
	struct spi_device *spi_dev;
	int cs_gpio;
	struct cdev cdev;
	struct device *device;
	dev_t dev;

	struct hcpl0872_spi_platform_data *pdata;
};

static struct class *hcpl0872_class = 0;


static ssize_t hcpl0872_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct hcpl0872_priv *priv = file->private_data;
	char rx[4];

	if( count < 4)
	{
		 return 0;
	}

	/* Read channel 0 */
	gpio_set_value(priv->pdata->channel_gpio, 0);
	spi_read(priv->spi_dev, rx + 0, 2);

	/* Read channel 1 */
	gpio_set_value(priv->pdata->channel_gpio, 1);
	spi_read(priv->spi_dev, rx + 2, 2);

	copy_to_user(buf, rx, 4);


	return 4;
}


static int hcpl0872_open(struct inode *inode, struct file *file)
{
	struct hcpl0872_priv *priv = container_of(inode->i_cdev, struct hcpl0872_priv, cdev);

	file->private_data = priv;

	return 0;
}

static int hcpl0872_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations hcpl0872_ops = {
	.owner		= THIS_MODULE,
	.read		= hcpl0872_read,
	.open		= hcpl0872_open,
	.release	= hcpl0872_release,
};

static int __devinit hcpl0872_probe(struct spi_device *spi)
{
	struct hcpl0872_priv *priv;
	int err;
	int id;

	dev_info(&spi->dev, "Probing...\n");

	priv = kzalloc(sizeof(struct hcpl0872_priv), GFP_KERNEL);

	if (!priv) {
		err = -ENOMEM;
		goto exit_free;
	}

	/* Initialize spi */
	priv->spi_dev = spi;
	priv->pdata = spi->dev.platform_data;

	/* Request GPIO for channel select */
	err = gpio_request(priv->pdata->channel_gpio, "channel_gpio");
	if(err < 0)
	{
		dev_info(&spi->dev, "Failed to request pin #%i\n", priv->pdata->channel_gpio);
		goto exit_free;
	}

	gpio_direction_output(priv->pdata->channel_gpio, 0);


	dev_set_drvdata(&spi->dev, priv);


	alloc_chrdev_region(&priv->dev, 0, 1, "hcpl0872");
	cdev_init(&priv->cdev, &hcpl0872_ops);
	priv->cdev.owner = THIS_MODULE;
	priv->cdev.ops = &hcpl0872_ops;

	cdev_add(&priv->cdev, priv->dev, 1);
	priv->device = device_create(hcpl0872_class, NULL, priv->dev, NULL, "hcpl0872");

	dev_info(&spi->dev, "hcpl0872 device registered\n");

	return 0;

exit_free:
	kzfree(priv);
	return err;
}




static int __devexit hcpl0872_remove(struct spi_device *spi)
{
	struct hcpl0872_priv *priv = dev_get_drvdata(&spi->dev);

	device_del(priv->device);
	cdev_del(&priv->cdev);
	unregister_chrdev_region(priv->dev, 1);

	class_destroy(hcpl0872_class);

	return 0;
}

static struct spi_driver hcpl0872_driver = {
	.probe	= hcpl0872_probe,
	.remove	= hcpl0872_remove,
	.driver = {
		.name	= "hcpl0872",
		.owner	= THIS_MODULE,
	},
};

static __init int hcpl0872_init_module(void)
{
	hcpl0872_class = class_create(THIS_MODULE, "hcpl0872");

	if(hcpl0872_class == NULL) {
		printk("Could not create class counter\n");
		return -ENOMEM;
	}

	return spi_register_driver(&hcpl0872_driver);
}

static __exit void hcpl0872_cleanup_module(void)
{
	spi_unregister_driver(&hcpl0872_driver);
}

module_init(hcpl0872_init_module);
module_exit(hcpl0872_cleanup_module);


