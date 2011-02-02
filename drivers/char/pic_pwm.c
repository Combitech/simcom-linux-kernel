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

MODULE_DESCRIPTION("Driver for a PIC Microcontroller connected to Nacelle used for PWM measurement");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marcus Folkesson <marcus.folkesson@combitech.se>");
MODULE_ALIAS("spi:pic_pwm");

/* Command to recieve measures */
#define	CMD_GET 0xAB


struct pic_priv {
	struct spi_dev *spi_dev;
	int cs_gpio;
	struct cdev cdev;
	struct device *device;
	dev_t dev;
};

static struct class *pic_class = 0;


static ssize_t pic_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{

	u8 rx[5];
	u8 tx[1];

	struct pic_priv *priv = file->private_data;

	if(count < 4) {
		return 0;
	}

	tx[0] = CMD_GET;
	spi_write_then_read(priv->spi_dev, tx, 1, rx, 5);

	if(copy_to_user(buf, rx, 5)) {
		return 0;
	}

	return 5;
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
	.open		= pic_open,
	.release	= pic_release,
};



static int pic_probe(struct spi_device *spi)
{
	struct pic_priv *priv;
	int err;

	dev_info(&spi->dev, "Probing pic_pwm device\n");

	priv = kzalloc(sizeof(struct pic_priv), GFP_KERNEL);
	if(priv == NULL)
	{
		err = -ENOMEM;
		goto exit_free;
	}

	priv->spi_dev = spi;
	dev_set_drvdata(&spi->dev, priv);


	alloc_chrdev_region(&priv->dev, 0, 1, "pic_pwm");
	cdev_init(&priv->cdev, &pic_ops);
	priv->cdev.owner = THIS_MODULE;
	priv->cdev.ops = &pic_ops;

	cdev_add(&priv->cdev, priv->dev, 1);
	priv->device = device_create(pic_class, NULL, priv->dev, NULL, "pic_pwm");

	dev_info(&spi->dev, "pic_pwm device registered\n");

	return 0;

exit_free:
	kzfree(priv);
	return err;
}

static int pic_remove(struct spi_device *spi)
{

	struct pic_priv *priv = dev_get_drvdata(&spi->dev);

	device_del(priv->device);
	cdev_del(&priv->cdev);
	unregister_chrdev_region(priv->dev, 1);

	class_destroy(pic_class);
	return 0;
}


static struct spi_driver pic_driver = {
		.probe		= pic_probe,
		.remove		= pic_remove,
		.driver		= {
						.name = "pic_pwm",
						.owner = THIS_MODULE,
		},
};


static  int pic_module_init()
{
	pic_class = class_create(THIS_MODULE, "pic_pwm");

	if(pic_class == NULL)
	{
		printk("pic_pwm: Could not create class");
		return -ENOMEM;
	}

	return spi_register_driver(&pic_driver);
}

static  void pic_module_exit()
{
	spi_unregister_driver(&pic_driver);
}

module_init(pic_module_init);
module_exit(pic_module_exit);





