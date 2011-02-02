/*
 *  tmp175.c - TMP175 Digital Temperature Sensor from Texas Instruments
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
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/device.h>

MODULE_DESCRIPTION("Driver for a TMP175 Digital Temperature Sensor from Texas Instruments");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marcus Folkesson <marcus.folkesson@combitech.se>");
MODULE_ALIAS("i2c:tmp175");


/*
 * Registers
 */

#define	TEMP_REG	0x00
#define	CONF_REG	0x01
#define TLOW_REG	0x02
#define THIGH_REG	0x03

/*
 * Private data
 */

struct tmp175_priv {
	struct i2c_client *client;
	struct cdev cdev;
	struct device *device;
	dev_t dev;
};

static struct class *tmp175_class = 0;


static ssize_t tmp175_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	u16 value;
	struct tmp175_priv *priv = file->private_data;

	if(count < 2)
	{
		return 0;
	}

	value = i2c_smbus_read_word_data(priv->client, TEMP_REG);

	if(value < 0)
		return 0;

	if(copy_to_user(buf, &value, 2))
	{
		return 0;
	}

	return 2;
}


static int tmp175_open(struct inode *inode, struct file *file)
{
	struct tmp175_priv *priv = container_of(inode->i_cdev, struct tmp175_priv, cdev);

	file->private_data = priv;

	return 0;
}

static int tmp175_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations tmp175_ops = {
	.owner		= THIS_MODULE,
	.read		= tmp175_read,
	.open		= tmp175_open,
	.release	= tmp175_release,
};



static int tmp175_probe(struct i2c_client *client,
		 const struct i2c_device_id *id)
{
	struct tmp175_priv *priv;
	int err;

	dev_info(&client->dev, "Probing tmp175 device\n");

	priv = kzalloc(sizeof(struct tmp175_priv), GFP_KERNEL);
	if(priv == NULL)
	{
		err = -ENOMEM;
		goto exit_free;
	}

	priv->client = client;
	dev_set_drvdata(&client->dev, priv);


	alloc_chrdev_region(&priv->dev, 0, 1, "tmp175");
	cdev_init(&priv->cdev, &tmp175_ops);
	priv->cdev.owner = THIS_MODULE;
	priv->cdev.ops = &tmp175_ops;

	cdev_add(&priv->cdev, priv->dev, 1);
	priv->device = device_create(tmp175_class, NULL, priv->dev, NULL, "tmp175");

	return 0;

exit_free:
	kzfree(priv);
	return err;
}

static int tmp175_remove(struct i2c_client *client)
{

	struct tmp175_priv *priv = dev_get_drvdata(&client->dev);

	device_del(priv->device);
	cdev_del(&priv->cdev);
	unregister_chrdev_region(priv->dev, 1);

	class_destroy(tmp175_class);

	return 0;
}


static const struct i2c_device_id tmp175_id[] = {
	{ "tmp175", 0x48 },
	{ }
};

static struct i2c_driver tmp175_driver = {
	.probe		= tmp175_probe,
	.remove		= tmp175_remove,
	.id_table	= tmp175_id,
	.driver		= {
					.name = "tmp175",
					.owner = THIS_MODULE,
	},
};



static  int tmp175_module_init()
{
	tmp175_class = class_create(THIS_MODULE, "tmp175");

	if(tmp175_class == NULL)
	{
		printk("tmp175_pwm: Could not create class");
		return -ENOMEM;
	}

	return i2c_add_driver(&tmp175_driver);
}

static  void tmp175_module_exit()
{
	class_destroy(tmp175_class);
	i2c_del_driver(&tmp175_driver);
}

module_init(tmp175_module_init);
module_exit(tmp175_module_exit);





