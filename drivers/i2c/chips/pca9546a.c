/*
 *  pca9546a.c - 4-channel I2C-bus switch with reset
 *
 *  Copyright (C) 2009 Combitech AB
 *  Tobias Knutsson <tobias.knutsson@combitech.se>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon-sysfs.h>

#define PCA9546A_CONF 	0

/* following are the sysfs callback functions */
static ssize_t pca9546a_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct sensor_device_attribute *psa = to_sensor_dev_attr(attr);
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%d\n", i2c_smbus_read_byte_data(client,
							     psa->index));
}

static ssize_t pca9546a_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct sensor_device_attribute *psa = to_sensor_dev_attr(attr);
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 0);
	if (val > 0xff)
		return -EINVAL;
	i2c_smbus_write_byte_data(client, psa->index, val);
	return count;
}

/* Define the device attributes */

static SENSOR_DEVICE_ATTR(config, S_IRUGO | S_IWUSR, pca9546a_show, \
	pca9546a_store, PCA9546A_CONF);

static struct attribute *pca9546a_attributes[] = {
	&sensor_dev_attr_config.dev_attr.attr,
	NULL
};

static struct attribute_group pca9546a_defattr_group = {
	.attrs = pca9546a_attributes,
};


static int pca9546a_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int conf;

	/* Enable first channel by default */
	i2c_smbus_write_byte_data(client, PCA9546A_CONF, 1);
	conf = i2c_smbus_read_byte_data(client, PCA9546A_CONF);
	dev_info(&client->dev, "switch state: 0:%s 1:%s 2:%s 3:%s\n", \
							conf & (1<<0)? "on": "off", \
							conf & (1<<1)? "on": "off", \
							conf & (1<<2)? "on": "off", \
							conf & (1<<3)? "on": "off");
	/* Register sysfs hooks */
	return sysfs_create_group(&client->dev.kobj,
				  &pca9546a_defattr_group);
}

static int pca9546a_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &pca9546a_defattr_group);
	return 0;
}

static const struct i2c_device_id pca9546a_id[] = {
	{ "pca9546a", 0 },
	{ }
};

static struct i2c_driver pca9546a_driver = {
	.driver = {
		.name	= "pca9546a",
	},
	.probe		= pca9546a_probe,
	.remove		= pca9546a_remove,
	.id_table	= pca9546a_id,
};

static int __init pca9546a_init(void)
{
	return i2c_add_driver(&pca9546a_driver);
}
module_init(pca9546a_init);

static void __exit pca9546a_exit(void)
{
	i2c_del_driver(&pca9546a_driver);
}
module_exit(pca9546a_exit);

MODULE_AUTHOR("Tobias Knutsson <tobias.knutsson@combitech.se>");
MODULE_DESCRIPTION("PCA9546A driver");
MODULE_LICENSE("GPL");
