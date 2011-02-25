/*
 *  solenoid.c - TMP175 Digital Temperature Sensor from Texas Instruments
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
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/device.h>

MODULE_DESCRIPTION("Driver for setting solenoid GPIOs on Nacelle");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marcus Folkesson <marcus.folkesson@combitech.se>");
MODULE_ALIAS("nacelle_solenoid");



/* Defines GPIOs for solenoids */
#define SOLENOID_1		97
#define SOLENOID_2		91


struct cdev cdev;
struct device *device;
dev_t dev;


static struct class *solenoid_class = 0;


static ssize_t solenoid_write(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char data[2];

	if(count != 2)
		return 0;

	if(copy_from_user(&data, buf, 2) != 0)
		return 0;

	gpio_set_value(SOLENOID_1, data[0]);
	gpio_set_value(SOLENOID_2, data[1]);

	return count;
}


static int solenoid_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int solenoid_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations solenoid_ops = {
	.owner		= THIS_MODULE,
	.write		= solenoid_write,
	.open		= solenoid_open,
	.release	= solenoid_release,
};




static  int solenoid_module_init()
{
	solenoid_class = class_create(THIS_MODULE, "solenoid");

	if(solenoid_class == NULL)
	{
		printk("solenoid_pwm: Could not create class");
		return -ENOMEM;
	}

	/* Request gpio for solenoid 1 */
	if(gpio_request(SOLENOID_1, "solenoid 1") < 0) {
		printk("Could not request gpio pin for solenoid1\n");
		goto exit_free;
	}

	gpio_direction_output(SOLENOID_1, 0);


	/* Request gpio for solenoid 2 */
	if(gpio_request(SOLENOID_2, "solenoid 2") < 0) {
		printk("Could not request gpio pin for solenoid2\n");
		goto exit_free;
	}

	gpio_direction_output(SOLENOID_2, 0);

	alloc_chrdev_region(&dev, 0, 1, "solenoid");
	cdev_init(&cdev, &solenoid_ops);
	cdev.owner = THIS_MODULE;
	cdev.ops = &solenoid_ops;

	cdev_add(&cdev, dev, 1);
	device = device_create(solenoid_class, NULL, dev, NULL, "solenoid");

	printk("Solenoid driver is loaded\n");

	exit_free:

	return 0;

}

static  void solenoid_module_exit()
{
	device_del(device);
	cdev_del(&cdev);
	unregister_chrdev_region(dev, 1);

	class_destroy(solenoid_class);
}

module_init(solenoid_module_init);
module_exit(solenoid_module_exit);





