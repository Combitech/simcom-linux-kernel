/**
 * drivers/gpio/ms6_gpio.c
 *
 * Copyright (C) 2010 Mats-Olov Rustad, Combitech
 * Copyright (C) 2010 David Kiland, Combitech
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/gpio.h>


#define DRIVER_NAME "ms6_gpio"
struct platform_device *gpio_dev;

/*
 * Some registers must be read back to modify.
 * To save time we cache them here in memory
 */
struct ms6 {
	struct mutex	lock;
	u8		port_config[8];	/* field 0 is unused */
	u32		out_level;	/* cached output levels */
	struct gpio_chip chip;
};


static int ms6_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	struct ms6 *ts = container_of(chip, struct ms6, chip);

	return 0;
}


static int ms6_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct ms6 *ts = container_of(chip, struct ms6, chip);

	return 0;
}


static int ms6_gpio_direction_output(struct gpio_chip *chip, unsigned int offset,
								     int value)
{
	struct ms6 *ts = container_of(chip, struct ms6, chip);

	return 0;
}


static void ms6_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct ms6 *ts = container_of(chip, struct ms6, chip);

}


static int __devinit ms6_gpio_probe(struct platform_device *dev)
{
	struct ms6 *ts;
	int ret;
	
	printk("Probing for ms6_gpio device\n");
	
	ts = kzalloc(sizeof(struct ms6), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	dev_set_drvdata(&dev->dev, ts);

	/* Initialize chip */
	


	ts->chip.label = DRIVER_NAME,

	ts->chip.direction_input = ms6_gpio_direction_input;
	ts->chip.get = ms6_gpio_get;
	ts->chip.direction_output = ms6_gpio_direction_output;
	ts->chip.set = ms6_gpio_set;

	ts->chip.base = 0;
	ts->chip.ngpio = 48;
	ts->chip.can_sleep = 1;
	ts->chip.owner = THIS_MODULE;

	ret = gpiochip_add(&ts->chip);
	if (ret)
		goto exit_destroy;

	return ret;

exit_destroy:
	dev_set_drvdata(&dev->dev, NULL);
	mutex_destroy(&ts->lock);
	kfree(ts);

	return ret;
}


static int __devexit ms6_gpio_remove(struct platform_device *dev)
{
	struct ms6 *ts;
	int ret;

	ts = dev_get_drvdata(&dev->dev);
	if (ts == NULL)
		return -ENODEV;

	ret = gpiochip_remove(&ts->chip);
	if (!ret) {
		mutex_destroy(&ts->lock);
		kfree(ts);
	}
	
	return ret;
}


static struct platform_driver ms6_gpio_driver = {
	.driver = {
		.name = "ms6_gpio",
		.owner = THIS_MODULE,
	},
	.probe = ms6_gpio_probe,
	.remove = __devexit_p(ms6_gpio_remove)
};


static int __init ms6_gpio_init(void)
{
	printk("Initializing ms6_gpio driver...");
	
	gpio_dev = platform_device_alloc("ms6_gpio", -1);
	
	if(!gpio_dev) {
		return -ENOMEM;
	}
	
	platform_device_add(gpio_dev);
	platform_driver_register(&ms6_gpio_driver);

	printk("Done!\n");
	
	return 0;
}
/* register after spi postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(ms6_gpio_init);


static void __exit ms6_gpio_exit(void)
{
	printk("Unloading ms6_gpio driver...");
	
	platform_driver_unregister(&ms6_gpio_driver);
	platform_device_del(gpio_dev);
	kzfree(gpio_dev);
	
	printk("Done!");
}
module_exit(ms6_gpio_exit);


MODULE_AUTHOR("Mats-Olov Rustad");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MS6 FPGA GPIO Expander");
