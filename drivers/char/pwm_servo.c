/*
 * PWM Servo
 *
 * Copyright 2010 (c) Combitech AB
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */


#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/platform_device.h>


#define SERVO_ENABLE	90

struct pwm_driver {
	struct pwm_device *dev;
	unsigned int duty_cycle;
	unsigned int period;
};


int pwm_export(int pwm);


static ssize_t pwm_period_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pwm_driver *drv = dev_get_drvdata(dev);

	sprintf(buf, "%dns\n", drv->period);
	return strlen(buf);
}

static ssize_t pwm_period_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct pwm_driver *drv = dev_get_drvdata(dev);
	unsigned long period = 0;

	strict_strtoul(buf, 0, &period);
	printk("period: %d \n", period);

	drv->period = period;

	pwm_config(drv->dev, drv->duty_cycle, drv->period);
	return size;
}
static const DEVICE_ATTR(period, 0644, pwm_period_show, pwm_period_store);


static ssize_t pwm_duty_cycle_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pwm_driver *drv = dev_get_drvdata(dev);

	sprintf(buf, "%dns\n", drv->duty_cycle);
	return strlen(buf);
}

static ssize_t pwm_duty_cycle_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct pwm_driver *drv = dev_get_drvdata(dev);
	unsigned long duty_cycle = 0;

	strict_strtoul(buf, 0, &duty_cycle);
	printk("duty: %d \n", duty_cycle);

	drv->duty_cycle = duty_cycle;

	pwm_config(drv->dev, drv->duty_cycle, drv->period);
	return size;
}
static const DEVICE_ATTR(duty_cycle, 0644, pwm_duty_cycle_show, pwm_duty_cycle_store);


static const struct attribute *pwm_attrs[] = {
	&dev_attr_duty_cycle.attr,
	&dev_attr_period.attr,
	NULL,
};

static const struct attribute_group pwm_attr_group = {
	.attrs = (struct attribute **) pwm_attrs,
};



static ssize_t enable_store(struct class *class, const char *buf, size_t len)
{
	long enable;
	int	status;

	status = strict_strtol(buf, 0, &enable);

	if (status < 0)
		return status;

	if(enable > 0)
		gpio_set_value(SERVO_ENABLE, 1);
	else
		gpio_set_value(SERVO_ENABLE, 0);

	return len;
}

static ssize_t enable_show(struct class *class, char *buf)
{
	sprintf(buf, "%d\n", gpio_get_value(SERVO_ENABLE));
	return strlen(buf);
}



static ssize_t export_store(struct class *class, const char *buf, size_t len)
{
	long pwm;
	int	status;

	status = strict_strtol(buf, 0, &pwm);

	if (status < 0)
		return status;

	status = pwm_export(pwm);

	if(status < 0) {
		return status;
	}

	return len;
}

static ssize_t unexport_store(struct class *class, const char *buf, size_t len)
{
	return len;
}


static struct class_attribute pwm_class_attrs[] = {
	__ATTR(export, 0200, NULL, export_store),
	__ATTR(unexport, 0200, NULL, unexport_store),
	__ATTR(enable, 0600, enable_show, enable_store),
	__ATTR_NULL,
};

static struct class pwm_class = {
	.name =		"pwm-servo",
	.owner =	THIS_MODULE,
	.class_attrs =	pwm_class_attrs,
};



int pwm_export(int pwm)
{
	struct pwm_driver *pwm_driv;
	struct device *dev;
	int status = 0;

	pwm_driv = kzalloc(sizeof(struct pwm_driver), GFP_KERNEL);

	pwm_driv->dev = pwm_request(pwm, "pwm");

	if(!pwm_driv->dev) {
		kzfree(pwm_driv);
		return -EINVAL;
	}

	dev = device_create(&pwm_class, 0, MKDEV(0, 0), pwm_driv, "pwm%d", pwm);
	status = sysfs_create_group(&dev->kobj, &pwm_attr_group);

	pwm_driv->period 		= 3333333;
	pwm_driv->duty_cycle 	= 1000000;

	/* 1ms period, 50% duty cycle */
	pwm_config(pwm_driv->dev, pwm_driv->duty_cycle, pwm_driv->period);
	pwm_enable(pwm_driv->dev);

	return status;
}


int pwm_unexport(int pwm)
{
	return 0;
}


static int pwm_servo_init_module(void)
{
	int status;

	status = class_register(&pwm_class);
	if(status < 0)
			return status;


	status = gpio_request(SERVO_ENABLE, "Servo enable");
	if(status < 0)
			return status;

	gpio_direction_output(SERVO_ENABLE, 0);

	return 0;
}
module_init(pwm_servo_init_module);


static void pwm_servo_exit_module (void)
{
	class_unregister(&pwm_class);

	gpio_free(SERVO_ENABLE);
}
module_exit(pwm_servo_exit_module);

MODULE_AUTHOR("David Kiland (david.kiland@combitech.se)");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Generic PWM driver");
