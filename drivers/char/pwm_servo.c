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
#include <linux/pwm.h>
#include <linux/platform_device.h>


struct pwm_driver {
	struct pwm_device *dev;
	int duty_cycle;
	int period;
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
	long period = 0;

	strict_strtol(buf, 0, &period);

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
	long duty_cycle = 0;

	strict_strtol(buf, 0, &duty_cycle);

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

	pwm_driv->period = 1000000000;
	pwm_driv->duty_cycle = 500000000;

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
	printk("Register PWM module\n");

	if(status < 0) {
		return status;
	}
	return 0;
}
module_init(pwm_servo_init_module);


static void pwm_servo_exit_module (void)
{
	class_unregister(&pwm_class);
}
module_exit(pwm_servo_exit_module);

MODULE_AUTHOR("David Kiland (david.kiland@combitech.se)");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Generic PWM driver");
