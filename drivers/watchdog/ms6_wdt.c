/*
 * Watchdog driver for MS6.
 *
 * Copyright (C) 2010 David Kiland <david.kiland@combitech.se>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/watchdog.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/bitops.h>
#include <linux/uaccess.h>


#define DRV_NAME "MS6 Watchdog"


struct ms6_wdt_data {
	struct i2c_client *client;
	struct timer_list timer;
	unsigned long next_heartbeat;
	int nowayout;
	int timeout;
	unsigned long status;
};

#define WDT_IN_USE			0
#define WDT_OK_TO_CLOSE		1
#define WDT_INTERVAL 		(HZ/5)
#define WDT_TIMEOUT 		30


static struct ms6_wdt_data ms6_data;


static void wdt_enable()
{

}

static void wdt_disable()
{

}

static void wdt_ping()
{

}

static void wdt_startup()
{

}

static void wdt_shutdown()
{

}


static void ms6_wdt_ping(unsigned long data)
{
	if (time_before(jiffies, ms6_data.next_heartbeat)) {
		wdt_ping();
	}

	/* Re-set the timer interval */
	mod_timer(&ms6_data.timer, jiffies + WDT_INTERVAL);
}

static void ms6_wdt_keepalive()
{
	/* user land ping */
	ms6_data.next_heartbeat = jiffies + (ms6_data.timeout * HZ);
}


static int ms6_wdt_open(struct inode *inode, struct file *file)
{
	if(test_and_set_bit(WDT_IN_USE, &ms6_data.status)) {
		return -EBUSY;
	}

	clear_bit(WDT_OK_TO_CLOSE, &ms6_data.status);

	ms6_data.next_heartbeat = jiffies + (ms6_data.timeout * HZ);
	wdt_enable();
	mod_timer(&ms6_data.timer, jiffies + WDT_INTERVAL);

	return nonseekable_open(inode, file);
}

/*
 * Close the watchdog device.
 */
static int ms6_wdt_release(struct inode *inode, struct file *file)
{
	if (test_bit(WDT_OK_TO_CLOSE, &ms6_data.status)) {
		del_timer_sync(&ms6_data.timer);
		wdt_disable();
	}
	else {
		dev_crit(&ms6_data.client->dev, "Device closed unexpectedly - timer will not stop\n");
	}

	clear_bit(WDT_IN_USE, &ms6_data.status);
	clear_bit(WDT_OK_TO_CLOSE, &ms6_data.status);

	return 0;
}

static int ms6_wdt_settimeout(unsigned int timeout)
{
	return 0;
}


static const struct watchdog_info ms6_wdt_info = {
	.identity	= DRV_NAME,
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
};

/*
 * Handle commands from user-space.
 */
static long ms6_wdt_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_value;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
	case WDIOC_KEEPALIVE:
	case WDIOC_SETTIMEOUT:
	case WDIOC_GETTIMEOUT:
		break;
	default:
		break;
	}

	return 0;
}

static ssize_t ms6_wdt_write(struct file *file, const char *data, size_t len, loff_t *ppos)
{
	if (len) {
		if (!ms6_data.nowayout) {
			size_t i;

			clear_bit(WDT_OK_TO_CLOSE, &ms6_data.status);

			for (i = 0; i != len; i++) {
				char c;

				if (get_user(c, data + i)) {
					return -EFAULT;
				}

				if (c == 'V') {
					set_bit(WDT_OK_TO_CLOSE, &ms6_data.status);
				}
				else {
					clear_bit(WDT_OK_TO_CLOSE, &ms6_data.status);
				}
			}
		}

		ms6_wdt_keepalive();
	}

	return len;
}

/* ......................................................................... */

static const struct file_operations ms6_wdt_fops = {
	.owner			= THIS_MODULE,
	.llseek			= no_llseek,
	.unlocked_ioctl	= ms6_wdt_ioctl,
	.open			= ms6_wdt_open,
	.release		= ms6_wdt_release,
	.write			= ms6_wdt_write,
};

static struct miscdevice ms6_wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "ms6_watchdog",
	.fops		= &ms6_wdt_fops,
};

static int ms6_wdt_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int res;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	if(!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_BYTE |
								I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
		dev_err(&client->dev, "Failed when checking i2c functionality\n");
		return -EIO;
	}

	res = misc_register(&ms6_wdt_miscdev);

	if(res) {
		dev_err(&client->dev, "Failed when registering misc device\n");
		return res;
	}

	dev_info(&client->dev, "Loaded successfully\n");

	setup_timer(&ms6_data.timer, ms6_wdt_ping, 0);

	return 0;
}

static int ms6_wdt_remove(struct i2c_client *client)
{
	int res;

	del_timer(&ms6_data.timer);
	wdt_disable();

	res = misc_deregister(&ms6_wdt_miscdev);

	if (!res) {
		ms6_wdt_miscdev.parent = NULL;
	}

	return res;
}

static struct i2c_board_info ms6_wdt_i2c_info = {
	I2C_BOARD_INFO("ms6_wdt", 0x18)
};

static int ms6_wdt_suspend(struct i2c_client *client, pm_message_t message)
{
	return 0;
}

static int ms6_wdt_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ms6_wdt_id[] = {
	{ "ms6_wdt", 0 },
	{ }
};

static struct i2c_driver ms6_wdt_driver = {
	.remove		= __exit_p(ms6_wdt_remove),
	.suspend	= ms6_wdt_suspend,
	.resume		= ms6_wdt_resume,
	.id_table	= ms6_wdt_id,
	.probe 		= ms6_wdt_probe,
	.driver		= {
		.name	= "ms6_wdt",
		.owner	= THIS_MODULE,
	},
};

static int __init ms6_wdt_init(void)
{
	struct i2c_adapter *adapter = i2c_get_adapter(0);

	if(!adapter) {
		printk("Could not find any i2c adapter\n");
		return -EINVAL;
	}

	ms6_data.client = i2c_new_device(i2c_get_adapter(0), &ms6_wdt_i2c_info);
	return i2c_add_driver(&ms6_wdt_driver);

}

static void __exit ms6_wdt_exit(void)
{
	if(ms6_data.client) {
		i2c_del_driver(&ms6_wdt_driver);
		i2c_unregister_device(ms6_data.client);
	}
}

module_init(ms6_wdt_init);
module_exit(ms6_wdt_exit);

MODULE_AUTHOR("David Kiland <david.kiland@combitech.se>");
MODULE_DESCRIPTION("Watchdog driver for MS6");
MODULE_LICENSE("GPL");
