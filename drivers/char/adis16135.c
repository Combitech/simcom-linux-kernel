/*
 * adis16135.c
 *
 *  Created on: Sep 17, 2010
 *      Author: dakila
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/adis16135.h>

MODULE_DESCRIPTION("Analog Devices adis16135 3-axis Digital Gyroscope");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Kiland <david.kiland@combitech.se>");



struct adis16135_priv {
	struct spi_device *spi_dev;
	int cs_gpio;
	struct cdev cdev;
	struct device *device;
	dev_t dev;
};

static struct class *adis_class = 0;


static int adis16135_read_register(struct adis16135_priv *priv, int reg)
{
	printk("read register %04x\n", reg);
	return 0;
}

static int adis16135_write_register(struct adis16135_priv *priv, int reg, int val)
{
	printk("write register %04x with %04x\n", reg, val);
	return 0;
}


int adis16135_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct adis16135_priv *priv = file->private_data;
	u16 data[2];
	int ret;


	switch (cmd) {
	case ADIS16135_WRITE_REGISTER:
		if(copy_from_user(data, (u16 *)arg, 4)) {
			ret = -ENOMEM;
		}
		ret = adis16135_write_register(priv, data[0], data[1]);
		break;
	case ADIS16135_READ_REGISTER:
		if(copy_from_user(data, (u16 *)arg, 2)) {
			ret = -ENOMEM;
		}
		ret = adis16135_read_register(priv, data[0]);
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}



static ssize_t adis16135_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct adis16135_priv *priv = file->private_data;

	return 0;
}

static ssize_t adis16135_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	struct adis16135_priv *priv = file->private_data;


	return count;
}


static int adis16135_open(struct inode *inode, struct file *file)
{
	struct adis16135_priv *priv = container_of(inode->i_cdev, struct adis16135_priv, cdev);

	file->private_data = priv;

	return 0;
}

static int adis16135_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations adis16135_ops = {
	.owner		= THIS_MODULE,
	.read		= adis16135_read,
	.write		= adis16135_write,
	.open		= adis16135_open,
	.release	= adis16135_release,
	.ioctl		= adis16135_ioctl,
};

static int __devinit adis16135_probe(struct spi_device *spi)
{
	struct adis16135_priv *priv;
	int err;

	dev_info(&spi->dev, "Probing adis16135..\n");

	priv = kzalloc(sizeof(struct adis16135_priv), GFP_KERNEL);

	if (!priv) {
		err = -ENOMEM;
		goto exit_free;
	}

	/* Initialize spi and read device id */
	priv->spi_dev = spi;
	dev_set_drvdata(&spi->dev, priv);

	alloc_chrdev_region(&priv->dev, 0, 1, "adxl");
	cdev_init(&priv->cdev, &adis16135_ops);
	priv->cdev.owner = THIS_MODULE;
	priv->cdev.ops = &adis16135_ops;

	cdev_add(&priv->cdev, priv->dev, 1);
	priv->device = device_create(adis_class, NULL, priv->dev, NULL, "adis16135");

	dev_info(&spi->dev, "adis16135 device registered\n");

	return 0;

exit_free:
	kzfree(priv);
	return err;
}




static int __devexit adis16135_remove(struct spi_device *spi)
{
	struct adis16135_priv *priv = dev_get_drvdata(&spi->dev);

	device_del(priv->device);
	cdev_del(&priv->cdev);
	unregister_chrdev_region(priv->dev, 1);

	return 0;
}

static struct spi_driver adis16135_driver = {
	.probe	= adis16135_probe,
	.remove	= adis16135_remove,
	.driver = {
		.name	= "adis16135",
		.owner	= THIS_MODULE,
	},
};

static __init int adis16135_init_module(void)
{
	adis_class = class_create(THIS_MODULE, "adis");

	if(adis_class == NULL) {
		printk("Could not create class adis\n");
		return -ENOMEM;
	}

	return spi_register_driver(&adis16135_driver);
}

static __exit void adis16135_cleanup_module(void)
{
	spi_unregister_driver(&adis16135_driver);
}

module_init(adis16135_init_module);
module_exit(adis16135_cleanup_module);

MODULE_ALIAS("spi:adis16135");
