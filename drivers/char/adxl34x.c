/*
 * adxl34x.c
 *
 *  Created on: Sep 16, 2010
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
#include <linux/adxl34x.h>

MODULE_DESCRIPTION("Analog Devices adxl345/adxl346 3-axis Digital Accelerometer");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Kiland <david.kiland@combitech.se>");


struct adxl34x_priv {
	struct spi_device *spi_dev;
	int cs_gpio;
	struct cdev cdev;
	struct device *device;
	dev_t dev;
};

static struct class *adxl_class = 0;



static int adxl34x_read_id(struct adxl34x_priv *priv)
{
	u8 tx[1];
	u8 rx[1];

	tx[0] = READ_SINGLE | DEVID;
	spi_write_then_read(priv->spi_dev, tx, 1, rx, 1);
	return rx[0];
}

static int adxl34x_read_reg(struct adxl34x_priv *priv, u8 reg)
{
	u8 tx[1];
	u8 rx[1];

	tx[0] = READ_SINGLE | reg;
	spi_write_then_read(priv->spi_dev, tx, 1, rx, 1);
	return rx[0];
}



static ssize_t adxl34x_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	u8 rx[6];
	u8 tx[1];

	struct adxl34x_priv *priv = file->private_data;

	if(count < 6) {
		return 0;
	}

	tx[0] = READ_MULT | DATAX0;
	spi_write_then_read(priv->spi_dev, tx, 1, rx, 6);

	if(copy_to_user(buf, rx, 6)) {
		return 0;
	}

	return 6;
}

static ssize_t adxl34x_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	struct adxl34x_priv *priv = file->private_data;
	u8 tx[2];
	u8 inbuf[2];


	if(count != 2) {
		return -EINVAL;
	}

	if(copy_from_user(inbuf, buf, 2)) {
		return -ENOMEM;
	}

	tx[0] = WRITE_SINGLE | inbuf[0];
	tx[1] = inbuf[1];
	spi_write(priv->spi_dev, tx, 2);


	return count;
}


int adxl34x_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct adxl34x_priv *priv = file->private_data;
	u16 data[2];
	int ret;


	switch (cmd) {
	case ADXL34X_READ_REGISTER:
		if(copy_from_user(data, (u16 *)arg, 2)) {
			ret = -ENOMEM;
		}
		ret = adxl34x_read_reg(priv, data[0]);
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}


static int adxl34x_open(struct inode *inode, struct file *file)
{
	struct adxl34x_priv *priv = container_of(inode->i_cdev, struct adxl34x_priv, cdev);

	file->private_data = priv;

	return 0;
}

static int adxl34x_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations adxl34x_ops = {
	.owner		= THIS_MODULE,
	.read		= adxl34x_read,
	.write		= adxl34x_write,
	.open		= adxl34x_open,
	.release	= adxl34x_release,
	.ioctl		= adxl34x_ioctl,
};

static int __devinit adxl34x_probe(struct spi_device *spi)
{
	struct adxl34x_priv *priv;
	int err;
	int id;

	dev_info(&spi->dev, "Probing adxl34x..\n");

	priv = kzalloc(sizeof(struct adxl34x_priv), GFP_KERNEL);

	if (!priv) {
		err = -ENOMEM;
		goto exit_free;
	}

	/* Initialize spi and read device id */
	priv->spi_dev = spi;
	dev_set_drvdata(&spi->dev, priv);
	id = adxl34x_read_id(priv);
	if(id != 0xe6) {
		dev_err(&spi->dev, "Invalid device id. (Read 0x%02x)", id);
		err = -EINVAL;
		goto exit_free;

	}


	alloc_chrdev_region(&priv->dev, 0, 1, "adxl");
	cdev_init(&priv->cdev, &adxl34x_ops);
	priv->cdev.owner = THIS_MODULE;
	priv->cdev.ops = &adxl34x_ops;

	cdev_add(&priv->cdev, priv->dev, 1);
	priv->device = device_create(adxl_class, NULL, priv->dev, NULL, "adxl34x");

	dev_info(&spi->dev, "adxl34x device registered\n");

	return 0;

exit_free:
	kzfree(priv);
	return err;
}




static int __devexit adxl34x_remove(struct spi_device *spi)
{
	struct adxl34x_priv *priv = dev_get_drvdata(&spi->dev);

	device_del(priv->device);
	cdev_del(&priv->cdev);
	unregister_chrdev_region(priv->dev, 1);

	class_destroy(adxl_class);

	return 0;
}

static struct spi_driver adxl34x_driver = {
	.probe	= adxl34x_probe,
	.remove	= adxl34x_remove,
	.driver = {
		.name	= "adxl34x",
		.owner	= THIS_MODULE,
	},
};

static __init int adxl34x_init_module(void)
{
	adxl_class = class_create(THIS_MODULE, "adxl");

	if(adxl_class == NULL) {
		printk("Could not create class counter\n");
		return -ENOMEM;
	}

	return spi_register_driver(&adxl34x_driver);
}

static __exit void adxl34x_cleanup_module(void)
{
	spi_unregister_driver(&adxl34x_driver);
}

module_init(adxl34x_init_module);
module_exit(adxl34x_cleanup_module);

MODULE_ALIAS("spi:adxl34x");
