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
#include <mach/ssp.h>
#include <mach/regs-ssp.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>


MODULE_DESCRIPTION("Analog Devices adxl345/adxl346 3-axis Digital Accelerometer");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Kiland <david.kiland@combitech.se>");


#define DEVID				0x00
#define THRESH_TAP			0x1d
#define OFSX				0x1e
#define OFSY				0x1f
#define OFSZ				0x20
#define DUR					0x21
#define LATENT				0x22
#define WINDOW				0x23
#define THRESH_ACT			0x24
#define THRESH_INACT		0x25
#define TIME_INAC			0x26
#define ACT_INACT_CTL		0x27
#define THRESH_FF			0x28
#define TIME_FF				0x29
#define TAP_AXES			0x2a
#define ACT_TAP_STATUS		0x2b
#define BW_RATE				0x2c
#define POWER_CTL			0x2d
#define INT_ENABLE			0x2e
#define INT_MAP				0x2f
#define INT_SOURCE			0x30
#define DATA_FORMAT			0x31
#define DATAX0				0x32
#define DATAX1				0x33
#define DATAY0				0x34
#define DATAY1				0x35
#define DATAZ0				0x36
#define DATAZ1				0x37
#define FIFO_CTL			0x38
#define FIFO_STATUS			0x39
#define TAP_SIGN			0x3a
#define ORIENT_CONF			0x3b

#define WRITE_SINGLE		0x00
#define WRITE_MULT			0x40
#define READ_SINGLE			0x80
#define READ_MULT			0xc0


#define ADXL345		1
#define ADXL346		2


static const struct platform_device_id adxl34x_id[] = {
	{ "adxl345", ADXL345 },
	{ "adxl346", ADXL346 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adxl34x_id);



struct adxl34x_priv {
	struct ssp_dev *spi_dev;
	int cs_gpio;
	int type;
	struct cdev cdev;
	struct device *device;
	dev_t dev;
};

static struct class *adxl_class = 0;



static int spi_read_byte(struct ssp_dev *dev, u8 *data)
{
	u32 d = 0;

	ssp_read_word(dev, &d);
	*data = d;
	return 0;
}

static int spi_write_byte(struct ssp_dev *dev, u8 data)
{
	ssp_write_word(dev, data);
	ssp_flush(dev);
	return 0;
}



static int adxl34x_read_id(struct adxl34x_priv *priv)
{
	u8 tx[1];
	u8 rx[1];

	tx[0] = READ_SINGLE | DEVID;
	spi_write_byte(&priv->spi_dev, tx[0]);
	spi_read_byte(&priv->spi_dev, rx[0]);
	//spi_write_then_read(priv->spi_dev, tx, 1, rx, 1);
	return rx[0];
}

static int adxl34x_read_reg(struct adxl34x_priv *priv, u8 reg)
{
	u8 tx[1];
	u8 rx[1];

	tx[0] = READ_SINGLE | reg;

	spi_write_byte(&priv->spi_dev, tx[0]);
	spi_read_byte(&priv->spi_dev, rx[0]);
	//spi_write_then_read(priv->spi_dev, tx, 1, rx, 1);
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

	spi_write_byte(&priv->spi_dev, tx[0]);
	spi_read_byte(&priv->spi_dev, rx[0]);
	spi_read_byte(&priv->spi_dev, rx[1]);
	spi_read_byte(&priv->spi_dev, rx[2]);
	spi_read_byte(&priv->spi_dev, rx[3]);
	spi_read_byte(&priv->spi_dev, rx[4]);
	spi_read_byte(&priv->spi_dev, rx[5]);
	//spi_write_then_read(priv->spi_dev, tx, 1, rx, 6);

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


	spi_write_byte(&priv->spi_dev, tx[0]);
	spi_write_byte(&priv->spi_dev, tx[1]);
	//spi_write(priv->spi_dev, tx, 2);


	return count;
}


int adxl34x_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct adxl34x_priv *priv = file->private_data;
	u16 data[2];
	int ret;


	switch (cmd) {
	case 1:
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

static int __devinit adxl34x_probe(struct platform_device *pdev)
{
	struct adxl34x_priv *priv;
	int err;
	int dev_id;
	struct platform_device_id *id;

	dev_info(&pdev->dev, "Probing adxl34x..\n");

	priv = kzalloc(sizeof(struct adxl34x_priv), GFP_KERNEL);

	if (!priv) {
		err = -ENOMEM;
		goto exit_free;
	}

	dev_set_drvdata(&pdev->dev, priv);

	/* Get no of channels */
	id = (struct platform_device_id *)platform_get_device_id(pdev);
	priv->type = id->driver_data;

	dev_id = adxl34x_read_id(priv);
	if(dev_id != 0xe6) {
		dev_err(&pdev->dev, "Invalid device id. (Read 0x%02x)", dev_id);
		err = -EINVAL;
		goto exit_free;

	}

	alloc_chrdev_region(&priv->dev, 0, 1, "adxl");
	cdev_init(&priv->cdev, &adxl34x_ops);
	priv->cdev.owner = THIS_MODULE;
	priv->cdev.ops = &adxl34x_ops;

	cdev_add(&priv->cdev, priv->dev, 1);
	priv->device = device_create(adxl_class, NULL, priv->dev, NULL, "adxl34x");

	dev_info(&pdev->dev, "adxl34x device registered\n");

	return 0;

exit_free:
	kzfree(priv);
	return err;
}




static int __devexit adxl34x_remove(struct platform_device *pdev)
{
	struct adxl34x_priv *priv = dev_get_drvdata(&pdev->dev);

	device_del(priv->device);
	cdev_del(&priv->cdev);
	unregister_chrdev_region(priv->dev, 1);

	class_destroy(adxl_class);

	return 0;
}

static struct platform_driver adxl34x_driver = {
	.probe	= adxl34x_probe,
	.remove	= adxl34x_remove,
	.id_table = adxl34x_id,
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

	return platform_driver_register(&adxl34x_driver);
}

static __exit void adxl34x_cleanup_module(void)
{
	platform_driver_unregister(&adxl34x_driver);
}

module_init(adxl34x_init_module);
module_exit(adxl34x_cleanup_module);

MODULE_ALIAS("spi:adxl34x");
