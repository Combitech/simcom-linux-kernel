/*
 *  ad77xx.c - Analog Devices AD7799/AD7798 3-channel Analog/Digital Converter
 *
 *  Copyright (C) 2010 Combitech AB
 *  Marcus Folkesson <marcus.folkesson@combitech.se>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/ad77xx.h>


MODULE_DESCRIPTION("Analog Devices AD7799/AD7798 3-channel Analog/Digital Converter");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marcus Folkesson <marcus.folkesson@combitech.se>");
MODULE_ALIAS("spi:ad77xx");


/*
 * Status
 */
#define AD77XX_STATUS_RDY       	0x80
#define AD77XX_STATUS_ERROR     	0x40
#define AD77XX_STATUS_NOREF     	0x20
#define AD77XX_STATUS_IS_AD7799 	0x08
#define AD77XX_STATUS_CHAN_MASK 	0x07

static u8 ad77xx_init_status;



/*
 * 	Configurations
 */
enum {	AD77XX_CONTINUOUS_CONVERSION_MODE = 0,
		AD77XX_SINGLE_CONVERSION_MODE,
		AD77XX_IDLE_MODE, AD77XX_POWERDOWN_MODE,
		AD77XX_INTERNAL_OFFSET_CAL_MODE,
		AD77XX_INTERNAL_SCALE_CAL_MODE,
		AD77XX_SYSTEM_OFFSET_CAL_MODE,
		AD77XX_SYSTEM_SCALE_CAL_MODE
};

enum {	AD77XX_470_HZ = 1,
		AD77XX_242_HZ,
		AD77XX_123_HZ,
		AD77XX_62_HZ,
		AD77XX_50_HZ,
		AD77XX_39_HZ,
		AD77XX_33_2_HZ,
		AD77XX_19_6_HZ,
		AD77XX_16_7_1_HZ,
		AD77XX_16_7_2_HZ,
		AD77XX_12_5_HZ,
		AD77XX_10_HZ,
		AD77XX_8_33_HZ,
		AD77XX_6_25_HZ,
		AD77XX_4_17_HZ
};

enum {
		AD77XX_1_GAIN = 0,
		AD77XX_2_GAIN,
		AD77XX_4_GAIN,
		AD77XX_8_GAIN,
		AD77XX_16_GAIN,
		AD77XX_32_GAIN,
		AD77XX_64_GAIN,
		AD77XX_128_GAIN
};

enum { 	AD77XX_AIN1_CHAN = 0,
		AD77XX_AIN2_CHAN,
		AD77XX_AIN3_CHAN,
		AD77XX_AIN11_CHAN,
		AD77XX_AVDD_CHAN
};



/*
 * Private data structure
 */
struct ad77xx_priv {
	struct spi_device *spi_dev;
	int cs_gpio;
	struct cdev cdev;
	struct device *device;
	dev_t dev;
};

static struct class *ad77xx_class = 0;

/*
 * Write to Communication Register
 *
 * @param reg Register
 * @param read Is the next operation read?
 * @param cont Continuous mode
 */
static inline void ad77xx_comm(struct spi_device *spi, u8 reg, u8 read, u8 cont)
{
	u8 byte = (read ? 0x40 : 0x00) | (reg << 3) | (cont ? 0x04 : 0x00);
	spi_write(spi, &byte, 1);
}


/*
 * Read Status Register
 */
static u8 ad77xx_status(struct spi_device *spi)
{
	u8 rx;
	ad77xx_comm(spi, AD77XX_STATUS_REG, 1, 0);

	spi_read(spi, &rx, 1);
	return rx;
}

/*
 * Read Offset Register
 */
u32 ad77xx_read_offset(struct spi_device *spi)
{
	u8 rx;
	u32 val = 0;

	ad77xx_comm(spi, AD77XX_OFFSET_REG, 1, 0);

	if(ad77xx_init_status & AD77XX_STATUS_IS_AD7799)
	{
		spi_read(spi, &rx,1);
		val |= rx << 16;
	}

	spi_read(spi, &rx,1);
	val |= rx << 8;

	spi_read(spi, &rx,1);
	val |= rx;

	return val;
}

/*
 * Read Full Scale Register
 */
u32 ad77xx_read_scale(struct spi_device *spi)
{
	u32 val = 0;
	u8 rx;

	ad77xx_comm(spi, AD77XX_SCALE_REG, 1, 0);

	if(ad77xx_init_status & AD77XX_STATUS_IS_AD7799)
	{
		spi_read(spi, &rx,1);
		val |= rx << 16;
	}

	spi_read(spi, &rx,1);
	val |= rx << 8;

	spi_read(spi, &rx,1);
	val |= rx;

	return val;
}

/*
 * Set Mode
 *
 * @param mode Mode (see Configuration above)
 * @param pwd Power Switch bit
 * @param rate Update rate
 */
static void ad77xx_set_mode(struct spi_device *spi, u8 mode, u8 psw, u8 rate)
{
	u8 tx;

    ad77xx_comm(spi, AD77XX_MODE_REG, 0, 0);

	tx = mode << 5 | (psw ? 0x10 : 0x00);
    spi_write(spi, &tx, 1);

    spi_write(spi, &rate, 1);
}

/*
 * Write to Configuration Register
 *
 * @param burnout Burn out current enable bit
 * @param unipolar Unipolar/Bipolar enable bit
 * @param gain Gain select bits
 * @param ref_det Reference detect bit
 * @param buf ADC Buffered mode
 * @param chan Channel select bit
 */
void ad77xx_write_config(	struct spi_device *spi,
							u8 burnout, u8 unipolar, u8 gain,
							u8 ref_det, u8 buf, u8 chan)
{
	u8 tx;

	ad77xx_comm(spi, AD77XX_CONFIG_REG, 0, 1);

	tx = ((burnout ? 0x40 : 0x00) | (unipolar ? 0x20 : 0x00) | gain);
	spi_write(spi, &tx, 1);

	tx = ((ref_det ? 0x20 : 0x00) | (buf ? 0x10 : 0x00) | chan);
	spi_write(spi, &tx, 1);
}


/*
 *  Request a read from the data register
 *
 *@param continuous Continuous reading
 */
void ad77xx_request_data(struct spi_device *spi, u8 continuous)
{
	ad77xx_comm(spi, AD77XX_DATA_REG, 1, continuous);
}


/* Read from data register, it should be previously requested
 *   from ad77xx_request_data. The value is signed!!
 *
 */
s32 ad77xx_read_data(struct spi_device *spi)
{
	s32 val = 0;
	u8 rx;

	if(ad77xx_init_status & AD77XX_STATUS_IS_AD7799)
	{
		spi_read(spi, &rx, 1);
		val = rx;
		val <<= 8;
	}

	spi_read(spi, &rx, 1);
	val |= rx;
	val <<= 8;

	spi_read(spi, &rx, 1);
	val |= rx;


	if(ad77xx_init_status & AD77XX_STATUS_IS_AD7799)
		return val - 0x800000;

	return val - 0x8000;

}


/*
 * Determine if data is ready to be read
 *
 */
u8 ad77xx_data_ready(struct spi_device *spi)
{
	u8 status;

	status = ad77xx_status(spi);
	status &= AD77XX_STATUS_RDY;

	return !status;
}

/*
 * Calibrate device
 */
void ad77xx_calibrate(struct spi_device *spi)
{
	u32 off = ad77xx_read_offset(spi);
	dev_info(&spi->dev, "ad77xx: offset: 0x%x\n", off);

	/* cal */
	ad77xx_set_mode(spi, AD77XX_INTERNAL_OFFSET_CAL_MODE, 0, AD77XX_16_7_1_HZ);
	//while(!ad77xx_data_ready(spi));

	off = ad77xx_read_offset(spi);
	dev_info(&spi->dev, "offset: 0x%x\n", off);
}

/*
 * Initialize AD77XX
 */
void ad77xx_init(struct spi_device *spi)
{
   ad77xx_init_status = ad77xx_status(spi);
}

/*
 * Handle IOCTL commands
 */
int ad77xx_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{

	struct ad77xx_priv *priv = file->private_data;
	u8 data[4];
	int ret = 0;

	switch (cmd) {
	case AD77XX_WRITE_REGISTER:
		if(copy_from_user(data, (u8 *)arg, 4)) {
			ret = -ENOMEM;
		}

		/* Tell the chip wich register we want to deal with */
		ad77xx_comm(priv->spi_dev, data[0], 0, 0);

		/* 16bit register? */
		if(data[0] == AD77XX_MODE_REG || data[0] == AD77XX_CONFIG_REG)
			spi_write(priv->spi_dev, data + 1, 2);
		else
			spi_write(priv->spi_dev, data + 1, 1);

		break;

	case AD77XX_READ_REGISTER:
		if(copy_from_user(data, (u8 *)arg, 4)) {
			ret = -ENOMEM;
		}

		/* Tell the chip wich register we want to deal with */
		ad77xx_comm(priv->spi_dev, data[0], 1, 0);

		/* 16bit register? */
		if(data[0] == AD77XX_MODE_REG || data[0] == AD77XX_CONFIG_REG)
			spi_read(priv->spi_dev, &ret, 2);
		else
			spi_read(priv->spi_dev, &ret, 1);
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

static ssize_t ad77xx_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	s32 val;

	struct ad77xx_priv *priv = file->private_data;

	if(count < 4) {
		return 0;
	}


	ad77xx_request_data(priv->spi_dev,1);
	val = ad77xx_read_data(priv->spi_dev);

	if(copy_to_user(buf, &val, 4)) {
		return 0;
	}

	return 4;
}


static int ad77xx_open(struct inode *inode, struct file *file)
{
	struct ad77xx_priv *priv = container_of(inode->i_cdev, struct ad77xx_priv, cdev);

	file->private_data = priv;

	return 0;
}

static int ad77xx_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations ad77xx_ops = {
	.owner		= THIS_MODULE,
	.read		= ad77xx_read,
	.open		= ad77xx_open,
	.release	= ad77xx_release,
	.ioctl		= ad77xx_ioctl,
};

static int __devinit ad77xx_probe(struct spi_device *spi)
{
	struct ad77xx_priv *priv;
	int err;

	dev_info(&spi->dev, "Probing ad77xx..\n");

	priv = kzalloc(sizeof(struct ad77xx_priv), GFP_KERNEL);

	if (!priv) {
		err = -ENOMEM;
		goto exit_free;
	}

	/* Initialize spi and read device id */

	dev_info(&spi->dev, "Initialize..\n");
	ad77xx_init(spi);

	dev_info(&spi->dev, "Calibrate..\n");
	ad77xx_calibrate(spi);

	dev_info(&spi->dev, "Set mode..\n");
	ad77xx_set_mode(spi, AD77XX_CONTINUOUS_CONVERSION_MODE, 0, AD77XX_16_7_1_HZ);

	dev_info(&spi->dev, "Writing config..\n");
	ad77xx_write_config(spi,
						0,					/* No burn out */
						1,					/* Unipolar */
						AD77XX_1_GAIN,		/* 1 Gain */
						0,					/* No reference detection */
						1,					/* Buffered */
						AD77XX_AIN1_CHAN	/* AIN1 Channel */
						);

	priv->spi_dev = spi;
	dev_set_drvdata(&spi->dev, priv);


	alloc_chrdev_region(&priv->dev, 0, 1, "ad77xx");
	cdev_init(&priv->cdev, &ad77xx_ops);
	priv->cdev.owner = THIS_MODULE;
	priv->cdev.ops = &ad77xx_ops;

	cdev_add(&priv->cdev, priv->dev, 1);
	priv->device = device_create(ad77xx_class, NULL, priv->dev, NULL, "ad77xx");

	dev_info(&spi->dev, "ad77xx device registered\n");

	return 0;

exit_free:
	kzfree(priv);
	return err;
}


static int __devexit ad77xx_remove(struct spi_device *spi)
{

	struct ad77xx_priv *priv = dev_get_drvdata(&spi->dev);

	device_del(priv->device);
	cdev_del(&priv->cdev);
	unregister_chrdev_region(priv->dev, 1);

	class_destroy(ad77xx_class);
	return 0;
}

static struct spi_driver ad77xx_driver = {
	.probe	= ad77xx_probe,
	.remove	= ad77xx_remove,
	.driver = {
		.name	= "ad77xx",
		.owner	= THIS_MODULE,
	},
};

static int ad77xx_init_module(void)
{
	ad77xx_class = class_create(THIS_MODULE, "ad77xx");

	if(ad77xx_class == NULL) {
		printk("Could not create class counter\n");
		return -ENOMEM;
	}

	return spi_register_driver(&ad77xx_driver);
}

static void ad77xx_cleanup_module(void)
{
	spi_unregister_driver(&ad77xx_driver);
}

module_init(ad77xx_init_module);
module_exit(ad77xx_cleanup_module);


