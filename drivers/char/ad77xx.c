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
#include <linux/platform_device.h>
#include <mach/ssp.h>
#include <mach/regs-ssp.h>


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
	struct ssp_dev spi_dev;
	int cs_gpio;
	struct cdev cdev;
	struct device *device;
	dev_t dev;
	u8 init_status;
};

static struct class *ad77xx_class = 0;

/*
 * Write to Communication Register
 *
 * @param reg Register
 * @param read Is the next operation read?
 * @param cont Continuous mode
 */


static int spi_read_byte(struct ad77xx_priv *priv, u8 output, u8 *data)
{
	u32 d = 0;


	ssp_write_word(&priv->spi_dev, output);
	ssp_read_word(&priv->spi_dev, &d);
	ssp_flush(&priv->spi_dev);
	*data = d;
	return 0;
}

static int spi_write_byte(struct ad77xx_priv *priv, u8 data)
{
	ssp_write_word(&priv->spi_dev, data);
	ssp_flush(&priv->spi_dev);
	return 0;
}


static inline void ad77xx_comm(struct ad77xx_priv *priv, u8 reg, u8 read, u8 cont)
{
	u8 byte = (read ? 0x40 : 0x00) | (reg << 3) | (cont ? 0x04 : 0x00);
	spi_write_byte(priv, byte);
}


/*
 * Read Status Register
 */
static u8 ad77xx_status(struct ad77xx_priv *priv)
{
	u8 rx;

	ad77xx_comm(priv, AD77XX_STATUS_REG, 1, 0);

	spi_read_byte(priv, 0xff, &rx);

	return rx;
}


/*
 * Read Status Register
 */
static u8 ad77xx_read_id(struct ad77xx_priv *priv)
{
	u8 rx;

	ad77xx_comm(priv, AD77XX_ID_REG, 1, 0);

	spi_read_byte(priv, 0xff, &rx);

	printk("Device id is 0x%x\n", rx);

	return rx;
}


/*
 * Read Status Register
 */
static void ad77xx_reset(struct ad77xx_priv *priv)
{

	spi_write_byte(priv, 0xff);
	spi_write_byte(priv, 0xff);
	spi_write_byte(priv, 0xff);
	spi_write_byte(priv, 0xff);
	spi_write_byte(priv, 0xff);

	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout(HZ/2);

}


/*
 * Read Offset Register
 */
u32 ad77xx_read_offset(struct ad77xx_priv *priv)
{
	u8 rx;
	u32 val = 0;

	ad77xx_comm(priv, AD77XX_OFFSET_REG, 1, 0);

	if(priv->init_status & AD77XX_STATUS_IS_AD7799)
	{
		spi_read_byte(priv, 0xff, &rx);
		val |= rx << 16;
	}

	spi_read_byte(priv, 0xff, &rx);
	val |= rx << 8;

	spi_read_byte(priv, 0xff, &rx);
	val |= rx;


	return val;
}

/*
 * Read Full Scale Register
 */
u32 ad77xx_read_scale(struct ad77xx_priv *priv)
{
	u32 val = 0;
	u8 rx;

	ad77xx_comm(priv, AD77XX_SCALE_REG, 1, 0);

	if(priv->init_status & AD77XX_STATUS_IS_AD7799)
	{
		spi_read_byte(priv, 0xff, &rx);
		val |= rx << 16;
	}

	spi_read_byte(priv, 0xff, &rx);
	val |= rx << 8;

	spi_read_byte(priv, 0xff, &rx);
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
static void ad77xx_set_mode(struct ad77xx_priv *priv, u8 mode, u8 psw, u8 rate)
{
	u8 tx;

    ad77xx_comm(priv, AD77XX_MODE_REG, 0, 0);

	tx = mode << 5 | (psw ? 0x10 : 0x00);
    spi_write_byte(priv, tx);
	spi_write_byte(priv, rate);
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
void ad77xx_write_config(	struct ad77xx_priv *priv,
							u8 burnout, u8 unipolar, u8 gain,
							u8 ref_det, u8 buf, u8 chan)
{
	u8 tx;

	ad77xx_comm(priv, AD77XX_CONFIG_REG, 0, 1);

	tx = ((burnout ? 0x20 : 0x00) | (unipolar ? 0x10 : 0x00) | gain);
	spi_write_byte(priv, tx);

	tx = ((ref_det ? 0x20 : 0x00) | (buf ? 0x10 : 0x00) | chan);
	spi_write_byte(priv, tx);
}


/*
 *  Request a read from the data register
 *
 *@param continuous Continuous reading
 */
void ad77xx_request_data(struct ad77xx_priv *priv, u8 continuous)
{
	ad77xx_comm(priv, AD77XX_DATA_REG, 1, continuous);
}



/*
 * Determine if data is ready to be read
 *
 */
u8 ad77xx_data_ready(struct ad77xx_priv *priv)
{
	u8 status;

	status = ad77xx_status(priv);
	status &= AD77XX_STATUS_RDY;

	return !status;
}


/* Read from data register, it should be previously requested
 *   from ad77xx_request_data. The value is signed!!
 *
 */
s32 ad77xx_read_data(struct ad77xx_priv *priv)
{
	s32 val = 0;
	u8 rx;

	//while(!ad77xx_data_ready(priv));

	//if(priv->init_status & AD77XX_STATUS_IS_AD7799)
	//{
		spi_read_byte(priv, 0xff, &rx);
		val = rx;
		val <<= 8;
	//}

	spi_read_byte(priv, 0xff, &rx);
	val |= rx;
	val <<= 8;

	spi_read_byte(priv, 0xff, &rx);
	val |= rx;

	return val;

}



/*
 * Calibrate device
 */
void ad77xx_calibrate(struct ad77xx_priv *priv)
{
	u32 off = ad77xx_read_offset(priv);
	printk("ad77xx: offset: 0x%x\n", off);

	/* cal */
	ad77xx_set_mode(priv, AD77XX_INTERNAL_OFFSET_CAL_MODE, 0, AD77XX_470_HZ);

	//while(!ad77xx_data_ready(spi));

	off = ad77xx_read_offset(priv);
	printk("offset: 0x%x\n", off);
}

/*
 * Initialize AD77XX
 */
void ad77xx_init(struct ad77xx_priv *priv)
{
	priv->init_status = ad77xx_status(priv);
	printk("Init status is %x\n", priv->init_status);
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

		gpio_set_value(priv->cs_gpio, 0);
		/* Tell the chip wich register we want to deal with */
		ad77xx_comm(priv, data[0], 0, 0);

		/* 16bit register? */
		if(data[0] == AD77XX_MODE_REG || data[0] == AD77XX_CONFIG_REG) {
			spi_write_byte(priv, data[1]);
			spi_write_byte(priv, data[2]);
		}
		else {
			spi_write_byte(priv, data[1]);
		}
		gpio_set_value(priv->cs_gpio, 1);
		break;

	case AD77XX_READ_REGISTER:
		if(copy_from_user(data, (u8 *)arg, 4)) {
			ret = -ENOMEM;
		}

		gpio_set_value(priv->cs_gpio, 0);
		/* Tell the chip wich register we want to deal with */
		ad77xx_comm(priv, data[0], 1, 0);

		/* 16bit register? */
		if(data[0] == AD77XX_MODE_REG || data[0] == AD77XX_CONFIG_REG) {
			spi_read_byte(priv, 0xff, &data[0]);
			spi_read_byte(priv, 0xff, &data[1]);
			ret = (data[1]<<8) | data[0];
		}
		else {
			spi_read_byte(priv, 0xff, &data[0]);
			ret = data[0];
		}
		gpio_set_value(priv->cs_gpio, 1);
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

	gpio_set_value(priv->cs_gpio, 0);
	ad77xx_request_data(priv, 1);
	while(gpio_get_value(41));
	val = ad77xx_read_data(priv);
	gpio_set_value(priv->cs_gpio, 1);

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

static int __devinit ad77xx_probe(struct platform_device *pdev)
{
	struct ad77xx_priv *priv;
	int err;
	struct resource *r;

	printk(KERN_INFO "Probing ad77xx..\n");

	priv = kzalloc(sizeof(struct ad77xx_priv), GFP_KERNEL);

	if (!priv) {
		err = -ENOMEM;
		goto exit_free;
	}


	dev_set_drvdata(&pdev->dev, priv);

	if(ssp_init(&priv->spi_dev, 3, SSP_NO_IRQ) == -ENODEV) {
		printk("Could not allocate device\n");
		goto exit_free;
	}

	ssp_disable(&priv->spi_dev);

	ssp_config(&priv->spi_dev, 	SSCR0_DataSize(8) | SSCR0_Motorola,
			SSCR1_TxTresh(1) | SSCR1_RxTresh(1) | SSCR1_SPO | SSCR1_SPH,
			0,
			SSCR0_SCR & SSCR0_SerClkDiv(6));

	ssp_enable(&priv->spi_dev);


	/* Get chip select signal from platform resource */
	r = platform_get_resource(pdev, IORESOURCE_IO, 0);
	priv->cs_gpio = r->start;
	printk("Chipselect on gpio%i\n", priv->cs_gpio);

	if(gpio_request(priv->cs_gpio, "mcp300x chipselect") < 0) {
		printk("Could not request chipselect pin for mcp300x\n");
		goto exit_free;
	}

	gpio_direction_output(priv->cs_gpio, 1);
	gpio_set_value(priv->cs_gpio, 1);


	/* Initialize spi and read device id */

	gpio_set_value(priv->cs_gpio, 0);

	ad77xx_reset(priv);

	ad77xx_read_id(priv);

	dev_info(&pdev->dev, "Initialize..\n");
	ad77xx_init(priv);

	dev_info(&pdev->dev, "Set mode..\n");
	ad77xx_set_mode(priv, AD77XX_CONTINUOUS_CONVERSION_MODE, 0, AD77XX_470_HZ);

	dev_info(&pdev->dev, "Calibrate..\n");
	//ad77xx_calibrate(priv);

	dev_info(&pdev->dev, "Writing config..\n");
	ad77xx_write_config(priv,
			0,					/* No burn out */
			1,					/* Unipolar */
			AD77XX_1_GAIN,		/* 1 Gain */
			0,					/* No reference detection */
			1,					/* Buffered */
			AD77XX_AIN1_CHAN	/* AIN1 Channel */
	);

	gpio_set_value(priv->cs_gpio, 1);


	alloc_chrdev_region(&priv->dev, 0, 1, "ad77xx");
	cdev_init(&priv->cdev, &ad77xx_ops);
	priv->cdev.owner = THIS_MODULE;
	priv->cdev.ops = &ad77xx_ops;

	cdev_add(&priv->cdev, priv->dev, 1);
	priv->device = device_create(ad77xx_class, NULL, priv->dev, NULL, "ad77xx");

	dev_info(&pdev->dev, "ad77xx device registered\n");

	return 0;

exit_free:
	kzfree(priv);
	return err;
}


static int __devexit ad77xx_remove(struct platform_device *pdev)
{

	struct ad77xx_priv *priv = dev_get_drvdata(&pdev->dev);

	device_del(priv->device);
	cdev_del(&priv->cdev);
	unregister_chrdev_region(priv->dev, 1);

	class_destroy(ad77xx_class);
	return 0;
}

static struct platform_driver ad77xx_driver = {
	.probe	= ad77xx_probe,
	.remove	= ad77xx_remove,
	.driver = {
		.name	= "ad7799",
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

	return platform_driver_register(&ad77xx_driver);
}

static void ad77xx_cleanup_module(void)
{
	platform_driver_unregister(&ad77xx_driver);
}

module_init(ad77xx_init_module);
module_exit(ad77xx_cleanup_module);


