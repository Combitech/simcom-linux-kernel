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
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/adis16135.h>
#include <mach/ssp.h>
#include <mach/regs-ssp.h>

MODULE_DESCRIPTION("Analog Devices adis16135 3-axis Digital Gyroscope");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Kiland <david.kiland@combitech.se>");



struct adis16135_priv {
	struct ssp_dev spi_dev;
	int cs_gpio;
	struct cdev cdev;
	struct device *device;
	dev_t dev;
};

static struct class *adis_class = 0;


static int spi_read_byte(struct adis16135_priv *priv, u8 output, u8 *data)
{
	u32 d = 0;


	ssp_write_word(&priv->spi_dev, output);
	ssp_read_word(&priv->spi_dev, &d);
	ssp_flush(&priv->spi_dev);
	*data = d;
	return 0;
}

static int spi_write_byte(struct adis16135_priv *priv, u8 data)
{
	ssp_write_word(&priv->spi_dev, data);
	ssp_flush(&priv->spi_dev);
	return 0;
}


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
	u8 values[6];
	s16 val[3];

	gpio_set_value(priv->cs_gpio, 0);

	spi_write_byte(priv, 0x06);
	spi_write_byte(priv, 0x00);

	spi_read_byte(priv, 0x04, &values[0]);
	spi_read_byte(priv, 0x00, &values[1]);

	spi_read_byte(priv, 0x02, &values[2]);
	spi_read_byte(priv, 0x00, &values[3]);

	spi_read_byte(priv, 0xff, &values[4]);
	spi_read_byte(priv, 0xff, &values[5]);

	gpio_set_value(priv->cs_gpio, 1);

	val[0] = (values[0]<<8) +  values[1];
	val[1] = (values[2]<<8) +  values[3];
	val[2] = (values[4]<<8) +  values[5];

	printk("values: %i, %i, %i\n", val[0], val[1], val[2]);

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

static int __devinit adis16135_probe(struct platform_device *pdev)
{
	struct adis16135_priv *priv;
	int err;
	struct resource *r;

	dev_info(&pdev->dev, "Probing adis16135..\n");

	priv = kzalloc(sizeof(struct adis16135_priv), GFP_KERNEL);

	if (!priv) {
		err = -ENOMEM;
		goto exit_free;
	}

	/* Initialize spi and read device id */
	dev_set_drvdata(&pdev->dev, priv);

	if(ssp_init(&priv->spi_dev, 3, SSP_NO_IRQ) == -ENODEV) {
		printk("Could not allocate device\n");
		goto exit_free;
	}

	ssp_disable(&priv->spi_dev);

	ssp_config(&priv->spi_dev, 	SSCR0_DataSize(8) | SSCR0_Motorola,
			SSCR1_TxTresh(1) | SSCR1_RxTresh(1) | SSCR1_SPO | SSCR1_SPH,
			0,
			SSCR0_SCR & SSCR0_SerClkDiv(16));

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



	alloc_chrdev_region(&priv->dev, 0, 1, "adxl");
	cdev_init(&priv->cdev, &adis16135_ops);
	priv->cdev.owner = THIS_MODULE;
	priv->cdev.ops = &adis16135_ops;

	cdev_add(&priv->cdev, priv->dev, 1);
	priv->device = device_create(adis_class, NULL, priv->dev, NULL, "adis16135");

	dev_info(&pdev->dev, "adis16135 device registered\n");

	return 0;

exit_free:
	kzfree(priv);
	return err;
}




static int __devexit adis16135_remove(struct platform_device *pdev)
{
	struct adis16135_priv *priv = dev_get_drvdata(&pdev->dev);

	device_del(priv->device);
	cdev_del(&priv->cdev);
	unregister_chrdev_region(priv->dev, 1);

	return 0;
}

static struct platform_driver adis16135_driver = {
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

	return platform_driver_register(&adis16135_driver);
}

static __exit void adis16135_cleanup_module(void)
{
	platform_driver_unregister(&adis16135_driver);
}

module_init(adis16135_init_module);
module_exit(adis16135_cleanup_module);

MODULE_ALIAS("spi:adis16135");
