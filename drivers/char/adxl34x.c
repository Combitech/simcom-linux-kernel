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


/* ADXL345/6 Register Map */
#define DEVID			0x00	/* R   Device ID */
#define THRESH_TAP		0x1D	/* R/W Tap threshold */
#define OFSX			0x1E	/* R/W X-axis offset */
#define OFSY			0x1F	/* R/W Y-axis offset */
#define OFSZ			0x20	/* R/W Z-axis offset */
#define DUR				0x21	/* R/W Tap duration */
#define LATENT			0x22	/* R/W Tap latency */
#define WINDOW			0x23	/* R/W Tap window */
#define THRESH_ACT		0x24	/* R/W Activity threshold */
#define THRESH_INACT	0x25	/* R/W Inactivity threshold */
#define TIME_INACT		0x26	/* R/W Inactivity time */
#define ACT_INACT_CTL	0x27	/* R/W Axis enable control for activity and */

/* inactivity detection */
#define THRESH_FF		0x28	/* R/W Free-fall threshold */
#define TIME_FF			0x29	/* R/W Free-fall time */
#define TAP_AXES		0x2A	/* R/W Axis control for tap/double tap */
#define ACT_TAP_STATUS	0x2B	/* R   Source of tap/double tap */
#define BW_RATE			0x2C	/* R/W Data rate and power mode control */
#define POWER_CTL		0x2D	/* R/W Power saving features control */
#define INT_ENABLE		0x2E	/* R/W Interrupt enable control */
#define INT_MAP			0x2F	/* R/W Interrupt mapping control */
#define INT_SOURCE		0x30	/* R   Source of interrupts */
#define DATA_FORMAT		0x31	/* R/W Data format control */
#define DATAX0			0x32	/* R   X-Axis Data 0 */
#define DATAX1			0x33	/* R   X-Axis Data 1 */
#define DATAY0			0x34	/* R   Y-Axis Data 0 */
#define DATAY1			0x35	/* R   Y-Axis Data 1 */
#define DATAZ0			0x36	/* R   Z-Axis Data 0 */
#define DATAZ1			0x37	/* R   Z-Axis Data 1 */
#define FIFO_CTL		0x38	/* R/W FIFO control */
#define FIFO_STATUS		0x39	/* R   FIFO status */
#define TAP_SIGN		0x3A	/* R   Sign and source for tap/double tap */

/* Orientation ADXL346 only */
#define ORIENT_CONF		0x3B	/* R/W Orientation configuration */
#define ORIENT			0x3C	/* R   Orientation status */

/* DEVIDs */
#define ID_ADXL345		0xE5
#define ID_ADXL346		0xE6

/* INT_ENABLE/INT_MAP/INT_SOURCE Bits */
#define DATA_READY		(1 << 7)
#define SINGLE_TAP		(1 << 6)
#define DOUBLE_TAP		(1 << 5)
#define ACTIVITY		(1 << 4)
#define INACTIVITY		(1 << 3)
#define FREE_FALL		(1 << 2)
#define WATERMARK		(1 << 1)
#define OVERRUN			(1 << 0)

/* ACT_INACT_CONTROL Bits */
#define ACT_ACDC		(1 << 7)
#define ACT_X_EN		(1 << 6)
#define ACT_Y_EN		(1 << 5)
#define ACT_Z_EN		(1 << 4)
#define INACT_ACDC		(1 << 3)
#define INACT_X_EN		(1 << 2)
#define INACT_Y_EN		(1 << 1)
#define INACT_Z_EN		(1 << 0)

/* TAP_AXES Bits */
#define SUPPRESS		(1 << 3)
#define TAP_X_EN		(1 << 2)
#define TAP_Y_EN		(1 << 1)
#define TAP_Z_EN		(1 << 0)

/* ACT_TAP_STATUS Bits */
#define ACT_X_SRC		(1 << 6)
#define ACT_Y_SRC		(1 << 5)
#define ACT_Z_SRC		(1 << 4)
#define ASLEEP			(1 << 3)
#define TAP_X_SRC		(1 << 2)
#define TAP_Y_SRC		(1 << 1)
#define TAP_Z_SRC		(1 << 0)

/* BW_RATE Bits */
#define LOW_POWER		(1 << 4)
#define RATE(x)			((x) & 0xF)

/* POWER_CTL Bits */
#define PCTL_LINK		(1 << 5)
#define PCTL_AUTO_SLEEP (1 << 4)
#define PCTL_MEASURE	(1 << 3)
#define PCTL_SLEEP		(1 << 2)
#define PCTL_WAKEUP(x)	((x) & 0x3)

/* DATA_FORMAT Bits */
#define SELF_TEST		(1 << 7)
#define SPI				(1 << 6)
#define INT_INVERT		(1 << 5)
#define FULL_RES		(1 << 3)
#define JUSTIFY			(1 << 2)
#define RANGE(x)		((x) & 0x3)
#define RANGE_PM_2g		0
#define RANGE_PM_4g		1
#define RANGE_PM_8g		2
#define RANGE_PM_16g	3

/*
 * Maximum value our axis may get in full res mode for the input device
 * (signed 13 bits)
 */
#define ADXL_FULLRES_MAX_VAL 4096

/*
 * Maximum value our axis may get in fixed res mode for the input device
 * (signed 10 bits)
 */
#define ADXL_FIXEDRES_MAX_VAL 512

/* FIFO_CTL Bits */
#define FIFO_MODE(x)	(((x) & 0x3) << 6)
#define FIFO_BYPASS		0
#define FIFO_FIFO		1
#define FIFO_STREAM		2
#define FIFO_TRIGGER	3
#define TRIGGER			(1 << 5)
#define SAMPLES(x)		((x) & 0x1F)

/* FIFO_STATUS Bits */
#define FIFO_TRIG		(1 << 7)
#define ENTRIES(x)		((x) & 0x3F)

/* TAP_SIGN Bits ADXL346 only */
#define XSIGN			(1 << 6)
#define YSIGN			(1 << 5)
#define ZSIGN			(1 << 4)
#define XTAP			(1 << 3)
#define YTAP			(1 << 2)
#define ZTAP			(1 << 1)

/* ORIENT_CONF ADXL346 only */
#define ORIENT_DEADZONE(x)	(((x) & 0x7) << 4)
#define ORIENT_DIVISOR(x)	((x) & 0x7)

/* ORIENT ADXL346 only */
#define ADXL346_2D_VALID			(1 << 6)
#define ADXL346_2D_ORIENT(x)		(((x) & 0x3) >> 4)
#define ADXL346_3D_VALID			(1 << 3)
#define ADXL346_3D_ORIENT(x)		((x) & 0x7)
#define ADXL346_2D_PORTRAIT_POS		0	/* +X */
#define ADXL346_2D_PORTRAIT_NEG		1	/* -X */
#define ADXL346_2D_LANDSCAPE_POS	2	/* +Y */
#define ADXL346_2D_LANDSCAPE_NEG	3	/* -Y */

#define ADXL346_3D_FRONT			3	/* +X */
#define ADXL346_3D_BACK				4	/* -X */
#define ADXL346_3D_RIGHT			2	/* +Y */
#define ADXL346_3D_LEFT				5	/* -Y */
#define ADXL346_3D_TOP				1	/* +Z */
#define ADXL346_3D_BOTTOM			6	/* -Z */

#undef ADXL_DEBUG

#define ADXL_X_AXIS		0
#define ADXL_Y_AXIS		1
#define ADXL_Z_AXIS		2

#define WRITE_SINGLE	0x00
#define WRITE_MULT		0x40
#define READ_SINGLE		0x80
#define READ_MULT		0xc0


struct adxl34x_priv {
	struct ssp_dev *spi_dev;
	int cs_gpio;
	int type;
	struct cdev cdev;
	struct device *device;
	dev_t dev;
};

static struct class *adxl_class = 0;



static int spi_read_byte(struct ssp_dev *dev, u8 output, u8 *data)
{
	u32 d = 0;

	ssp_write_word(dev, output);
	ssp_read_word(dev, &d);
	ssp_flush(dev);
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

	gpio_set_value(priv->cs_gpio, 0);

	spi_write_byte(&priv->spi_dev, tx[0]);
	spi_read_byte(&priv->spi_dev, 0xff, &rx[0]);

	gpio_set_value(priv->cs_gpio, 1);
	//spi_write_then_read(priv->spi_dev, tx, 1, rx, 1);
	return rx[0];
}


static int adxl34x_read_reg(struct adxl34x_priv *priv, u8 reg)
{
	u8 tx[1];
	u8 rx[1];

	tx[0] = READ_SINGLE | reg;

	gpio_set_value(priv->cs_gpio, 0);

	spi_write_byte(&priv->spi_dev, tx[0]);
	spi_read_byte(&priv->spi_dev, 0xff, &rx[0]);

	gpio_set_value(priv->cs_gpio, 1);
	//spi_write_then_read(priv->spi_dev, tx, 1, rx, 1);
	return rx[0];
}


static void adxl34x_write_reg(struct adxl34x_priv *priv, u8 reg, u8 value)
{
	u8 tx[1];
	u8 rx[1];

	tx[0] = WRITE_SINGLE | reg;

	gpio_set_value(priv->cs_gpio, 0);

	spi_write_byte(&priv->spi_dev, tx[0]);
	spi_write_byte(&priv->spi_dev, value);

	gpio_set_value(priv->cs_gpio, 1);
}



static ssize_t adxl34x_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	u8 rx[6];
	u8 tx[1];

	struct adxl34x_priv *priv = file->private_data;

	if(count < 6) {
		return 0;
	}

	rx[0] = adxl34x_read_id(priv);
	printk("ID = 0x%02x \n", rx[0]);
	return 0;

	tx[0] = READ_MULT | DATAX0;

	gpio_set_value(priv->cs_gpio, 0);
	spi_write_byte(&priv->spi_dev, tx[0]);
	spi_read_byte(&priv->spi_dev, 0xff,  &rx[0]);
	spi_read_byte(&priv->spi_dev, 0xff,  &rx[1]);
	spi_read_byte(&priv->spi_dev, 0xff,  &rx[2]);
	spi_read_byte(&priv->spi_dev, 0xff,  &rx[3]);
	spi_read_byte(&priv->spi_dev, 0xff,  &rx[4]);
	spi_read_byte(&priv->spi_dev, 0xff,  &rx[5]);
	gpio_set_value(priv->cs_gpio, 1);

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
	struct resource *r;
	u8 id;

	dev_info(&pdev->dev, "Probing adxl34x..\n");

	priv = kzalloc(sizeof(struct adxl34x_priv), GFP_KERNEL);

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

	if(gpio_request(priv->cs_gpio, "adxl346 chipselect") < 0) {
		printk("Could not request chipselect pin for adxl34x\n");
		goto exit_free;
	}

	gpio_direction_output(priv->cs_gpio, 1);
	gpio_set_value(priv->cs_gpio, 1);

	id = adxl34x_read_id(priv);
	dev_info(&pdev->dev, "Read device ID: 0x%02x", id);

	adxl34x_write_reg(priv, FIFO_CTL, FIFO_MODE(FIFO_STREAM) | SAMPLES(32));


	alloc_chrdev_region(&priv->dev, 0, 1, "adxl34x");
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
	.driver = {
		.name	= "adxl34x",
		.owner	= THIS_MODULE,
	},
};

static __init int adxl34x_init_module(void)
{
	adxl_class = class_create(THIS_MODULE, "adxl34x");

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
