
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <mach/ssp.h>
#include <mach/regs-ssp.h>
#include <linux/platform_device.h>

MODULE_DESCRIPTION("Microchip MCP300x A/D Converters");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Kiland <david.kiland@combitech.se>");


#define MCP3001		1
#define MCP3002		2
#define MCP3008		8

static const struct platform_device_id mcp300x_id[] = {
	{ "mcp3001", MCP3001 },
	{ "mcp3002", MCP3002 },
	{ "mcp3008", MCP3008 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mcp300x_id);


struct class *mcp300x_class;

struct mcp300x_priv {
	struct ssp_dev 	spi_dev;
	int cs_gpio;
	struct cdev cdev;
	struct device *device;
	dev_t dev;
	int no_of_channels;
};


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


static int read_mcp3008(struct mcp300x_priv *priv, u16 *values)
{
	u8 i;
	u8 rx[3];



	for(i=0; i<priv->no_of_channels; i++) {
		values[i] = 0;
		gpio_set_value(priv->cs_gpio, 0);
		spi_write_byte(&priv->spi_dev, 0x01);
		spi_read_byte(&priv->spi_dev, 0x80 | (i<<4), &rx[0]);
		spi_read_byte(&priv->spi_dev, 0xff, &rx[1]);
		values[i] = ((rx[0]&0x3)<<8) | rx[1];
		gpio_set_value(priv->cs_gpio, 1);
		printk("Read ch %i: %i\n", i, values[i]);

	}




	return (2*priv->no_of_channels);
}



static int mcp300x_read_values(struct mcp300x_priv *priv, u16 *values)
{
	u8 rx[16];


	switch(priv->no_of_channels) {
	case 1:
		gpio_set_value(priv->cs_gpio, 0);
		spi_read_byte(&priv->spi_dev, 0xff, &rx[0]);
		spi_read_byte(&priv->spi_dev, 0xff, &rx[1]);
		values[0] = ((rx[0]&0x1f)<<5) | (((rx[1]&0xf8)>>3)&0x1f);
		gpio_set_value(priv->cs_gpio, 1);
		printk("Read value %i\n", values[0]);
		return 2;
	case 2:
	case 4:
	case 8:
		return read_mcp3008(priv, values);
	default:
		break;
	}


	return 0;
}


static ssize_t mcp300x_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	u16 val[8];
	struct mcp300x_priv *priv = file->private_data;
	int size;
	printk("read mcp300x\n");
	if(count < (priv->no_of_channels*2)) {
		/* Must read all A/D values */
		return 0;
	}

	size = mcp300x_read_values(priv, val);

	if(copy_to_user(buf, val, size)) {
		return -ENOMEM;
	}

	return size;
}

static ssize_t mcp300x_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	return 0;
}


static int mcp300x_open(struct inode *inode, struct file *file)
{
	struct mcp300x_priv *priv = container_of(inode->i_cdev, struct mcp300x_priv, cdev);

	file->private_data = priv;

	return 0;
}

static int mcp300x_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations mcp300x_ops = {
	.owner		= THIS_MODULE,
	.read		= mcp300x_read,
	.write		= mcp300x_write,
	.open		= mcp300x_open,
	.release	= mcp300x_release,
};

static int __devinit mcp300x_probe(struct platform_device *pdev)
{
	struct mcp300x_priv *priv;
	struct platform_device_id *id;
	struct resource *r;

	dev_info(&pdev->dev, "Probing mcp300x..\n");

	priv = kzalloc(sizeof(struct mcp300x_priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "Could not allocate memory\n");
		return -ENOMEM;
	}

	/* Get no of channels */
	id = (struct platform_device_id *)platform_get_device_id(pdev);
	priv->no_of_channels = id->driver_data;

	dev_set_drvdata(&pdev->dev, priv);

	if(ssp_init(&priv->spi_dev, 3, SSP_NO_IRQ) == -ENODEV) {
		printk("Could not allocate device\n");
		goto exit_free;
	}

	ssp_disable(&priv->spi_dev);

	ssp_config(&priv->spi_dev, 	SSCR0_DataSize(8) | SSCR0_Motorola,
			SSCR1_TxTresh(1) | SSCR1_RxTresh(1),
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


	alloc_chrdev_region(&priv->dev, 0, priv->no_of_channels, id->name);

	cdev_init(&priv->cdev, &mcp300x_ops);
	priv->cdev.owner = THIS_MODULE;
	priv->cdev.ops = &mcp300x_ops;
	cdev_add(&priv->cdev, priv->dev, priv->no_of_channels);

	priv->device = device_create(mcp300x_class, NULL, priv->dev, NULL, id->name);
	printk(KERN_INFO "mcp300x device registered\n");

	return 0;

exit_free:
	return -1;
}




static int __devexit mcp300x_remove(struct platform_device *pdev)
{
	struct mcp300x_priv *priv = dev_get_drvdata(&pdev->dev);

	device_del(priv->device);
	cdev_del(&priv->cdev);
	unregister_chrdev_region(priv->dev, priv->no_of_channels);

	return 0;
}

static struct platform_driver mcp300x_driver = {
	.probe	= mcp300x_probe,
	.remove	= mcp300x_remove,
	.id_table = mcp300x_id,
	.driver = {
		.name	= "mcp300x",
		.owner	= THIS_MODULE,
	},
};

static __init int mcp300x_init_module(void)
{
	mcp300x_class = class_create(THIS_MODULE, "mcp300x");

	if(mcp300x_class == NULL) {
		printk("Could not create class counter\n");
		return -ENOMEM;
	}

	return platform_driver_register(&mcp300x_driver);
}

static __exit void mcp300x_cleanup_module(void)
{
	platform_driver_unregister(&mcp300x_driver);
}

module_init(mcp300x_init_module);
module_exit(mcp300x_cleanup_module);

MODULE_ALIAS("spi:mcp300x");
