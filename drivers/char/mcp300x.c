
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

MODULE_DESCRIPTION("Microchip MCP300x A/D Converters");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Kiland <david.kiland@combitech.se>");


#define MCP3001		1
#define MCP3002		2
#define MCP3008		8

static const struct spi_device_id mcp300x_id[] = {
	{ "mcp3001", MCP3001 },
	{ "mcp3002", MCP3002 },
	{ "mcp3008", MCP3008 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mcp300x_id);


struct class *mcp300x_class;

struct mcp300x_priv {
	struct spi_device *spi_dev;
	int cs_gpio;
	struct cdev cdev;
	struct device *device;
	dev_t dev;
	int no_of_channels;
};



static int read_mcp3008(struct mcp300x_priv *priv, u16 *values)
{
	u8 i;
	struct spi_transfer xfer;
	struct spi_message msg;
	u8 tx[3];
	u8 rx[3];


	for(i=0; i<priv->no_of_channels; i++) {
		spi_message_init(&msg);
		tx[0] = 0x01;	// Startbit;
		tx[1] = 0x80 | (i<<4);
		tx[2] = 0;
		xfer.tx_buf = &tx;
		xfer.rx_buf = &rx;
		xfer.len = 3;
		spi_message_add_tail(&xfer, &msg);
		spi_sync(priv->spi_dev, &msg);
		values[i] = ((rx[1]&0x7)<<8) | rx[2];
	}

	return (2*priv->no_of_channels);
}



static int mcp300x_read_values(struct mcp300x_priv *priv, u16 *values)
{
	u8 rx[16];


	switch(priv->no_of_channels) {
	case 1:
		spi_read(priv->spi_dev, rx, 2);
		values[0] = (rx[0]<<8) | rx[1];
		values[0] = (values[0]>>4)&0x3ff;
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

static int __devinit mcp300x_probe(struct spi_device *spi)
{
	struct mcp300x_priv *priv;
	struct spi_device_id *id;

	dev_info(&spi->dev, "Probing mcp300x..\n");

	priv = kzalloc(sizeof(struct mcp300x_priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&spi->dev, "Could not allocate memory\n");
		return -ENOMEM;
	}


	/* Get no of channels */
	id = (struct spi_device_id *)spi_get_device_id(spi);
	priv->no_of_channels = id->driver_data;

	priv->spi_dev = spi;
	dev_set_drvdata(&spi->dev, priv);

	alloc_chrdev_region(&priv->dev, 0, priv->no_of_channels, id->name);

	cdev_init(&priv->cdev, &mcp300x_ops);
	priv->cdev.owner = THIS_MODULE;
	priv->cdev.ops = &mcp300x_ops;
	cdev_add(&priv->cdev, priv->dev, priv->no_of_channels);

	priv->device = device_create(mcp300x_class, NULL, priv->dev, NULL, id->name);
	printk(KERN_INFO "mcp300x device registered\n");

	return 0;
}




static int __devexit mcp300x_remove(struct spi_device *spi)
{
	struct mcp300x_priv *priv = dev_get_drvdata(&spi->dev);

	device_del(priv->device);
	cdev_del(&priv->cdev);
	unregister_chrdev_region(priv->dev, priv->no_of_channels);

	return 0;
}

static struct spi_driver mcp300x_driver = {
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
	mcp300x_class = class_create(THIS_MODULE, "adxl");

	if(mcp300x_class == NULL) {
		printk("Could not create class counter\n");
		return -ENOMEM;
	}

	return spi_register_driver(&mcp300x_driver);
}

static __exit void mcp300x_cleanup_module(void)
{
	spi_unregister_driver(&mcp300x_driver);
}

module_init(mcp300x_init_module);
module_exit(mcp300x_cleanup_module);

MODULE_ALIAS("spi:mcp300x");
