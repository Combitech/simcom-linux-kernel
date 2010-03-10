
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

MODULE_DESCRIPTION("Microchip MCP3002 A/D Converter");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Kiland <david.kiland@combitech.se>");

struct mcp3002_priv {
	struct cdev	dev;
	struct spi_device *spi_dev;
	int cs_gpio;
};

static int mcp3002_read_values(struct mcp3002_priv *priv, u16 *values)
{
	struct spi_transfer xfer;
	struct spi_message msg;
	u8 txch[2];
	u8 rxch[2];

	spi_message_init(&msg);
	txch[0] = 0xd0;
	xfer.tx_buf = &txch;
	xfer.rx_buf = &rxch;
	xfer.len = 2;
	spi_message_add_tail(&xfer, &msg);
	spi_sync(priv->spi_dev, &msg);
	values[0] = ((rxch[0]&0x03)<<2) | rxch[1];

	spi_message_init(&msg);
	txch[0] = 0xf0;
	xfer.tx_buf = &txch;
	xfer.rx_buf = &rxch;
	xfer.len = 2;
	spi_message_add_tail(&xfer, &msg);
	spi_sync(priv->spi_dev, &msg);
	values[1] = ((rxch[0]&0x03)<<2) | rxch[1];

	return 2;
}


static ssize_t mcp3002_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	u16 val[2];
	struct mcp3002_priv *priv = file->private_data;

	if(count < 4) {
		return 0;
	}

	mcp3002_read_values(priv, val);
	copy_to_user(buf, val, 4);

	return 4;
}

static ssize_t mcp3002_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	return 0;
}


static int mcp3002_open(struct inode *inode, struct file *file)
{
	struct mcp3002_priv *priv = container_of(inode->i_cdev, struct mcp3002_priv, dev);

	file->private_data = priv;

	return 0;
}

static int mcp3002_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations mcp3002_ops = {
	.owner		= THIS_MODULE,
	.read		= mcp3002_read,
	.write		= mcp3002_write,
	.open		= mcp3002_open,
	.release	= mcp3002_release,
};

static int __devinit mcp3002_probe(struct spi_device *spi)
{
	struct mcp3002_priv *priv;
	int err;
	int devno = MKDEV(10, 1);

	printk("Probing mcp3002..\n");

	priv = kzalloc(sizeof(struct mcp3002_priv), GFP_KERNEL);

	if (!priv) {
		err = -ENOMEM;
		goto exit_free;
	}

	cdev_init(&priv->dev, &mcp3002_ops);
	priv->dev.owner = THIS_MODULE;
	priv->dev.ops = &mcp3002_ops;

	priv->spi_dev = spi;

	printk("Setting driver data\n");
	dev_set_drvdata(&spi->dev, priv);

	printk("Adding chardev\n");
	cdev_add(&priv->dev, devno, 1);

	printk(KERN_INFO "mcp3002 device registered\n");

	return 0;

exit_free:
	return err;
}




static int __devexit mcp3002_remove(struct spi_device *spi)
{
	struct mcp3002_priv *priv = dev_get_drvdata(&spi->dev);

	return 0;
}

static struct spi_driver mcp3002_driver = {
	.probe	= mcp3002_probe,
	.remove	= mcp3002_remove,
	.driver = {
		.name	= "mcp3002",
		.owner	= THIS_MODULE,
	},
};

static __init int mcp3002_init_module(void)
{
	return spi_register_driver(&mcp3002_driver);
}

static __exit void mcp3002_cleanup_module(void)
{
	spi_unregister_driver(&mcp3002_driver);
}

module_init(mcp3002_init_module);
module_exit(mcp3002_cleanup_module);

MODULE_ALIAS("spi:mcp3002");
