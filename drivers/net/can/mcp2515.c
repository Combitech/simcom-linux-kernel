
#include <linux/module.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/spi/mcp2515.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/sched.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <net/rtnetlink.h>


MODULE_DESCRIPTION("Microchip MCP2515 CAN interface");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Kiland <david.kiland@combitech.se>");


#define CANSTAT		0x0E
#define CANCTRL		0x0F

#define TEC			0x1C
#define REC			0x1D



#define CNF3		0x28
#define CNF2		0x29
#define CNF1		0x2A

/* Interupt enable register */
#define CANINTE		0x2B
#define MERRE		(1<<7)
#define WAKIE		(1<<6)
#define ERRIE		(1<<5)
#define TX2IE		(1<<4)
#define TX1IE		(1<<3)
#define TX0IE		(1<<2)
#define RX1IE		(1<<1)
#define RX0IE		1

/* Interrupt flag register */
#define CANINTF		0x2C
#define MERRF		(1<<7)
#define WAKIF		(1<<6)
#define ERRIF		(1<<5)
#define TX2IF		(1<<4)
#define TX1IF		(1<<3)
#define TX0IF		(1<<2)
#define RX1IF		(1<<1)
#define RX0IF		1

#define EFLG		0x2D

#define TXBCTRL(x)	(0x30+0x10*x)
#define TXBSIDH(x)	(0x31+0x10*x)
#define TXBSIDL(x)	(0x32+0x10*x)
#define TXBDLC(x)	(0x35+0x10*x)
#define TXBDM(x)	(0x36+0x10*x)

#define RXBCTRL(x)	(0x60+0x10*x)
#define RXBSIDH(x)	(0x61+0x10*x)
#define RXBSIDL(x)	(0x62+0x10*x)
#define RXBDLC(x)	(0x65+0x10*x)
#define RXBDM(x)	(0x66+0x10*x)

#define RXMSIDH(x)	(0x20+x*4)
#define RXMSIDL(x)	(0x21+x*4)



/* Instructions set */
#define MCP_WRITE			0x02
#define MCP_READ			0x03
#define MCP_BITMOD			0x05
#define MCP_LOAD_TX0		0x40
#define MCP_LOAD_TX1		0x42
#define MCP_LOAD_TX2		0x44
#define MCP_RTS_TX(x)		(0x80 | (1<<x))
#define MCP_RTS_ALL			0x87
#define MCP_READ_RX0		0x90
#define MCP_READ_RX1		0x94
#define MCP_READ_STATUS		0xA0
#define MCP_RX_STATUS		0xB0
#define MCP_RESET			0xC0



struct mcp2515_priv {
	struct can_priv		can;
	struct net_device	*dev;
	struct napi_struct	napi;
	struct spi_device *spi_dev;
	struct sk_buff *tx_skb;
	struct work_struct tx_work;
	struct work_struct irq_work;
	struct workqueue_struct *work_queue;
	int tx_next;
	int cs_gpio;
	int reset_gpio;
};


static struct can_bittiming_const mcp2515_bittiming_const = {
	.name = "MCP2515",
	.tseg1_min	= 2,
	.tseg1_max	= 16,
	.tseg2_min	= 2,
	.tseg2_max	= 16,
	.sjw_max	= 4,
	.brp_min 	= 1,
	.brp_max	= 32,
	.brp_inc	= 1,
};


static int mcp2515_exec(struct mcp2515_priv *priv, int command)
{
	unsigned char b = command;
	u8 ret;
	gpio_set_value(priv->cs_gpio, 0);
	ret = spi_write(priv->spi_dev, &b, 1);
	gpio_set_value(priv->cs_gpio, 1);
	return ret;
}

static int mcp2515_read_buffer(struct mcp2515_priv *priv, struct can_frame *cf, int index)
{
	u8 tx[2];
	u8 rx[15];

	switch(index) {
	case 0:
		tx[0] = 0x90;
		break;
	case 1:
		tx[0] = 0x94;
		break;
	default:
		tx[0] = 0x90;
		break;
	}

	gpio_set_value(priv->cs_gpio, 0);
	spi_write_then_read(priv->spi_dev, tx, 1, rx, 13);
	gpio_set_value(priv->cs_gpio, 1);

	cf->can_id = (rx[0]<<3) | ((rx[1]&0xe0)>>5);
	cf->can_dlc = rx[4];
	cf->data[0] = rx[5];
	cf->data[1] = rx[6];
	cf->data[2] = rx[7];
	cf->data[3] = rx[8];
	cf->data[4] = rx[9];
	cf->data[5] = rx[10];
	cf->data[6] = rx[11];
	cf->data[7] = rx[12];

	return 0;
}

static int mcp2515_write_buffer(struct mcp2515_priv *priv, struct can_frame *cf, int index)
{
	u8 tx[14];
	switch(index) {
	case 0:
		tx[0] = 0x40;
		break;
	case 1:
		tx[0] = 0x42;
		break;
	case 2:
		tx[0] = 0x44;
		break;
	default:
		tx[0] = 0x40;
		break;
	}

	tx[1] = cf->can_id>>3;
	tx[2] = (cf->can_id&0x03)<<5;
	tx[3] = 0;
	tx[4] = 0;
	tx[5] = cf->can_dlc;

	memcpy(&tx[6], cf->data, cf->can_dlc);

	gpio_set_value(priv->cs_gpio, 0);
	spi_write(priv->spi_dev, tx, 6+cf->can_dlc);
	gpio_set_value(priv->cs_gpio, 1);


	return 0;
}


static int mcp2515_read_reg(struct mcp2515_priv *priv, int reg)
{
	u8 tx[2];
	u8 rx[1];
	tx[0] = MCP_READ;
	tx[1] = reg;
	gpio_set_value(priv->cs_gpio, 0);
	spi_write_then_read(priv->spi_dev, tx, 2, rx, 1);
	gpio_set_value(priv->cs_gpio, 1);
	return rx[0];
}

static int mcp2515_bit_modify(struct mcp2515_priv *priv, int reg, int bitno, int val)
{
	unsigned char b[4];
	int ret;
	b[0] = MCP_BITMOD;
	b[1] = reg;
	b[2] = (1<<bitno);
	b[3] = (val<<bitno);
	gpio_set_value(priv->cs_gpio, 0);
	ret = spi_write(priv->spi_dev, b, 4);
	gpio_set_value(priv->cs_gpio, 1);
	return ret;
}

static int mcp2515_write_reg(struct mcp2515_priv *priv, int reg, int val)
{
	unsigned char b[3];
	int ret;
	b[0] = MCP_WRITE;
	b[1] = reg;
	b[2] = val;
	gpio_set_value(priv->cs_gpio, 0);
	ret = spi_write(priv->spi_dev, b, 3);
	gpio_set_value(priv->cs_gpio, 1);
	return ret;
}


static inline int mcp2515_get_next_tx_buffer(struct mcp2515_priv *priv)
{
	priv->tx_next = (priv->tx_next < 3) ? priv->tx_next : 0;
	return priv->tx_next;
}


static void mcp2515_chip_start(struct net_device *dev)
{
	struct mcp2515_priv *priv = netdev_priv(dev);

	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(200);

	mcp2515_exec(priv, MCP_RESET);

	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(50);

	mcp2515_write_reg(priv, CNF1, ((1<<6) | 9));
	mcp2515_write_reg(priv, CNF2, ((1<<7) | (1<<6) | (4<<3)));
	mcp2515_write_reg(priv, CNF3, 2);

	mcp2515_write_reg(priv, RXMSIDH(0), 0x00);
	mcp2515_write_reg(priv, RXMSIDL(0), 0x00);

	mcp2515_write_reg(priv, RXMSIDH(1), 0x00);
	mcp2515_write_reg(priv, RXMSIDL(1), 0x00);

	/* Set normal mode */
	mcp2515_write_reg(priv, CANCTRL, 0x00);

	mcp2515_write_reg(priv, RXBCTRL(0), 0x64);
	mcp2515_write_reg(priv, RXBCTRL(1), 0x60);

	/* Setup interrupts */
	mcp2515_write_reg(priv, CANINTE, (RX0IE | RX1IE | TX0IE));
	mcp2515_write_reg(priv, CANINTF, 0);
	mcp2515_write_reg(priv, EFLG, 0);
}

static void mcp2515_chip_stop(struct net_device *dev)
{
	struct mcp2515_priv *priv = netdev_priv(dev);
	printk("mcp2515 stop\n");
	mcp2515_exec(priv, MCP_RESET);

	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(50);
}


static int mcp2515_set_bittiming(struct net_device *dev)
{
	struct mcp2515_priv *priv = netdev_priv(dev);
	struct can_bittiming *bt = &priv->can.bittiming;
	int reg_cnf1, reg_cnf2, reg_cnf3;

	printk("mcp2515 set bittiming\n");
	reg_cnf1 = (bt->sjw&0x3)<<6 | (bt->brp&0x3f);
	mcp2515_write_reg(priv, CNF1, reg_cnf1);

	reg_cnf2 = (1<<7) | (1<<6) | (((bt->phase_seg1-1)&0x7)<<5) | (bt->prop_seg&0x7);
	mcp2515_write_reg(priv, CNF2, reg_cnf2);

	reg_cnf3 = bt->phase_seg2&0x7;
	mcp2515_write_reg(priv, CNF3, reg_cnf3);

	return 0;
}



static void mcp2515_irq_handler(struct work_struct *work)
{
	struct mcp2515_priv *priv =	container_of(work, struct mcp2515_priv, irq_work);
	struct net_device *dev = priv->dev;
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	u8 flags;
	gpio_set_value(priv->cs_gpio, 1);
	while(!gpio_get_value(priv->spi_dev->irq))
	{
	flags = mcp2515_read_reg(priv, CANINTF);


		if(flags&RX0IF) {
			skb = netdev_alloc_skb(dev, sizeof(struct can_frame));
			skb->protocol = htons(ETH_P_CAN);
			skb->ip_summed = CHECKSUM_UNNECESSARY;
			cf = (struct can_frame *)skb_put(skb, sizeof(struct can_frame));

			mcp2515_read_buffer(priv, cf, 0);

			stats->rx_packets++;
			stats->rx_bytes += cf->can_dlc;
			netif_receive_skb(skb);
		}

		if(flags&RX1IF) {
			skb = netdev_alloc_skb(dev, sizeof(struct can_frame));
			skb->protocol = htons(ETH_P_CAN);
			skb->ip_summed = CHECKSUM_UNNECESSARY;
			cf = (struct can_frame *)skb_put(skb, sizeof(struct can_frame));

			mcp2515_read_buffer(priv, cf, 1);

			stats->rx_packets++;
			stats->rx_bytes += cf->can_dlc;
			netif_receive_skb(skb);
		}

		if(flags&TX0IF) {
			dev_kfree_skb_any(priv->tx_skb);
			netif_wake_queue(dev);
			mcp2515_bit_modify(priv, CANINTF, 2, 0);
		}

		if(flags&ERRIF) {
			printk("ERRIF!! EFLG: 0x%x\n", mcp2515_read_reg(priv, EFLG));
			mcp2515_write_reg(priv, EFLG, 0);
			mcp2515_bit_modify(priv, CANINTF, 5, 0);
		}

		if(flags&MERRF) {
			printk("MRRIF!!\n");
			mcp2515_bit_modify(priv, CANINTF, 7, 0);
		}
	}
}

static void mcp2515_tx_handler(struct work_struct *work)
{
	struct mcp2515_priv *priv =	container_of(work, struct mcp2515_priv, tx_work);
	struct net_device_stats *stats = &priv->dev->stats;
	struct can_frame *cf = (struct can_frame *)priv->tx_skb->data;
	int buf;


	mcp2515_write_buffer(priv, cf, 0);
	/* Send message */
	mcp2515_exec(priv, MCP_RTS_TX(0));

	stats->tx_bytes += cf->can_dlc;
}


static netdev_tx_t mcp2515_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct mcp2515_priv *priv = netdev_priv(dev);

	netif_stop_queue(dev);

	/* save the tinetif_receive_skbmestamp */
	priv->dev->trans_start = jiffies;
	/* Remember the skb for deferred processing */
	priv->tx_skb = skb;

	queue_work(priv->work_queue, &priv->tx_work);

	return NETDEV_TX_OK;
}


static irqreturn_t mcp2515_irq(int irq, void *dev_id)
{
	struct mcp2515_priv *priv = dev_id;

	queue_work(priv->work_queue, &priv->irq_work);

	return IRQ_HANDLED;
}


static int mcp2515_open(struct net_device *dev)
{
	int err;

	printk("Opening mcp2515...");
	err = open_candev(dev);

	if(err) {
		printk("Could not open mcp2515\n");
		goto out;
	}

	mcp2515_chip_start(dev);
	netif_start_queue(dev);

	printk("Ok!\n");

	return 0;

out:
	return err;
}

static int mcp2515_close(struct net_device *dev)
{
	printk("Closing mcp2515...");

	mcp2515_chip_stop(dev);
	close_candev(dev);

	printk("Ok!\n");
	return 0;
}


static int mcp2515_set_mode(struct net_device *dev, enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		mcp2515_chip_start(dev);
		netif_wake_queue(dev);
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}


static const struct net_device_ops mcp2515_netdev_ops = {
	.ndo_open	= mcp2515_open,
	.ndo_stop	= mcp2515_close,
	.ndo_start_xmit	= mcp2515_start_xmit,
};


static int __devinit mcp2515_probe(struct spi_device *spi)
{
	struct net_device *dev = 0;
	struct mcp2515_priv *priv;
	struct mcp2515_spi_platform_data *pdata;
	int err;

	dev = alloc_candev(sizeof(struct mcp2515_priv));

	dev->type				= ARPHRD_CAN;
	dev->mtu				= sizeof(struct can_frame);
	dev->hard_header_len	= 0;
	dev->addr_len			= 0;
	dev->tx_queue_len		= 10;
	dev->flags				= IFF_NOARP;
	dev->netdev_ops			= &mcp2515_netdev_ops;
	dev->destructor			= free_netdev;

	priv = netdev_priv(dev);
	priv->spi_dev = spi;
	priv->dev = dev;

	priv->can.clock.freq = 2500000;
	priv->can.bittiming_const = &mcp2515_bittiming_const;
	priv->can.do_set_bittiming = mcp2515_set_bittiming;
	priv->can.do_set_mode = mcp2515_set_mode;
	priv->can.bittiming.bitrate = 125000;
	priv->can.bittiming.brp = 9;
	priv->can.bittiming.phase_seg1 = 5;
	priv->can.bittiming.phase_seg2 = 3;
	priv->can.bittiming.prop_seg = 1;
	priv->can.bittiming.sample_point = 1;
	priv->can.bittiming.sjw = 2;
	priv->can.bittiming.tq = 800;

	priv->work_queue = create_rt_workqueue("mcp2515");

	INIT_WORK(&priv->tx_work, mcp2515_tx_handler);
	INIT_WORK(&priv->irq_work, mcp2515_irq_handler);

	dev_set_drvdata(&spi->dev, priv);
	SET_NETDEV_DEV(dev, &spi->dev);

	if(register_candev(dev)) {
		printk("Could not register mcp2515\n");
		goto exit_free;
	}

	pdata = spi->dev.platform_data;
	priv->cs_gpio = pdata->cs_gpio;
	priv->reset_gpio = pdata->reset_gpio;

	gpio_request(priv->cs_gpio, "mcp2515_cs");
	gpio_direction_output(priv->cs_gpio, 0);
	gpio_set_value(priv->cs_gpio, 1);

	gpio_request(spi->irq, "mcp2515_irq");
	gpio_direction_input(spi->irq);
	set_irq_type(IRQ_GPIO(spi->irq), IRQ_TYPE_EDGE_FALLING);

	if(request_irq(IRQ_GPIO(spi->irq), mcp2515_irq, 0, "mcp2515", priv) < 0) {
		printk("Could not request IRQ %i\n", spi->irq);
		goto exit_free;
	}

	printk(KERN_INFO "%s: mcp2515 device registred\n", dev->name);

	return 0;

exit_free:
	free_netdev(dev);
	return err;
}

static int __devexit mcp2515_remove(struct spi_device *spi)
{
	struct mcp2515_priv *priv = dev_get_drvdata(&spi->dev);

	//unregister_netdev(priv->dev);
	free_irq(IRQ_GPIO(spi->irq), priv);
	//free_netdev(priv->dev);

	return 0;
}

static struct spi_driver mcp2515_driver = {
	.probe	= mcp2515_probe,
	.remove	= mcp2515_remove,
	.driver = {
		.name	= "mcp2515",
		.owner	= THIS_MODULE,
	},
};

static __init int mcp2515_init_module(void)
{
	return spi_register_driver(&mcp2515_driver);
}

static __exit void mcp2515_cleanup_module(void)
{
	spi_unregister_driver(&mcp2515_driver);
}

module_init(mcp2515_init_module);
module_exit(mcp2515_cleanup_module);

MODULE_ALIAS("spi:mcp2515");
