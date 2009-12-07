
#include <linux/module.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/can.h>
#include <linux/can/dev.h>
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
#define RX01E		1

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
#define MCP_RTS_TX0			0x81
#define MCP_RTS_TX1			0x82
#define MCP_RTS_TX2			0x84
#define MCP_RTS_ALL			0x87
#define MCP_READ_RX0		0x90
#define MCP_READ_RX1		0x94
#define MCP_READ_STATUS		0xA0
#define MCP_RX_STATUS		0xB0
#define MCP_RESET			0xC0


#define MCP2515_NAPI_WEIGHT 12


struct mcp2515_priv {
	struct can_priv		can;
	struct net_device	*dev;
	struct napi_struct	napi;
	struct spi_device *spi_dev;
	int tx_next;
	int rx_next;
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


static int mcp2515_exec(struct spi_device *spi, int command)
{
	unsigned char b = command;
	return spi_write(spi, &b, 1);
}

static int mcp2515_read(struct spi_device *spi, int reg)
{
	unsigned char b[2];
	b[0] = MCP_READ;
	b[1] = reg;
	spi_write(spi, b, 2);
	spi_read(spi, b, 1);

	return b[0];
}

static int mcp2515_write(struct spi_device *spi, int reg, int val)
{
	unsigned char b[3];
	b[0] = MCP_WRITE;
	b[1] = reg;
	b[2] = val;
	return spi_write(spi, b, 2);
}


static inline int mcp2515_get_next_tx_buffer(struct mcp2515_priv *priv)
{
	priv->tx_next = (priv->tx_next > 2) ? priv->tx_next : 0;
	return priv->tx_next;
}


static void mcp2515_chip_start(struct net_device *dev)
{
	struct mcp2515_priv *priv = netdev_priv(dev);

	printk("mcp2515 start\n");

	mcp2515_exec(priv->spi_dev, MCP_RESET);

	mcp2515_write(priv->spi_dev, RXMSIDH(0), 0x00);
	mcp2515_write(priv->spi_dev, RXMSIDL(0), 0x00);

	mcp2515_write(priv->spi_dev, RXMSIDH(1), 0x00);
	mcp2515_write(priv->spi_dev, RXMSIDL(1), 0x00);

	/* Setup interrupts */
	mcp2515_write(priv->spi_dev, CANINTE, (TX2IE | TX1IE | TX0IE | RX1IE | RX01E));
	mcp2515_write(priv->spi_dev, CANINTF, 0);

	/* Set normal mode */
	mcp2515_write(priv->spi_dev, CANCTRL, 0);

	/*
	mcp2515_write(priv->spi_dev, RXF0SIDH, 0x07); // TODO: Fix value
	mcp2515_write(priv->spi_dev, RXF0SIDL, 0x88); // TODO: Fix value
	mcp2515_write(priv->spi_dev, RXF1SIDH, 0x07); // TODO: Fix value
	mcp2515_write(priv->spi_dev, RXF1SIDL, 0x88); // TODO: Fix value
	mcp2515_write(priv->spi_dev, RXF2SIDL, 0x08);	// TODO: Fix value
	mcp2515_write(priv->spi_dev, RXF2EID8, 0x00);	// TODO: Fix value
	mcp2515_write(priv->spi_dev, RXF3SIDL, 0x00);	// TODO: Fix value
	mcp2515_write(priv->spi_dev, RXF3EID8, 0x00);	// TODO: Fix value
	mcp2515_write(priv->spi_dev, RXF4SIDL, 0x00); // TODO: Fix value
	mcp2515_write(priv->spi_dev, RXF4EID8, 0x00); // TODO: Fix value
	mcp2515_write(priv->spi_dev, RXF5SIDL, 0x00); // TODO: Fix value
	mcp2515_write(priv->spi_dev, RXF5EID8, 0x00); // TODO: Fix value
	*/
}

static void mcp2515_chip_stop(struct net_device *dev)
{
	//struct mcp2515_priv *priv = netdev_priv(dev);
	printk("mcp2515 stop\n");
}


static int mcp2515_set_bittiming(struct net_device *dev)
{
	const struct mcp2515_priv *priv = netdev_priv(dev);
	const struct can_bittiming *bt = &priv->can.bittiming;
	unsigned char reg_cnf1, reg_cnf2, reg_cnf3;

	printk("mcp2515 set bittiming\n");
	reg_cnf1 = (bt->sjw&0x3)<<6 | (bt->brp&0x3f);
	mcp2515_write(priv->spi_dev, CNF1, reg_cnf1);

	reg_cnf2 = (1<<7) | (1<<6) | (((bt->phase_seg1-1)&0x7)<<5) | (bt->prop_seg&0x7);
	mcp2515_write(priv->spi_dev, CNF2, reg_cnf2);

	reg_cnf3 = bt->phase_seg2&0x7;
	mcp2515_write(priv->spi_dev, CNF3, reg_cnf3);

	return 0;
}



static netdev_tx_t mcp2515_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct mcp2515_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf = (struct can_frame *)skb->data;
	int buf;
	int i;

	printk("mcp2515 start xmit\n");
	/* Get next buffer */
	buf = mcp2515_get_next_tx_buffer(priv);


	/* Check that buffer is free, should be! */
	if(mcp2515_read(priv->spi_dev, MCP_READ_STATUS)&(1<<(2*(buf+1)))) {
		dev_err(dev->dev.parent, "BUG! TX buffer full when queue awake!\n");
		return NETDEV_TX_BUSY;
	}

	/* Send identifier */
	mcp2515_write(priv->spi_dev, TXBSIDH(buf), cf->can_id>>3);
	mcp2515_write(priv->spi_dev, TXBSIDL(buf), cf->can_id&0x7<<5);

	/* send data to can controller */
	for(i=0; i<cf->can_dlc; i++) {
		mcp2515_write(priv->spi_dev, TXBDM(buf)+i, cf->data[i]);
	}

	/* Send message */
	mcp2515_write(priv->spi_dev, TXBCTRL(buf), (1<<3) | 2);

	priv->tx_next++;
	buf = mcp2515_get_next_tx_buffer(priv);

	if(mcp2515_read(priv->spi_dev, MCP_READ_STATUS)&(1<<(2*(buf+1))) || !priv->tx_next) {
		netif_stop_queue(dev);
	}

	stats->tx_bytes += cf->can_dlc;

	return NETDEV_TX_OK;
}


static irqreturn_t mcp2515_irq(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct mcp2515_priv *priv = netdev_priv(dev);
	irqreturn_t handled = IRQ_NONE;
	u8 flags;

	printk("mcp2515 irq\n");

	flags = mcp2515_read(priv->spi_dev, CANINTF);

	/* Schedule poll routine on reveive */
	if(flags&RX0IF || flags&RX1IF) {
		napi_schedule(&priv->napi);
		return IRQ_HANDLED;
	}

	if((flags&TX0IF) | (flags&TX1IF) | (flags&TX2IF)) {
		return IRQ_HANDLED;
	}

	return handled;
}


static int mcp2515_open(struct net_device *dev)
{
	int err;
	struct mcp2515_priv *priv = netdev_priv(dev);

	printk("mcp2515 start\n");
	err = open_candev(dev);

	if(err) {
		printk("Could not open mcp2515\n");
		goto out;
	}

	mcp2515_chip_start(dev);
	napi_enable(&priv->napi);
	netif_start_queue(dev);

	return 0;

out:
	return err;
}

static int mcp2515_close(struct net_device *dev)
{
	int err;
	struct mcp2515_priv *priv = netdev_priv(dev);
	mcp2515_chip_stop(dev);
	close_candev(dev);
	napi_disable(&priv->napi);

	return 0;
}

static int mcp2515_poll(struct napi_struct *napi, int quota)
{
	struct net_device *dev = napi->dev;
	struct mcp2515_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	int i, buf, status;

	printk("mcp2515 poll\n");

	skb = netdev_alloc_skb(dev, sizeof(struct can_frame));
	if (unlikely(!skb)) {
		stats->rx_dropped++;
		return 0;
	}

	skb->protocol = htons(ETH_P_CAN);
	skb->ip_summed = CHECKSUM_UNNECESSARY;

	cf = (struct can_frame *)skb_put(skb, sizeof(struct can_frame));

	status = mcp2515_read(priv->spi_dev, CANINTF);

	buf = (status&RX0IF) ? 0 : 1;

	/* read buffer from controller */
	cf->can_id = 	mcp2515_read(priv->spi_dev, RXBSIDH(buf))<<3 |
					mcp2515_read(priv->spi_dev, RXBSIDL(buf))>>5;

	cf->can_dlc = mcp2515_read(priv->spi_dev, RXBDLC(buf));

	for(i=0; i<cf->can_dlc; i++) {
		cf->data[i] = mcp2515_read(priv->spi_dev, RXBDM(buf));
	}

	netif_receive_skb(skb);

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;

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
	int err;

	dev = alloc_candev(sizeof(struct mcp2515_priv));

	dev->type				= ARPHRD_CAN;
	dev->mtu				= sizeof(struct can_frame);
	dev->hard_header_len	= 0;
	dev->addr_len			= 0;
	dev->tx_queue_len		= 0;
	dev->flags				= IFF_NOARP;
	dev->netdev_ops			= &mcp2515_netdev_ops;
	dev->destructor			= free_netdev;

	priv = netdev_priv(dev);
	priv->spi_dev = spi;
	priv->dev = dev;

	priv->can.clock.freq = 20000000;
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

	netif_napi_add(dev, &priv->napi, mcp2515_poll, MCP2515_NAPI_WEIGHT);
	dev_set_drvdata(&spi->dev, dev);
	SET_NETDEV_DEV(dev, &spi->dev);
	err = register_candev(dev);


	if(err) {
		printk("Could not register mcp2515\n");
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
