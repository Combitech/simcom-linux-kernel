/*
 * nacelle_can.c
 *
 *  Created on: Jan 28, 2011
 *      Author: dakila
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/skbuff.h>
#include <linux/platform_device.h>
#include <linux/can/platform/nacelle.h>
#include <mach/ssp.h>
#include <mach/regs-ssp.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/sched.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <net/rtnetlink.h>


MODULE_DESCRIPTION("Special CAN driver for Nacelle");
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

#define BFPCTRL		(0x0c)
#define TXRTSCTRL	(0x0d)



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



struct nacelle_priv {
	struct can_priv		can;
	struct net_device	*dev;
	struct napi_struct	napi;
	struct ssp_dev 		spi_dev;
	struct platform_device *pdev;
	struct work_struct tx_work;
	struct work_struct irq_work;
	struct workqueue_struct *work_queue;
	struct sk_buff 		*tx_skb;
	int 				tx_next;
	int 				cs_gpio;
	int 				reset_gpio;
	int 				irq_gpio;
};


static struct can_bittiming_const nacelle_bittiming_const = {
	.name = "NACELLE_CAN",
	.tseg1_min	= 2,
	.tseg1_max	= 16,
	.tseg2_min	= 2,
	.tseg2_max	= 16,
	.sjw_max	= 4,
	.brp_min 	= 1,
	.brp_max	= 32,
	.brp_inc	= 1,
};


static int nacelle_read_byte(struct nacelle_priv *priv, unsigned char *b)
{
	int data;
	int ret;

	ssp_write_word(&priv->spi_dev, 0xff);
	ret = ssp_read_word(&priv->spi_dev, &data);
	if(ssp_flush(&priv->spi_dev) == -ETIMEDOUT) { printk("Timeout\n"); }
	*b = data;

	return ret;
}

static int nacelle_write_byte(struct nacelle_priv *priv, unsigned char b)
{
	int ret;
	ret = ssp_write_word(&priv->spi_dev, b);
	if(ssp_flush(&priv->spi_dev) == -ETIMEDOUT) { printk("Timeout\n"); }
	return ret;
}


static int nacelle_exec(struct nacelle_priv *priv, int command)
{
	int ret;
	gpio_set_value(priv->cs_gpio, 0);
	ret = ssp_write_word(&priv->spi_dev, command);
	if(ssp_flush(&priv->spi_dev) == -ETIMEDOUT) { printk("Timeout\n"); }
	gpio_set_value(priv->cs_gpio, 1);
	return ret;
}

static int nacelle_read_buffer(struct nacelle_priv *priv, struct can_frame *cf, int index)
{
	u8 tx[2];
	u8 rx[15];
	int i;

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
	nacelle_write_byte(priv, tx[0]);

	for(i=0; i<13; i++) {
		nacelle_read_byte(priv, &rx[i]);
	}

	//spi_write_then_read(priv->spi_dev, tx, 1, rx, 13);
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

static int nacelle_write_buffer(struct nacelle_priv *priv, struct can_frame *cf, int index)
{
	int i;
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
	tx[2] = (cf->can_id&0x07)<<5;
	tx[3] = 0;
	tx[4] = 0;
	tx[5] = cf->can_dlc;

	memcpy(&tx[6], cf->data, cf->can_dlc);

	gpio_set_value(priv->cs_gpio, 0);

	for(i=0; i<(6+cf->can_dlc); i++) {
		nacelle_write_byte(priv, tx[i]);
	}

	gpio_set_value(priv->cs_gpio, 1);

	return 0;
}


static int nacelle_read_reg(struct nacelle_priv *priv, int reg)
{
	u8 tx[2];
	u8 rx[1];
	tx[0] = MCP_READ;
	tx[1] = reg;
	gpio_set_value(priv->cs_gpio, 0);
	nacelle_write_byte(priv, tx[0]);
	nacelle_write_byte(priv, tx[1]);
	nacelle_read_byte(priv, &rx[0]);
	gpio_set_value(priv->cs_gpio, 1);
	return rx[0];
}

static int nacelle_bit_modify(struct nacelle_priv *priv, int reg, int bitno, int val)
{
	unsigned char b[4];
	int ret;
	b[0] = MCP_BITMOD;
	b[1] = reg;
	b[2] = (1<<bitno);
	b[3] = (val<<bitno);
	gpio_set_value(priv->cs_gpio, 0);
	nacelle_write_byte(priv, b[0]);
	nacelle_write_byte(priv, b[1]);
	nacelle_write_byte(priv, b[2]);
	nacelle_write_byte(priv, b[3]);
	gpio_set_value(priv->cs_gpio, 1);
	return ret;
}

static int nacelle_write_reg(struct nacelle_priv *priv, int reg, int val)
{
	unsigned char b[3];
	int ret;
	b[0] = MCP_WRITE;
	b[1] = reg;
	b[2] = val;
	gpio_set_value(priv->cs_gpio, 0);
	nacelle_write_byte(priv, b[0]);
	nacelle_write_byte(priv, b[1]);
	nacelle_write_byte(priv, b[2]);
	gpio_set_value(priv->cs_gpio, 1);
	return ret;
}


static inline int nacelle_get_next_tx_buffer(struct nacelle_priv *priv)
{
	priv->tx_next = (priv->tx_next < 3) ? priv->tx_next : 0;
	return priv->tx_next;
}

static int nacelle_set_bittiming(struct net_device *dev);
static void nacelle_chip_start(struct net_device *dev)
{
	struct nacelle_priv *priv = netdev_priv(dev);

	nacelle_exec(priv, MCP_RESET);
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout(HZ/10);

	nacelle_set_bittiming(dev);

	nacelle_write_reg(priv, RXMSIDH(0), 0x00);
	nacelle_write_reg(priv, RXMSIDL(0), 0x00);

	nacelle_write_reg(priv, RXMSIDH(1), 0x00);
	nacelle_write_reg(priv, RXMSIDL(1), 0x00);

	/* Set normal mode */
	nacelle_write_reg(priv, CANCTRL, 0x00);

	nacelle_write_reg(priv, RXBCTRL(0), 0x64);
	nacelle_write_reg(priv, RXBCTRL(1), 0x60);

	/* Setup interrupts */
	nacelle_write_reg(priv, CANINTE, (RX0IE | RX1IE | TX0IE));
	nacelle_write_reg(priv, CANINTF, 0);
	nacelle_write_reg(priv, EFLG, 0);
}

static void nacelle_chip_stop(struct net_device *dev)
{
	struct nacelle_priv *priv = netdev_priv(dev);
	printk("nacelle stop\n");
	nacelle_exec(priv, MCP_RESET);

	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(50);
}


static int nacelle_set_bittiming(struct net_device *dev)
{
	struct nacelle_priv *priv = netdev_priv(dev);
	struct can_bittiming *bt = &priv->can.bittiming;
	int reg_cnf1, reg_cnf2, reg_cnf3;

	printk("nacelle set bittiming\n");
	reg_cnf1 = (((bt->sjw - 1) & 0x3) << 6) | ((bt->brp - 1) & 0x3f );
	nacelle_write_reg(priv, CNF1, reg_cnf1);

	reg_cnf2 = (1<<7) | (1<<6) | (((bt->phase_seg1-1) & 0x7) << 3 ) | ((bt->prop_seg - 1) & 0x7 );
	nacelle_write_reg(priv, CNF2, reg_cnf2);

	reg_cnf3 = (bt->phase_seg2 - 1) & 0x7;
	nacelle_write_reg(priv, CNF3, reg_cnf3);


	return 0;
}



static void nacelle_tx_handler(struct work_struct *work)
{
	struct nacelle_priv *priv =	container_of(work, struct nacelle_priv, tx_work);
	struct net_device_stats *stats = &priv->dev->stats;
	struct can_frame *cf = (struct can_frame *)priv->tx_skb->data;
	int buf;
	unsigned long flags;

	local_irq_save(flags);

	nacelle_write_buffer(priv, cf, 0);
	/* Send message */
	nacelle_exec(priv, MCP_RTS_TX(0));

	local_irq_restore(flags);

	stats->tx_bytes += cf->can_dlc;
}



static netdev_tx_t nacelle_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct nacelle_priv *priv = netdev_priv(dev);
	struct can_frame *cf = (struct can_frame *)skb->data;

	netif_stop_queue(dev);

	/* save the tinetif_receive_skbmestamp */
	priv->dev->trans_start = jiffies;
	/* Remember the skb for deferred processing */
	priv->tx_skb = skb;

	nacelle_tx_handler(&priv->tx_work);

	return NETDEV_TX_OK;
}

static void nacelle_irq_handler(struct work_struct *work)
{
	struct nacelle_priv *priv =	container_of(work, struct nacelle_priv, irq_work);
	struct net_device *dev = priv->dev;
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	u8 flags;


	for(;;) {
		flags = nacelle_read_reg(priv, CANINTF);

		if(flags == 0) {
			break;
		}

		if(flags&RX0IF) {

			skb = netdev_alloc_skb(dev, sizeof(struct can_frame));
			skb->protocol = htons(ETH_P_CAN);
			skb->ip_summed = CHECKSUM_UNNECESSARY;
			cf = (struct can_frame *)skb_put(skb, sizeof(struct can_frame));

			nacelle_read_buffer(priv, cf, 0);

			stats->rx_packets++;
			stats->rx_bytes += cf->can_dlc;

			if(netif_rx(skb) != NET_RX_SUCCESS) { printk("error!!!"); }
		}

		if(flags&RX1IF) {
			skb = netdev_alloc_skb(dev, sizeof(struct can_frame));
			skb->protocol = htons(ETH_P_CAN);
			skb->ip_summed = CHECKSUM_UNNECESSARY;
			cf = (struct can_frame *)skb_put(skb, sizeof(struct can_frame));

			nacelle_read_buffer(priv, cf, 1);

			stats->rx_packets++;
			stats->rx_bytes += cf->can_dlc;

			if(netif_rx(skb) != NET_RX_SUCCESS) { printk("error!!!"); }
		}

		if(flags&TX0IF) {
			nacelle_bit_modify(priv, CANINTF, 2, 0);
			dev_kfree_skb_any(priv->tx_skb);
			netif_wake_queue(dev);
			stats->tx_packets++;

		}

		if(flags&TX1IF) {
			nacelle_bit_modify(priv, CANINTF, 3, 0);
			dev_kfree_skb_any(priv->tx_skb);
			netif_wake_queue(dev);

		}

		if(flags&TX2IF) {
			nacelle_bit_modify(priv, CANINTF, 4, 0);
			dev_kfree_skb_any(priv->tx_skb);
			netif_wake_queue(dev);

		}

		if(flags&ERRIF) {
			nacelle_bit_modify(priv, CANINTF, 5, 0);
			printk("ERRIF!! EFLG: 0x%x\n", nacelle_read_reg(priv, EFLG));
			nacelle_write_reg(priv, EFLG, 0);
			stats->tx_dropped++;

		}

		if(flags&MERRF) {
			nacelle_bit_modify(priv, CANINTF, 7, 0);
			//printk("MRRIF!!\n");
			stats->tx_dropped++;

		}
	}
}


static irqreturn_t nacelle_irq(int irq, void *dev_id)
{
	struct nacelle_priv *priv = dev_id;
	unsigned long flags;

	local_irq_save(flags);

	nacelle_irq_handler(&priv->irq_work);

	local_irq_restore(flags);
	//queue_work(priv->work_queue, &priv->irq_work);

	return IRQ_HANDLED;
}


static int nacelle_open(struct net_device *dev)
{
	int err;

	printk("Opening nacelle...");
	err = open_candev(dev);

	if(err) {
		printk("Could not open nacelle\n");
		goto out;
	}

	nacelle_chip_start(dev);
	netif_start_queue(dev);

	printk("Ok!\n");

	return 0;

out:
	return err;
}

static int nacelle_close(struct net_device *dev)
{
	printk("Closing nacelle...");

	nacelle_chip_stop(dev);
	close_candev(dev);

	printk("Ok!\n");
	return 0;
}


static int nacelle_set_mode(struct net_device *dev, enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		nacelle_chip_start(dev);
		netif_wake_queue(dev);
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}


static const struct net_device_ops nacelle_netdev_ops = {
	.ndo_open	= nacelle_open,
	.ndo_stop	= nacelle_close,
	.ndo_start_xmit	= nacelle_start_xmit,
};


static int __devinit nacelle_probe(struct platform_device *pdev)
{
	struct net_device *dev = 0;
	struct nacelle_priv *priv;
	int err;
	struct nacelle_can_platform_data *pdata;

	dev = alloc_candev(sizeof(struct nacelle_priv));

	dev->type				= ARPHRD_CAN;
	dev->mtu				= sizeof(struct can_frame);
	dev->hard_header_len	= 0;
	dev->addr_len			= 0;
	dev->tx_queue_len		= 1000;
	dev->flags				= IFF_NOARP;
	dev->netdev_ops			= &nacelle_netdev_ops;
	dev->destructor			= free_netdev;

	priv = netdev_priv(dev);

	if(ssp_init(&priv->spi_dev, 1, SSP_NO_IRQ) == -ENODEV) {
		printk("Could not allocate device\n");
	}

	ssp_disable(&priv->spi_dev);


	ssp_config(&priv->spi_dev, 	SSCR0_DataSize(8) | SSCR0_Motorola,
								SSCR1_TxTresh(1) | SSCR1_RxTresh(1),
								0,
								SSCR0_SCR & SSCR0_SerClkDiv(2));

	ssp_enable(&priv->spi_dev);
	priv->dev = dev;
	priv->can.clock.freq = 2000000;
	priv->can.bittiming_const = &nacelle_bittiming_const;
	priv->can.do_set_bittiming = nacelle_set_bittiming;
	priv->can.do_set_mode = nacelle_set_mode;
	priv->can.bittiming.bitrate = 500000;
	priv->can.bittiming.brp = 2;
	priv->can.bittiming.phase_seg1 = 5;
	priv->can.bittiming.phase_seg2 = 3;
	priv->can.bittiming.prop_seg = 1;
	priv->can.bittiming.sample_point = 1;
	priv->can.bittiming.sjw = 2;
	priv->can.bittiming.tq = 200;


	priv->work_queue = create_rt_workqueue("nacelle");

	INIT_WORK(&priv->tx_work, nacelle_tx_handler);
	INIT_WORK(&priv->irq_work, nacelle_irq_handler);

	//platform_set_drvdata(pdev, priv);
	dev_set_drvdata(&pdev->dev, priv);
	SET_NETDEV_DEV(dev, &pdev->dev);

	if(register_candev(dev)) {
		printk("Could not register nacelle\n");
		goto exit_free;
	}

	pdata = pdev->dev.platform_data;
	priv->cs_gpio = pdata->cs_gpio;
	priv->reset_gpio = pdata->reset_gpio;
	priv->irq_gpio = pdata->irq_gpio;

	if(gpio_request(priv->reset_gpio, "nacelle_reset") < 0) {
		printk("Could not request reset pin for nacelle\n");
		goto exit_free;
	}

	gpio_direction_output(priv->reset_gpio, 1);
	gpio_set_value(priv->reset_gpio, 1);


	if(gpio_request(priv->cs_gpio, "nacelle_cs") < 0) {
		printk("Could not request chip select pin for nacelle\n");
		goto exit_free;
	}

	gpio_direction_output(priv->cs_gpio, 0);
	gpio_set_value(priv->cs_gpio, 1);


	if(gpio_request(priv->irq_gpio, "nacelle_irq") < 0) {
		printk("Could not request irq pin for nacelle\n");
		goto exit_free;
	}

	gpio_direction_input(priv->irq_gpio);
	set_irq_type(IRQ_GPIO(priv->irq_gpio), IRQ_TYPE_EDGE_FALLING);

	if(request_irq(IRQ_GPIO(priv->irq_gpio), nacelle_irq, 0, "nacelle", priv) < 0) {
		printk("Could not request IRQ %i\n", priv->irq_gpio);
		goto exit_free;
	}

	printk(KERN_INFO "%s: nacelle device registred\n", dev->name);

	return 0;

exit_free:
	free_netdev(dev);
	return err;
}

static int __devexit nacelle_remove(struct platform_device *pdev)
{

	return 0;
}

static struct platform_driver nacelle_driver = {
	.probe	= nacelle_probe,
	.remove	= nacelle_remove,
	.driver = {
		.name	= "nacelle_can",
		.owner	= THIS_MODULE,
	},
};

static __init int nacelle_init_module(void)
{
	return platform_driver_register(&nacelle_driver);
}

static __exit void nacelle_cleanup_module(void)
{
	platform_driver_unregister(&nacelle_driver);
}

module_init(nacelle_init_module);
module_exit(nacelle_cleanup_module);

MODULE_ALIAS("spi:nacelle_can");
