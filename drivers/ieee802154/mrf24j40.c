/*
 * Sample driver for HardMAC IEEE 802.15.4 devices
 *
 * Copyright (C) 2009 Siemens AG
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Written by:
 * Dmitry Eremin-Solenikov <dmitry.baryshkov@siemens.com>
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/if_arp.h>

#include <net/af_ieee802154.h>
#include <net/ieee802154_netdev.h>
#include <net/ieee802154.h>
#include <net/nl802154.h>
#include <net/wpan-phy.h>

struct wpan_phy *net_to_phy(struct net_device *dev)
{
	return container_of(dev->dev.parent, struct wpan_phy, dev);
}

/**
 * fake_get_pan_id - Retrieve the PAN ID of the device.
 * @dev: The network device to retrieve the PAN of.
 *
 * Return the ID of the PAN from the PIB.
 */
static u16 mrf24j40_get_pan_id(struct net_device *dev)
{
	BUG_ON(dev->type != ARPHRD_IEEE802154);

	return 0xeba1;
}

/**
 * mrf24j40_get_short_addr - Retrieve the short address of the device.
 * @dev: The network device to retrieve the short address of.
 *
 * Returns the IEEE 802.15.4 short-form address cached for this
 * device. If the device has not yet had a short address assigned
 * then this should return 0xFFFF to indicate a lack of association.
 */
static u16 mrf24j40_get_short_addr(struct net_device *dev)
{
	BUG_ON(dev->type != ARPHRD_IEEE802154);

	return 0x1;
}

/**
 * mrf24j40_get_dsn - Retrieve the DSN of the device.
 * @dev: The network device to retrieve the DSN for.
 *
 * Returns the IEEE 802.15.4 DSN for the network device.
 * The DSN is the sequence number which will be added to each
 * packet or MAC command frame by the MAC during transmission.
 *
 * DSN means 'Data Sequence Number'.
 *
 * Note: This is in section 7.2.1.2 of the IEEE 802.15.4-2006
 *       document.
 */
static u8 mrf24j40_get_dsn(struct net_device *dev)
{
	BUG_ON(dev->type != ARPHRD_IEEE802154);

	return 0x00; /* DSN are implemented in HW, so return just 0 */
}

/**
 * mrf24j40_get_bsn - Retrieve the BSN of the device.
 * @dev: The network device to retrieve the BSN for.
 *
 * Returns the IEEE 802.15.4 BSN for the network device.
 * The BSN is the sequence number which will be added to each
 * beacon frame sent by the MAC.
 *
 * BSN means 'Beacon Sequence Number'.
 *
 * Note: This is in section 7.2.1.2 of the IEEE 802.15.4-2006
 *       document.
 */
static u8 mrf24j40_get_bsn(struct net_device *dev)
{
	BUG_ON(dev->type != ARPHRD_IEEE802154);

	return 0x00; /* BSN are implemented in HW, so return just 0 */
}

/**
 * mrf24j40_assoc_req - Make an association request to the HW.
 * @dev: The network device which we are associating to a network.
 * @addr: The coordinator with which we wish to associate.
 * @channel: The channel on which to associate.
 * @cap: The capability information field to use in the association.
 *
 * Start an association with a coordinator. The coordinator's address
 * and PAN ID can be found in @addr.
 *
 * Note: This is in section 7.3.1 and 7.5.3.1 of the IEEE
 *       802.15.4-2006 document.
 */
static int mrf24j40_assoc_req(struct net_device *dev,
		struct ieee802154_addr *addr, u8 channel, u8 page, u8 cap)
{
	struct wpan_phy *phy = net_to_phy(dev);

	mutex_lock(&phy->pib_lock);
	phy->current_channel = channel;
	phy->current_page = page;
	mutex_unlock(&phy->pib_lock);

	/* We simply emulate it here */
	return ieee802154_nl_assoc_confirm(dev, mrf24j40_get_short_addr(dev),
			IEEE802154_SUCCESS);
}

/**
 * mrf24j40_assoc_resp - Send an association response to a device.
 * @dev: The network device on which to send the response.
 * @addr: The address of the device to respond to.
 * @short_addr: The assigned short address for the device (if any).
 * @status: The result of the association request.
 *
 * Queue the association response of the coordinator to another
 * device's attempt to associate with the network which we
 * coordinate. This is then added to the indirect-send queue to be
 * transmitted to the end device when it polls for data.
 *
 * Note: This is in section 7.3.2 and 7.5.3.1 of the IEEE
 *       802.15.4-2006 document.
 */
static int mrf24j40_assoc_resp(struct net_device *dev,
		struct ieee802154_addr *addr, u16 short_addr, u8 status)
{
	return 0;
}

/**
 * mrf24j40_disassoc_req - Disassociate a device from a network.
 * @dev: The network device on which we're disassociating a device.
 * @addr: The device to disassociate from the network.
 * @reason: The reason to give to the device for being disassociated.
 *
 * This sends a disassociation notification to the device being
 * disassociated from the network.
 *
 * Note: This is in section 7.5.3.2 of the IEEE 802.15.4-2006
 *       document, with the reason described in 7.3.3.2.
 */
static int mrf24j40_disassoc_req(struct net_device *dev,
		struct ieee802154_addr *addr, u8 reason)
{
	return ieee802154_nl_disassoc_confirm(dev, IEEE802154_SUCCESS);
}

/**
 * mrf24j40_start_req - Start an IEEE 802.15.4 PAN.
 * @dev: The network device on which to start the PAN.
 * @addr: The coordinator address to use when starting the PAN.
 * @channel: The channel on which to start the PAN.
 * @bcn_ord: Beacon order.
 * @sf_ord: Superframe order.
 * @pan_coord: Whether or not we are the PAN coordinator or just
 *             requesting a realignment perhaps?
 * @blx: Battery Life Extension feature bitfield.
 * @coord_realign: Something to realign something else.
 *
 * If pan_coord is non-zero then this starts a network with the
 * provided parameters, otherwise it attempts a coordinator
 * realignment of the stated network instead.
 *
 * Note: This is in section 7.5.2.3 of the IEEE 802.15.4-2006
 * document, with 7.3.8 describing coordinator realignment.
 */
static int mrf24j40_start_req(struct net_device *dev, struct ieee802154_addr *addr,
				u8 channel, u8 page,
				u8 bcn_ord, u8 sf_ord, u8 pan_coord, u8 blx,
				u8 coord_realign)
{
	struct wpan_phy *phy = net_to_phy(dev);

	mutex_lock(&phy->pib_lock);
	phy->current_channel = channel;
	phy->current_page = page;
	mutex_unlock(&phy->pib_lock);

	/* We don't emulate beacons here at all, so START should fail */
	ieee802154_nl_start_confirm(dev, IEEE802154_INVALID_PARAMETER);
	return 0;
}

/**
 * mrf24j40_scan_req - Start a channel scan.
 * @dev: The network device on which to perform a channel scan.
 * @type: The type of scan to perform.
 * @channels: The channel bitmask to scan.
 * @duration: How long to spend on each channel.
 *
 * This starts either a passive (energy) scan or an active (PAN) scan
 * on the channels indicated in the @channels bitmask. The duration of
 * the scan is measured in terms of superframe duration. Specifically,
 * the scan will spend aBaseSuperFrameDuration * ((2^n) + 1) on each
 * channel.
 *
 * Note: This is in section 7.5.2.1 of the IEEE 802.15.4-2006 document.
 */
static int mrf24j40_scan_req(struct net_device *dev, u8 type, u32 channels,
		u8 page, u8 duration)
{
	u8 edl[27] = {};
	return ieee802154_nl_scan_confirm(dev, IEEE802154_SUCCESS, type,
			channels, page,
			type == IEEE802154_MAC_SCAN_ED ? edl : NULL);
}

static struct ieee802154_mlme_ops mrf24j40_mlme = {
	.assoc_req = mrf24j40_assoc_req,
	.assoc_resp = mrf24j40_assoc_resp,
	.disassoc_req = mrf24j40_disassoc_req,
	.start_req = mrf24j40_start_req,
	.scan_req = mrf24j40_scan_req,

	.get_pan_id = mrf24j40_get_pan_id,
	.get_short_addr = mrf24j40_get_short_addr,
	.get_dsn = mrf24j40_get_dsn,
	.get_bsn = mrf24j40_get_bsn,
};

static int mrf24j40_open(struct net_device *dev)
{
	netif_start_queue(dev);
	return 0;
}

static int mrf24j40_close(struct net_device *dev)
{
	netif_stop_queue(dev);
	return 0;
}

static netdev_tx_t mrf24j40_xmit(struct sk_buff *skb,
					      struct net_device *dev)
{
	skb->iif = dev->ifindex;
	skb->dev = dev;
	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb->len;

	dev->trans_start = jiffies;

	/* FIXME: do hardware work here ... */

	return NETDEV_TX_OK;
}


static int mrf24j40_ioctl(struct net_device *dev, struct ifreq *ifr,
		int cmd)
{
	struct sockaddr_ieee802154 *sa =
		(struct sockaddr_ieee802154 *)&ifr->ifr_addr;
	u16 pan_id, short_addr;

	switch (cmd) {
	case SIOCGIFADDR:
		/* FIXME: fixed here, get from device IRL */
		pan_id = mrf24j40_get_pan_id(dev);
		short_addr = mrf24j40_get_short_addr(dev);
		if (pan_id == IEEE802154_PANID_BROADCAST ||
		    short_addr == IEEE802154_ADDR_BROADCAST)
			return -EADDRNOTAVAIL;

		sa->family = AF_IEEE802154;
		sa->addr.addr_type = IEEE802154_ADDR_SHORT;
		sa->addr.pan_id = pan_id;
		sa->addr.short_addr = short_addr;
		return 0;
	}
	return -ENOIOCTLCMD;
}

static int mrf24j40_mac_addr(struct net_device *dev, void *p)
{
	return -EBUSY; /* HW address is built into the device */
}

static const struct net_device_ops mrf24j40_ops = {
	.ndo_open				= mrf24j40_open,
	.ndo_stop				= mrf24j40_close,
	.ndo_start_xmit			= mrf24j40_xmit,
	.ndo_do_ioctl			= mrf24j40_ioctl,
	.ndo_set_mac_address	= mrf24j40_mac_addr,
};

static void mrf24j40_destruct(struct net_device *dev)
{
	struct wpan_phy *phy = net_to_phy(dev);

	wpan_phy_unregister(phy);
	free_netdev(dev);
	wpan_phy_free(phy);
}

static void mrf24j40_setup(struct net_device *dev)
{
	dev->addr_len		= IEEE802154_ADDR_LEN;
	memset(dev->broadcast, 0xff, IEEE802154_ADDR_LEN);
	dev->features		= NETIF_F_NO_CSUM;
	dev->needed_tailroom	= 2; /* FCS */
	dev->mtu		= 127;
	dev->tx_queue_len	= 10;
	dev->type		= ARPHRD_IEEE802154;
	dev->flags		= IFF_NOARP | IFF_BROADCAST;
	dev->watchdog_timeo	= 0;
	dev->destructor		= mrf24j40_destruct;
}


static int __devinit mrf24j40_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	struct wpan_phy *phy = wpan_phy_alloc(0);
	int err;

	if (!phy)
		return -ENOMEM;

	dev = alloc_netdev(0, "wpan%d", mrf24j40_setup);
	if (!dev) {
		wpan_phy_free(phy);
		return -ENOMEM;
	}

	phy->dev.platform_data = dev;

	memcpy(dev->dev_addr, "\xba\xbe\xca\xfe\xde\xad\xbe\xef",
			dev->addr_len);
	memcpy(dev->perm_addr, dev->dev_addr, dev->addr_len);

	phy->channels_supported = (1 << 27) - 1;
	phy->transmit_power = 0xbf;

	dev->netdev_ops = &mrf24j40_ops;
	dev->ml_priv = &mrf24j40_mlme;

	/*
	 * If the name is a format string the caller wants us to do a
	 * name allocation.
	 */
	if (strchr(dev->name, '%')) {
		err = dev_alloc_name(dev, dev->name);
		if (err < 0)
			goto out;
	}

	SET_NETDEV_DEV(dev, &phy->dev);

	platform_set_drvdata(pdev, dev);

	err = wpan_phy_register(&pdev->dev, phy);
	if (err)
		goto out;

	err = register_netdev(dev);
	if (err < 0)
		goto out;

	dev_info(&pdev->dev, "Microhip mrf24j40 radio chip driver loaded\n");
	return 0;

out:
	unregister_netdev(dev);
	return err;
}

static int __devexit mrf24j40_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	unregister_netdev(dev);
	return 0;
}

static struct platform_device *mrf24j40_dev;

static struct platform_driver mrf24j40_driver = {
	.probe = mrf24j40_probe,
	.remove = __devexit_p(mrf24j40_remove),
	.driver = {
			.name = "mrf24j40",
			.owner = THIS_MODULE,
	},
};

static __init int mrf24j40_init(void)
{
	mrf24j40_dev = platform_device_register_simple("mrf24j40", -1, NULL, 0);
	return platform_driver_register(&mrf24j40_driver);
}

static __exit void mrf24j40_exit(void)
{
	platform_driver_unregister(&mrf24j40_driver);
	platform_device_unregister(mrf24j40_dev);
}

module_init(mrf24j40_init);
module_exit(mrf24j40_exit);
MODULE_LICENSE("GPL");

