/*******************************************************************************

  EZchip Network Linux driver
  Copyright(c) 2012 EZchip Technologies.

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

*******************************************************************************/

#include <linux/platform_device.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include "nps_enet.h"
#include "nps_enet_irq.h"

#define  DRV_NAME      "eth"
#define  DRV_VERSION   "0.1"
#define  DRV_RELDATE   "11.Feb.2012"

MODULE_AUTHOR("Tal Zilcer <talz@ezchip.com>");
MODULE_DESCRIPTION("EZCHIP NPS-HE LAN driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

static char version[] =
	DRV_NAME "driver version " DRV_VERSION " (" DRV_RELDATE ")\n";

struct net_device *npsdev;
struct sockaddr mac_addr = { 0, {0x84, 0x66, 0x46, 0x88, 0x63, 0x33} };

struct nps_enet_info *eznet_global_p;

static int ez_net_set_mac_address(struct net_device *netdev, void *p)
{
	int res;
	struct ez_net_priv *net_priv = netdev_priv(netdev);
	struct sockaddr *x = (struct sockaddr *)p;
	char *uc_addr = (char *)x->sa_data;

	netif_info(net_priv, hw, netdev,
			"new MAC addr %02x:%02x:%02x:%02x:%02x:%02x\n",
			uc_addr[0]&0x0FF, uc_addr[1]&0x0FF, uc_addr[2]&0x0FF,
			uc_addr[3]&0x0FF, uc_addr[4]&0x0FF, uc_addr[5]&0x0FF);

	res = eth_mac_addr(netdev, p);
	if (!res)
		memcpy(net_priv->address, uc_addr, ETH_ALEN);

	return res;
}

static void ez_net_hw_enable_control(void)
{
	unsigned int ctrl;

	ctrl = __raw_readl(REG_GEMAC_TX_CTL);
	ctrl &= ~(TX_CTL_RESET);
	__raw_writel(ctrl, REG_GEMAC_TX_CTL);

	ctrl = __raw_readl(REG_GEMAC_RX_CTL);
	ctrl &= ~(RX_CTL_RESET);
	__raw_writel(ctrl, REG_GEMAC_RX_CTL);

	if (eznet_global_p->board_enable_ctrl)
		eznet_global_p->board_enable_ctrl();
}

static void ez_net_hw_disable_control(void)
{
	unsigned int ctrl;

	ctrl = __raw_readl(REG_GEMAC_TX_CTL);
	ctrl |= TX_CTL_RESET;
	__raw_writel(ctrl, REG_GEMAC_TX_CTL);

	ctrl = __raw_readl(REG_GEMAC_RX_CTL);
	ctrl |= RX_CTL_RESET;
	__raw_writel(ctrl, REG_GEMAC_RX_CTL);

	if (eznet_global_p->board_disable_ctrl)
		eznet_global_p->board_disable_ctrl();
}

static void ez_net_send_buf(struct net_device *netdev,
				struct sk_buff *skb, short length)
{

	int i, k, buf;
	unsigned int ctrl;
	unsigned int *src_data = (unsigned int *)virt_to_phys(skb->data);

	k = array_int_len(length);

	/* Check alignment of source */
	if (((int)src_data & 0x3) == 0) {
		for (i = 0; i < k; i++)
			__raw_writel(src_data[i], REG_GEMAC_TXBUF_DATA);
	} else { /* Not aligned */
		for (i = 0; i < k; i++) {
			/*
			 * Tx buffer is a FIFO
			 * so we have to write all 4 bytes at once
			 */
			memcpy(&buf, src_data+i, EZ_NET_REG_SIZE);
			__raw_writel(buf, REG_GEMAC_TXBUF_DATA);
		}
	}
	/* Write the length of the Frame */
	ctrl = __raw_readl(REG_GEMAC_TX_CTL);
	ctrl = (ctrl & ~(TX_CTL_LENGTH_MASK)) | length;

	/* Send Frame */
	ctrl |= TX_CTL_BUSY;
	__raw_writel(ctrl, REG_GEMAC_TX_CTL);

	return;
}

static int ez_net_open(struct net_device *netdev)
{
	struct ez_net_priv *net_priv = netdev_priv(netdev);

	if (net_priv->status != EZ_NET_DEV_OFF)
		return -EAGAIN;

	ez_net_hw_enable_control();

	net_priv->status = EZ_NET_DEV_ON;

	/* Driver can now transmit frames */
	netif_start_queue(netdev);

	return  eznet_global_p->board_irq_alloc(netdev);
}

static int ez_net_stop(struct net_device *netdev)
{
	struct ez_net_priv *net_priv = netdev_priv(netdev);

	netif_stop_queue(netdev);

	ez_net_hw_disable_control();

	if (net_priv->status != EZ_NET_DEV_OFF) {
		net_priv->status = EZ_NET_DEV_OFF;
		net_priv->down_event++;
	}
	return 0;
}

static netdev_tx_t ez_net_start_xmit(struct sk_buff *skb,
					struct net_device *netdev)
{
	struct ez_net_priv *net_priv = netdev_priv(netdev);
	short length = skb->len;

	if (skb->len < ETH_ZLEN) {
		if (unlikely(skb_padto(skb, ETH_ZLEN) != 0))
			return NETDEV_TX_OK;
		length = ETH_ZLEN;
	}

	net_priv->packet_len = length;
	ez_net_send_buf(netdev, skb, length);

	dev_kfree_skb(skb);

	/* This driver handles one frame at a time  */
	netif_stop_queue(netdev);

	/* Enable LAN TX end IRQ  */
	eznet_global_p->board_enable_tx_irq();

	return NETDEV_TX_OK;
}

static struct net_device_stats *ez_net_get_stats(struct net_device *netdev)
{
	return &netdev->stats;
}

static int ez_net_change_mtu(struct net_device *netdev, int new_mtu)
{
	struct ez_net_priv *net_priv = netdev_priv(netdev);

	netif_printk(net_priv, hw, KERN_INFO, netdev ,
		   "mtu change from %d to %d\n",
		   (int)netdev->mtu, new_mtu);

	if ((new_mtu < ETH_ZLEN) || (new_mtu > EZ_NET_ETH_PKT_SZ))
		return -EINVAL;

	netdev->mtu = new_mtu;

	return 0;
}

static void ez_net_set_rx_mode(struct net_device *netdev)
{
	struct ez_net_priv *net_priv = netdev_priv(netdev);

	if (netdev->flags & IFF_PROMISC)
		net_priv->rx_mode = EZ_NET_PROMISC_MODE;
	else
		net_priv->rx_mode = EZ_NET_BRODCAST_MODE;

	return;
}

static int ez_net_do_ioctl(struct net_device *netdev,
				 struct ifreq *ifr, int cmd)
{
	switch (cmd) {
	case SIOCSIFTXQLEN:
		if (ifr->ifr_qlen < 0)
			return -EINVAL;

		netdev->tx_queue_len = ifr->ifr_qlen;
		break;

	case SIOCGIFTXQLEN:
		ifr->ifr_qlen = netdev->tx_queue_len;
		break;

	case SIOCSIFFLAGS:      /* Set interface flags */
		return dev_change_flags(netdev, ifr->ifr_flags);

	case SIOCGIFFLAGS:      /* Get interface flags */
		ifr->ifr_flags = (short) dev_get_flags(netdev);
		break;

	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

static void ez_net_tx_timeout(struct net_device *netdev)
{
	struct ez_net_priv *net_priv = netdev_priv(netdev);

	netif_printk(net_priv, tx_err, KERN_DEBUG, netdev,
		     "transmission timed out\n");
}

static const struct net_device_ops nps_netdev_ops = {
	.ndo_open		= ez_net_open,
	.ndo_stop		= ez_net_stop,
	.ndo_start_xmit		= ez_net_start_xmit,
	.ndo_get_stats		= ez_net_get_stats,
	.ndo_set_mac_address	= ez_net_set_mac_address,
	.ndo_set_rx_mode	= ez_net_set_rx_mode,
	.ndo_change_mtu		= ez_net_change_mtu,
	.ndo_do_ioctl		= ez_net_do_ioctl,
	.ndo_tx_timeout		= ez_net_tx_timeout,

};

static int ez_net_probe(struct platform_device *pdev)
{
	struct net_device *netdev;
	struct ez_net_priv *net_priv;
	int err;

	if (!pdev) {
		pr_info("%s: ERROR - no platform device defined\n",
			__func__);
		return -EINVAL;
	}

	netdev = (struct net_device *)
			alloc_etherdev(sizeof(struct ez_net_priv));
	if (!netdev)
		return -ENOMEM;

	net_priv = netdev_priv(netdev);

	/* The EZ NET specific entries in the device structure. */
	netdev->netdev_ops     = &nps_netdev_ops;
	netdev->watchdog_timeo = (400*HZ/1000);
	netdev->ml_priv = net_priv;

	/* initialize driver private data structure. */
	net_priv->status = EZ_NET_DEV_OFF;
	net_priv->msg_enable = 1; /* enable messages for netif_printk */

	netif_info(net_priv, hw, netdev, "%s: port %s\n",
			 __func__, netdev->name);

	/* config GEMAC register */
	__raw_writel(GEMAC_CFG_INIT_VALUE, REG_GEMAC_MAC_CFG);
	__raw_writel(0, REG_GEMAC_RX_CTL);
	ez_net_hw_disable_control();

	/* set kernel MAC address to dev */
	ez_net_set_mac_address(netdev, &mac_addr);

	/* We dont support MULTICAST */
	netdev->flags &= ~IFF_MULTICAST;

	/* Register the driver. Should  be the last thing in probe */
	err = register_netdev(netdev);
	if (err != 0) {
		netif_err(net_priv, probe, netdev,
			"%s: Failed to register netdev for %s, err = 0x%08x\n",
			__func__, netdev->name, (int)err);
		err = -ENODEV;
		goto ez_net_probe_error;
	}
	return 0;

ez_net_probe_error:
	free_netdev(netdev);
	return err;
}

static int  ez_net_remove(struct platform_device *pdev)
{
	struct net_device *netdev;
	struct ez_net_priv *net_priv;

	if (!pdev) {
		pr_info("%s: Can not remove device\n", __func__);
		return -EINVAL;
	}
	netdev = platform_get_drvdata(pdev);

	unregister_netdev(netdev);

	net_priv = netdev_priv(netdev);

	ez_net_hw_disable_control();

	eznet_global_p->board_free_irq(netdev);

	free_netdev(netdev);

	return 0;
}

static struct platform_driver nps_lan_driver = {
	.probe = ez_net_probe,
	.remove = ez_net_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = DRV_NAME,
	},
};

static int __init nps_enet_init_global(void)
{
	int version = eznps_he_version;

	eznet_global_p = kmalloc(sizeof(*eznet_global_p), GFP_KERNEL);
	if (!eznet_global_p)
		return -ENOMEM;

	memset(eznet_global_p, 0, sizeof(*eznet_global_p));
	switch (version) {
	case 0:
		he0_board_global_init(eznet_global_p);
		break;
	case 1:
		he1_board_global_init(eznet_global_p);
		break;
	default:
		break;
	}

	return 0;
}

static int __init nps_enet_init_module(void)
{
	int rc;

	pr_info("%s", version);
	rc = nps_enet_init_global();
	if (rc) {
		pr_info("%s: Can not initialize device\n", __func__);
		return rc;
	}
	rc = platform_driver_register(&nps_lan_driver);
	if (rc) {
		pr_info("%s: Can not register device\n", __func__);
		return rc;
	}
	return 0;
}

static void __exit nps_enet_cleanup_module(void)
{
	platform_driver_unregister(&nps_lan_driver);
	kfree(eznet_global_p);
}

module_exit(nps_enet_cleanup_module);
module_init(nps_enet_init_module);
