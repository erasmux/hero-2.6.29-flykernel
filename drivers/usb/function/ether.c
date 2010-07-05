/* drivers/usb/function/ether.c
 *
 * RNDIS function device
 *
 * Copyright (C) 2008 htc, Inc.
 * Author: Arec Kao <arec_kao@htc.com>
 *
 * Based heavily on the function ethernet and gadget ethernet driver in
 * android/drivers/usb/function/ether.c
 * linux/drivers/usb/gadget/ether.c
 * 
 */ 

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/usb/ch9.h>
#include <linux/usb/cdc.h>

#include "usb_function.h"
#include "rndis.h"

/* Ethernet frame is 1514 + FCS, but round up to 512 * 3 so we
 * always queue a multiple of the USB max packet size (64 or 512)
 */
#define USB_MTU 1600//1536

#define MAX_TX 8
#define MAX_RX 8
#define MAX_INTR_TX 2
#define MAX_EP0_RX 2

#define DEBUG
#ifdef DEBUG
#define DBG(fmt,args...) \
	printk(KERN_DEBUG fmt, ## args)
#else
#define DBG(fmt,args...) \
	do { } while (0)
#endif /* DEBUG */

static char		manufacturer [10] = "HTC";

static int enabled = 0;

#define STATUS_INTERVAL_MSEC 9 /* 2^8*125us = 32ms */

#define	HS_BPS		(13 * 512 * 8 * 1000 * 8)

static struct usb_interface_descriptor
control_intf = {
	.bLength =              sizeof control_intf,
	.bDescriptorType =      USB_DT_INTERFACE,
	//.bInterfaceNumber =     0,
	.bAlternateSetting =	0,
	.bNumEndpoints =        1,
	.bInterfaceClass =      USB_CLASS_COMM,
	.bInterfaceSubClass =   USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol =   USB_CDC_ACM_PROTO_VENDOR,
	.iInterface =           STRING_ES,
};

static struct usb_cdc_header_desc header_desc = {
	.bLength =		sizeof header_desc,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_HEADER_TYPE,

	.bcdCDC =		0x0110,
};

static struct usb_cdc_call_mgmt_descriptor call_mgmt_desc = {
	.bLength =		sizeof call_mgmt_desc,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_CALL_MANAGEMENT_TYPE,

	.bmCapabilities =	0x00,
	//.bDataInterface =	0x01,
};

static struct usb_cdc_acm_descriptor acm_desc = {
	.bLength =		sizeof acm_desc,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_ACM_TYPE,

	.bmCapabilities =	0x00,
};

static struct usb_cdc_union_desc union_desc = {
	.bLength =		sizeof union_desc,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_UNION_TYPE,

	//.bMasterInterface0 =	0,	/* index of control interface */
	//.bSlaveInterface0 =	1,	/* index of DATA interface */
};

static struct usb_endpoint_descriptor
status_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	//.bEndpointAddress = 0x81,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize = 64,
	.bInterval = STATUS_INTERVAL_MSEC,
};

static struct usb_interface_descriptor
data_intf = {
	.bLength =		sizeof data_intf,
	.bDescriptorType =	USB_DT_INTERFACE,

	//.bInterfaceNumber =	1,
	.bAlternateSetting =	0,
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_CDC_DATA,
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	0,
	.iInterface =		STRING_ES,
};

//OUT
static struct usb_endpoint_descriptor
sink_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	//.bEndpointAddress = 0x01,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = 512,
};

//IN
static struct usb_endpoint_descriptor
source_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	//.bEndpointAddress = 0x82,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = 512,
};

static struct usb_descriptor_header *rndis_desc [] = {
	(struct usb_descriptor_header *) &control_intf,
	(struct usb_descriptor_header *) &header_desc,
	(struct usb_descriptor_header *) &call_mgmt_desc,
	(struct usb_descriptor_header *) &acm_desc,
	(struct usb_descriptor_header *) &union_desc,
	(struct usb_descriptor_header *) &status_desc,
	(struct usb_descriptor_header *) &data_intf,
	(struct usb_descriptor_header *) &sink_desc,
	(struct usb_descriptor_header *) &source_desc,
	NULL,
};

struct ether_context {
	spinlock_t lock;
	struct net_device *dev;
	
	struct usb_endpoint *intr_in;
	struct usb_endpoint *out;
	struct usb_endpoint *in;
	struct usb_endpoint *ep0in;
	struct usb_endpoint *ep0out;


	struct list_head rx_reqs;
	struct list_head tx_reqs;
	struct list_head tx_intr_reqs;
	struct list_head rx_cmd_reqs;
	
	struct net_device_stats stats;
	u8			host_mac [ETH_ALEN];
	int			rndis_config;
	u16			cdc_filter;
	int registered;
	int online;

	//unsigned char contrl_bInterfaceNumber;
};

static void receive_rndis_command (struct usb_endpoint *ep, struct usb_request *req);

static int ether_queue_out(struct ether_context *ctxt,
			   struct usb_request *req);
static void ether_in_complete(struct usb_endpoint *ept,
			      struct usb_request *req);
static void ether_out_complete(struct usb_endpoint *ept,
			       struct usb_request *req);
static void eth_bind(struct usb_endpoint **ept,
				void *_ctxt);
static void eth_unbind(void *_ctxt);

static void eth_configure(int configured, void *_ctxt);

static int eth_setup(struct usb_ctrlrequest *ctrl, void *buf,
			int len, void *_ctxt);

static void eth_start (struct ether_context *ctxt, gfp_t gfp_flags);

static void rx_fill(struct ether_context *ctxt);

static struct usb_function usb_func_ether = {
	.bind = eth_bind,
	.unbind = eth_unbind,
	.configure = eth_configure,
	.setup = eth_setup,

	.name = "ether",

	.ifc_class = 0x02,
	.ifc_subclass = 0x02,
	.ifc_protocol = 0xff,

	.ifc_name = "ether",
	.cdc_desc = rndis_desc,

	.ifc_num = 2,
	.ifc_ept_count = 3,
	.ifc_ept_type = { EPT_INT_IN, EPT_BULK_OUT, EPT_BULK_IN },
	.position_bit = USB_FUNCTION_INTERNET_SHARING_NUM,	
	.disabled = 1,
	.ifc_index = STRING_ES,
};

/* remove a request from the head of a list */

static struct usb_request *req_get(struct ether_context *ctxt, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&ctxt->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&ctxt->lock, flags);
	return req;
}

static void
rndis_control_ack_complete (struct usb_endpoint *ept, struct usb_request *req)
{
	struct ether_context *ctxt = req->context;
	unsigned long flags;

	//DBG("rndis_control_ack_complete\n");
	if (req->status || req->actual != req->length)
		printk(KERN_ERR	"rndis control ack complete --> %d, %d/%d\n",
			req->status, req->actual, req->length);
	req->context = NULL;
	spin_lock_irqsave(&ctxt->lock, flags);
	list_add_tail(&req->list, &ctxt->tx_intr_reqs);
	spin_unlock_irqrestore(&ctxt->lock, flags);

}

static int rndis_control_ack (struct net_device *net)
{
	struct ether_context *ctxt = netdev_priv(net);
	struct usb_request *resp = req_get(ctxt, &ctxt->tx_intr_reqs);
	if (!resp) {
		printk(KERN_ERR "rndis_control_ack: get request fial\n");
		return -1;
	}
	resp->length = 8;
	resp->context = ctxt;

	*((__le32 *) resp->buf) = __constant_cpu_to_le32 (1);
	*((__le32 *) resp->buf + 1) = __constant_cpu_to_le32 (0);
	
	if (usb_ept_queue_xfer(ctxt->intr_in, resp)) {
		resp->status = 0;
		rndis_control_ack_complete (ctxt->intr_in, resp);
	}

	return 0;
}

static ssize_t store_enable(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
        unsigned long ul;
        if (count > 2 || (buf[0] != '0' && buf[0] != '1')) {
                printk(KERN_ERR "Can't enable/disable ether %s\n", buf);
                return -EINVAL;
        }
        ul = simple_strtoul(buf, NULL, 10);
        enabled = usb_function_enable("ether", ul);

        return count;
}

static ssize_t show_enable(struct device *dev, struct device_attribute *attr,
                char *buf)
{
        if (enabled) {
                buf[0] = '1';
                buf[1] = '\n';
        }
        else {
                buf[0] = '0';
                buf[1] = '\n';
        }
        return 2;

}

static DEVICE_ATTR(enable, 0644, show_enable, store_enable);

static void eth_unbind(void *_ctxt)
{
	struct ether_context *ctxt = _ctxt;
	struct usb_request *req;
	
	while ((req = req_get(ctxt, &ctxt->tx_intr_reqs))) {
		usb_ept_free_req(ctxt->intr_in, req);
    }
	while ((req = req_get(ctxt, &ctxt->rx_cmd_reqs))) {
		usb_ept_free_req(ctxt->ep0out, req);
	}
	while ((req = req_get(ctxt, &ctxt->rx_reqs))) {
		usb_ept_free_req(ctxt->out, req);
    }
	while ((req = req_get(ctxt, &ctxt->tx_reqs))) {
		usb_ept_free_req(ctxt->in, req);
	}
	if (ctxt->registered == 1) {
		device_remove_file(&ctxt->dev->dev, &dev_attr_enable);
		ctxt->registered = 0;
	}
	rndis_deregister (ctxt->rndis_config);
	//rndis_exit ();
	//unregister_netdev (ctxt->dev);
	//free_netdev(ctxt->dev);
}

static void eth_bind(struct usb_endpoint **ept, void *_ctxt)
{
	struct ether_context *ctxt = _ctxt;
	struct usb_request *req;
	unsigned long flags;
	int n;
	int	status = -ENOMEM;
	struct usb_interface_descriptor *ifc_desc;
	struct usb_fi_ept *ept_info;
	
	u32	vendorID = 0x0bb4;

	ctxt->registered = 0;
	
	ctxt->intr_in = ept[0];
	ctxt->out = ept[1];
	ctxt->in = ept[2];
	ctxt->ep0in = ept[3];
	ctxt->ep0out = ept[4];

	//ctxt->contrl_bInterfaceNumber = ctxt->intr_in->owner->ifc.bInterfaceNumber;
	for (n = 0; n < MAX_INTR_TX; n++) {
		req = usb_ept_alloc_req(ctxt->intr_in, 8);
		if (!req)
			break;
		req->complete = rndis_control_ack_complete;
		spin_lock_irqsave(&ctxt->lock, flags);
		list_add_tail(&req->list, &ctxt->tx_intr_reqs);
		spin_unlock_irqrestore(&ctxt->lock, flags);
	}

	for (n = 0; n < MAX_EP0_RX; n++) {
		req = usb_ept_alloc_req(ctxt->ep0out, 4096);
		if (!req)
			break;
		req->complete = receive_rndis_command;
		spin_lock_irqsave(&ctxt->lock, flags);
		list_add_tail(&req->list, &ctxt->rx_cmd_reqs);
		spin_unlock_irqrestore(&ctxt->lock, flags);
	}

	for (n = 0; n < MAX_RX; n++) {
		req = usb_ept_alloc_req(ctxt->out, 0);
		if (!req)
			break;
		req->complete = ether_out_complete;
		spin_lock_irqsave(&ctxt->lock, flags);
		list_add_tail(&req->list, &ctxt->rx_reqs);
		spin_unlock_irqrestore(&ctxt->lock, flags);
	}
	for (n = 0; n < MAX_TX; n++) {
		req = usb_ept_alloc_req(ctxt->in, 0);
		if (!req)
			break;
		req->complete = ether_in_complete;
		spin_lock_irqsave(&ctxt->lock, flags);
		list_add_tail(&req->list, &ctxt->tx_reqs);
		spin_unlock_irqrestore(&ctxt->lock, flags);
	}

	ifc_desc= get_ifc_desc("ether");
	if (!ifc_desc)
		goto fail;
	control_intf.bInterfaceNumber = ifc_desc[0].bInterfaceNumber;
	data_intf.bInterfaceNumber = ifc_desc[1].bInterfaceNumber;
	
	ept_info = get_ept_info("ether");
	if (!ept_info)
		goto fail;
	
	status_desc.bEndpointAddress = ept_info[0].desc.bEndpointAddress;
	sink_desc.bEndpointAddress = ept_info[1].desc.bEndpointAddress;
	source_desc.bEndpointAddress = ept_info[2].desc.bEndpointAddress;

	call_mgmt_desc.bDataInterface = data_intf.bInterfaceNumber;
	union_desc.bMasterInterface0 = control_intf.bInterfaceNumber;
	union_desc.bSlaveInterface0 = data_intf.bInterfaceNumber;
	
	status = device_create_file(&ctxt->dev->dev, &dev_attr_enable);
	if (status != 0) {
		printk(KERN_ERR "ether device_create_file failed: %d\n", status);
        goto fail;
	}
	ctxt->registered = 1;
	ctxt->online = 0;
	status = rndis_init();
	if (status < 0) {
		printk(KERN_ERR "can't init RNDIS, %d\n", status);
		goto fail;
	}

	netif_stop_queue (ctxt->dev);
	netif_carrier_off (ctxt->dev);

	ctxt->rndis_config = rndis_register (rndis_control_ack);
	if (ctxt->rndis_config < 0) {
		printk(KERN_ERR "ether bind: rndis_register fial, %d\n", ctxt->rndis_config);
		//unregister_netdev (ctxt->dev);
		goto fail;
	}

	rndis_set_host_mac (ctxt->rndis_config, ctxt->host_mac);
	if (rndis_set_param_dev (ctxt->rndis_config, ctxt->dev,
					 &ctxt->stats, &ctxt->cdc_filter))
		goto fail;
	if (rndis_set_param_vendor(ctxt->rndis_config, vendorID,
					manufacturer))
		goto fail;
	if (rndis_set_param_medium(ctxt->rndis_config,
					NDIS_MEDIUM_802_3, 0))
		goto fail;
	
	return;

fail:
	eth_unbind (ctxt);
	return;
}

static void ether_in_complete(struct usb_endpoint *ept,
			      struct usb_request *req)
{
	unsigned long flags;
	struct sk_buff *skb = req->context;
	struct ether_context *ctxt = *((void **) skb->cb);

	if (req->status == 0) {
		ctxt->stats.tx_packets++;
		ctxt->stats.tx_bytes += req->actual;
	} else {
		ctxt->stats.tx_errors++;
	}

	dev_kfree_skb_any(skb);

	spin_lock_irqsave(&ctxt->lock, flags);
	if (list_empty(&ctxt->tx_reqs))
		netif_start_queue(ctxt->dev);
	list_add_tail(&req->list, &ctxt->tx_reqs);
	spin_unlock_irqrestore(&ctxt->lock, flags);
}

static void ether_out_complete(struct usb_endpoint *ept,
			       struct usb_request *req)
{
	struct sk_buff *skb = req->context;
	struct ether_context *ctxt = *((void **) skb->cb);
	int status = req->status;

	if (status == 0) {
		skb_put(skb, req->actual);
		status = rndis_rm_hdr (skb);
		if (status < 0
				|| ETH_HLEN > skb->len
				|| skb->len > ETH_FRAME_LEN) {
			ctxt->stats.rx_length_errors++;
			printk(KERN_ERR "rx length %d\n", skb->len);
			goto error;
		}
		skb->protocol = eth_type_trans(skb, ctxt->dev);
		ctxt->stats.rx_packets++;
		ctxt->stats.rx_bytes += req->actual;
		netif_rx(skb);
	} else {
error:
		dev_kfree_skb_any(skb);
		ctxt->stats.rx_errors++;
	}

	/* don't bother requeuing if we just went offline */
	if (req->status == -ENODEV) {
		unsigned long flags;
		spin_lock_irqsave(&ctxt->lock, flags);
		list_add_tail(&req->list, &ctxt->rx_reqs);
		spin_unlock_irqrestore(&ctxt->lock, flags);
	} else {
		if (ether_queue_out(ctxt, req))
			pr_err("ether_out: cannot requeue\n");
	}
}

static int ether_queue_out(struct ether_context *ctxt,
			   struct usb_request *req)
{
	unsigned long flags;
	struct sk_buff *skb;
	int ret;

	skb = alloc_skb(USB_MTU + NET_IP_ALIGN, GFP_ATOMIC);
	if (!skb) {
		pr_err("ether_queue_out: failed to alloc skb\n");
		ret = -ENOMEM;
		goto fail;
	}

	skb_reserve(skb, NET_IP_ALIGN);

	*((void **) skb->cb) = ctxt;
	req->buf = skb->data;
	req->length = USB_MTU;
	req->context = skb;

	ret = usb_ept_queue_xfer(ctxt->out, req);
	if (ret) {
fail:
		spin_lock_irqsave(&ctxt->lock, flags);
		list_add_tail(&req->list, &ctxt->rx_reqs);
		spin_unlock_irqrestore(&ctxt->lock, flags);
	}

	return ret;
}

static void rx_fill(struct ether_context *ctxt)
{
	struct usb_request *req;
	/* we're online -- get all rx requests queued */
	for (;;) {
		req = req_get(ctxt, &ctxt->rx_reqs);
		if (!req)
			break;
		if (ether_queue_out(ctxt, req))
			break;
	}
}

static void eth_configure(int configured, void *_ctxt)
{
	struct ether_context *ctxt = _ctxt;
	netif_stop_queue(ctxt->dev);
	netif_carrier_off(ctxt->dev);
	ctxt->online = 0;

	if (configured) {
		ctxt->online = 1;
		rx_fill(ctxt);
		netif_carrier_on(ctxt->dev);
		if (netif_running(ctxt->dev))
			eth_start(ctxt, GFP_KERNEL);
	} else {
		rndis_uninit(ctxt->rndis_config);
		/* all pending requests will be canceled */
	}
}

/*
static void queue_command_request(struct usb_endpoint *ep, struct usb_request *req)
{
	struct ether_context *ctxt = usb_func_ether.context;
	unsigned long flags;

	req->complete = 0;
	spin_lock_irqsave(&ctxt->lock, flags);
	list_add_tail(&req->list, &ctxt->rx_cmd_reqs);
	spin_unlock_irqrestore(&ctxt->lock, flags);
}
*/

static void rndis_command_complete(struct usb_endpoint *ep, struct usb_request *req)
{
	struct ether_context *ctxt = usb_func_ether.context;
	unsigned long flags;
	req->length = 0;
	//req->complete = queue_command_request;
	//usb_ept_queue_xfer(ctxt->ep0out, req);
	req->complete = 0;
	spin_lock_irqsave(&ctxt->lock, flags);
	list_add_tail(&req->list, &ctxt->rx_cmd_reqs);
	spin_unlock_irqrestore(&ctxt->lock, flags);	

}

static void receive_rndis_command (struct usb_endpoint *ep, struct usb_request *req)
{
	struct ether_context *ctxt = usb_func_ether.context;
	int			status;
/*
	DBG("receive_rndis_command: data %02x %02x %02x, actual length %d, req status %d\n", \
		*((char *)(req->buf)), \
		*((char *)(req->buf + 1)), \
		*((char *)(req->buf + 2)), \
		req->actual, \
		req->status);
*/	
	/* received RNDIS command from USB_CDC_SEND_ENCAPSULATED_COMMAND */
	spin_lock(&ctxt->lock);
	status = rndis_msg_parser (ctxt->rndis_config, (u8 *) req->buf);
	if (status < 0)
		printk(KERN_ERR "%s: rndis parse error %d\n", __FUNCTION__, status);
	spin_unlock(&ctxt->lock);
	req->length = 0;
	req->complete = rndis_command_complete;
	usb_ept_queue_xfer(ctxt->ep0in, req);
}


static int eth_setup(struct usb_ctrlrequest *ctrl, void *buf,
			int len, void *_ctxt)
{
	struct ether_context *ctxt = _ctxt;
	int value = -EOPNOTSUPP;
	u16			w_index = le16_to_cpu(ctrl->wIndex);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u16			w_length = le16_to_cpu(ctrl->wLength);

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_CLASS) {
		switch (ctrl->bRequest) {
		case USB_CDC_SEND_ENCAPSULATED_COMMAND:
		{
			struct usb_request *req;
			if ( w_value || control_intf.bInterfaceNumber != w_index)
				break;
			req = req_get(ctxt, &ctxt->rx_cmd_reqs);
			if (!req) {
				printk(KERN_ERR "%s: could not obtain tx request\n", __FUNCTION__);
				value = -ENOMEM;
				break;
			}
			value = w_length;
			req->length = w_length;
			req->complete = receive_rndis_command;
			usb_ept_queue_xfer(ctxt->ep0out, req);
			break;
		}
		case USB_CDC_GET_ENCAPSULATED_RESPONSE:
			if ((USB_DIR_IN|USB_TYPE_CLASS|USB_RECIP_INTERFACE)
						== ctrl->bRequestType
					&& !w_value
					&& control_intf.bInterfaceNumber == w_index) {
				u8 *tmp;
				u32 n;

				/* return the result */
				tmp = rndis_get_next_response(ctxt->rndis_config, &n);
				if (tmp) {
					memcpy(buf, tmp, n);
					//req->complete = rndis_response_complete;
					rndis_free_response(ctxt->rndis_config, tmp);
					value = n;
				}
				/* else stalls ... spec says to avoid that */
			}
			break;
		}
	}

	if (value == -EOPNOTSUPP)
		printk(KERN_ERR
			"unknown class-specific control req "
			"%02x.%02x v%04x i%04x l%u\n",
			ctrl->bRequestType, ctrl->bRequest,
			le16_to_cpu(ctrl->wValue), w_index, w_length);

	return value;
}

static int usb_ether_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct ether_context *ctxt = netdev_priv(dev);
	struct usb_request *req;
	unsigned long flags;
	unsigned len;
	struct sk_buff	*skb_rndis;

	req = req_get(ctxt, &ctxt->tx_reqs);
	if (!req) {
		pr_err("usb_ether_xmit: could not obtain tx request\n");
		return 1;
	}
	spin_lock_irqsave(&ctxt->lock, flags);
	if (list_empty(&ctxt->tx_reqs))
		netif_stop_queue(dev);
	spin_unlock_irqrestore(&ctxt->lock, flags);
	
	skb_rndis = skb_realloc_headroom (skb,
				sizeof (struct rndis_packet_msg_type));
	if (!skb_rndis)
		goto drop;

	dev_kfree_skb_any (skb);
	skb = skb_rndis;
	rndis_add_hdr (skb);
	
	/* ensure that we end with a short packet */
	len = skb->len;
	if (!(len & 63) || !(len & 511))
		len++;

	*((void **) skb->cb) = ctxt;
	req->context = skb;
	req->buf = skb->data;
	req->length = len;

	if (usb_ept_queue_xfer(ctxt->in, req)) {
drop:
		spin_lock_irqsave(&ctxt->lock, flags);
		list_add_tail(&req->list, &ctxt->tx_reqs);
		if (list_empty(&ctxt->tx_reqs))
			netif_start_queue(dev);
		spin_unlock_irqrestore(&ctxt->lock, flags);
		dev_kfree_skb_any(skb);
		ctxt->stats.tx_dropped++;

		pr_err("usb_ether_xmit: could not queue tx request\n");
	}

	return 0;
}

static void eth_start (struct ether_context *ctxt, gfp_t gfp_flags)
{

	/* fill the rx queue */
	rx_fill (ctxt);

	/* and open the tx floodgates */
	netif_wake_queue (ctxt->dev);
	if (ctxt->online) {
	rndis_set_param_medium (ctxt->rndis_config, NDIS_MEDIUM_802_3,
					HS_BPS/100);
	(void) rndis_signal_connect (ctxt->rndis_config);
	}
}

static int usb_ether_open(struct net_device *dev)
{
	struct ether_context *ctxt = netdev_priv(dev);
	DBG("usb_ether_open\n");
	if (netif_carrier_ok (ctxt->dev))
		eth_start (ctxt, GFP_KERNEL);
	return 0;
}

static int usb_ether_stop(struct net_device *dev)
{
	struct ether_context *ctxt = netdev_priv(dev);
	DBG("usb_ether_stop\n");
	netif_stop_queue (dev);
	if (ctxt->online) {
	rndis_set_param_medium(ctxt->rndis_config, NDIS_MEDIUM_802_3, 0);
	(void) rndis_signal_disconnect (ctxt->rndis_config);
	}
	return 0;
}

static struct net_device_stats *usb_ether_get_stats(struct net_device *dev)
{
	struct ether_context *ctxt = netdev_priv(dev);
	return &ctxt->stats;
}

static void __init usb_ether_setup(struct net_device *dev)
{
	struct ether_context *ctxt = netdev_priv(dev);

	INIT_LIST_HEAD(&ctxt->rx_reqs);
	INIT_LIST_HEAD(&ctxt->tx_reqs);
	INIT_LIST_HEAD(&ctxt->tx_intr_reqs);
	INIT_LIST_HEAD(&ctxt->rx_cmd_reqs);
	spin_lock_init(&ctxt->lock);
	ctxt->dev = dev;

	dev->open = usb_ether_open;
	dev->stop = usb_ether_stop;
	dev->hard_start_xmit = usb_ether_xmit;
	dev->get_stats = usb_ether_get_stats;
	dev->watchdog_timeo = 20;

	ether_setup(dev);

	random_ether_addr(dev->dev_addr);
	random_ether_addr(ctxt->host_mac);
}

static int __init ether_init(void)
{
	struct net_device *dev;
	struct ether_context *ctxt;
	int ret;

	dev = alloc_netdev(sizeof(struct ether_context),
			   "usb%d", usb_ether_setup);
	if (!dev)
		return -ENOMEM;

	ret = register_netdev(dev);
	if (ret)
		goto err_register_netdev;
	
	ctxt = netdev_priv(dev);
	usb_func_ether.context = ctxt;
	ret = usb_function_register(&usb_func_ether);
	if (ret < 0)
		goto err_register_function;
	return ret;
	
err_register_function:
	unregister_netdev(dev);
err_register_netdev:
	free_netdev(dev);

	return ret;

}

module_init(ether_init);
