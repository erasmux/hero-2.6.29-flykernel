/* drivers/usb/function/msm_hsusb.c
 *
 * Driver for HighSpeed USB Client Controller in MSM7K
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include <linux/clk.h>

#include <linux/usb/ch9.h>
#include <linux/usb/cdc.h>
#include <linux/io.h>

#include <asm/mach-types.h>

#include <mach/board.h>
#include <mach/msm_hsusb.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/switch.h>
#define MSM_USB_BASE ((unsigned) ui->addr)

#include <mach/msm_hsusb_hw.h>

#include "usb_function.h"

#define EPT_FLAG_IN        0x0001
#define EPT_FLAG_HALT      0x0002

#define SETUP_BUF_SIZE      4096

#define USB_TEST_MODE
#define USB_DIR_MASK	USB_DIR_IN

/* IDs for string descriptors */
#define STRING_LANGUAGE_ID      0
#define STRING_SERIAL           1
#define STRING_PRODUCT          2
#define STRING_MANUFACTURER     3

#if defined(CONFIG_USB_FUNCTION_CAT_KIT)
static bool car_kit_detect_start = 0;
static bool car_kit_unpluged_start = 0;
static struct switch_dev dock_switch = {
	.name = "dock",
};

#define DOCK_STATE_UNDOCKED     0
#define DOCK_STATE_DESK         (1 << 0)
#define DOCK_STATE_CAR          (1 << 1)
#endif

static char *strings [] = {
	"Mass Storage",
	"ADB",
	"HTC Ethernet Sharing",
	"HTC DIAG",
	"HTC USB Serial Function",
	"HTC PROJECTOR",
	"HTC FSYNC",
	"HTC MTP",
	"HTC Modem",
	"HTC NMEA",
	"HTC SERIAL",
};

#ifdef CONFIG_MACH_BRAVO
static struct wake_lock usb_inet_idle_lock;
#endif
#define LANGUAGE_ID             0x0409 /* en-US */

/* MS OS Descriptor */
#define STRING_MS_OS_DESC		0xee
#define MS_VENDOR_CODE	0xb4

struct ms_comp_feature_descriptor {
	__le32 dwLength;
	__le16 bcdVersion;
	__le16 wIndex;
	__u8 bCount;
	__u8 resv1[7];
	__u8 bFirstInterfaceNumber;
	__u8 resv2;
	__u8 compatibleID[8];
	__u8 subCompatibleID[8];
	__u8 resv3[6];
} __attribute__ ((packed));


static const struct ms_comp_feature_descriptor ms_comp_desc = {
	.dwLength = sizeof(ms_comp_desc),
	.bcdVersion = __constant_cpu_to_le16(0x0100),
	.wIndex = 0x0004,
	.bCount = 0x01,
	.bFirstInterfaceNumber = 0,
	.resv2 = 1,
};

/* current state of VBUS */
static int vbus;

struct usb_function_info {
	struct list_head list;
	unsigned endpoints; /* nonzero if bound */
	unsigned enabled;

	struct usb_function *func;
	struct usb_interface_descriptor ifc[2];
	struct usb_fi_ept ept[0];
};

struct msm_request {
	struct usb_request req;

	struct usb_info *ui;
	struct msm_request *next;

	unsigned busy:1;
	unsigned live:1;
	unsigned alloced:1;
	unsigned dead:1;

	dma_addr_t dma;

	struct ept_queue_item *item;
	dma_addr_t item_dma;
};

#define to_msm_request(r) container_of(r, struct msm_request, req)

struct usb_endpoint {
	struct usb_info *ui;
	struct msm_request *req; /* head of pending requests */
	struct msm_request *last;
	unsigned flags;

	/* bit number (0-31) in various status registers
	** as well as the index into the usb_info's array
	** of all endpoints
	*/
	unsigned char bit;
	unsigned char num;

	unsigned char type;

	unsigned short max_pkt;

	/* pointers to DMA transfer list area */
	/* these are allocated from the usb_info dma space */
	struct ept_queue_head *head;

	struct usb_function_info *owner;
};

static void usb_do_work(struct work_struct *w);


#define USB_STATE_IDLE    0
#define USB_STATE_ONLINE  1
#define USB_STATE_OFFLINE 2

#define USB_FLAG_START          0x0001
#define USB_FLAG_VBUS_ONLINE    0x0002
#define USB_FLAG_VBUS_OFFLINE   0x0004
#define USB_FLAG_RESET          0x0008

struct usb_info {
	/* lock for register/queue/device state changes */
	spinlock_t lock;

	/* single request used for handling setup transactions */
	struct usb_request *setup_req;

	struct platform_device *pdev;
	int irq;
	void *addr;

	unsigned state;
	unsigned flags;

	unsigned online;
	unsigned running;
	unsigned bound;

	enum usb_device_speed speed;

	struct dma_pool *pool;

	/* dma page to back the queue heads and items */
	unsigned char *buf;
	dma_addr_t dma;

	struct ept_queue_head *head;

	/* used for allocation */
	unsigned next_item;
	unsigned next_ifc_num;

	/* endpoints are ordered based on their status bits,
	** so they are OUT0, OUT1, ... OUT15, IN0, IN1, ... IN15
	*/
	struct usb_endpoint ept[32];

	int *phy_init_seq;
	void (*phy_reset)(void);
	void (*usb_uart_switch)(int);

	struct work_struct work;
	unsigned phy_status;
	unsigned phy_fail_count;

	struct usb_function_info **func;
	unsigned num_funcs;
	unsigned in_lpm;
#define ep0out ept[0]
#define ep0in  ept[16]

#ifdef USB_TEST_MODE
	u16 test_mode;
#endif
	struct clk *clk;
	struct clk *pclk;
	struct clk *ebi1clk;
#if defined(CONFIG_USB_FUNCTION_CAT_KIT)
	int usb_id_pin_gpio;
	unsigned int irq_usb_id_btn;
	void (*config_usb_id_gpios)(bool output_enable);
	bool enable_car_kit_detect;
#endif
};

struct usb_device_descriptor desc_device = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,

	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	/* the following fields are filled in by usb_probe */
	.idVendor = 0,
	.idProduct = 0,
	.bcdDevice = 0,
	.iManufacturer = 0,
	.iProduct = 0,
	.iSerialNumber = 0,
	.bNumConfigurations = 1,
};

static struct usb_qualifier_descriptor dev_qualifier = {
	.bLength =		sizeof dev_qualifier,
	.bDescriptorType =	USB_DT_DEVICE_QUALIFIER,
	.bcdUSB =		__constant_cpu_to_le16(0x0200),
	.bDeviceClass =		USB_CLASS_PER_INTERFACE,
	.bMaxPacketSize0 = 64,
	.bNumConfigurations =	1
};
uint32_t enabled_functions;

static struct usb_info *the_usb_info;

static void flush_endpoint(struct usb_endpoint *ept);
static struct usb_function_info *usb_find_function(const char *name);


static unsigned ulpi_read(struct usb_info *ui, unsigned reg)
{
	unsigned timeout = 100000;

	/* initiate read operation */
	writel(ULPI_RUN | ULPI_READ | ULPI_ADDR(reg),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout)) ;

	if (timeout == 0) {
		printk(KERN_ERR "ulpi_read: timeout %08x\n", readl(USB_ULPI_VIEWPORT));
		return 0xffffffff;
	}
	return ULPI_DATA_READ(readl(USB_ULPI_VIEWPORT));
}

static void ulpi_write(struct usb_info *ui, unsigned val, unsigned reg)
{
	unsigned timeout = 10000;

	/* initiate write operation */
	writel(ULPI_RUN | ULPI_WRITE |
	       ULPI_ADDR(reg) | ULPI_DATA(val),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout)) ;

	if (timeout == 0)
		printk(KERN_WARNING "%s: timeout: reg: 0x%X, var: 0x%X\n",
		__func__, reg, val);
}

static void ulpi_init(struct usb_info *ui)
{
	int *seq = ui->phy_init_seq;

	if (!seq)
		return;

	while (seq[0] >= 0) {
		printk("ulpi: write 0x%02x to 0x%02x\n", seq[0], seq[1]);
		ulpi_write(ui, seq[0], seq[1]);
		seq += 2;
	}
}

static void init_endpoints(struct usb_info *ui)
{
	unsigned n;
	for (n = 0; n < 32; n++) {
		struct usb_endpoint *ept = ui->ept + n;

		ept->ui = ui;
		ept->bit = n;
		ept->num = n & 15;

		if (ept->bit > 15) {
			/* IN endpoint */
			ept->head = ui->head + (ept->num << 1) + 1;
			ept->flags = EPT_FLAG_IN;
		} else {
			/* OUT endpoint */
			ept->head = ui->head + (ept->num << 1);
			ept->flags = 0;
		}

	}
}

static void configure_endpoints(struct usb_info *ui)
{
	unsigned n;
	unsigned cfg;

	for (n = 0; n < 32; n++) {
		struct usb_endpoint *ept = ui->ept + n;

		cfg = CONFIG_MAX_PKT(ept->max_pkt) | CONFIG_ZLT;

		if (ept->bit == 0)
			/* ep0 out needs interrupt-on-setup */
			cfg |= CONFIG_IOS;

		ept->head->config = cfg;
		ept->head->next = TERMINATE;
		/*
		if (ept->max_pkt)
			printk(KERN_DEBUG "ept #%d %s max:%d head:%p bit:%d\n",
			       ept->num,
			       (ept->flags & EPT_FLAG_IN) ? "in" : "out",
			       ept->max_pkt, ept->head, ept->bit);
		*/
	}
}

static struct usb_endpoint *alloc_endpoint(struct usb_info *ui,
					   struct usb_function_info *owner,
					   unsigned kind)
{
	unsigned n;
	for (n = 0; n < 32; n++) {
		struct usb_endpoint *ept = ui->ept + n;
		if (ept->num == 0)
			continue;
		if (ept->owner)
			continue;

		if (ept->flags & EPT_FLAG_IN) {
			if (kind == EPT_BULK_IN) {
				ept->max_pkt = 512;
				ept->owner = owner;
				return ept;
			} else if (kind == EPT_INT_IN) {
				ept->max_pkt = 64;
				ept->owner = owner;
				return ept;
			}
		} else {
			if (kind == EPT_BULK_OUT) {
				ept->max_pkt = 512;
				ept->owner = owner;
				return ept;
			} else if (kind == EPT_INT_OUT) {
				ept->max_pkt = 64;
				ept->owner = owner;
				return ept;
			}
		}
	}

	return 0;
}

static void free_endpoints(struct usb_info *ui, struct usb_function_info *owner)
{
	unsigned n;
	for (n = 0; n < 32; n++) {
		struct usb_endpoint *ept = ui->ept + n;
		if (ept->owner == owner) {
			ept->owner = 0;
			ept->max_pkt = 0;
		}
	}
}

struct usb_request *usb_ept_alloc_req(struct usb_endpoint *ept, unsigned bufsize)
{
	struct usb_info *ui = ept->ui;
	struct msm_request *req;

	req = kzalloc(sizeof(*req), GFP_ATOMIC);
	if (!req)
		goto fail1;

	req->item = dma_pool_alloc(ui->pool, GFP_ATOMIC, &req->item_dma);
	if (!req->item)
		goto fail2;

	if (bufsize) {
		req->req.buf = kmalloc(bufsize, GFP_ATOMIC);
		if (!req->req.buf)
			goto fail3;
		req->alloced = 1;
	}

	return &req->req;

fail3:
	dma_pool_free(ui->pool, req->item, req->item_dma);
fail2:
	kfree(req);
fail1:
	return 0;
}

static void do_free_req(struct usb_info *ui, struct msm_request *req)
{
	if (req->alloced)
		kfree(req->req.buf);

	dma_pool_free(ui->pool, req->item, req->item_dma);
	kfree(req);
}

void usb_ept_free_req(struct usb_endpoint *ept, struct usb_request *_req)
{
	struct msm_request *req = to_msm_request(_req);
	struct usb_info *ui = ept->ui;
	unsigned long flags;
	int dead = 0;

	spin_lock_irqsave(&ui->lock, flags);
	/* defer freeing resources if request is still busy */
	if (req->busy)
		dead = req->dead = 1;
	spin_unlock_irqrestore(&ui->lock, flags);

	/* if req->dead, then we will clean up when the request finishes */
	if (!dead)
		do_free_req(ui, req);
}

static void usb_ept_enable(struct usb_endpoint *ept, int yes)
{
	struct usb_info *ui = ept->ui;
	int in = ept->flags & EPT_FLAG_IN;
	unsigned char type = ept->type;
	unsigned n;

	if (yes) {
		if (ui->speed == USB_SPEED_HIGH)
			ept->max_pkt = 512;
		else
			ept->max_pkt = 64;
	}
	ept->flags &= ~EPT_FLAG_HALT;

	n = readl(USB_ENDPTCTRL(ept->num));

	if (in) {
		if (type == EPT_BULK_IN)
			n = (n & (~CTRL_TXT_MASK)) | CTRL_TXT_BULK;
		else if (type == EPT_INT_IN)
			n = (n & (~CTRL_TXT_MASK)) | CTRL_TXT_INT;
		if (yes)
			n |= CTRL_TXE | CTRL_TXR;
		else
			n &= (~CTRL_TXE);
	} else {
		if (type == EPT_BULK_OUT)
			n = (n & (~CTRL_RXT_MASK)) | CTRL_RXT_BULK;
		else if (type == EPT_INT_OUT)
			n = (n & (~CTRL_RXT_MASK)) | CTRL_RXT_INT;
		if (yes)
			n |= CTRL_RXE | CTRL_RXR;
		else
			n &= ~(CTRL_RXE);
	}
	writel(n, USB_ENDPTCTRL(ept->num));

#if 0
	printk(KERN_DEBUG "ept %d %s %s\n",
	       ept->num, in ? "in" : "out", yes ? "enabled" : "disabled");
#endif
}

static void usb_ept_start(struct usb_endpoint *ept)
{
	struct usb_info *ui = ept->ui;
	struct msm_request *req = ept->req;

	BUG_ON(req->live);

	/* link the hw queue head to the request's transaction item */
	ept->head->next = req->item_dma;
	ept->head->info = 0;

	/* start the endpoint */
	writel(1 << ept->bit, USB_ENDPTPRIME);

	/* mark this chain of requests as live */
	while (req) {
		req->live = 1;
		req = req->next;
	}
}

int usb_ept_queue_xfer(struct usb_endpoint *ept, struct usb_request *_req)
{
	unsigned long flags;
	struct msm_request *req = to_msm_request(_req);
	struct msm_request *last;
	struct usb_info *ui = ept->ui;
	struct ept_queue_item *item = req->item;
	unsigned length = req->req.length;

	if (length > 0x4000)
		return -EMSGSIZE;

	spin_lock_irqsave(&ui->lock, flags);

	if (req->busy) {
		req->req.status = -EBUSY;
		spin_unlock_irqrestore(&ui->lock, flags);
		printk(KERN_WARNING
		       "usb_ept_queue_xfer() tried to queue busy request\n");
		return -EBUSY;
	}

	if (!ui->online && (ept->num != 0)) {
		req->req.status = -ENODEV;
		spin_unlock_irqrestore(&ui->lock, flags);
		printk(KERN_WARNING "usb_ept_queue_xfer() tried to queue request while offline\n");
		return -ENODEV;
	}

	req->busy = 1;
	req->live = 0;
	req->next = 0;
	req->req.status = -EBUSY;

	req->dma = dma_map_single(NULL, req->req.buf, length,
				  (ept->flags & EPT_FLAG_IN) ?
				  DMA_TO_DEVICE : DMA_FROM_DEVICE);

	/* prepare the transaction descriptor item for the hardware */
	item->next = TERMINATE;
	item->info = INFO_BYTES(length) | INFO_IOC | INFO_ACTIVE;
	item->page0 = req->dma;
	item->page1 = (req->dma + 0x1000) & 0xfffff000;
	item->page2 = (req->dma + 0x2000) & 0xfffff000;
	item->page3 = (req->dma + 0x3000) & 0xfffff000;

	/* Add the new request to the end of the queue */
	last = ept->last;
	if (last) {
		/* Already requests in the queue. add us to the
		 * end, but let the completion interrupt actually
		 * start things going, to avoid hw issues
		 */
		last->next = req;

		/* only modify the hw transaction next pointer if
		 * that request is not live
		 */
		if (!last->live)
			last->item->next = req->item_dma;
	} else {
		/* queue was empty -- kick the hardware */
		ept->req = req;
		usb_ept_start(ept);
	}
	ept->last = req;

	spin_unlock_irqrestore(&ui->lock, flags);
	return 0;
}

int usb_ept_flush(struct usb_endpoint *ept)
{
	printk(KERN_INFO "%s \n", __func__);
	flush_endpoint(ept);
	return 0;
}

int usb_ept_get_max_packet(struct usb_endpoint *ept)
{
	return ept->max_pkt;
}

/* --- endpoint 0 handling --- */

static void set_configuration(struct usb_info *ui)
{
	unsigned int i, n, online;

	online = ui->online;
	for (i = 0; i < ui->num_funcs; i++) {
		struct usb_function_info *fi = ui->func[i];

		if (fi->endpoints == 0)
			continue;

		for (n = 0; n < fi->endpoints; n++)
			usb_ept_enable(fi->ept[n].ept, online);

		fi->func->configure(online, fi->func->context);
	}
}

static void ep0out_complete(struct usb_endpoint *ept, struct usb_request *req)
{
	req->complete = 0;
}

static void ep0in_complete(struct usb_endpoint *ept, struct usb_request *req)
{
	/* queue up the receive of the ACK response from the host */
	if (req->status == 0) {
		struct usb_info *ui = ept->ui;
		req->length = 0;
		req->complete = ep0out_complete;
		usb_ept_queue_xfer(&ui->ep0out, req);
	}
}

static void ep0in_complete_sendzero(struct usb_endpoint *ept, struct usb_request *req)
{
	if (req->status == 0) {
		struct usb_info *ui = ept->ui;
		req->length = 0;
		req->complete = ep0in_complete;
		usb_ept_queue_xfer(&ui->ep0in, req);
	}
}

#ifdef USB_TEST_MODE
static void ep0_status_complete(
		struct usb_endpoint *ept, struct usb_request *req)
{
	struct usb_info *ui = ept->ui;
	unsigned int i;

	if (!ui->test_mode)
		return;

	switch (ui->test_mode) {
	case J_TEST:
		pr_info("usb electrical test mode: (J)\n");
		i = readl(USB_PORTSC) & (~PORTSC_PTC);
		writel(i | PORTSC_PTC_J_STATE, USB_PORTSC);
		break;

	case K_TEST:
		pr_info("usb electrical test mode: (K)\n");
		i = readl(USB_PORTSC) & (~PORTSC_PTC);
		writel(i | PORTSC_PTC_K_STATE, USB_PORTSC);
		break;

	case SE0_NAK_TEST:
		pr_info("usb electrical test mode: (SE0-NAK)\n");
		i = readl(USB_PORTSC) & (~PORTSC_PTC);
		writel(i | PORTSC_PTC_SE0_NAK, USB_PORTSC);
		break;

	case TST_PKT_TEST:
		pr_info("usb electrical test mode: (TEST_PKT)\n");
		i = readl(USB_PORTSC) & (~PORTSC_PTC);
		writel(i | PORTSC_PTC_TST_PKT, USB_PORTSC);
		break;
	default:
		pr_err("usb:%s: undefined test mode: (%x)\n",
				__func__, ui->test_mode);
	}

}
static void ep0_setup_ack(struct usb_info *ui)
{
	struct usb_request *req = ui->setup_req;
	req->length = 0;
	req->complete = ep0_status_complete;
	usb_ept_queue_xfer(&ui->ep0in, req);
}
#else
static void ep0_setup_ack(struct usb_info *ui)
{
	struct usb_request *req = ui->setup_req;
	req->length = 0;
	req->complete = 0;
	usb_ept_queue_xfer(&ui->ep0in, req);
}
#endif

static void ep0_setup_stall(struct usb_info *ui)
{
	writel((1<<16) | (1<<0), USB_ENDPTCTRL(0));
}

static void ep0_setup_send(struct usb_info *ui, unsigned wlen)
{
	struct usb_request *req = ui->setup_req;
	struct usb_endpoint *ept = &ui->ep0in;

	/* never send more data than the host requested */
	if (req->length > wlen)
		req->length = wlen;

	/* if we are sending a short response that ends on
	 * a packet boundary, we'll need to send a zero length
	 * packet as well.
	 */
	if ((req->length != wlen) && ((req->length & 63) == 0))
		req->complete = ep0in_complete_sendzero;
	else
		req->complete = ep0in_complete;

	usb_ept_queue_xfer(ept, req);
}


static int usb_find_descriptor(struct usb_info *ui, unsigned id, struct usb_request *req);

static void handle_setup(struct usb_info *ui)
{
	struct usb_ctrlrequest ctl;

	memcpy(&ctl, ui->ep0out.head->setup_data, sizeof(ctl));
	writel(EPT_RX(0), USB_ENDPTSETUPSTAT);

	/* any pending ep0 transactions must be canceled */
	flush_endpoint(&ui->ep0out);
	flush_endpoint(&ui->ep0in);

#if 0
	printk(KERN_DEBUG
	       "setup: type=%02x req=%02x val=%04x idx=%04x len=%04x\n",
	       ctl.bRequestType, ctl.bRequest, ctl.wValue,
	       ctl.wIndex, ctl.wLength);
#endif

	if (ctl.bRequestType == (USB_DIR_IN | USB_TYPE_VENDOR)) {
		if (ctl.bRequest == MS_VENDOR_CODE) {
			struct usb_request *req = ui->setup_req;
			printk(KERN_DEBUG "vendor specific request\n");
			if (ctl.wIndex == 0x0004 && ctl.wLength <= sizeof(ms_comp_desc)) {
				printk(KERN_DEBUG "Get OS feature descriptor\n");
				memcpy((char *)&ms_comp_desc.compatibleID[0], "MTP", 3);
				memcpy(req->buf, (char *)&ms_comp_desc, sizeof(ms_comp_desc));
				req->length = ctl.wLength;
				ep0_setup_send(ui, ctl.wLength);
				return;
			}
		}
	}
	if (ctl.bRequestType == (USB_DIR_IN | USB_TYPE_STANDARD)) {
		if (ctl.bRequest == USB_REQ_GET_STATUS) {
			if (ctl.wLength == 2) {
				struct usb_request *req = ui->setup_req;
				req->length = 2;
				memset(req->buf, 0, 2);
				ep0_setup_send(ui, 2);
				return;
			}
		}
		if (ctl.bRequest == USB_REQ_GET_DESCRIPTOR) {
			struct usb_request *req = ui->setup_req;

			if (!usb_find_descriptor(ui, ctl.wValue, req)) {
				ep0_setup_send(ui, ctl.wLength);
				return;
			}
		}

		if (ctl.bRequest == USB_REQ_GET_CONFIGURATION) {
			struct usb_request *req = ui->setup_req;
			char bConfigureVal;
			bConfigureVal = ui->online;
			req->length = 1;
			memcpy(req->buf, &bConfigureVal, req->length);
			ep0_setup_send(ui, ctl.wLength);
			return;
		}

	}
	if (ctl.bRequestType == (USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_ENDPOINT)) {
		if (ctl.bRequest == USB_REQ_CLEAR_FEATURE) {
			if ((ctl.wValue == 0) && (ctl.wLength == 0)) {
				unsigned num = ctl.wIndex & 0x0f;

				if (num != 0) {
					if (ctl.wIndex & 0x80)
						num += 16;

					usb_ept_enable(ui->ept + num, 1);
					ep0_setup_ack(ui);
					return;
				}
			}
		}
		if (ctl.bRequest == USB_REQ_SET_FEATURE) {
			/* Set Endpoint_Halt */
			if ((ctl.wValue == 0) && (ctl.wLength == 0)) {
				unsigned num = ctl.wIndex & 0x0f;
				struct usb_endpoint *ept;

				if (num != 0) {
					if (ctl.wIndex & 0x80)
						num += 16;
					if (num < 32) {
						ept = ui->ept + num;
						ept->flags |= EPT_FLAG_HALT;
						ep0_setup_ack(ui);
						return;
					}
				}
			}
		}
	}

	if (ctl.bRequestType ==
		(USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_ENDPOINT)) {
		if (ctl.bRequest == USB_REQ_GET_STATUS) {
			if ((ctl.wValue == 0) && (ctl.wLength == 2)) {
				struct usb_request *req = ui->setup_req;
				unsigned num = ctl.wIndex & 0x0f;
				struct usb_endpoint *ept;
				char status[2];

				if (ctl.wIndex & 0x80)
					num += 16;
				if (num < 32) {
					ept = ui->ept + num;
					req->length = 2;
					status[0] = (ept->flags & EPT_FLAG_HALT)?1:0;
					status[1] = 0;
					memcpy(req->buf, &status, 2);
					ep0_setup_send(ui, 2);
					return;
				}
			}
		}
	}

	if (ctl.bRequestType == (USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_INTERFACE)) {
		if (ctl.bRequest == USB_REQ_SET_INTERFACE) {
			if ((ctl.wValue == 0) /*&& (ctl.wIndex == 0)*/ && (ctl.wLength == 0)) {
				/* XXX accept for non-0 interfaces */
				ep0_setup_ack(ui);
				return;
			}
		}
	}

	if (ctl.bRequestType ==
		(USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_INTERFACE)) {
		if (ctl.bRequest == USB_REQ_GET_INTERFACE) {
			struct usb_request *req = ui->setup_req;

			req->length = 1;
			memset(req->buf, 0, 1);
			ep0_setup_send(ui, ctl.wLength);
			return;
		}
	}

	if (ctl.bRequestType == (USB_DIR_OUT | USB_TYPE_STANDARD)) {
		if (ctl.bRequest == USB_REQ_SET_CONFIGURATION) {
			ui->online = !!ctl.wValue;
			set_configuration(ui);
			configure_endpoints(ui);
			goto ack;
		}
		if (ctl.bRequest == USB_REQ_SET_ADDRESS) {
			/* write address delayed (will take effect
			** after the next IN txn)
			*/
			writel((ctl.wValue << 25) | (1 << 24), USB_DEVICEADDR);
			goto ack;
		}
	}

	if ((ctl.bRequestType & USB_TYPE_MASK) != USB_TYPE_STANDARD) {
		/* let functions handle vendor and class requests */

		int i;
		for (i = 0; i < ui->num_funcs; i++) {
			struct usb_function_info *fi = ui->func[i];

			if (fi->func->setup) {
				if (ctl.bRequestType & USB_DIR_IN) {
					/* IN request */
					struct usb_request *req = ui->setup_req;

					int ret = fi->func->setup(&ctl,
						req->buf,
						SETUP_BUF_SIZE,
						fi->func->context);
					if (ret >= 0) {
						req->length = ret;
						ep0_setup_send(ui, ctl.wLength);
						return;
					}
				} else {
					/* OUT request */
					/* FIXME - support reading setup
					 * data from host.
					 */
					int ret = fi->func->setup(&ctl, NULL, 0,
							fi->func->context);
					if (ret >= 0)
						goto ack;
					else if (ret == DATA_STAGE_REQUIRED) {
						/*printk(KERN_DEBUG "Control: Data Stage\n");*/
						return;
					}
				}
			}
		}
	}

	#ifdef USB_TEST_MODE
	if ((ctl.bRequestType & (USB_DIR_MASK)) == (USB_DIR_OUT)) {
		if (ctl.bRequest == USB_REQ_SET_FEATURE) {
			if (ctl.wLength != 0)
				return;
			switch (ctl.bRequestType & USB_RECIP_MASK) {
			case USB_RECIP_DEVICE:
				if (ctl.wValue == USB_DEVICE_TEST_MODE) {
					if (ctl.wIndex & 0x0f)
						break;
					ui->test_mode = ctl.wIndex;
					ep0_setup_ack(ui);
					return;
				}
				break;

			default:
				pr_err("usb: %s: set_feature: unrecognized recipient\n",
						__func__);
				break;
			}
		}
	}
	#endif

	ep0_setup_stall(ui);
	return;

ack:
	ep0_setup_ack(ui);
}

static void handle_endpoint(struct usb_info *ui, unsigned bit)
{
	struct usb_endpoint *ept = ui->ept + bit;
	struct msm_request *req;
	unsigned long flags;
	unsigned info;

#if 0
	printk(KERN_DEBUG "handle_endpoint() %d %s req=%p(%08x)\n",
	       ept->num, (ept->flags & EPT_FLAG_IN) ? "in" : "out",
	       ept->req, ept->req ? ept->req->item_dma : 0);
#endif

	/* expire all requests that are no longer active */
	spin_lock_irqsave(&ui->lock, flags);
	while ((req = ept->req)) {
		info = req->item->info;

		/* if we've processed all live requests, time to
		 * restart the hardware on the next non-live request
		 */
		if (!req->live) {
			usb_ept_start(ept);
			break;
		}

		/* if the transaction is still in-flight, stop here */
		if (info & INFO_ACTIVE)
			break;

		/* advance ept queue to the next request */
		ept->req = req->next;
		if (ept->req == 0)
			ept->last = 0;

		dma_unmap_single(NULL, req->dma, req->req.length,
				 (ept->flags & EPT_FLAG_IN) ?
				 DMA_TO_DEVICE : DMA_FROM_DEVICE);

		if (info & (INFO_HALTED | INFO_BUFFER_ERROR | INFO_TXN_ERROR)) {
			/* XXX pass on more specific error code */
			req->req.status = -EIO;
			req->req.actual = 0;
			printk(KERN_INFO "hsusb: ept %d %s error. info=%08x\n",
			       ept->num,
			       (ept->flags & EPT_FLAG_IN) ? "in" : "out",
			       info);
		} else {
			req->req.status = 0;
			req->req.actual = req->req.length - ((info >> 16) & 0x7FFF);
		}
		req->busy = 0;
		req->live = 0;
		if (req->dead)
			do_free_req(ui, req);

		if (req->req.complete) {
			spin_unlock_irqrestore(&ui->lock, flags);
			req->req.complete(ept, &req->req);
			spin_lock_irqsave(&ui->lock, flags);
		}
	}
	spin_unlock_irqrestore(&ui->lock, flags);
}

#define FLUSH_WAIT_US	10
#define FLUSH_TIMEOUT	(2 * (USEC_PER_SEC / FLUSH_WAIT_US))
static void flush_endpoint_hw(struct usb_info *ui, unsigned bits)
{
	uint32_t unflushed = 0;
	uint32_t stat = 0;
	int cnt = 0;

	/* flush endpoint, canceling transactions
	** - this can take a "large amount of time" (per databook)
	** - the flush can fail in some cases, thus we check STAT
	**   and repeat if we're still operating
	**   (does the fact that this doesn't use the tripwire matter?!)
	*/
	while (cnt < FLUSH_TIMEOUT) {
		writel(bits, USB_ENDPTFLUSH);
		while (((unflushed = readl(USB_ENDPTFLUSH)) & bits) &&
		       cnt < FLUSH_TIMEOUT) {
			cnt++;
			udelay(FLUSH_WAIT_US);
		}

		stat = readl(USB_ENDPTSTAT);
		if (cnt >= FLUSH_TIMEOUT)
			goto err;
		if (!(stat & bits))
			goto done;
		cnt++;
		udelay(FLUSH_WAIT_US);
	}

err:
	pr_warning("%s: Could not complete flush! NOT GOOD! "
		   "stat: %x unflushed: %x bits: %x\n", __func__,
		   stat, unflushed, bits);
	return;

done:
	/* pr_info("%s: Flush took ~%dus\n", __func__, cnt * FLUSH_WAIT_US); */
	return;
}

static void flush_endpoint_sw(struct usb_endpoint *ept)
{
	struct usb_info *ui = ept->ui;
	struct msm_request *req, *next;
	unsigned long flags;

	/* inactive endpoints have nothing to do here */
	if (ept->max_pkt == 0)
		return;

	/* put the queue head in a sane state */
	ept->head->info = 0;
	ept->head->next = TERMINATE;

	/* cancel any pending requests */
	spin_lock_irqsave(&ui->lock, flags);
	req = ept->req;
	ept->req = 0;
	ept->last = 0;
	while (req != 0) {
		next = req->next;
		req->busy = 0;
		req->live = 0;
		req->req.status = -ENODEV;
		req->req.actual = 0;

		if (req->dead)
			req->dead = 0;

		if (req->req.complete) {
			spin_unlock_irqrestore(&ui->lock, flags);
			req->req.complete(ept, &req->req);
			spin_lock_irqsave(&ui->lock, flags);
		}
		req = req->next;
	}

	spin_unlock_irqrestore(&ui->lock, flags);

}

static void flush_endpoint(struct usb_endpoint *ept)
{
	flush_endpoint_hw(ept->ui, (1 << ept->bit));
	flush_endpoint_sw(ept);
}

static void flush_all_endpoints(struct usb_info *ui)
{
	unsigned n;

	flush_endpoint_hw(ui, 0xffffffff);

	for (n = 0; n < 32; n++)
		flush_endpoint_sw(ui->ept + n);
}

/* add usb_connected notify behavior ===== */
static DEFINE_MUTEX(notify_sem);
/* 0: None, 1: USB host or unknown charger, 2: China AC */
static atomic_t connect_type = ATOMIC_INIT(0);
static atomic_t atomic_usb_connected = ATOMIC_INIT(0);
static struct work_struct usb_connect_notifier_wq;

static void send_usb_connect_notify(struct work_struct *send_usb_wq)
{
	static struct t_usb_status_notifier *notifier = NULL;

	mutex_lock(&notify_sem);
	list_for_each_entry(notifier,
		&g_lh_usb_notifier_list,
		notifier_link) {
			if (notifier->func != NULL) {
				/* Notify other drivers about connect type. */
				notifier->func(atomic_read(&connect_type));
			}
		}
	mutex_unlock(&notify_sem);
}

int usb_register_notifier(struct t_usb_status_notifier *notifier)
{
	if (!notifier || !notifier->name || !notifier->func)
		return -EINVAL;

	mutex_lock(&notify_sem);
	list_add(&notifier->notifier_link,
		&g_lh_usb_notifier_list);
	mutex_unlock(&notify_sem);
	return 0;
}

#ifdef MSM_HSUSB_SHOW_USB_NOTIFIER_MESSAGE
static void usb_notify_connected(int connected)
{
	printk(KERN_DEBUG "\n\n%s() RUN(%d)...............\n\n\n", __func__, connected);
}

static struct t_usb_status_notifier usb_notifier = {
	.name = "usb_connected",
	.func = usb_notify_connected,
};
#endif /* MSM_HSUSB_SHOW_USB_NOTIFIER_MESSAGE */
/* END: add usb_connected notify behavior ===== */

static irqreturn_t usb_interrupt(int irq, void *data)
{
	struct usb_info *ui = data;
	unsigned n;

	n = readl(USB_USBSTS);
	writel(n, USB_USBSTS);

	/* somehow we got an IRQ while in the reset sequence: ignore it */
	if (ui->running == 0)
		return IRQ_HANDLED;

	if (n & STS_PCI) {
		switch (readl(USB_PORTSC) & PORTSC_PSPD_MASK) {
		case PORTSC_PSPD_FS:
			printk(KERN_INFO "usb: portchange USB_SPEED_FULL\n");
			ui->speed = USB_SPEED_FULL;
			break;
		case PORTSC_PSPD_LS:
			printk(KERN_INFO "usb: portchange USB_SPEED_LOW\n");
			ui->speed = USB_SPEED_LOW;
			break;
		case PORTSC_PSPD_HS:
			printk(KERN_INFO "usb: portchange USB_SPEED_HIGH\n");
			ui->speed = USB_SPEED_HIGH;
			break;
		}
	}

	if (n & STS_URI) {
		printk(KERN_INFO "usb: reset\n");

		writel(readl(USB_ENDPTSETUPSTAT), USB_ENDPTSETUPSTAT);
		writel(readl(USB_ENDPTCOMPLETE), USB_ENDPTCOMPLETE);
		writel(0xffffffff, USB_ENDPTFLUSH);
		writel(0, USB_ENDPTCTRL(1));

		if (ui->online != 0) {
			/* marking us offline will cause ept queue attempts to fail */
			ui->online = 0;

			flush_all_endpoints(ui);

			/* XXX: we can't seem to detect going offline, so deconfigure
			 * XXX: on reset for the time being
			 */
			set_configuration(ui);
		}
		if (atomic_read(&atomic_usb_connected) == 0) {
			atomic_set(&atomic_usb_connected, 1);
			atomic_set(&connect_type, 1);
			schedule_work(&usb_connect_notifier_wq);
		}
	}

	if (n & STS_SLI)
		printk(KERN_INFO "usb: suspend\n");

	if (n & STS_UI) {
		n = readl(USB_ENDPTSETUPSTAT);
		if (n & EPT_RX(0))
			handle_setup(ui);

		n = readl(USB_ENDPTCOMPLETE);
		writel(n, USB_ENDPTCOMPLETE);
		while (n) {
			unsigned bit = __ffs(n);
			handle_endpoint(ui, bit);
			n = n & (~(1 << bit));
		}
	}
	return IRQ_HANDLED;
}

static void msm_hsusb_request_reset(struct usb_info *reset_ui)
{
	unsigned long flags;
	spin_lock_irqsave(&reset_ui->lock, flags);
	reset_ui->flags |= USB_FLAG_RESET;
	schedule_work(&reset_ui->work);
	spin_unlock_irqrestore(&reset_ui->lock, flags);
}

static ssize_t show_usb_cable_connect(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	unsigned length;
	length = sprintf(buf, "%d\n", atomic_read(&atomic_usb_connected));
	return length;
}

static DEVICE_ATTR(usb_cable_connect, 0444, show_usb_cable_connect, NULL);

static int usb_setting_serial_number_mfg = -1;
static int usb_mfg_recovery_mode = 0;
static char mfg_df_serialno[16];

static ssize_t show_usb_serial_number(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	unsigned length;
	struct msm_hsusb_platform_data *pdata = dev->platform_data;

	length = sprintf(buf, "%s", pdata->serial_number);
	return length;
}

static ssize_t store_usb_serial_number(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct usb_info *ui = the_usb_info;
	struct msm_hsusb_platform_data *pdata = dev->platform_data;

	if (buf[0] == '0' || buf[0] == '1') {
		memset(mfg_df_serialno, 0x0, sizeof(mfg_df_serialno));
		if (buf[0] == '0') {
			strncpy(mfg_df_serialno, "000000000000", strlen("000000000000"));
			usb_setting_serial_number_mfg = 1;
		}	else	{
			strncpy(mfg_df_serialno, pdata->serial_number, strlen(pdata->serial_number));
			usb_setting_serial_number_mfg = 0;
		}
		/* reset_device */
		if (ui)
			msm_hsusb_request_reset(ui);
	}

	return count;
}

static DEVICE_ATTR(usb_serial_number, 0644, show_usb_serial_number, store_usb_serial_number);

static ssize_t show_dummy_usb_serial_number(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	unsigned length;
	struct msm_hsusb_platform_data *pdata = dev->platform_data;

	if (usb_setting_serial_number_mfg)
		length = sprintf(buf, "%s", mfg_df_serialno); /* dummy */
	else
		length = sprintf(buf, "%s", pdata->serial_number); /* Real */
	return length;
}

static ssize_t store_dummy_usb_serial_number(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct usb_info *ui = the_usb_info;
	int data_buff_size = (sizeof(mfg_df_serialno) > strlen(buf))?
		strlen(buf):sizeof(mfg_df_serialno);
	int loop_i;

	/* avoid overflow, mfg_df_serialno[16] always is 0x0 */
	if (data_buff_size == 16)
		data_buff_size--;

	for (loop_i = 0; loop_i < data_buff_size; loop_i++)	{
		if (buf[loop_i] >= 0x30 && buf[loop_i] <= 0x39) /* 0-9 */
			continue;
		else if (buf[loop_i] >= 0x41 && buf[loop_i] <= 0x5A) /* A-Z */
			continue;
		if (buf[loop_i] == 0x0A) /* Line Feed */
			continue;
		else {
			printk(KERN_WARNING "%s(): get invaild char (0x%2.2X)\n", __func__, buf[loop_i]);
			return -EINVAL;
		}
	}

	if (!usb_setting_serial_number_mfg)
		usb_setting_serial_number_mfg = 1;
	memset(mfg_df_serialno, 0x0, sizeof(mfg_df_serialno));
	strncpy(mfg_df_serialno, buf, data_buff_size);
	/*device_reset */
	if (ui)
		msm_hsusb_request_reset(ui);

	return count;
}

static DEVICE_ATTR(dummy_usb_serial_number, 0644,
	show_dummy_usb_serial_number, store_dummy_usb_serial_number);

static ssize_t show_usb_function_switch(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	unsigned length = 0, loop_i;
	struct msm_hsusb_platform_data *pdata;
	struct usb_function_info *fi;
	struct usb_info *ui = the_usb_info;
	pdata = ui->pdev->dev.platform_data;
	for (loop_i = 0; loop_i < pdata->num_functions; loop_i++) {
		fi = usb_find_function(pdata->functions[loop_i]);
		if (!fi)	{
			printk(KERN_ERR "%s(): usb_find_function fail\n", __func__);
			return 0;
		}
		length += sprintf(buf + length, "%s:%s\n", fi->func->name, (fi->enabled)?"enable":"disable");
	}
	return length;
}

int usb_function_switch(unsigned func_switch)
{
	unsigned loop_i;
	struct usb_info *ui = the_usb_info;
	struct msm_hsusb_platform_data *pdata;
	struct usb_function_info *fi;
	unsigned u;
	printk(KERN_INFO "%s: %d\n", __func__, func_switch);

#ifdef CONFIG_MACH_BRAVO
	/* prevent idle pwrc for internet sharing */
	if (func_switch == 4)
		wake_lock(&usb_inet_idle_lock);
	else
		wake_unlock(&usb_inet_idle_lock);
#endif

	pdata = ui->pdev->dev.platform_data;
	u = func_switch;
	if (u < 0 || u > 1023)
		return -1;
	if (u == enabled_functions)
		return -1;

	for (loop_i = 0; loop_i < pdata->num_functions; loop_i++) {
		fi = usb_find_function(pdata->functions[loop_i]);
		if (!fi) {
			printk(KERN_ERR "%s(): usb_find_function fail\n", __func__);
			return -1;
		}
		if (u & (1 << fi->func->position_bit))
			fi->enabled = 1;
		else
			fi->enabled = 0;
	}

	if (ui->state == USB_STATE_ONLINE)
	msm_hsusb_request_reset(ui);
	return 0;
}


static ssize_t store_usb_function_switch(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned u;
	ssize_t  ret;

	u = simple_strtoul(buf, NULL, 10);
	ret = usb_function_switch(u);

	if (ret == 0)
		return count;
	else
		return ret;
}

static DEVICE_ATTR(usb_function_switch, 0666,
	show_usb_function_switch, store_usb_function_switch);


static int mfg_USB_ID_pin_status;
static ssize_t show_USB_ID_status(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	unsigned length;

	length = sprintf(buf, "%d", mfg_USB_ID_pin_status);
	printk(KERN_INFO "mfg_USB_ID_pin_status %d, buf %s  \n", mfg_USB_ID_pin_status, buf);
	return length;
}

static DEVICE_ATTR(USB_ID_status, 0444,
	show_USB_ID_status, NULL);
static ssize_t show_usb_phy_setting(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct usb_info *ui = the_usb_info;
	unsigned length = 0;
	int i;

	for (i = 0; i <= 0x14; i++)
		length += sprintf(buf + length, "0x%x = 0x%x\n", i, ulpi_read(ui, i));

	for (i = 0x30; i <= 0x37; i++)
		length += sprintf(buf + length, "0x%x = 0x%x\n", i, ulpi_read(ui, i));

	return length;
}

static ssize_t store_usb_phy_setting(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct usb_info *ui = the_usb_info;
	char *token[10];
	unsigned reg;
	unsigned value;
	int i;

	printk(KERN_INFO "%s\n", buf);
	for (i = 0; i < 2; i++)
		token[i] = strsep((char **)&buf, " ");

	reg = simple_strtoul(token[0], NULL, 16);
	value = simple_strtoul(token[1], NULL, 16);
	printk(KERN_INFO "Set 0x%x = 0x%x\n", reg, value);

	ulpi_write(ui, value, reg);

	return 0;
}

static DEVICE_ATTR(usb_phy_setting, 0666,
	show_usb_phy_setting, store_usb_phy_setting);

#if defined(CONFIG_USB_FUNCTION_CAT_KIT)
static int dock_online = 0;
static ssize_t dock_status_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	if (dock_online)
		return sprintf(buf, "online\n");
	else
		return sprintf(buf, "offline\n");
}
static DEVICE_ATTR(status, S_IRUGO | S_IWUSR, dock_status_show, NULL);

static int mfg_usb_carkit_enable = 0;
static ssize_t show_mfg_carkit_enable(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	unsigned length;

	length = sprintf(buf, "%d", mfg_usb_carkit_enable);
	printk(KERN_INFO "show mfg_usb_carkit_enable %d, buf %s  \n", mfg_usb_carkit_enable, buf);
	return length;

}

static ssize_t store_mfg_carkit_enable(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long ul;

	if ((buf[0] != '0' && buf[0] != '1') && buf[1] != '\n')
	{
		printk(KERN_ERR"Can't enable/disable mfg_usb_carkit_enable %s\n", buf);
		return -EINVAL;
	}
	ul = simple_strtoul(buf, NULL, 10);
	mfg_usb_carkit_enable = ul;
	if(mfg_usb_carkit_enable == 1 && dock_online == 1 &&
		usb_setting_serial_number_mfg == 1){
		switch_set_state(&dock_switch, DOCK_STATE_CAR );
	}
	printk(KERN_INFO "store mfg_usb_carkit_enable %d, buf %s  \n", mfg_usb_carkit_enable, buf);
	return count;
}

static DEVICE_ATTR(usb_mfg_carkit_enable, 0644, show_mfg_carkit_enable, store_mfg_carkit_enable);
/*usb_carkit_enable == 1, car kit plugin, Navi AP would popup up, defult disable for MFG*/

#endif

static void usb_prepare(struct usb_info *ui)
{
	int ret = -1;

	spin_lock_init(&ui->lock);

	memset(ui->buf, 0, 4096);
	ui->head = (void *) (ui->buf + 0);

	/* only important for reset/reinit */
	memset(ui->ept, 0, sizeof(ui->ept));
	ui->next_item = 0;
	ui->next_ifc_num = 0;

	init_endpoints(ui);

	ui->ep0in.max_pkt = 64;
	ui->ep0out.max_pkt = 64;

	ui->setup_req = usb_ept_alloc_req(&ui->ep0in, SETUP_BUF_SIZE);

	INIT_WORK(&ui->work, usb_do_work);

#ifdef MSM_HSUSB_SHOW_USB_NOTIFIER_MESSAGE
	usb_register_notifier(&usb_notifier);
#endif
	INIT_WORK(&usb_connect_notifier_wq, send_usb_connect_notify);

	ret = device_create_file(&ui->pdev->dev,
		&dev_attr_usb_cable_connect);
	if (ret != 0) {
		printk(KERN_WARNING "dev_attr_usb_cable_connect failed\n");
	}

	ret = device_create_file(&ui->pdev->dev,
		&dev_attr_usb_serial_number);
	if (ret != 0) {
		printk(KERN_WARNING "dev_attr_usb_serial_number failed\n");
	}

	ret = device_create_file(&ui->pdev->dev,
		&dev_attr_dummy_usb_serial_number);
	if (ret != 0) {
		printk(KERN_WARNING "dev_attr_dummy_usb_serial_number failed\n");
	}

	ret = device_create_file(&ui->pdev->dev,
		&dev_attr_usb_function_switch);
	if (ret != 0) {
		printk(KERN_WARNING "dev_attr_usb_function_switch failed\n");
	}

	ret = device_create_file(&ui->pdev->dev,
		&dev_attr_USB_ID_status);
	if (ret != 0) {
		printk(KERN_WARNING "dev_attr_USB_ID_status failed\n");
	}
	ret = device_create_file(&ui->pdev->dev,
		&dev_attr_usb_phy_setting);
	if (ret != 0)
		printk(KERN_WARNING "dev_attr_usb_phy_setting failed\n");

#if defined(CONFIG_USB_FUNCTION_CAT_KIT)
	ret = device_create_file(&ui->pdev->dev,
		&dev_attr_usb_mfg_carkit_enable);
	if (ret != 0)
		printk(KERN_WARNING "dev_attr_usb_mfg_carkit_enable failed\n");
#endif

}

static void usb_suspend_phy(struct usb_info *ui)
{
	printk(KERN_INFO "%s\n", __func__);
#ifdef CONFIG_ARCH_MSM7X00A
	/* disable unused interrupt */
	ulpi_write(ui, 0x01, 0x0d);
	ulpi_write(ui, 0x01, 0x10);

	/* disable interface protect circuit to drop current consumption */
	ulpi_write(ui, (1 << 7), 0x08);
	/* clear the SuspendM bit -> suspend the PHY */
	ulpi_write(ui, 1 << 6, 0x06);
#else
	ulpi_read(ui, 0x14); /* clear PHY interrupt latch register*/

	ulpi_write(ui, 0x08, 0x09);/* turn off PLL on integrated phy */

	/* set phy to be in lpm */
	writel(readl(USB_PORTSC) | PORTSC_PHCD, USB_PORTSC);
	mdelay(1);
	if (!(readl(USB_PORTSC) & PORTSC_PHCD))
		printk(KERN_INFO "%s: unable to set lpm\n", __func__);
#endif
	/* FIXME: need to check the below lines.
	 *        'adb' could be unstable if the below lines are not commented out.
	 */
#if 0
	/* disable interface protect circuit to drop current consumption */
	ulpi_write(ui, (1 << 7), 0x08);
	/* clear the SuspendM bit -> suspend the PHY */
	ulpi_write(ui, 1 << 6, 0x06);
#endif
}

static void usb_bind_driver(struct usb_info *ui, struct usb_function_info *fi)
{
	struct usb_endpoint *ept;
	struct usb_endpoint_descriptor *ed;
	struct usb_endpoint *elist[10];
	struct usb_function *func = fi->func;
	unsigned n, count;

	printk(KERN_DEBUG "usb_bind_func() '%s'\n", func->name);

	if (func->ifc_num > 2)
		return;

	count = func->ifc_ept_count;

	if (count > 8)
		return;

	fi->ifc[0].bLength = USB_DT_INTERFACE_SIZE;
	fi->ifc[0].bDescriptorType = USB_DT_INTERFACE;
	fi->ifc[0].bAlternateSetting = 0;
	fi->ifc[0].bNumEndpoints = count;
	fi->ifc[0].bInterfaceClass = func->ifc_class;
	fi->ifc[0].bInterfaceSubClass = func->ifc_subclass;
	fi->ifc[0].bInterfaceProtocol = func->ifc_protocol;
	fi->ifc[0].iInterface = func->ifc_index;

	if (func->cdc_desc) {
		fi->ifc[0].bNumEndpoints = 1;
		fi->ifc[1].bLength = USB_DT_INTERFACE_SIZE;
		fi->ifc[1].bDescriptorType = USB_DT_INTERFACE;
		fi->ifc[1].bAlternateSetting = 0;
		fi->ifc[1].bNumEndpoints = 2;
		fi->ifc[1].bInterfaceClass = 0x0A;
		fi->ifc[1].bInterfaceSubClass = 0;
		fi->ifc[1].bInterfaceProtocol = 0;
		fi->ifc[1].iInterface = func->ifc_index;
	}
	for (n = 0; n < count; n++) {
		ept = alloc_endpoint(ui, fi, func->ifc_ept_type[n]);
		if (!ept) {
			printk(KERN_WARNING
			       "failed to allocated endpoint %d\n", n);
			free_endpoints(ui, fi);
			return;
		}
		ed = &(fi->ept[n].desc);

		ed->bLength = USB_DT_ENDPOINT_SIZE;
		ed->bDescriptorType = USB_DT_ENDPOINT;
		ed->bEndpointAddress = ept->num | ((ept->flags & EPT_FLAG_IN) ? 0x80 : 0);
		ed->bmAttributes = (func->ifc_ept_type[n] == EPT_INT_IN || func->ifc_ept_type[n] == EPT_INT_OUT) ? 0x03 : 0x02;
		ed->bInterval = (func->ifc_ept_type[n] == EPT_INT_IN || func->ifc_ept_type[n] == EPT_INT_OUT) ? 4 : 0;

		elist[n] = ept;
		ept->type = func->ifc_ept_type[n];
		fi->ept[n].ept = ept;
	}

	elist[count] = &ui->ep0in;
	elist[count + 1] = &ui->ep0out;

	fi->ifc[0].bInterfaceNumber = ui->next_ifc_num++;

	/* Workaround:  RNDIS have to be interfaceNumber = 0*/
	if (func->cdc_desc) {
		fi->ifc[0].bInterfaceNumber = 0;
		fi->ifc[1].bInterfaceNumber = 1;
		ui->next_ifc_num++;
	}
	fi->endpoints = count;

	func->bind(elist, func->context);
}

static void usb_reset(struct usb_info *ui)
{
	unsigned long flags;
	printk(KERN_INFO "hsusb: reset controller\n");

	spin_lock_irqsave(&ui->lock, flags);
	ui->running = 0;
	spin_unlock_irqrestore(&ui->lock, flags);

#if 0
	/* we should flush and shutdown cleanly if already running */
	writel(0xffffffff, USB_ENDPTFLUSH);
	msleep(2);
#endif
	/* disable usb interrupts */
	writel(0, USB_USBINTR);

	/* wait for a while after enable usb clk*/
	msleep(5);

	/* RESET */
	writel(2, USB_USBCMD);
	msleep(10);

	if (ui->phy_reset)
		ui->phy_reset();

#ifdef CONFIG_ARCH_MSM7X00A
	/* INCR8 BURST mode */
	writel(0x02, USB_SBUSCFG);	/*boost performance to fix CRC error.*/
#else
	/* bursts of unspecified length. */
	writel(0, USB_AHBBURST);
	/* Use the AHB transactor */
	writel(0, USB_AHBMODE);
#endif
	/* select DEVICE mode */
	writel(0x12, USB_USBMODE);
	msleep(1);

	/* select ULPI phy */
	writel(0x80000000, USB_PORTSC);

	ulpi_init(ui);

	writel(ui->dma, USB_ENDPOINTLISTADDR);

	configure_endpoints(ui);

	/* marking us offline will cause ept queue attempts to fail */
	ui->online = 0;

	/* terminate any pending transactions */
	flush_all_endpoints(ui);

	printk(KERN_DEBUG "usb: notify offline\n");
	set_configuration(ui);

	/* enable interrupts */
	writel(STS_URI | STS_SLI | STS_UI | STS_PCI, USB_USBINTR);

	/* go to RUN mode (D+ pullup enable) */
	writel(0x00080001, USB_USBCMD);

	spin_lock_irqsave(&ui->lock, flags);
	ui->running = 1;
	spin_unlock_irqrestore(&ui->lock, flags);
}

static void usb_start(struct usb_info *ui)
{
	unsigned long flags;
	unsigned count = 0;
	int i;

	for (i = 0; i < ui->num_funcs; i++) {
		struct usb_function_info *fi = ui->func[i];
		usb_bind_driver(ui, fi);
		if (fi->endpoints)
			count++;
	}

	if (count == 0) {
		printk(KERN_DEBUG
		       "usb_start: no functions bound. not starting\n");
		return;
	}

	spin_lock_irqsave(&ui->lock, flags);
	ui->flags |= USB_FLAG_START;
	schedule_work(&ui->work);
	spin_unlock_irqrestore(&ui->lock, flags);
}

static LIST_HEAD(usb_function_list);
static DEFINE_MUTEX(usb_function_list_lock);

static struct usb_info *the_usb_info;

static struct usb_function_info *usb_find_function(const char *name)
{
	struct list_head *entry;
	list_for_each(entry, &usb_function_list) {
		struct usb_function_info *fi =
			list_entry(entry, struct usb_function_info, list);

		if (!strcmp(name, fi->func->name))
			return fi;
	}

	return NULL;
}

struct usb_fi_ept *get_ept_info(const char *function)
{
	struct usb_function_info *fi;
	fi = usb_find_function(function);
	if (!fi)
		return NULL;
	return &fi->ept[0];
}

struct usb_interface_descriptor *get_ifc_desc(const char *function)
{
	struct usb_function_info *fi;
	fi = usb_find_function(function);
	if (!fi)
		return NULL;
	return &fi->ifc[0];
}
static void usb_try_to_bind(void)
{
	struct usb_info *ui = the_usb_info;
	struct msm_hsusb_platform_data *pdata;
	struct usb_function_info *fi;
	int i;

	if (!ui || ui->bound || !ui->pdev)
		return;

	pdata = ui->pdev->dev.platform_data;

	for (i = 0; i < pdata->num_functions; i++) {
		if (ui->func[i])
			continue;
		fi = usb_find_function(pdata->functions[i]);
		if (!fi)
			return;
		ui->func[i] = fi;
		ui->num_funcs++;
	}

	/* we have found all the needed functions */
	ui->bound = 1;
	printk(KERN_DEBUG "%s: functions bound. starting.\n", __func__);
	usb_start(ui);
}

int usb_function_register(struct usb_function *driver)
{
	struct usb_function_info *fi;
	unsigned n;
	int ret = 0;

	printk(KERN_INFO "%s() '%s'\n", __func__, driver->name);

	mutex_lock(&usb_function_list_lock);

	n = driver->ifc_ept_count;
	fi = kzalloc(sizeof(*fi) + n * sizeof(struct usb_fi_ept), GFP_KERNEL);
	if (!fi) {
		ret = -ENOMEM;
		goto fail;
	}
	fi->func = driver;
	fi->enabled = !driver->disabled;
	list_add(&fi->list, &usb_function_list);

	usb_try_to_bind();

fail:
	mutex_unlock(&usb_function_list_lock);
	return ret;
}

unsigned return_usb_function_enabled(const char *function)
{
	struct usb_function_info *fi = usb_find_function(function);
	return fi->enabled;
}

int usb_function_enable(const char *function, int enable)
{
	struct usb_function_info *fi = usb_find_function(function);
	struct usb_info *ui = the_usb_info;

	if (fi && fi->enabled != enable) {
		fi->enabled = enable;

		if (ui) {
			msm_hsusb_request_reset(ui);
		}
	}
	return fi?fi->enabled:0;
}

static int usb_free(struct usb_info *ui, int ret)
{
	if (ui->irq)
		free_irq(ui->irq, 0);
	if (ui->pool)
		dma_pool_destroy(ui->pool);
	if (ui->dma)
		dma_free_coherent(&ui->pdev->dev, 4096, ui->buf, ui->dma);
	if (ui->addr)
		iounmap(ui->addr);
	if (ui->clk)
		clk_put(ui->clk);
	if (ui->pclk)
		clk_put(ui->pclk);
	if (ui->ebi1clk)
		clk_put(ui->ebi1clk);
	kfree(ui);
	return ret;
}

static void usb_do_work_check_vbus(struct usb_info *ui)
{
	unsigned long iflags;


#if defined(CONFIG_USB_BYPASS_VBUS_NOTIFY)
	ui->flags |= USB_FLAG_VBUS_ONLINE;/*because battery driver fail*/
	pr_info("%s: ignore check vbus \n", __func__);
	return;
#endif
	spin_lock_irqsave(&ui->lock, iflags);

	if (vbus)
		ui->flags |= USB_FLAG_VBUS_ONLINE;
	else
		ui->flags |= USB_FLAG_VBUS_OFFLINE;
	spin_unlock_irqrestore(&ui->lock, iflags);
}

static void usb_switch_dummy_serial_num_by_GPIO(void)
{
	unsigned n = 0;
	struct usb_info *ui = the_usb_info;
	struct msm_hsusb_platform_data *pdata;

	pdata = ui->pdev->dev.platform_data;

	if (pdata->usb_id_pin_gpio != 0) {
		n = gpio_get_value(pdata->usb_id_pin_gpio);
		printk(KERN_INFO "usb: Get USB ID PIN, mfg_df_serialno: %s,%s,usb_id_pin_gpio %d \n",
				mfg_df_serialno, __func__, pdata->usb_id_pin_gpio);
	} else {
		n = 1;/*normal cable*/
		printk(KERN_INFO "usb:%s, usb_id_pin_gpio %d\n",
		   __func__, pdata->usb_id_pin_gpio);
	}

	if (n == 1) {
		mfg_USB_ID_pin_status = 1;
		printk(KERN_INFO "usb: mfg_USB_ID_pin_status: %d, ID_Pin 1: %x,%s \n", mfg_USB_ID_pin_status, n, __func__);

	} else {
		mfg_USB_ID_pin_status = 0;
		printk(KERN_INFO "usb: mfg_USB_ID_pin_status: %d, ID_Pin 0: %x,%s \n", mfg_USB_ID_pin_status, n, __func__);
	}
}

static void usb_lpm_enter(struct usb_info *ui)
{
	unsigned long iflags;
	if (ui->in_lpm)
		return;
	spin_lock_irqsave(&ui->lock, iflags);
	usb_suspend_phy(ui);
	clk_disable(ui->pclk);
	clk_disable(ui->clk);
	clk_set_rate(ui->ebi1clk, 0);
	ui->in_lpm = 1;
	spin_unlock_irqrestore(&ui->lock, iflags);
}

#if defined(CONFIG_USB_FUNCTION_CAT_KIT)
void enter_lpm(struct usb_info *ui)
{
	unsigned n = 0;

	printk(KERN_INFO "usb EnterLPM ++\r\n");
	n =  readl(USB_PORTSC);
	printk(KERN_INFO "usb: USB_PORTSC, %x\n",n);
	writel(n |(PORTSC_PHCD), USB_PORTSC);
	printk(KERN_INFO "usb EnterLPM --\r\n");
}

static void exit_lpm(struct usb_info *ui)
{
	/* Set the PHCD bit in the port control register to zero.
	** This restarts the PHY clock and transfers the PHY into
	** normal operating state.
	*/
	unsigned n = 0;
	int itest_count = 0;

	printk(KERN_INFO "usb ExitLPM ++\r\n");
	n =  readl(USB_PORTSC);
	printk(KERN_INFO "usb: USB_PORTSC, %x\n",n);
	writel(n & ~(PORTSC_PHCD), USB_PORTSC);
	// Wait until the PHY finished its transition out of low power mode.
	while ((readl(USB_PORTSC) & PORTSC_PHCD) != 0) {
		itest_count++;
		if(itest_count>10000){
			printk(KERN_INFO "usb: Wait USB_PORTSC, %x,%x\n",
			readl(USB_PORTSC),(readl(USB_PORTSC) & PORTSC_PHCD));
			break;
		}
	}

	msleep(3);
	printk(KERN_INFO "usb ExitLPM --USB_PORTSC, %x,%x\r\n",
			readl(USB_PORTSC),(readl(USB_PORTSC) & PORTSC_PHCD));
}

static void usb_car_kit_wait_id_pin(struct usb_info *ui, unsigned  want_value){
	unsigned  gpio_value = 0;
	int itest_count = 0;

	while (1) {
			gpio_value = gpio_get_value(ui ->usb_id_pin_gpio);
			if(gpio_value == want_value || itest_count>10000){
				break;
			}
			itest_count++;
		}
	printk(KERN_INFO "usb: Wait car usb_id_pin_gpio , itest_count %d, gpio_value %d\n",
				itest_count,gpio_value);
}

static void usb_car_kit_detect(struct usb_info *ui)
{
	unsigned n = 0, gpio_value = 0;

	if(ui->enable_car_kit_detect == 0){
		printk(KERN_INFO "usb: no enable car kit detect\n");
		return;
	}

	if(car_kit_detect_start == 1){

		gpio_value = gpio_get_value(ui ->usb_id_pin_gpio);
		if(gpio_value == 1){
			set_irq_type(ui->irq_usb_id_btn, IRQF_TRIGGER_LOW );
			enable_irq(ui->irq_usb_id_btn);
			car_kit_detect_start = 0;
			printk(KERN_INFO "usb: no car kit\n");
			return;
		}
		if (ui->config_usb_id_gpios !=NULL){
			ui->config_usb_id_gpios(1);
		}

		clk_set_rate(ui->ebi1clk, 128000000);
		udelay(10);
		clk_enable(ui->clk);
		clk_enable(ui->pclk);

		exit_lpm(ui);
		n =  readl(USB_OTGSC);// & OTGSC_ID;
		printk(KERN_INFO "usb: first car kit  USB_OTGSC, %x\n",n);

		writel(n | OTGSC_IDPU,USB_OTGSC);//This bit provides control over the ID pull-up register.

		msleep(200);
		usb_car_kit_wait_id_pin(ui,1);

		n =  readl(USB_OTGSC);// & OTGSC_ID;
		printk(KERN_INFO "usb:second  car kit  USB_OTGSC, 0x%x, 0x%x\n",n,(n & OTGSC_ID) );
		if (ui->config_usb_id_gpios !=NULL){
			ui->config_usb_id_gpios(0);
		}
		usb_car_kit_wait_id_pin(ui,0);

		if((n & OTGSC_ID) == OTGSC_ID){
			printk(KERN_INFO "usb: car kit plug-in \n");
			if((mfg_usb_carkit_enable == 1 && usb_setting_serial_number_mfg == 1) ||
				usb_setting_serial_number_mfg == 0) {
				switch_set_state(&dock_switch, DOCK_STATE_CAR );
			}else{
				printk(KERN_INFO "usb: disable send u-event for car kit plug-in in MFG\n");
			}
			dock_online = 1;/*for MFG*/
			car_kit_detect_start = 0;
			car_kit_unpluged_start = 0;
			enable_irq(ui->irq_usb_id_btn);
		}
		enter_lpm(ui);
		clk_disable(ui->pclk);
		clk_disable(ui->clk);
		clk_set_rate(ui->ebi1clk, 0);

	}

	if (car_kit_unpluged_start == 1) {
		dock_online = 0;/*for MFG*/
		car_kit_detect_start = 0;
		car_kit_unpluged_start = 0;
		switch_set_state(&dock_switch,DOCK_STATE_UNDOCKED );
		printk(KERN_INFO "usb: car kit un-pluged end \n");
	}
}
#endif

static void usb_do_work(struct work_struct *w)
{
	struct usb_info *ui = container_of(w, struct usb_info, work);
	unsigned long iflags;
	unsigned flags, _vbus;

	for (;;) {
		spin_lock_irqsave(&ui->lock, iflags);
		flags = ui->flags;
		ui->flags = 0;
		_vbus = vbus;
		spin_unlock_irqrestore(&ui->lock, iflags);
	#if defined(CONFIG_USB_FUNCTION_CAT_KIT)
		usb_car_kit_detect(ui);
	#endif
		/* give up if we have nothing to do */
		if (flags == 0)
			break;

		switch (ui->state) {
		case USB_STATE_IDLE:
			if (flags & USB_FLAG_START) {
				pr_info("hsusb: IDLE -> ONLINE\n");
				clk_set_rate(ui->ebi1clk, 128000000);
				udelay(10);
				clk_enable(ui->clk);
				clk_enable(ui->pclk);
				ui->in_lpm = 0;
				usb_reset(ui);
				usb_switch_dummy_serial_num_by_GPIO();
				/* detect USB/AC by D+/D- Line Status */
				msleep(10);
				if (_vbus && (readl(USB_PORTSC) & PORTSC_LS) != PORTSC_LS) {
					printk(KERN_INFO "usb: not AC charger\n");
					atomic_set(&connect_type, 1);
					schedule_work(&usb_connect_notifier_wq);
				} else if (_vbus) {
					printk(KERN_INFO "usb: AC charger\n");
					atomic_set(&connect_type, 2);
					schedule_work(&usb_connect_notifier_wq);
					writel(0x00080000, USB_USBCMD);
					msleep(500);
					usb_lpm_enter(ui);
				}

				ui->state = USB_STATE_ONLINE;
				usb_do_work_check_vbus(ui);
			}
			break;
		case USB_STATE_ONLINE:
			/* If at any point when we were online, we received
			 * the signal to go offline, we must honor it
			 */
			if (flags & USB_FLAG_VBUS_OFFLINE) {
				pr_info("hsusb: ONLINE -> OFFLINE\n");

				/* prevent irq context stuff from doing anything */
				spin_lock_irqsave(&ui->lock, iflags);

				atomic_set(&atomic_usb_connected, 0);
				if (atomic_read(&connect_type) != 0) {
					atomic_set(&connect_type, 0);
					schedule_work(&usb_connect_notifier_wq);
				}

				ui->running = 0;
				ui->online = 0;
				spin_unlock_irqrestore(&ui->lock, iflags);
				if (ui->in_lpm) {
					clk_enable(ui->clk);
					clk_enable(ui->pclk);
					ui->in_lpm = 0;
					msleep(5);
				}

				/* terminate any transactions, etc */
				flush_all_endpoints(ui);
				set_configuration(ui);

				/*pull low D+*/
				writel(0x00080000, USB_USBCMD);

				/* reset the phy so we know that we can
				 * suspend it */
				if (ui->phy_reset)
					ui->phy_reset();

				/* power down phy, clock down usb */
				spin_lock_irqsave(&ui->lock, iflags);
				usb_suspend_phy(ui);
				clk_disable(ui->pclk);
				clk_disable(ui->clk);
				clk_set_rate(ui->ebi1clk, 0);
				ui->in_lpm = 1;
				spin_unlock_irqrestore(&ui->lock, iflags);

				ui->state = USB_STATE_OFFLINE;
				usb_do_work_check_vbus(ui);

				break;
			}
			if (flags & USB_FLAG_RESET) {
				pr_info("hsusb: ONLINE -> RESET\n");
				if (atomic_read(&connect_type) == 2) {
					pr_info("hsusb: RESET -> ONLINE\n");
					break;
				}
				spin_lock_irqsave(&ui->lock, iflags);
				ui->online = 0;
				writel(0x00080000, USB_USBCMD);
#ifndef CONFIG_ARCH_MSM7X00A
				ulpi_write(ui, 0x48, 0x04);
#endif
				spin_unlock_irqrestore(&ui->lock, iflags);
				usb_reset(ui);
				pr_info("hsusb: RESET -> ONLINE\n");
				break;
			}
			break;
		case USB_STATE_OFFLINE:
			/* If we were signaled to go online and vbus is still
			 * present when we received the signal, go online.
			 */
			if ((flags & USB_FLAG_VBUS_ONLINE) && _vbus) {
				pr_info("hsusb: OFFLINE -> ONLINE\n");
				clk_set_rate(ui->ebi1clk, 128000000);
				udelay(10);
				clk_enable(ui->clk);
				clk_enable(ui->pclk);
				ui->in_lpm = 0;
				usb_reset(ui);
				usb_switch_dummy_serial_num_by_GPIO();
				/* detect USB/AC by D+/D- Line Status */
				msleep(10);
				if ((readl(USB_PORTSC) & PORTSC_LS) != PORTSC_LS) {
					printk(KERN_INFO "usb: not AC charger\n");
					if (atomic_read(&connect_type) != 1) {
						atomic_set(&connect_type, 1);
						schedule_work(&usb_connect_notifier_wq);
					}
				} else {
					printk(KERN_INFO "usb: AC charger\n");
					if (atomic_read(&connect_type) != 2) {
						atomic_set(&connect_type, 2);
						schedule_work(&usb_connect_notifier_wq);
					}
					writel(0x00080000, USB_USBCMD);
					msleep(500);
					usb_lpm_enter(ui);
				}

				ui->state = USB_STATE_ONLINE;
				usb_do_work_check_vbus(ui);
			}
			break;
		}
	}
}

void msm_hsusb_set_vbus_state(int online)
{
	unsigned long flags = 0;
	struct usb_info *ui = the_usb_info;

	pr_info("%s: %d\n", __func__, online);
	if (ui)
		spin_lock_irqsave(&ui->lock, flags);
	if (vbus != online) {
		vbus = online;
		if (ui) {
			if (online)
				ui->flags |= USB_FLAG_VBUS_ONLINE;
			else
				ui->flags |= USB_FLAG_VBUS_OFFLINE;

			if (ui->usb_uart_switch)
				ui->usb_uart_switch(ui->flags & USB_FLAG_VBUS_OFFLINE); /* low for usb */

			schedule_work(&ui->work);
		}
	}
	if (ui)
		spin_unlock_irqrestore(&ui->lock, flags);
}

void usb_function_reenumerate(void)
{
	struct usb_info *ui = the_usb_info;

	/* disable and re-enable the D+ pullup */
	printk(KERN_DEBUG "%s(): runing...\n", __func__);
	writel(0x00080000, USB_USBCMD);
	msleep(10);
	writel(0x00080001, USB_USBCMD);
}

#if defined(CONFIG_DEBUG_FS)
static char debug_buffer[PAGE_SIZE];

static ssize_t debug_read_status(struct file *file, char __user *ubuf,
				 size_t count, loff_t *ppos)
{
	struct usb_info *ui = file->private_data;
	char *buf = debug_buffer;
	unsigned long flags;
	struct usb_endpoint *ept;
	struct msm_request *req;
	int n;
	int i = 0;

	spin_lock_irqsave(&ui->lock, flags);

	i += scnprintf(buf + i, PAGE_SIZE - i,
		       "regs: setup=%08x prime=%08x stat=%08x done=%08x\n",
		       readl(USB_ENDPTSETUPSTAT),
		       readl(USB_ENDPTPRIME),
		       readl(USB_ENDPTSTAT),
		       readl(USB_ENDPTCOMPLETE));
	i += scnprintf(buf + i, PAGE_SIZE - i,
		       "regs:   cmd=%08x   sts=%08x intr=%08x port=%08x\n\n",
		       readl(USB_USBCMD),
		       readl(USB_USBSTS),
		       readl(USB_USBINTR),
		       readl(USB_PORTSC));


	for (n = 0; n < 32; n++) {
		ept = ui->ept + n;
		if (ept->max_pkt == 0)
			continue;

		i += scnprintf(buf + i, PAGE_SIZE - i,
			       "ept%d %s cfg=%08x active=%08x next=%08x info=%08x\n",
			       ept->num, (ept->flags & EPT_FLAG_IN) ? "in " : "out",
			       ept->head->config, ept->head->active,
			       ept->head->next, ept->head->info);

		for (req = ept->req; req; req = req->next)
			i += scnprintf(buf + i, PAGE_SIZE - i,
				       "  req @%08x next=%08x info=%08x page0=%08x %c %c\n",
				       req->item_dma, req->item->next,
				       req->item->info, req->item->page0,
				       req->busy ? 'B' : ' ',
				       req->live ? 'L' : ' '
				);
	}

	i += scnprintf(buf + i, PAGE_SIZE - i,
		       "phy failure count: %d\n", ui->phy_fail_count);

	spin_unlock_irqrestore(&ui->lock, flags);

	return simple_read_from_buffer(ubuf, count, ppos, buf, i);
}

static ssize_t debug_write_reset(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct usb_info *ui = file->private_data;
	unsigned long flags;

	spin_lock_irqsave(&ui->lock, flags);
	ui->flags |= USB_FLAG_RESET;
	schedule_work(&ui->work);
	spin_unlock_irqrestore(&ui->lock, flags);

	return count;
}

static ssize_t debug_write_cycle(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	usb_function_reenumerate();
	return count;
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

const struct file_operations debug_stat_ops = {
	.open = debug_open,
	.read = debug_read_status,
};

const struct file_operations debug_reset_ops = {
	.open = debug_open,
	.write = debug_write_reset,
};

const struct file_operations debug_cycle_ops = {
	.open = debug_open,
	.write = debug_write_cycle,
};

static void usb_debugfs_init(struct usb_info *ui)
{
	struct dentry *dent;
	dent = debugfs_create_dir("usb", 0);
	if (IS_ERR(dent))
		return;

	debugfs_create_file("status", 0444, dent, ui, &debug_stat_ops);
	debugfs_create_file("reset", 0222, dent, ui, &debug_reset_ops);
	debugfs_create_file("cycle", 0222, dent, ui, &debug_cycle_ops);
}
#else
static void usb_debugfs_init(struct usb_info *ui) {}
#endif

int usb_check_mfg_recovery_mode()
{
	return usb_mfg_recovery_mode;
}

#if defined(CONFIG_USB_FUNCTION_CAT_KIT)
static irqreturn_t usb_car_kit_interrupt_handler(int irq, void *dev_id){

	struct usb_info *ui = dev_id;
	unsigned i = 0, gpio_value = 0;
	int retry_limit = 10;
	int value1, value2;
	bool unstable = 0;
	static int itestcount = 0;

	gpio_value = gpio_get_value(ui ->usb_id_pin_gpio);
	for(i = 0; i<10;i++){
		value1 = gpio_get_value(ui ->usb_id_pin_gpio);
		if(gpio_value != value1){
			unstable = 1;
		}
		printk(KERN_INFO "%s(): IO %d = %d, v1= %d, unstable= %d\n",__func__,ui ->usb_id_pin_gpio,gpio_value,value1,unstable);
	}
	if(unstable == 1 && itestcount<100){
		printk(KERN_INFO "%s(): return\n",__func__);
		itestcount++;

		return IRQ_HANDLED;
	}
	do {
		value1 = gpio_get_value(ui ->usb_id_pin_gpio);
			set_irq_type(ui->irq_usb_id_btn, value1 ?
				     IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
		value2 = gpio_get_value(ui ->usb_id_pin_gpio);
	} while (value1 != value2 && retry_limit-- > 0);
	printk(KERN_INFO "%s(): itestcount %d\n",__func__,itestcount);
	if(value1 == 0){
		printk(KERN_INFO "usb: car_kit_detect_start \n");
		disable_irq(ui->irq_usb_id_btn);
		car_kit_detect_start = 1;
	}else{
		printk(KERN_INFO "usb: car kit un-pluged start \n");
		car_kit_unpluged_start = 1;
		}
	schedule_work(&ui->work);
	return IRQ_HANDLED;
}

static void usb_car_kit_irq_init(struct usb_info *ui)
{
	int ret;

	ret = gpio_request(ui->usb_id_pin_gpio, "USB_car_kit_button");
	if (ret < 0){
		printk(KERN_ERR "%s(): gpio_request  fail, ui->usb_id_pin_gpio %d\n",
			__func__,ui->usb_id_pin_gpio);
		goto err_request_detect_gpio;
	}

	ret = gpio_direction_input(ui->usb_id_pin_gpio);
	if (ret < 0){
		printk(KERN_ERR "%s(): gpio_direction_input  fail, ui->usb_id_pin_gpio %d\n",
			__func__,ui->usb_id_pin_gpio);
		goto err_set_detect_gpio;
	}

	ui->irq_usb_id_btn = gpio_to_irq(ui->usb_id_pin_gpio);
	if (ui->irq_usb_id_btn < 0) {
		printk(KERN_ERR "%s(): 1 gpio_to_irq  fail, ui->usb_id_pin_gpio %d\n",
			__func__,ui->usb_id_pin_gpio);
		goto err_get_carkit_detect_irq_num_failed;
	}

	printk(KERN_INFO "%s():  ui->usb_id_pin_gpio %d\n",
				__func__,ui->usb_id_pin_gpio);

	ret = request_irq(ui->irq_usb_id_btn, usb_car_kit_interrupt_handler,
				IRQF_TRIGGER_LOW, "USB_car_kit_button", ui);
	if (ret < 0){
		printk(KERN_ERR "%s(): request_irq  fail, ui->usb_id_pin_gpio %d\n",
			__func__,ui->usb_id_pin_gpio);
		goto err_request_carkit_button_irq;
	}

	ret = set_irq_wake(ui->irq_usb_id_btn, 1);
	if (ret < 0){
		printk(KERN_ERR "%s(): set_irq_wake  fail, ui->usb_id_pin_gpio %d\n",
			__func__,ui->usb_id_pin_gpio);
		goto err_set_irq_wake;
	}
	if (switch_dev_register(&dock_switch) == 0){
		ret = device_create_file(dock_switch.dev, &dev_attr_status);
		WARN_ON(ret);
	}else{
		printk(KERN_ERR "fail to register dock switch!\n");
	}

err_set_irq_wake:
err_request_carkit_button_irq:
	if (ui->usb_id_pin_gpio)
		free_irq(ui->irq_usb_id_btn, 0);
err_get_carkit_detect_irq_num_failed:
err_set_detect_gpio:
	if (ui->usb_id_pin_gpio)
		gpio_free(ui->usb_id_pin_gpio);
err_request_detect_gpio:
	printk(KERN_ERR "%s(): gpio_request  fail, ui->usb_id_pin_gpio %d\n",
			__func__,ui->usb_id_pin_gpio);

}
#endif

static int usb_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct usb_info *ui;
	int irq;
	int ret;

	ui = kzalloc(sizeof(struct usb_info), GFP_KERNEL);
	if (!ui)
		return -ENOMEM;

	spin_lock_init(&ui->lock);
	ui->pdev = pdev;

	if (pdev->dev.platform_data) {
		struct msm_hsusb_platform_data *pdata = pdev->dev.platform_data;
		ui->phy_reset = pdata->phy_reset;
		ui->phy_init_seq = pdata->phy_init_seq;
		ui->usb_uart_switch = pdata->usb_uart_switch;
	#if defined(CONFIG_USB_FUNCTION_CAT_KIT)
		if(pdata->enable_car_kit_detect != 0){
			ui->enable_car_kit_detect = pdata->enable_car_kit_detect;
			printk(KERN_INFO "%s(): pdata->enable_car_kit_detect, %d\n",__func__,pdata->enable_car_kit_detect);
		}
		if(ui->enable_car_kit_detect == 1){
			if(pdata->usb_id_pin_gpio != 0){
			ui->usb_id_pin_gpio = pdata->usb_id_pin_gpio;
				printk(KERN_INFO "%s(): pdata->usb_id_pin_gpio, %d\n",__func__,pdata->usb_id_pin_gpio);
			}
			if(pdata->config_usb_id_gpios != NULL){
				ui->config_usb_id_gpios= pdata->config_usb_id_gpios;
				printk(KERN_INFO "%s(): config_usb_id_gpios \n",__func__);
			}
		}
	#endif
		/* USB device descriptor fields */
		desc_device.idVendor = pdata->vendor_id;
		desc_device.idProduct = pdata->product_id;
		desc_device.bcdDevice = pdata->version;
		if (pdata->serial_number)
			desc_device.iSerialNumber = STRING_SERIAL;
		if (pdata->product_name)
			desc_device.iProduct = STRING_PRODUCT;
		if (pdata->manufacturer_name)
			desc_device.iManufacturer = STRING_MANUFACTURER;

		ui->func = kzalloc(sizeof(struct usb_function *) *
				   pdata->num_functions, GFP_KERNEL);
		if (!ui->func) {
			kfree(ui);
			return -ENOMEM;
		}
	}

	irq = platform_get_irq(pdev, 0);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || (irq < 0))
		return usb_free(ui, -ENODEV);

	ui->addr = ioremap(res->start, 4096);
	if (!ui->addr)
		return usb_free(ui, -ENOMEM);

	ui->buf = dma_alloc_coherent(&pdev->dev, 4096, &ui->dma, GFP_KERNEL);
	if (!ui->buf)
		return usb_free(ui, -ENOMEM);

	ui->pool = dma_pool_create("hsusb", NULL, 32, 32, 0);
	if (!ui->pool)
		return usb_free(ui, -ENOMEM);

	printk(KERN_INFO "usb_probe() io=%p, irq=%d, dma=%p(%x)\n",
	       ui->addr, irq, ui->buf, ui->dma);

	ui->clk = clk_get(&pdev->dev, "usb_hs_clk");
	if (IS_ERR(ui->clk))
		return usb_free(ui, PTR_ERR(ui->clk));

	ui->pclk = clk_get(&pdev->dev, "usb_hs_pclk");
	if (IS_ERR(ui->pclk))
		return usb_free(ui, PTR_ERR(ui->pclk));

	/* memory barrier initialization in non-interrupt context */
	dmb();
	ui->ebi1clk = clk_get(NULL, "ebi1_clk");
	if (IS_ERR(ui->ebi1clk))
		return usb_free(ui, PTR_ERR(ui->ebi1clk));

	/* disable interrupts before requesting irq */
	clk_enable(ui->clk);
	clk_enable(ui->pclk);
	writel(0, USB_USBINTR);
	writel(readl(USB_OTGSC) & ~OTGSC_INTR_MASK, USB_OTGSC);
	clk_disable(ui->pclk);
	clk_disable(ui->clk);

	ret = request_irq(irq, usb_interrupt, 0, pdev->name, ui);
	if (ret)
		return usb_free(ui, ret);
	enable_irq_wake(irq);
	ui->irq = irq;

	the_usb_info = ui;

	usb_debugfs_init(ui);

	usb_prepare(ui);

	/* initialize mfg serial number */

	switch (board_mfg_mode()) {
	case 0: /*normal mode*/
		usb_setting_serial_number_mfg = 0;
		break;
	case 1: /*factory mode*/
		usb_setting_serial_number_mfg = 1;
		break;
	case 2: /*recovery mode*/
		usb_setting_serial_number_mfg = 0;
		usb_mfg_recovery_mode = 1;
		break;
	case 3: /*charge mode*/
		usb_setting_serial_number_mfg = 0;
		break;
	default:
		usb_setting_serial_number_mfg = 0;
		break;
	}
	strncpy(mfg_df_serialno, "000000000000", strlen("000000000000"));

#ifdef CONFIG_MACH_BRAVO
	wake_lock_init(&usb_inet_idle_lock, WAKE_LOCK_IDLE, "usb_inet_idle_lock");
#endif

#if defined(CONFIG_USB_FUNCTION_CAT_KIT)
	if(ui->enable_car_kit_detect == 1){
		usb_car_kit_irq_init(ui);
	}
#endif
	return 0;
}

static struct platform_driver usb_driver = {
	.probe = usb_probe,
	.driver = { .name = "msm_hsusb", },
};

static int __init usb_init(void)
{
	return platform_driver_register(&usb_driver);
}

module_init(usb_init);

static void copy_string_descriptor(char *string, char *buffer)
{
	int length, i;

	if (string) {
		length = strlen(string);
		buffer[0] = 2 * length + 2;
		buffer[1] = USB_DT_STRING;
		for (i = 0; i < length; i++) {
			buffer[2 * i + 2] = string[i];
			buffer[2 * i + 3] = 0;
		}
	}
}

static void copy_os_string_descriptor(char *string, char *buffer)
{
	int length, i;

	if (string) {
		length = strlen(string);
		buffer[0] = 2 * length + 4;
		buffer[1] = USB_DT_STRING;
		for (i = 0; i < length; i++) {
			buffer[2 * i + 2] = string[i];
			buffer[2 * i + 3] = 0;
		}
		buffer[2 * length + 2] = MS_VENDOR_CODE;
		buffer[2 * length + 3] = 0;
	}
}

static int usb_find_descriptor(struct usb_info *ui, unsigned id, struct usb_request *req)
{
	struct msm_hsusb_platform_data *pdata = ui->pdev->dev.platform_data;
	int i;
	unsigned type = id >> 8;
	id &= 0xff;

	if ((type == USB_DT_DEVICE) && (id == 0)) {
		/* Compute our product ID based on which of our functions
		** are enabled. Also compute and save our list of enabled
		** functions to avoid a race condition between the
		** USB_DT_DEVICE and USB_DT_CONFIG requests.
		** (that is, make sure the product ID we return for
		** USB_DT_DEVICE matches the list of interfaces we
		** return for USB_DT_CONFIG.
		*/
		enabled_functions = 0;
		for (i = 0; i < ui->num_funcs; i++) {
			struct usb_function_info *fi = ui->func[i];
			if (fi->enabled)
				enabled_functions |= (1 << fi->func->position_bit);
		}

		if (enabled_functions & (1 << USB_FUNCTION_INTERNET_SHARING_NUM))
			desc_device.bDeviceClass = 0x02;
		else
			desc_device.bDeviceClass = 0;

		/* default product ID */
		desc_device.idProduct = pdata->product_id;
		/* set idProduct based on which functions are enabled */
		for (i = 0; i < pdata->num_products; i++) {

			if (pdata->products[i].functions == enabled_functions) {
				struct msm_hsusb_product *product =
					&pdata->products[i];
				if (product->functions == enabled_functions)
					desc_device.idProduct =
						product->product_id;
			}
		}
		/*
		printk(KERN_INFO "GET USB -> VID: 0x%4.4X, PID: 0x%4.4X, (0x%X) ###\n",
			desc_device.idVendor,
			desc_device.idProduct,enabled_functions);
		*/

		req->length = sizeof(desc_device);
		memcpy(req->buf, &desc_device, req->length);
		return 0;
	}

	if ((type == USB_DT_DEVICE_QUALIFIER) && (id == 0)) {
		req->length = sizeof(dev_qualifier);
		memcpy(req->buf, &dev_qualifier, req->length);
		return 0;
	}

	if (((type == USB_DT_CONFIG) || (type == USB_DT_OTHER_SPEED_CONFIG))
		&& (id == 0)) {
		struct usb_config_descriptor cfg;
		unsigned ifc_count = 0;
		unsigned n;
		char *ptr, *start;
		int max_packet, other_max_packet;

		if (ui->speed == USB_SPEED_HIGH) {
			max_packet = 512;
			other_max_packet = 64;
		} else {
			max_packet = 64;
			other_max_packet = 512;
		}
		start = req->buf;
		ptr = start + USB_DT_CONFIG_SIZE;

		for (i = 0; i < ui->num_funcs; i++) {
			struct usb_function_info *fi = ui->func[i];

			/* check to see if the function was enabled when we
			** received the USB_DT_DEVICE request to make ensure
			** that the interfaces we return here match the
			** product ID we returned in the USB_DT_DEVICE.
			*/
			/* modify it, because occur lost interface/endpoint desctiptor
			 * by Anthony_Chang */
			if (fi->enabled)	{
				if (fi->func->cdc_desc) {
					struct usb_endpoint_descriptor *ept_desc;
					struct usb_descriptor_header **src = fi->func->cdc_desc;
					fi->ifc[0].bInterfaceNumber = ifc_count;
					fi->ifc[1].bInterfaceNumber = ifc_count+1;
					ifc_count += 2;
					for (; NULL != *src; src++) {
						unsigned len = (*src)->bLength;
						ept_desc = (struct usb_endpoint_descriptor *)ptr;
						memcpy(ptr, *src, len);

						if (((*src)->bDescriptorType == USB_DT_ENDPOINT) &&
							(ept_desc->bmAttributes == USB_ENDPOINT_XFER_BULK)) {

							if (type == USB_DT_OTHER_SPEED_CONFIG)
								ept_desc->wMaxPacketSize = other_max_packet;
							else
								ept_desc->wMaxPacketSize = max_packet;
						}
						ptr += len;
					}
				} else {
					fi->ifc[0].bInterfaceNumber = ifc_count;
					ifc_count++;
					memcpy(ptr, &fi->ifc[0], fi->ifc[0].bLength);
					ptr += fi->ifc[0].bLength;

					for (n = 0; n < fi->endpoints; n++) {
						struct usb_endpoint_descriptor *ept_desc;
						/* XXX hardcoded bulk */
						fi->ept[n].desc.wMaxPacketSize = max_packet;
						memcpy(ptr, &(fi->ept[n].desc), fi->ept[n].desc.bLength);
						ept_desc = (struct usb_endpoint_descriptor *)ptr;
						if (ept_desc->bmAttributes == USB_ENDPOINT_XFER_INT)
							ept_desc->wMaxPacketSize = 64;
						else if (type == USB_DT_OTHER_SPEED_CONFIG)
							ept_desc->wMaxPacketSize = other_max_packet;
						ptr += fi->ept[n].desc.bLength;
					}
				}
			}
		}

		cfg.bLength = USB_DT_CONFIG_SIZE;
		cfg.bDescriptorType = type;
		cfg.wTotalLength = ptr - start;
		cfg.bNumInterfaces = ifc_count;
		cfg.bConfigurationValue = 1;
		cfg.iConfiguration = 0;
		cfg.bmAttributes = 0x80;
		cfg.bMaxPower = 0xFA; /* 500mA */

		memcpy(start, &cfg, USB_DT_CONFIG_SIZE);

		req->length = ptr - start;
		return 0;
	}

	if (type == USB_DT_STRING) {
		char *buffer = req->buf;

		buffer[0] = 0;
		switch (id) {
		case STRING_LANGUAGE_ID:
			/* return language ID */
			buffer[0] = 4;
			buffer[1] = USB_DT_STRING;
			buffer[2] = LANGUAGE_ID & 0xFF;
			buffer[3] = LANGUAGE_ID >> 8;
			break;
		case STRING_SERIAL:
			if (!usb_setting_serial_number_mfg) /* real */
				copy_string_descriptor(pdata->serial_number, buffer);
			else	/* dummy */
				copy_string_descriptor(mfg_df_serialno, buffer);
			break;
		case STRING_PRODUCT:
			copy_string_descriptor(pdata->product_name, buffer);
			break;
		case STRING_MANUFACTURER:
			copy_string_descriptor(pdata->manufacturer_name, buffer);
			break;
		case STRING_UMS:
		case STRING_ADB:
		case STRING_ES:
		case STRING_FSERIAL:
		case STRING_PROJECTOR:
		case STRING_FSYNC:
		case STRING_DIAG:
		case STRING_MTP:
		case STRING_MODEM:
		case STRING_NMEA:
		case STRING_QSERIAL:
			copy_string_descriptor(strings[id-4], buffer);
		break;
		case STRING_MS_OS_DESC:
			if (enabled_functions & (1 << USB_FUNCTION_MTP_TUNNEL_NUM))
				copy_os_string_descriptor("MSFT100", buffer);
			break;
		default:
		printk(KERN_WARNING "usb:wrong string id = 0x%x\n", id);
			break;
		}

		if (buffer[0]) {
			req->length = buffer[0];
			return 0;
		} else {
			return -1;
		}
	}

	return -1;
}

void usb_free_endpoint_all_req(struct usb_endpoint *ep)
{
	struct msm_request *temp;
	struct msm_request *req;
	if (!ep)
		return;
	req = ep->req;
	while (req) {
		temp = req->next;
		req->busy = 0;
		if (&req->req)
			usb_ept_free_req(ep, &req->req);
		req = temp;
	}
}
EXPORT_SYMBOL(usb_free_endpoint_all_req);

int usb_find_interface_number(const char *name)
{
	int intr = -1;
	struct list_head *entry;
	list_for_each(entry, &usb_function_list) {
		struct usb_function_info *fi =
			list_entry(entry, struct usb_function_info, list);

		if (!strcmp(name, fi->func->name)) {
			if (fi->enabled)
				intr = fi->ifc[0].bInterfaceNumber;
			break;
		}
	}

	return intr;
}

int usb_ept_cancel_xfer(struct usb_endpoint *ept, struct usb_request *_req)
{
	struct usb_info 	*ui = the_usb_info;
	struct msm_request      *req = to_msm_request(_req);
	struct msm_request 	*temp_req, *prev_req;
	unsigned long		flags;
	int cnt;
	uint32_t stat = 0;

	if (!(ui && req && ept->req))
		return -EINVAL;

	spin_lock_irqsave(&ui->lock, flags);
	if (req->busy) {
		req->req.status = 0;
		req->busy = 0;

		/* See if the request is the first request in the ept queue */
		if (ept->req == req) {
			/* Stop the transfer */
			cnt = 0;
			while (cnt < FLUSH_TIMEOUT) {
				writel((1 << ept->bit), USB_ENDPTFLUSH);
				while ((readl(USB_ENDPTFLUSH) & (1 << ept->bit)) &&
					cnt < FLUSH_TIMEOUT) {
						cnt++;
						udelay(FLUSH_WAIT_US);
				}

				stat = readl(USB_ENDPTSTAT);
				if (cnt >= FLUSH_TIMEOUT)
					break;
				if (!(stat & (1 << ept->bit)))
					break;
				cnt++;
				udelay(FLUSH_WAIT_US);
			}
			if (!req->next)
				ept->last = NULL;
			ept->req = req->next;
			ept->head->next = req->item->next;
			goto cancel_req;
		}
		/* Request could be in the middle of ept queue */
		prev_req = temp_req = ept->req;
		do {
			if (req == temp_req) {
				if (req->live) {
					/* Stop the transfer */
					cnt = 0;
					while (cnt < FLUSH_TIMEOUT) {
						writel((1 << ept->bit), USB_ENDPTFLUSH);
						while ((readl(USB_ENDPTFLUSH) & (1 << ept->bit)) &&
							cnt < FLUSH_TIMEOUT) {
								cnt++;
								udelay(FLUSH_WAIT_US);
						}

						stat = readl(USB_ENDPTSTAT);
						if (cnt >= FLUSH_TIMEOUT)
							break;
						if (!(stat & (1 << ept->bit)))
							break;
						cnt++;
						udelay(FLUSH_WAIT_US);
					}
				}
				prev_req->next = temp_req->next;
				prev_req->item->next = temp_req->item->next;
				if (!req->next)
					ept->last = prev_req;
				goto cancel_req;
			}
			prev_req = temp_req;
			temp_req = temp_req->next;
		} while (temp_req != NULL);
		goto error;
cancel_req:
	if (req->live) {
		/* prepare the transaction descriptor item for the hardware */
		req->item->next = TERMINATE;
		req->item->info = 0;
		req->live = 0;
		/* memory barrier to flush the data */
		dmb();
		dma_unmap_single(NULL, req->dma, req->req.length,
				(ept->flags & EPT_FLAG_IN) ?
				DMA_TO_DEVICE : DMA_FROM_DEVICE);
		/* Reprime the endpoint for the remaining transfers */
		if (ept->req) {
			temp_req = ept->req;
			while (temp_req != NULL) {
				temp_req->live = 0;
				temp_req = temp_req->next;
			}
			usb_ept_start(ept);
		}
	} else
		dma_unmap_single(NULL, req->dma, req->req.length,
				(ept->flags & EPT_FLAG_IN) ?
				DMA_TO_DEVICE : DMA_FROM_DEVICE);
	spin_unlock_irqrestore(&ui->lock, flags);
	return 0;
	}
error:
	spin_unlock_irqrestore(&ui->lock, flags);
	return -EINVAL;
}
EXPORT_SYMBOL(usb_ept_cancel_xfer);

int usb_get_connect_type(void)
{
	return atomic_read(&connect_type);
}
