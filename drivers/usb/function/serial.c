/*
 * serial.c -- USB Serial Function driver
 *
 * Copyright 2003 (C) Al Borchers (alborchers@steinerpoint.com)
 * Copyright (c) 2008 QUALCOMM USA, INC.
 *
 * This code is based in part on the Gadget Zero driver, which
 * is Copyright (C) 2003 by David Brownell, all rights reserved.
 *
 * This code also borrows from usbserial.c, which is
 * Copyright (C) 1999 - 2002 Greg Kroah-Hartman (greg@kroah.com)
 * Copyright (C) 2000 Peter Berger (pberger@brimson.com)
 * Copyright (C) 2000 Al Borchers (alborchers@steinerpoint.com)
 *
 * All source code in this file is licensed under the following license except
 * where indicated.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/utsname.h>
#include <linux/wait.h>
#include <linux/serial.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <asm/byteorder.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <linux/platform_device.h>
#include <asm/mach-types.h>

#include <linux/usb/cdc.h>
#include "usb_function.h"

#include <linux/workqueue.h>
/* Defines */

#define GS_VERSION_STR			"v2.2"
#define GS_VERSION_NUM			0x0202

#define GS_LONG_NAME			"Serial Function"
#define GS_SHORT_NAME			"serial"

#define MAX_INSTANCES 3
static int instances = MAX_INSTANCES;

#define GS_MAJOR			127
#define GS_MINOR_START			0

#define GS_NUM_PORTS			3

#define GS_NO_CONFIG_ID			0
#define GS_ACM_CONFIG_ID		2

#define GS_MAX_DESC_LEN			256

/* defines for maintaining serial states */
#define	MSR_CTS		(1 << 4)
#define	MSR_DSR		(1 << 5)
#define	MSR_RI		(1 << 6)
#define	MSR_CD		(1 << 7)
#define	MCR_DTR		(1 << 0)
#define	MCR_RTS		(1 << 1)
#define	MCR_LOOP	(1 << 4)

/* USB CDC control line state defines */
#define USB_CDC_SET_CONTROL_LINE_STATE_DTR 0x1
#define USB_CDC_SET_CONTROL_LINE_STATE_RTS 0x2

#define GS_DEFAULT_READ_Q_SIZE		32
#define GS_DEFAULT_WRITE_Q_SIZE		32
#define GS_DEFAULT_INT_REQ		1

#define TRANSFER_SIZE				512
#define GS_DEFAULT_WRITE_BUF_SIZE	65536
#define GS_TMP_BUF_SIZE			8192

#define GS_CLOSE_TIMEOUT		15

#define GS_DEFAULT_USE_ACM		0

#define GS_DEFAULT_DTE_RATE		9600
#define GS_DEFAULT_DATA_BITS		8
#define GS_DEFAULT_PARITY		USB_CDC_NO_PARITY
#define GS_DEFAULT_CHAR_FORMAT		USB_CDC_1_STOP_BITS

/* #define GS_DEBUG */

/* debug settings */
#ifdef GS_DEBUG
static int debug = 1;

#define gs_debug(format, arg...) \
	do { if (debug) printk(KERN_DEBUG format, ## arg); } while (0)
#define gs_debug_level(level, format, arg...) \
	do { if (debug >= level) printk(KERN_DEBUG format, ## arg); } while (0)

#else

#define gs_debug(format, arg...) \
	do { } while (0)
#define gs_debug_level(level, format, arg...) \
	do { } while (0)

#endif /* GS_DEBUG */

#define GS_LOG2_NOTIFY_INTERVAL		5	/* 1 << 5 == 32 msec */
#define GS_NOTIFY_MAXPACKET		8
#define SERIAL_CONFIGURED        1
#define SERIAL_UNCONFIGURED      0

#define MODEM_INSTANCE		0
#define NMEA_INSTANCE		1
#define SERIAL_INSTANCE		2

/* Structures */

struct gs_dev;

/* circular buffer */
struct gs_buf {
	unsigned int buf_size;
	char *buf_buf;
	char *buf_get;
	char *buf_put;
};

/* list of requests */
struct gs_req_entry {
	struct list_head re_entry;
	struct usb_request *re_req;
};

/* the port structure holds info for each port, one for each minor number */
struct gs_port {
	struct gs_dev *port_dev;	/* pointer to device struct */
	struct tty_struct *port_tty;	/* pointer to tty struct */
	spinlock_t port_lock;
	struct mutex	mutex_lock;
	int port_num;
	int port_open_count;
	int port_in_use;	/* open/close in progress */
	wait_queue_head_t port_write_wait;	/* waiting to write */
	struct gs_buf *port_write_buf;
	struct usb_cdc_line_coding port_line_coding;
	struct list_head        read_pool;
	struct list_head        read_queue;
	struct list_head	write_pool;
	unsigned                n_read;
	unsigned int msr;
	unsigned int prev_msr;
	unsigned int mcr;
	wait_queue_head_t msr_change_wait;
	struct work_struct push_work;
};

/*-------------------------------------------------------------*/
/*Allocate DMA buffer in non interrupt context(gs_bind)*/

struct gs_reqbuf {
	void *buf;
};

/*-------------------------------------------------------------*/

/* the device structure holds info for the USB device */
struct gs_dev {
	/* lock for set/reset config */
	spinlock_t dev_lock;
	/* configuration number */
	int dev_config;
	/* address of notify endpoint */
	struct usb_endpoint *dev_notify_ep;
	/* address of in endpoint */
	struct usb_endpoint *dev_in_ep;
	struct usb_request *notify_req;
	/* address of out endpoint */
	struct usb_endpoint *dev_out_ep;
	/* list of write requests */
	struct list_head dev_req_list;
	/* round robin port scheduled */
	int dev_sched_port;
	struct gs_port *dev_port[GS_NUM_PORTS];	/* the ports */
	struct gs_reqbuf statusreqbuf;
	u16 interface_num;

	/*interface, endpoint descriptors*/
	struct usb_interface_descriptor gs_ifc_desc;
	struct usb_endpoint_descriptor gs_hs_bulkin_desc, gs_fs_bulkin_desc;
	struct usb_endpoint_descriptor gs_hs_bulkout_desc, gs_fs_bulkout_desc;
	struct usb_endpoint_descriptor gs_hs_notifyin_desc, gs_fs_notifyin_desc;
	struct usb_descriptor_header **gs_fullspeed_header;
	struct usb_descriptor_header **gs_highspeed_header;

	struct usb_function *func;
	int configured;
	int bound;
};

/* Functions */

/* module */
static int __init gs_module_init(void);
/* static void __exit gs_module_exit(void); */

static int send_notify_data(struct usb_endpoint *ep, struct usb_request *req);
/* tty driver */
static int gs_open(struct tty_struct *tty, struct file *file);
static void gs_close(struct tty_struct *tty, struct file *file);
static int gs_write(struct tty_struct *tty,
		    const unsigned char *buf, int count);
static int gs_put_char(struct tty_struct *tty, unsigned char ch);
static void gs_flush_chars(struct tty_struct *tty);
static int gs_write_room(struct tty_struct *tty);
static int gs_chars_in_buffer(struct tty_struct *tty);
static void gs_throttle(struct tty_struct *tty);
static void gs_unthrottle(struct tty_struct *tty);
static int gs_break(struct tty_struct *tty, int break_state);
static int gs_ioctl(struct tty_struct *tty, struct file *file,
		    unsigned int cmd, unsigned long arg);
static void gs_set_termios(struct tty_struct *tty, struct ktermios *old);
static unsigned gs_start_rx(struct gs_dev *dev);

static int gs_send(struct gs_dev *dev);
static int gs_send_packet(struct gs_dev *dev, char *packet, unsigned int size);
static void gs_read_complete(struct usb_endpoint *ep, struct usb_request *req);
static void gs_write_complete(struct usb_endpoint *ep, struct usb_request *req);
static int gs_tiocmget(struct tty_struct *tty, struct file *file);
static int gs_tiocmset(struct tty_struct *tty, struct file *file,
			unsigned int set, unsigned int clear);

/* Function driver */
static void gs_bind(struct usb_endpoint **ept, void *_ctxt);
static void gs_unbind(void *);
static int gs_setup(struct usb_ctrlrequest *req,
		void *buf, int len, void *_ctxt);

static void gs_configure(int config, void *_ctxt);
/* static void gs_disconnect(void *_ctxt); */
static void gs_reset_config(struct gs_dev *dev);

static struct usb_request *gs_alloc_req(struct usb_endpoint *ep,
					unsigned int len);
static void gs_free_req(struct usb_endpoint *ep, struct usb_request *req);


static int gs_alloc_ports(struct gs_dev *dev, gfp_t kmalloc_flags);
static void gs_free_ports(struct gs_dev *dev);

/* circular buffer */
static struct gs_buf *gs_buf_alloc(unsigned int size, gfp_t kmalloc_flags);
static void gs_buf_free(struct gs_buf *gb);
static void gs_buf_clear(struct gs_buf *gb);
static unsigned int gs_buf_data_avail(struct gs_buf *gb);
static unsigned int gs_buf_space_avail(struct gs_buf *gb);
static unsigned int gs_buf_put(struct gs_buf *gb, const char *buf,
			       unsigned int count);
static unsigned int gs_buf_get(struct gs_buf *gb, char *buf,
			       unsigned int count);
/* creates, returns async_icount struct with current port msr,mcr values */
static struct async_icount gs_current_icount(struct gs_port *port);

/* Globals */
static struct gs_dev **gs_devices;

static struct semaphore gs_open_close_sem[GS_NUM_PORTS];

static unsigned int read_q_size = GS_DEFAULT_READ_Q_SIZE;
static unsigned int write_q_size = GS_DEFAULT_WRITE_Q_SIZE;

static unsigned int write_buf_size = GS_DEFAULT_WRITE_BUF_SIZE;

static struct workqueue_struct *gs_tty_wq;


/* tty driver struct */
static const struct tty_operations gs_tty_ops = {
	.open = gs_open,
	.close = gs_close,
	.write = gs_write,
	.put_char = gs_put_char,
	.flush_chars = gs_flush_chars,
	.write_room = gs_write_room,
	.ioctl = gs_ioctl,
	.set_termios = gs_set_termios,
	.throttle = gs_throttle,
	.unthrottle = gs_unthrottle,
	.break_ctl = gs_break,
	.chars_in_buffer = gs_chars_in_buffer,
	.tiocmget = gs_tiocmget,
	.tiocmset = gs_tiocmset,
};
static struct tty_driver *gs_tty_driver;

/* Function  driver struct */
static struct usb_function usb_function_serial[MAX_INSTANCES];

struct usb_function *global_func_serial;
struct gs_dev **dum_device;

/* Module */
MODULE_DESCRIPTION(GS_LONG_NAME);
MODULE_AUTHOR("Al Borchers");
MODULE_LICENSE("GPL");

#ifdef GS_DEBUG
module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging, 0=off, 1=on");
#endif

module_param(read_q_size, uint, S_IRUGO);
MODULE_PARM_DESC(read_q_size, "Read request queue size, default=32");

module_param(write_q_size, uint, S_IRUGO);
MODULE_PARM_DESC(write_q_size, "Write request queue size, default=32");

module_param(write_buf_size, uint, S_IRUGO);
MODULE_PARM_DESC(write_buf_size, "Write buffer size, default=8192");

module_param(instances, int, 0);
MODULE_PARM_DESC(instances, "Number of serial instances");

module_init(gs_module_init);
/* module_exit(gs_module_exit); */

/******************************************************************************/

/*
 * CDC-ACM Class specific Descriptors
 */

static const struct usb_cdc_header_desc gs_header_desc = {
	.bLength = sizeof(gs_header_desc),
	.bDescriptorType = USB_DT_CS_INTERFACE,
	.bDescriptorSubType = USB_CDC_HEADER_TYPE,
	.bcdCDC = __constant_cpu_to_le16(0x0110),
};

static const struct usb_cdc_call_mgmt_descriptor gs_call_mgmt_descriptor = {
	.bLength = sizeof(gs_call_mgmt_descriptor),
	.bDescriptorType = USB_DT_CS_INTERFACE,
	.bDescriptorSubType = USB_CDC_CALL_MANAGEMENT_TYPE,
	.bmCapabilities = 0,
	.bDataInterface = 0,
};

static struct usb_cdc_acm_descriptor gs_acm_descriptor = {
	.bLength = sizeof(gs_acm_descriptor),
	.bDescriptorType = USB_DT_CS_INTERFACE,
	.bDescriptorSubType = USB_CDC_ACM_TYPE,
	.bmCapabilities = 3,  /* bits should be 00000011 (refer to 5.2.3.3) */
};

static const struct usb_cdc_union_desc gs_union_desc = {
	.bLength = sizeof(gs_union_desc),
	.bDescriptorType = USB_DT_CS_INTERFACE,
	.bDescriptorSubType = USB_CDC_UNION_TYPE,
	.bMasterInterface0 = 0,
	.bSlaveInterface0 = 0,
};

static void gs_init_ifc_desc(struct usb_interface_descriptor *ifc_desc)
{
	ifc_desc->bLength =		USB_DT_INTERFACE_SIZE;
	ifc_desc->bDescriptorType =	USB_DT_INTERFACE;
	ifc_desc->bNumEndpoints =	3;
	ifc_desc->bInterfaceClass =	USB_CLASS_VENDOR_SPEC;
	ifc_desc->bInterfaceSubClass =	USB_CLASS_VENDOR_SPEC;
	ifc_desc->bInterfaceProtocol =	USB_CLASS_VENDOR_SPEC;
	ifc_desc->iInterface =		0;
}

#define HIGHSPEED	1
#define	FULLSPEED	2

#define BULK	1
#define INTERRUPT	2

static void gs_init_ep_desc(struct usb_endpoint_descriptor *ep_desc,
				unsigned type, unsigned speed)
{
	ep_desc->bLength =		USB_DT_ENDPOINT_SIZE;
	ep_desc->bDescriptorType =	USB_DT_ENDPOINT;

	if (type == BULK) {
		ep_desc->bmAttributes = USB_ENDPOINT_XFER_BULK;
		if (speed == HIGHSPEED)
			ep_desc->wMaxPacketSize = 512;
		else
			ep_desc->wMaxPacketSize = 64;
	} else {

		ep_desc->bmAttributes = USB_ENDPOINT_XFER_INT;
		ep_desc->wMaxPacketSize = 64;
		ep_desc->bInterval = 4;
	}
}

static void gs_init_header_desc(struct gs_dev *dev)
{
	dev->gs_highspeed_header[0] =
		(struct usb_descriptor_header *)&dev->gs_ifc_desc;
	dev->gs_highspeed_header[1] =
		(struct usb_descriptor_header *)&dev->gs_hs_bulkin_desc;
	dev->gs_highspeed_header[2] =
		(struct usb_descriptor_header *)&dev->gs_hs_bulkout_desc;
	dev->gs_highspeed_header[3] =
		(struct usb_descriptor_header *)&dev->gs_hs_notifyin_desc;
	dev->gs_highspeed_header[4] = NULL;

	dev->gs_fullspeed_header[0] =
		(struct usb_descriptor_header *)&dev->gs_ifc_desc;
	dev->gs_fullspeed_header[1] =
		(struct usb_descriptor_header *)&dev->gs_fs_bulkin_desc;
	dev->gs_fullspeed_header[2] =
		(struct usb_descriptor_header *)&dev->gs_fs_bulkout_desc;
	dev->gs_fullspeed_header[3] =
		(struct usb_descriptor_header *)&dev->gs_fs_notifyin_desc;
	dev->gs_fullspeed_header[4] = NULL;
}

char *ifc_name[] = {"modem", "nmea", "serial"};
static ssize_t store_modem_enable(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long ul;
	if ((buf[0] != '0' && buf[0] != '1') && buf[1] != '\n') {
		printk(KERN_WARNING "Can't enable/disable modem %s\n", buf);
		return -EINVAL;
	}
	ul = simple_strtoul(buf, NULL, 10);
	usb_function_enable(ifc_name[0], ul);
	return count;
}

static ssize_t show_modem_enable(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int rc;
	rc = sprintf(buf, "%d", return_usb_function_enabled(ifc_name[0]));
	return rc;

}
static DEVICE_ATTR(modem_enable, 0644, show_modem_enable, store_modem_enable);

static ssize_t store_nmea_enable(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long ul;
	if ((buf[0] != '0' && buf[0] != '1') && buf[1] != '\n') {
		printk(KERN_WARNING "Can't enable/disable nmea %s\n", buf);
		return -EINVAL;
	}
	ul = simple_strtoul(buf, NULL, 10);
	usb_function_enable(ifc_name[1], ul);
	return count;
}

static ssize_t show_nmea_enable(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int rc;
	rc = sprintf(buf, "%d", return_usb_function_enabled(ifc_name[1]));
	return rc;

}
static DEVICE_ATTR(nmea_enable, 0644, show_nmea_enable, store_nmea_enable);

static ssize_t store_serial_enable(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long ul;
	if ((buf[0] != '0' && buf[0] != '1') && buf[1] != '\n') {
		printk(KERN_WARNING "Can't enable/disable serial %s\n", buf);
		return -EINVAL;
	}
	ul = simple_strtoul(buf, NULL, 10);
	usb_function_enable(ifc_name[2], ul);
	return count;
}

static ssize_t show_serial_enable(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int rc;
	rc = sprintf(buf, "%d", return_usb_function_enabled(ifc_name[2]));
	return rc;
}

static DEVICE_ATTR(serial_enable, 0644, show_serial_enable, store_serial_enable);

static int usbserial_probe (struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver usbserial_driver = {
	.probe = usbserial_probe,
	.driver = { .name = "usbserial", },
};

static void usbserial_release (struct device *dev) {}

static struct platform_device usbserial_device = {
	.name		= "usbserial",
	.id		= -1,
	.dev		= {
		.release	= usbserial_release,
	},
};
/*****************************************************************************/
/*
 *  gs_module_init
 *
 *  Register as a USB gadget driver and a tty driver.
 */

static int __init gs_module_init(void)
{
	int i, retval, j;
	struct usb_function *func;

	if (instances > MAX_INSTANCES || instances == 0) {
		printk(KERN_ERR "Incorrect instances entered \n");
		return -ENODEV;
	}

	gs_tty_wq = create_singlethread_workqueue("gs_tty");
	if (gs_tty_wq == 0)
		return -ENOMEM;
	gs_tty_driver = alloc_tty_driver(GS_NUM_PORTS);
	if (!gs_tty_driver) {
		destroy_workqueue(gs_tty_wq);
		return -ENOMEM;
	}
	gs_tty_driver->owner = THIS_MODULE;
	gs_tty_driver->driver_name = GS_SHORT_NAME;
	gs_tty_driver->name = "ttyHSUSB";
	gs_tty_driver->major = GS_MAJOR;
	gs_tty_driver->minor_start = GS_MINOR_START;
	gs_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	gs_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	gs_tty_driver->flags =  TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV
				| TTY_DRIVER_RESET_TERMIOS;
	gs_tty_driver->init_termios = tty_std_termios;
	gs_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL
	    | CLOCAL;
	gs_tty_driver->init_termios.c_lflag = 0;
	gs_tty_driver->init_termios.c_iflag = 0;
	gs_tty_driver->init_termios.c_oflag = 0;
	tty_set_operations(gs_tty_driver, &gs_tty_ops);

	for (i = 0; i < GS_NUM_PORTS; i++)
		sema_init(&gs_open_close_sem[i], 1);

	retval = tty_register_driver(gs_tty_driver);
	if (retval) {
		put_tty_driver(gs_tty_driver);
		printk(KERN_ERR
		       "gs_module_init: cannot register tty driver,ret = %d\n",
		       retval);
		return retval;
	}

	for (i = 0; i < MAX_INSTANCES; i++)
		tty_register_device(gs_tty_driver, i, NULL);

	gs_devices = kzalloc(sizeof(struct gs_dev *) * instances,
				GFP_KERNEL);
	if (!gs_devices) {
		retval = -ENOMEM;
		goto fail1;
	}

	for (i = 0; i < instances; i++) {
		func = &usb_function_serial[i];

		gs_devices[i] = kzalloc(sizeof(struct gs_dev), GFP_KERNEL);
		if (!gs_devices[i]) {
			retval = -ENOMEM;
			goto fail2;
		}
		spin_lock_init(&gs_devices[i]->dev_lock);
		INIT_LIST_HEAD(&gs_devices[i]->dev_req_list);
		gs_devices[i]->func = func;

		/*1 - Interface, 3 Endpoints-> Total 4 + 1 for NULL*/
		gs_devices[i]->gs_fullspeed_header =
		kmalloc(sizeof(struct usb_descriptor_header *) * 5, GFP_KERNEL);
		gs_devices[i]->gs_highspeed_header =
		kmalloc(sizeof(struct usb_descriptor_header *) * 5, GFP_KERNEL);

		/* setup usb interface descriptor */
		gs_init_ifc_desc(&gs_devices[i]->gs_ifc_desc);
		gs_init_ep_desc(&gs_devices[i]->gs_hs_bulkin_desc, BULK,
				HIGHSPEED);
		gs_init_ep_desc(&gs_devices[i]->gs_hs_bulkout_desc, BULK,
				HIGHSPEED);
		gs_init_ep_desc(&gs_devices[i]->gs_hs_notifyin_desc, INTERRUPT,
				HIGHSPEED);

		gs_init_ep_desc(&gs_devices[i]->gs_fs_bulkin_desc, BULK,
				FULLSPEED);
		gs_init_ep_desc(&gs_devices[i]->gs_fs_bulkout_desc, BULK,
				FULLSPEED);
		gs_init_ep_desc(&gs_devices[i]->gs_fs_notifyin_desc, INTERRUPT,
				FULLSPEED);
		gs_init_header_desc(gs_devices[i]);

		/*Initializing Directions*/
		gs_devices[i]->gs_hs_bulkin_desc.bEndpointAddress = USB_DIR_IN;
		gs_devices[i]->gs_hs_bulkout_desc.bEndpointAddress =
								USB_DIR_OUT;
		gs_devices[i]->gs_hs_notifyin_desc.bEndpointAddress =
								USB_DIR_IN;
		gs_devices[i]->gs_fs_bulkin_desc.bEndpointAddress = USB_DIR_IN;
		gs_devices[i]->gs_fs_bulkout_desc.bEndpointAddress =
								USB_DIR_OUT;
		gs_devices[i]->gs_fs_notifyin_desc.bEndpointAddress =
								USB_DIR_IN;

		func->bind = gs_bind;
		func->unbind = gs_unbind;
		func->configure = gs_configure;
		/* func->disconnect = gs_disconnect; */
		func->setup = gs_setup;
		func->name = ifc_name[i];
		func->context = gs_devices[i];

		func->ifc_class = gs_devices[i]->gs_ifc_desc.bInterfaceClass;
		func->ifc_subclass = gs_devices[i]->gs_ifc_desc.bInterfaceSubClass;
		func->ifc_protocol = gs_devices[i]->gs_ifc_desc.bInterfaceProtocol;
		func->ifc_name = ifc_name[i];
		func->ifc_ept_count = gs_devices[i]->gs_ifc_desc.bNumEndpoints;
		func->ifc_ept_type[0] =  EPT_INT_IN;
		func->ifc_ept_type[1] =  EPT_BULK_OUT;
		func->ifc_ept_type[2] =  EPT_BULK_IN;

		/* func->fs_descriptors = gs_devices[i]->gs_fullspeed_header; */
		/* func->hs_descriptors = gs_devices[i]->gs_highspeed_header; */
		if (i == MODEM_INSTANCE) {
			func->disabled = 1;
			func->position_bit = USB_FUNCTION_MODEM_NUM;
			func->ifc_index = STRING_MODEM;
		} else if (i == NMEA_INSTANCE) {
			func->disabled = 1;
			func->position_bit = USB_FUNCTION_NMEA_NUM;
			func->ifc_index = STRING_NMEA;
		} else if (i == SERIAL_INSTANCE) {
			func->disabled = 1;
			func->position_bit = USB_FUNCTION_SERIAL_NUM;
			func->ifc_index = STRING_QSERIAL;
		}

		retval = usb_function_register(func);
		if (retval) {
			printk(KERN_ERR
	      "gs_module_init: cannot register Function driver, ret = %d\n",
			       retval);
			goto fail3;
		}
	}

	retval = platform_driver_register(&usbserial_driver);
	if (retval < 0) {
		printk("fail3\n");
		goto fail3;
	}
	retval = platform_device_register(&usbserial_device);
	if (retval < 0) {
		printk("fail4\n");
		goto fail4;
	}
	retval = device_create_file(&usbserial_device.dev, &dev_attr_modem_enable);
	if (retval != 0)
		goto fail5;

	retval = device_create_file(&usbserial_device.dev, &dev_attr_nmea_enable);
	if (retval != 0)
		goto fail6;

	retval = device_create_file(&usbserial_device.dev, &dev_attr_serial_enable);
	if (retval != 0)
		goto fail7;

	return 0;

/*free all allocated resources*/
fail7:
	device_remove_file(&usbserial_device.dev, &dev_attr_nmea_enable);
fail6:
	device_remove_file(&usbserial_device.dev, &dev_attr_modem_enable);
fail5:
	platform_device_unregister(&usbserial_device);
fail4:
	platform_driver_unregister(&usbserial_driver);
fail3:
	for (j = 0 ; j == i ; j++) {
		kfree(gs_devices[j]->gs_highspeed_header);
		kfree(gs_devices[j]->gs_fullspeed_header);
		kfree(gs_devices[j]);
	}
fail2:
	kfree(gs_devices);
fail1:
	for (i = 0; i < MAX_INSTANCES; i++)
		tty_unregister_device(gs_tty_driver, i);
	tty_unregister_driver(gs_tty_driver);
	put_tty_driver(gs_tty_driver);
	return retval;
}

#if 0
/*
* gs_module_exit
*
* Unregister as a tty driver and a USB gadget driver.
*/
static void __exit gs_module_exit(void)
{
	int i;
	for (i = 0; i < instances; i++)
		usb_function_unregister(&usb_function_serial[i]);

	for (i = 0; i < instances; ++i) {
		kfree(gs_devices[i]->gs_fullspeed_header);
		kfree(gs_devices[i]->gs_highspeed_header);
		kfree(gs_devices[i]);
	}
	for (i = 0; i < MAX_INSTANCES; i++)
		tty_unregister_device(gs_tty_driver, i);
	tty_unregister_driver(gs_tty_driver);
	put_tty_driver(gs_tty_driver);
	printk(KERN_INFO "gs_module_exit: %s %s unloaded\n", GS_LONG_NAME,
	       GS_VERSION_STR);
}
#endif
/* TTY Driver */
/*
 * gs_open
 */
static int gs_open(struct tty_struct *tty, struct file *file)
{
	int port_num;
	unsigned long flags;
	struct gs_port *port;
	struct gs_dev *dev;
	struct gs_buf *buf;
	struct semaphore *sem;
	int ret;

	port_num = tty->index;

	gs_debug("gs_open: (%d,%p,%p)\n", port_num, tty, file);

	if (port_num < 0 || port_num >= GS_NUM_PORTS) {
		printk(KERN_ERR "gs_open: (%d,%p,%p) invalid port number\n",
		       port_num, tty, file);
		return -ENODEV;
	}

	dev = gs_devices[tty->index];

	if (dev == NULL) {
		printk(KERN_ERR "gs_open: (%d,%p,%p) NULL device pointer\n",
		       port_num, tty, file);
		return -ENODEV;
	}

	sem = &gs_open_close_sem[port_num];
	if (down_interruptible(sem)) {
		printk(KERN_ERR
	       "gs_open: (%d,%p,%p) interrupted waiting for semaphore\n",
		       port_num, tty, file);
		return -ERESTARTSYS;
	}

	spin_lock_irqsave(&dev->dev_lock, flags);
	port = dev->dev_port[0];

	if (port == NULL) {
		printk(KERN_ERR "gs_open: (%d,%p,%p) NULL port pointer\n",
		       port_num, tty, file);
		ret = -ENODEV;
		goto exit_unlock_dev;
	}

	spin_unlock_irqrestore(&dev->dev_lock, flags);

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev == NULL) {
		printk(KERN_ERR "gs_open: (%d,%p,%p) port disconnected (1)\n",
		       port_num, tty, file);
		ret = -EIO;
		goto exit_unlock_port;
	}

	if (port->port_open_count > 0) {
		++port->port_open_count;
		gs_debug("gs_open: (%d,%p,%p) already open\n",
			 port_num, tty, file);
		ret = 0;
		goto exit_unlock_port;
	}

	tty->driver_data = NULL;

	/* mark port as in use, we can drop port lock and sleep if necessary */
	port->port_in_use = 1;

	/* allocate write buffer on first open */
	if (port->port_write_buf == NULL) {
		spin_unlock_irqrestore(&port->port_lock, flags);
		buf = gs_buf_alloc(write_buf_size, GFP_KERNEL);
		spin_lock_irqsave(&port->port_lock, flags);

		/* might have been disconnected while asleep, check */
		if (port->port_dev == NULL) {
			printk(KERN_ERR
			       "gs_open: (%d,%p,%p) port disconnected (2)\n",
			       port_num, tty, file);
			port->port_in_use = 0;
			ret = -EIO;
			goto exit_unlock_port;
		}

		port->port_write_buf = buf;
		if (port->port_write_buf == NULL) {
			printk(KERN_ERR
	       "gs_open: (%d,%p,%p) cannot allocate port write buffer\n",
			       port_num, tty, file);
			port->port_in_use = 0;
			ret = -ENOMEM;
			goto exit_unlock_port;
		}

	}

	/* wait for carrier detect (not implemented) */

	/* might have been disconnected while asleep, check */
	if (port->port_dev == NULL) {
		printk(KERN_ERR "gs_open: (%d,%p,%p) port disconnected (3)\n",
		       port_num, tty, file);
		port->port_in_use = 0;
		ret = -EIO;
		goto exit_unlock_port;
	}

	tty->driver_data = port;
	port->port_tty = tty;
	port->port_tty->low_latency = 1;
	port->port_open_count = 1;
	port->port_in_use = 0;

	gs_debug("gs_open: (%d,%p,%p) completed\n", port_num, tty, file);
	port->msr |= MSR_CTS;
	/* Queue RX requests */
	port->n_read = 0;
	gs_start_rx(dev);
	wake_up_interruptible(&port->msr_change_wait);

	ret = 0;

exit_unlock_port:
	spin_unlock_irqrestore(&port->port_lock, flags);
	up(sem);
	return ret;

exit_unlock_dev:
	spin_unlock_irqrestore(&dev->dev_lock, flags);
	up(sem);
	return ret;

}

/*
 * gs_close
 */

#define GS_WRITE_FINISHED_EVENT_SAFELY(p)			\
({								\
	int cond;						\
								\
	spin_lock_irq(&(p)->port_lock);				\
	cond = !(p)->port_dev || !gs_buf_data_avail((p)->port_write_buf); \
	spin_unlock_irq(&(p)->port_lock);			\
	cond;							\
})

static void gs_close(struct tty_struct *tty, struct file *file)
{
	struct gs_port *port = tty->driver_data;
	struct semaphore *sem;


	if (port == NULL) {
		printk(KERN_ERR "gs_close: NULL port pointer\n");
		return;
	}

	gs_debug("gs_close: (%d,%p,%p)\n", port->port_num, tty, file);

	sem = &gs_open_close_sem[port->port_num];
	down(sem);

	spin_lock_irq(&port->port_lock);

	port->msr &= ~MSR_CTS;
	wake_up_interruptible(&port->msr_change_wait);

	if (port->port_open_count == 0) {
		printk(KERN_ERR
		       "gs_close: (%d,%p,%p) port is already closed\n",
		       port->port_num, tty, file);
		goto exit;
	}

	if (port->port_open_count > 1) {
		--port->port_open_count;
		goto exit;
	}

	/* free disconnected port on final close */
	if (port->port_dev == NULL)
		goto exit;


	/* mark port as closed but in use, we can drop port lock */
	/* and sleep if necessary */
	port->port_in_use = 1;
	port->port_open_count = 0;

	/* wait for write buffer to drain, or */
	/* at most GS_CLOSE_TIMEOUT seconds */
	if (gs_buf_data_avail(port->port_write_buf) > 0) {
		spin_unlock_irq(&port->port_lock);
		wait_event_interruptible_timeout(port->port_write_wait,
						 GS_WRITE_FINISHED_EVENT_SAFELY
						 (port), GS_CLOSE_TIMEOUT * HZ);
		spin_lock_irq(&port->port_lock);
	}

	/* free disconnected port on final close */
	/* (might have happened during the above sleep) */
	if (port->port_dev == NULL)
		goto exit;


	gs_buf_clear(port->port_write_buf);

	/* Flush bulk-out pipe */
	usb_ept_flush(port->port_dev->dev_out_ep);
	tty->driver_data = NULL;
	port->port_tty = NULL;
	port->port_in_use = 0;

	gs_debug("gs_close: (%d,%p,%p) completed\n", port->port_num, tty, file);

exit:
	spin_unlock_irq(&port->port_lock);
	up(sem);
	if (port->port_dev == NULL)
		kfree(port);
}

/*
 * gs_write
 */
static int gs_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	unsigned long flags;
	struct gs_port *port = tty->driver_data;
	int ret;

	if (port == NULL) {
		printk(KERN_ERR "gs_write: NULL port pointer\n");
		return -EIO;
	}

	gs_debug("gs_write: (%d,%p) writing %d bytes\n", port->port_num, tty,
		 count);

	if (count == 0)
		return 0;

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev == NULL) {
		printk(KERN_ERR "gs_write: (%d,%p) port is not connected\n",
		       port->port_num, tty);
		ret = -EIO;
		goto exit;
	}

	if (port->port_open_count == 0) {
		printk(KERN_ERR "gs_write: (%d,%p) port is closed\n",
		       port->port_num, tty);
		ret = -EBADF;
		goto exit;
	}

	count = gs_buf_put(port->port_write_buf, buf, count);


	if (port->port_dev->dev_config)
		gs_send(gs_devices[tty->index]);
	spin_unlock_irqrestore(&port->port_lock, flags);

	gs_debug("gs_write: (%d,%p) wrote %d bytes\n", port->port_num, tty,
		 count);

	return count;

exit:
	spin_unlock_irqrestore(&port->port_lock, flags);
	return ret;
}

/*
 * gs_put_char
 */
static int gs_put_char(struct tty_struct *tty, unsigned char ch)
{
	unsigned long flags;
	struct gs_port *port = tty->driver_data;
	int status = 0;

	if (port == NULL) {
		printk(KERN_ERR "gs_put_char: NULL port pointer\n");
		return 0;
	}

	gs_debug("gs_put_char: (%d,%p) char=0x%x, called from %p, %p, %p\n",
		 port->port_num, tty, ch, __builtin_return_address(0),
		 __builtin_return_address(1), __builtin_return_address(2));

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev == NULL) {
		printk(KERN_ERR "gs_put_char: (%d,%p) port is not connected\n",
		       port->port_num, tty);
		goto exit;
	}

	if (port->port_open_count == 0) {
		printk(KERN_ERR "gs_put_char: (%d,%p) port is closed\n",
		       port->port_num, tty);
		goto exit;
	}

	status = gs_buf_put(port->port_write_buf, &ch, 1);

exit:
	spin_unlock_irqrestore(&port->port_lock, flags);
	return status;
}

/*
 * gs_flush_chars
 */
static void gs_flush_chars(struct tty_struct *tty)
{
	unsigned long flags;
	struct gs_port *port = tty->driver_data;

	if (port == NULL) {
		printk(KERN_ERR "gs_flush_chars: NULL port pointer\n");
		return;
	}

	gs_debug("gs_flush_chars: (%d,%p)\n", port->port_num, tty);

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev == NULL) {
		printk(KERN_ERR
		       "gs_flush_chars: (%d,%p) port is not connected\n",
		       port->port_num, tty);
		goto exit;
	}

	if (port->port_open_count == 0) {
		printk(KERN_ERR "gs_flush_chars: (%d,%p) port is closed\n",
		       port->port_num, tty);
		goto exit;
	}

	if (port->port_dev->dev_config)
		gs_send(gs_devices[tty->index]);
	spin_unlock_irqrestore(&port->port_lock, flags);


	return;

exit:
	spin_unlock_irqrestore(&port->port_lock, flags);
}

/*
 * gs_write_room
 */
static int gs_write_room(struct tty_struct *tty)
{

	int room = 0;
	unsigned long flags;
	struct gs_port *port = tty->driver_data;

	if (port == NULL)
		return 0;

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev != NULL && port->port_open_count > 0
	    && port->port_write_buf != NULL)
		room = gs_buf_space_avail(port->port_write_buf);

	spin_unlock_irqrestore(&port->port_lock, flags);

	gs_debug("gs_write_room: (%d,%p) room=%d\n", port->port_num, tty, room);

	return room;
}

/*
 * gs_chars_in_buffer
 */
static int gs_chars_in_buffer(struct tty_struct *tty)
{
	int chars = 0;
	unsigned long flags;
	struct gs_port *port = tty->driver_data;

	if (port == NULL)
		return 0;

	spin_lock_irqsave(&port->port_lock, flags);

	if (port->port_dev != NULL && port->port_open_count > 0
	    && port->port_write_buf != NULL)
		chars = gs_buf_data_avail(port->port_write_buf);

	spin_unlock_irqrestore(&port->port_lock, flags);

	gs_debug("gs_chars_in_buffer: (%d,%p) chars=%d\n",
		 port->port_num, tty, chars);

	return chars;
}

/*
 * gs_throttle
 */
static void gs_throttle(struct tty_struct *tty)
{
}

/*
 * gs_unthrottle
 */
static void gs_unthrottle(struct tty_struct *tty)
{
	struct gs_port		*port = tty->driver_data;
	unsigned long		flags;

	spin_lock_irqsave(&port->port_lock, flags);
	queue_work(gs_tty_wq, &port->push_work);
	spin_unlock_irqrestore(&port->port_lock, flags);
}

/*
 * gs_break
 */
static int gs_break(struct tty_struct *tty, int break_state)
{
	return 0;
}

/*
 * gs_ioctl
 */
static int gs_ioctl(struct tty_struct *tty, struct file *file,
		    unsigned int cmd, unsigned long arg)
{
	struct gs_port *port = tty->driver_data;
	DECLARE_WAITQUEUE(wait, current);
	struct async_icount cnow;
	struct async_icount cprev;

	if (port == NULL) {
		printk(KERN_ERR "gs_ioctl: NULL port pointer\n");
		return -EIO;
	}

	gs_debug("gs_ioctl: (%d,%p,%p) cmd=0x%4.4x, arg=%lu\n",
		 port->port_num, tty, file, cmd, arg);

	/* handle ioctls */
	switch (cmd) {
	case TIOCMIWAIT:
		cprev = gs_current_icount(port);
		while (1) {
			add_wait_queue(&port->msr_change_wait, &wait);
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
			remove_wait_queue(&port->msr_change_wait, &wait);
			if (signal_pending(current))
				return -ERESTARTSYS;
			cnow = gs_current_icount(port);
			if (cnow.rng == cprev.rng &&
				cnow.dsr == cprev.dsr &&
				cnow.dcd == cprev.dcd &&
				cnow.cts == cprev.cts)
				return -EIO;
			if (((arg & TIOCM_RI) &&
					(cnow.rng != cprev.rng)) ||
				((arg & TIOCM_DSR) &&
					(cnow.dsr != cprev.dsr)) ||
				((arg & TIOCM_CD)  &&
					(cnow.dcd != cprev.dcd)) ||
				((arg & TIOCM_CTS) &&
					(cnow.cts != cprev.cts)))
				return 0;
			cprev = cnow;
		}
		break;
	}

	/* could not handle ioctl */
	return -ENOIOCTLCMD;
}

/*
 * gs_set_termios
 */
static void gs_set_termios(struct tty_struct *tty, struct ktermios *old)
{
}

/*
* gs_send
*
* This function finds available write requests, calls
* gs_send_packet to fill these packets with data, and
* continues until either there are no more write requests
* available or no more data to send.  This function is
* run whenever data arrives or write requests are available.
*/
static int gs_send(struct gs_dev *dev)
{
	struct gs_port *port = dev->dev_port[0];
	struct list_head *pool = &port->write_pool;
	int status = 0;
	bool do_tty_wake = false;
	struct usb_endpoint *ep = dev->dev_in_ep;

	while (!list_empty(pool)) {
		struct usb_request *req;
		int len;
		req = list_entry(pool->next, struct usb_request, list);
		len = gs_send_packet(dev, req->buf, TRANSFER_SIZE);
		if (len == 0) {
			wake_up_interruptible(&port->port_write_wait);
			break;
		}
		do_tty_wake = true;

		req->length = len;
		list_del(&req->list);

		/* Drop lock while we call out of driver; completions
		 * could be issued while we do so.  Disconnection may
		 * happen too; maybe immediately before we queue this!
		 * NOTE that we may keep sending data for a while after
		 * the TTY closed (dev->ioport->port_tty is NULL).
		 */
		spin_unlock(&port->port_lock);
		status = usb_ept_queue_xfer(ep, req);
		spin_lock(&port->port_lock);

		if (status) {
			printk(KERN_ERR "%s: %s err %d\n",
					__func__, "queue", status);
			list_add(&req->list, pool);
			break;
		}

	}

	if (do_tty_wake && port->port_tty)
		tty_wakeup(port->port_tty);
	return status;

}

/*
 * gs_send_packet
 *
 * If there is data to send, a packet is built in the given
 * buffer and the size is returned.  If there is no data to
 * send, 0 is returned.  If there is any error a negative
 * error number is returned.
 *
 * Called during USB completion routine, on interrupt time.
 *
 * We assume that disconnect will not happen until all completion
 * routines have completed, so we can assume that the dev_port
 * array does not change during the lifetime of this function.
 */
static int gs_send_packet(struct gs_dev *dev, char *packet, unsigned int size)
{
	unsigned int len;
	struct gs_port *port;

	if (dev == NULL) {
		printk(KERN_ERR "gs_recv_packet:NULL device pointer\n");
		return -EIO;
	}

	/* TEMPORARY -- only port 0 is supported right now */
	port = dev->dev_port[0];
	if (port == NULL) {
		printk(KERN_ERR
		       "gs_send_packet: port=%d, NULL port pointer\n", 0);
		return -EIO;
	}


	len = gs_buf_data_avail(port->port_write_buf);
	if (len < size)
		size = len;
	if (size != 0)
		size = gs_buf_get(port->port_write_buf, packet, size);



	if (port->port_tty)
		tty_wakeup(port->port_tty);

	return size;
}

static void gs_rx_push(struct work_struct *work)
{
	struct gs_port *port = container_of(work,
					struct gs_port,
					push_work);
	struct tty_struct *tty;
	struct list_head *queue = &port->read_queue;
	bool do_push = false;
	struct gs_dev *dev = port->port_dev;

	/* hand any queued data to the tty */
	spin_lock_irq(&port->port_lock);
	tty = port->port_tty;
	while (!list_empty(queue)) {
		struct usb_request	*req;

		req = list_first_entry(queue, struct usb_request, list);

		/* discard data if tty was closed */
		if (!tty)
			goto recycle;

		if (req->actual) {
			char		*packet = req->buf;
			unsigned	size = req->actual;
			unsigned	n;
			int		count;
			/* we may have pushed part of this packet already... */
			n = port->n_read;
			if (n) {
				packet += n;
				size -= n;
			}
			/*printk(KERN_INFO "tty_push:%d\n",size);*/
			count = tty_insert_flip_string(tty, packet, size);
			/*
			if (count == 0)
				printk(KERN_INFO "%s: tty buffer is full: throttle\n",
							__func__);
			*/
			if (count)
				do_push = true;
			if (count != size) {
				/* stop pushing; TTY layer can't handle more */
				port->n_read += count;
				break;
			}
			port->n_read = 0;
		}
recycle:
		list_move(&req->list, &port->read_pool);
	}
	if (tty && do_push) {
		spin_unlock_irq(&port->port_lock);
		tty_flip_buffer_push(tty);
		wake_up_interruptible(&tty->read_wait);
		spin_lock_irq(&port->port_lock);
		/* tty may have been closed */
		tty = port->port_tty;
	}
	if (!list_empty(queue) && tty) {
		if (!test_bit(TTY_THROTTLED, &tty->flags)) {
			if (do_push)
				queue_work(gs_tty_wq, &port->push_work);
		}
	}
	gs_start_rx(dev);
	spin_unlock_irq(&port->port_lock);
}

/*
* gs_read_complete
*/
static void gs_read_complete(struct usb_endpoint *ep, struct usb_request *req)
{
	/* used global variable */
	struct gs_dev *dev = (struct gs_dev *)req->device;
	struct gs_port *port;
	struct tty_struct *tty;

	if (dev == NULL) {
		printk(KERN_ERR "gs_read_complete: NULL device pointer\n");
		return;
	}

	port = dev->dev_port[0];
	tty = port->port_tty;
	if (tty == NULL)
		return;
	switch (req->status) {
	case 0:
		spin_lock(&port->port_lock);
		list_add_tail(&req->list, &port->read_queue);
		if (!test_bit(TTY_THROTTLED, &tty->flags))
			queue_work(gs_tty_wq, &port->push_work);
		spin_unlock(&port->port_lock);
		break;

	case -ESHUTDOWN:
		/* disconnect */
		gs_free_req(ep, req);
		break;

	case -ENODEV:
		list_add_tail(&req->list, &port->read_pool);
		/* Implemented handling in future if needed */
		break;
	default:
		list_add_tail(&req->list, &port->read_pool);
		printk(KERN_ERR
		"gs_read_complete: unexpected status error, status=%d\n",
			req->status);
		/* goto requeue; */
		break;
	}
}

/*
* gs_write_complete
*/
static void gs_write_complete(struct usb_endpoint *ep, struct usb_request *req)
{
	struct gs_dev *dev = (struct gs_dev *)req->device;
	struct gs_port	*port;

	if (dev == NULL) {
		printk(KERN_ERR "gs_write_complete: NULL device pointer\n");
		return;
	} else
		port = dev->dev_port[0];

	spin_lock(&port->port_lock);
	list_add(&req->list, &port->write_pool);

	switch (req->status) {
	default:
		/* presumably a transient fault */
		printk(KERN_ERR "%s: unexpected status %d\n",
				__func__, req->status);
		/* FALL THROUGH */
	case 0:
		/* normal completion */

		if (dev->dev_config)
			gs_send(dev);

		break;

	case -ESHUTDOWN:
		/* disconnect */
		printk(KERN_DEBUG "%s: shutdown\n", __func__);
		break;
	}
	spin_unlock(&port->port_lock);
}

/* Send Notification to host if Status changes */
static int send_notify_data(struct usb_endpoint *ep, struct usb_request *req)
{
	struct gs_dev *dev = (struct gs_dev *)req->device;
	struct usb_cdc_notification *notify = req->buf;
	struct gs_port *port;
	unsigned int msr, ret;
	__le16 *data;

	if (dev == NULL) {
		printk(KERN_ERR "send_notify_data: NULL device pointer\n");
		return 0;
	}

	port = dev->dev_port[0];

	if (port == NULL) {
		printk(KERN_ERR"send_notify_data:port is NULL\n");
		return 0;
	}

	msr = port->msr;
	req->length = 10;
	notify->bmRequestType  = 0xA1;
	notify->bNotificationType  = USB_CDC_NOTIFY_SERIAL_STATE;
	notify->wValue  = __constant_cpu_to_le16(0);
	notify->wIndex  = __constant_cpu_to_le16(dev->interface_num);
	notify->wLength  = __constant_cpu_to_le16(2);
	data = req->buf + sizeof *notify;
	data[0] = __constant_cpu_to_le16(((msr & MSR_CD) ? 1 : 0)
			| ((msr & MSR_DSR) ? (1<<1) : (0<<1))
			| ((msr & MSR_RI) ? (1<<3) : (0<<3)));

	ret = usb_ept_queue_xfer(ep, req);
	if (ret) {
		printk(KERN_ERR
		"send_notify_data: cannot queue status request,ret = %d\n",
			       ret);
		return 0;
	}
	return 1;
}

/* Free request if -ESHUTDOWN */
static void gs_status_complete(struct usb_endpoint *ep,
				struct usb_request *req)
{
	struct gs_dev *dev = (struct gs_dev *)req->device;
	struct gs_port *port;

	if (dev == NULL) {
		printk(KERN_ERR"gs_status_complete : NULL device pointer\n");
		return;
	}

	port = dev->dev_port[0];

	if (port == NULL) {
		printk(KERN_ERR "gs_status_complete: NULL port pointer\n");
		return;
	}

	switch (req->status) {
	case 0:

		gs_debug("%s:port->msr=%x,dev=%p,ep=%p,req=%p", __func__,
			port->msr, dev, dev->dev_notify_ep, dev->notify_req);
		/* executed only if data missed because of
		** request already in queue and user modifies using tiocmset */
		if (port->prev_msr != port->msr) {
			send_notify_data(dev->dev_notify_ep, dev->notify_req);
			port->prev_msr = port->msr;
		}
		break;

	case -ESHUTDOWN:
		/* disconnect */
		gs_debug("gs_status_complete: shutdown\n");
		gs_free_req(ep, req);
		break;

	default:
		printk(KERN_ERR
	       "gs_status_complete: unexpected status error, status=%d\n",
		       req->status);
		break;
	}
}

/* Function Driver */
/*
 * gs_bind
 *
 * Called on module load.  Allocates and initializes the device
 * structure and a control request.
 */
static void gs_bind(struct usb_endpoint **ept, void *_ctxt)
{
	struct gs_dev *dev = _ctxt;
	struct usb_function *func = dev->func;
	int i = 0;
	int ret;

	if (func == NULL) {
		pr_err("%s: NULL function pointer\n", __func__);
		return;
	} else
		printk("gs_bind:%s\n", func->ifc_name);

	ret = gs_alloc_ports(dev, GFP_KERNEL);
	if (ret != 0) {
		pr_err("%s: cannot allocate ports\n", __func__);
		gs_unbind(_ctxt);
		return;
	}

	dev->dev_in_ep = ept[2];
	dev->dev_out_ep = ept[1];
	dev->dev_notify_ep = ept[0];

	for (i = 0; i < GS_DEFAULT_INT_REQ; ++i) {
		struct gs_reqbuf *bh = &dev->statusreqbuf;
		bh->buf = kmalloc(64, GFP_KERNEL);
		if (!bh->buf)
			return;
	}

	dev->bound = 1;
	return;
}

/*
 * gs_unbind
 *
 * Called on module unload.  Frees the control request and device
 * structure.
 */
static void /* __init_or_exit */ gs_unbind(void *_ctxt)
{
	struct gs_dev *dev = _ctxt;

	if (!dev) {
		pr_err("%s: error: null device\n", __func__);
		return;
	}
	if (!dev->bound)
		return;

	kfree(dev->statusreqbuf.buf);
	gs_free_ports(dev);
	dev->bound = 0;
	pr_debug("%s: %s %s\n", __func__, GS_LONG_NAME, GS_VERSION_STR);
}

static int gs_setup(struct usb_ctrlrequest *ctrl,
		void *buf, int len, void *_ctxt)
{
	int ret = -EOPNOTSUPP;
	struct gs_dev *dev = _ctxt;
	struct gs_port *port;/* ACM only has one port */
	u16 wIndex = le16_to_cpu(ctrl->wIndex);
	u16 wValue = le16_to_cpu(ctrl->wValue);
	u16 wLength = le16_to_cpu(ctrl->wLength);

	if (dev == NULL) {
		printk(KERN_ERR"gs_setup:device pointer NULL\n");
		return 0;
	}
	port = dev->dev_port[0];

	if (port == NULL) {
		printk(KERN_ERR"gs_setup: port pointer is NULL\n");
		return 0;
	}
	switch (ctrl->bRequest) {
#if 0
	case USB_CDC_REQ_SET_LINE_CODING:
		ret = min(wLength,
		(u16)sizeof(struct usb_cdc_line_coding));
		if (port) {
			spin_lock(&port->port_lock);
			memcpy(&port->port_line_coding, buf , ret);
			spin_unlock(&port->port_lock);
		}
	break;
#endif

	case USB_CDC_REQ_GET_LINE_CODING:
		port = dev->dev_port[0];/* ACM only has one port */
		ret = min(wLength, (u16) sizeof(struct usb_cdc_line_coding));
		if (port) {
			spin_lock(&port->port_lock);
			memcpy(buf, &port->port_line_coding, ret);
			spin_unlock(&port->port_lock);
		}
		break;
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
		port = dev->dev_port[0];/* ACM only has one port */
		if (dev->interface_num != wIndex)
			break;
		if (wValue & USB_CDC_SET_CONTROL_LINE_STATE_DTR) {
			port->mcr |= MCR_DTR;
			port->msr |= MSR_DSR;
			wake_up_interruptible(&port->msr_change_wait);
		} else	{
			port->mcr &= ~MCR_DTR;
			/* port->msr &= ~MSR_DSR; */
			wake_up_interruptible(&port->msr_change_wait);
		}
		if (wValue & USB_CDC_SET_CONTROL_LINE_STATE_RTS)
			port->mcr |= MCR_RTS;
		else
			port->mcr &= ~MCR_RTS;

		gs_debug("%s: prev_msr = 0x%x, msr = 0x%x\n",
			__func__, port->prev_msr, port->msr);
		if (port->prev_msr != port->msr) {
			if (send_notify_data(dev->dev_notify_ep, dev->notify_req))
				port->prev_msr = port->msr;
		}

		ret = 0;
		break;

	default:
		break;
	}

	return ret;
}

#if 0
static void gs_disconnect(void *_ctxt)
{
	struct gs_dev *dev = _ctxt;
	struct gs_port *port = dev->dev_port[0];
	unsigned long flags;

	/* tell the TTY glue not to do I/O here any more */
	spin_lock_irqsave(&port->port_lock, flags);
	dev->dev_config = 0;
	if (port->port_open_count > 0 || port->port_in_use) {
		wake_up_interruptible(&port->port_write_wait);
		if (port->port_tty) {
			wake_up_interruptible(&port->port_tty->read_wait);
			wake_up_interruptible(&port->port_tty->write_wait);
			tty_hangup(port->port_tty);
		}
	}
	port->mcr &= ~(MCR_DTR | MCR_RTS);
	port->msr &= ~(MSR_DSR | MSR_CD | MSR_RI);
	spin_unlock_irqrestore(&port->port_lock, flags);

}
#endif
/*
 * gs_configure
 *
 * Configures the device by enabling device specific
 * optimizations, setting up the endpoints, allocating
 * read and write requests and queuing read requests.
 *
 * The device lock must be held when calling this function.
 */
static void gs_configure(int config, void *_ctxt)
{
	int i, ret = 0;
	unsigned MaxPacketSize;
	struct gs_dev *dev = _ctxt;
	struct usb_endpoint *ep;
	struct usb_request *req;
	struct gs_port *port;
	struct list_head *rhead;
	struct list_head *whead;
	unsigned started = 0;

	if (dev == NULL) {
		printk(KERN_ERR "gs_configure: NULL device pointer\n");
		return;
	}
	if (!dev->bound)
		return;

	port = dev->dev_port[0];
	rhead = &port->read_pool;
	whead = &port->write_pool;
	if (port == NULL) {
		printk(KERN_ERR "gs_configure:port is NULL\n");
		return;
	}

	if (!config) {
		gs_debug("gs_configure: Deconfigure\n");
		dev->configured = SERIAL_UNCONFIGURED;
		port->prev_msr = 0;
		gs_reset_config(dev);
		return;
	}
	dev->dev_config = config;

	port->msr |= MSR_DSR;
	wake_up_interruptible(&port->msr_change_wait);

	if (dev->dev_in_ep == NULL || dev->dev_out_ep == NULL ||
	    (dev->dev_notify_ep == NULL)) {
		printk(KERN_ERR "gs_configure : cannot find endpoints\n");
		ret = -ENODEV;
		goto reset_config;
	}

	gs_debug("gs_configure: endpoint sizes and buffers\n");
	/* allocate and queue read requests */
	ep = dev->dev_out_ep;
	MaxPacketSize = usb_ept_get_max_packet(ep);
	for (i = 0; i < read_q_size; i++) {
		req = gs_alloc_req(ep, TRANSFER_SIZE);
		if (req) {
			req->device = (void *)dev;
			req->length = TRANSFER_SIZE;
			req->complete = gs_read_complete;
			list_add_tail(&req->list, rhead);
			gs_debug("gs_configure: queuing read request(%d)\n", i);
		} else {
			printk(KERN_ERR
			"gs_configure: cannot allocate read request(%d)\n", i);
			goto reset_config;
		}
	}

	/* allocate write requests, and put on free list */
	ep = dev->dev_in_ep;
	MaxPacketSize = usb_ept_get_max_packet(ep);
	for (i = 0; i < write_q_size; i++) {
		req = gs_alloc_req(ep, TRANSFER_SIZE);
		if (req) {
			req->device = (void *)dev;
			req->length = TRANSFER_SIZE;
			req->complete = gs_write_complete;
			list_add_tail(&req->list, whead);
		} else {
			printk(KERN_ERR
			"gs_configure: cannot allocate write request(%d)\n", i);
			goto reset_config;
		}
	}

	ep = dev->dev_notify_ep;
	MaxPacketSize = usb_ept_get_max_packet(ep);
	for (i = 0; i < GS_DEFAULT_INT_REQ; ++i) {
		struct gs_reqbuf *bh = &dev->statusreqbuf;
		dev->notify_req = req = gs_alloc_req(ep, 0);
		if (req) {
			req->device = (void *)dev;
			req->buf = bh->buf;
			req->length = MaxPacketSize;
			req->complete = gs_status_complete;
		}
	}

	if (port->port_open_count) {
		unsigned long flags;
		spin_lock_irqsave(&port->port_lock, flags);
		started = gs_start_rx(dev);
		spin_unlock_irqrestore(&port->port_lock, flags);
		if (started)
			tty_wakeup(port->port_tty);
	}

	dev->configured = SERIAL_CONFIGURED;
	dev->interface_num = usb_find_interface_number(dev->func->ifc_name);
	gs_debug("func(%s) intr=%d\n", dev->func->ifc_name,
		dev->interface_num);
	printk(KERN_DEBUG "gs_configure done\n");
	return;

reset_config:
	printk(KERN_ERR "gs_configure(end): error, calling gs_reset_config\n");
	gs_reset_config(dev);
	return;
}

static unsigned gs_start_rx(struct gs_dev *dev)
{
	struct gs_port *port = dev->dev_port[0];
	struct list_head *pool = &port->read_pool;
	unsigned ret = 0;
	struct usb_endpoint *ep = dev->dev_out_ep;
	unsigned started = 0;

	while (!list_empty(pool)) {
		struct usb_request	*req;
		struct tty_struct	*tty;
		tty = port->port_tty;
		if (!tty) {
			printk(KERN_ERR "%s: tty is null\n", __func__);
			break;
		}

		req = list_entry(pool->next, struct usb_request, list);
		list_del(&req->list);
		spin_unlock(&port->port_lock);
		ret = usb_ept_queue_xfer(ep, req);
		spin_lock(&port->port_lock);
		if (ret) {
			list_add(&req->list, pool);
			break;
		}
		started++;

	}
	return started;
}
/*
 * gs_reset_config
 *
 * Mark the device as not configured, disable all endpoints,
 * which forces completion of pending I/O and frees queued
 * requests, and free the remaining write requests on the
 * free list.
 *
 * The device lock must be held when calling this function.
 */
static void gs_reset_config(struct gs_dev *dev)
{
	struct gs_port *port;
	struct usb_request *req;
	unsigned long flags;

	if (dev == NULL) {
		printk(KERN_ERR "gs_reset_config: NULL device pointer\n");
		return;
	}

	port = dev->dev_port[0];


#if 0 /*-- workaround -- Remove them, because they frees same requests in read_pool/write_pool*/
	if (dev->dev_out_ep)
		usb_free_endpoint_all_req(dev->dev_out_ep);
	if (dev->dev_in_ep)
		usb_free_endpoint_all_req(dev->dev_in_ep);
#endif
	if (dev->dev_notify_ep)
		usb_free_endpoint_all_req(dev->dev_notify_ep);


	spin_lock_irqsave(&port->port_lock, flags);
	dev->dev_config = GS_NO_CONFIG_ID;
	/* free write requests on the free list */
	while (!list_empty(&port->write_pool)) {
		req = list_entry(port->write_pool.next,
				       struct usb_request, list);
		list_del(&req->list);
		gs_free_req(dev->dev_in_ep, req);
	}

	/* free read requests from read pool */
	while (!list_empty(&port->read_pool)) {
		req = list_entry(port->read_pool.next,
				       struct usb_request, list);
		list_del(&req->list);
		gs_free_req(dev->dev_out_ep, req);
	}

	/* free read requests from read queue */
	while (!list_empty(&port->read_queue)) {
		req = list_entry(port->read_queue.next,
				       struct usb_request, list);
		list_del(&req->list);
		gs_free_req(dev->dev_out_ep, req);
	}
	spin_unlock_irqrestore(&port->port_lock, flags);
}

/*
 * gs_alloc_req
 *
 * Allocate a usb_request and its buffer.  Returns a pointer to the
 * usb_request or NULL if there is an error.
 */
static struct usb_request *gs_alloc_req(struct usb_endpoint *ep,
					unsigned int len)
{
	struct usb_request *req;
	if (ep == NULL)
		return NULL;
	req = usb_ept_alloc_req(ep, len);
	return req;
}

/*
 * gs_free_req
 *
 * Free a usb_request and its buffer.
 */
static void gs_free_req(struct usb_endpoint *ep, struct usb_request *req)
{
	if (ep != NULL && req != NULL)
		usb_ept_free_req(ep, req);
}



/*
 * gs_alloc_ports
 *
 * Allocate all ports and set the gs_dev struct to point to them.
 * Return 0 if successful, or a negative error number.
 *
 * The device lock is normally held when calling this function.
 */
static int gs_alloc_ports(struct gs_dev *dev, gfp_t kmalloc_flags)
{
	int i;
	struct gs_port *port;

	if (dev == NULL)
		return -EIO;

	for (i = 0; i < GS_NUM_PORTS; i++) {
		port = kzalloc(sizeof(struct gs_port), kmalloc_flags);
		if (port == NULL)
			return -ENOMEM;

		INIT_WORK(&port->push_work, gs_rx_push);
		INIT_LIST_HEAD(&port->read_pool);
		INIT_LIST_HEAD(&port->read_queue);
		INIT_LIST_HEAD(&port->write_pool);
		port->msr = 0;
		port->prev_msr = 0;
		port->mcr = 0;
		init_waitqueue_head(&port->msr_change_wait);
		port->port_dev = dev;
		port->port_num = i;
		port->port_line_coding.dwDTERate =
		    cpu_to_le32(GS_DEFAULT_DTE_RATE);
		port->port_line_coding.bCharFormat = GS_DEFAULT_CHAR_FORMAT;
		port->port_line_coding.bParityType = GS_DEFAULT_PARITY;
		port->port_line_coding.bDataBits = GS_DEFAULT_DATA_BITS;
		spin_lock_init(&port->port_lock);
		mutex_init(&port->mutex_lock);
		init_waitqueue_head(&port->port_write_wait);

		dev->dev_port[i] = port;
	}

	return 0;
}

/*
 * gs_free_ports
 *
 * Free all closed ports.  Open ports are disconnected by
 * freeing their write buffers, setting their device pointers
 * and the pointers to them in the device to NULL.  These
 * ports will be freed when closed.
 *
 * The device lock is normally held when calling this function.
 */
static void gs_free_ports(struct gs_dev *dev)
{
	int i;
	unsigned long flags;
	struct gs_port *port;

	if (dev == NULL)
		return;

	for (i = 0; i < GS_NUM_PORTS; i++) {
		port = dev->dev_port[i];
		if (port != NULL) {
			dev->dev_port[i] = NULL;
			spin_lock_irqsave(&port->port_lock, flags);

			if (port->port_write_buf != NULL) {
				gs_buf_free(port->port_write_buf);
				port->port_write_buf = NULL;
			}

			if (port->port_open_count > 0 || port->port_in_use) {
				port->port_dev = NULL;
				wake_up_interruptible(&port->port_write_wait);
				if (port->port_tty) {
					wake_up_interruptible
					    (&port->port_tty->read_wait);
					wake_up_interruptible
					    (&port->port_tty->write_wait);
				}
				spin_unlock_irqrestore(&port->port_lock, flags);
			} else {
				spin_unlock_irqrestore(&port->port_lock, flags);
				kfree(port);
			}

		}
	}
}

/* Circular Buffer */

/*
 * gs_buf_alloc
 *
 * Allocate a circular buffer and all associated memory.
 */
static struct gs_buf *gs_buf_alloc(unsigned int size, gfp_t kmalloc_flags)
{
	struct gs_buf *gb;

	if (size == 0)
		return NULL;

	gb = kmalloc(sizeof(struct gs_buf), kmalloc_flags);
	if (gb == NULL)
		return NULL;

	gb->buf_buf = kmalloc(size, kmalloc_flags);
	if (gb->buf_buf == NULL) {
		kfree(gb);
		return NULL;
	}

	gb->buf_size = size;
	gb->buf_get = gb->buf_put = gb->buf_buf;

	return gb;
}

/*
 * gs_buf_free
 *
 * Free the buffer and all associated memory.
 */
void gs_buf_free(struct gs_buf *gb)
{
	if (gb) {
		kfree(gb->buf_buf);
		kfree(gb);
	}
}

/*
 * gs_buf_clear
 *
 * Clear out all data in the circular buffer.
 */
void gs_buf_clear(struct gs_buf *gb)
{
	if (gb != NULL)
		gb->buf_get = gb->buf_put;
	/* equivalent to a get of all data available */
}

/*
 * gs_buf_data_avail
 *
 * Return the number of bytes of data available in the circular
 * buffer.
 */
unsigned int gs_buf_data_avail(struct gs_buf *gb)
{
	if (gb != NULL)
		return (gb->buf_size + gb->buf_put - gb->buf_get)
		    % gb->buf_size;
	else
		return 0;
}

/*
 * gs_buf_space_avail
 *
 * Return the number of bytes of space available in the circular
 * buffer.
 */
unsigned int gs_buf_space_avail(struct gs_buf *gb)
{
	if (gb != NULL)
		return (gb->buf_size + gb->buf_get - gb->buf_put - 1)
		    % gb->buf_size;
	else
		return 0;
}

/*
 * gs_buf_put
 *
 * Copy data data from a user buffer and put it into the circular buffer.
 * Restrict to the amount of space available.
 *
 * Return the number of bytes copied.
 */
unsigned int gs_buf_put(struct gs_buf *gb, const char *buf, unsigned int count)
{
	unsigned int len;

	if (gb == NULL)
		return 0;

	len = gs_buf_space_avail(gb);
	if (count > len)
		count = len;

	if (count == 0)
		return 0;

	len = gb->buf_buf + gb->buf_size - gb->buf_put;
	if (count > len) {
		memcpy(gb->buf_put, buf, len);
		memcpy(gb->buf_buf, buf + len, count - len);
		gb->buf_put = gb->buf_buf + count - len;
	} else {
		memcpy(gb->buf_put, buf, count);
		if (count < len)
			gb->buf_put += count;
		else		/* count == len */
			gb->buf_put = gb->buf_buf;
	}

	return count;
}

/*
 * gs_buf_get
 *
 * Get data from the circular buffer and copy to the given buffer.
 * Restrict to the amount of data available.
 *
 * Return the number of bytes copied.
 */
unsigned int gs_buf_get(struct gs_buf *gb, char *buf, unsigned int count)
{
	unsigned int len;

	if (gb == NULL)
		return 0;

	len = gs_buf_data_avail(gb);
	if (count > len)
		count = len;

	if (count == 0)
		return 0;

	len = gb->buf_buf + gb->buf_size - gb->buf_get;
	if (count > len) {
		memcpy(buf, gb->buf_get, len);
		memcpy(buf + len, gb->buf_buf, count - len);
		gb->buf_get = gb->buf_buf + count - len;
	} else {
		memcpy(buf, gb->buf_get, count);
		if (count < len)
			gb->buf_get += count;
		else		/* count == len */
			gb->buf_get = gb->buf_buf;
	}

	return count;
}

/*
* gs_tiocmget
*/
static int gs_tiocmget(struct tty_struct *tty, struct file *file)
{
	struct gs_port *port;
	unsigned int mcr, msr;
	unsigned int result = 0;
	struct gs_dev *dev = gs_devices[tty->index];

	if (dev == NULL)
		return -EIO;

	port = dev->dev_port[0];
	if (port == NULL)
		return -EIO;

	mutex_lock(&port->mutex_lock);
	mcr = port->mcr;
	msr = port->msr;

	result = ((mcr & MCR_RTS) ? TIOCM_RTS : 0)
		| ((mcr & MCR_DTR) ? TIOCM_DTR : 0)
		| ((mcr & MCR_LOOP) ? TIOCM_LOOP : 0)
		| ((msr & MSR_CD) ? TIOCM_CD : 0)
		| ((msr & MSR_RI) ? TIOCM_RI : 0)
		| ((msr & MSR_DSR) ? TIOCM_DSR : 0)
		| ((msr & MSR_CTS) ? TIOCM_CTS : 0);

	mutex_unlock(&port->mutex_lock);
	return result;
}

/*
* gs_tiocmset
*/
static int gs_tiocmset(struct tty_struct *tty, struct file *file,
	unsigned int set, unsigned int clear)
{
	struct gs_port *port;
	unsigned int mcr;
	unsigned int msr;
	struct gs_dev *dev = gs_devices[tty->index];
	gs_debug("%s: (tty%d) set 0x%x, clear 0x%x\n", __func__,
		tty->index, set, clear);

	if (dev == NULL)
		return -EIO;
	port = dev->dev_port[0];

	if (port == NULL)
		return -EIO;

	mcr = port->mcr;
	msr = port->msr;
	if (dev->configured != SERIAL_CONFIGURED)
		return -EIO;

	if (set & TIOCM_DTR)
		mcr |= MCR_DTR;
	if (set & TIOCM_RTS)
		mcr |= MCR_RTS;
	if (set & TIOCM_LOOP)
		mcr |= MCR_LOOP;
	if (set & TIOCM_RI)
		msr |= MSR_RI;
	if (set & TIOCM_CD)
		msr |= MSR_CD;
	if (set & TIOCM_OUT1)
		msr |= MSR_CD;

	if (clear & TIOCM_DTR)
		mcr &= ~MCR_DTR;
	if (clear & TIOCM_RTS)
		mcr &= ~MCR_RTS;
	if (clear & TIOCM_LOOP)
		mcr &= ~MCR_LOOP;
	if (clear & TIOCM_RI)
		msr &= ~MSR_RI;
	if (clear & TIOCM_CD)
		msr &= ~MSR_CD;
	if (clear & TIOCM_OUT1)
		msr &= ~MSR_CD;

	mutex_lock(&port->mutex_lock);
	port->mcr = mcr;
	port->msr = msr;

	if (port->prev_msr != port->msr) {
		if (send_notify_data(dev->dev_notify_ep, dev->notify_req))
			port->prev_msr = port->msr;
	}
	mutex_unlock(&port->mutex_lock);
	return 0;
}

static struct async_icount gs_current_icount(struct gs_port *port)
{
	struct async_icount icount;

	if (port->msr & MSR_RI)
		icount.rng = TIOCM_RI;
	else
		icount.rng = 0;

	if (port->msr & MSR_DSR)
		icount.dsr = TIOCM_DSR;
	else
		icount.dsr = 0;

	if (port->msr & MSR_CD)
		icount.dcd = TIOCM_CD;
	else
		icount.dcd = 0;

	if (port->msr & MSR_CTS)
		icount.cts = TIOCM_CTS;
	else
		icount.cts = 0;

	return icount;
}

static int serial_set_modem_enabled(const char *val, struct kernel_param *kp)
{
	int enabled = simple_strtol(val, NULL, 0);
	usb_function_serial[MODEM_INSTANCE].disabled = !enabled;

	if (enabled) {
		gs_tty_driver->init_termios = tty_std_termios;
		gs_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL
		    | CLOCAL;
	}

	usb_function_enable("modem", enabled);
	return 0;
}

static int serial_get_modem_enabled(char *buffer, struct kernel_param *kp)
{
	buffer[0] = '0' + !usb_function_serial[MODEM_INSTANCE].disabled;
	return 1;
}

module_param_call(modem_enabled, serial_set_modem_enabled, serial_get_modem_enabled, 0, 0664);

static int serial_set_nmea_enabled(const char *val, struct kernel_param *kp)
{
	int enabled = simple_strtol(val, NULL, 0);
	usb_function_serial[NMEA_INSTANCE].disabled = !enabled;

	if (enabled) {
		gs_tty_driver->init_termios = tty_std_termios;
		gs_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL
		    | CLOCAL;

		gs_tty_driver->init_termios.c_lflag = 0;
		gs_tty_driver->init_termios.c_iflag = 0;
		gs_tty_driver->init_termios.c_oflag = 0;
	}

	usb_function_enable("nmea", enabled);
	return 0;
}

static int serial_get_nmea_enabled(char *buffer, struct kernel_param *kp)
{
	buffer[0] = '0' + !usb_function_serial[NMEA_INSTANCE].disabled;
	return 1;
}

module_param_call(nmea_enabled, serial_set_nmea_enabled, serial_get_nmea_enabled, 0, 0664);

static int serial_set_serial_enabled(const char *val, struct kernel_param *kp)
{
	int enabled = simple_strtol(val, NULL, 0);
	usb_function_serial[SERIAL_INSTANCE].disabled = !enabled;

	if (enabled) {
		gs_tty_driver->init_termios = tty_std_termios;
		gs_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL
		    | CLOCAL;

		gs_tty_driver->init_termios.c_lflag = 0;
		gs_tty_driver->init_termios.c_iflag = 0;
		gs_tty_driver->init_termios.c_oflag = 0;
	}

	usb_function_enable("serial", enabled);

	return 0;
}

static int serial_get_serial_enabled(char *buffer, struct kernel_param *kp)
{
	buffer[0] = '0' + !usb_function_serial[SERIAL_INSTANCE].disabled;
	return 1;
}

module_param_call(serial_enabled, serial_set_serial_enabled, serial_get_serial_enabled, 0, 0664);

