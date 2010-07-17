/* drivers/usb/function/diag.c
 *
 * Diag Function Device - Route DIAG frames between SMD and USB
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
#include <linux/platform_device.h>

#include <mach/msm_smd.h>

#include "usb_function.h"

#if 1
#define TRACE(tag,data,len,decode) do {} while(0)
#else
static void TRACE(const char *tag, const void *_data, int len, int decode)
{
	const unsigned char *data = _data;
	int escape = 0;

	printk(KERN_INFO "%s", tag);
	if (decode) {
		while (len-- > 0) {
			unsigned x = *data++;
			if (x == 0x7e) {
				printk(" $$");
				escape = 0;
				continue;
			}
			if (x == 0x7d) {
				escape = 1;
				continue;
			}
			if (escape) {
				escape = 0;
				printk(" %02x", x ^ 0x20);
			} else {
				printk(" %02x", x);
			}
		}
	} else {
		while (len-- > 0) {
			printk(" %02x", *data++);
		}
		printk(" $$");
	}
	printk("\n");
}
#endif

#define HDLC_MAX 4096

struct diag_context
{
	struct usb_endpoint *out;
	struct usb_endpoint *in;
	struct usb_request *req_out;
	struct usb_request *req_in;
	smd_channel_t *ch;
	int in_busy;

	/* assembly buffer for USB->A9 HDLC frames */
	unsigned char hdlc_buf[HDLC_MAX];
	unsigned hdlc_count;
	unsigned hdlc_escape;
};

static struct diag_context _context;

static void smd_try_to_send(struct diag_context *ctxt);

static void diag_bind(struct usb_endpoint **ept, void *_ctxt)
{
	struct diag_context *ctxt = _ctxt;

	ctxt->out = ept[0];
	ctxt->in = ept[1];

	ctxt->req_out = usb_ept_alloc_req(ctxt->out, 4096);
	ctxt->req_in = usb_ept_alloc_req(ctxt->in, 4096);
}

static void diag_queue_in(struct diag_context *ctxt, void *data, unsigned len);
static void diag_queue_out(struct diag_context *ctxt);

static void diag_in_complete(struct usb_endpoint *ept, struct usb_request *req)
{
	struct diag_context *ctxt = req->context;
	ctxt->in_busy = 0;
	smd_try_to_send(ctxt);
}

static void diag_process_hdlc(struct diag_context *ctxt, void *_data, unsigned len)
{
	unsigned char *data = _data;
	unsigned count = ctxt->hdlc_count;
	unsigned escape = ctxt->hdlc_escape;
	unsigned char *hdlc = ctxt->hdlc_buf;

	while (len-- > 0) {
		unsigned char x = *data++;
		if (x == 0x7E) { 
			if (count > 2) {
				/* we're just ignoring the crc here */
				TRACE("PC>", hdlc, count - 2, 0);
				if (ctxt->ch)
					smd_write(ctxt->ch, hdlc, count - 2);
			}
			count = 0;
			escape = 0;
		} else if (x == 0x7D) {
			escape = 1;
		} else {
			if (escape) {
				x = x ^ 0x20;
				escape = 0;
			}
			hdlc[count++] = x;

			/* discard frame if we overflow */
			if (count == HDLC_MAX)
				count = 0;
		}
	}

	ctxt->hdlc_count = count;
	ctxt->hdlc_escape = escape;
}

static void diag_out_complete(struct usb_endpoint *ept, struct usb_request *req)
{
	struct diag_context *ctxt = req->context;

	if (req->status == 0) {
		diag_process_hdlc(ctxt, req->buf, req->actual);		
	}
	diag_queue_out(ctxt);
}

static void diag_queue_out(struct diag_context *ctxt)
{
	struct usb_request *req = ctxt->req_out;

	req->complete = diag_out_complete;
	req->context = ctxt;
	req->length = 4096;

	usb_ept_queue_xfer(ctxt->out, req);
}

static void diag_queue_in(struct diag_context *ctxt, void *data, unsigned len)
{
	struct usb_request *req = ctxt->req_in;

	memcpy(req->buf, data, len);
	req->complete = diag_in_complete;
	req->context = ctxt;
	req->length = len;

	usb_ept_queue_xfer(ctxt->in, req);
}

static void smd_try_to_send(struct diag_context *ctxt)
{
	if (ctxt->ch && (!ctxt->in_busy)) {
		int r = smd_read_avail(ctxt->ch);

		if (r > 4096) {
			return;
		}
		if (r > 0) {
			struct usb_request *req = ctxt->req_in;
			smd_read(ctxt->ch, req->buf, r);
			req->complete = diag_in_complete;
			req->context = ctxt;
			req->length = r;

			TRACE("A9>", req->buf, r, 1);
			ctxt->in_busy = 1;
			usb_ept_queue_xfer(ctxt->in, req);
		}
	}
}

static void smd_diag_notify(void *priv, unsigned event)
{
	struct diag_context *ctxt = priv;
	smd_try_to_send(ctxt);
}


static void diag_configure(int configured, void *_ctxt)
{
	struct diag_context *ctxt = _ctxt;
	printk(KERN_INFO "diag_configure() %d\n", configured);

	if (configured) {
		diag_queue_out(ctxt);
	} else {
		/* all pending requests will be canceled */
	}
}

static struct usb_function usb_func_diag = {
	.bind = diag_bind,
	.configure = diag_configure,

	.name = "diag",
	.context = &_context,

	.ifc_class = 0xff,
	.ifc_subclass = 0xff,
	.ifc_protocol = 0xff,

	.ifc_name = "diag",

	.ifc_ept_count = 2,
	.ifc_ept_type = { EPT_BULK_OUT, EPT_BULK_IN },
};

static int msm_diag_probe(struct platform_device *pdev)
{
	struct diag_context *ctxt = &_context;
	int r;
	r = smd_open("SMD_DIAG", &ctxt->ch, ctxt, smd_diag_notify);
	return 0;
}

static struct platform_driver msm_smd_ch1_driver = {
	.probe = msm_diag_probe,
	.driver = {
		.name = "SMD_DIAG",
		.owner = THIS_MODULE,
	},
};

static int __init diag_init(void)
{
	int r;
	r = usb_function_register(&usb_func_diag);

	if (!r)
		r = platform_driver_register(&msm_smd_ch1_driver);

	return r;
}

module_init(diag_init);
