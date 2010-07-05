/* linux/arch/arm/mach-msm/board-heroc-panel.c
 *
 * Copyright (C) 2008 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/leds.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>

#include <mach/msm_fb.h>
#include <mach/vreg.h>
#include <mach/pmic.h>
#include <mach/board_htc.h>
#include "proc_comm.h"
#include "devices.h"
#include "board-heroc.h"

#if 0
#define B(s...) printk(s)
#else
#define B(s...) do {} while(0)
#endif

static struct led_trigger *heroc_lcd_backlight;
static void heroc_set_backlight(int on)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);
	if (on) {
		/* vsync back porch is about 17 ms */
		msleep(40);
		led_trigger_event(heroc_lcd_backlight, LED_FULL);
	} else
		led_trigger_event(heroc_lcd_backlight, LED_OFF);
}

static struct vreg *vreg_lcm_2v6;
static struct vreg *vreg_lcm_2v85;

static void 
heroc_mddi_eid_power(struct msm_mddi_client_data *client_data, int on)
{
	unsigned id, on_off = 1;

	B(KERN_DEBUG "%s: power %s.\n", __func__, on ? "on" : "off");
	if (on) {
		on_off = 0;
		/* 2V6(pmic synt) */
		id = PM_VREG_PDOWN_SYNT_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcm_2v6);
		mdelay(1);

		/* 2V8(pmic gp5) */
		id = PM_VREG_PDOWN_GP5_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcm_2v85);
		mdelay(2);

		gpio_set_value(HEROC_GPIO_MDDI_RST_N, 1);
		mdelay(15);
	} else {
		on_off = 1;
		mdelay(5);
		gpio_set_value(HEROC_GPIO_MDDI_RST_N, 0);
		mdelay(3);

		/* 2V8(pmic gp5) */
		id = PM_VREG_PDOWN_GP5_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_disable(vreg_lcm_2v85);
		mdelay(1);

		/* 2V6(pmic synt) */
		id = PM_VREG_PDOWN_SYNT_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_disable(vreg_lcm_2v6);
	}
}

enum {
	PANEL_SHARP,
	PANEL_SAMSUNG,
	PANEL_EID_40pin,
	PANEL_EID_24pin,
	PANEL_HEROC_EID_BOTTOM,
	PANEL_TPO,
	PANEL_HEROC_TPO,
	PANEL_ESPRESSO_TPO,
	PANEL_ESPRESSO_SHARP,
	PANEL_LIBERTY_TPO,
	PANEL_LIBERTY_EID_24pin,
	PANEL_EIDII,
	PANEL_UNKNOWN,
};

static int heroc_panel_detect(void)
{
	return panel_type;
}

static int 
heroc_panel_unblank(struct msm_mddi_bridge_platform_data *bridge_data,
		struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);
	heroc_set_backlight(1);
	return 0;
}

static int 
heroc_panel_blank(struct msm_mddi_bridge_platform_data *bridge_data,
		struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);
	heroc_set_backlight(0);
	return 0;
}

static void panel_eid_fixup(uint16_t *mfr_name, uint16_t *product_code)
{
	B("%s: enter.\n", __func__);
	*mfr_name = 0x0101;
	*product_code= 0x0;
}

static int config_vsync(void)
{
	int ret;
	uint32_t config;

	ret = gpio_request(HEROC_GPIO_VSYNC, "vsync");
	if (ret)
		return ret;

	config = PCOM_GPIO_CFG(HEROC_GPIO_VSYNC, 1, GPIO_INPUT,
				GPIO_PULL_DOWN, GPIO_2MA);
	ret = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
	if (ret)
		gpio_free(HEROC_GPIO_VSYNC);
	return ret;
}

static u8 pwm_eid[10] = {30, 34, 45, 61, 81, 108, 145, 182, 219, 255};
static u8 pwm_tpo[10] = {30, 34, 45, 61, 81, 108, 145, 182, 219, 255};

static struct msm_mddi_bridge_platform_data eid_client_data = {
	.blank = heroc_panel_blank,
	.unblank = heroc_panel_unblank,
	.fb_data = {
		.xres = 320,
		.yres = 480,
		.output_format = 0,
	},
};

static struct resource resources_msm_fb[] = {
	{
		.start = MSM_FB_BASE,
		.end = MSM_FB_BASE + MSM_FB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct msm_mddi_platform_data heroc_pdata = {
	.clk_rate = 106000000,
	.power_client = heroc_mddi_eid_power,
	.fixup = panel_eid_fixup,
	.fb_resource = resources_msm_fb,
	.num_clients = 1,
	.client_platform_data = {
		{
			.product_id = (0x0101 << 16 | 0),
			.name = "mddi_c_0101_0000",
			.id = 1,
			.client_data = &eid_client_data,
			.clk_rate = 0,
		},
	},
};

/*
 * In boot loader, mddi is powered on already.
 * So, we just detect panel here, setting different power function for each
 * panel. Then we did not have to detect panel in each time mddi_client_power
 * or panel_power is called.
 *
 * jay: Nov 20, 08'
 */
int __init heroc_init_panel(void)
{
	int panel, rc;
	struct panel_data *config = &eid_client_data.panel_conf;

	if (!machine_is_heroc())
		return -1;

	B(KERN_INFO "%s: enter.\n", __func__);

	vreg_lcm_2v6 = vreg_get(0, "synt");
	if (IS_ERR(vreg_lcm_2v6))
		return PTR_ERR(vreg_lcm_2v6);

	vreg_lcm_2v85 = vreg_get(0, "gp5");
	if (IS_ERR(vreg_lcm_2v85))
		return PTR_ERR(vreg_lcm_2v85);

	panel = heroc_panel_detect();

	if (panel == PANEL_HEROC_EID_BOTTOM ||
		panel == PANEL_EIDII ||
		panel == PANEL_HEROC_TPO) {
		printk(KERN_INFO "%s: init %s panel\n", __func__,
			panel == PANEL_HEROC_TPO ? "TPO" : "EID");
		config->panel_id = panel;
		config->caps = MSMFB_CAP_CABC;
		if (panel == PANEL_HEROC_EID_BOTTOM ||
			panel == PANEL_EIDII)
			config->pwm = pwm_eid;
		else
			config->pwm = pwm_tpo;
		config->shrink = 1;
		config->min_level = 2;
	} else {
		printk(KERN_ERR "unknown panel type!\n");
		return -EIO;
	}

	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	msm_device_mddi0.dev.platform_data = &heroc_pdata;
	rc = platform_device_register(&msm_device_mddi0);
	if (rc)
		return rc;
	
	led_trigger_register_simple("lcd-backlight-gate", &heroc_lcd_backlight);
	if (IS_ERR(heroc_lcd_backlight))
		printk(KERN_ERR "%s: backlight registration failed!\n", __func__);

	return 0;
}
device_initcall(heroc_init_panel);

