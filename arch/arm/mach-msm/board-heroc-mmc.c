/* linux/arch/arm/mach-msm7201a/board-heroc-mmc.c
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
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/irq.h>
#include <linux/gpio.h>

#include <asm/io.h>
#include <asm/mach/mmc.h>

#include <mach/vreg.h>
#include "proc_comm.h"
#include <mach/board.h>
#include <mach/htc_pwrsink.h>

#include "board-heroc.h"

#define DEBUG_SDSLOT_VDD 1

/* r porting 29 */
extern int msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat,
                        unsigned int stat_irq, unsigned long stat_irq_flags);

/* ---- SDCARD ---- */

static uint32_t sdcard_on_gpio_table[] = {
	PCOM_GPIO_CFG(62, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	PCOM_GPIO_CFG(63, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(64, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT3 */
	PCOM_GPIO_CFG(65, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT2 */
	PCOM_GPIO_CFG(66, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(67, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT0 */
};

static uint32_t sdcard_off_gpio_table[] = {
	PCOM_GPIO_CFG(62, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CLK */
	PCOM_GPIO_CFG(63, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CMD */
	PCOM_GPIO_CFG(64, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(65, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(66, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(67, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
};

static uint opt_disable_sdcard;

static int __init heroc_disablesdcard_setup(char *str)
{
	int cal = simple_strtol(str, NULL, 0);

	opt_disable_sdcard = cal;
	return 1;
}

__setup("board_heroc.disable_sdcard=", heroc_disablesdcard_setup);

static struct vreg *vreg_sdslot;	/* SD slot power */

struct mmc_vdd_xlat {
	int mask;
	int level;
};

static struct mmc_vdd_xlat mmc_vdd_table[] = {
	{ MMC_VDD_27_28,	2800 },
	{ MMC_VDD_28_29,	2850 },
	{ MMC_VDD_29_30,	2900 },
};

static unsigned int sdslot_vdd = 0xffffffff;
static unsigned int sdslot_vreg_enabled;

static uint32_t heroc_sdslot_switchvdd(struct device *dev, unsigned int vdd)
{
	int i;

	BUG_ON(!vreg_sdslot);

	if (vdd == sdslot_vdd)
		return 0;

	sdslot_vdd = vdd;

	if (vdd == 0) {
#if DEBUG_SDSLOT_VDD
		printk(KERN_INFO "%s: Disabling SD slot power\n", __func__);
#endif
		config_gpio_table(sdcard_off_gpio_table,
				  ARRAY_SIZE(sdcard_off_gpio_table));
		vreg_disable(vreg_sdslot);
		sdslot_vreg_enabled = 0;
		return 0;
	}

	if (!sdslot_vreg_enabled) {
		mdelay(5);
		vreg_enable(vreg_sdslot);
		udelay(500);
		config_gpio_table(sdcard_on_gpio_table,
				  ARRAY_SIZE(sdcard_on_gpio_table));
		sdslot_vreg_enabled = 1;
	}

	for (i = 0; i < ARRAY_SIZE(mmc_vdd_table); i++) {
		if (mmc_vdd_table[i].mask == (1 << vdd)) {
#if DEBUG_SDSLOT_VDD
			printk(KERN_INFO "%s: Setting level to %u\n",
					__func__, mmc_vdd_table[i].level);
#endif
			vreg_set_level(vreg_sdslot, mmc_vdd_table[i].level);
			return 0;
		}
	}

	printk(KERN_ERR "%s: Invalid VDD %d specified\n", __func__, vdd);
	return 0;
}

static unsigned int heroc_sdslot_status(struct device *dev)
{
	unsigned int status;

	status = (unsigned int) gpio_get_value(HEROC_GPIO_SDMC_CD_N);
	return (!status);
}

#define HEROC_MMC_VDD	MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30

static unsigned int heroc_sdslot_type = MMC_TYPE_SD;

static struct mmc_platform_data heroc_sdslot_data = {
	.ocr_mask	= HEROC_MMC_VDD,
	.status_irq	= MSM_GPIO_TO_INT(HEROC_GPIO_SDMC_CD_N),
	.status		= heroc_sdslot_status,
	.translate_vdd	= heroc_sdslot_switchvdd,
	.slot_type	= &heroc_sdslot_type,
};



/* ---- WIFI ---- */

static uint32_t wifi_on_gpio_table[] = {
	PCOM_GPIO_CFG(51, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(52, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(53, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(54, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(55, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(56, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	PCOM_GPIO_CFG(29, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA),  /* WLAN IRQ */
};

static uint32_t wifi_off_gpio_table[] = {
	PCOM_GPIO_CFG(51, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(52, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(53, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(54, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(55, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CMD */
	PCOM_GPIO_CFG(56, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CLK */
	PCOM_GPIO_CFG(29, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* WLAN IRQ */
};

static struct vreg *vreg_wifi_osc;	/* WIFI 32khz oscilator */
static int heroc_wifi_cd;		/* WIFI virtual 'card detect' status */

static struct sdio_embedded_func wifi_func = {
	.f_class 	= SDIO_CLASS_WLAN,
	.f_maxblksize   = 512,
};

static struct embedded_sdio_data heroc_wifi_emb_data = {
	.cis	= {
		.vendor		= 0x104c,
		.device		= 0x9066,
		.blksize	= 512,
		/*.max_dtr	= 24000000,  Max of chip - no worky on Trout */
		.max_dtr	= 20000000,
	},
	.cccr	= {
		.multi_block	= 0,
		.low_speed	= 0,
		.wide_bus	= 1,
		.high_power	= 0,
		.high_speed	= 0,
	},
	.funcs	= &wifi_func,
	.num_funcs = 1,
};

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int
heroc_wifi_status_register(void (*callback)(int card_present, void *dev_id),
				void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static unsigned int heroc_wifi_status(struct device *dev)
{
	return heroc_wifi_cd;
}

static struct mmc_platform_data heroc_wifi_data = {
	.ocr_mask		= MMC_VDD_28_29,
	.status			= heroc_wifi_status,
	.register_status_notify	= heroc_wifi_status_register,
	.embedded_sdio		= &heroc_wifi_emb_data,
};

int heroc_wifi_set_carddetect(int val)
{
	printk(KERN_INFO "%s: %d\n", __func__, val);
	heroc_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
	return 0;
}

static int heroc_wifi_power_state;
static int heroc_bt_power_state;

int heroc_wifi_power(int on)
{
	int rc;

	printk(KERN_INFO "%s: %d\n", __func__, on);

	if (on) {
		config_gpio_table(wifi_on_gpio_table,
				  ARRAY_SIZE(wifi_on_gpio_table));
		mdelay(50);
		rc = vreg_enable(vreg_wifi_osc);
		vreg_set_level(vreg_wifi_osc, 1800);
		mdelay(50);
		if (rc)
			return rc;
		htc_pwrsink_set(PWRSINK_WIFI, 70);
	} else {
		config_gpio_table(wifi_off_gpio_table,
				  ARRAY_SIZE(wifi_off_gpio_table));
		htc_pwrsink_set(PWRSINK_WIFI, 0);
	}

	gpio_set_value(HEROC_GPIO_WIFI_EN, on);
	mdelay(100);

	if (!on) {
		if (!heroc_bt_power_state)
			vreg_disable(vreg_wifi_osc);
		else
			printk(KERN_ERR "WiFi shouldn't disable "
				"vreg_wifi_osc. BT is using it!!\n");
	}
	heroc_wifi_power_state = on;
	return 0;
}

/* Eenable VREG_MMC pin to turn on fastclock oscillator : colin */
int heroc_bt_fastclock_power(int on)
{
	int rc;

	printk(KERN_INFO "%s: %d\n", __func__, on);

	if (vreg_wifi_osc) {
		if (on) {
			rc = vreg_enable(vreg_wifi_osc);

			if (rc)	{
				printk(KERN_ERR "Error turn "
					"bt_fastclock_power rc=%d\n", rc);
				return rc;
			}
		} else {
			if (!heroc_wifi_power_state)
				vreg_disable(vreg_wifi_osc);
		}
	}
	heroc_bt_power_state = on;
	return 0;
}
EXPORT_SYMBOL(heroc_bt_fastclock_power);

static int heroc_wifi_reset_state;
int heroc_wifi_reset(int on)
{
	printk(KERN_INFO "%s: %d\n", __func__, on);
	/* HRRO use power off/on instead wifi reset*/
	heroc_wifi_reset_state = on;
	/* mdelay(50); */
	return 0;
}

int __init heroc_init_mmc(unsigned int sys_rev)
{
	wifi_status_cb = NULL;
	sdslot_vreg_enabled = 0;

	vreg_wifi_osc = vreg_get(0, "gp4");
	if (IS_ERR(vreg_wifi_osc))
		return PTR_ERR(vreg_wifi_osc);
	vreg_set_level(vreg_wifi_osc, 1800);

	msm_add_sdcc(1, &heroc_wifi_data, 0, 0); /* r porting 29: change func*/


	if (opt_disable_sdcard) {
		printk(KERN_INFO "heroc: SD-Card interface disabled\n");
		goto done;
	}

	vreg_sdslot = vreg_get(0, "gp6");
	if (IS_ERR(vreg_sdslot))
		return PTR_ERR(vreg_sdslot);

	set_irq_wake(MSM_GPIO_TO_INT(HEROC_GPIO_SDMC_CD_N), 1);

	msm_add_sdcc(2, &heroc_sdslot_data, MSM_GPIO_TO_INT(HEROC_GPIO_SDMC_CD_N),
		IORESOURCE_IRQ_LOWEDGE | IORESOURCE_IRQ_HIGHEDGE); /* r porting 29 */

done:
	return 0;
}


