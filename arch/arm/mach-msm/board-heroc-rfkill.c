/*
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
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

/* Control bluetooth power for heroc platform */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>

#include "proc_comm.h"
#include "board-heroc.h"

extern int heroc_bt_fastclock_power(int on);

static struct rfkill *bt_rfk;
static const char bt_name[] = "brf6350";

static int heroc_bt_status;

static uint32_t heroc_bt_init_table[] = {
	PCOM_GPIO_CFG(HEROC_GPIO_UART1_RTS, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* BT_RTS */
	PCOM_GPIO_CFG(HEROC_GPIO_UART1_CTS, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_8MA),	/* BT_CTS */
	PCOM_GPIO_CFG(HEROC_GPIO_UART1_RX, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_8MA),		/* BT_RX */
	PCOM_GPIO_CFG(HEROC_GPIO_UART1_TX, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* BT_TX */
	
	PCOM_GPIO_CFG(HEROC_GPIO_WB_SHUT_DOWN_N, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_8MA),		/* BT_ENABLE */
};

static uint32_t heroc_bt_on_table[] = {
	PCOM_GPIO_CFG(HEROC_GPIO_UART1_RTS, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* BT_RTS */
	PCOM_GPIO_CFG(HEROC_GPIO_UART1_CTS, 2, GPIO_INPUT, GPIO_NO_PULL, GPIO_8MA),	/* BT_CTS */
	PCOM_GPIO_CFG(HEROC_GPIO_UART1_RX, 2, GPIO_INPUT, GPIO_NO_PULL, GPIO_8MA),		/* BT_RX */
	PCOM_GPIO_CFG(HEROC_GPIO_UART1_TX, 3, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* BT_TX */
	
	PCOM_GPIO_CFG(HEROC_GPIO_WB_SHUT_DOWN_N, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_8MA),		/* BT_ENABLE */
};

static uint32_t heroc_bt_off_table[] = {
	PCOM_GPIO_CFG(HEROC_GPIO_UART1_RTS, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* BT_RTS */
	PCOM_GPIO_CFG(HEROC_GPIO_UART1_CTS, 2, GPIO_INPUT, GPIO_NO_PULL, GPIO_8MA),	/* BT_CTS */
	PCOM_GPIO_CFG(HEROC_GPIO_UART1_RX, 2, GPIO_INPUT, GPIO_NO_PULL, GPIO_8MA),		/* BT_RX */
	PCOM_GPIO_CFG(HEROC_GPIO_UART1_TX, 3, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* BT_TX */
	
	PCOM_GPIO_CFG(HEROC_GPIO_WB_SHUT_DOWN_N, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_8MA),		/* BT_ENABLE */
};

static uint32_t heroc_bt_disable_active_table[] = {
	PCOM_GPIO_CFG(HEROC_GPIO_UART1_RTS, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* BT_RTS */
	PCOM_GPIO_CFG(HEROC_GPIO_UART1_CTS, 2, GPIO_INPUT, GPIO_NO_PULL, GPIO_8MA),	/* BT_CTS */
	PCOM_GPIO_CFG(HEROC_GPIO_UART1_RX, 2, GPIO_INPUT, GPIO_NO_PULL, GPIO_8MA),		/* BT_RX */
	PCOM_GPIO_CFG(HEROC_GPIO_UART1_TX, 3, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* BT_TX */
};

static uint32_t heroc_bt_disable_sleep_table[] = {
	PCOM_GPIO_CFG(HEROC_GPIO_UART1_RTS, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* O(L) */
	PCOM_GPIO_CFG(HEROC_GPIO_UART1_CTS, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA),	/* I(PU) */
	PCOM_GPIO_CFG(HEROC_GPIO_UART1_RX, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA),		/* I(PU) */
	PCOM_GPIO_CFG(HEROC_GPIO_UART1_TX, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* O(H) */
};

static void config_bt_table(uint32_t *table, int len)
{
	int n;
	unsigned id;
	for(n = 0; n < len; n++) {
		id = table[n];
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

static void heroc_config_bt_init(void)
{
	heroc_bt_status = 0;
	config_bt_table(heroc_bt_init_table, ARRAY_SIZE(heroc_bt_init_table));
	mdelay(5);
	gpio_configure(HEROC_GPIO_WB_SHUT_DOWN_N, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
}

static void heroc_config_bt_on(void)
{
	config_bt_table(heroc_bt_on_table, ARRAY_SIZE(heroc_bt_on_table));
	mdelay(2);

	gpio_configure(HEROC_GPIO_WB_SHUT_DOWN_N,
			GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
	mdelay(15);
	gpio_configure(HEROC_GPIO_WB_SHUT_DOWN_N,
			GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	mdelay(1);
	gpio_configure(HEROC_GPIO_WB_SHUT_DOWN_N,
			GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
	mdelay(1);

	heroc_bt_fastclock_power(1);
	mdelay(2);
	heroc_bt_status = 1;
}

static void heroc_config_bt_off(void)
{
	gpio_configure(HEROC_GPIO_WB_SHUT_DOWN_N, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
	heroc_bt_fastclock_power(0);
	config_bt_table(heroc_bt_off_table, ARRAY_SIZE(heroc_bt_off_table));
	mdelay(5);
	heroc_bt_status = 0;
}

void heroc_config_bt_disable_active(void)
{	
	config_bt_table(heroc_bt_disable_active_table, ARRAY_SIZE(heroc_bt_disable_active_table));
}

void heroc_config_bt_disable_sleep(void)
{
	config_bt_table(heroc_bt_disable_sleep_table, ARRAY_SIZE(heroc_bt_disable_sleep_table));
	mdelay(5);
	gpio_configure(HEROC_GPIO_UART1_RTS, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);	/* O(L) */
	gpio_configure(HEROC_GPIO_UART1_TX, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);	/* O(H) */
}

int heroc_is_bluetooth_off(void)
{
	return !heroc_bt_status;	//ON:1, OFF:0
}

static int bluetooth_set_power(void *data, enum rfkill_state state)
{
	switch (state) {
	case RFKILL_STATE_UNBLOCKED:
			heroc_config_bt_on();
		break;
	case RFKILL_STATE_SOFT_BLOCKED:
			heroc_config_bt_off();
		break;
	default:
		printk(KERN_ERR "bad bluetooth rfkill state %d\n", state);
	}
	return 0;
}

static int __init heroc_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	enum rfkill_state default_state = RFKILL_STATE_SOFT_BLOCKED;  /* off */

	heroc_config_bt_init();	/* bt gpio initial config */

	rfkill_set_default(RFKILL_TYPE_BLUETOOTH, default_state);
	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_allocate(&pdev->dev, RFKILL_TYPE_BLUETOOTH);
	if (!bt_rfk)
		return -ENOMEM;

	bt_rfk->name = bt_name;
	bt_rfk->state = default_state;
	/* userspace cannot take exclusive control */
	bt_rfk->user_claim_unsupported = 1;
	bt_rfk->user_claim = 0;
	bt_rfk->data = NULL;  // user data
	bt_rfk->toggle_radio = bluetooth_set_power;

	rc = rfkill_register(bt_rfk);

	if (rc)
		rfkill_free(bt_rfk);
	return rc;
}

static int heroc_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_free(bt_rfk);

	return 0;
}

static struct platform_driver heroc_rfkill_driver = {
	.probe = heroc_rfkill_probe,
	.remove = heroc_rfkill_remove,
	.driver = {
		.name = "heroc_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init heroc_rfkill_init(void)
{
	if (!machine_is_heroc())
		return 0;
	return platform_driver_register(&heroc_rfkill_driver);
}

static void __exit heroc_rfkill_exit(void)
{
	platform_driver_unregister(&heroc_rfkill_driver);
}

module_init(heroc_rfkill_init);
module_exit(heroc_rfkill_exit);
MODULE_DESCRIPTION("heroc rfkill");
MODULE_AUTHOR("Nick Pelly <npelly@google.com>");
MODULE_LICENSE("GPL");
