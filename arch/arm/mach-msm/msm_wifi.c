/*
 * platform driver for msm_wifi device
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * Copyright (C) 2008 Google Inc
 */
#include <linux/platform_device.h>
#include <linux/wifi_tiwlan.h>

static int wifi_probe(struct platform_device *pdev)
{
	struct wifi_platform_data *wifi_ctrl =
		(struct wifi_platform_data *)(pdev->dev.platform_data);

	printk(KERN_DEBUG "wifi probe start\n");

	if (!wifi_ctrl)
		return -ENODEV;

	if (wifi_ctrl->set_power)
		wifi_ctrl->set_power(1);	/* Power On */
	if (wifi_ctrl->set_reset)
		wifi_ctrl->set_reset(0);	/* Reset clear */
	if (wifi_ctrl->set_carddetect)
		wifi_ctrl->set_carddetect(1);	/* CardDetect (0->1) */

	printk(KERN_DEBUG "wifi probe done\n");
	return 0;
}

static int wifi_remove(struct platform_device *pdev)
{
	struct wifi_platform_data *wifi_ctrl =
		(struct wifi_platform_data *)(pdev->dev.platform_data);

	printk(KERN_DEBUG "wifi remove start\n");
	if (!wifi_ctrl)
		return -ENODEV;

	if (wifi_ctrl->set_carddetect)
		wifi_ctrl->set_carddetect(0);	/* CardDetect (1->0) */
	if (wifi_ctrl->set_reset)
		wifi_ctrl->set_reset(1);	/* Reset active */
	if (wifi_ctrl->set_power)
		wifi_ctrl->set_power(0);	/* Power Off */

	printk(KERN_DEBUG "wifi remove end\n");
	return 0;
}

static struct platform_driver wifi_device = {
	.probe		= wifi_probe,
	.remove		= wifi_remove,
	.driver		= {
		.name   = "msm_wifi",
	},
};

static int __init msm_wifi_sdio_init(void)
{
	return platform_driver_register(&wifi_device);
}

static void __exit msm_wifi_sdio_exit(void)
{
	platform_driver_unregister(&wifi_device);
}

module_init(msm_wifi_sdio_init);
module_exit(msm_wifi_sdio_exit);
MODULE_LICENSE("GPL");
