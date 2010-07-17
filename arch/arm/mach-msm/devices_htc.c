/* linux/arch/arm/mach-msm/devices.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (C) 2007-2009 HTC Corporation.
 * Author: Thomas Tsai <thomas_tsai@htc.com>
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
#include <linux/platform_device.h>

#include <linux/dma-mapping.h>
#include <mach/msm_iomap.h>
#include <mach/dma.h>
#include "gpio_chip.h"
#include "devices.h"
#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/msm_hsusb.h>
#include <linux/usb/android.h>

#include <asm/mach/flash.h>
#include <asm/setup.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <linux/android_pmem.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_iomap.h>
#include <asm/mach/mmc.h>
#include "proc_comm.h"


struct platform_device *devices[] __initdata = {
	&msm_device_nand,
	&msm_device_smd,
};

void __init msm_add_devices(void)
{
	platform_add_devices(devices, ARRAY_SIZE(devices));
}

/*
void __init msm_change_usb_id(__u16 vendor_id, __u16 product_id)
{
	msm_hsusb_pdata.vendor_id = vendor_id;
	msm_hsusb_pdata.product_id = product_id;
}
*/
static struct android_pmem_platform_data pmem_pdata = {
	.name = "pmem",
	.no_allocator = 1,
	.cached = 1,
};

static struct android_pmem_platform_data pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.no_allocator = 0,
#if defined(CONFIG_ARCH_MSM7227)
	.cached = 1,
#else
	.cached = 0,
#endif
};

static struct android_pmem_platform_data pmem_camera_pdata = {
	.name = "pmem_camera",
	.no_allocator = 1,
	.cached = 0,
};

#ifdef CONFIG_BUILD_CIQ
static struct android_pmem_platform_data pmem_ciq_pdata = {
	.name = "pmem_ciq",
	.no_allocator = 0,
	.cached = 0,
};

static struct android_pmem_platform_data pmem_ciq1_pdata = {
	.name = "pmem_ciq1",
	.no_allocator = 0,
	.cached = 0,
};

static struct android_pmem_platform_data pmem_ciq2_pdata = {
	.name = "pmem_ciq2",
	.no_allocator = 0,
	.cached = 0,
};

static struct android_pmem_platform_data pmem_ciq3_pdata = {
	.name = "pmem_ciq3",
	.no_allocator = 0,
	.cached = 0,
};
#endif

static struct platform_device pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &pmem_pdata },
};

static struct platform_device pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &pmem_adsp_pdata },
};

static struct platform_device pmem_camera_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &pmem_camera_pdata },
};

#ifdef CONFIG_BUILD_CIQ
static struct platform_device pmem_ciq_device = {
	.name = "android_pmem",
	.id = 5,
	.dev = { .platform_data = &pmem_ciq_pdata },
};

static struct platform_device pmem_ciq1_device = {
	.name = "android_pmem",
	.id = 6,
	.dev = { .platform_data = &pmem_ciq1_pdata },
};

static struct platform_device pmem_ciq2_device = {
	.name = "android_pmem",
	.id = 7,
	.dev = { .platform_data = &pmem_ciq2_pdata },
};

static struct platform_device pmem_ciq3_device = {
	.name = "android_pmem",
	.id = 8,
	.dev = { .platform_data = &pmem_ciq3_pdata },
};
#endif

static struct platform_device msm_camera_device = {
	.name	= "msm_camera",
	.id	= 0,
};

static void __init msm_register_device(struct platform_device *pdev, void *data)
{
	int ret;

	pdev->dev.platform_data = data;

	ret = platform_device_register(pdev);
	if (ret)
		printk("%s: platform_device_register() failed = %d\n",
			 __func__, ret);
}

void __init msm_camera_register_device(void *res, uint32_t num,
	void *data)
{
	msm_camera_device.num_resources = num;
	msm_camera_device.resource = res;

	msm_register_device(&msm_camera_device, data);
}

static struct resource ram_console_resource[] = {
	{
		.flags	= IORESOURCE_MEM,
	}
};

static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
	.num_resources  = ARRAY_SIZE(ram_console_resource),
	.resource       = ram_console_resource,
};

#if defined(CONFIG_MSM_HW3D)
static struct resource resources_hw3d[] = {
	{
		.start	= 0xA0000000,
		.end	= 0xA00fffff,
		.flags	= IORESOURCE_MEM,
		.name	= "regs",
	},
	{
		.flags	= IORESOURCE_MEM,
		.name	= "smi",
	},
	{
		.flags	= IORESOURCE_MEM,
		.name	= "ebi",
	},
	{
		.start	= INT_GRAPHICS,
		.end	= INT_GRAPHICS,
		.flags	= IORESOURCE_IRQ,
		.name	= "gfx",
	},
};

static struct platform_device hw3d_device = {
	.name		= "msm_hw3d",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_hw3d),
	.resource	= resources_hw3d,
};
#endif

#if defined(CONFIG_GPU_MSM_KGSL)
static struct resource msm_kgsl_resources[] = {
	{
		.name	= "kgsl_reg_memory",
		.start	= MSM_GPU_REG_PHYS,
		.end	= MSM_GPU_REG_PHYS + MSM_GPU_REG_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "kgsl_phys_memory",
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_GRAPHICS,
		.end	= INT_GRAPHICS,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_kgsl_device = {
	.name		= "kgsl",
	.id		= -1,
	.resource	= msm_kgsl_resources,
	.num_resources	= ARRAY_SIZE(msm_kgsl_resources),
};

#define PWR_RAIL_GRP_CLK               8
static int kgsl_power_rail_mode(int follow_clk)
{
       int mode = follow_clk ? 0 : 1;
       int rail_id = PWR_RAIL_GRP_CLK;

       return msm_proc_comm(PCOM_CLKCTL_RPC_RAIL_CONTROL, &rail_id, &mode);
}

static int kgsl_power(bool on)
{
       int cmd;
       int rail_id = PWR_RAIL_GRP_CLK;

       cmd = on ? PCOM_CLKCTL_RPC_RAIL_ENABLE : PCOM_CLKCTL_RPC_RAIL_DISABLE;
       return msm_proc_comm(cmd, &rail_id, NULL);
}

#endif

void __init msm_add_mem_devices(struct msm_pmem_setting *setting)
{
	if (setting->pmem_size) {
		pmem_pdata.start = setting->pmem_start;
		pmem_pdata.size = setting->pmem_size;
		platform_device_register(&pmem_device);
	}

	if (setting->pmem_adsp_size) {
		pmem_adsp_pdata.start = setting->pmem_adsp_start;
		pmem_adsp_pdata.size = setting->pmem_adsp_size;
		platform_device_register(&pmem_adsp_device);
	}

#if defined(CONFIG_MSM_HW3D)
	if (setting->pmem_gpu0_size && setting->pmem_gpu1_size) {
		struct resource *res;

		res = platform_get_resource_byname(&hw3d_device, IORESOURCE_MEM,
						   "smi");
		res->start = setting->pmem_gpu0_start;
		res->end = res->start + setting->pmem_gpu0_size - 1;

		res = platform_get_resource_byname(&hw3d_device, IORESOURCE_MEM,
						   "ebi");
		res->start = setting->pmem_gpu1_start;
		res->end = res->start + setting->pmem_gpu1_size - 1;
		platform_device_register(&hw3d_device);
	}
#endif

	if (setting->pmem_camera_size) {
		pmem_camera_pdata.start = setting->pmem_camera_start;
		pmem_camera_pdata.size = setting->pmem_camera_size;
		platform_device_register(&pmem_camera_device);
	}

	if (setting->ram_console_size) {
		ram_console_resource[0].start = setting->ram_console_start;
		ram_console_resource[0].end = setting->ram_console_start
			+ setting->ram_console_size - 1;
		platform_device_register(&ram_console_device);
	}

#if defined(CONFIG_GPU_MSM_KGSL)
	if (setting->kgsl_size) {
		msm_kgsl_resources[1].start = setting->kgsl_start;
		msm_kgsl_resources[1].end = setting->kgsl_start
			+ setting->kgsl_size - 1;
		kgsl_power_rail_mode(0);
		kgsl_power(true);
		platform_device_register(&msm_kgsl_device);
	}
#endif

#ifdef CONFIG_BUILD_CIQ
	if(setting->pmem_ciq_size) {
		pmem_ciq_pdata.start = setting->pmem_ciq_start;
		pmem_ciq_pdata.size = setting->pmem_ciq_size;
		platform_device_register(&pmem_ciq_device);
	}

	if(setting->pmem_ciq1_size) {
		pmem_ciq1_pdata.start = setting->pmem_ciq1_start;
		pmem_ciq1_pdata.size = setting->pmem_ciq1_size;
		platform_device_register(&pmem_ciq1_device);
	}

	if(setting->pmem_ciq2_size) {
		pmem_ciq2_pdata.start = setting->pmem_ciq2_start;
		pmem_ciq2_pdata.size = setting->pmem_ciq2_size;
		platform_device_register(&pmem_ciq2_device);
	}

	if(setting->pmem_ciq3_size) {
		pmem_ciq3_pdata.start = setting->pmem_ciq3_start;
		pmem_ciq3_pdata.size = setting->pmem_ciq3_size;
		platform_device_register(&pmem_ciq3_device);
	}
#endif
}

static struct platform_device *msm_serial_devices[] __initdata = {
	&msm_device_uart1,
	&msm_device_uart2,
	&msm_device_uart3,
	#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm1,
	&msm_device_uart_dm2,
	#endif
};

int __init msm_add_serial_devices(unsigned num)
{
	if (num > MSM_SERIAL_NUM)
		return -EINVAL;

	return platform_device_register(msm_serial_devices[num]);
}

#define ATAG_SMI 0x4d534D71
/* setup calls mach->fixup, then parse_tags, parse_cmdline
 * We need to setup meminfo in mach->fixup, so this function
 * will need to traverse each tag to find smi tag.
 */
int __init parse_tag_smi(const struct tag *tags)
{
	int smi_sz = 0, find = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_SMI) {
			printk(KERN_DEBUG "find the smi tag\n");
			find = 1;
			break;
		}
	}
	if (!find)
		return -1;

	printk(KERN_DEBUG "parse_tag_smi: smi size = %d\n", t->u.mem.size);
	smi_sz = t->u.mem.size;
	return smi_sz;
}
__tagtable(ATAG_SMI, parse_tag_smi);


#define ATAG_HWID 0x4d534D72
int __init parse_tag_hwid(const struct tag *tags)
{
	int hwid = 0, find = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_HWID) {
			printk(KERN_DEBUG "find the hwid tag\n");
			find = 1;
			break;
		}
	}

	if (find)
		hwid = t->u.revision.rev;
	printk(KERN_DEBUG "parse_tag_hwid: hwid = 0x%x\n", hwid);
	return hwid;
}
__tagtable(ATAG_HWID, parse_tag_hwid);

#define ATAG_MONODIE 0x4d534D76
static int mono_die;;
int __init parse_tag_monodie(const struct tag *tags)
{
	int find = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_MONODIE) {
			printk(KERN_DEBUG "find the flash id tag\n");
			find = 1;
			break;
		}
	}

	if (find)
		mono_die = t->u.revision.rev;
	printk(KERN_DEBUG "parse_tag_monodie: mono-die = 0x%x\n", mono_die);
	return mono_die;
}
__tagtable(ATAG_MONODIE, parse_tag_monodie);

int __init board_mcp_monodie(void)
{
	return mono_die;
}



static char *keycap_tag = NULL;
static int __init board_keycaps_tag(char *get_keypads)
{
	if(strlen(get_keypads))
		keycap_tag = get_keypads;
	else
		keycap_tag = NULL;
	return 1;
}
__setup("androidboot.keycaps=", board_keycaps_tag);

void board_get_keycaps_tag(char **ret_data)
{
	*ret_data = keycap_tag;
}
EXPORT_SYMBOL(board_get_keycaps_tag);

static char *cid_tag = NULL;
static int __init board_set_cid_tag(char *get_hboot_cid)
{
	if(strlen(get_hboot_cid))
		cid_tag = get_hboot_cid;
	else
		cid_tag = NULL;
	return 1;
}
__setup("androidboot.cid=", board_set_cid_tag);

void board_get_cid_tag(char **ret_data)
{
	*ret_data = cid_tag;
}
EXPORT_SYMBOL(board_get_cid_tag);

static int *batt_poweron_tag = NULL;
static int __init board_set_batt_poweron_tag(int *get_hboot_batt_poweron)
{
	if(strlen(get_hboot_batt_poweron))
		batt_poweron_tag = get_hboot_batt_poweron;
	else
		batt_poweron_tag = NULL;
	return 1;
}
__setup("androidboot.batt_poweron=", board_set_batt_poweron_tag);

void board_get_batt_poweron_tag(int **ret_data)
{
	*ret_data = batt_poweron_tag;
}
EXPORT_SYMBOL(board_get_batt_poweron_tag);

static char *carrier_tag = NULL;
static int __init board_set_carrier_tag(char *get_hboot_carrier)
{
	if(strlen(get_hboot_carrier))
		carrier_tag = get_hboot_carrier;
	else
		carrier_tag = NULL;
	return 1;
}
__setup("androidboot.carrier=", board_set_carrier_tag);

void board_get_carrier_tag(char **ret_data)
{
	*ret_data = carrier_tag;
}
EXPORT_SYMBOL(board_get_carrier_tag);

static char *mid_tag = NULL;
static int __init board_set_mid_tag(char *get_hboot_mid)
{
	if(strlen(get_hboot_mid))
		mid_tag = get_hboot_mid;
	else
		mid_tag = NULL;
	return 1;
}
__setup("androidboot.mid=", board_set_mid_tag);

void board_get_mid_tag(char **ret_data)
{
	*ret_data = mid_tag;
}
EXPORT_SYMBOL(board_get_mid_tag);
