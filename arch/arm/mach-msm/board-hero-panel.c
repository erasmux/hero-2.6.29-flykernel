/* linux/arch/arm/mach-msm7201a/board-hero-panel.c
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
#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/pmic.h>

#include "proc_comm.h"
#include "board-hero.h"
#include "devices.h"

#if 0
#define B(s...) printk(s)
#else
#define B(s...) do {} while(0)
#endif

#define SHARP_POWER 1

static struct led_trigger *hero_lcd_backlight;
static void hero_set_backlight(int on)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);
	
	if (on) {
		/* vsync back porch is about 17 ms */
		msleep(40);
		led_trigger_event(hero_lcd_backlight, LED_FULL);
	} else
		led_trigger_event(hero_lcd_backlight, LED_OFF);
}

#define MDDI_CLIENT_CORE_BASE  0x108000
#define LCD_CONTROL_BLOCK_BASE 0x110000
#define SPI_BLOCK_BASE         0x120000
#define I2C_BLOCK_BASE         0x130000
#define PWM_BLOCK_BASE         0x140000
#define GPIO_BLOCK_BASE        0x150000
#define SYSTEM_BLOCK1_BASE     0x160000
#define SYSTEM_BLOCK2_BASE     0x170000

#define	DPSUS       (MDDI_CLIENT_CORE_BASE|0x24)
#define	SYSCLKENA   (MDDI_CLIENT_CORE_BASE|0x2C)
#define	PWM0OFF	    (PWM_BLOCK_BASE|0x1C)

#define	MDDICAP0    (MDDI_CLIENT_CORE_BASE|0x00)
#define	MDDICAP1    (MDDI_CLIENT_CORE_BASE|0x04)
#define	MDDICAP2    (MDDI_CLIENT_CORE_BASE|0x08)
#define	MDDICAP3    (MDDI_CLIENT_CORE_BASE|0x0C)
#define	MDCAPCHG    (MDDI_CLIENT_CORE_BASE|0x10)
#define	MDCRCERC    (MDDI_CLIENT_CORE_BASE|0x14)
#define	TTBUSSEL    (MDDI_CLIENT_CORE_BASE|0x18)
#define	DPSET0      (MDDI_CLIENT_CORE_BASE|0x1C)
#define	DPSET1      (MDDI_CLIENT_CORE_BASE|0x20)
#define	DPSUS       (MDDI_CLIENT_CORE_BASE|0x24)
#define	DPRUN       (MDDI_CLIENT_CORE_BASE|0x28)
#define	SYSCKENA    (MDDI_CLIENT_CORE_BASE|0x2C)
#define	TESTMODE    (MDDI_CLIENT_CORE_BASE|0x30)
#define	FIFOMONI    (MDDI_CLIENT_CORE_BASE|0x34)
#define	INTMONI     (MDDI_CLIENT_CORE_BASE|0x38)
#define	MDIOBIST    (MDDI_CLIENT_CORE_BASE|0x3C)
#define	MDIOPSET    (MDDI_CLIENT_CORE_BASE|0x40)
#define	BITMAP0     (MDDI_CLIENT_CORE_BASE|0x44)
#define	BITMAP1     (MDDI_CLIENT_CORE_BASE|0x48)
#define	BITMAP2     (MDDI_CLIENT_CORE_BASE|0x4C)
#define	BITMAP3     (MDDI_CLIENT_CORE_BASE|0x50)
#define	BITMAP4     (MDDI_CLIENT_CORE_BASE|0x54)

#define	SRST        (LCD_CONTROL_BLOCK_BASE|0x00)
#define	PORT_ENB    (LCD_CONTROL_BLOCK_BASE|0x04)
#define	START       (LCD_CONTROL_BLOCK_BASE|0x08)
#define	PORT        (LCD_CONTROL_BLOCK_BASE|0x0C)
#define	CMN         (LCD_CONTROL_BLOCK_BASE|0x10)
#define	GAMMA       (LCD_CONTROL_BLOCK_BASE|0x14)
#define	INTFLG      (LCD_CONTROL_BLOCK_BASE|0x18)
#define	INTMSK      (LCD_CONTROL_BLOCK_BASE|0x1C)
#define	MPLFBUF     (LCD_CONTROL_BLOCK_BASE|0x20)
#define	HDE_LEFT    (LCD_CONTROL_BLOCK_BASE|0x24)
#define	VDE_TOP     (LCD_CONTROL_BLOCK_BASE|0x28)
#define	PXL         (LCD_CONTROL_BLOCK_BASE|0x30)
#define	HCYCLE      (LCD_CONTROL_BLOCK_BASE|0x34)
#define	HSW         (LCD_CONTROL_BLOCK_BASE|0x38)
#define	HDE_START   (LCD_CONTROL_BLOCK_BASE|0x3C)
#define	HDE_SIZE    (LCD_CONTROL_BLOCK_BASE|0x40)
#define	VCYCLE      (LCD_CONTROL_BLOCK_BASE|0x44)
#define	VSW         (LCD_CONTROL_BLOCK_BASE|0x48)
#define	VDE_START   (LCD_CONTROL_BLOCK_BASE|0x4C)
#define	VDE_SIZE    (LCD_CONTROL_BLOCK_BASE|0x50)
#define	WAKEUP      (LCD_CONTROL_BLOCK_BASE|0x54)
#define	WSYN_DLY    (LCD_CONTROL_BLOCK_BASE|0x58)
#define	REGENB      (LCD_CONTROL_BLOCK_BASE|0x5C)
#define	VSYNIF      (LCD_CONTROL_BLOCK_BASE|0x60)
#define	WRSTB       (LCD_CONTROL_BLOCK_BASE|0x64)
#define	RDSTB       (LCD_CONTROL_BLOCK_BASE|0x68)
#define	ASY_DATA    (LCD_CONTROL_BLOCK_BASE|0x6C)
#define	ASY_DATB    (LCD_CONTROL_BLOCK_BASE|0x70)
#define	ASY_DATC    (LCD_CONTROL_BLOCK_BASE|0x74)
#define	ASY_DATD    (LCD_CONTROL_BLOCK_BASE|0x78)
#define	ASY_DATE    (LCD_CONTROL_BLOCK_BASE|0x7C)
#define	ASY_DATF    (LCD_CONTROL_BLOCK_BASE|0x80)
#define	ASY_DATG    (LCD_CONTROL_BLOCK_BASE|0x84)
#define	ASY_DATH    (LCD_CONTROL_BLOCK_BASE|0x88)
#define	ASY_CMDSET  (LCD_CONTROL_BLOCK_BASE|0x8C)

#define	SSICTL      (SPI_BLOCK_BASE|0x00)
#define	SSITIME     (SPI_BLOCK_BASE|0x04)
#define	SSITX       (SPI_BLOCK_BASE|0x08)
#define	SSIRX       (SPI_BLOCK_BASE|0x0C)
#define	SSIINTC     (SPI_BLOCK_BASE|0x10)
#define	SSIINTS     (SPI_BLOCK_BASE|0x14)
#define	SSIDBG1     (SPI_BLOCK_BASE|0x18)
#define	SSIDBG2     (SPI_BLOCK_BASE|0x1C)
#define	SSIID       (SPI_BLOCK_BASE|0x20)

#define	WKREQ       (SYSTEM_BLOCK1_BASE|0x00)
#define	CLKENB      (SYSTEM_BLOCK1_BASE|0x04)
#define	DRAMPWR     (SYSTEM_BLOCK1_BASE|0x08)
#define	INTMASK     (SYSTEM_BLOCK1_BASE|0x0C)
#define	GPIOSEL     (SYSTEM_BLOCK2_BASE|0x00)

#define	GPIODATA    (GPIO_BLOCK_BASE|0x00)
#define	GPIODIR     (GPIO_BLOCK_BASE|0x04)
#define	GPIOIS      (GPIO_BLOCK_BASE|0x08)
#define	GPIOIBE     (GPIO_BLOCK_BASE|0x0C)
#define	GPIOIEV     (GPIO_BLOCK_BASE|0x10)
#define	GPIOIE      (GPIO_BLOCK_BASE|0x14)
#define	GPIORIS     (GPIO_BLOCK_BASE|0x18)
#define	GPIOMIS     (GPIO_BLOCK_BASE|0x1C)
#define	GPIOIC      (GPIO_BLOCK_BASE|0x20)
#define	GPIOOMS     (GPIO_BLOCK_BASE|0x24)
#define	GPIOPC      (GPIO_BLOCK_BASE|0x28)
#define	GPIOID      (GPIO_BLOCK_BASE|0x30)

#define SPI_WRITE(reg, val) \
	{ SSITX,        0x00010000 | (((reg) & 0xff) << 8) | ((val) & 0xff) }, \
	{ 0, 5 },

#define SPI_WRITE1(reg) \
	{ SSITX,        (reg) & 0xff }, \
	{ 0, 5 },

struct mddi_table {
	uint32_t reg;
	uint32_t value;
};
static struct mddi_table mddi_toshiba_init_table[] = {
	{ DPSET0,       0x09e90046 },
	{ DPSET1,       0x00000118 },
	{ DPSUS,        0x00000000 },
	{ DPRUN,        0x00000001 },
	{ 1,            14         }, /* msleep 14 */
	{ SYSCKENA,     0x00000001 },
	//{ CLKENB,       0x000000EF },
	{ CLKENB,       0x0000A1EF },  /*    # SYS.CLKENB  # Enable clocks for each module (without DCLK , i2cCLK) */
	//{ CLKENB,       0x000025CB }, /* Clock enable register */

	{ GPIODATA,     0x02000200 },  /*   # GPI .GPIODATA  # GPIO2(RESET_LCD_N) set to 0 , GPIO3(eDRAM_Power) set to 0 */
	{ GPIODIR,      0x000030D  },  /* 24D   # GPI .GPIODIR  # Select direction of GPIO port (0,2,3,6,9 output) */
	{ GPIOSEL,      0/*0x00000173*/},  /*   # SYS.GPIOSEL  # GPIO port multiplexing control */
	{ GPIOPC,       0x03C300C0 },  /*   # GPI .GPIOPC  # GPIO2,3 PD cut */
	{ WKREQ,        0x00000000 },  /*   # SYS.WKREQ  # Wake-up request event is VSYNC alignment */

	{ GPIOIBE,      0x000003FF },
	{ GPIOIS,       0x00000000 },
	{ GPIOIC,       0x000003FF },
	{ GPIOIE,       0x00000000 },

	{ GPIODATA,     0x00040004 },  /*   # GPI .GPIODATA  # eDRAM VD supply */
	{ 1,            1          }, /* msleep 1 */
	{ GPIODATA,     0x02040004 },  /*   # GPI .GPIODATA  # eDRAM VD supply */
	{ DRAMPWR,      0x00000001 }, /* eDRAM power */
};

static struct mddi_table mddi_toshiba_panel_init_table[] = {
	{ SRST,         0x00000003 }, /* FIFO/LCDC not reset */
	{ PORT_ENB,     0x00000001 }, /* Enable sync. Port */
	{ START,        0x00000000 }, /* To stop operation */
	//{ START,        0x00000001 }, /* To start operation */
	{ PORT,         0x00000004 }, /* Polarity of VS/HS/DE. */
	{ CMN,          0x00000000 },
	{ GAMMA,        0x00000000 }, /* No Gamma correction */
	{ INTFLG,       0x00000000 }, /* VSYNC interrupt flag clear/status */
	{ INTMSK,       0x00000000 }, /* VSYNC interrupt mask is off. */
	{ MPLFBUF,      0x00000000 }, /* Select frame buffer's base address. */
	{ HDE_LEFT,     0x00000000 }, /* The value of HDE_LEFT. */
	{ VDE_TOP,      0x00000000 }, /* The value of VDE_TPO. */
	{ PXL,          0x00000001 }, /* 1. RGB666 */
	                              /* 2. Data is valid from 1st frame of beginning. */
	{ HDE_START,    0x00000006 }, /* HDE_START= 14 PCLK */
	{ HDE_SIZE,     0x0000009F }, /* HDE_SIZE=320 PCLK */
	{ HSW,          0x00000004 }, /* HSW= 10 PCLK */
	{ VSW,          0x00000001 }, /* VSW=2 HCYCLE */
	{ VDE_START,    0x00000003 }, /* VDE_START=4 HCYCLE */
	{ VDE_SIZE,     0x000001DF }, /* VDE_SIZE=480 HCYCLE */
	{ WAKEUP,       0x000001e2 }, /* Wakeup position in VSYNC mode. */
	{ WSYN_DLY,     0x00000000 }, /* Wakeup position in VSIN mode. */
	{ REGENB,       0x00000001 }, /* Set 1 to enable to change the value of registers. */
	{ CLKENB,       0x000025CB }, /* Clock enable register */

	{ SSICTL,       0x00000170 }, /* SSI control register */
	{ SSITIME,      0x00000250 }, /* SSI timing control register */
	{ SSICTL,       0x00000172 }, /* SSI control register */
};


static struct mddi_table mddi_sharp_init_table[] = {
	{ VCYCLE,       0x000001eb },
	{ HCYCLE,       0x000000ae },
	{ REGENB,       0x00000001 }, /* Set 1 to enable to change the value of registers. */
	{ GPIODATA,     0x00040000 }, /* GPIO2 low */
	{ GPIODIR,      0x00000004 }, /* GPIO2 out */
	{ 1,            1          }, /* msleep 1 */
	{ GPIODATA,     0x00040004 }, /* GPIO2 high */
	{ 1,            10         }, /* msleep 10 */
	SPI_WRITE(0x5f, 0x01)
	SPI_WRITE1(0x11)
	{ 1,            200        }, /* msleep 200 */
	SPI_WRITE1(0x29)
	SPI_WRITE1(0xde)
	{ START,        0x00000001 }, /* To start operation */
};

static struct mddi_table mddi_sharp_deinit_table[] = {
	{ 1,            200        }, /* msleep 200 */
	SPI_WRITE(0x10, 0x1)
	{ 1,            100        }, /* msleep 100 */
	{ GPIODATA,     0x00040004 }, /* GPIO2 high */
	{ GPIODIR,      0x00000004 }, /* GPIO2 out */
	{ GPIODATA,     0x00040000 }, /* GPIO2 low */
	{ 1,            10         }, /* msleep 10 */
};

static struct mddi_table mddi_tpo_init_table[] = {
	{ VCYCLE,       0x000001e5 },
	{ HCYCLE,       0x000000ac },
	{ REGENB,       0x00000001 }, /* Set 1 to enable to change the value of registers. */
	{ 0,            20         }, /* udelay 20 */
	{ GPIODATA,     0x00000004 }, /* GPIO2 high */
	{ GPIODIR,      0x00000004 }, /* GPIO2 out */
	{ 0,            20         }, /* udelay 20 */

	SPI_WRITE(0x08, 0x01)
	{ 0,            500        }, /* udelay 500 */
	SPI_WRITE(0x08, 0x00)
	SPI_WRITE(0x02, 0x00)
	SPI_WRITE(0x03, 0x04)
	SPI_WRITE(0x04, 0x0e)
	SPI_WRITE(0x09, 0x02)
	SPI_WRITE(0x0b, 0x08)
	SPI_WRITE(0x0c, 0x53)
	SPI_WRITE(0x0d, 0x01)
	SPI_WRITE(0x0e, 0xe0)
	SPI_WRITE(0x0f, 0x01)
	SPI_WRITE(0x10, 0x58)
	SPI_WRITE(0x20, 0x1e)
	SPI_WRITE(0x21, 0x0a)
	SPI_WRITE(0x22, 0x0a)
	SPI_WRITE(0x23, 0x1e)
	SPI_WRITE(0x25, 0x32)
	SPI_WRITE(0x26, 0x00)
	SPI_WRITE(0x27, 0xac)
	SPI_WRITE(0x29, 0x06)
	SPI_WRITE(0x2a, 0xa4)
	SPI_WRITE(0x2b, 0x45)
	SPI_WRITE(0x2c, 0x45)
	SPI_WRITE(0x2d, 0x15)
	SPI_WRITE(0x2e, 0x5a)
	SPI_WRITE(0x2f, 0xff)
	SPI_WRITE(0x30, 0x6b)
	SPI_WRITE(0x31, 0x0d)
	SPI_WRITE(0x32, 0x48)
	SPI_WRITE(0x33, 0x82)
	SPI_WRITE(0x34, 0xbd)
	SPI_WRITE(0x35, 0xe7)
	SPI_WRITE(0x36, 0x18)
	SPI_WRITE(0x37, 0x94)
	SPI_WRITE(0x38, 0x01)
	SPI_WRITE(0x39, 0x5d)
	SPI_WRITE(0x3a, 0xae)
	SPI_WRITE(0x3b, 0xff)
	SPI_WRITE(0x07, 0x09)
	{ 0,            10         }, /* udelay 10 */
	{ START,        0x00000001 }, /* To start operation */
};

static struct mddi_table mddi_tpo_deinit_table[] = {
	SPI_WRITE(0x07, 0x19)
	{ START,        0x00000000 }, /* To stop operation */
	{ GPIODATA,     0x00040004 }, /* GPIO2 high */
	{ GPIODIR,      0x00000004 }, /* GPIO2 out */
	{ GPIODATA,     0x00040000 }, /* GPIO2 low */
	{ 0,            5        }, /* usleep 5 */
};


#define GPIOSEL_VWAKEINT (1U << 0)
#define INTMASK_VWAKEOUT (1U << 0)

static void hero_process_mddi_table(struct msm_mddi_client_data *client_data,
				     struct mddi_table *table, size_t count)
{
	int i;
	for(i = 0; i < count; i++) {
		uint32_t reg = table[i].reg;
		uint32_t value = table[i].value;

		if (reg == 0)
			udelay(value);
		else if (reg == 1)
			msleep(value);
		else
			client_data->remote_write(client_data, value, reg);
	}
}

static struct vreg *vreg_lcm_2v6;
static struct vreg *vreg_lcm_2v85;

#define GP_NS_REG (0x005c)
#define LCD_RSTz_ID1 58

static void 
hero_mddi_eid_power(struct msm_mddi_client_data *client_data, int on)
{
	unsigned id, on_off = 1;

	B(KERN_DEBUG "%s: power %s.\n", __func__, on ? "on" : "off");
	if (on) {
		on_off = 0;
		/* 2V6(pmic gp4) */
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcm_2v6);
		mdelay(1);

		/* 2V8(pmic rfrx2) */
		id = PM_VREG_PDOWN_RFRX2_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcm_2v85);
		mdelay(2);

		gpio_set_value(LCD_RSTz_ID1, 1);
		mdelay(15);
	} else {
		on_off = 1;
		mdelay(5);
		gpio_set_value(LCD_RSTz_ID1, 0);
		mdelay(3);

		/* 2V8(pmic rfrx2) */
		id = PM_VREG_PDOWN_RFRX2_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_disable(vreg_lcm_2v85);
		mdelay(1);

		/* 2V6(pmic gp4) */
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_disable(vreg_lcm_2v6);
	}
}

static void 
hero_mddi_sharp_power(struct msm_mddi_client_data *client_data, int on)
{
#if SHARP_POWER 	
	unsigned id, on_off;

	if(on) {
		writel(0xa06, MSM_CLK_CTL_BASE + GP_NS_REG);
		on_off = 0;
		/* 1V5 */
		gpio_set_value(HERO_GPIO_MDDI_1V5_EN, 1);
		msleep(5);
		/* 2V6(pmic gp4), 1V8 */
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcm_2v6);
		gpio_set_value(HERO_GPIO_MDDI_1V8_EN, 1);
		msleep(5);
		/* 2V85(pmic rfrx2) */
		id = PM_VREG_PDOWN_RFRX2_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcm_2v85);
		msleep(1);
		gpio_set_value(HERO_GPIO_MDDI_RST_N, 1);
		msleep(10);
	} else {
		writel(0x006, MSM_CLK_CTL_BASE + GP_NS_REG);
		on_off = 1;
		gpio_set_value(HERO_GPIO_MDDI_RST_N, 0);
		msleep(10);
		/* 2V85(pmic rfrx2) */
		id = PM_VREG_PDOWN_RFRX2_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcm_2v85);
		msleep(5);
		/* 2V6(pmic gp4), 1V8 */
		id = PM_VREG_PDOWN_AUX_ID;
		msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);
		vreg_enable(vreg_lcm_2v6);
		gpio_set_value(HERO_GPIO_MDDI_1V8_EN, 0);
		msleep(200);
		gpio_set_value(HERO_GPIO_MDDI_1V5_EN, 0);
	}
#endif	
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

static int hero_panel_detect(void)
{
	return panel_type;
}

static int mddi_toshiba_client_init(
		struct msm_mddi_bridge_platform_data *bridge_data,
		struct msm_mddi_client_data *client_data)
{
#if SHARP_POWER	
	int panel_id;

	client_data->auto_hibernate(client_data, 0);
	hero_process_mddi_table(client_data, mddi_toshiba_init_table,
				 ARRAY_SIZE(mddi_toshiba_init_table));
	client_data->auto_hibernate(client_data, 1);
	panel_id = (client_data->remote_read(client_data, GPIODATA) >> 4) & 3;
	if (panel_id > 1) {
		printk("unknown panel id at mddi_enable\n");
		return -1;
	}
#endif	
	return 0;
}

static int 
mddi_toshiba_client_uninit(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *cdata)
{
	return 0;
}

static int 
hero_panel_unblank(struct msm_mddi_bridge_platform_data *bridge_data,
		struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);
	hero_set_backlight(1);
	return 0;
}

static int 
hero_panel_blank(struct msm_mddi_bridge_platform_data *bridge_data,
		struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s: enter.\n", __func__);
	hero_set_backlight(0);
	return 0;
}

static int 
panel_sharp_unblank(struct msm_mddi_bridge_platform_data *bridge_data,
		struct msm_mddi_client_data *client_data)
{
#if SHARP_POWER       
	int panel_id, ret = 0;

	hero_set_backlight(0);
	client_data->auto_hibernate(client_data, 0);
	hero_process_mddi_table(client_data, mddi_toshiba_panel_init_table,
		ARRAY_SIZE(mddi_toshiba_panel_init_table));
	panel_id = (client_data->remote_read(client_data, GPIODATA) >> 4) & 3;
	switch(panel_id) {
	 case 0:
		B("init sharp panel\n");
		hero_process_mddi_table(client_data,
					 mddi_sharp_init_table,
					 ARRAY_SIZE(mddi_sharp_init_table));
		break;
	case 1:
		B("init tpo panel\n");
		hero_process_mddi_table(client_data,
					 mddi_tpo_init_table,
					 ARRAY_SIZE(mddi_tpo_init_table));
		break;
	default:
		B("unknown panel_id: %d\n", panel_id);
		ret = -1;
	};
	hero_set_backlight(1);
	client_data->auto_hibernate(client_data, 1);
	// reenable vsync
	client_data->remote_write(client_data, GPIOSEL_VWAKEINT,
				  GPIOSEL);
	client_data->remote_write(client_data, INTMASK_VWAKEOUT,
				  INTMASK);
	return ret;
#else
	return 0;
#endif
}

static int 
panel_sharp_blank(struct msm_mddi_bridge_platform_data *bridge,
		struct msm_mddi_client_data *client_data)
{
#if SHARP_POWER	
	int panel_id, ret = 0;

	panel_id = (client_data->remote_read(client_data, GPIODATA) >> 4) & 3;
	client_data->auto_hibernate(client_data, 0);
	switch(panel_id) {
	case 0:
		B("deinit sharp panel\n");
		hero_process_mddi_table(client_data,
					 mddi_sharp_deinit_table,
					 ARRAY_SIZE(mddi_sharp_deinit_table));
		break;
	case 1:
		B("deinit tpo panel\n");
		hero_process_mddi_table(client_data,
					 mddi_tpo_deinit_table,
					 ARRAY_SIZE(mddi_tpo_deinit_table));
		break;
	default:
		B("unknown panel_id: %d\n", panel_id);
		ret = -1;
	};
	client_data->auto_hibernate(client_data,1);
	hero_set_backlight(0);
	client_data->remote_write(client_data, 0, SYSCLKENA);
	client_data->remote_write(client_data, 1, DPSUS);

	return ret;
#else
	return 0;
#endif
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

	ret = gpio_request(HERO_GPIO_VSYNC, "vsync");
	if (ret)
		return ret;

	config = PCOM_GPIO_CFG(HERO_GPIO_VSYNC, 1, GPIO_INPUT,
			GPIO_PULL_DOWN, GPIO_2MA);
	ret = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
	if (ret)
		gpio_free(HERO_GPIO_VSYNC);
	return ret;
}

static struct msm_mddi_bridge_platform_data toshiba_client_data = {
	.init = mddi_toshiba_client_init,
	.uninit = mddi_toshiba_client_uninit,
	.blank = panel_sharp_blank,
	.unblank = panel_sharp_unblank,
	.fb_data = {
		.xres = 320,
		.yres = 480,
		.width = 45,
		.height = 68,
		.output_format = 0,
	},
};

static u8 pwm_eid[] = {8, 16, 34, 61, 96, 138, 167, 195, 227, 255};
static struct msm_mddi_bridge_platform_data eid_client_data = {
	.blank = hero_panel_blank,
	.unblank = hero_panel_unblank,
	.fb_data = {
		.xres = 320,
		.yres = 480,
		.width = 45,
		.height = 68,
		.output_format = 0,
	},
	.panel_conf = {
		.panel_id = PANEL_EID_24pin,
		.caps = 0x0,
		.pwm = pwm_eid,
	},
};

static struct resource resources_msm_fb[] = {
	{
		.start = MSM_FB_BASE,
		.end = MSM_FB_BASE + MSM_FB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct msm_mddi_platform_data hero_pdata = {
	.clk_rate = 122880000,
	.power_client = hero_mddi_sharp_power,
	.fixup = panel_eid_fixup,
	.fb_resource = resources_msm_fb,
	.num_clients = 2,
	.client_platform_data = {
		{
			.product_id = (0xd263 << 16 | 0),
			.name = "mddi_c_d263_0000",
			.id = 0,
			.client_data = &toshiba_client_data,
			.clk_rate = 0,
		},
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
 * So, we just detect panel here, setting different power function for each panel.
 * Then we did not have to detect panel in each time mddi_client_power or panel_power
 * is called.
 *
 * jay: Nov 20, 08'
 */
int __init hero_init_panel(void)
{
	int panel, rc;
	struct panel_data *panel_data = &eid_client_data.panel_conf;

	if(!machine_is_hero())
		return -1;
	
	B(KERN_INFO "%s: enter.\n", __func__);

	vreg_lcm_2v6 = vreg_get(0, "gp4");
	if (IS_ERR(vreg_lcm_2v6))
		return PTR_ERR(vreg_lcm_2v6);

	vreg_lcm_2v85 = vreg_get(0, "rfrx2");
	if (IS_ERR(vreg_lcm_2v85))
		return PTR_ERR(vreg_lcm_2v85);

	panel = hero_panel_detect();

	if (panel == PANEL_SHARP) {
		printk(KERN_INFO "%s: init sharp panel\n", __func__);
		hero_pdata.power_client = hero_mddi_sharp_power;
	} else if ((panel == PANEL_EID_24pin) ||
			(panel == PANEL_EID_40pin) ||
			(panel == PANEL_EIDII) ||
			(panel == PANEL_SAMSUNG)) {
		if (panel != PANEL_SAMSUNG)
			printk(KERN_INFO "init EID panel\n");
		else
			printk(KERN_INFO "init Samsung panel\n");
		hero_pdata.power_client = hero_mddi_eid_power;
		panel_data->panel_id = panel;

		/* Hero enable CABC criteria:
		 * engineer id > 0, board > XC */
		if (engineer_id || system_rev > 2) {
			panel_data->caps |= MSMFB_CAP_CABC;
		} else {
			printk(KERN_DEBUG "CABC will not work on older board, "
					"engineer_id = %d\n", engineer_id);
		}
	} else {
		printk(KERN_ERR "unknown panel type!\n");
		return -EIO;
	}

	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	rc = config_vsync();
	if (rc)
		return rc;

	msm_device_mddi0.dev.platform_data = &hero_pdata;
	rc = platform_device_register(&msm_device_mddi0);
	if (rc)
		return rc;
	
	led_trigger_register_simple("lcd-backlight-gate", &hero_lcd_backlight);
	if (IS_ERR(hero_lcd_backlight))
		printk(KERN_ERR "%s: backlight registration failed!\n", __func__);

	return 0;
}
device_initcall(hero_init_panel);

