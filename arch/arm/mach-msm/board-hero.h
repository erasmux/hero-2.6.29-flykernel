/* linux/arch/arm/mach-msm/board-hero.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_HERO_H
#define __ARCH_ARM_MACH_MSM_BOARD_HERO_H

#include <mach/board.h>

#define MSM_LINUX_BASE         0x19200000
#define MSM_LINUX_SIZE         0xC600000

#define MSM_PMEM_GPU0_BASE      0x00000000
#define MSM_PMEM_GPU0_SIZE      0x00700000

#define MSM_FB_BASE             0x00700000
#define MSM_FB_SIZE             0x9b000

#define MSM_RAM_CONSOLE_BASE    0x007A0000
#define MSM_RAM_CONSOLE_SIZE    128 * SZ_1K

#define MSM_PMEM_GPU1_BASE      0x25800000
#define MSM_PMEM_GPU1_SIZE      0x800000

#define MSM_PMEM_MDP_BASE       0x26000000
#define MSM_PMEM_MDP_SIZE       0x800000

#define MSM_PMEM_ADSP_BASE      0x26800000
#define MSM_PMEM_ADSP_SIZE      0x800000

#define MSM_PMEM_CAMERA_BASE	0x27000000
#define MSM_PMEM_CAMERA_SIZE    0x1000000

#define DECLARE_MSM_IOMAP
#include <mach/msm_iomap.h>

#define HERO_POWER_KEY                  (20)
#define HERO_GPIO_PS_HOLD               (25)
#define HERO_GPIO_MDDI_1V8_EN           (26)
#define HERO_GPIO_UP_INT_N		(27)
#define HERO_GPIO_COMPASS_INT_N   (36)
#define HERO_GPIO_SDMC_CD_N             (38)
#define HERO_GPIO_GSENSOR_INT_N         (49)

/* BT */
#define HERO_GPIO_UART1_RTS             (43)
#define HERO_GPIO_UART1_CTS             (44)
#define HERO_GPIO_UART1_RX              (45)
#define HERO_GPIO_UART1_TX              (46)
#define HERO_GPIO_WB_SHUT_DOWN_N        (101)

#define HERO_GPIO_I2C_CLK			(60)
#define HERO_GPIO_I2C_DAT			(61)
#define HERO_GPIO_UP_RESET_N            (76)
#define HERO_GPIO_COMPASS_INT_N_XAXB   (83)/*for XA,XB*/
#define HERO_GPIO_COMPASS_RST_N         (84)
#define HERO_PROJECT_NAME        "hero"
#define HERO_LAYOUTS			{ \
			{ {  0,  1, 0}, {-1,  0, 0}, {0, 0, 1} }, \
			{ {  0, -1, 0}, {-1,  0, 0}, {0, 0, 1} }, \
			{ { -1,  0, 0}, { 0, -1, 0}, {0, 0, 1} }, \
			{ {  1,  0, 0}, { 0,  0, 1}, {0, 1, 0} }  \
								}
#define HERO_GPIO_MDDI_BRIDGE_ID        (85)
#define HERO_GPIO_TP_ATT_N              (90)
#define HERO_GPIO_VCM_PWDN              (91)
#define HERO_GPIO_CAM_RST_N             (92)
#define HERO_GPIO_EXT_3V_EN             (93)
#define HERO_GPIO_UP_INT_N_XAXB              (94) /*for XA,XB*/
#define HERO_GPIO_MDDI_1V5_EN           (98)
#define HERO_GPIO_MDDI_RST_N            (99)
#define HERO_GPIO_USB_PHY_RST_N         (100)
#define HERO_GPIO_WIFI_EN               (102)
#define HERO_CAM_PWDN                   (107)
#define HERO_TP_LS_EN                   (108)
#define HERO_GPIO_TP_EN                 (109)

#define HERO_GPIO_TO_INT(x)             (x+64)/*from gpio_to_irq*/

/* JogBall, exist in XC */
#define HERO_GPIO_JOGBALL_EN		(98)
#define HERO_GPIO_JOGBALL_UP_0		(94)
#define HERO_GPIO_JOGBALL_LEFT_0	(39)
#define HERO_GPIO_JOGBALL_DOWN_0	(83)
#define HERO_GPIO_JOGBALL_RIGHT_0	(37)

/* H2W */
#define HERO_GPIO_CABLE_IN1_XAXB        (18)
#define HERO_GPIO_CABLE_IN1             (49)
#define HERO_GPIO_CABLE_IN2             (31)
#define HERO_GPIO_UART3_RX              (86)
#define HERO_GPIO_UART3_TX              (87)
#define HERO_GPIO_H2W_DATA              (86)
#define HERO_GPIO_H2W_CLK               (87)
#define HERO_GPIO_HEADSET_MIC           (17)
#define HERO_GPIO_AUD_EXTMIC_SEL        (82)

#define HERO_GPIO_VSYNC			(97)

int hero_init_mmc(unsigned int sys_rev);
void config_hero_camera_on_gpios(void);
void config_hero_camera_off_gpios(void);
unsigned int camera_is_micron_5M(void);
unsigned int hero_get_hwid(void);
unsigned int hero_get_skuid(void);
unsigned int hero_get_engineerid(void);
#endif /* GUARD */
