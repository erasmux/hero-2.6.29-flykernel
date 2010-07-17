/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "mt9p012.h"
#include <linux/kernel.h>

/*Micron settings from Applications for lower power consumption.*/
struct reg_struct mt9p012_reg_pat[2] = {
	{			/* Preview */
	 /* vt_pix_clk_div          REG=0x0300 */
	 5,//6,			/* 5 */

	 /* vt_sys_clk_div          REG=0x0302 */
	 2,//1,

	 /* pre_pll_clk_div         REG=0x0304 */
	 2,

	 /* pll_multiplier          REG=0x0306 */
	 60,

	 /* op_pix_clk_div          REG=0x0308 */
	 10,//8,			/* 10 */

	 /* op_sys_clk_div          REG=0x030A */
	 1,

	 /* scale_m                 REG=0x0404 */
	 16,

	 /* row_speed               REG=0x3016 */
	 0x0111,

	 /* x_addr_start            REG=0x3004 */
	 8,

	 /* x_addr_end              REG=0x3008 */
	 2597,

	 /* y_addr_start            REG=0x3002 */
	 8,

	 /* y_addr_end              REG=0x3006 */
	 1949,

	 /* read_mode               REG=0x3040
	  * Preview 2x2 skipping *///Kevin modify 20100304
	 0x04C3,

	 /* x_output_size           REG=0x034C */
	 1296,

	 /* y_output_size           REG=0x034E */
	 972,

	 /* line_length_pck         REG=0x300C */
	 3400,//3784,

	 /* frame_length_lines      REG=0x300A */
	 1057,

	 /* coarse_integration_time REG=0x3012 */
	 16,

	 /* fine_integration_time   REG=0x3014 */
	 1764},
	{			/*Snapshot */
	 /* vt_pix_clk_div          REG=0x0300 */
	 5,//6,

	 /* vt_sys_clk_div          REG=0x0302 */
	 2,//1,

	 /* pre_pll_clk_div         REG=0x0304 */
	 2,

	 /* pll_multiplier          REG=0x0306
	  * 60 for 10fps snapshot */
	 60,

	 /* op_pix_clk_div          REG=0x0308 */
	 10,//8,

	 /* op_sys_clk_div          REG=0x030A */
	 1,

	 /* scale_m                 REG=0x0404 */
	 16,

	 /* row_speed               REG=0x3016 */
	 0x0111,

	 /* x_addr_start            REG=0x3004 */
	 8,

	 /* x_addr_end              REG=0x3008 */
	 2615,

	 /* y_addr_start            REG=0x3002 */
	 8,

	 /* y_addr_end              REG=0x3006 */
	 1967,

	 /* read_mode               REG=0x3040 */
	 0x0041,

	 /* x_output_size           REG=0x034C */
	 2608,

	 /* y_output_size           REG=0x034E */
	 1960,

	 /* line_length_pck         REG=0x300C */
	 3788,//3911,

	 /* frame_length_lines      REG=0x300A //10 fps snapshot */
	 2058,//2045,

	 /* coarse_integration_time REG=0x3012 */
	 16,

	 /* fine_integration_time   REG=0x3014 */
	 882}
};

struct mt9p012_i2c_reg_conf mt9p012_test_tbl[] = {
	{0x3044, 0x0544 & 0xFBFF},
	{0x30CA, 0x0004 | 0x0001},
	{0x30D4, 0x9020 & 0x7FFF},
	{0x31E0, 0x0003 & 0xFFFE},
	{0x3180, 0x91FF & 0x7FFF},
	{0x301A, (0x10CC | 0x8000) & 0xFFF7},
	{0x301E, 0x0000},
	{0x3780, 0x0000},
};

struct mt9p012_i2c_reg_conf mt9p012_lc_tbl[] = {
	/* [Lens shading 85 Percent TL84] *///Kevin modify 20100304
	/* P_RD_P0Q0 */
	{0x360A, 0x0210},
	/* P_RD_P0Q1 */
	{0x360C, 0x10CD},
	/* P_RD_P0Q2 */
	{0x360E, 0x13B1},
	/* P_RD_P0Q3 */
	{0x3610, 0x28CD},
	/* P_RD_P0Q4 */
	{0x3612, 0x87F1},
	/* P_RD_P1Q0 */
	{0x364A, 0xCB2C},
	/* P_RD_P1Q1 */
	{0x364C, 0x694C},
	/* P_RD_P1Q2 */
	{0x364E, 0x2A8E},
	/* P_RD_P1Q3 */
	{0x3650, 0xC3AE},
	/* P_RD_P1Q4 */
	{0x3652, 0xD20D},
	/* P_RD_P2Q0 */
	{0x368A, 0x2B51},
	/* P_RD_P2Q1 */
	{0x368C, 0x3BF0},
	/* P_RD_P2Q2 */
	{0x368E, 0xBCB2},
	/* P_RD_P2Q3 */
	{0x3690, 0xC992},
	/* P_RD_P2Q4 */
	{0x3692, 0x3411},
	/* P_RD_P3Q0 */
	{0x36CA, 0x7F2D},
	/* P_RD_P3Q1 */
	{0x36CC, 0xA42F},
	/* P_RD_P3Q2 */
	{0x36CE, 0x89D1},
	/* P_RD_P3Q3 */
	{0x36D0, 0x03D2},
	/* P_RD_P3Q4 */
	{0x36D2, 0x0192},
	/* P_RD_P4Q0 */
	{0x370A, 0x90F1},
	/* P_RD_P4Q1 */
	{0x370C, 0xCBB2},
	/* P_RD_P4Q2 */
	{0x370E, 0xB7F0},
	/* P_RD_P4Q3 */
	{0x3710, 0x6A54},
	/* P_RD_P4Q4 */
	{0x3712, 0x4A34},
	/* P_GR_P0Q0 */
	{0x3600, 0x0550},
	/* P_GR_P0Q1 */
	{0x3602, 0xDD2D},
	/* P_GR_P0Q2 */
	{0x3604, 0x0AF1},
	/* P_GR_P0Q3 */
	{0x3606, 0x7E2E},
	/* P_GR_P0Q4 */
	{0x3608, 0x9CB1},
	/* P_GR_P1Q0 */
	{0x3640, 0xDE8C},
	/* P_GR_P1Q1 */
	{0x3642, 0x0C0B},
	/* P_GR_P1Q2 */
	{0x3644, 0x1F6E},
	/* P_GR_P1Q3 */
	{0x3646, 0xEBEE},
	/* P_GR_P1Q4 */
	{0x3648, 0xE32D},
	/* P_GR_P2Q0 */
	{0x3680, 0x02F1},
	/* P_GR_P2Q1 */
	{0x3682, 0x5950},
	/* P_GR_P2Q2 */
	{0x3684, 0xB912},
	/* P_GR_P2Q3 */
	{0x3686, 0xABD1},
	/* P_GR_P2Q4 */
	{0x3688, 0x2511},
	/* P_GR_P3Q0 */
	{0x36C0, 0x590D},
	/* P_GR_P3Q1 */
	{0x36C2, 0xBC8F},
	/* P_GR_P3Q2 */
	{0x36C4, 0xC650},
	/* P_GR_P3Q3 */
	{0x36C6, 0x5491},
	/* P_GR_P3Q4 */
	{0x36C8, 0x01D2},
	/* P_GR_P4Q0 */
	{0x3700, 0xFFF0},
	/* P_GR_P4Q1 */
	{0x3702, 0xDF91},
	/* P_GR_P4Q2 */
	{0x3704, 0xDE72},
	/* P_GR_P4Q3 */
	{0x3706, 0x1D33},
	/* P_GR_P4Q4 */
	{0x3708, 0x2B55},
	/* P_BL_P0Q0 */
	{0x3614, 0x0190},
	/* P_BL_P0Q1 */
	{0x3616, 0xD7AD},
	/* P_BL_P0Q2 */
	{0x3618, 0x53B0},
	/* P_BL_P0Q3 */
	{0x361A, 0x5EAE},
	/* P_BL_P0Q4 */
	{0x361C, 0xDB70},
	/* P_BL_P1Q0 */
	{0x3654, 0xC7EB},
	/* P_BL_P1Q1 */
	{0x3656, 0x70AC},
	/* P_BL_P1Q2 */
	{0x3658, 0x0F6F},
	/* P_BL_P1Q3 */
	{0x365A, 0xB26F},
	/* P_BL_P1Q4 */
	{0x365C, 0x8BEE},
	/* P_BL_P2Q0 */
	{0x3694, 0x5690},
	/* P_BL_P2Q1 */
	{0x3696, 0x4810},
	/* P_BL_P2Q2 */
	{0x3698, 0xDAD2},
	/* P_BL_P2Q3 */
	{0x369A, 0xB5F1},
	/* P_BL_P2Q4 */
	{0x369C, 0x0973},
	/* P_BL_P3Q0 */
	{0x36D4, 0x60AE},
	/* P_BL_P3Q1 */
	{0x36D6, 0xE86F},
	/* P_BL_P3Q2 */
	{0x36D8, 0x8611},
	/* P_BL_P3Q3 */
	{0x36DA, 0x2172},
	/* P_BL_P3Q4 */
	{0x36DC, 0x73AF},
	/* P_BL_P4Q0 */
	{0x3714, 0xCB50},
	/* P_BL_P4Q1 */
	{0x3716, 0xCBF1},
	/* P_BL_P4Q2 */
	{0x3718, 0x6372},
	/* P_BL_P4Q3 */
	{0x371A, 0x28D3},
	/* P_BL_P4Q4 */
	{0x371C, 0xD18F},
	/* P_GB_P0Q0 */
	{0x361E, 0x00F0},
	/* P_GB_P0Q1 */
	{0x3620, 0x7F0C},
	/* P_GB_P0Q2 */
	{0x3622, 0x0571},
	/* P_GB_P0Q3 */
	{0x3624, 0x48E7},
	/* P_GB_P0Q4 */
	{0x3626, 0x82F1},
	/* P_GB_P1Q0 */
	{0x365E, 0x6D6A},
	/* P_GB_P1Q1 */
	{0x3660, 0x484B},
	/* P_GB_P1Q2 */
	{0x3662, 0x43CE},
	/* P_GB_P1Q3 */
	{0x3664, 0x9D6E},
	/* P_GB_P1Q4 */
	{0x3666, 0x026A},
	/* P_GB_P2Q0 */
	{0x369E, 0x0211},
	/* P_GB_P2Q1 */
	{0x36A0, 0x616F},
	/* P_GB_P2Q2 */
	{0x36A2, 0xC4F2},
	/* P_GB_P2Q3 */
	{0x36A4, 0xFD71},
	/* P_GB_P2Q4 */
	{0x36A6, 0x6E11},
	/* P_GB_P3Q0 */
	{0x36DE, 0x51AE},
	/* P_GB_P3Q1 */
	{0x36E0, 0xEA4E},
	/* P_GB_P3Q2 */
	{0x36E2, 0xF7D0},
	/* P_GB_P3Q3 */
	{0x36E4, 0x35D1},
	/* P_GB_P3Q4 */
	{0x36E6, 0x00D1},
	/* P_GB_P4Q0 */
	{0x371E, 0xDD50},
	/* P_GB_P4Q1 */
	{0x3720, 0x8CF2},
	/* P_GB_P4Q2 */
	{0x3722, 0xC951},
	/* P_GB_P4Q3 */
	{0x3724, 0x23B4},
	/* P_GB_P4Q4 */
	{0x3726, 0x6C74},
	/* POLY_ORIGIN_C */
	{0x3782, 0x04F4},
	/* POLY_ORIGIN_R  */
	{0x3784, 0x03C8},
	/* POLY_SC_ENABLE */
	{0x3780, 0x8000},
};

/* rolloff table for illuminant A *///Kevin modify 20100304
struct mt9p012_i2c_reg_conf mt9p012_rolloff_tbl[] = {
	/* P_RD_P0Q0 */
	{0x360A, 0x0210},
	/* P_RD_P0Q1 */
	{0x360C, 0x10CD},
	/* P_RD_P0Q2 */
	{0x360E, 0x13B1},
	/* P_RD_P0Q3 */
	{0x3610, 0x28CD},
	/* P_RD_P0Q4 */
	{0x3612, 0x87F1},
	/* P_RD_P1Q0 */
	{0x364A, 0xCB2C},
	/* P_RD_P1Q1 */
	{0x364C, 0x694C},
	/* P_RD_P1Q2 */
	{0x364E, 0x2A8E},
	/* P_RD_P1Q3 */
	{0x3650, 0xC3AE},
	/* P_RD_P1Q4 */
	{0x3652, 0xD20D},
	/* P_RD_P2Q0 */
	{0x368A, 0x2B51},
	/* P_RD_P2Q1 */
	{0x368C, 0x3BF0},
	/* P_RD_P2Q2 */
	{0x368E, 0xBCB2},
	/* P_RD_P2Q3 */
	{0x3690, 0xC992},
	/* P_RD_P2Q4 */
	{0x3692, 0x3411},
	/* P_RD_P3Q0 */
	{0x36CA, 0x7F2D},
	/* P_RD_P3Q1 */
	{0x36CC, 0xA42F},
	/* P_RD_P3Q2 */
	{0x36CE, 0x89D1},
	/* P_RD_P3Q3 */
	{0x36D0, 0x03D2},
	/* P_RD_P3Q4 */
	{0x36D2, 0x0192},
	/* P_RD_P4Q0 */
	{0x370A, 0x90F1},
	/* P_RD_P4Q1 */
	{0x370C, 0xCBB2},
	/* P_RD_P4Q2 */
	{0x370E, 0xB7F0},
	/* P_RD_P4Q3 */
	{0x3710, 0x6A54},
	/* P_RD_P4Q4 */
	{0x3712, 0x4A34},
	/* P_GR_P0Q0 */
	{0x3600, 0x0550},
	/* P_GR_P0Q1 */
	{0x3602, 0xDD2D},
	/* P_GR_P0Q2 */
	{0x3604, 0x0AF1},
	/* P_GR_P0Q3 */
	{0x3606, 0x7E2E},
	/* P_GR_P0Q4 */
	{0x3608, 0x9CB1},
	/* P_GR_P1Q0 */
	{0x3640, 0xDE8C},
	/* P_GR_P1Q1 */
	{0x3642, 0x0C0B},
	/* P_GR_P1Q2 */
	{0x3644, 0x1F6E},
	/* P_GR_P1Q3 */
	{0x3646, 0xEBEE},
	/* P_GR_P1Q4 */
	{0x3648, 0xE32D},
	/* P_GR_P2Q0 */
	{0x3680, 0x02F1},
	/* P_GR_P2Q1 */
	{0x3682, 0x5950},
	/* P_GR_P2Q2 */
	{0x3684, 0xB912},
	/* P_GR_P2Q3 */
	{0x3686, 0xABD1},
	/* P_GR_P2Q4 */
	{0x3688, 0x2511},
	/* P_GR_P3Q0 */
	{0x36C0, 0x590D},
	/* P_GR_P3Q1 */
	{0x36C2, 0xBC8F},
	/* P_GR_P3Q2 */
	{0x36C4, 0xC650},
	/* P_GR_P3Q3 */
	{0x36C6, 0x5491},
	/* P_GR_P3Q4 */
	{0x36C8, 0x01D2},
	/* P_GR_P4Q0 */
	{0x3700, 0xFFF0},
	/* P_GR_P4Q1 */
	{0x3702, 0xDF91},
	/* P_GR_P4Q2 */
	{0x3704, 0xDE72},
	/* P_GR_P4Q3 */
	{0x3706, 0x1D33},
	/* P_GR_P4Q4 */
	{0x3708, 0x2B55},
	/* P_BL_P0Q0 */
	{0x3614, 0x0190},
	/* P_BL_P0Q1 */
	{0x3616, 0xD7AD},
	/* P_BL_P0Q2 */
	{0x3618, 0x53B0},
	/* P_BL_P0Q3 */
	{0x361A, 0x5EAE},
	/* P_BL_P0Q4 */
	{0x361C, 0xDB70},
	/* P_BL_P1Q0 */
	{0x3654, 0xC7EB},
	/* P_BL_P1Q1 */
	{0x3656, 0x70AC},
	/* P_BL_P1Q2 */
	{0x3658, 0x0F6F},
	/* P_BL_P1Q3 */
	{0x365A, 0xB26F},
	/* P_BL_P1Q4 */
	{0x365C, 0x8BEE},
	/* P_BL_P2Q0 */
	{0x3694, 0x5690},
	/* P_BL_P2Q1 */
	{0x3696, 0x4810},
	/* P_BL_P2Q2 */
	{0x3698, 0xDAD2},
	/* P_BL_P2Q3 */
	{0x369A, 0xB5F1},
	/* P_BL_P2Q4 */
	{0x369C, 0x0973},
	/* P_BL_P3Q0 */
	{0x36D4, 0x60AE},
	/* P_BL_P3Q1 */
	{0x36D6, 0xE86F},
	/* P_BL_P3Q2 */
	{0x36D8, 0x8611},
	/* P_BL_P3Q3 */
	{0x36DA, 0x2172},
	/* P_BL_P3Q4 */
	{0x36DC, 0x73AF},
	/* P_BL_P4Q0 */
	{0x3714, 0xCB50},
	/* P_BL_P4Q1 */
	{0x3716, 0xCBF1},
	/* P_BL_P4Q2 */
	{0x3718, 0x6372},
	/* P_BL_P4Q3 */
	{0x371A, 0x28D3},
	/* P_BL_P4Q4 */
	{0x371C, 0xD18F},
	/* P_GB_P0Q0 */
	{0x361E, 0x00F0},
	/* P_GB_P0Q1 */
	{0x3620, 0x7F0C},
	/* P_GB_P0Q2 */
	{0x3622, 0x0571},
	/* P_GB_P0Q3 */
	{0x3624, 0x48E7},
	/* P_GB_P0Q4 */
	{0x3626, 0x82F1},
	/* P_GB_P1Q0 */
	{0x365E, 0x6D6A},
	/* P_GB_P1Q1 */
	{0x3660, 0x484B},
	/* P_GB_P1Q2 */
	{0x3662, 0x43CE},
	/* P_GB_P1Q3 */
	{0x3664, 0x9D6E},
	/* P_GB_P1Q4 */
	{0x3666, 0x026A},
	/* P_GB_P2Q0 */
	{0x369E, 0x0211},
	/* P_GB_P2Q1 */
	{0x36A0, 0x616F},
	/* P_GB_P2Q2 */
	{0x36A2, 0xC4F2},
	/* P_GB_P2Q3 */
	{0x36A4, 0xFD71},
	/* P_GB_P2Q4 */
	{0x36A6, 0x6E11},
	/* P_GB_P3Q0 */
	{0x36DE, 0x51AE},
	/* P_GB_P3Q1 */
	{0x36E0, 0xEA4E},
	/* P_GB_P3Q2 */
	{0x36E2, 0xF7D0},
	/* P_GB_P3Q3 */
	{0x36E4, 0x35D1},
	/* P_GB_P3Q4 */
	{0x36E6, 0x00D1},
	/* P_GB_P4Q0 */
	{0x371E, 0xDD50},
	/* P_GB_P4Q1 */
	{0x3720, 0x8CF2},
	/* P_GB_P4Q2 */
	{0x3722, 0xC951},
	/* P_GB_P4Q3 */
	{0x3724, 0x23B4},
	/* P_GB_P4Q4 */
	{0x3726, 0x6C74},
	/* POLY_ORIGIN_C */
	{0x3782, 0x04F4},
	/* POLY_ORIGIN_R  */
	{0x3784, 0x03C8},
	/* POLY_SC_ENABLE */
	{0x3780, 0x8000},
};

struct mt9p012_reg mt9p012_regs = {
	.reg_pat = &mt9p012_reg_pat[0],
	.reg_pat_size = ARRAY_SIZE(mt9p012_reg_pat),
	.ttbl = &mt9p012_test_tbl[0],
	.ttbl_size = ARRAY_SIZE(mt9p012_test_tbl),
	.lctbl = &mt9p012_lc_tbl[0],
	.lctbl_size = ARRAY_SIZE(mt9p012_lc_tbl),
	.rftbl = &mt9p012_rolloff_tbl[0],
	.rftbl_size = ARRAY_SIZE(mt9p012_rolloff_tbl)
};
