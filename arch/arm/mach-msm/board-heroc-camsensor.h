/* linux/arch/arm/mach-msm/board-heroc-camsensor.h
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
 
#ifndef _HEROC_CAMSENSOR_
#define _HEROC_CAMSENSOR_

int heroc_s5k3e2fx_i2c_write(unsigned short waddr, unsigned short wdata);
int heroc_s5k3e2fx_i2c_read(unsigned short u_addr, unsigned short *pu_data);
int heroc_s5k3e2fx_probe_init(void *client);
void heroc_s5k3e2fx_sensor_deinit(void);
int heroc_s5k3e2fx_write_exposuregain(
	uint32_t mode, uint16_t line, uint16_t gain, 
	uint16_t linelengthpck, uint16_t framelengthlines);
int heroc_s5k3e2fx_set_pclk(int rt, int div_adj);
int heroc_s5k3e2fx_sensor_setting(unsigned long arg);
void heroc_s5k3e2fx_late_resume(struct early_suspend *handler);
int heroc_s5k3e2fx_resume(void *client);
int heroc_s5k3e2fx_suspend(void *client,pm_message_t mesg);
int heroc_s5k3e2fx_power_down(void);
int heroc_s5k3e2fx_power_up(void);
int heroc_msm_camio_clk_rate_set(int rate);
int heroc_msm_camio_clk_disable(int clk_type);
int heroc_msm_camio_clk_enable (int clk_type);
int heroc_s5k3e2fx_camif_pad_reg_reset(void);
int heroc_s5k3e2fx_camif_app_reset(void);
void heroc_s5k3e2fx_camif_reset2(void);
int heroc_camif_clk_select(int internal);

#endif
