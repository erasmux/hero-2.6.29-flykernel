/*
 * Copyright (C) 2008 HTC, Inc.
 * Author: Max Tsai <max_tsai@htc.com>
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




#ifndef SDIO_EMAPI_H
#define SDIO_EMAPI_H

#include <linux/ioctl.h>

/* TI 1251 chip info defined in board-trout */
#define VENDOR_ID 0x104c
#define DEVICE_ID 0x9066

/* Documentation/ioctl-number.txt */
/* the same with linux/wireless.h */
#define MAGIC			0x8B

#define EMAPI_IOC_GETMEM	_IOR(MAGIC, 1, unsigned long)
#define EMAPI_IOC_SETMEM	_IOW(MAGIC, 2, unsigned long)
#define EMAPI_IOC_GETMAC	_IOR(MAGIC, 3, unsigned long)
#define EMAPI_IOC_SETMAC	_IOW(MAGIC, 4, unsigned long)
#define EMAPI_IOC_SETPAR	_IOW(MAGIC, 5, unsigned long)
#define EMAPI_IOC_SETPAR2	_IOW(MAGIC, 6, unsigned long)
#define EMAPI_IOC_GETPAR	_IOR(MAGIC, 7, unsigned long)
#define EMAPI_IOC_GETEEPROM	_IOR(MAGIC, 8, unsigned long)
#define EMAPI_IOC_SETEEPROM	_IOW(MAGIC, 9, unsigned long)
#define EMAPI_IOC_TADDR2SADDR   _IOW(MAGIC, 10, unsigned long)
#define EMAPI_IOC_INIT		_IOW(MAGIC, 11, unsigned long)

#define EMAPI_IOC_MAXNR		11

#define MAX_NVS_SIZE    0x800U

struct sdio_request {
	unsigned long	addr;
	unsigned long	len;
	uint8_t		*buf;
};

/* wifi_nvs.c */
extern unsigned char *get_wifi_nvs_ram(void);
extern int wifi_power(int on);
extern void wifi_reset(int on);
extern void wifi_set_carddetect(int val);


void emapi_setfunc(struct sdio_func *);
struct sdio_func *emapi_getfunc(void);
void emapi_configure_partition(void);
int emapi_set_partition(unsigned long, unsigned long);
int emapi_set_partition2(unsigned long, unsigned long);
int emapi_get_partition(void);

int emapi_getmem(struct sdio_request *);
int emapi_setmem(struct sdio_request *);
int emapi_setmac(struct sdio_request *);
int emapi_getmac(struct sdio_request *);
int emapi_tnetwaddr2sdioaddr(unsigned long, unsigned long *);

int emapi_get_eeprom(unsigned char *);
int emapi_set_eeporm(void);

#endif
