#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/delay.h>

#include <linux/scatterlist.h> // maxtsai for cmd53 debug

#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>

#include "sdio_emapi.h"

static struct sdio_func *emapi_func = NULL;
static unsigned long g_tnetw_start_addr;
static unsigned long g_tnetw_end_addr;

#define SDIO_FIRST_MEM_PARTITION_START			0
#define SDIO_FIRST_MEM_PARTITION_SIZE			0x16800
#define SDIO_SECOND_MEM_PARTITION_START			0x16800
#define SDIO_SECOND_MEM_PARTITION_SIZE			0x16800
#define SDIO_THIRD_MEM_PARTITION_START			0x30000
#define SDIO_THIRD_MEM_PARTITION_SIZE			0x16800
#define SDIO_MAC_REG_PARTITION_START			0x300000
#define SDIO_MAC_REG_PARTITION_SIZE			0x8800
#define SDIO_FIRST_PHY_REG_PARTITION_START		0x3C0000
#define SDIO_SECOND_PHY_REG_PARTITION_START		0x3D0000
#define SDIO_PHY_REG_PARTITION_SIZE			0x10000
#define SDIO_LOWEST_REGISTER_ADDR			0x1FFC0
#define SDIO_HIGHEST_REGISTER_ADDR			0x1FFFF
#define SDIO_FIRST_PARTITION_SIZE			0x16800

/* restore sdio func */
void emapi_setfunc(struct sdio_func *func)
{
   	emapi_func = func;
}

struct sdio_func *emapi_getfunc(void)
{
   	return emapi_func;
}

/* sync with Dmitry's implementation +++*/
static int read_direct(struct sdio_func *func, unsigned char *buf, 
                                  unsigned long addr, unsigned len)
{
	unsigned i;
	int rc0, rc = 0;

	for (i=0; i < len ;i++,addr++) {
		*buf++ = (unsigned char)sdio_readb(func, addr, &rc0);
		if( rc0 != 0 ) 
			rc = rc0;
	}   
	return rc; 
}
static int write_direct(struct sdio_func *func, unsigned long addr,
			uint8_t *buf, unsigned len)
{
	unsigned i;
	int rc0, rc = 0;
	
	//printk("--- %s: %x\n", __FUNCTION__, addr);
		
	for (i = 0; i < len; i++, addr++) {
		sdio_writeb(func, *buf++, addr, &rc0);
		udelay(100);
		if( rc0 != 0 ) 
			rc = rc0;
	}   
	if (rc != 0)
		printk("%s: %d\n", __func__, rc);
	return rc; 
}
/* sync with Dmitry's implementation ---*/

static int emapi_sdio_write_register(struct sdio_request *req)
{
   	int retries = 5;
	int rc = 0;
	struct sdio_func *func;

	//printk("%s\n");

	func = emapi_getfunc();
	if (!func)
	   	return -EIO;
	while(retries) {
		rc = write_direct(func, req->addr, req->buf, req->len);
		if (rc) {
		   	printk(KERN_ERR "%s - write operation failed (%d)\n", 
			      		__FUNCTION__, 6-retries);
			retries --;
			continue;
		}
		/*{
			int i;
			printk("%s\t0x%08x\t", __func__, req->addr);
			for (i = 0; i < req->len; i++)
				printk("%02x", req->buf[i]);
			printk("\n");
		}*/
		return 0;
	}
	printk(KERN_ERR "%s - give up\n", __FUNCTION__);
	return -EIO;	
}

static int emapi_sdio_read_register(struct sdio_request *req)
{
   	int retries = 5;
	int rc = 0;
   	struct sdio_func *func;
       
	func = emapi_getfunc();
	if (!func)
	   	return -EIO;
	while(retries) {
		rc = read_direct(func, req->buf, req->addr, req->len);
		if (rc) {
		   	printk(KERN_ERR "%s - read operation failed (%d)\n", 
			      		__FUNCTION__, 6-retries);
			retries --;
			continue;
		}
		//printk("%s\t0x%08x\n", __func__, req->addr);
		return 0;
	}
	printk(KERN_ERR "%s - give up\n", __FUNCTION__);
	return -EIO;	
}



static int emapi_sdio_write(struct sdio_request *req)
{
   	int retries = 5;
	int rc = 0;
	struct sdio_func *func;

	func = emapi_getfunc();
	if (!func)
	   	return -EIO;
	while(retries) {
	   	if (retries > 2) {
		   	rc = sdio_memcpy_toio(func, req->addr, req->buf, req->len);
		}
		else 
		   	rc = write_direct(func, req->addr, req->buf, req->len);
		if (rc) {
		   	printk(KERN_ERR "%s - write operation failed (%d)(rc=%d)\n", 
			      		__FUNCTION__, 6-retries, rc);
			retries --;
			continue;
		}
		/*{
			int i;
			printk("%s\t0x%08x\t", __func__, req->addr);
			for (i = 0; i < req->len; i++)
				printk("%02x", req->buf[i]);
			printk("\n");
		}*/
		return 0;
	}
	printk(KERN_ERR "%s - give up\n", __FUNCTION__);
	return -EIO;	
}

static int emapi_sdio_read(struct sdio_request *req)
{
   	int retries = 5;
	int rc = 0;
   	struct sdio_func *func;
       
	func = emapi_getfunc();
	if (!func)
	   	return -EIO;
	while(retries) {
	   	if (retries > 2)
		   	rc = sdio_memcpy_fromio(func, req->buf, req->addr, req->len);
		else 
		   	rc = read_direct(func, req->buf, req->addr, req->len);
		if (rc) {
		   	printk(KERN_ERR "%s - read operation failed (%d)\n", 
			      		__FUNCTION__, 6-retries);
			retries --;
			continue;
		}
		/*{
			int i;
			printk("%s\t0x%08x\t", __func__, req->addr);
			for (i = 0; i < req->len; i++)
				printk("%02x", req->buf[i]);
			printk("\n");
		}*/
		return 0;
	}
	printk(KERN_ERR "%s - give up\n", __FUNCTION__);
	return -EIO;	
}


/***********************************************************
	exported func since here  
************************************************************/
int emapi_set_partition(unsigned long start_addr, unsigned long size) 
{
	struct sdio_func *func;
	int rc = 0, rc0;

	printk("emapi set partition: \t%lx\t%lx\n", start_addr, size);

	func = emapi_getfunc();
	if (!func)
	   	return -EIO;

	//regardless the actual parameter pass in
	//Mt: 'cos all mem partition sizes are the same.
	size = SDIO_FIRST_PARTITION_SIZE;

	/* Set size */
	rc0 = write_direct(func, 0x1ffc0, (uint8_t *) &size, 4); 
	if (rc0)
	   	rc = rc0;
	/* Set offset */
	rc0 = write_direct(func, 0x1ffc4, (uint8_t *) &start_addr, 4); 
	if (rc0)
	   	rc = rc0;

	g_tnetw_start_addr = start_addr; g_tnetw_end_addr = start_addr + size;

	return rc;
}

int emapi_set_partition2(unsigned long start_addr, unsigned long size)
{
	struct sdio_func *func; int rc = 0, rc0;
	
	printk("emapi set partition2:\t%lx\t%lx\n", start_addr, size);

	func = emapi_getfunc(); if (!func) return -EIO;

	/* Set size */
	rc0 = write_direct(func, 0x1ffc8, (uint8_t *) &size, 4); if (rc0) rc = rc0;
	/* Set offset */
	rc0 = write_direct(func, 0x1ffcc, (uint8_t *) &start_addr, 4); if (rc0) rc = rc0;
	return rc;
}

int emapi_get_partition(void)
{
	struct sdio_func *func; int rc = 0;
	unsigned char buf[4];

	func = emapi_getfunc();
	rc = read_direct(func, buf, 0x1ffc0, 4);
	printk("%s(%s): 0x%02x", __FUNCTION__, rc? "FAIL" : "OK", buf[0]);
	printk("%02x%02x%02x\n", buf[1], buf[2], buf[3]);
	return 0;
}

int emapi_tnetwaddr2sdioaddr(unsigned long in_tnetw_address, unsigned long *out_sdio_address)
{
	int retval = 0;

	/* Configure MEM 3th partition 0x30000-0x46800 */
	if (in_tnetw_address >= SDIO_THIRD_MEM_PARTITION_START) {
		//printk("%s: Configure MEM 2nd part 0x30000-0x46800\n", __func__);
		if (g_tnetw_start_addr != SDIO_THIRD_MEM_PARTITION_START)
			retval = emapi_set_partition(SDIO_THIRD_MEM_PARTITION_START, SDIO_THIRD_MEM_PARTITION_SIZE);
	}
	/* Configure MEM 2nd partition 0x10000-0x22FFF */
	else if ((in_tnetw_address >= SDIO_SECOND_MEM_PARTITION_START) &&
			(in_tnetw_address < SDIO_THIRD_MEM_PARTITION_START)) {
		/*
		printk("%s: Configure MEM 2nd part 0x10000-0x22FFF, 0x%08x, 0x%08x\n",
				__FUNCTION__,
				in_tnetw_address,
				g_tnetw_start_addr);
		*/
		if (g_tnetw_start_addr != SDIO_SECOND_MEM_PARTITION_START) {
			retval = emapi_set_partition(SDIO_SECOND_MEM_PARTITION_START, SDIO_SECOND_MEM_PARTITION_SIZE);
		}
	}
	/* Configure MEM 1st partition 0-0x1FFC0 */
	else if (g_tnetw_start_addr != SDIO_FIRST_MEM_PARTITION_START) {
		//printk("%s: Configure MEM 1st part 0-0x1FFC0\n", __FUNCTION__);
		retval = emapi_set_partition(SDIO_FIRST_MEM_PARTITION_START,
				SDIO_FIRST_MEM_PARTITION_SIZE); }

	(*out_sdio_address) = in_tnetw_address - g_tnetw_start_addr;
	return retval;
}

void emapi_configure_partition(void)
{
	emapi_set_partition(SDIO_FIRST_MEM_PARTITION_START, SDIO_FIRST_MEM_PARTITION_SIZE);
	emapi_set_partition2(SDIO_MAC_REG_PARTITION_START, SDIO_MAC_REG_PARTITION_SIZE);
}


int emapi_getmac(struct sdio_request *req)
{
	int rc = 0; uint8_t *buf;
	struct sdio_request req1;

	buf = kmalloc(req->len, GFP_ATOMIC);
	if (!buf)
		return -ENOMEM;
	memset(buf, 0, req->len);
	req1.addr = req->addr;
	req1.len = req->len;
	req1.buf = buf;
	rc = emapi_sdio_read_register(&req1);
	if (rc < 0)
		goto rlt1;

	/*
	printk("%s\t%08x\t%02x%02x%02x%02x\n", __FUNCTION__, req1.addr +
	0x300000 - 0x16800, req1.buf[0], req1.buf[1], req1.buf[2],
	req1.buf[3]);
	*/

	if (copy_to_user((uint8_t*)req->buf, buf, req->len)) {
		rc = -EFAULT;
		goto rlt1;
	}
rlt1:
	kfree(buf);
	return rc;
}


int emapi_setmac(struct sdio_request *req)
{
	int rc = 0; uint8_t *buf;
	struct sdio_request req1;

	buf = kmalloc(req->len, GFP_ATOMIC);
	if (!buf)
		return -ENOMEM;
	memset(buf, 0, req->len);
	if(copy_from_user(buf, req->buf, req->len)) {
		rc = -EFAULT;
		goto rtl1;
	}
	req1.addr = req->addr;
	req1.len = req->len;
	req1.buf = buf;
	rc = emapi_sdio_write_register(&req1);
	/*
	printk("%s\t%08x\t%02x%02x%02x%02x\n", __FUNCTION__, req1.addr +
	0x300000 - 0x16800, req1.buf[0], req1.buf[1], req1.buf[2],
	req1.buf[3]);
	*/
rtl1:
	kfree(buf);
	return rc;
}


int emapi_getmem(struct sdio_request *req)
{
	int rc = 0;
	uint8_t *buf;
	struct sdio_request req1;

	buf = kmalloc(req->len, GFP_ATOMIC);
	if (!buf)
	   	return -ENOMEM;
	memset(buf, 0, req->len);
	req1.addr = req->addr;
	req1.len = req->len;
	req1.buf = buf;
	rc = emapi_sdio_read(&req1);
	if (rc < 0) 
	   	goto rlt1;
   	if (copy_to_user((uint8_t*)req->buf, buf, req->len)) {
	   	rc = -EFAULT;
		goto rlt1;
	}
rlt1:
	kfree(buf);
	return rc;
}

int emapi_setmem(struct sdio_request *req)
{
   	int rc = 0;
	uint8_t *buf;
	struct sdio_request req1;

	buf = kmalloc(req->len, GFP_ATOMIC);
	if (!buf)
	   	return -ENOMEM;
	if(copy_from_user(buf, req->buf, req->len)) {
	   	rc = -EFAULT;
		goto rtl1;
	}	   
	req1.addr = req->addr;
	req1.len = req->len;
	req1.buf = buf;
	//printk("%s\t%08x\t%d\n", __FUNCTION__, req1.addr, req1.len);
	rc = emapi_sdio_write(&req1);
rtl1:
	kfree(buf);
	return rc;
}

int emapi_get_eeprom(unsigned char *usr_eeprom)
{
   	unsigned char *eeprom;

   	eeprom = get_wifi_nvs_ram();

	if (copy_to_user(usr_eeprom, eeprom, MAX_NVS_SIZE))
		return -EFAULT;
	return 0;
}












