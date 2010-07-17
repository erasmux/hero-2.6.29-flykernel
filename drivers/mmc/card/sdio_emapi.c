#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/mach-types.h>

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>

#include <linux/delay.h>

#include <linux/miscdevice.h>
//#include <linux/cdev.h>
#include <asm/uaccess.h>

/* sdio function */
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>

#include "sdio_emapi.h"

#ifdef CONFIG_WIFI_CONTROL_FUNC
#include <linux/platform_device.h>
#include <linux/wifi_tiwlan.h>
int msm_wifi_power(int on);
int msm_wifi_reset(int on);
#endif


#define DEV_NAME "emapi"

struct _emapi {
	struct class		*emapi_class;
	struct device		*device;
	dev_t			emapi_cdevno;
	//struct cdev		emapi_cdev;
	struct proc_dir_entry	*calibration;
	int			sdio_status;
};
static struct _emapi emapi;


/* sdio driver */
static const struct sdio_device_id emapi_sdio_ids[] ={
  	{ SDIO_DEVICE_CLASS(SDIO_CLASS_WLAN) },
	{				     },
};

MODULE_DEVICE_TABLE(sdio, emapi_sdio_ids);

#ifdef CONFIG_WIFI_CONTROL_FUNC
static struct wifi_platform_data *wifi_control_data = NULL;

static int wifi_probe( struct platform_device *pdev )
{
    struct wifi_platform_data *wifi_ctrl = (struct wifi_platform_data *)(pdev->dev.platform_data);

    printk("%s\n", __FUNCTION__);
    if( wifi_ctrl ) {
	wifi_control_data = wifi_ctrl;
	if( wifi_ctrl->set_power )
	    wifi_ctrl->set_power(1);		/* Power On */
	if( wifi_ctrl->set_reset )
	    wifi_ctrl->set_reset(0);		/* Reset clear */
	if( wifi_ctrl->set_carddetect )
	    wifi_ctrl->set_carddetect(1);	/* CardDetect (0->1) */
    }
    return 0;
}

static int wifi_remove( struct platform_device *pdev )
{
    struct wifi_platform_data *wifi_ctrl = (struct wifi_platform_data *)(pdev->dev.platform_data);

    printk("%s\n", __FUNCTION__);
    if( wifi_ctrl ) {
	if( wifi_ctrl->set_carddetect )
	    wifi_ctrl->set_carddetect(0);	/* CardDetect (1->0) */
	if( wifi_ctrl->set_reset )
	    wifi_ctrl->set_reset(1);		/* Reset active */
	if( wifi_ctrl->set_power )
	    wifi_ctrl->set_power(0);		/* Power Off */
    }
    return 0;
}

static struct platform_driver wifi_device = {
    .probe          = wifi_probe,
    .remove         = wifi_remove,
    .suspend        = NULL,
    .resume         = NULL,
    .driver         = {
        .name   = "msm_wifi",
    },
};

static int wifi_add_dev( void )
{
    return platform_driver_register( &wifi_device );
}

static void wifi_del_dev( void )
{
    platform_driver_unregister( &wifi_device );
}

int msm_wifi_power( int on )
{
    printk("%s\n", __FUNCTION__);
    if( wifi_control_data && wifi_control_data->set_power ) {
	wifi_control_data->set_power(on);
    }
    return 0;
}

int msm_wifi_reset( int on )
{
    printk("%s\n", __FUNCTION__);
    if( wifi_control_data && wifi_control_data->set_reset ) {
	wifi_control_data->set_reset(on);
    }
    return 0;
}
#endif

static void emapi_sdio_irq(struct sdio_func *func)
{
   	printk("%s\n", __func__);
}

static int emapi_sdio_probe(struct sdio_func *func, const struct sdio_device_id *id)
{
   	int rc;

	emapi_setfunc(NULL);

	printk("emapi router - class 0x%x, Vendor 0x%x, Device 0x%x\n",
	      	func->class, func->vendor, func->device);

	if (func->vendor != VENDOR_ID || func->device != DEVICE_ID)
	   	return -ENODEV;

	sdio_claim_host(func);
	rc = sdio_enable_func(func);
	if (rc)
	   	goto sdio_err1;

	rc = sdio_claim_irq(func, emapi_sdio_irq);
	if (rc)
	   	goto sdio_err2;

	rc = sdio_set_block_size(func, 512);
	if (rc)
	   	goto sdio_err2;

	emapi_setfunc(func);

	return 0;

sdio_err2:
	sdio_disable_func(func);
sdio_err1:
	sdio_release_host(func);
	return rc;
}

static void emapi_sdio_remove(struct sdio_func *func)
{
   	sdio_release_irq(func);
	sdio_disable_func(func);
	sdio_release_host(func);
}

static struct sdio_driver emapi_sdio_drv = {
   	.name		= "emapi_sdio",
	.id_table	= emapi_sdio_ids,
	.probe		= emapi_sdio_probe,
	.remove		= emapi_sdio_remove,
};
/* end of sdio */


/* calibration read func */
static int emapi_calibration_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
   	unsigned char *nvs;
	unsigned long len;

	nvs = get_wifi_nvs_ram();
	if (nvs) {
	   	/* htc specific, 0x0c: body len */
	   	memcpy(&len, nvs + 0x0c, 4);
		/* max size is 2048 */
		len = min(len+ 0x40, (unsigned long)2048);
		memcpy((void *)page, (void *) nvs, len);
		//memcpy(page, nvs, len);
		printk("%s - read %d bytes\n", __FUNCTION__, (unsigned int)len);
		return len;
	}
	else {
	   	printk("%s - no calibration data\n", __FUNCTION__);
		return -EIO;
	}
}

static int emapi_calibration_write(struct file *file, const char __user *buffer,
			   unsigned long count, void *data)
{
	printk("%s do nothing\n", __FUNCTION__);
   	return 0;
}

int emapi_sdio_init(unsigned long arg)
{
	int rc = 0;
	unsigned long cmd;

	get_user(cmd, (unsigned long *) arg);

	switch (cmd) {
		case 0:
#ifdef CONFIG_WIFI_CONTROL_FUNC
			wifi_add_dev();
#else
			printk(KERN_CRIT "EMAPI: fail to power on\n");
#endif
			mdelay(50);
			break;
		case 1:
			if (emapi.sdio_status)
				sdio_unregister_driver(&emapi_sdio_drv);
#ifdef CONFIG_WIFI_CONTROL_FUNC
			wifi_del_dev();
#else
			printk(KERN_CRIT "EMAPI: fail to power off\n");
#endif
			emapi.sdio_status = 0;
			mdelay(50);
			break;
		case 2:
			rc = sdio_register_driver(&emapi_sdio_drv);
			if (rc < 0) {
	   			printk("emapi sdio driver init error\n");
				goto error;
			}
			emapi.sdio_status = 1;
			break;
		default:
			printk("unkown command\n");
			rc = -EINVAL;
			goto error;
	};
	return rc;
error:
	if (emapi_sdio_drv.drv.bus != NULL)
		sdio_unregister_driver(&emapi_sdio_drv);
#ifdef CONFIG_WIFI_CONTROL_FUNC
	wifi_del_dev();
#else
	printk(KERN_CRIT "EMAPI: fail to power off\n");
#endif
	return rc;
}

/* most content happens here */
static int emapi_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
   	int error = 0;
	int retval = 0;
	struct sdio_request req;

	if (_IOC_TYPE(cmd) != MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > EMAPI_IOC_MAXNR)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		error = !access_ok(VERIFY_WRITE, (void __user*)arg, _IOC_SIZE(cmd));
	if (_IOC_DIR(cmd) & _IOC_WRITE)
		error = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	if (error)
		return -ENOTTY;

	if (cmd != EMAPI_IOC_INIT) {
		if (copy_from_user(&req, (void __user *) arg, sizeof(struct sdio_request)))
			return -EFAULT;
	}

	switch(cmd) {
		case EMAPI_IOC_INIT:
			retval = emapi_sdio_init(arg);
			break;
		case EMAPI_IOC_GETMEM:
	      		retval = emapi_getmem(&req);
	      		break;
		case EMAPI_IOC_SETMEM:
	      		retval = emapi_setmem(&req);
			break;
		case EMAPI_IOC_GETMAC:
	      		retval = emapi_getmac(&req);
			break;
		case EMAPI_IOC_SETMAC:
	      		retval = emapi_setmac(&req);
			break;
		case EMAPI_IOC_SETPAR:
			retval = emapi_set_partition(req.addr, req.len);
			break;
		case EMAPI_IOC_SETPAR2:
			retval = emapi_set_partition2(req.addr, req.len);
			break;
		case EMAPI_IOC_GETPAR:
			retval = emapi_get_partition();
			break;
		case EMAPI_IOC_GETEEPROM:
			retval = emapi_get_eeprom(req.buf);
			break;
		case EMAPI_IOC_SETEEPROM:
			break;
		case EMAPI_IOC_TADDR2SADDR:
			{
				unsigned long outaddr = 0;
				retval = emapi_tnetwaddr2sdioaddr(req.addr, &outaddr);
				if (!retval) {
					req.addr = outaddr;
					if (copy_to_user((void __user *) arg, &req, 4))
						return -EFAULT;
				}
				break;
			}
		default:
			printk("emapi: unknown command\n");
			return -ENOTTY;
	};

	return retval;
}

static int emapi_open(struct inode *inode, struct file *filp)
{
	printk("emapi is open\n");
	return nonseekable_open(inode, filp);
}

static int emapi_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static struct file_operations emapi_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= emapi_ioctl,
	.open		= emapi_open,
	.release	= emapi_release,
};

static struct miscdevice emapi_device = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "emapi",
	.fops	= &emapi_fops,
};

static int emapi_init_module(void)
{
	int rc;

#if 1
	rc = misc_register(&emapi_device);
	if (rc) {
		printk(KERN_ERR "emapi: register fail\n");
		return rc;
	}
	emapi.calibration = create_proc_entry("emapi_calibration", 0644, NULL);
	if (!emapi.calibration) {
	   	printk(KERN_ERR "emapi: alloc calibration error\n");
		rc = PTR_ERR(emapi.calibration);
		if (misc_deregister(&emapi_device))
			printk(KERN_ERR "emapi: deregister fail\n");
		return rc;
	}

	emapi.calibration->read_proc = emapi_calibration_read;
	emapi.calibration->write_proc = emapi_calibration_write;

	emapi.sdio_status = 0;
	return 0;
#else
	/* create device node */
	emapi.emapi_class = class_create(THIS_MODULE, DEV_NAME);
	if (IS_ERR(emapi.emapi_class)) {
		rc = PTR_ERR(emapi.emapi_class);
		emapi.emapi_class = NULL;
		return rc;
	}

	/* register char drv */
	rc = alloc_chrdev_region(&emapi.emapi_cdevno, 0, 1, DEV_NAME);
	if (rc) {
		class_destroy(emapi.emapi_class);
		device_destroy(emapi.emapi_class, 0);
		emapi.device = NULL;
		emapi.emapi_class = NULL;
		return rc;
	}

	emapi.device = device_create(emapi.emapi_class, NULL, 0, "%s", "emapi");

	if (unlikely(IS_ERR(emapi.device))) {
		rc = PTR_ERR(emapi.device);
		emapi.device = NULL;
		class_destroy(emapi.emapi_class);
		emapi.emapi_class = NULL;
		return rc;
	}

	cdev_init(&emapi.emapi_cdev, &emapi_cdev_fops);
	emapi.emapi_cdev.owner = THIS_MODULE;
	cdev_add(&emapi.emapi_cdev, emapi.emapi_cdevno, 1);

	emapi.calibration = create_proc_entry("emapi_calibration", 0644, NULL);
	if (!emapi.calibration) {
	   	printk("emapi calibration proc error\n");
		goto init_err;
	}

	emapi.calibration->read_proc = emapi_calibration_read;
	emapi.calibration->write_proc = emapi_calibration_write;

	emapi.sdio_status = 0;

	return 0;
init_err1:
	remove_proc_entry("emapi_calibration", NULL);
init_err:
	class_destroy(emapi.emapi_class);
	device_destroy(emapi.emapi_class, 0);
	emapi.device = NULL;
	emapi.emapi_class = NULL;
	unregister_chrdev_region(emapi.emapi_cdevno, 1);
	return rc;
#endif
}

static void emapi_exit_module(void)
{
	if (emapi.sdio_status)
   		sdio_unregister_driver(&emapi_sdio_drv);
#ifdef CONFIG_WIFI_CONTROL_FUNC
	wifi_del_dev();
#else
	printk(KERN_CRIT "EMAPI: fail to power off\n");
#endif

#if 1
	if (misc_deregister(&emapi_device))
		printk("emapi: deregister fail\n");
#else
	class_destroy(emapi.emapi_class);
	device_destroy(emapi.emapi_class, 0);
	unregister_chrdev_region(emapi.emapi_cdevno, 1);
#endif
	remove_proc_entry("emapi_calibration", NULL);
}

module_init(emapi_init_module);
module_exit(emapi_exit_module);

MODULE_LICENSE("GPL");
