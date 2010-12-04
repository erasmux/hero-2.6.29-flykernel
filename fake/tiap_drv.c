/*
 * dummy tiap_drv module
 *
 */

#include <linux/platform_device.h>

static int fake_probe(struct platform_device *pdev)
{

	printk(KERN_INFO "fake tiap_drv probe\n");
	return 0;
}

static int fake_remove(struct platform_device *pdev)
{
	printk(KERN_INFO "fake tiap_drv remove\n");
	return 0;
}

static struct platform_driver fake_device = {
	.probe		= fake_probe,
	.remove		= fake_remove,
	.driver		= {
	.name   	= "tiap_drv",
	},
};

static int __init fake_init(void)
{
	return platform_driver_register(&fake_device);
}

static void __exit fake_exit(void)
{
	platform_driver_unregister(&fake_device);
}

module_init(fake_init);
module_exit(fake_exit);
MODULE_LICENSE("GPL");
