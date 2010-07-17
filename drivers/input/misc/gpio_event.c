/* drivers/input/misc/gpio_event.c
 *
 * Copyright (C) 2007 Google, Inc.
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

#include <linux/earlysuspend.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/gpio_event.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#ifdef CONFIG_MACH_HEROC
#include <linux/jiffies.h>
#include <asm/mach-types.h>
/* hero_c only */
static spinlock_t filter_key_lock;
static unsigned long filter_time = 50;

struct _filter_key_set {
	uint16_t check_key;
	uint16_t filter_key[2];
//	uint32_t filter_key_pos[2]; /* dependence on keycode position */
	unsigned long key_jiff_stamp;
	uint8_t force_release_key;
};

struct _filter_key_data {
	struct _filter_key_set *filter_key_set;
	uint8_t set_num;
};

static struct _filter_key_set key_set[] = {
	{
		.check_key = KEY_SEND,
		.filter_key = { KEY_MENU, KEY_HOME },
		.key_jiff_stamp = 0,
	}, {
		.check_key = KEY_END,
		.filter_key = { KEY_COMPOSE, KEY_BACK },
		.key_jiff_stamp = 0,
	}
};

static struct _filter_key_data filter_key_data = {
	.filter_key_set = key_set,
	.set_num = ARRAY_SIZE(key_set),
};

#endif

struct gpio_event {
	struct gpio_event_input_devs *input_devs;
	const struct gpio_event_platform_data *info;
	struct early_suspend early_suspend;
	void *state[0];
};

static int gpio_input_event(
	struct input_dev *dev, unsigned int type, unsigned int code, int value)
{
	int i;
	int devnr;
	int ret = 0;
	int tmp_ret;
	struct gpio_event_info **ii;
	struct gpio_event *ip = input_get_drvdata(dev);

	for (devnr = 0; devnr < ip->input_devs->count; devnr++)
		if (ip->input_devs->dev[devnr] == dev)
			break;
	if (devnr == ip->input_devs->count) {
		pr_err("gpio_input_event: unknown device %p\n", dev);
		return -EIO;
	}

	for (i = 0, ii = ip->info->info; i < ip->info->info_count; i++, ii++) {
		if ((*ii)->event) {
			tmp_ret = (*ii)->event(ip->input_devs, *ii,
						&ip->state[i],
						devnr, type, code, value);
			if (tmp_ret)
				ret = tmp_ret;
		}
	}
	return ret;
}

static int gpio_event_call_all_func(struct gpio_event *ip, int func)
{
	int i;
	int ret;
	struct gpio_event_info **ii;

	if (func == GPIO_EVENT_FUNC_INIT || func == GPIO_EVENT_FUNC_RESUME) {
		ii = ip->info->info;
		for (i = 0; i < ip->info->info_count; i++, ii++) {
			if ((*ii)->func == NULL) {
				ret = -ENODEV;
				pr_err("gpio_event_probe: Incomplete pdata, "
					"no function\n");
				goto err_no_func;
			}
			if (func == GPIO_EVENT_FUNC_RESUME && (*ii)->no_suspend)
				continue;
			ret = (*ii)->func(ip->input_devs, *ii, &ip->state[i],
					  func);
			if (ret) {
				pr_err("gpio_event_probe: function failed\n");
				goto err_func_failed;
			}
		}
		return 0;
	}

	ret = 0;
	i = ip->info->info_count;
	ii = ip->info->info + i;
	while (i > 0) {
		i--;
		ii--;
		if ((func & ~1) == GPIO_EVENT_FUNC_SUSPEND && (*ii)->no_suspend)
			continue;
		(*ii)->func(ip->input_devs, *ii, &ip->state[i], func & ~1);
err_func_failed:
err_no_func:
		;
	}
	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void gpio_event_suspend(struct early_suspend *h)
{
	struct gpio_event *ip;
	ip = container_of(h, struct gpio_event, early_suspend);
	gpio_event_call_all_func(ip, GPIO_EVENT_FUNC_SUSPEND);
	ip->info->power(ip->info, 0);
}

void gpio_event_resume(struct early_suspend *h)
{
	struct gpio_event *ip;
	ip = container_of(h, struct gpio_event, early_suspend);
	ip->info->power(ip->info, 1);
	gpio_event_call_all_func(ip, GPIO_EVENT_FUNC_RESUME);
}
#endif

#ifdef CONFIG_MACH_HEROC
static ssize_t gpio_key_filter_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = sprintf(buf, "%lu\n", filter_time);
	return ret;
}
static ssize_t gpio_key_filter_time_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	unsigned long tmp = 0;
	tmp = simple_strtoul(buf, NULL, 10);
	if(tmp)
		filter_time = tmp;
	else
		printk(KERN_WARNING "%s: setup filter_key fail...\n", __func__);

	return count;
}

static DEVICE_ATTR(key_filter_time, 0644,
	gpio_key_filter_time_show, gpio_key_filter_time_store);
#endif

static int __init gpio_event_probe(struct platform_device *pdev)
{
	int err;
	struct gpio_event *ip;
	struct gpio_event_platform_data *event_info;
	int dev_count = 1;
	int i;
	int registered = 0;

	event_info = pdev->dev.platform_data;
	if (event_info == NULL) {
		pr_err("gpio_event_probe: No pdata\n");
		return -ENODEV;
	}
	if ((!event_info->name && !event_info->names[0]) ||
	    !event_info->info || !event_info->info_count) {
		pr_err("gpio_event_probe: Incomplete pdata\n");
		return -ENODEV;
	}
	if (!event_info->name)
		while (event_info->names[dev_count])
			dev_count++;
	ip = kzalloc(sizeof(*ip) +
		     sizeof(ip->state[0]) * event_info->info_count +
		     sizeof(*ip->input_devs) +
		     sizeof(ip->input_devs->dev[0]) * dev_count, GFP_KERNEL);
	if (ip == NULL) {
		err = -ENOMEM;
		pr_err("gpio_event_probe: Failed to allocate private data\n");
		goto err_kp_alloc_failed;
	}
	ip->input_devs = (void*)&ip->state[event_info->info_count];
	platform_set_drvdata(pdev, ip);

	for (i = 0; i < dev_count; i++) {
		struct input_dev *input_dev = input_allocate_device();
		if (input_dev == NULL) {
			err = -ENOMEM;
			pr_err("gpio_event_probe: "
				"Failed to allocate input device\n");
			goto err_input_dev_alloc_failed;
		}
		input_set_drvdata(input_dev, ip);
		input_dev->name = event_info->name ?
					event_info->name : event_info->names[i];
		input_dev->event = gpio_input_event;
		ip->input_devs->dev[i] = input_dev;
	}
	ip->input_devs->count = dev_count;
	ip->info = event_info;
	if (event_info->power) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		ip->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		ip->early_suspend.suspend = gpio_event_suspend;
		ip->early_suspend.resume = gpio_event_resume;
		register_early_suspend(&ip->early_suspend);
#endif
		ip->info->power(ip->info, 1);
	}

	err = gpio_event_call_all_func(ip, GPIO_EVENT_FUNC_INIT);
	if (err)
		goto err_call_all_func_failed;

	for (i = 0; i < dev_count; i++) {
		err = input_register_device(ip->input_devs->dev[i]);
		if (err) {
			pr_err("gpio_event_probe: Unable to register %s "
				"input device\n", ip->input_devs->dev[i]->name);
			goto err_input_register_device_failed;
		}
		registered++;
	}

#ifdef CONFIG_MACH_HEROC
	if(machine_is_heroc()) {
		spin_lock_init(&filter_key_lock);
		err = device_create_file(&pdev->dev,
		&dev_attr_key_filter_time);
		if(err)
			pr_err("%s: Unable to create filter_key sysfs\n", __func__);
	}
#endif

	return 0;

err_input_register_device_failed:
	gpio_event_call_all_func(ip, GPIO_EVENT_FUNC_UNINIT);
err_call_all_func_failed:
	if (event_info->power) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&ip->early_suspend);
#endif
		ip->info->power(ip->info, 0);
	}
	for (i = 0; i < registered; i++)
		input_unregister_device(ip->input_devs->dev[i]);
	for (i = dev_count - 1; i >= registered; i--) {
		input_free_device(ip->input_devs->dev[i]);
err_input_dev_alloc_failed:
		;
	}
	kfree(ip);
err_kp_alloc_failed:
	return err;
}

static int gpio_event_remove(struct platform_device *pdev)
{
	struct gpio_event *ip = platform_get_drvdata(pdev);
	int i;

	gpio_event_call_all_func(ip, GPIO_EVENT_FUNC_UNINIT);
	if (ip->info->power) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&ip->early_suspend);
#endif
		ip->info->power(ip->info, 0);
	}
	for (i = 0; i < ip->input_devs->count; i++)
		input_unregister_device(ip->input_devs->dev[i]);
	kfree(ip);
	return 0;
}

static struct platform_driver gpio_event_driver = {
	.probe		= gpio_event_probe,
	.remove		= gpio_event_remove,
	.driver		= {
		.name	= GPIO_EVENT_DEV_NAME,
	},
};

static int __devinit gpio_event_init(void)
{
	return platform_driver_register(&gpio_event_driver);
}

static void __exit gpio_event_exit(void)
{
	platform_driver_unregister(&gpio_event_driver);
}

module_init(gpio_event_init);
module_exit(gpio_event_exit);

MODULE_DESCRIPTION("GPIO Event Driver");
MODULE_LICENSE("GPL");

#ifdef CONFIG_MACH_HEROC
uint8_t button_filter(struct input_dev *dev,
		 unsigned int type, unsigned int code, int value, unsigned long *key_pressed)
{
	int ret = 1;
	uint8_t loop_i, loop_j;
	unsigned long irqflags;

	if(!machine_is_heroc())
		return ret;

	spin_lock_irqsave(&filter_key_lock, irqflags);
	if(!strcmp("heroc-keypad", dev->name) && (EV_KEY == type)) {
		for(loop_i = 0; loop_i < filter_key_data.set_num; loop_i++) {
			if(code == filter_key_data.filter_key_set[loop_i].filter_key[0] ||
				code == filter_key_data.filter_key_set[loop_i].filter_key[1]) {
				for(loop_j = 0; loop_j < 2; loop_j++) {
					if(value) {
						if(code == filter_key_data.filter_key_set[loop_i].filter_key[loop_j])
							filter_key_data.filter_key_set[loop_i].force_release_key |= (0x1 << loop_j);
					} else {
						if(code == filter_key_data.filter_key_set[loop_i].filter_key[loop_j])
							filter_key_data.filter_key_set[loop_i].force_release_key &= ~(0x1 << loop_j);
					}
				}
				if(filter_key_data.filter_key_set[loop_i].key_jiff_stamp && value &&
					time_after((filter_key_data.filter_key_set[loop_i].key_jiff_stamp + (filter_time * (HZ / 100))), jiffies)) {
					for(loop_j = 0; loop_j < 2; loop_j++) {
						if(code == filter_key_data.filter_key_set[loop_i].filter_key[loop_j]) {
							printk(KERN_DEBUG "%s: in filter_time, discard: %d(%lu)\n", __func__, code, *key_pressed);
							ret = 0;
							break;
						}
					}
				}
			}
		}
		for(loop_i = 0; loop_i < filter_key_data.set_num && value; loop_i++) {
			if(code == filter_key_data.filter_key_set[loop_i].check_key) {
				for(loop_j = 0; loop_j < 2; loop_j++) {
					if(filter_key_data.filter_key_set[loop_i].force_release_key & (0x1 << loop_j)) {
						filter_key_data.filter_key_set[loop_i].force_release_key &= ~(0x1 << loop_j);
						input_event(dev, type, filter_key_data.filter_key_set[loop_i].filter_key[loop_j], 0);
					}
				}
				filter_key_data.filter_key_set[loop_i].key_jiff_stamp = jiffies;
				printk(KERN_DEBUG "%s: store %d jiff_stamp: %lu\n", __func__,
					filter_key_data.filter_key_set[loop_i].check_key,
					filter_key_data.filter_key_set[loop_i].key_jiff_stamp);
				break;
			}
		}
	}
	spin_unlock_irqrestore(&filter_key_lock, irqflags);
	return ret;
}
#endif
