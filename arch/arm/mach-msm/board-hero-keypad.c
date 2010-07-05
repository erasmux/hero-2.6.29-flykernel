/* linux/arch/arm/mach-msm7201a/board-hero-keypad.c
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


#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio_event.h>
#include <linux/keyreset.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>

#include "board-hero.h"

static int opt_x_axis_threshold = 1, opt_y_axis_threshold = 1;

struct hero_axis_info {
	struct gpio_event_axis_info info;
	uint16_t in_state;
	uint16_t out_state;
	uint16_t temp_state;
};

static bool nav_just_on;
static int nav_on_jiffies;

static unsigned int hero_col_gpios[] = { 35, 34, 33 };
static unsigned int hero_row_gpios[] = { 42, 41, 40 };

#define KEYMAP_INDEX(col, row) ((col)*ARRAY_SIZE(hero_row_gpios) + (row))

static const unsigned short hero_keymap0[ARRAY_SIZE(hero_col_gpios) * ARRAY_SIZE(hero_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_HOME,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 2)] = KEY_RESERVED,
	//[KEYMAP_INDEX(0, 3)] = KEY_RESERVED, // XA, XB unuse, XC remove

	[KEYMAP_INDEX(1, 0)] = KEY_MENU,
	[KEYMAP_INDEX(1, 1)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(1, 2)] = KEY_RESERVED,
	//[KEYMAP_INDEX(1, 3)] = KEY_RESERVED, // XA, XB unuse, XC remove

	[KEYMAP_INDEX(2, 0)] = KEY_BACK,
	[KEYMAP_INDEX(2, 1)] = KEY_RESERVED,
	[KEYMAP_INDEX(2, 2)] = KEY_RESERVED,
	//[KEYMAP_INDEX(2, 3)] = KEY_RESERVED, // XA, XB unuse, XC remove

};

static const unsigned short hero_keymap1[ARRAY_SIZE(hero_col_gpios) * ARRAY_SIZE(hero_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_BACK,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 2)] = KEY_SEND,
	//[KEYMAP_INDEX(0, 3)] = KEY_RESERVED, // XA, XB unuse, XC remove

	[KEYMAP_INDEX(1, 0)] = KEY_MENU,
	[KEYMAP_INDEX(1, 1)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(1, 2)] = KEY_RESERVED,
	//[KEYMAP_INDEX(1, 3)] = KEY_RESERVED, // XA, XB unuse, XC remove

	[KEYMAP_INDEX(2, 0)] = KEY_HOME,
	[KEYMAP_INDEX(2, 1)] = KEY_RESERVED,
	[KEYMAP_INDEX(2, 2)] = KEY_RESERVED,
	//[KEYMAP_INDEX(2, 3)] = KEY_RESERVED, // XA, XB unuse, XC remove
};

static const unsigned short hero_keymap2[ARRAY_SIZE(hero_col_gpios) * ARRAY_SIZE(hero_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_SEARCH,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 2)] = KEY_SEND,

	[KEYMAP_INDEX(1, 0)] = KEY_MENU,
	[KEYMAP_INDEX(1, 1)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(1, 2)] = KEY_BACK,

	[KEYMAP_INDEX(2, 0)] = KEY_HOME,
	[KEYMAP_INDEX(2, 1)] = KEY_RESERVED,
	[KEYMAP_INDEX(2, 2)] = MATRIX_KEY(1, BTN_MOUSE),
};

static const unsigned short hero_keymap2_engin3[ARRAY_SIZE(hero_col_gpios) * ARRAY_SIZE(hero_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_BACK,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 2)] = KEY_SEND,

	[KEYMAP_INDEX(1, 0)] = KEY_HOME,
	[KEYMAP_INDEX(1, 1)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(1, 2)] = KEY_MENU,

	[KEYMAP_INDEX(2, 0)] = KEY_SEARCH,
	[KEYMAP_INDEX(2, 1)] = KEY_RESERVED,
	[KEYMAP_INDEX(2, 2)] = MATRIX_KEY(1, BTN_MOUSE),
};

static struct gpio_event_matrix_info hero_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.keymap = hero_keymap2,
	.output_gpios = hero_col_gpios,
	.input_gpios = hero_row_gpios,
	.noutputs = ARRAY_SIZE(hero_col_gpios),
	.ninputs = ARRAY_SIZE(hero_row_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.debounce_delay.tv.nsec = 5 * NSEC_PER_MSEC,	
	.flags = GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_REMOVE_PHANTOM_KEYS |GPIOKPF_PRINT_UNMAPPED_KEYS /*| GPIOKPF_PRINT_MAPPED_KEYS*/
};

static struct gpio_event_direct_entry hero_keypad_nav_map[] = {
	{
		.gpio	= HERO_POWER_KEY,
		.code	= KEY_END
	},
};

static struct gpio_event_input_info hero_keypad_nav_info = {
	.info.func = gpio_event_input_func,
	.info.no_suspend = true,
	.flags = 0,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = hero_keypad_nav_map,
	.keymap_size = ARRAY_SIZE(hero_keypad_nav_map)
};

uint16_t hero_x_axis_map(struct gpio_event_axis_info *info, uint16_t in)
{
	struct hero_axis_info *ai =
			container_of(info, struct hero_axis_info, info);
	uint16_t out = ai->out_state;

	if (nav_just_on) {
		if (jiffies == nav_on_jiffies || jiffies == nav_on_jiffies + 1)
			goto ignore;
		nav_just_on = 0;
	}
	if ((ai->in_state ^ in) & 1)
		out--;
	if ((ai->in_state ^ in) & 2)
		out++;
	ai->out_state = out;
ignore:
	ai->in_state = in;
	if (ai->out_state - ai->temp_state == opt_x_axis_threshold) {
		ai->temp_state++;
		ai->out_state = ai->temp_state;
	} else if (ai->temp_state - ai->out_state == opt_x_axis_threshold) {
		ai->temp_state--;
		ai->out_state = ai->temp_state;
	} else if (abs(ai->out_state - ai->temp_state) > opt_x_axis_threshold)
		ai->temp_state = ai->out_state;

	return ai->temp_state;
}

uint16_t hero_y_axis_map(struct gpio_event_axis_info *info, uint16_t in)
{
	struct hero_axis_info *ai =
			container_of(info, struct hero_axis_info, info);
	uint16_t out = ai->out_state;

	if (nav_just_on) {
		if (jiffies == nav_on_jiffies || jiffies == nav_on_jiffies + 1)
			goto ignore;
		nav_just_on = 0;
	}
	if ((ai->in_state ^ in) & 1)
		out--;
	if ((ai->in_state ^ in) & 2)
		out++;
	ai->out_state = out;
ignore:
	ai->in_state = in;
	if (ai->out_state - ai->temp_state == opt_y_axis_threshold) {
		ai->temp_state++;
		ai->out_state = ai->temp_state;
	} else if (ai->temp_state - ai->out_state == opt_y_axis_threshold) {
		ai->temp_state--;
		ai->out_state = ai->temp_state;
	} else if (abs(ai->out_state - ai->temp_state) > opt_y_axis_threshold)
		ai->temp_state = ai->out_state;

	return ai->temp_state;
}


int hero_nav_power(const struct gpio_event_platform_data *pdata, bool on)
{
	gpio_set_value(HERO_GPIO_JOGBALL_EN, on);
	if (on) {
		nav_just_on = 1;
		nav_on_jiffies = jiffies;
	}
	return 0;
}

static uint32_t hero_x_axis_gpios[] = {
	HERO_GPIO_JOGBALL_LEFT_0, HERO_GPIO_JOGBALL_RIGHT_0
};

static struct hero_axis_info hero_x_axis = {
	.info = {
		.info.func = gpio_event_axis_func,
		.count = ARRAY_SIZE(hero_x_axis_gpios),
		.dev = 1,
		.type = EV_REL,
		.code = REL_X,
		.decoded_size = 1U << ARRAY_SIZE(hero_x_axis_gpios),
		.map = hero_x_axis_map,
		.gpio = hero_x_axis_gpios,
		.flags = GPIOEAF_PRINT_UNKNOWN_DIRECTION
			/*| GPIOEAF_PRINT_RAW | GPIOEAF_PRINT_EVENT */,
		.enable_emc_protect_delay = 1 * NSEC_PER_MSEC,
	}
};

static uint32_t hero_y_axis_gpios[] = {
	HERO_GPIO_JOGBALL_UP_0, HERO_GPIO_JOGBALL_DOWN_0
};

static struct hero_axis_info hero_y_axis = {
	.info = {
		.info.func = gpio_event_axis_func,
		.count = ARRAY_SIZE(hero_y_axis_gpios),
		.dev = 1,
		.type = EV_REL,
		.code = REL_Y,
		.decoded_size = 1U << ARRAY_SIZE(hero_y_axis_gpios),
		.map = hero_y_axis_map,
		.gpio = hero_y_axis_gpios,
		.flags = GPIOEAF_PRINT_UNKNOWN_DIRECTION
			/*| GPIOEAF_PRINT_RAW | GPIOEAF_PRINT_EVENT  */,
		.enable_emc_protect_delay = 1 * NSEC_PER_MSEC,
	}
};

static struct gpio_event_info *hero_input_info[] = {
	&hero_keypad_matrix_info.info,
	&hero_keypad_nav_info.info,
	&hero_x_axis.info.info,
	&hero_y_axis.info.info,
};

static struct gpio_event_platform_data hero_keypad_data = {
	.names = {
		"hero-keypad",
		"hero-nav",
		NULL,
	},
	.info = hero_input_info,
	.info_count = ARRAY_SIZE(hero_input_info),
	.power = hero_nav_power,
};

static struct platform_device hero_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &hero_keypad_data,
	},
};

module_param_named(x_axis_sens, opt_x_axis_threshold, int,
						S_IRUGO|S_IWUSR|S_IWGRP);
module_param_named(y_axis_sens, opt_y_axis_threshold, int,
						S_IRUGO|S_IWUSR|S_IWGRP);

static int hero_reset_keys_up[] = {
	BTN_MOUSE,
	0
};

static struct keyreset_platform_data hero_reset_keys_pdata = {
	.keys_up = hero_reset_keys_up,
	.keys_down = {
		KEY_SEND,
		KEY_MENU,
		KEY_END,
		0
	},
};

static struct platform_device hero_reset_keys_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &hero_reset_keys_pdata,
};

static int __init hero_init_keypad(void)
{
	if (!machine_is_hero())
		return 0;

	if (system_rev == 0) { /* XA */
		hero_keypad_matrix_info.keymap = hero_keymap0;
		hero_reset_keys_pdata.keys_down[0] = KEY_HOME;
		hero_reset_keys_pdata.keys_down[2] = KEY_POWER;
	} else
	if (system_rev == 1) { /* XB */
		hero_keypad_matrix_info.keymap = hero_keymap1;
	}

	if (hero_get_engineerid() == 3)
		hero_keypad_matrix_info.keymap = hero_keymap2_engin3;

	if (platform_device_register(&hero_reset_keys_device))
		printk(KERN_WARNING "%s: register reset key fail\n", __func__);

	return platform_device_register(&hero_keypad_device);
}

device_initcall(hero_init_keypad);
