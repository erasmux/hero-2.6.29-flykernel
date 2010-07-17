/* linux/drivers/i2c/chips/microp_i2c.c
 * Copyright (C) 2009 HTC Corporation.
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
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <mach/microp_i2c.h>
#include <linux/wakelock.h>
#include <mach/htc_pwrsink.h>
#include <linux/bma150.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/earlysuspend.h>
#include <mach/htc_battery.h>
#include <mach/drv_callback.h>
#include <linux/lightsensor.h>

#define ENABLE_READ_35MM_ADC_VALUE_FROM_ATTR_FILE

#define FUNC_MICROP_VERSION						1
#define FUNC_MICROP_SET_FREQ_FADE				2
#define FUNC_MICROP_GPO							3
#define FUNC_MICROP_PWM							4
#define FUNC_MICROP_PWM_AUTO					5
#define FUNC_MICROP_OFF_TIMER					6
#define FUNC_MICROP_ADC_SELECT					7
#define FUNC_MICROP_RESET_IF					8
#define FUNC_MICROP_INTR_IF						9
#define FUNC_MICROP_EN_SPI_AUTO_BACKLIGHT		10
#define FUNC_MICROP_PULL_UP						11
#define FUNC_MICROP_LS_GPIO					12 /* simulation GPIO as PWM */
#define FUNC_MICROP_PWM_TRACKBALL				13 /* track ball bring function */
#define FUNC_MICROP_PULL_UP1					14
#define FUNC_MICROP_DISABLE_INTR				15
#define FUNC_MICROP_ADC_INTR					16
#define FUNC_MICROP_LED_AUTO					17

#define MICROP_I2C_CMD_MISC1					0x20
#define MICROP_I2C_CMD_SPI_INTERFACE			0x21
#define MICROP_I2C_CMD_PIN_PULL_UP				0x22
#define MICROP_I2C_CMD_PIN_PULL_UP1				0x23
#define MICROP_I2C_CMD_VERSION					0x29
#define MICROP_I2C_CMD_SET_INTERRUPT_CONTROL	0x40
#define MICROP_I2C_CMD_GET_INTERRUPT_STATUS		0x41
#define MICROP_I2C_CMD_CLEAR_INTERRUPT			0x42
#define MICROP_I2C_CMD_ADC_READ_DATA_CMD		0x50
#define MICROP_I2C_CMD_ADC_READ_DATA			0x51
#define MICROP_I2C_CMD_ADC_INTR					0x52
#define MICROP_I2C_CMD_PIN_CONFIG				0x60
#define MICROP_I2C_CMD_PIN_MODE					0x61
#define MICROP_I2C_CMD_PIN_PWM					0x62
#define MICROP_I2C_CMD_ADC_TABLE				0x63
#define MICROP_I2C_CMD_PIN_OFF					0x64
#define MICROP_I2C_CMD_RESET_SETTING			0x65
#define MICROP_I2C_CMD_INTR_SETTING				0x66
#define MICROP_I2C_CMD_SET_GPIO_PWM_FUNC		0x67
#define MICROP_I2C_CMD_READ_PIN					0x68
#define MICROP_I2C_CMD_LED_AUTO_TABLE				0x69
#define MICROP_I2C_CMD_LED_AUTO_ENABLE			0x6A
#define MICROP_GSENSOR_I2C_CMD_WRITE_DATA		0x73
#define MICROP_GSENSOR_I2C_CMD_REQUEST_REG		0x74
#define MICROP_GSENSOR_I2C_CMD_READ_DATA		0x75

#define MICROP_I2C_PWM_FREQ				0
#define MICROP_I2C_PWM_MANUAL			1
#define MICROP_I2C_PWM_AUTO				2
#define MICROP_I2C_PWM_LEVELS			3
#define MICROP_I2C_PWM_FADE				4
#define MICROP_I2C_PWM_JOGBALL_FUNC		5

#define MICROP_INT_GPIO					0x01
#define MICROP_INT_LIGHT_SENSOR			0x02
#define MICROP_INT_G_SENSOR				0x04
#define MICROP_INT_ADC					0x08

#define LCD_BACKLIGHT_GATE 				"lcd-backlight-gate"
#define FADE_DELAY  					msecs_to_jiffies(512)
#define HPIN_DELAY  					msecs_to_jiffies(700)
#define LS_DELAY					msecs_to_jiffies(800)

#define I2C_READ_RETRY_TIMES  		10
#define I2C_WRITE_RETRY_TIMES 		10
#define MICROP_I2C_WRITE_BLOCK_SIZE 21

static struct wake_lock microp_i2c_wakelock;
static void microp_i2c_gate_work_func(struct work_struct *work);
static int microp_i2c_auto_backlight_set_interrupt_mode(struct i2c_client *client, uint8_t enabled);
static struct work_struct gate_work;
static struct work_struct notifier_work;
static struct delayed_work notifier_delay_work;
static struct delayed_work auto_bl_delay_work;
static void microp_i2c_fade_work_func(struct work_struct *work);
static struct delayed_work fade_work;
static void microp_i2c_hpin_work_func(struct work_struct *work);
static struct delayed_work hpin_work;
static struct led_classdev *ldev_lcd_backlight;
static struct led_classdev *ldev_vkey_backlight;
static int microp_i2c_is_backlight_on;
static int cabc_backlight_enabled;
static int remote_adc_read_channel;
static unsigned long ls_power_on_jiffy;
static int ls_enable_num;

static int is_35mm_hpin;

static atomic_t als_intr_enabled = ATOMIC_INIT(0);
static atomic_t als_intr_enable_flag = ATOMIC_INIT(0);

static void microp_lcd_backlight_gate_set(struct led_classdev *led_cdev,
			       enum led_brightness brightness);
static struct led_classdev ldev_lcd_backlight_gate = {
	.name = LCD_BACKLIGHT_GATE,
	.brightness_set = microp_lcd_backlight_gate_set,
	.brightness = 255,
	.default_trigger = LCD_BACKLIGHT_GATE,
};

static void microp_lcd_backlight_notifier_set(struct led_classdev *led_cdev,
			       enum led_brightness brightness);
static struct led_classdev ldev_lcd_backlight_notifier = {
	.name = "lcd_notifier",
	.brightness_set = microp_lcd_backlight_notifier_set,
	.brightness = 0,
	.default_trigger = LCD_BACKLIGHT_GATE,
};

struct microp_led_data {
	struct led_classdev ldev;
	struct microp_pin_config *pin_config;
	struct mutex pin_mutex;
	uint8_t mode;
	uint8_t is_auto;
	uint8_t fade_timer; /* change every 32 ms */
	uint16_t off_timer; /* high byte min, low byte sec */
	uint8_t skip_config;
};

struct microp_i2c_work {
	struct work_struct work;
	struct i2c_client *client;
	int microp_intr_pin;
	int (*intr_debounce)(uint8_t *pin_status);
	void (*intr_function)(uint8_t *pin_status);
};

struct microp_i2c_client_data {
	struct microp_led_data *led_data;
	int num_led_data;
	struct mutex microp_i2c_mutex;
	uint16_t version;
	int use_irq;
	struct microp_i2c_work work;
	struct workqueue_struct *microp_queue;
	struct early_suspend early_suspend;
	struct device *adc_device;
	struct input_dev *ls_input_dev;
#ifdef ENABLE_READ_35MM_ADC_VALUE_FROM_ATTR_FILE
	int num_adc_read_data;
#endif
	uint8_t enable_early_suspend;
	uint8_t enable_reset_button;
	uint8_t microp_gpio_pwm_is_enabled;
	uint8_t enable_pwm_delay_250ms;
	atomic_t suspended_now;
};

static struct i2c_client *private_microp_client;

inline int microp_i2c_is_led(uint8_t config)
{
	if (config & MICROP_PIN_CONFIG_OTHER)
		return 0;
	if (config & (MICROP_PIN_CONFIG_PWM | MICROP_PIN_CONFIG_PWM_INV |
		MICROP_PIN_CONFIG_GPO | MICROP_PIN_CONFIG_GPO_INV))
		return 1;
	return 0;
}

inline int microp_i2c_is_pwm(uint8_t config)
{
	if (config & MICROP_PIN_CONFIG_OTHER)
		return 0;
	if (config & (MICROP_PIN_CONFIG_PWM | MICROP_PIN_CONFIG_PWM_INV))
		return 1;
	return 0;
}

inline int microp_i2c_is_gpo(uint8_t config)
{
	if (config & MICROP_PIN_CONFIG_OTHER)
		return 0;
	if (config & (MICROP_PIN_CONFIG_GPO | MICROP_PIN_CONFIG_GPO_INV))
		return 1;
	return 0;
}

inline int microp_i2c_is_intr(uint8_t config)
{
	switch (config) {
	case MICROP_PIN_CONFIG_INTR:
	case MICROP_PIN_CONFIG_INTR_ALL:
		return 1;
		break;
	}
	return 0;
}

inline int microp_i2c_is_intr_all(uint8_t config)
{
	return (config == MICROP_PIN_CONFIG_INTR_ALL) ? 1:0;
}

inline int microp_i2c_is_adc(uint8_t config)
{
	return (config == MICROP_PIN_CONFIG_ADC) ? 1:0;
}

inline int microp_i2c_is_adc_read(uint8_t config)
{
	return ((config == MICROP_PIN_CONFIG_ADC_READ) ||
			(config == MICROP_PIN_CONFIG_UP_ADC)) ? 1:0;
}

inline int microp_i2c_is_pullup(uint8_t config)
{
	return (config == MICROP_PIN_CONFIG_PULL_UP)?1:0;
}

inline int microp_i2c_is_pullup1(uint8_t config)
{
	return (config == MICROP_PIN_CONFIG_PULL_UP1)?1:0;
}

inline int microp_i2c_is_ls_gpio(uint8_t config)
{
	return (config == MICROP_PIN_CONFIG_LS_GPO)? 1:0;
}

inline int microp_i2c_is_mic(uint8_t config)
{
	return (config == MICROP_PIN_CONFIG_MIC) ? 1:0;
}

inline int microp_i2c_is_uP_adc(uint8_t config)
{
	return (config == MICROP_PIN_CONFIG_UP_ADC) ? 1 : 0;
}

int microp_i2c_is_supported(uint8_t func, uint16_t version)
{
	int is_supported = 0;

	if (version == 0x050C)
		return is_supported;

	switch (func) {
	case FUNC_MICROP_VERSION:
		is_supported = 1;
		break;
	case FUNC_MICROP_GPO:
	case FUNC_MICROP_PWM:
		is_supported = 1;
		break;
	case FUNC_MICROP_SET_FREQ_FADE:
		if (version == 0x0101)
			is_supported = 1;
		break;
	case FUNC_MICROP_PWM_AUTO:
		if (version >= 0x0102)
			is_supported = 1;
		break;
	case FUNC_MICROP_OFF_TIMER:
		if (version >= 0x0102)
			is_supported = 1;
		break;
	case FUNC_MICROP_ADC_SELECT:
		if (version >= 0x0103)
			is_supported = 1;
		break;
	case FUNC_MICROP_RESET_IF:
		if (version >= 0x0103)
			is_supported = 1;
		break;
	case FUNC_MICROP_INTR_IF:
		if (version >= 0x0103)
			is_supported = 1;
		break;
	case FUNC_MICROP_EN_SPI_AUTO_BACKLIGHT:
		if (version >= 0x0109)
			is_supported = 1;
		break;
	case FUNC_MICROP_PULL_UP:
		if ((version >= 0x010B) && (version < 0x010E))
			is_supported = 1;
		break;
	case FUNC_MICROP_LS_GPIO:
		if (version >= 0x010C)
			is_supported = 1;
		break;
	case FUNC_MICROP_PWM_TRACKBALL:
		if (version >= 0x010D)
			is_supported = 1;
		break;
	case FUNC_MICROP_PULL_UP1:
		if (((version >= 0x010E) && (version < 0x0200)) ||
			(version >= 0x0202))
			is_supported = 1;
		break;
	case FUNC_MICROP_ADC_INTR:
		if (version >= 0x0110)
			is_supported = 1;
		break;
	case FUNC_MICROP_DISABLE_INTR:
		if (((version >= 0x0110) && (version < 0x0200)) ||
			(version >= 0x0202))
			is_supported = 1;
		break;
	case FUNC_MICROP_LED_AUTO:
		if ((version >= 0x0111) && (version < 0x0200))
			is_supported = 1;
		break;
	}

	return is_supported;
}

static char *hex2string(uint8_t *data, int len)
{
	static char buf[101];
	int i;

	i = (sizeof(buf) - 1) / 4;
	if (len > i)
		len = i;

	for (i = 0; i < len; i++)
		sprintf(buf + i * 4, "[%02X]", data[i]);

	return buf;
}

static int i2c_read_block(struct i2c_client *client, uint8_t addr,
	uint8_t *data, int length)
{
	int retry;
	struct i2c_msg msgs[] = {
	{
		.addr = client->addr,
		.flags = 0,
		.len = 1,
		.buf = &addr,
	},
	{
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = length,
		.buf = data,
	}
	};

	for (retry = 1; retry <= I2C_READ_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msgs, 2) == 2)
			break;
		mdelay(10);
	}

	dev_dbg(&client->dev, "R [%02X] = %s\n", addr, hex2string(data, length));

	if (retry > I2C_READ_RETRY_TIMES) {
		dev_err(&client->dev, "i2c_read_block retry over %d\n",
			I2C_READ_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

static int i2c_write_block(struct i2c_client *client, uint8_t addr,
	uint8_t *data, int length)
{
	int retry;
	uint8_t buf[MICROP_I2C_WRITE_BLOCK_SIZE];
	int i;

	struct i2c_msg msg[] = {
	{
		.addr = client->addr,
		.flags = 0,
		.len = length + 1,
		.buf = buf,
	}
	};

	dev_dbg(&client->dev, "W [%02X] = %s\n", addr, hex2string(data, length));

	if (length + 1 > MICROP_I2C_WRITE_BLOCK_SIZE) {
		dev_err(&client->dev, "i2c_write_block length too long\n");
		return -E2BIG;
	}

	buf[0] = addr;
	for (i = 0; i < length; i++)
		buf[i+1] = data[i];

	for (retry = 1; retry <= I2C_WRITE_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		mdelay(10);
	}
	if (retry > I2C_WRITE_RETRY_TIMES) {
		dev_err(&client->dev, "i2c_write_block retry over %d\n",
			I2C_WRITE_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

static int microp_i2c_write_pin_mode(struct i2c_client *client,
	struct microp_led_data *ldata)
{
	uint8_t data[2];
	struct microp_i2c_client_data *cdata;

	cdata = i2c_get_clientdata(client);
	data[0] = ldata->pin_config->pin;

	if ((ldev_lcd_backlight == &ldata->ldev && !microp_i2c_is_backlight_on)
			|| (ldata->pin_config->suspend_off &&
			atomic_read(&cdata->suspended_now)))
		data[1] = 0;
	else
		data[1] = ldata->mode;

	return i2c_write_block(client, MICROP_I2C_CMD_PIN_MODE, data, 2);
}

static int microp_i2c_gpio_pwm_write(struct i2c_client *client,
	struct microp_led_data *ldata, unsigned is_on)
{
	uint8_t data[4];
	static int ret;
	static unsigned gpio_pwm_status = 0;
	unsigned tmp = 0;
	struct microp_i2c_client_data *cdata;
	cdata = i2c_get_clientdata(client);

	mutex_lock(&ldata->pin_mutex);
	ret = 0;
	tmp = (is_on)?1:0;
	if (tmp != gpio_pwm_status) {
		data[0] = ldata->pin_config->pin;
		data[1] = tmp;
		data[2] = ((ldata->pin_config->levels[0] >> 8) & 0xFF);
		data[3] = (ldata->pin_config->levels[0] & 0xFF);
		ret = i2c_write_block(client, MICROP_I2C_CMD_SET_GPIO_PWM_FUNC,
				data, 4);
		if (ret < 0) {
			dev_err(&client->dev,
				"%s: fail: MICROP_I2C_CMD_SET_GPIO_PWM_FUNC\n",
				__func__);
			goto exit;
		}
		if (!tmp) {
			data[0] = ldata->pin_config->pin;
			data[1] = tmp;
			ret = i2c_write_block(client, MICROP_I2C_CMD_PIN_MODE, data, 2);
			if (ret < 0) {
				dev_err(&client->dev,
					"%s: fail write: MICROP_I2C_CMD_PIN_MODE\n",
					__func__);
				goto exit;
			}
		}
		gpio_pwm_status = tmp;
	}
exit:
	mutex_unlock(&ldata->pin_mutex);
	return ret;
}

static int microp_i2c_write_pin_duty(struct i2c_client *client,
	struct microp_led_data *ldata)
{
	uint8_t data[3];
	uint16_t *levels;
	uint16_t *dutys;
	int duty, b1, b2, d1, d2, i;

	data[0] = ldata->pin_config->pin;
	data[1] = MICROP_I2C_PWM_MANUAL;
	data[2] = ldata->ldev.brightness;

	levels = ldata->pin_config->levels;
	dutys = ldata->pin_config->dutys;

	for (i = 9; i > 0; i--)
		if (data[2] >= levels[i])
			break;

	if (i == 9)
		duty = dutys[9];
	else if (i) {
		b1 = levels[i];
		d1 = dutys[i];
		b2 = levels[i + 1];
		d2 = dutys[i + 1];
		duty = (data[2] - b1) * (d2 - d1) / (b2 - b1) + d1;
	} else
		duty = dutys[0];

	data[2] = (uint8_t)duty;
	return i2c_write_block(client, MICROP_I2C_CMD_PIN_PWM, data, 3);
}

static void microp_i2c_suspend_led_control(struct i2c_client *client)
{
	struct microp_i2c_client_data *cdata;
	int i;

	cdata = i2c_get_clientdata(client);
	for (i = 0; i < cdata->num_led_data; i++) {
		if (!cdata->led_data[i].pin_config->suspend_off)
			continue;
		if (microp_i2c_write_pin_mode(client, &cdata->led_data[i]))
			dev_err(&client->dev,
				"led_brightness_set failed on set mode\n");
	}
}

static void microp_i2c_reset_microp(struct i2c_client *client)
{
	struct microp_i2c_platform_data *pdata;

	pdata = client->dev.platform_data;

	gpio_set_value(pdata->gpio_reset, 0);
	udelay(120);
	gpio_set_value(pdata->gpio_reset, 1);
	mdelay(5);
}

static int microp_i2c_version_check(struct i2c_client *client, uint8_t *data)
{
	if (data[0] == 0x01 || data[0] == 0x02)
		return 0;

	dev_info(&client->dev, "microp is in Bootloader [%02X][%02X]\n",
		data[0], data[1]);
	data[0] = 1;
	i2c_write_block(client, 0x02, data, 1);
	mdelay(250);
	microp_i2c_reset_microp(client);
	mdelay(20);

	return i2c_read_block(client, MICROP_I2C_CMD_VERSION, data, 2);
}

static void microp_i2c_clear_led_data(struct i2c_client *client)
{
	struct microp_i2c_client_data *cdata;
	int i;
	uint8_t brightness;

	cdata = i2c_get_clientdata(client);

	for (i = 0; i < cdata->num_led_data; i++) {
		mutex_lock(&cdata->led_data[i].pin_mutex);

		cdata->led_data[i].is_auto = 0;
		brightness = cdata->led_data[i].pin_config->init_value;
		cdata->led_data[i].ldev.brightness = brightness;
		if (brightness)
			cdata->led_data[i].mode = 1;
		else
			cdata->led_data[i].mode = 0;
		cdata->led_data[i].fade_timer = 0;
		cdata->led_data[i].off_timer = 0;
		cdata->led_data[i].skip_config = 0;

		mutex_unlock(&cdata->led_data[i].pin_mutex);
	}
}

static int microp_i2c_auto_backlight_set_interrupt_mode(struct i2c_client *client,
			uint8_t enabled)
{
	struct microp_i2c_platform_data *pdata;
	struct microp_i2c_client_data *cdata;
	uint8_t data[2] = {0, 0};
	int ret = 0;
	unsigned long delay_time, jiffy_now;

	pdata = client->dev.platform_data;
	cdata = i2c_get_clientdata(client);

	if (!microp_i2c_is_supported(FUNC_MICROP_EN_SPI_AUTO_BACKLIGHT,
			cdata->version)) {
		dev_warn(&client->dev, "%s: the version not support: 0x%X\n",
			__func__, cdata->version);
		return -EINVAL;
	}

	if (enabled)
		data[0] |= 0x4;
	else
		data[0] &= ~0x4;

	if (cdata->enable_reset_button)
		data[0] |= 0x8;

	if (pdata->ls_power)
		cancel_delayed_work(&auto_bl_delay_work);

	if (!enabled || !pdata->ls_power) {
		dev_info(&client->dev,  "%s: %d\n", __func__, enabled);
		ret = i2c_write_block(client, MICROP_I2C_CMD_MISC1, data, 1);
		if (ret != 0)
			dev_err(&client->dev,
				"%s: setting MICROP_I2C_CMD_MISC1 fail\n",
				__func__);
	} else {
		jiffy_now = jiffies;
		if (time_after_eq(jiffy_now, ls_power_on_jiffy + LS_DELAY))
			delay_time = 0;
		else
			delay_time = ls_power_on_jiffy + LS_DELAY - jiffy_now;
		queue_delayed_work(cdata->microp_queue,
				&auto_bl_delay_work, delay_time);
	}
	return ret;
}

static int microp_i2c_config_microp(struct i2c_client *client)
{
	struct microp_i2c_platform_data *pdata;
	struct microp_i2c_client_data *cdata;
	uint8_t data[20];
	uint8_t config;
	int pin, led, i, j;
	int ret;

	pdata = client->dev.platform_data;
	cdata = i2c_get_clientdata(client);

	for (i = 0, led = 0; i < pdata->num_pins; i++) {
		pin = pdata->pin_config[i].pin;
		config = pdata->pin_config[i].config;
		if (microp_i2c_is_gpo(config)) {
			if (pdata->pin_config[i].name) {
				if (!microp_i2c_is_supported(FUNC_MICROP_GPO, cdata->version))
					continue;
				if (cdata->led_data[led].skip_config) {
					led++;
					continue;
				}
				mutex_lock(&cdata->led_data[led].pin_mutex);

				data[0] = pin;
				data[1] = config;
				ret = i2c_write_block(client, MICROP_I2C_CMD_PIN_CONFIG, data, 2);
				if (ret)
					goto err_led;

				ret = microp_i2c_write_pin_mode(client, &cdata->led_data[led]);
				if (ret)
					goto err_led;

				mutex_unlock(&cdata->led_data[led].pin_mutex);
				led++;
			} else {
				data[0] = pin;
				data[1] = config;
				ret = i2c_write_block(client, MICROP_I2C_CMD_PIN_CONFIG, data, 2);
				if (ret)
					goto exit;
			}
		} else if (microp_i2c_is_pwm(config)) {
			if (!microp_i2c_is_supported(FUNC_MICROP_PWM, cdata->version))
				continue;
			mutex_lock(&cdata->led_data[led].pin_mutex);

			data[0] = pin;

			if (cdata->led_data[led].skip_config == 0) {
				data[1] = config;
				ret = i2c_write_block(client, MICROP_I2C_CMD_PIN_CONFIG, data, 2);
				if (ret)
					goto err_led;

				ret = microp_i2c_write_pin_mode(client, &cdata->led_data[led]);
				if (ret)
					goto err_led;

				data[1] = MICROP_I2C_PWM_FREQ;
				data[2] = pdata->pin_config[i].freq;
				if (microp_i2c_is_supported(FUNC_MICROP_SET_FREQ_FADE,
					cdata->version)) {
					data[3] = 0;
					data[4] = 0;
					j = 5;
				} else {
					j = 3;
				}
				ret = i2c_write_block(client, MICROP_I2C_CMD_PIN_PWM, data, j);
				if (ret)
					goto err_led;

				if (!microp_i2c_is_supported(FUNC_MICROP_SET_FREQ_FADE,
					cdata->version)) {
					data[1] = MICROP_I2C_PWM_FADE;
					data[2] = cdata->led_data[led].fade_timer;
					ret = i2c_write_block(client, MICROP_I2C_CMD_PIN_PWM, data, 3);
					if (ret)
						goto err_led;
				}
			}

			if (microp_i2c_is_supported(FUNC_MICROP_PWM_AUTO, cdata->version)) {
				if (pdata->pin_config[i].i_am_jogball_function &&
					microp_i2c_is_supported(FUNC_MICROP_PWM_TRACKBALL, cdata->version)) {
					data[1] = MICROP_I2C_PWM_FADE;
					ret = i2c_write_block(client, MICROP_I2C_CMD_PIN_CONFIG, data, 2);
					if (ret)
						goto err_led;
				} else {
					data[1] = MICROP_I2C_PWM_LEVELS;
					for (j = 0; j < 10; j++)
						data[j+2] = pdata->pin_config[i].dutys[j];
					ret = i2c_write_block(client, MICROP_I2C_CMD_PIN_PWM, data, 12);
					if (ret)
						goto err_led;
				}
			}

			if (cdata->led_data[led].skip_config == 0) {
				if (cdata->led_data[led].is_auto &&
					microp_i2c_is_supported(FUNC_MICROP_PWM_AUTO, cdata->version)) {
					data[1] = MICROP_I2C_PWM_AUTO;
					ret = i2c_write_block(client, MICROP_I2C_CMD_PIN_PWM, data, 2);
					if (ret)
						goto err_led;
				} else {
					ret = microp_i2c_write_pin_duty(client, &cdata->led_data[led]);
					if (ret)
						goto err_led;
				}
			}
			mutex_unlock(&cdata->led_data[led].pin_mutex);
			led++;
		} else if (microp_i2c_is_adc(config)) {
			if (!microp_i2c_is_supported(FUNC_MICROP_PWM_AUTO, cdata->version))
				continue;
			if (microp_i2c_is_supported(FUNC_MICROP_ADC_SELECT,
				cdata->version) && (pin == 16 || pin == 17)) {
				data[0] = pin;

				data[1] = config;
				ret = i2c_write_block(client, MICROP_I2C_CMD_PIN_CONFIG, data,
						2);
				if (ret)
					goto exit;
			}
			for (j = 0; j < 10; j++) {
				data[j * 2] = (uint8_t)(pdata->pin_config[i].levels[j] >> 8);
				data[j * 2 + 1] = (uint8_t)(pdata->pin_config[i].levels[j]);
			}
			ret = i2c_write_block(client, MICROP_I2C_CMD_ADC_TABLE, data, 20);
			if (ret)
				goto exit;
		} else if (microp_i2c_is_intr(config)) {
			if (!microp_i2c_is_supported(FUNC_MICROP_INTR_IF, cdata->version))
				continue;
			data[0] = pin;

			data[1] = MICROP_PIN_CONFIG_INTR;
			ret = i2c_write_block(client, MICROP_I2C_CMD_PIN_CONFIG, data, 2);
			if (ret)
				goto exit;

			data[1] = pdata->pin_config[i].mask[0];
			data[2] = pdata->pin_config[i].mask[1];
			data[3] = pdata->pin_config[i].mask[2];
			data[4] = pdata->pin_config[i].setting[0];
			data[5] = pdata->pin_config[i].setting[1];
			data[6] = pdata->pin_config[i].setting[2];
			data[7] = microp_i2c_is_intr_all(config);
			ret = i2c_write_block(client, MICROP_I2C_CMD_INTR_SETTING, data, 8);
			if (ret)
				goto exit;
		} else if (microp_i2c_is_pullup(config)) {
			if (!microp_i2c_is_supported(FUNC_MICROP_PULL_UP, cdata->version))
				continue;
			data[0] = pin;
			data[1] = 1;
			ret = i2c_write_block(client, MICROP_I2C_CMD_PIN_PULL_UP, data, 2);
			if (ret)
				goto exit;
		} else if (microp_i2c_is_pullup1(config)) {
			if (!microp_i2c_is_supported(FUNC_MICROP_PULL_UP1, cdata->version))
				continue;
			data[0] = pdata->pin_config[i].mask[0];
			data[1] = pdata->pin_config[i].mask[1];
			data[2] = pdata->pin_config[i].mask[2];
			ret = i2c_write_block(client, MICROP_I2C_CMD_PIN_PULL_UP1, data, 3);
			if (ret)
				goto exit;
		} else if (microp_i2c_is_ls_gpio(config)) {
			if (!microp_i2c_is_supported(FUNC_MICROP_LS_GPIO, cdata->version))
				continue;
			mutex_lock(&cdata->led_data[led].pin_mutex);
			data[0] = pin;
			data[1] = 0x1;
			ret = i2c_write_block(client, MICROP_I2C_CMD_PIN_CONFIG, data, 2);
			if (ret)
				goto err_led;
			mutex_unlock(&cdata->led_data[led].pin_mutex);
			led++;
			cdata->microp_gpio_pwm_is_enabled = 1;
		} else if (microp_i2c_is_uP_adc(config)) {
			if (!microp_i2c_is_supported(FUNC_MICROP_ADC_INTR, cdata->version))
				continue;
			data[0] = pdata->pin_config[i].intr_pin;
			data[1] = pdata->pin_config[i].adc_pin;
			ret = i2c_write_block(client, MICROP_I2C_CMD_ADC_INTR, data, 2);
			if (ret)
				goto exit;

			INIT_DELAYED_WORK(&hpin_work, microp_i2c_hpin_work_func);
		}

		if (pdata->pin_config[i].led_auto) {
			if (!microp_i2c_is_supported(FUNC_MICROP_LED_AUTO,
					cdata->version))
				continue;
			data[0] = pin;
			data[1] = (uint8_t)(pdata->pin_config[i].levels[0] >> 8);
			data[2] = (uint8_t)(pdata->pin_config[i].levels[0]);
			data[3] = (uint8_t)(pdata->pin_config[i].levels[1] >> 8);
			data[4] = (uint8_t)(pdata->pin_config[i].levels[1]);
			ret = i2c_write_block(client,
				MICROP_I2C_CMD_LED_AUTO_TABLE, data, 5);
			/* make sure FW has finished this command handle */
			mdelay(1);
			if (ret)
				goto exit;
		}
	}

	return 0;

err_led:
	mutex_unlock(&cdata->led_data[led].pin_mutex);

exit:
	return ret;
}

/* APIs for drivers */
int get_adc_value(uint8_t pin, int *value)
{
	uint8_t buffer[3];
	int ret = 0, loop_i = 0;
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;

	client = private_microp_client;

	if (!client) {
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return -EIO;
	}

	cdata = i2c_get_clientdata(client);
	memset(buffer, 0x0, sizeof(buffer));
	buffer[0] = pin;

	mutex_lock(&cdata->microp_i2c_mutex);
	if (i2c_write_block(client, MICROP_I2C_CMD_ADC_READ_DATA_CMD,
			buffer, 1) >= 0) {
		for (loop_i = 0; loop_i < 10; loop_i++) {
			udelay(200);
			memset(buffer, 0x0, sizeof(buffer));
			if (i2c_read_block(client,
					MICROP_I2C_CMD_ADC_READ_DATA,
					buffer, 3) < 0) {
				ret = -EIO;
				break;
			}

			if (buffer[0] == (1 << 4 | pin))
				break;
		}
	} else
		ret = -EIO;
	mutex_unlock(&cdata->microp_i2c_mutex);

	if (loop_i != 10 && ret >= 0)
		*value = ((int)buffer[1] << 8 | (int)buffer[2]);
	else
		*value = -EIO;

	return ret;
}

int microp_get_3_button_value(void *argu)
{
	int value_tmp = 0;
	int ret = -1;
	unsigned char loop_i;
	unsigned short adc_level[4][2] =
		{{200, 0x3FF}, {0, 33}, {38, 82}, {95, 167} };
	int *detect_key = (int *) argu;

	ret = get_adc_value(remote_adc_read_channel, &value_tmp);
	if (ret < 0) {
		printk(KERN_ERR "%s: cannot get data\n", __func__);
		return ret;
	}
	ret = -1;
	for (loop_i = 0; loop_i < 4; loop_i++) {
		if (value_tmp >= adc_level[loop_i][0] &&
			value_tmp <= adc_level[loop_i][1]) {
			ret = loop_i;
			break;
		}
	}
	if (ret < 0)
		printk(KERN_ERR "%s: get wrong value: 0x%X\n",
			__func__, value_tmp);
#ifdef CONFIG_HTC_HEADSET_V1
	else {
		printk(KERN_INFO "%s: data: 0x%2.2X, ret: 0x%2.2X\n",
			__func__, value_tmp, ret);
		if (*detect_key)
			cnf_driver_event("H2W_3button", &ret);
	}
#endif
	return ret;
}

void get_3_button_key(void)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	struct microp_i2c_platform_data *pdata;
	uint8_t data[3], i;
	uint8_t config = 0, pin = 0;
	int adc_value = 0;
	int ret;

	client = private_microp_client;
	cdata = i2c_get_clientdata(client);
	pdata = client->dev.platform_data;

	for (i = 0; i < pdata->num_pins; i++) {
		pin = i;
		config = pdata->pin_config[i].config;
		if (microp_i2c_is_uP_adc(config))
			break;
	}

	memset(data, 0x00, sizeof(data));
	if (i2c_read_block(client, MICROP_I2C_CMD_ADC_READ_DATA, data, 3) < 0) {
		dev_err(&client->dev, "%s: read adc fail\n", __func__);
	} else {
		adc_value = ((int)data[1] << 8 | (int)data[2]);
		ret = -1;
		if (adc_value & 0x8000)
			goto exit;
		for (i = 0; i < 8; i = i+2) {
			if ((adc_value >= pdata->pin_config[pin].levels[i])
				&& (adc_value <= pdata->pin_config[pin].levels[i+1])) {
				ret = i/2;
				break;
			}
		}

		if (ret < 0)
			dev_err(&client->dev, "%s: get wrong value: 0x%X\n",
				__func__, adc_value);

#ifdef CONFIG_HTC_HEADSET_V1
		else {
			printk(KERN_DEBUG "Remote key data: 0x%2.2X, ret: 0x%2.2X\n",
				adc_value, ret);
			if (is_35mm_hpin)
				cnf_driver_event("H2W_3button", &ret);
		}
#endif
	}
exit:;
}

int set_microp_mic_intr(int enable)
{
	struct microp_i2c_platform_data *pdata;
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	int ret = -1;
	uint8_t config = 0;
	int i, pin = 0, pin_intr = 0, pin_adc = 0, intr_i = 0;
	uint8_t data[8];

	client = private_microp_client;
	if (!client)	{
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return -1;
	}
	cdata = i2c_get_clientdata(client);
	pdata = client->dev.platform_data;

	if (!microp_i2c_is_supported(FUNC_MICROP_DISABLE_INTR, cdata->version))	{
		dev_info(&client->dev,  "%s: the version not support: 0x%X\n",
				__func__, cdata->version);
		return -EINVAL;
	}

	memset(data, 0x00, sizeof(data));
	for (i = 0; i < pdata->num_pins; i++) {
		pin = pdata->pin_config[i].pin;
		config = pdata->pin_config[i].config;
		if (microp_i2c_is_mic(config))
			pin_adc = pin;
		else if (microp_i2c_is_intr(config)) {
			pin_intr = pin;
			intr_i = i;
		}
	}

	data[0] = pin_intr;
	data[1] = pdata->pin_config[intr_i].mask[0];
	data[2] = pdata->pin_config[intr_i].mask[1];
	data[3] = pdata->pin_config[intr_i].mask[2];
	data[4] = pdata->pin_config[intr_i].setting[0];
	data[5] = pdata->pin_config[intr_i].setting[1];
	data[6] = pdata->pin_config[intr_i].setting[2];
	data[7] = microp_i2c_is_intr_all(config);
	if (pin_adc < 8)
		data[1] = enable ? (data[1] | (1 << pin_adc))
			: (data[1] & ~(1 << pin_adc));
	else if (pin_adc < 16)
		data[2] = enable ? (data[2] | (1 << (pin_adc - 8)))
			: (data[2] & ~(1 << (pin_adc - 8)));
	else {
		if (pin_adc == 16)
			data[3] = enable ? (data[3] | 0x02) : (data[3] & ~0x02);
		else
			data[3] = enable ? (data[3] | 0x08) : (data[3] & ~0x08);
	}
	ret = i2c_write_block(client, MICROP_I2C_CMD_INTR_SETTING, data, 8);
	if (ret < 0)
		dev_err(&client->dev, "%s: set microp mic interrupt error\n",
			__func__);

	return ret;
}

static int microp_get_pin_status(uint8_t query_pin)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	uint8_t data[6];
	int ret;
	int8_t pin_alloc[3][8] = {
			{   0,  1,  2,  3,  4,  5,  6,  7 }, /* PD */
			{   8,  9, 10, 11, 12, 13, 14, 15 }, /* PB */
			{  -1, 16, -1, 17, -1, -1, -1, -1 }, /* PC */
		};
	static uint8_t loop_i, loop_j;
	static int8_t tmp_pin, tmp_pinset;
	client = private_microp_client;
	cdata = i2c_get_clientdata(client);

	tmp_pin = -1;
	tmp_pinset = -1;
	memset(data, 0x0, sizeof(data));
	for (loop_i = 0; loop_i < 3; loop_i++) {
		for (loop_j = 0; loop_j < 8; loop_j++) {
			if (pin_alloc[loop_i][loop_j] == query_pin) {
				tmp_pin = loop_j;
				tmp_pinset = loop_i;
				break;
			}
		}
		if (tmp_pin >= 0 && tmp_pinset >= 0)
			break;
	}
	if (tmp_pin >= 0 && tmp_pinset >= 0) {
		ret = i2c_read_block(client, MICROP_I2C_CMD_READ_PIN, data, 6);
		if (ret == 0)
			ret = (data[tmp_pinset] & (0x1 << tmp_pin))?1:0;
	} else
		ret = -1;

	return ret;
}

static int microp_get_mic_value(void *argu)
{
	int ret = 0;
	uint8_t config = 0, data[2] = {0, 0};
	int i, pin = 0, pin_adc = -1, node_mic = -1, node_adc = -1;
	struct microp_i2c_platform_data *pdata;
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;

	client = private_microp_client;
	if (!client)	{
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return -EIO;
	}
	pdata = client->dev.platform_data;
	cdata = i2c_get_clientdata(client);

	for (i = 0; i < pdata->num_pins; i++) {
		pin = pdata->pin_config[i].pin;
		config = pdata->pin_config[i].config;
		if (microp_i2c_is_mic(config)) {
			pin_adc = pin;
			node_mic = i;
		}

		if (microp_i2c_is_uP_adc(config))
			node_adc = i;
	}
	if (pin_adc < 0) {
		dev_err(&client->dev, "%s: cannot get adc pin\n", __func__);
		return -EIO;
	}

	ret = microp_get_pin_status(pin_adc);
	if (ret < 0)
		dev_err(&client->dev, "%s: get microp pin value error: 0x%X\n",
				__func__, ret);
	else {
		if (ret == pdata->pin_config[node_mic].init_value) {
			ret = 0;
			pdata->microp_mic_status = 0;
		} else {
			if (node_adc < 0) {
				dev_err(&client->dev, "%s: cannot get adc int\n", __func__);
				return -EIO;
			}

			if (!microp_i2c_is_supported(FUNC_MICROP_ADC_INTR,
					cdata->version)) {
				dev_err(&client->dev,
					"%s: microp version doesn't support ADC INT: 0x%X\n",
					__func__, cdata->version);
				pdata->microp_mic_status = 0;
				return -EIO;
			}

			pdata->microp_mic_status = 1;
			ret = 1;
		}
	}

	cancel_delayed_work(&hpin_work);
	schedule_delayed_work(&hpin_work, 0);
	data[0] = pdata->pin_config[node_adc].intr_pin;
	data[1] = pdata->pin_config[node_adc].adc_pin;

	if (i2c_write_block(client,
		MICROP_I2C_CMD_ADC_INTR, data, 2) < 0) {
		dev_err(&client->dev,
			"%s: set microp ADC INT error\n",
			__func__);
		return -EIO;
	}

	dev_dbg(&client->dev, "%s: microp_mic_status (plug in): 0x%d\n",
			__func__, pdata->microp_mic_status);
	is_35mm_hpin = 1;
	return ret;
}

static void microp_unplug_mic(void *argu)
{
	struct microp_i2c_platform_data *pdata;
	struct i2c_client *client;
	int ret;

	is_35mm_hpin = 0;
	client = private_microp_client;
	if (!client)	{
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return;
	}
	pdata = client->dev.platform_data;

	cancel_delayed_work(&hpin_work);
	ret = set_microp_mic_intr(0);
	if (ret < 0) {
		dev_err(&client->dev, "%s: set microp mic interrupt error\n", __func__);
		return;
	}

	pdata->microp_mic_status = 0;
	dev_dbg(&client->dev, "%s: microp_mic_status (plug out): 0x%d\n", __func__, pdata->microp_mic_status);

	return;
}


/* DEVICE_ATTR */
static ssize_t microp_i2c_reset_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	buf[0] = 0;
	return 0;
}

static ssize_t microp_i2c_reset_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	int val;

	val = -1;
	sscanf(buf, "%u", &val);
	if (val != 1)
		return -EINVAL;

	client = to_i2c_client(dev);
	cdata = i2c_get_clientdata(client);

	microp_i2c_reset_microp(client);
	microp_i2c_clear_led_data(client);
	microp_i2c_config_microp(client);

	return count;
}

static DEVICE_ATTR(reset, 0644, microp_i2c_reset_show,
	microp_i2c_reset_store);

static ssize_t microp_i2c_version_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct microp_i2c_client_data *cdata;

	cdata = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%04X\n", cdata->version);
}

static ssize_t microp_i2c_version_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	return -EINVAL;
}

static DEVICE_ATTR(version, 0644, microp_i2c_version_show,
	microp_i2c_version_store);

static ssize_t microp_i2c_status_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	uint8_t data[6];
	int ret;

	client = to_i2c_client(dev);
	cdata = i2c_get_clientdata(client);

	if (i2c_read_block(client, MICROP_I2C_CMD_READ_PIN, data, 6)) {
		buf[0] = 0;
		ret = 0;
	} else
		ret = sprintf(buf,
			"PIND %02X, PINB %02X, PINC %02X, ADC[%d] => level %d\n",
			data[0], data[1], data[2], (data[4] << 8 | data[5]), data[3]);

	return ret;
}

static ssize_t microp_i2c_status_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	return -EINVAL;
}

static DEVICE_ATTR(status, 0644, microp_i2c_status_show,
	microp_i2c_status_store);

static int microp_jogball_setting_func(struct i2c_client *client,
	struct microp_led_data *ldata)
{
	struct microp_i2c_client_data *cdata;
	uint8_t data[3];
	uint8_t brightness_func;
	int ret = 0;

	cdata = i2c_get_clientdata(client);

	memset(data, 0x0, sizeof(data));
	data[0] = ldata->pin_config->pin;
	brightness_func = ldata->ldev.brightness;

	if (!brightness_func) { /* disable */
		data[1] = 0x00;
		ret = i2c_write_block(client, MICROP_I2C_CMD_PIN_MODE, data, 2);
		if (ret < 0) {
			dev_err(&client->dev, "%s failed on set disable\n", __func__);
			goto fail_exit;
		}
	} else {
		data[1] = 0x00;
		data[2] = 0x01;
		ret = i2c_write_block(client, MICROP_I2C_CMD_PIN_PWM, data, 3);
		if (ret < 0) {
			dev_err(&client->dev, "%s failed on set flash quickly mode\n", __func__);
			goto fail_exit;
		}

		/* setting mode: flash: 3, breathing: 7 */
		if (brightness_func == 3)
			data[1] = 0x05;
		else
			data[1] = 0x06;
		ret = i2c_write_block(client, MICROP_I2C_CMD_PIN_PWM, data, 2);
		if (ret < 0) {
			dev_err(&client->dev, "%s failed on set jogball mode: 0x%2.2X\n", __func__, data[1]);
			goto fail_exit;
		}

		/* setting breathing timeout */
		if (brightness_func == 7) {
			data[1] = 0x0A; /* 10 min */
			data[2] = 0x00;
			ret = i2c_write_block(client, MICROP_I2C_CMD_PIN_OFF, data, 3);
			if (ret < 0) {
				dev_err(&client->dev, "%s failed on set breathing timeout\n", __func__);
				goto fail_exit;
			}
		}
	}
fail_exit:

	return ret;
}

static void microp_jogball_brightness_set(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
	struct microp_led_data *ldata;
	struct i2c_client *client;

	ldata = container_of(led_cdev, struct microp_led_data, ldev);
	client = to_i2c_client(led_cdev->dev->parent);
	mutex_lock(&ldata->pin_mutex);
	if (brightness == 0 || brightness == 3 || brightness == 7) {
		ldata->ldev.brightness = brightness;
		if (microp_jogball_setting_func(client, ldata) < 0)
			dev_err(&client->dev, "%s: setting jogball function fail\n", __func__);
	} else
		dev_warn(&client->dev, "%s: unknown value: %d\n", __func__, brightness);
	mutex_unlock(&ldata->pin_mutex);
}

static enum led_brightness microp_led_brightness_get(struct led_classdev *led_cdev)
{
	struct microp_led_data *ldata;
	struct i2c_client *client;
	uint8_t data[6];
	enum led_brightness ret;

	ldata = container_of(led_cdev, struct microp_led_data, ldev);
	client = to_i2c_client(led_cdev->dev->parent);

	mutex_lock(&ldata->pin_mutex);

	if ((microp_i2c_is_pwm(ldata->pin_config->config) && !ldata->pin_config->auto_if_on && ldata->is_auto) ||
	      ldata->pin_config->auto_if_on) {
		if (i2c_read_block(client, MICROP_I2C_CMD_READ_PIN, data, 6))
			ret = 0;
		else
			ret = ldata->pin_config->levels[data[3]];
	} else
		ret = ldata->ldev.brightness;

	mutex_unlock(&ldata->pin_mutex);

	return ret;
}

static void microp_led_brightness_set(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
	struct microp_led_data *ldata;
	unsigned percent;
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	uint8_t pin;
	uint8_t config;
	uint8_t data[2];
	int ret;

	ldata = container_of(led_cdev, struct microp_led_data, ldev);
	client = to_i2c_client(led_cdev->dev->parent);
	cdata = i2c_get_clientdata(client);

	if (brightness > 255)
		brightness = 255;
	ldata->ldev.brightness = brightness;

	percent = ((int)brightness * 100) / 255;

	mutex_lock(&ldata->pin_mutex);

	if (ldata->is_auto)
		ldata->is_auto = 0;

	if (ldata->mode == 0 && brightness) {
		ldata->mode = 1;
		if (ldata->pin_config->auto_if_on)
			ldata->is_auto = 1;
	} else if (brightness == 0)
		ldata->mode = 0;

	pin = ldata->pin_config->pin;
	config = ldata->pin_config->config;

	data[0] = pin;

	if (microp_i2c_is_ls_gpio(ldata->pin_config->config)) {
		mutex_unlock(&ldata->pin_mutex);
		ret = microp_i2c_gpio_pwm_write(client, ldata, ldata->ldev.brightness);
		if(ret < 0)
			dev_err(&client->dev, "%s: gpio_pwm_write failed: %d\n", __func__, ret);
		return;
	} else
	if (microp_i2c_is_pwm(config)) {
		if (ldata->is_auto) {
			data[1] = MICROP_I2C_PWM_AUTO;
			if (i2c_write_block(client, MICROP_I2C_CMD_PIN_PWM, data, 2))
				dev_err(&client->dev, "led_brightness_set failed on set auto\n");
		} else
			if (microp_i2c_write_pin_duty(client, ldata))
				dev_err(&client->dev, "led_brightness_set failed on set manual\n");
	}

	if (microp_i2c_write_pin_mode(client, ldata))
		dev_err(&client->dev, "led_brightness_set failed on set mode\n");

	if (ldev_lcd_backlight == &ldata->ldev)
		htc_pwrsink_set(PWRSINK_BACKLIGHT, percent);

	mutex_unlock(&ldata->pin_mutex);

}

static ssize_t microp_i2c_led_auto_brightness_show(
				struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev;
	struct microp_led_data *ldata;
	struct i2c_client *client;
	uint8_t data[6];
	int ret;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct microp_led_data, ldev);
	client = to_i2c_client(dev->parent);

	mutex_lock(&ldata->pin_mutex);

	if (i2c_read_block(client, MICROP_I2C_CMD_READ_PIN, data, 6)) {
		buf[0] = 0;
		ret = 0;
	} else
		ret = sprintf(buf, "%d\n", ldata->pin_config->levels[data[3]]);

	mutex_unlock(&ldata->pin_mutex);

	return ret;
}

static ssize_t microp_i2c_led_auto_brightness_store(
				struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(auto_brightness, 0644,
						microp_i2c_led_auto_brightness_show,
						microp_i2c_led_auto_brightness_store);

static ssize_t microp_i2c_led_blink_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct microp_led_data *ldata;
	int blink;

	ldata = container_of(dev_get_drvdata(dev), struct microp_led_data, ldev);

	mutex_lock(&ldata->pin_mutex);

	if (ldata->mode > 1)
		blink = ldata->mode - 1;
	else
		blink = 0;

	mutex_unlock(&ldata->pin_mutex);

	return sprintf(buf, "%d\n", blink);
}

static ssize_t microp_i2c_led_blink_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct microp_led_data *ldata;
	struct i2c_client *client;
	int val;

	val = -1;
	sscanf(buf, "%u", &val);

	if (val < 0 || val > 255)
		return -EINVAL;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct microp_led_data, ldev);
	client = to_i2c_client(dev->parent);

	mutex_lock(&ldata->pin_mutex);

	if (val)
		ldata->mode = val + 1;
	else {
		if (led_cdev->brightness)
			ldata->mode = 1;
		else
			ldata->mode = 0;
	}

	if (microp_i2c_write_pin_mode(client, ldata))
		dev_err(&client->dev, "%s set blink failed\n", ldata->pin_config->name);

	mutex_unlock(&ldata->pin_mutex);

	return count;
}

static DEVICE_ATTR(blink, 0644, microp_i2c_led_blink_show,
	microp_i2c_led_blink_store);

static ssize_t microp_i2c_led_fade_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct microp_led_data *ldata;
	int fade;

	ldata = container_of(dev_get_drvdata(dev), struct microp_led_data, ldev);

	mutex_lock(&ldata->pin_mutex);

	fade = ldata->fade_timer;

	mutex_unlock(&ldata->pin_mutex);

	return sprintf(buf, "%d\n", fade);
}

static ssize_t microp_i2c_led_fade_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct microp_led_data *ldata;
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	uint8_t data[5];
	int val;

	val = -1;
	sscanf(buf, "%u", &val);
	if (val < 0 || val > 255)
		return -EINVAL;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct microp_led_data, ldev);
	client = to_i2c_client(dev->parent);
	cdata = i2c_get_clientdata(client);

	mutex_lock(&ldata->pin_mutex);

	ldata->fade_timer = val;

	if (microp_i2c_is_supported(FUNC_MICROP_SET_FREQ_FADE, cdata->version)) {
		data[0] = ldata->pin_config->pin;
		data[1] = MICROP_I2C_PWM_FREQ;
		data[2] = ldata->pin_config->freq;
		data[3] = ldata->fade_timer;
		data[4] = ldata->fade_timer;
		if (i2c_write_block(client, MICROP_I2C_CMD_PIN_PWM, data, 5))
			dev_err(&client->dev, "%s set fade %d failed\n",
				ldata->pin_config->name, val);
	} else {
		data[0] = ldata->pin_config->pin;
		data[1] = MICROP_I2C_PWM_FADE;
		data[2] = ldata->fade_timer;
		if (i2c_write_block(client, MICROP_I2C_CMD_PIN_PWM, data, 3))
			dev_err(&client->dev, "%s set fade %d failed\n",
				ldata->pin_config->name, val);
	}

	mutex_unlock(&ldata->pin_mutex);

	return count;
}

static DEVICE_ATTR(fade, 0644, microp_i2c_led_fade_show,
	microp_i2c_led_fade_store);

static ssize_t microp_i2c_led_off_timer_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	buf[0] = 0;
	return 0;
}

static ssize_t microp_i2c_led_off_timer_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct microp_led_data *ldata;
	struct i2c_client *client;
	uint8_t data[5];
	int min, sec;

	min = -1;
	sec = -1;
	sscanf(buf, "%d %d", &min, &sec);

	if (min < 0 || min > 255)
		return -EINVAL;
	if (sec < 0 || sec > 255)
		return -EINVAL;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct microp_led_data, ldev);
	client = to_i2c_client(dev->parent);

	mutex_lock(&ldata->pin_mutex);

	ldata->off_timer = (min << 8) | sec;

	data[0] = ldata->pin_config->pin;
	data[1] = min;
	data[2] = sec;
	if (i2c_write_block(client, MICROP_I2C_CMD_PIN_OFF, data, 3))
		dev_err(&client->dev,
			"%s set off_timer %d min %d sec failed\n",
			ldata->pin_config->name, min, sec);

	mutex_unlock(&ldata->pin_mutex);

	return count;
}

static DEVICE_ATTR(off_timer, 0644, microp_i2c_led_off_timer_show,
	microp_i2c_led_off_timer_store);

static ssize_t microp_i2c_led_auto_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct microp_led_data *ldata;
	int is_auto;

	ldata = container_of(dev_get_drvdata(dev), struct microp_led_data, ldev);

	mutex_lock(&ldata->pin_mutex);

	is_auto = ldata->is_auto;

	mutex_unlock(&ldata->pin_mutex);

	return sprintf(buf, "%d\n", is_auto);
}

static ssize_t microp_i2c_led_auto_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct microp_led_data *ldata;
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	uint8_t data[6];
	uint8_t level;
	int val;

	val = -1;
	sscanf(buf, "%u", &val);
	if (val < 0 || val > 1)
		return -EINVAL;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct microp_led_data, ldev);
	client = to_i2c_client(dev->parent);
	cdata = i2c_get_clientdata(client);

	mutex_lock(&ldata->pin_mutex);

	if (ldata->is_auto == val)
		goto exit_unlock;

	ldata->is_auto = val;

	if (i2c_read_block(client, MICROP_I2C_CMD_READ_PIN, data, 6)) {
		dev_err(&client->dev, "led_auto_store failed on read level\n");
		dev_err(&client->dev, "led_auto_store set to default level 4\n");
		level = 4;
	} else
		level = data[3];

	/*Set fade timer*/
	if (strcmp(ldata->ldev.name, "lcd-backlight") == 0) {
		if (ldata->is_auto)
			ldata->fade_timer = 0;
		else
			ldata->fade_timer = 10;

		if (microp_i2c_is_supported(FUNC_MICROP_SET_FREQ_FADE, cdata->version)) {
			data[0] = ldata->pin_config->pin;
			data[1] = MICROP_I2C_PWM_FREQ;
			data[2] = ldata->pin_config->freq;
			data[3] = ldata->fade_timer;
			data[4] = ldata->fade_timer;
			if (i2c_write_block(client, MICROP_I2C_CMD_PIN_PWM, data, 5))
				dev_err(&client->dev, "%s set fade %d failed\n",
					ldata->pin_config->name, val);
		} else {
			data[0] = ldata->pin_config->pin;
			data[1] = MICROP_I2C_PWM_FADE;
			data[2] = ldata->fade_timer;
			if (i2c_write_block(client, MICROP_I2C_CMD_PIN_PWM, data, 3))
				dev_err(&client->dev, "%s set fade %d failed\n",
					ldata->pin_config->name, val);
		}
	}

	/*Set auto */
	data[0] = ldata->pin_config->pin;
	if (ldata->is_auto) {
		data[1] = MICROP_I2C_PWM_AUTO;
		if (i2c_write_block(client, MICROP_I2C_CMD_PIN_PWM, data, 2)) {
			dev_err(&client->dev, "pwm: led_auto_store failed on set auto\n");
			goto exit_unlock;
		}
	} else {
		ldata->ldev.brightness = ldata->pin_config->levels[level];
		if (microp_i2c_write_pin_duty(client, ldata)) {
			dev_err(&client->dev, "led_auto_store failed on set manual\n");
			goto exit_unlock;
		}
	}

	if (ldata->mode == 0) {
		ldata->mode = 1;
		if (microp_i2c_write_pin_mode(client, ldata)) {
			dev_err(&client->dev, "led_auto_store failed on set mode\n");
			goto exit_unlock;
		}
	}

	if ((strcmp(ldata->ldev.name, "lcd-backlight") == 0)
			&& ldata->is_auto && microp_i2c_is_backlight_on)
		schedule_delayed_work(&fade_work, FADE_DELAY);

exit_unlock:
	mutex_unlock(&ldata->pin_mutex);

	return count;
}

static DEVICE_ATTR(auto, 0644, microp_i2c_led_auto_show,
	microp_i2c_led_auto_store);

static ssize_t microp_i2c_light_sensor_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	uint8_t data[6];
	int ret;

	client = to_i2c_client(dev);
	cdata = i2c_get_clientdata(client);

	if (i2c_read_block(client, MICROP_I2C_CMD_READ_PIN, data, 6)) {
		buf[0] = 0;
		ret = 0;
	} else
		ret = sprintf(buf, "0x%X\n", data[4] << 8 | data[5]);

	return ret;
}

static DEVICE_ATTR(light_sensor, 0444, microp_i2c_light_sensor_show,
	NULL);

#ifdef ENABLE_READ_35MM_ADC_VALUE_FROM_ATTR_FILE
static ssize_t show_adc_value(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int ret = 0, value_tmp = 0;
	struct microp_pin_config *dev_config = dev->driver_data;
	struct i2c_client *client;

	client = to_i2c_client(dev);

	if (!dev_config) {
		dev_err(&client->dev, "%s: data is empty\n", __func__);
		return 0;
	}
	dev_dbg(&client->dev, "%s: name: %s, pin: %d\n", __func__, dev_config->name, dev_config->adc_pin);
	if (get_adc_value(dev_config->adc_pin, &value_tmp) < 0)
		ret = 0;
	else
		ret = sprintf(buf, "0x%X\n", value_tmp);
	return ret;
}
static DEVICE_ATTR(adc_value, 0444, show_adc_value, NULL);
#endif


static struct led_trigger light_sensor_trigger = {
	.name     = "light-sensor-trigger",
};

static void microp_auto_backlight_function(void)
{
	struct i2c_client *client;
	uint8_t buffer[1], buffer_tmp[6];
	struct microp_led_data *ldata;
	struct microp_i2c_client_data *cdata;

	client = private_microp_client;
	cdata = i2c_get_clientdata(client);

	ldata = container_of(ldev_lcd_backlight, struct microp_led_data, ldev);

	/* Wait for Framework event polling ready */
	if (ls_enable_num == 0) {
		ls_enable_num = 1;
		msleep(300);
	}

	memset(buffer, 0x0, sizeof(buffer));
	memset(buffer_tmp, 0x0, sizeof(buffer_tmp));
	if (i2c_read_block(client, MICROP_I2C_CMD_READ_PIN, buffer_tmp, 6) < 0)
		dev_err(&client->dev, "%s: read adc fail\n", __func__);
	else if (buffer_tmp[3] > 9)
		dev_warn(&client->dev, "%s: get wrong value: 0x%X/0x%X \n", __func__,
			buffer_tmp[3], 9);
	else {
		printk(KERN_DEBUG "ALS value: 0x%X, level: %d #\n",
			(buffer_tmp[4] << 8 | buffer_tmp[5]), buffer_tmp[3]);
		led_trigger_event(&light_sensor_trigger, buffer_tmp[3]);
		input_report_abs(cdata->ls_input_dev, ABS_MISC, (int)buffer_tmp[3]);
		input_sync(cdata->ls_input_dev);
	}
}

static void microp_i2c_hpin_work_func(struct work_struct *work)
{
	if (set_microp_mic_intr(1) < 0)
		printk(KERN_ERR "%s: set microp mic interrupt error\n", __func__);
}


static void microp_i2c_intr_work_func(struct work_struct *work)
{
	struct microp_i2c_work *up_work;
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	uint8_t data[6], gpio_pin_status[6], intr_status = 0;

	up_work = container_of(work, struct microp_i2c_work, work);
	client = up_work->client;
	cdata = i2c_get_clientdata(client);

	if (microp_i2c_is_supported(FUNC_MICROP_EN_SPI_AUTO_BACKLIGHT, cdata->version)) {
		memset(data, 0x00, sizeof(data));
		if (i2c_read_block(client, MICROP_I2C_CMD_GET_INTERRUPT_STATUS, data, 1) < 0)
			dev_err(&client->dev, "%s: read interrupt status fail\n", __func__);

		intr_status = data[0];
		if (intr_status & MICROP_INT_LIGHT_SENSOR)
			microp_auto_backlight_function();
		if (intr_status & MICROP_INT_ADC)
			get_3_button_key();
		data[0] = intr_status;
		if (i2c_write_block(client, MICROP_I2C_CMD_CLEAR_INTERRUPT, data, 1) < 0)
			dev_err(&client->dev, "%s: clear interrupts fail\n", __func__);

		if (intr_status & MICROP_INT_GPIO) { /* 3.5 mm headset */
			memset(gpio_pin_status, 0x00, sizeof(gpio_pin_status));
			if (up_work->intr_debounce)
				up_work->intr_debounce(gpio_pin_status);

			if (i2c_read_block(client, MICROP_I2C_CMD_READ_PIN, gpio_pin_status, 6)) {
				gpio_pin_status[0] = 0;
				gpio_pin_status[1] = 0;
				gpio_pin_status[2] = 0;
			}

			if (up_work->intr_function)
				up_work->intr_function(gpio_pin_status);
		}
	}

	enable_irq(client->irq);
}

static irqreturn_t microp_i2c_intr_irq_handler(int irq, void *dev_id)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;

	client = to_i2c_client(dev_id);
	cdata = i2c_get_clientdata(client);

	disable_irq(client->irq);
	queue_work(cdata->microp_queue, &cdata->work.work);
	return IRQ_HANDLED;
}

int microp_i2c_set_pin_mode(uint8_t pin, uint8_t mode, void *dev_id)
{
	struct i2c_client *client;
	uint8_t data[2];

	client = to_i2c_client(dev_id);

	data[0] = pin;
	data[1] = mode;

	return i2c_write_block(client, MICROP_I2C_CMD_PIN_MODE, data, 2);
}

EXPORT_SYMBOL(microp_i2c_set_pin_mode);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void microp_early_suspend(struct early_suspend *h)
{
	int ret;
	uint8_t data[2];
#if 0
	uint8_t lcd_backlight_value = 0x10;
	uint8_t lcd_backlight_pin = 6;
	uint8_t lcd_backlight_level = 50;
#endif
	uint8_t loop_i, config;
	struct microp_led_data *ldata;
	struct microp_i2c_client_data *cdata;
	struct i2c_client *client = private_microp_client;
	struct microp_i2c_platform_data *pdata;
	ldata = container_of(ldev_lcd_backlight, struct microp_led_data, ldev);
	if (!client)
		return;
	cdata = i2c_get_clientdata(client);
	pdata = client->dev.platform_data;
	atomic_set(&cdata->suspended_now, 1);

	if (pdata->ls_power) {
		pdata->ls_power(0);
		ls_power_on_jiffy = 0;
	}

	if (atomic_read(&als_intr_enabled)) {
		ret = microp_i2c_auto_backlight_set_interrupt_mode(client, 0);
		if (ret < 0)
			pr_err("%s: disable auto light sensor fail\n",
				__func__);
		else
			atomic_set(&als_intr_enabled, 0);
	}
	if (cdata->microp_gpio_pwm_is_enabled) {
		for (loop_i = 0; loop_i < cdata->num_led_data; loop_i++)	{
			config = cdata->led_data[loop_i].pin_config->config;
			if (microp_i2c_is_ls_gpio(config)) {
				ret = microp_i2c_gpio_pwm_write(client, &cdata->led_data[loop_i], 0);
				if (ret < 0)
					dev_err(&client->dev, "disable gpio_pwm fail: %d\n", ret);
			}
		}
	}
#if 0
	ret = batt_notifier_call_chain(BATT_EVENT_SUSPEND, (void *)private_microp_client);
	if ((ret == NOTIFY_STOP) && ldata)	{ /* Turn on LCD backlight */
		dev_dbg(&client->dev, "%s: setting LCD backlight value: %d\n",
				__func__, lcd_backlight_value);
		atomic_set(&cdata->suspended_now, 0);
		mutex_lock(&ldata->pin_mutex);
		ldata->ldev.brightness = lcd_backlight_level;
		ldata->pin_config->pin = lcd_backlight_pin;
		if (microp_i2c_write_pin_duty(client, ldata) < 0)
			dev_err(&client->dev, "%s: setting LCM backlight fail -\n", __func__);

		memset(data, 0x0, sizeof(data));
		data[0] = lcd_backlight_pin;
		data[1] = 1;
		i2c_write_block(client, MICROP_I2C_CMD_PIN_MODE, data, 2);
		mutex_unlock(&ldata->pin_mutex);
	}
#endif
	microp_i2c_suspend_led_control(client);
}

static void microp_late_resume(struct early_suspend *h)
{
	struct i2c_client *client = private_microp_client;
	struct microp_i2c_client_data *cdata;
	struct microp_i2c_platform_data *pdata;
	uint8_t loop_i, config;
	int ret;
	if (!client)
		return;
	cdata = i2c_get_clientdata(client);
	pdata = client->dev.platform_data;

	if (pdata->ls_power) {
		pdata->ls_power(1);
		ls_power_on_jiffy = jiffies;
	}

	if (atomic_read(&als_intr_enable_flag)) {
		ret = microp_i2c_auto_backlight_set_interrupt_mode(client, 1);
		if (ret < 0)
			pr_err("%s: set auto light sensor fail\n", __func__);
		else {
			atomic_set(&als_intr_enabled, 1);
			input_report_abs(cdata->ls_input_dev, ABS_MISC, -1);
			input_sync(cdata->ls_input_dev);
		}
	}
	if (cdata->microp_gpio_pwm_is_enabled) {
		for (loop_i = 0; loop_i < cdata->num_led_data; loop_i++)	{
			config = cdata->led_data[loop_i].pin_config->config;
			if (microp_i2c_is_ls_gpio(config)) {
				ret = microp_i2c_gpio_pwm_write(client, &cdata->led_data[loop_i], 1);
				if (ret < 0)
					dev_err(&client->dev, "enable gpio_pwm fail\n");
			}
		}
	}
	atomic_set(&cdata->suspended_now, 0);
	microp_i2c_suspend_led_control(client);
}
#endif

static int lightsensor_enable(void)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	int ret;

	client = private_microp_client;
	cdata = i2c_get_clientdata(client);

	pr_info("%s\n", __func__);
	atomic_set(&als_intr_enable_flag, 1);
	if (atomic_read(&cdata->suspended_now)) {
		atomic_set(&als_intr_enabled, 1);
		pr_err("%s: microp is suspended\n", __func__);
		return 0;
	}

	ret = microp_i2c_auto_backlight_set_interrupt_mode(client, 1) ;
	if (ret < 0)
		pr_err("%s: set auto light sensor fail\n", __func__);
	else {
		atomic_set(&als_intr_enabled, 1);
		/* report an invalid value first to ensure we trigger an event
		 * when adc_level is zero.
		 */
		input_report_abs(cdata->ls_input_dev, ABS_MISC, -1);
		input_sync(cdata->ls_input_dev);
	}
	return 0;
}

static int lightsensor_disable(void)
{
	/* update trigger data when done */
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	int ret;

	client = private_microp_client;
	cdata = i2c_get_clientdata(client);

	pr_info("%s\n", __func__);
	atomic_set(&als_intr_enable_flag, 0);
	if (atomic_read(&cdata->suspended_now)) {
		atomic_set(&als_intr_enabled, 0);
		pr_err("%s: microp is suspended\n", __func__);
		return 0;
	}

	ret = microp_i2c_auto_backlight_set_interrupt_mode(client, 0) ;
	if (ret < 0)
		pr_err("%s: disable auto light sensor fail\n",
		       __func__);
	else
		atomic_set(&als_intr_enabled, 0);
	return 0;
}

DEFINE_MUTEX(microp_i2c_api_lock);
static int lightsensor_opened;

static int lightsensor_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	pr_debug("%s\n", __func__);
	mutex_lock(&microp_i2c_api_lock);
	if (lightsensor_opened) {
		pr_err("%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	lightsensor_opened = 1;
	mutex_unlock(&microp_i2c_api_lock);
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	pr_debug("%s\n", __func__);
	mutex_lock(&microp_i2c_api_lock);
	lightsensor_opened = 0;
	mutex_unlock(&microp_i2c_api_lock);
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int rc, val;
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;

	mutex_lock(&microp_i2c_api_lock);

	client = private_microp_client;
	cdata = i2c_get_clientdata(client);

	pr_debug("%s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		rc = val ? lightsensor_enable() : lightsensor_disable();
		break;
	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		val = atomic_read(&als_intr_enabled);
		pr_debug("%s enabled %d\n", __func__, val);
		rc = put_user(val, (unsigned long __user *)arg);
		break;
	default:
		pr_err("%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
		rc = -EINVAL;
	}

	mutex_unlock(&microp_i2c_api_lock);
	return rc;
}

static struct file_operations lightsensor_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lightsensor_fops
};

static int __devexit microp_i2c_remove(struct i2c_client *client)
{
	struct microp_i2c_platform_data *pdata;
	struct microp_i2c_client_data *cdata;
	int i;

	pdata = client->dev.platform_data;
	cdata = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (cdata->enable_early_suspend)
		unregister_early_suspend(&cdata->early_suspend);
#endif
#ifdef ENABLE_READ_35MM_ADC_VALUE_FROM_ATTR_FILE
	if (cdata->num_adc_read_data)	{
		for (i = 0; i < cdata->num_adc_read_data; i++)	{
			device_remove_file(&cdata->adc_device[i], &dev_attr_adc_value);
			device_unregister(&cdata->adc_device[i]);
		}
		kfree(cdata->adc_device);
		cdata->num_adc_read_data = 0;
	}
#endif
	if (cdata->use_irq)
		free_irq(client->irq, &client->dev);

	gpio_free(pdata->gpio_reset);

	misc_deregister(&lightsensor_misc);
	input_unregister_device(cdata->ls_input_dev);
	input_free_device(cdata->ls_input_dev);
	device_remove_file(&client->dev, &dev_attr_reset);
	device_remove_file(&client->dev, &dev_attr_version);
	device_remove_file(&client->dev, &dev_attr_status);
	device_remove_file(&client->dev, &dev_attr_light_sensor);
	destroy_workqueue(cdata->microp_queue);

	for (i = cdata->num_led_data - 1; i >= 0 ; i--) {
		device_remove_file(cdata->led_data[i].ldev.dev, &dev_attr_blink);
		if (microp_i2c_is_supported(FUNC_MICROP_OFF_TIMER, cdata->version))
			device_remove_file(cdata->led_data[i].ldev.dev,
				&dev_attr_off_timer);
		if (microp_i2c_is_pwm(cdata->led_data[i].pin_config->config) &&
			!cdata->led_data[i].pin_config->auto_if_on &&
			strcmp(cdata->led_data[i].pin_config->name, "lcd-backlight"))
			device_remove_file(cdata->led_data[i].ldev.dev, &dev_attr_fade);
		if (microp_i2c_is_pwm(cdata->led_data[i].pin_config->config)) {
			if (microp_i2c_is_supported(FUNC_MICROP_PWM_AUTO, cdata->version))
				device_remove_file(cdata->led_data[i].ldev.dev, &dev_attr_auto);
				device_remove_file(cdata->led_data[i].ldev.dev,
					&dev_attr_auto_brightness);
		}
		if (ldev_lcd_backlight == &cdata->led_data[i].ldev)
			ldev_lcd_backlight = NULL;
		led_classdev_unregister(&cdata->led_data[i].ldev);
	}

	kfree(cdata->led_data);
	kfree(cdata);

	return 0;
}

static void light_sensor_activate(struct led_classdev *led_cdev)
{
	struct complete_data {
		int done;
	} *data;
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	client = private_microp_client;
	cdata = i2c_get_clientdata(client);
	data = (struct complete_data *)led_cdev->trigger_data;

	if (!microp_i2c_is_supported(FUNC_MICROP_EN_SPI_AUTO_BACKLIGHT, cdata->version) &&
		!cabc_backlight_enabled)
		return;

	if (atomic_read(&cdata->suspended_now)) {
		dev_err(&client->dev, "%s: abort, uP is going to suspend after #\n", __func__);
		data->done = 0;
		return;
	}

	if (microp_i2c_auto_backlight_set_interrupt_mode(client, 1) < 0) {
		dev_err(&client->dev, "%s: set_interrupt_mode fail\n", __func__);
		data->done = 0;
	} else {
		data->done = 1;
		atomic_set(&als_intr_enabled, 1);
	}
}

static void light_sensor_deactivate(struct led_classdev *led_cdev)
{
	/* update trigger data when done */
	struct i2c_client *client;
	struct complete_data {
		int done;
	} *data;
	struct microp_i2c_client_data *cdata;
	data = (struct complete_data *)led_cdev->trigger_data;
	client = private_microp_client;
	cdata = i2c_get_clientdata(client);

	if (!microp_i2c_is_supported(FUNC_MICROP_EN_SPI_AUTO_BACKLIGHT, cdata->version) &&
		!cabc_backlight_enabled)
		return;

	if (atomic_read(&cdata->suspended_now)) {
		dev_err(&client->dev, "%s: abort, uP is going to suspend after #\n", __func__);
		data->done = 0;
		return;
	}

	if (microp_i2c_auto_backlight_set_interrupt_mode(client, 0) < 0) {
		dev_err(&client->dev, "%s: set_interrupt_mode fail\n", __func__);
		data->done = 0;
	} else {
		data->done = 1;
		atomic_set(&als_intr_enabled, 0);
	}
}

static void microp_lcd_backlight_gate_set(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
	int on;
	struct microp_led_data *ldata;
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;

	ldata = container_of(ldev_lcd_backlight, struct microp_led_data, ldev);
	client = to_i2c_client(ldev_lcd_backlight->dev->parent);
	cdata = i2c_get_clientdata(client);

	led_cdev->brightness = brightness;

	on = (brightness != 0) ? 1 : 0;

	if (microp_i2c_is_backlight_on == on)
		goto exit;

	wake_lock(&microp_i2c_wakelock);

	microp_i2c_is_backlight_on = on;
	schedule_work(&gate_work);

exit:
	return;
}

static void microp_i2c_gate_work_func(struct work_struct *work)
{
	struct microp_led_data *ldata;
	struct i2c_client *client;

	ldata = container_of(ldev_lcd_backlight, struct microp_led_data, ldev);
	client = to_i2c_client(ldev_lcd_backlight->dev->parent);

	mutex_lock(&ldata->pin_mutex);

	if (microp_i2c_write_pin_mode(client, ldata))
		dev_err(&client->dev, "set lcd-backlight on/off failed\n");

	mutex_unlock(&ldata->pin_mutex);

	if ((strcmp(ldata->ldev.name, "lcd-backlight") == 0)  && (ldata->is_auto))
		schedule_delayed_work(&fade_work, FADE_DELAY);

	wake_unlock(&microp_i2c_wakelock);
}

static void microp_lcd_backlight_notifier_set(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	struct microp_led_data *ldata;

	client = private_microp_client;
	cdata = i2c_get_clientdata(client);
	ldata = container_of(ldev_vkey_backlight, struct microp_led_data, ldev);

	led_cdev->brightness = ldata->ldev.brightness = brightness;
	ldata->mode = (brightness != 0) ? 1 : 0;

	queue_work(cdata->microp_queue, &notifier_work);

	return;
}

static void microp_i2c_notifier_work_func(struct work_struct *work)
{
	struct microp_led_data *ldata;
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	unsigned long delay_time;
	uint8_t data[3];

	ldata = container_of(ldev_vkey_backlight, struct microp_led_data, ldev);
	client = to_i2c_client(ldev_vkey_backlight->dev->parent);
	cdata = i2c_get_clientdata(client);

	cancel_delayed_work(&notifier_delay_work);

	data[0] = ldata->pin_config->pin;
	data[1] = MICROP_I2C_PWM_MANUAL;
	data[2] = ldata->ldev.brightness;
	if (i2c_write_block(client, MICROP_I2C_CMD_PIN_PWM, data, 3))
		dev_err(&client->dev,
			"led_brightness_set failed on set manual\n");

	if (microp_i2c_write_pin_mode(client, ldata))
		dev_err(&client->dev,
			"led_brightness_set failed on set mode\n");

	if (ldata->mode)
		delay_time = HZ * 10;
	else
		delay_time = 0;
	queue_delayed_work(cdata->microp_queue,
			&notifier_delay_work, delay_time);
}

static void microp_i2c_notifier_delay(struct work_struct *work)
{
	struct microp_led_data *ldata;
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	uint8_t data[3];

	ldata = container_of(ldev_vkey_backlight, struct microp_led_data, ldev);
	client = to_i2c_client(ldev_vkey_backlight->dev->parent);
	cdata = i2c_get_clientdata(client);

	if (microp_i2c_is_supported(FUNC_MICROP_LED_AUTO,
					cdata->version)) {
		data[0] = ldata->pin_config->pin;
		data[1] = ldata->mode;
		if (i2c_write_block(client,
				MICROP_I2C_CMD_LED_AUTO_ENABLE, data, 2))
			dev_err(&client->dev,
				"failed on set LED auto\n");
	}
}

static void microp_i2c_fade_work_func(struct work_struct *work)
{
	struct microp_led_data *ldata;
	struct i2c_client *client;
	uint8_t data[6];

	ldata = container_of(ldev_lcd_backlight, struct microp_led_data, ldev);
	client = to_i2c_client(ldev_lcd_backlight->dev->parent);

	mutex_lock(&ldata->pin_mutex);
	if ((strcmp(ldata->ldev.name, "lcd-backlight") == 0) && ldata->is_auto && microp_i2c_is_backlight_on) {
		ldata->fade_timer = 1;
		data[0] = ldata->pin_config->pin;
		data[1] = MICROP_I2C_PWM_FADE;
		data[2] = ldata->fade_timer;
		if (i2c_write_block(client, MICROP_I2C_CMD_PIN_PWM, data, 3))
			dev_err(&client->dev, "%s set fade %d failed\n",
				ldata->pin_config->name, 1);
	}
	mutex_unlock(&ldata->pin_mutex);
}

static void microp_i2c_auto_bl_work_func(struct work_struct *work)
{
	struct i2c_client *client = private_microp_client;
	struct microp_i2c_client_data *cdata;
	uint8_t data[2] = {0, 0};
	int ret = 0;

	cdata = i2c_get_clientdata(client);

	data[0] |= 0x04;
	if (cdata->enable_reset_button)
		data[0] |= 0x08;

	dev_info(&client->dev,  "%s: ls enable\n", __func__);
	ret = i2c_write_block(client, MICROP_I2C_CMD_MISC1, data, 1);
	if (ret != 0)
		dev_err(&client->dev,
			"%s: setting MICROP_I2C_CMD_MISC1 fail\n", __func__);
}

#ifndef CONFIG_HAS_EARLYSUSPEND
static int microp_i2c_suspend(struct i2c_client *client,
	pm_message_t mesg)
{
	return 0;
}

static int microp_i2c_resume(struct i2c_client *client)
{
	return 0;
}
#endif

static int microp_i2c_probe(struct i2c_client *client
	, const struct i2c_device_id *id)
{
	struct microp_i2c_platform_data *pdata;
	struct microp_i2c_client_data *cdata;
	int num_leds;
	int i, j, ret;
	uint8_t data[6];
	int lcd = -1;
	static uint8_t is_enable_lcd_backlight_func;
	struct cnf_driver *h2w_3btval, *h2w_query_microp_mic, *h2w_unplug_microp_mic;

	private_microp_client = client;
	pdata = client->dev.platform_data;
	if (!pdata) {
		ret = -EBUSY;
		dev_err(&client->dev, "failed on get pdata\n");
		goto err_exit;
	}
	pdata->dev_id = (void *)&client->dev;

	ret = i2c_read_block(client, MICROP_I2C_CMD_VERSION, data, 2);
	ret = microp_i2c_version_check(client, data);
	if (ret || !(data[0] && data[1])) {
		ret = -ENODEV;
		dev_err(&client->dev, "failed on get microp version\n");
		goto err_exit;
	}
	dev_info(&client->dev, "microp version [%02X][%02X]\n", data[0],
		data[1]);

	if (!microp_i2c_is_supported(FUNC_MICROP_VERSION, data[0] << 8 | data[1])) {
		dev_err(&client->dev, "Not Supporting This Version\n");
		goto err_exit;
	}

	ret = gpio_request(pdata->gpio_reset, "microp_i2c");
	if (ret < 0) {
		dev_err(&client->dev, "failed on request gpio reset\n");
		goto err_exit;
	}
	ret = gpio_direction_output(pdata->gpio_reset, 1);
	if (ret < 0) {
		dev_err(&client->dev, "failed on gpio_direction_output reset\n");
		goto err_gpio_reset;
	}

	cdata = kzalloc(sizeof(struct microp_i2c_client_data), GFP_KERNEL);
	if (!cdata) {
		ret = -ENOMEM;
		dev_err(&client->dev, "failed on allocat cdata\n");
		goto err_cdata;
	}

	cdata->version = data[0] << 8 | data[1];
	cdata->enable_early_suspend = pdata->microp_enable_early_suspend;
	cdata->enable_reset_button = pdata->microp_enable_reset_button;
	atomic_set(&cdata->suspended_now, 0);

	for (i = 0, num_leds = 0; i < pdata->num_pins; i++)
		if (microp_i2c_is_gpo(pdata->pin_config[i].config) &&
			pdata->pin_config[i].name &&
			microp_i2c_is_supported(FUNC_MICROP_GPO, cdata->version))
			num_leds++;
		else if (microp_i2c_is_pwm(pdata->pin_config[i].config) &&
			microp_i2c_is_supported(FUNC_MICROP_PWM, cdata->version))
			num_leds++;
		else if (microp_i2c_is_ls_gpio(pdata->pin_config[i].config) &&
			microp_i2c_is_supported(FUNC_MICROP_LS_GPIO, cdata->version))
			num_leds++;

	cdata->num_led_data = num_leds;
	cdata->led_data = kzalloc(sizeof(struct microp_led_data) * num_leds,
							GFP_KERNEL);
	if (!cdata->led_data && num_leds) {
		ret = -ENOMEM;
		dev_err(&client->dev, "failed on allocate cdata->led_data\n");
		goto err_led_data;
	}
	mutex_init(&cdata->microp_i2c_mutex);
	i2c_set_clientdata(client, cdata);

	for (i = 0, j = 0; i < num_leds; i++, j++) {
		while (1) {
			if (microp_i2c_is_gpo(pdata->pin_config[j].config) &&
				pdata->pin_config[j].name &&
				microp_i2c_is_supported(FUNC_MICROP_GPO, cdata->version))
				break;
			else if (microp_i2c_is_pwm(pdata->pin_config[j].config) &&
				microp_i2c_is_supported(FUNC_MICROP_PWM, cdata->version))
				break;
			else if (microp_i2c_is_ls_gpio(pdata->pin_config[j].config) &&
				microp_i2c_is_supported(FUNC_MICROP_LS_GPIO, cdata->version))
				break;
			j++;
		}
		mutex_init(&cdata->led_data[i].pin_mutex);
		cdata->led_data[i].ldev.name = pdata->pin_config[j].name;
		if (pdata->pin_config[j].i_am_jogball_function)
			cdata->led_data[i].ldev.brightness_set = microp_jogball_brightness_set;
		else {
			cdata->led_data[i].ldev.brightness_get = microp_led_brightness_get;
			cdata->led_data[i].ldev.brightness_set = microp_led_brightness_set;
		}
		cdata->led_data[i].ldev.brightness = 0;
		cdata->led_data[i].pin_config = pdata->pin_config + j;

		ret = led_classdev_register(&client->dev, &cdata->led_data[i].ldev);
		if (ret < 0) {
			dev_err(&client->dev, "failed on led_classdev_register [%d]\n", i);
			goto err_register_led_cdev;
		}

		if (strcmp(cdata->led_data[i].ldev.name, "lcd-backlight") == 0) {
			lcd = i;
			ret = led_classdev_register(&client->dev, &ldev_lcd_backlight_gate);
			if (ret < 0) {
				dev_err(&client->dev,
					"failed on register lcd-backlight-gate\n");
				goto err_register_led_cdev;
			}
			ldev_lcd_backlight = &cdata->led_data[i].ldev;
			INIT_WORK(&gate_work, microp_i2c_gate_work_func);
			INIT_DELAYED_WORK(&fade_work, microp_i2c_fade_work_func);
			is_enable_lcd_backlight_func++;
		}

		if (strcmp(cdata->led_data[i].ldev.name, "vkey-backlight") == 0) {
			ret = led_classdev_register(&client->dev,
				&ldev_lcd_backlight_notifier);
			if (ret < 0) {
				dev_err(&client->dev,
					"failed on register ldev_lcd_backlight_notifier\n");
				goto err_register_led_cdev;
			}
			ldev_vkey_backlight = &cdata->led_data[i].ldev;
			INIT_WORK(&notifier_work, microp_i2c_notifier_work_func);
			INIT_DELAYED_WORK(&notifier_delay_work,
					microp_i2c_notifier_delay);
		}
	}

	for (i = 0; i < num_leds; i++) {
		ret = device_create_file(cdata->led_data[i].ldev.dev, &dev_attr_blink);
		if (ret < 0) {
			dev_err(&client->dev, "failed on create attr blink [%d]\n", i);
			goto err_register_attr_blink;
		}
	}

	if (microp_i2c_is_supported(FUNC_MICROP_OFF_TIMER, cdata->version))
		for (i = 0; i < num_leds; i++) {
			ret = device_create_file(cdata->led_data[i].ldev.dev,
					&dev_attr_off_timer);
			if (ret < 0) {
				dev_err(&client->dev,
					"failed on create attr off_timer [%d]\n", i);
				goto err_register_attr_off_timer;
			}
		}

	for (i = 0; i < num_leds; i++) {
		if (microp_i2c_is_pwm(cdata->led_data[i].pin_config->config) &&
			!cdata->led_data[i].pin_config->auto_if_on &&
			strcmp(cdata->led_data[i].pin_config->name, "lcd-backlight")) {
			ret = device_create_file(cdata->led_data[i].ldev.dev,
					&dev_attr_fade);
			if (ret < 0) {
				dev_err(&client->dev, "failed on create attr fade [%d]\n", i);
				goto err_register_attr_fade;
			}
		}
	}

	for (i = 0; i < num_leds; i++) {
		if (microp_i2c_is_pwm(cdata->led_data[i].pin_config->config) &&
			!cdata->led_data[i].pin_config->auto_if_on) {
			if (!microp_i2c_is_supported(FUNC_MICROP_PWM_AUTO, cdata->version))
				continue;
				ret = device_create_file(cdata->led_data[i].ldev.dev,
						&dev_attr_auto);
			if (ret < 0) {
				dev_err(&client->dev, "failed on create attr auto [%d]\n", i);
				goto err_register_attr_auto;
			}
		}
	}

	for (i = 0; i < num_leds; i++) {
		if (microp_i2c_is_pwm(cdata->led_data[i].pin_config->config) &&
			!cdata->led_data[i].pin_config->auto_if_on) {
			if (!microp_i2c_is_supported(FUNC_MICROP_PWM_AUTO, cdata->version))
				continue;
			ret = device_create_file(cdata->led_data[i].ldev.dev,
					&dev_attr_auto_brightness);
			if (ret < 0) {
				dev_err(&client->dev,
					"failed on create attr auto_brightness [%d]\n", i);
				goto err_register_attr_auto_brightness;
			}
		}
	}

	microp_i2c_clear_led_data(client);
	if (lcd > -1)
		cdata->led_data[lcd].skip_config = 1;
	ret = microp_i2c_config_microp(client);
	if (ret) {
		dev_err(&client->dev, "failed on config microp\n");
		goto err_config;
	}

	if (client->irq)
		for (i = pdata->num_pins - 1; i >= 0 ; i--)
			if (microp_i2c_is_intr(pdata->pin_config[i].config) &&
				microp_i2c_is_supported(FUNC_MICROP_OFF_TIMER, cdata->version)
				) {
				cdata->use_irq = 1;
				break;
			}
	if (cdata->use_irq) {
		INIT_WORK(&cdata->work.work, microp_i2c_intr_work_func);
		cdata->work.client = client;
		cdata->work.microp_intr_pin = pdata->pin_config[i].pin;
		cdata->work.intr_debounce = pdata->pin_config[i].intr_debounce;
		cdata->work.intr_function = pdata->pin_config[i].intr_function;
		ret = request_irq(client->irq, microp_i2c_intr_irq_handler,
				IRQF_TRIGGER_LOW, pdata->pin_config[i].name, &client->dev);
		if (ret) {
			dev_err(&client->dev, "request_irq failed\n");
			goto err_intr;
		}
		ret = set_irq_wake(client->irq, 1);
		if (ret) {
			dev_err(&client->dev, "set_irq_wake failed\n");
			goto err_intr;
		}

		if (pdata->pin_config[i].init_intr_function) {
			if (i2c_read_block(client, MICROP_I2C_CMD_READ_PIN, data, 6)) {
				data[0] = 0;
				data[1] = 0;
				data[2] = 0;
			}
			pdata->pin_config[i].intr_function(data);
		}
		if (ret == 0)	{
			data[0] = 1;
			ret = i2c_write_block(client, MICROP_I2C_CMD_SET_INTERRUPT_CONTROL, data, 1);
			if (ret != 0)
				goto err_intr;
		}
	}
#ifdef ENABLE_READ_35MM_ADC_VALUE_FROM_ATTR_FILE
	cdata->num_adc_read_data = 0;
	for (i = 0; i < pdata->num_pins; i++)
		if (microp_i2c_is_adc_read(pdata->pin_config[i].config) &&
			pdata->pin_config[i].name &&
			microp_i2c_is_supported(FUNC_MICROP_EN_SPI_AUTO_BACKLIGHT, cdata->version)) {
			cdata->num_adc_read_data++;
	}
	if (cdata->num_adc_read_data)	{
		cdata->adc_device = kzalloc(sizeof(struct device) * cdata->num_adc_read_data,
			GFP_KERNEL);
		if (!cdata->adc_device) {
			ret = -ENOMEM;
			dev_err(&client->dev, "%s: failed on allocat adc_device\n", __func__);
			goto err_intr;
		}
		for (i = 0, j = 0; i < pdata->num_pins; i++) {
			if (microp_i2c_is_adc_read(pdata->pin_config[i].config) &&
				pdata->pin_config[i].name) {
				strcpy(cdata->adc_device[j].bus_id, pdata->pin_config[i].name);
				cdata->adc_device[j].parent = &client->dev;
				cdata->adc_device[j].driver_data = &pdata->pin_config[i];
				remote_adc_read_channel = pdata->pin_config[i].adc_pin;
				if (device_register(&cdata->adc_device[j]) != 0) {
					dev_err(&client->dev, "%s: can't register device_register\n", __func__);
					goto err_cant_register_adc_read_free_mem;
				}
				if (device_create_file(&cdata->adc_device[j], &dev_attr_adc_value) != 0) {
					dev_err(&client->dev, "%s: dev_attr_adc_value failed", __func__);
					goto err_cant_register_adc_read_attr_file;
				}
				j++;
				if (j == cdata->num_adc_read_data)
					break;
			}
		}
	}
#endif
	ret = device_create_file(&client->dev, &dev_attr_reset);
	ret = device_create_file(&client->dev, &dev_attr_version);
	ret = device_create_file(&client->dev, &dev_attr_status);
	ret = device_create_file(&client->dev, &dev_attr_light_sensor);
	cdata->microp_queue = create_singlethread_workqueue("microp_work_q");
	if (cdata->microp_queue == NULL) {
		ret = -ENOMEM;
		goto err_create_work_queue;
	}
	INIT_DELAYED_WORK(&auto_bl_delay_work,
			microp_i2c_auto_bl_work_func);

	/* Light Sensor */
	cdata->ls_input_dev = input_allocate_device();
	if (!cdata->ls_input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		ret = -ENOMEM;
		goto err_request_input_dev;
	}
	cdata->ls_input_dev->name = "lightsensor-level";
	set_bit(EV_ABS, cdata->ls_input_dev->evbit);
	input_set_abs_params(cdata->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

	ret = input_register_device(cdata->ls_input_dev);
	if (ret < 0) {
		dev_err(&client->dev, "%s: can not register input device\n",
				__func__);
		goto err_register_input_dev;
	}

	ret = misc_register(&lightsensor_misc);
	if (ret < 0) {
		dev_err(&client->dev, "%s: can not register misc device\n",
				__func__);
		goto err_register_misc_register;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (cdata->enable_early_suspend)	{
		cdata->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		cdata->early_suspend.suspend = microp_early_suspend;
		cdata->early_suspend.resume = microp_late_resume;
		register_early_suspend(&cdata->early_suspend);
	}
#endif

	cabc_backlight_enabled = pdata->cabc_backlight_enable;

	if (pdata->ls_power) {
		pdata->ls_power(1);
		ls_power_on_jiffy = jiffies;
	}

	h2w_3btval = kzalloc(sizeof(struct cnf_driver), GFP_KERNEL);
	if (h2w_3btval) {
		h2w_3btval->name = "get_3btn_val";
		h2w_3btval->func = &microp_get_3_button_value;
	} else {
		dev_err(&client->dev, "h2w_extern alloc failed (no memory)\n");
		goto err_h2w_3btval;
	}
	cnf_driver_register(h2w_3btval);

	h2w_query_microp_mic = kzalloc(sizeof(struct cnf_driver), GFP_KERNEL);
	if (h2w_query_microp_mic) {
		h2w_query_microp_mic->name = "query_mic_state";
		h2w_query_microp_mic->func = &microp_get_mic_value;
	} else {
		dev_err(&client->dev, "h2w_extern2 alloc failed (no memory)\n");
		goto err_h2w_query_microp_mic;
	}
	cnf_driver_register(h2w_query_microp_mic);

	h2w_unplug_microp_mic = kzalloc(sizeof(struct cnf_driver), GFP_KERNEL);
	if (h2w_unplug_microp_mic) {
		h2w_unplug_microp_mic->name = "unplug_microp_mic";
		h2w_unplug_microp_mic->func = &microp_unplug_mic;
	} else {
		dev_err(&client->dev, "h2w_extern2 alloc failed (no memory)\n");
		goto err_h2w_unplug_microp_mic;
	}
	cnf_driver_register(h2w_unplug_microp_mic);

	cnf_driver_event("H2W_microp_ready", &ret);

	return 0;

err_h2w_unplug_microp_mic:
	kfree(h2w_query_microp_mic);
err_h2w_query_microp_mic:
	kfree(h2w_3btval);

err_h2w_3btval:
	misc_deregister(&lightsensor_misc);

err_register_misc_register:
	input_unregister_device(cdata->ls_input_dev);

err_register_input_dev:
	input_free_device(cdata->ls_input_dev);

err_request_input_dev:
	destroy_workqueue(cdata->microp_queue);

err_create_work_queue:

#ifdef ENABLE_READ_35MM_ADC_VALUE_FROM_ATTR_FILE
err_cant_register_adc_read_attr_file:
	if (cdata->num_adc_read_data)	{
		for (i = 0; i < cdata->num_adc_read_data; i++)	{
			device_remove_file(&cdata->adc_device[i], &dev_attr_adc_value);
			device_unregister(&cdata->adc_device[i]);
		}
	}

err_cant_register_adc_read_free_mem:
	kfree(cdata->adc_device);
	cdata->num_adc_read_data = 0;
#endif

err_intr:

err_config:
	i = num_leds;
err_register_attr_auto_brightness:
	for (i--; i >= 0; i--)
		if (microp_i2c_is_pwm(cdata->led_data[i].pin_config->config) &&
			microp_i2c_is_supported(FUNC_MICROP_PWM_AUTO, cdata->version) &&
			!cdata->led_data[i].pin_config->auto_if_on)
			device_remove_file(cdata->led_data[i].ldev.dev,
				&dev_attr_auto_brightness);
	i = num_leds;

err_register_attr_auto:
	for (i--; i >= 0; i--)
		if (microp_i2c_is_pwm(cdata->led_data[i].pin_config->config) &&
			microp_i2c_is_supported(FUNC_MICROP_PWM_AUTO, cdata->version) &&
			!cdata->led_data[i].pin_config->auto_if_on)
			device_remove_file(cdata->led_data[i].ldev.dev, &dev_attr_auto);
	i = num_leds;

err_register_attr_fade:
	for (i--; i >= 0; i--)
		if (microp_i2c_is_pwm(cdata->led_data[i].pin_config->config) &&
			!cdata->led_data[i].pin_config->auto_if_on &&
			strcmp(cdata->led_data[i].pin_config->name, "lcd-backlight"))
			device_remove_file(cdata->led_data[i].ldev.dev, &dev_attr_fade);
	i = num_leds;

err_register_attr_off_timer:
	if (microp_i2c_is_supported(FUNC_MICROP_OFF_TIMER, cdata->version))
		for (i--; i >= 0; i--)
			device_remove_file(cdata->led_data[i].ldev.dev,
			&dev_attr_off_timer);
	i = num_leds;

err_register_attr_blink:
	for (i--; i >= 0; i--)
		device_remove_file(cdata->led_data[i].ldev.dev, &dev_attr_blink);
	i = num_leds;

err_register_led_cdev:
	for (i--; i >= 0; i--) {
		if (ldev_lcd_backlight == &cdata->led_data[i].ldev) {
			ldev_lcd_backlight = NULL;
			led_classdev_unregister(&ldev_lcd_backlight_gate);
		}
		led_classdev_unregister(&cdata->led_data[i].ldev);
	}
	kfree(cdata->led_data);

err_led_data:
	kfree(cdata);

err_cdata:

err_gpio_reset:
	gpio_free(pdata->gpio_reset);

err_exit:
	return ret;
}

static const struct i2c_device_id microp_i2c_id[] = {
	{ MICROP_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver microp_i2c_driver = {
	.driver = {
		   .name = MICROP_I2C_NAME,
		   },
	.id_table = microp_i2c_id,
	.probe = microp_i2c_probe,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = microp_i2c_suspend,
	.resume = microp_i2c_resume,
#endif
	.remove = __devexit_p(microp_i2c_remove),
};

static int __init microp_i2c_init(void)
{
	microp_i2c_is_backlight_on = 1;

	wake_lock_init(&microp_i2c_wakelock, WAKE_LOCK_SUSPEND, "microp_i2c_present");

	led_trigger_register(&light_sensor_trigger);
	light_sensor_trigger.activate = light_sensor_activate;
	light_sensor_trigger.deactivate = light_sensor_deactivate;

	return i2c_add_driver(&microp_i2c_driver);
}

static void __exit microp_i2c_exit(void)
{
	i2c_del_driver(&microp_i2c_driver);
}

module_init(microp_i2c_init);
module_exit(microp_i2c_exit);


MODULE_DESCRIPTION("MicroP I2C driver");
MODULE_LICENSE("GPL");
