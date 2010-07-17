/*
 * include/mach/microp_i2c.h - platform data structure for microp
 */

#ifndef _LINUX_MICROP_I2C_H
#define _LINUX_MICROP_I2C_H

#define MICROP_I2C_NAME "microp-i2c"

#define MICROP_PIN_CONFIG_GPO		0x01
#define MICROP_PIN_CONFIG_GPO_INV	0x02
#define MICROP_PIN_CONFIG_PWM		0x04
#define MICROP_PIN_CONFIG_PWM_INV	0x08
#define MICROP_PIN_CONFIG_OTHER		0xF0
#define MICROP_PIN_CONFIG_INTR		0x11
#define MICROP_PIN_CONFIG_ADC		0x12

#define MICROP_PIN_CONFIG_ADC_READ	0x30
#define MICROP_PIN_CONFIG_INTR_ALL	0x31
#define MICROP_PIN_CONFIG_LS_GPO	0x32
#define MICROP_PIN_CONFIG_PULL_UP	0x35
#define MICROP_PIN_CONFIG_PULL_UP1	0x36
#define MICROP_PIN_CONFIG_MIC		0x37
#define MICROP_PIN_CONFIG_UP_ADC	0x38


#define MICROP_PIN_PWM_FREQ_HZ_15600	1
#define MICROP_PIN_PWM_FREQ_HZ_1953		2
#define MICROP_PIN_PWM_FREQ_HZ_244		3
#define MICROP_PIN_PWM_FREQ_HZ_61		4
#define MICROP_PIN_PWM_FREQ_HZ_15		5

struct microp_pin_config {
	const char *name;
	uint8_t	pin;
	uint8_t adc_pin;
	uint8_t intr_pin;
	uint8_t	config;
	uint8_t	freq;
	uint8_t	init_value;
	uint8_t	mask[3];
	uint8_t	setting[3];
	uint8_t	init_intr_function;
	uint8_t	auto_if_on;
	uint8_t	suspend_off;
	uint8_t	led_auto;
	uint16_t levels[10];
	uint16_t dutys[10];
	int (*intr_debounce)(uint8_t *pin_status);
	void (*intr_function)(uint8_t *pin_status);
	uint8_t i_am_jogball_function;
	uint8_t microp_pwm_pin_enable_250ms_delay;
};

#define MICROP_PIN(_pin, _config)	\
{						\
	.pin = _pin,				\
	.config = _config,			\
}

struct microp_i2c_platform_data {
	int			num_pins;
	struct microp_pin_config *pin_config;
	uint32_t		gpio_reset;
	void 			*dev_id;
	uint8_t		cabc_backlight_enable;
	uint8_t		microp_enable_early_suspend;
	uint8_t		microp_enable_reset_button;
	uint8_t		microp_enable_pwm_delay_250ms;
	uint8_t		microp_mic_status;
	int         (*ls_power)(int); /* power to the chip */
};


int microp_i2c_set_pin_mode(uint8_t pin, uint8_t mode, void *dev_id);
#endif /* _LINUX_MICROP_I2C_H */
