#ifndef CY8C_I2C_H
#define CY8C_I2C_H

struct cy8c_i2c_platform_data {
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int abs_pressure_min;
	int abs_pressure_max;
	int abs_width_min;
	int abs_width_max;
	int (*power)(int on);
};

#define CY8C_I2C_NAME "cypress-touch"

#endif

