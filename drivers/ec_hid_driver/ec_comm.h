#ifndef _EC_COMMON_H_
#define _EC_COMMON_H_

struct ec_i2c_platform_data 
{
	struct i2c_client *client;
	dev_t devt;
};

struct ec_check_int_interface
{
	int (*i2c_check_interrupt)(char *type, char *event);
};

struct ec_set_gpio_interface
{
	int (*i2c_to_gpio_set)(u8 gpio, u8 value);
};

struct ec_get_gpio_interface
{
	int (*i2c_get_gpio_data)(char *buffer,char gpio);
};

struct ec_battery_interface
{
	int (*i2c_get_battery_cap)(int *cap);
	int (*i2c_get_battery_vol)(int *vol);
	int (*i2c_get_battery_cur)(short *cur);
	int (*i2c_get_charger_type)(int *type,short *vol,short *cur);
	int (*i2c_set_ultra_low_power_mode)(u8 type);
	int (*i2c_set_phone_panel_state)(u8 type);
	int (*i2c_set_station_cover_state)(u8 type);
};

struct ec_set_dp_display_interface
{
	int (*i2c_set_dp_display_id)(char display_id);
	int (*i2c_set_display_bl)(char* brightness);
	int (*i2c_control_display)(char on);
	int (*i2c_set_display_fps)(char fps);
	int (*i2c_set_hbm)(char hbm);
};

struct ec_porta_cc_interface
{
	int (*ec_i2c_get_porta_cc_state)(int *state);
};

struct ec_fw_ver_interface
{
	int (*i2c_get_ec_fw_ver)(void);
};
#endif

