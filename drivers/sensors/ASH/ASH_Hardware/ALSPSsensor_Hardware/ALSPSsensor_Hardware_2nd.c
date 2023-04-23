/* 
 * Copyright (C) 2015 ASUSTek Inc.
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

/********************************/
/* ALSPS Sensor Hardware Module */
/******************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include "ALSPSsensor_Hardware_2nd.h"
#include <linux/input/ASH.h>
#include <linux/delay.h>

/**************************/
/* Debug and Log System */
/************************/
#define MODULE_NAME			"ASH_HW"
#define SENSOR_TYPE_NAME	"ALSPS_2nd"

#undef dbg
#ifdef ASH_HW_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,__func__,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

/********************/
/* Global Variables */
/******************/
static ALSPS_I2C* g_ALSPS_I2C;
static struct i2c_client * g_i2c_client = NULL;

extern struct ALSPS_hw *g_ALSPS_hw_client_2nd;
static int mALSPS_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	if(g_ALSPS_I2C->ALSPS_probe == NULL){
		err("ALSPS_probe NOT implement. \n");
		return -EINVAL;
	}

	/* We use Probe function to get i2c client */
	g_i2c_client = client;

	g_ALSPS_I2C->ALSPS_probe(client);
	return 0;
}

static int mALSPS_remove(struct i2c_client *client)
{
	if(g_ALSPS_I2C->ALSPS_remove == NULL){
		err("ALSPS_remove NOT implement. \n");
		return -EINVAL;
	}
	
	g_ALSPS_I2C->ALSPS_remove();
	return 0;
}

static void mALSPS_shutdown(struct i2c_client *client)
{
	if(g_ALSPS_I2C->ALSPS_shutdown == NULL){
		err("ALSPS_shutdown NOT implement. \n");
	}
	
	g_ALSPS_I2C->ALSPS_shutdown();
}

static int mALSPS_suspend(struct device *client)
{
	if(g_ALSPS_I2C->ALSPS_suspend == NULL){
		err("ALSPS_suspend NOT implement. \n");
		return -EINVAL;
	}
	
	g_ALSPS_I2C->ALSPS_suspend();
	return 0;
}

static int mALSPS_resume(struct device *client)
{
	if(g_ALSPS_I2C->ALSPS_resume == NULL){
		err("ALSPS_resume NOT implement. \n");
		return -EINVAL;
	}
	
	g_ALSPS_I2C->ALSPS_resume();
	return 0;
}

static const struct dev_pm_ops alsps_dev_pm_ops = {
	.suspend = mALSPS_suspend,
	.resume = mALSPS_resume,
};

static struct i2c_driver ALSPS_i2c_driver_client = {
	.probe = mALSPS_probe,
	.remove = mALSPS_remove,
	.shutdown = mALSPS_shutdown,
	.driver.pm = &alsps_dev_pm_ops,
};

int ALSPS_i2c_register_2nd(ALSPS_I2C *alsps_i2c)
{
	if(alsps_i2c == NULL){
		err("%s : ALSPS_I2C is NULL pointer. \n", __FUNCTION__);
		return -EINVAL;
	}
	
	g_ALSPS_I2C = alsps_i2c;
	return 0;
}
EXPORT_SYMBOL(ALSPS_i2c_register_2nd);

int ALSPS_i2c_unregister_2nd(void)
{
	i2c_del_driver(&ALSPS_i2c_driver_client);

	return 0;
}
EXPORT_SYMBOL(ALSPS_i2c_unregister_2nd);

/***********************/
/*VCNL36866 I2c Driver*/
/**********************/
static const struct i2c_device_id vcnl36866_i2c_id[] = {
	{"vcnl36866_2nd", 0},
	{}
};

static struct of_device_id vcnl36866_match_table[] = {
	{ .compatible = "qcom,vcnl36866_2nd",},
	{},
};

static int ALSPS_hw_setI2cDriver(struct i2c_driver* i2c_driver_client, 
	int hardware_source)
{
	switch(hardware_source) {
		case ALSPS_hw_source_vcnl36866:
			log("set i2c client : vcnl36866_2nd \n");
			i2c_driver_client->driver.name = "vcnl36866_2nd";
			i2c_driver_client->driver.owner = THIS_MODULE;
			i2c_driver_client->driver.of_match_table = vcnl36866_match_table;
			i2c_driver_client->id_table = vcnl36866_i2c_id;
			break;

		default:
			err("get hardware client ERROR : hardware enum=%d\n", hardware_source);
			return -EINVAL;
	}
	
	return 0;
}

static ALSPS_hw* ALSPS_hw_getHardwareClient(int hardware_source)
{
	ALSPS_hw* ALSPS_hw_client = NULL;
		
	switch(hardware_source) {
		case ALSPS_hw_source_vcnl36866:
			log("get hardware client : vcnl36866_2nd \n");
			ALSPS_hw_client = ALSPS_hw_vcnl36866_getHardware_2nd();
			break;

		default:
			err("get hardware client ERROR : hardware enum=%d\n", hardware_source);
	}

	return ALSPS_hw_client;
}

int ALSPS_i2c_add_driver_2nd(void)
{
	int ALSPS_sensor_source;
	int ret = 0;

	/*check i2c function pointer*/
	if(g_ALSPS_I2C == NULL) {
		err("g_ALSPS_I2C is NULL. Please use 'ALSPS_i2c_register' first. \n");
		return -1;
	}

	/* i2c Registration */
	for (ALSPS_sensor_source = 0; ALSPS_sensor_source < ALSPS_hw_source_max; 
			ALSPS_sensor_source++) {
				
		/* i2c Registration and g_client will get i2c client */
		ALSPS_hw_setI2cDriver(&ALSPS_i2c_driver_client, ALSPS_sensor_source);
		ret = i2c_add_driver(&ALSPS_i2c_driver_client);
		if ( ret != 0 ) {
			err("%s: i2c_add_driver ERROR(%d). \n", __FUNCTION__, ret);	
			return -1;
		}
	}

	return 0;
}
EXPORT_SYMBOL(ALSPS_i2c_add_driver_2nd);

ALSPS_hw* ALSPS_hw_getHardware_2nd(void)
{
	int ALSPS_sensor_source;
	int ret = 0;
	ALSPS_hw* ALSPS_hw_client = NULL;

	/*check i2c function pointer*/
	if(g_ALSPS_I2C == NULL) {
		err("g_ALSPS_I2C is NULL. Please use 'ALSPS_i2c_register' first. \n");
		return NULL;
	}

	/* i2c Registration */
	for (ALSPS_sensor_source = 0; ALSPS_sensor_source < ALSPS_hw_source_max; 
			ALSPS_sensor_source++) {
		
		if(g_i2c_client == NULL){
			err("%s: g_i2c_client is NULL pointer. \n", __FUNCTION__);
			return NULL;
		}

		// get hardware client and check the i2c status 
		ALSPS_hw_client = ALSPS_hw_getHardwareClient(ALSPS_sensor_source);
		if(ALSPS_hw_client == NULL){
			err("ALSPS_hw_client is NULL pointer. \n");
			return NULL;
		}
			
		if(ALSPS_hw_client->ALSPS_hw_init == NULL){
			err("ALSPS_hw_init is NULL pointer. \n");
			return NULL;
		}
		ret = ALSPS_hw_client->ALSPS_hw_init(g_i2c_client);
		if (ret < 0) {
			//i2c_del_driver(&ALSPS_i2c_driver_client);
			log("%s %s Probe Fail. \n", __FUNCTION__, ALSPS_i2c_driver_client.driver.name);
			continue;
		}else{
			log("%s %s Probe Success. \n", __FUNCTION__, ALSPS_i2c_driver_client.driver.name);
			break;
		}
	}

	if(ALSPS_sensor_source == ALSPS_hw_source_max) {
		err("There is NO source can Probe.\n");
		return NULL;
	}

	return ALSPS_hw_client;
}
EXPORT_SYMBOL(ALSPS_hw_getHardware_2nd);

