/*****************************************************************************
* File: i2c-bus.c
*
* (c) 2016 Sentons Inc. - All Rights Reserved.
*
* All information contained herein is and remains the property of Sentons
* Incorporated and its suppliers if any. The intellectual and technical
* concepts contained herein are proprietary to Sentons Incorporated and its
* suppliers and may be covered by U.S. and Foreign Patents, patents in
* process, and are protected by trade secret or copyright law. Dissemination
* of this information or reproduction of this material is strictly forbidden
* unless prior written permission is obtained from Sentons Incorporated.
*
* SENTONS PROVIDES THIS SOURCE CODE STRICTLY ON AN "AS IS" BASIS,
* WITHOUT ANY WARRANTY WHATSOEVER, AND EXPRESSLY DISCLAIMS ALL
* WARRANTIES, EXPRESS, IMPLIED OR STATUTORY WITH REGARD THERETO, INCLUDING
* THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
* PURPOSE, TITLE OR NON-INFRINGEMENT OF THIRD PARTY RIGHTS. SENTONS SHALL
* NOT BE LIABLE FOR ANY DAMAGES SUFFERED BY YOU AS A RESULT OF USING,
* MODIFYING OR DISTRIBUTING THIS SOFTWARE OR ITS DERIVATIVES.
*
*
*****************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/delay.h>
#define CONFIG_I2C_MODULE
#include <linux/i2c.h>

#include "config.h"
#include "device.h"
#include "memory.h"
#include "serial_bus.h"
#include "main.h"
#include "event.h"
#include "hardware.h"
#include "sysfs.h"
#include "utils.h"
#include "debug.h"

#include "sonacomm.h"

#ifdef USE_I2C_BUS
static int snt_i2c_wake_device_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id)
{
	struct device *dev = &i2c->dev;
	struct device_node *np = dev->of_node;
	struct snt8100fsr *snt8100fsr;
	PRINT_FUNC();
	PRINT_INFO("snt_i2c_wake_device_probe Enter");
	PRINT_INFO("I2C Address: 0x%02X", i2c->addr);
	
	snt8100fsr = kzalloc(sizeof(*snt8100fsr), GFP_KERNEL);

	if(!snt8100fsr) {
		PRINT_CRIT("failed to allocate memory for struct snt8100fsr");
		return -ENOMEM;
	}

	memset(snt8100fsr, 0, sizeof(*snt8100fsr));

	snt8100fsr->bus_type = BUS_TYPE_I2C;
	snt8100fsr->dev = dev;
	i2c_set_clientdata(i2c, snt8100fsr);
	snt8100fsr->i2c = i2c;

	/* We use two I2C devices to communicate with the snt8100fsr chip.
	* The main address is used for general communication, the secondary
	* address is used only to awaken the device when it is in sleep mode.
	* We must detect which I2C device is being initialized, the main one,
	* or the secondary address.
	*/	
	if (of_property_read_bool(np, "wake-device")) {
	// Initialization for wake i2c device only, should be put after this line
		PRINT_INFO("Wake I2C device discovered");
		snt8100fsr->wake_i2c_device = true;
		snt8100fsr_wake_i2c_g = snt8100fsr;
	
		// We don't do any more init for this device
		PRINT_DEBUG("done");
		PRINT_INFO("snt_i2c_wake_device_probe End\n");
		return 0;
	}
	PRINT_INFO("snt_i2c_wake_device_probe End\n");
	return 0;
}

static int snt_i2c_wake_device_remove(struct i2c_client *i2c)
{
	struct snt8100fsr *snt8100fsr;
	PRINT_FUNC();

	/* Check if this is the i2c_wake_device used to awaken a sleeping
	* snt8100fsr chip.
	*/
	snt8100fsr = i2c_get_clientdata(i2c);
	if (snt8100fsr && snt8100fsr->wake_i2c_device) {
		PRINT_INFO("removing wake_i2c_device");
		return 0;
	}

	// Else we are on the main i2c device, uninit the sysfs interface
	if (snt8100fsr_g) {
		snt_sysfs_init(snt8100fsr_g, false);
	}

	main_exit();
	return 0;
}

static int snt_i2c_wake_device_suspend(struct device *dev)
{
	int ret;
	PRINT_FUNC();
	PRINT_INFO("snt_i2c_wake_device_suspend+++");
	ret = snt_suspend(dev);
	snt8100fsr_g->suspend_status = true;
	PRINT_INFO("snt_i2c_wake_device_suspend---");
	return ret;
}

static int snt_i2c_wake_device_resume(struct device *dev)
{
	int ret;
	PRINT_FUNC();
	PRINT_INFO("snt_i2c_wake_device_resume+++");
	ret = snt_resume(dev);
	snt8100fsr_g->suspend_status = false;
	PRINT_INFO("snt_i2c_wake_device_resume---");
	return ret;
}

static const struct of_device_id snt8100fsr_i2c_wake_device_dt_ids[] = {
	{ .compatible = "sentons, snt8100fsr-i2c-wake-device" },
};

MODULE_DEVICE_TABLE(of, snt8100fsr_i2c_wake_device_dt_ids);

static struct i2c_device_id snt_i2c_wake_device_id[] = {
	{SENTONS_DRIVER_WAKE_DEVICE_NAME, 0},
	{},
};

static const struct dev_pm_ops snt_i2c_wake_device_pm_ops = {
	.suspend = snt_i2c_wake_device_suspend,
	.resume = snt_i2c_wake_device_resume,
};

static struct i2c_driver snt_i2c_wake_device_driver = {
	.driver = {
		.name = SENTONS_DRIVER_WAKE_DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(snt8100fsr_i2c_wake_device_dt_ids),
		.pm = &snt_i2c_wake_device_pm_ops,
	},
	.probe = snt_i2c_wake_device_probe,
	.remove = snt_i2c_wake_device_remove,
	.id_table = snt_i2c_wake_device_id,
};

MODULE_DEVICE_TABLE(i2c, snt_i2c_wake_device_id);
//module_i2c_driver(snt_i2c_wake_device_driver);


static int __init snt_wake_i2c_init(void)
{
	PRINT_INFO("INIT");
	i2c_add_driver(&snt_i2c_wake_device_driver);
	PRINT_INFO("END");
	return 0;
}

static void __exit snt_wake_i2c_exit(void)
{
	i2c_del_driver(&snt_i2c_wake_device_driver);
	PRINT_INFO("EXIT");
}

module_init(snt_wake_i2c_init);
module_exit(snt_wake_i2c_exit);
MODULE_DESCRIPTION("snt8155_wake Hardware Module");
MODULE_AUTHOR("snt8155_wake, Inc.");
MODULE_LICENSE("GPL v2");
 
#endif
