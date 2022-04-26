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

 /*************************************/
/* ALSPS Sensor GPIO Module */
/************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/input/ASH.h>

#define ALSPS_INTEL_NAME 	"ALSPS_INT#_2nd"
#define ALSPS_QCOM_NAME 	"qcom,alsps-gpio_2nd"
#define ALSPS_IRQ_NAME		"ALSPS_SENSOR_IRQ_2nd"
#define ALSPS_INT_NAME		"ALSPS_SENSOR_INT_2nd"
 
static int ALSPS_SENSOR_GPIO;
static ALSPSsensor_GPIO * mALSPSsensor_GPIO;

/******************************/
/* Debug and Log System */
/*****************************/
#define MODULE_NAME			"ASH_GPIO"
#define SENSOR_TYPE_NAME		"ALSPS_2nd"

#undef dbg
#ifdef ASH_GPIO_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,__func__,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

static irqreturn_t ALSPSsensor_irq_handler(int irq, void *dev_id);

#ifdef GPIO_INTEL
#include <asm/intel-mid.h>
#endif

#ifdef GPIO_QCOM
#include <linux/of_gpio.h>
#define GPIO_LOOKUP_STATE	"alsps_gpio_high_2nd"

static void set_pinctrl(struct i2c_client *client)
{
	int ret;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;
	
	key_pinctrl = devm_pinctrl_get(&client->dev);
	set_state = pinctrl_lookup_state(key_pinctrl, GPIO_LOOKUP_STATE);
	ret = pinctrl_select_state(key_pinctrl, set_state);
	if(ret < 0)
		err("%s: pinctrl_select_state ERROR(%d).\n", __FUNCTION__, ret);
}
#endif
static int init_irq (void)
{
	int ret = 0;
	int irq = 0;

	/* GPIO to IRQ */
	irq = gpio_to_irq(ALSPS_SENSOR_GPIO);
	if (irq < 0) {
		err("%s: gpio_to_irq ERROR(%d). \n", __FUNCTION__, irq);
		return irq;
	}else {
		log("gpio_to_irq IRQ %d successed on GPIO:%d\n", irq, ALSPS_SENSOR_GPIO);
	}

	/*Request IRQ*/	
	#ifdef GPIO_INTEL
	ret = request_irq(irq,ALSPSsensor_irq_handler, IRQF_TRIGGER_LOW, ALSPS_INT_NAME, NULL);
	#endif
	
	#ifdef GPIO_QCOM
	ret = request_threaded_irq(irq, NULL, ALSPSsensor_irq_handler,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT, ALSPS_INT_NAME, NULL);
	#endif
	if (ret < 0) {
		err("%s: request_irq/request_threaded_irq ERROR(%d).\n", __FUNCTION__, ret);
		return ret;
	}else {		
		dbg("Disable irq !! \n");
		disable_irq(irq);
	}

	return irq;
}

irqreturn_t ALSPSsensor_irq_handler(int irq, void *dev_id)
{
	mALSPSsensor_GPIO->ALSPSsensor_isr();
	return IRQ_HANDLED;
}

int ALSPSsensor_gpio_register_2nd(struct i2c_client *client, ALSPSsensor_GPIO *gpio_ist)
{
	int ret = 0;
	int irq = 0;

	mALSPSsensor_GPIO = gpio_ist;
	
	/* GPIO */
	#ifdef GPIO_INTEL
	log("Intel GPIO \n");
	ALSPS_SENSOR_GPIO = get_gpio_by_name(ALSPS_INTEL_NAME);
	#endif
	
	#ifdef GPIO_QCOM
	log("Qcom GPIO \n");
	set_pinctrl(client);
//	ALSPS_SENSOR_GPIO = of_get_named_gpio_flags(client->dev.of_node, ALSPS_QCOM_NAME, 0, NULL);
	ALSPS_SENSOR_GPIO = of_get_named_gpio(client->dev.of_node, ALSPS_QCOM_NAME, 0);
	#endif
		
	dbg("[GPIO] GPIO =%d(%d)\n", ALSPS_SENSOR_GPIO, gpio_get_value(ALSPS_SENSOR_GPIO));	
	/* GPIO Request */
	ret = gpio_request(ALSPS_SENSOR_GPIO, ALSPS_IRQ_NAME);
	if (ret) {
		err("%s: gpio_request ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}

	/* GPIO Direction */
	ret = gpio_direction_input(ALSPS_SENSOR_GPIO);

	if (ret < 0) {
		err("%s: gpio_direction_input ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}
	/*IRQ*/
	irq = init_irq();

	return irq;

}
EXPORT_SYMBOL(ALSPSsensor_gpio_register_2nd);


int ALSPSsensor_gpio_unregister_2nd(int irq)
{
	free_irq(irq, NULL);
	gpio_free(ALSPS_SENSOR_GPIO);
	return 0;
}
EXPORT_SYMBOL(ALSPSsensor_gpio_unregister_2nd);
