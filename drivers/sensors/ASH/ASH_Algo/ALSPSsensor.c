/* 
 * Copyright (C) 2014 ASUSTek Inc.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/input.h>
//#include <linux/wakelock.h>
#include <linux/input/ASH.h>
#include "ALSPSsensor.h"
#include "ASH_Wakelock.h"

#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/ioctl.h>

/**************************/
/* Debug and Log System */
/**************************/
#define MODULE_NAME			"ASH_ALGO"
#define SENSOR_TYPE_NAME		"ALSPSsensor"
#undef dbg
#ifdef ASH_ALGO_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,__func__,##args)
#define err(fmt, args...) do{	\
		printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args);	\
		sprintf(g_error_mesg, "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args);	\
	}while(0)

/******************************/
/* ALSPSsensor Sensor Global Variables */
/*****************************/
static int ALSPS_SENSOR_IRQ;
static int ALSPS_SENSOR_INT;
static struct lsensor_data			*g_als_data;
static struct psensor_data			*g_ps_data;
//HW global
struct ALSPS_hw			*g_ALSPS_hw_client;
static struct workqueue_struct 		*ALSPS_workqueue;
static struct workqueue_struct 		*ALSPS_delay_workqueue;
static struct mutex 				g_alsps_lock;
static struct mutex 				g_i2c_lock;
static struct wakeup_source			*g_alsps_wake_lock;
static struct hrtimer 			g_alsps_timer;
static struct i2c_client *g_i2c_client;
static bool g_alsps_probe_status = false;
static int g_als_last_lux = 0;
static char *g_error_mesg;
static int resume_flag = 0;

#define ALSPS_SUSPEND 0
#define ALSPS_RESUME 1
static int g_alsps_power_status = ALSPS_RESUME;
#define WAIT_I2C_DELAY 5
static bool g_psensor_polling_cancel_flag = false;
static int g_pocket_mode_threshold = 0;
static int anti_oil_enable = 1;

/**********************************/
/* ALSPS Sensor IO Control */
/**********************************/
#define VCNL36866_I2C_NAME "VCNL36866"
#define ENABLE_PROXIMITY_IOCTL_LIB 1
#define ENABLE_LIGHT_IOCTL_LIB 1
static int prox_open_count = 0;
static int light_open_count = 0;


/***********************/
/* ALSPS Sensor Functions*/
/**********************/
/*Device Layer Part*/
static int 	proximity_turn_onoff(bool bOn);
static int 	proximity_set_threshold(void);
static void proximity_polling_adc(struct work_struct *work);
static void proximity_check_period(void);
static int 	light_turn_onoff(bool bOn);
static int 	light_get_lux(int adc);
static int 	light_get_accuracy_gain(void);
static void light_polling_lux(struct work_struct *work);

/*Interrupt Service Routine Part*/
static void ALSPS_ist(struct work_struct *work);

/* Check for power off */
static int check_ALSP_onoff(void);

/* For ALSPS check event rest time */
static int ALS_check_event_rest_time(int lux);

/* Light dynamic ctl checl*/
static int ALS_dynamic_ctl_check(int lux);

/* Export Functions */
bool proximityStatus(void);

/*Initialization Part*/
static int init_data(void);

/*Proximity auto calibration*/
static void proximity_autok(struct work_struct *work);
static int proximity_check_minCT(void);

/* Light sensor average array reset for psensor noise issue */
static void light_sensor_reset_status(void);

/* CFI failure, can't do this when write 1 to load_cal.
Therefore, do this function at turn_onoff when load_cal = 1 */
static int mproximity_store_load_calibration_data(void);

/*Work Queue*/
static 		DECLARE_WORK(ALSPS_ist_work, ALSPS_ist);
static 		DECLARE_WORK(proximity_autok_work, proximity_autok);
static 		DECLARE_DELAYED_WORK(proximity_polling_adc_work, proximity_polling_adc);
static 		DECLARE_DELAYED_WORK(light_polling_lux_work, light_polling_lux);

/*Disable touch for detecting near when phone call*/
//extern void ftxxxx_disable_touch(bool flag);
extern int get_audiomode(void);

/*******************************/
/* ALS and PS data structure */
/******************************/
struct psensor_data 
{	
	int g_ps_calvalue_lo;						/* Proximitysensor setting low calibration value(adc) */
	int g_ps_calvalue_hi;						/* Proximitysensor setting high calibration value(adc) */
	int g_ps_calvalue_inf;						/* Proximitysensor setting inf calibration value(adc) */

	int g_ps_factory_cal_lo;				/* Proximitysensor factory low calibration value(adc) */
	int g_ps_factory_cal_hi;				/* Proximitysensor factory high calibration value(adc) */

	int g_ps_autok_min;
	int g_ps_autok_max;
	
	bool HAL_switch_on;						/* this var. means if HAL is turning on ps or not */
	bool Device_switch_on;					/* this var. means is turning on ps or not */	
	bool polling_mode;							/* Polling for adc of proximity */
	bool autok;							/*auto calibration status*/
	
	int g_ps_int_status;					/* Proximitysensor interrupt status */

	int int_counter;
	int event_counter;
	int crosstalk_diff;

	int selection;
	/* ASUS BSP+++ Clay: dynamic proxmity period due to psensor noise effect */
	int cur_period;
	/* ASUS BSP--- */
};

struct lsensor_data 
{	
	int g_als_calvalue;						/* Lightsensor calibration value(adc) */
	int g_als_accuracy_gain;					/* Lightsensor Gain calibration value X LIGHT_GAIN_ACCURACY_CALVALUE*/
	int g_als_change_sensitivity;			/* Lightsensor Change sensitivity */
	int g_als_log_threshold;				/* Lightsensor Log Print Threshold */
	bool g_als_log_first_event;                             /* Lightsensor Log Print Threshold */

	bool HAL_switch_on;						/* this var. means if HAL is turning on als or not */
	bool Device_switch_on;					/* this var. means if als hw is turn on or not */

	int int_counter;
	int event_counter;
	
	int selection;

	int dynamic_sensitive;			/* used in dynamic control machanism */
	uint8_t dynamic_IT;					/* used in dynamic control machanism */
	
	struct timespec ts; 
	u64 evt_skip_time_ns;
	
	int g_als_retry_count;                          /* polling workqueue retry to get adc count, set 0 when polling cancel */
	
	/* ASUS BSP Clay: shift lux to mitigate psensor noise when psensor on and lux < offset +++ */
	int offset_adc; 					/* get adc should shift offset_adc when psensor on to mitigate psensor noise */
	int offset_lux; 					/* reported lux should shift offset_lux when psensor on to mitigate psensor noise*/
	/* ASUS BSP Clay: shift lux to mitigate psensor noise when psensor on and lux < offset--- */

	/* ASUS BSP Clay: average 5 lux for offset behavior to mitigate the low lux gap +++ */
	int avg_array_index;
	int avg_array[5];
	bool avg_enable_flag;
	/* ASUS BSP Clay: average 5 lux for offset behavior to mitigate the low lux gap --- */
	
	bool freeze_psensor;
};

/*=======================
 *|| I2c Stress Test Part ||
 *=======================
 */

#ifdef CONFIG_I2C_STRESS_TEST

#include <linux/i2c_testcase.h>

#define I2C_TEST_Lsensor_FAIL (-1)
#define I2C_TEST_Psensor_FAIL (-1)

static int ALSPS_I2C_stress_test(struct i2c_client *client)
{
	int lnResult = I2C_TEST_PASS;
	int ret = 0;
	int adc = 0;
	int low_threshold = 0;
	int high_threshold = 0;

	i2c_log_in_test_case("TestIRSensorI2C ++\n");

	/* Check Hardware Support First */
	if(g_ALSPS_hw_client->mpsensor_hw->proximity_hw_turn_onoff== NULL) {
		err("proximity_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(g_ALSPS_hw_client->mpsensor_hw->proximity_hw_get_adc == NULL) {
		err("proximity_hw_get_adc NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(g_ALSPS_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold == NULL) {
		err("proximity_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(g_ALSPS_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold == NULL) {
		err("proximity_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(g_ALSPS_hw_client->mlsensor_hw->light_hw_turn_onoff== NULL) {
		err("light_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(g_ALSPS_hw_client->mlsensor_hw->light_hw_get_adc == NULL) {
		err("light_hw_get_adc NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(g_ALSPS_hw_client->mlsensor_hw->light_hw_set_hi_threshold == NULL) {
		err("light_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(g_ALSPS_hw_client->mlsensor_hw->light_hw_set_lo_threshold == NULL) {
		err("light_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}

	/* Turn on Proximity and Light Sensor */
	if(!g_ps_data->Device_switch_on){
		ret = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);
	}
	if(!g_als_data->Device_switch_on){
		ret = g_ALSPS_hw_client->mlsensor_hw->light_hw_turn_onoff(true);
	}

	/* Proximity i2c read test */
	ret = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_get_adc();
	if(ret < 0){
		i2c_log_in_test_case("IRsensor Proximity Fail to get adc\n");
		lnResult = I2C_TEST_Psensor_FAIL;
		return lnResult;	
	}

	/* Proximity i2c write test */
	ret = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold(g_ps_data->g_ps_calvalue_hi);
	if(ret < 0){
		i2c_log_in_test_case("IRsensor Proximity Fail to set high threshold.\n");
		lnResult = I2C_TEST_Psensor_FAIL;
		return lnResult;	
	}
	ret = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold(g_ps_data->g_ps_calvalue_lo);
	if(ret < 0){
		i2c_log_in_test_case("IRsensor Proximity Fail to set low threshold. \n");
		lnResult = I2C_TEST_Psensor_FAIL;
		return lnResult;
	}

	/* Light Sensor i2c read test */
	adc = g_ALSPS_hw_client->mlsensor_hw->light_hw_get_adc();

	if(adc < 0){
		i2c_log_in_test_case("IRsensor Light Sensor Fail to get adc\n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;
	}
	/* Light Sensor Low Threshold */
	low_threshold = adc * (100 - LIGHT_CHANGE_MID_SENSITIVITY) / 100;

	/* Light Sensor High Threshold */
	high_threshold = adc * (100 + LIGHT_CHANGE_MID_SENSITIVITY) / 100;
	if (high_threshold > g_ALSPS_hw_client->mlsensor_hw->light_max_threshold)
		high_threshold = g_ALSPS_hw_client->mlsensor_hw->light_max_threshold;

	ret = g_ALSPS_hw_client->mlsensor_hw->light_hw_set_hi_threshold(high_threshold);
	if(ret < 0) {
		i2c_log_in_test_case("IRsensor Light Sensor Fail to set high threshold. \n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;
	}

	ret = g_ALSPS_hw_client->mlsensor_hw->light_hw_set_lo_threshold(low_threshold);
	if(ret < 0) {
		i2c_log_in_test_case("IRsensor Light Sensor Fail to set low threshold. \n");
		lnResult = I2C_TEST_Lsensor_FAIL;
		return lnResult;
	}

	if(!g_ps_data->HAL_switch_on) {
		ret = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);
	}
	if(!g_als_data->HAL_switch_on) {
		ret = g_ALSPS_hw_client->mlsensor_hw->light_hw_turn_onoff(false);
	}
	
	i2c_log_in_test_case("TestLSensorI2C --\n");
	return lnResult;
}

static struct i2c_test_case_info ALSPS_TestCaseInfo[] ={
	__I2C_STRESS_TEST_CASE_ATTR(ALSPS_I2C_stress_test),
};
#endif

/*====================
 *|| Device Layer Part ||
 *====================
 */ 
 static void proximity_turn_on_check(void){
	int adc_value, threshold_high;
	if(g_ps_data->HAL_switch_on==true && g_als_data->freeze_psensor == false){
		if(g_ps_data->Device_switch_on == false){
			log("turn on proximity since freeze_psensor = %d", g_als_data->freeze_psensor);
			proximity_turn_onoff(true);
			/* ASUS BSP Clay: average 5 lux for offset behavior to mitigate the low lux gap +++ */
			light_sensor_reset_status();
			/* ASUS BSP Clay: ---*/
			msleep(PROXIMITY_TURNON_DELAY_TIME);
			
			adc_value = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_get_adc();
			threshold_high = (g_ps_data->g_ps_calvalue_hi);
			log("Proximity adc_value=%d, threshold_high=%d\n", adc_value, threshold_high);
			if (adc_value < threshold_high) {
				psensor_report_abs(PSENSOR_REPORT_PS_AWAY);
				g_ps_data->g_ps_int_status = ALSPS_INT_PS_AWAY;
				log("Proximity Report First Away abs.\n");
			}
		}else{
			dbg("Enable psensor already");
		}
	}else{
		dbg("skip turn on proximity since psensor=%d, freeze_psensor = %d", 
			g_ps_data->HAL_switch_on, g_als_data->freeze_psensor);
	}
}

static void psensor_onoff_recovery(bool bOn){
#ifdef RECOVERY_PSENSOR
	if(g_als_data->freeze_psensor == bOn){
		g_als_data->freeze_psensor = !bOn;
		dbg("freeze_psensor=%d", g_als_data->freeze_psensor);
		if(bOn){ //psensor on
			if(g_ps_data->HAL_switch_on == true){
				dbg("Re-enable psensor");
				proximity_turn_on_check();
			}
		}else{ //psensor off
			if(g_ps_data->HAL_switch_on == true){
				log("Close psensor temporary");
				proximity_turn_onoff(bOn);
			}
		}
	}
#endif
}

static int proximity_turn_onoff(bool bOn)
{
	int ret = 0;
	ktime_t autok_delay;
	/* Check Hardware Support First */
	if(g_ALSPS_hw_client->mpsensor_hw->proximity_hw_turn_onoff == NULL) {
		err("proximity_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(g_ALSPS_hw_client->mpsensor_hw->proximity_hw_interrupt_onoff == NULL) {
		err("proximity_hw_interrupt_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}

	if (bOn == 1)	{	/* power on */
		/*set turn on register*/
		if(g_ps_data->Device_switch_on == false){
			/*Power ON*/
			ret = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);
			if(ret < 0){
				err("proximity_hw_turn_onoff(true) ERROR\n");
				return ret;
			}

			proximity_check_period();

			/*Enable INT*/
			ret = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_interrupt_onoff(true);
			if(ret < 0){
				err("proximity_hw_interrupt_onoff(true) ERROR\n");
				return ret;
			}
		}

		/*Set Proximity Threshold*/
		ret = proximity_set_threshold();
		if (ret < 0) {	
			err("proximity_set_threshold ERROR\n");
			return ret;
		}

		/*check the min for auto calibration*/
		if(true == g_ps_data->autok){
			g_ps_data->crosstalk_diff = 0;
			/*Stage 1 : check first 6 adc which spend about 50ms~100ms*/
			ret = proximity_check_minCT();
			if (ret < 0) {	
				log("proximity_check_minCT ERROR\n");	
				g_ps_data->autok = false;
			}
		}
		
		/*enable IRQ only when proximity and light sensor is off*/
		if (g_ps_data->Device_switch_on == false && g_als_data->Device_switch_on == false) {
			dbg("[IRQ] Enable irq !! \n");
			enable_irq(ALSPS_SENSOR_IRQ);
		}
		/*change the Device Status*/
		g_ps_data->Device_switch_on = true;
		/*check the polling mode*/
		if(g_ps_data->polling_mode == true) {
			queue_delayed_work(ALSPS_delay_workqueue, &proximity_polling_adc_work, msecs_to_jiffies(PROXIMITY_POLLING_TIME));
			log("[Polling] Proximity polling adc START. \n");
		}

		/*Stage 2 : start polling proximity adc(500ms) to check min value*/
		if(true == g_ps_data->autok && g_ps_data->crosstalk_diff != 0){
			autok_delay = ns_to_ktime( PROXIMITY_AUTOK_POLLING * NSEC_PER_MSEC);
			hrtimer_start(&g_alsps_timer, autok_delay, HRTIMER_MODE_REL);
		}

	} else{/* power off */
		/*set turn off register*/
		if(g_ps_data->Device_switch_on == true){
			/*disable IRQ before switch off*/
			dbg("[IRQ] Disable irq !! \n");
			disable_irq_nosync(ALSPS_SENSOR_IRQ);

			ret = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);
			if(ret < 0){
				err("proximity_hw_turn_onoff(false) ERROR\n");
			}

			/*Disable INT*/
			ret = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_interrupt_onoff(false);
			if(ret < 0){
				err("proximity_hw_interrupt_onoff(false) ERROR\n");
			}


			/*reset the threshold data*/
			g_ps_data->g_ps_calvalue_hi = g_ps_data->g_ps_factory_cal_hi;
			g_ps_data->g_ps_calvalue_lo = g_ps_data->g_ps_factory_cal_lo;

			/*change the Device Status*/
			g_ps_data->Device_switch_on = false;

			/*enable IRQ when light sensor is ON*/
			if (g_als_data->Device_switch_on == true) {
				dbg("[IRQ] Enable irq !! \n");
				enable_irq(ALSPS_SENSOR_IRQ);
			}

			/*diable the timer*/
			if(g_ps_data->autok == true){
				hrtimer_cancel(&g_alsps_timer);
			}

			proximity_check_period();
		}
	}

	if(check_ALSP_onoff()){
		g_ALSPS_hw_client->ALSPS_hw_close_power();
	}
	return ret;
}

static int proximity_set_threshold(void)
{
	int ret = 0;
	int temp = 0;

	/* Check Hardware Support First */
	if(g_ALSPS_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold == NULL) {
		err("proximity_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(g_ALSPS_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold == NULL) {
		err("proximity_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	//Set Proximity High Threshold
	/*
	ret = psensor_factory_read_high(PSENSOR_HI_CALIBRATION_FILE);

	//For transition period from 3/5 to 2/4 +++//
	if(0 == g_ps_data->selection)
		ret = psensor_factory_read_2cm(PSENSOR_2CM_CALIBRATION_FILE);
	else if(1 == g_ps_data->selection)
		ret = psensor_factory_read_3cm(PSENSOR_3CM_CALIBRATION_FILE);
	else
		err("INVALID selection : %d\n", g_ps_data->selection);
	//For transition period from 3/5 to 2/4 ---/

	if(ret > 0) {
		g_ps_data->g_ps_calvalue_hi = ret;
		g_ps_data->g_ps_factory_cal_hi = ret;
		log("Proximity read High Calibration : %d\n", g_ps_data->g_ps_calvalue_hi);
	}else{
		err("Proximity read DEFAULT High Calibration : %d\n", g_ps_data->g_ps_calvalue_hi);
	}
	ret = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold(g_ps_data->g_ps_calvalue_hi);
	if(ret < 0){
		err("proximity_hw_set_hi_threshold ERROR. \n");
		return -ENOENT;
	}

	//Set Proximity Low Threshold
	ret = psensor_factory_read_low(PSENSOR_LOW_CALIBRATION_FILE);

	//For transition period from 3/5 to 2/4 +++
	if(0 == g_ps_data->selection)
		ret = psensor_factory_read_4cm(PSENSOR_4CM_CALIBRATION_FILE);
	else if(1 == g_ps_data->selection)
		ret = psensor_factory_read_5cm(PSENSOR_5CM_CALIBRATION_FILE);
	else
		err("INVALID selection : %d\n", g_ps_data->selection);
	//For transition period from 3/5 to 2/4 ---

	if(ret > 0) {
		g_ps_data->g_ps_calvalue_lo = ret;
		log("Proximity read Low Calibration : %d\n", g_ps_data->g_ps_calvalue_lo);
	}else{
		err("Proximity read DEFAULT Low Calibration : %d\n", g_ps_data->g_ps_calvalue_lo);
	}
	*/
	if(anti_oil_enable == 1){
		temp = (g_ps_data->g_ps_calvalue_hi - g_ps_data->g_ps_calvalue_lo) / 3;
		g_ps_data->g_ps_calvalue_lo = temp + g_ps_data->g_ps_calvalue_lo;
		if(g_ps_data->g_ps_calvalue_lo > g_ps_data->g_ps_calvalue_hi - 5){
			g_ps_data->g_ps_calvalue_lo = g_ps_data->g_ps_calvalue_hi - 5;
			log("Proximity final low threshold:%d\n", g_ps_data->g_ps_calvalue_lo);
		} else {
			log("Proximity final low threshold:%d (offset:%d)\n", g_ps_data->g_ps_calvalue_lo, temp);
		}
	}

	ret = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_set_hi_threshold(g_ps_data->g_ps_calvalue_hi);
	if(ret < 0){
		err("proximity_hw_set_hi_threshold ERROR. \n");
		return -ENOENT;
	}
	
	ret = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_set_lo_threshold(g_ps_data->g_ps_calvalue_lo);
	if(ret < 0){
		err("proximity_hw_set_lo_threshold ERROR. \n");
		return -ENOENT;
	}
/*
	ret = psensor_factory_read_1cm(PSENSOR_1CM_CALIBRATION_FILE);
	if(ret > 0){
		g_pocket_mode_threshold = ret;
		log("Proximity read Pocket Mode Calibration : %d\n", g_pocket_mode_threshold);
	}else{
		err("Proximity read DEFAULT Pocket Mode Calibration : %d\n", g_pocket_mode_threshold);
	}
*/
	log("Proximity set threshold hi: %d, low: %d\n", 
			g_ps_data->g_ps_calvalue_hi, g_ps_data->g_ps_calvalue_lo);
	return 0;
}

static void proximity_polling_adc(struct work_struct *work)
{
	int adc_value = 0;

	/* Check Hardware Support First */
	if(g_ALSPS_hw_client->mpsensor_hw->proximity_hw_get_adc == NULL) {
		err("proximity_hw_get_adc NOT SUPPORT. \n");
	} else {
		mutex_lock(&g_alsps_lock);
		/* proximity sensor go to suspend, cancel proximity get adc polling */
		if(g_alsps_power_status == ALSPS_SUSPEND){
			g_psensor_polling_cancel_flag = true;
			log("proximity sensor has suspended, cancel proximity get adc polling!!");
		} else {
			if(g_ps_data->Device_switch_on == true) {
				adc_value = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_get_adc();
				dbg("[Polling] Proximity get adc = %d\n", adc_value);
				
				if(adc_value < 0){
					err("Proximity get adc ERROR\n");	
				} else {
					if(g_ps_data->g_ps_int_status != ALSPS_INT_PS_CLOSE &&
							(adc_value >= g_ps_data->g_ps_calvalue_hi &&
							(g_pocket_mode_threshold <= 0 || adc_value < g_pocket_mode_threshold))) {
						log("[Polling] Proximity Detect Object Close. (adc = %d)\n", adc_value);
						psensor_report_abs(PSENSOR_REPORT_PS_CLOSE);
						g_ps_data->g_ps_int_status = ALSPS_INT_PS_CLOSE;
					}else if(g_ps_data->g_ps_int_status != ALSPS_INT_PS_AWAY &&
							adc_value <= g_ps_data->g_ps_calvalue_lo){
						log("[Polling] Proximity Detect Object Away. (adc = %d)\n", adc_value);
						psensor_report_abs(PSENSOR_REPORT_PS_AWAY);
						g_ps_data->g_ps_int_status = ALSPS_INT_PS_AWAY;
					}else if(g_ps_data->g_ps_int_status != ALSPS_INT_PS_POCKET &&
							(g_pocket_mode_threshold > 0 && adc_value >= g_pocket_mode_threshold)){
						log("[Polling] Proximity Detect Object Close. (adc = %d, distance <= 1cm)\n", adc_value);
						psensor_report_abs(PSENSOR_REPORT_PS_POCKET);
						g_ps_data->g_ps_int_status = ALSPS_INT_PS_POCKET;
					}
				}
			}
			if(g_ps_data->Device_switch_on == true)
				queue_delayed_work(ALSPS_delay_workqueue, &proximity_polling_adc_work, msecs_to_jiffies(PROXIMITY_POLLING_TIME));
			else
				log("[Polling] Proximity polling adc STOP. \n");
		}
		mutex_unlock(&g_alsps_lock);
	}
}

/* ASUS BSP+++ Clay: dynamic proxmity period due to psensor noise effect */
static void proximity_check_period(void){
	/* ASUS BSP Clay: don't do anything when there is no noise at this project */
	if(PROXIMITY_NOISE_PERIOD == PROXIMITY_PERIOD){
		return;
	}

	if(g_ps_data->Device_switch_on == true || g_ps_data->HAL_switch_on == true){
		if(g_als_data->Device_switch_on == true || g_als_data->HAL_switch_on == true){
			if(g_ps_data->cur_period != PROXIMITY_NOISE_PERIOD){
				g_ps_data->cur_period = PROXIMITY_NOISE_PERIOD;
				g_ALSPS_hw_client->mpsensor_hw->proximity_hw_set_period(g_ps_data->cur_period);
				log("Proximity period set to %d when Light sensor on", g_ps_data->cur_period);
			}
		}else{
			if(g_ps_data->cur_period != PROXIMITY_PERIOD){
				g_ps_data->cur_period = PROXIMITY_PERIOD;
				g_ALSPS_hw_client->mpsensor_hw->proximity_hw_set_period(g_ps_data->cur_period);
				log("Proximity period recovery to %d when Light sensor off", g_ps_data->cur_period);
			}
		}
	}else{
		g_ps_data->cur_period = PROXIMITY_PERIOD;
	}
}
/* ASUS BSP--- */

static int light_turn_onoff(bool bOn)
{
	int ret=0;

	/* Check Hardware Support First */
	if(g_ALSPS_hw_client->mlsensor_hw->light_hw_turn_onoff == NULL) {
		err("light_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(g_ALSPS_hw_client->mlsensor_hw->light_hw_interrupt_onoff == NULL) {
		err("light_hw_interrupt_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(g_ALSPS_hw_client->mlsensor_hw->light_hw_set_hi_threshold == NULL) {
		err("light_hw_set_hi_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(g_ALSPS_hw_client->mlsensor_hw->light_hw_set_lo_threshold == NULL) {
		err("light_hw_set_lo_threshold NOT SUPPORT. \n");
		return -ENOENT;
	}
	
	if (bOn == 1)	{	/* power on */	
		if(g_als_data->Device_switch_on == false) {
			/*Power ON*/
			ret = g_ALSPS_hw_client->mlsensor_hw->light_hw_turn_onoff(true);
			if(ret < 0){
				err("light_hw_turn_onoff(true) ERROR. \n");
				return -ENOENT;
			}
			proximity_check_period();
			/*Enable INT*/
			ret = g_ALSPS_hw_client->mlsensor_hw->light_hw_interrupt_onoff(true);
			if(ret < 0){
				err("light_hw_interrupt_onoff(true) ERROR. \n");
				return -ENOENT;
			}
		}
		
		if(1 == resume_flag){
			resume_flag=0;
		}else{
			light_get_accuracy_gain();
			
			/* ASUS BSP Clay: shift lux to mitigate psensor noise when psensor on and lux < offset +++ */
			//get offset_lux after first get accuracy gain
			if(g_als_data->offset_lux == 0){
				g_als_data->offset_lux = light_get_lux(g_als_data->offset_adc);
			}
			/* ASUS BSP Clay: shift lux to mitigate psensor noise when psensor on and lux < offset --- */
		}
		
		log("[Cal] Light Sensor Set Accuracy Gain : %d, Cal : %d\n", g_als_data->g_als_accuracy_gain, g_als_data->g_als_calvalue);

		if(g_als_data->Device_switch_on == false) {
			ret = g_ALSPS_hw_client->mlsensor_hw->light_hw_set_hi_threshold(0);
			if(ret < 0){
				err("light_hw_set_hi_threshold ERROR. \n");
				return -ENOENT;
			}
			ret = g_ALSPS_hw_client->mlsensor_hw->light_hw_set_lo_threshold(0);
			if(ret < 0){
				err("light_hw_set_lo_threshold ERROR. \n");
				return -ENOENT;
			}
		}
		/*enable IRQ only when proximity and light sensor is off*/
		if (g_ps_data->Device_switch_on == false && g_als_data->Device_switch_on == false) {
			dbg("[IRQ] Enable irq !! \n");
			enable_irq(ALSPS_SENSOR_IRQ);
		}
		g_als_data->Device_switch_on = true;
		g_als_data->g_als_log_first_event = true;	
	} else	{	/* power off */	
		/*set turn off register*/
		if(g_als_data->Device_switch_on == true){
			/*disable IRQ before switch off*/		
			dbg("[IRQ] Disable irq !! \n");
			disable_irq_nosync(ALSPS_SENSOR_IRQ);

			/*Power OFF*/
			ret = g_ALSPS_hw_client->mlsensor_hw->light_hw_turn_onoff(false);
			if(ret < 0){
				err("light_hw_turn_onoff(false) ERROR. \n");
				return ret;
			}
			/*Disbale INT*/
			ret = g_ALSPS_hw_client->mlsensor_hw->light_hw_interrupt_onoff(false);
			if(ret < 0){
				err("light_hw_interrupt_onoff(false) ERROR. \n");
				return ret;
			}

			g_als_data->Device_switch_on = false;

			/*enable IRQ when proximity sensor is ON*/
			if (g_ps_data->Device_switch_on == true) {
				dbg("[IRQ] Enable irq !! \n");
				enable_irq(ALSPS_SENSOR_IRQ);
			}
			proximity_check_period();
		}
		g_ALSPS_hw_client->mlsensor_hw->light_hw_reset_ALS_dynamic_status();
		g_als_data->dynamic_sensitive = g_ALSPS_hw_client->mlsensor_hw->light_hw_get_current_sensitive();
		g_als_data->dynamic_IT = g_ALSPS_hw_client->mlsensor_hw->light_hw_get_current_IT();
	}
	if(check_ALSP_onoff()){
		g_ALSPS_hw_client->ALSPS_hw_close_power();
	}
	return ret;
}

/* ASUS BSP Clay: average 5 lux for offset behavior to mitigate the low lux gap +++ */
static void light_sensor_reset_status(void){
	g_als_data->avg_enable_flag = false;
	g_als_data->avg_array_index = 0;
	memset(g_als_data->avg_array, -1, sizeof(g_als_data->avg_array));
}
/* ASUS BSP Clay: average 5 lux for offset behavior to mitigate the low lux gap --- */
/* ASUS BSP Clay: shift lux to mitigate psensor noise when psensor on and lux < offset +++ */
static int light_lux_check_psensor_noise(int lux)
{
	int index = 0;
	int lux_temp = 0;
	/* ASUS BSP Clay: when offset_adc = 0, don't do offset behavior */
	if(g_als_data->offset_adc == 0){
		return lux;
	}

	if(g_ps_data->Device_switch_on == true || g_ps_data->HAL_switch_on == true){
		if(lux <= g_als_data->offset_lux){
			dbg("psensor on, lux < offset_lux(%d), return 0", lux,  g_als_data->offset_lux);
			lux = 0;
		}else{/* Do nothing */	}

		/* ASUS BSP Clay: average 5 lux for offset behavior to mitigate the low lux gap +++ */
		if(lux > 100 && g_als_data->avg_enable_flag == true){
			log("Light Sensor disable avg workaround when pon and lux > 100, lux=%d", lux);
			light_sensor_reset_status();
		}else if(lux <= 100 && g_als_data->avg_enable_flag == false){
			g_als_data->avg_enable_flag = true;
			log("Light Sensor enable avg workaround when pon and lux <= 100");
		}else{/* Do nothing */}

		if(g_als_data->avg_enable_flag == true){
			g_als_data->avg_array[g_als_data->avg_array_index] = lux;
			g_als_data->avg_array_index++;
			if(g_als_data->avg_array_index == LIGHT_LOW_LUX_AVG_COUNT){
				g_als_data->avg_array_index = 0;
			}
			for(index = 0; index < LIGHT_LOW_LUX_AVG_COUNT; index++){
				if(g_als_data->avg_array[index] >= 0){
					lux_temp += g_als_data->avg_array[index];
				}else{
					break;
				}
			}
			dbg("total final_lux=%d", lux_temp);
			lux = lux_temp / index;
			if(g_als_data->avg_array_index == 1){ /* print data when save data in first array */
				log("index = %d, final_lux=%d, lux_array=[%d, %d, %d, %d, %d]", index, lux
					, g_als_data->avg_array[0], g_als_data->avg_array[1], g_als_data->avg_array[2]
					, g_als_data->avg_array[3], g_als_data->avg_array[4]);
			}else{/* Do nothing */}
		}
		/* ASUS BSP Clay: average 5 lux for offset behavior to mitigate the low lux gap --- */
	}

	return lux;
}

static int light_adc_check_psensor_noise(int adc)
{
	int lux = -1;
	/* ASUS BSP Clay: when adc = 0, don't do offset behavior */
	if(g_als_data->offset_adc == 0){
		return adc;
	}
	
	lux = light_get_lux(adc);
	if(lux < g_als_data->offset_lux){
		if(g_ps_data->Device_switch_on == true || g_ps_data->HAL_switch_on == true){
			adc = 0;
			log("psensor on, lux=%d, final_adc=%d, offset_adc=%d", lux, adc, g_als_data->offset_adc);
		}else{/* Do nothing */	}
	}else{/* Do nothing */}
	return adc;
}
/* ASUS BSP Clay: shift lux to mitigate psensor noise when psensor on and lux < offset --- */

static int light_suspend_turn_off(bool bOn)
{
	int ret=0;

	/* Check Hardware Support First */
	if(g_ALSPS_hw_client->mlsensor_hw->light_hw_turn_onoff == NULL) {
		err("light_hw_turn_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}
	if(g_ALSPS_hw_client->mlsensor_hw->light_hw_interrupt_onoff == NULL) {
		err("light_hw_interrupt_onoff NOT SUPPORT. \n");
		return -ENOENT;
	}

	/* power off */	
	/*set turn off register*/
	if(g_als_data->Device_switch_on == true){
		/*disable IRQ before switch off*/
		dbg("[IRQ] Disable irq !! \n");
		disable_irq_nosync(ALSPS_SENSOR_IRQ);

		/*Power OFF*/
		ret = g_ALSPS_hw_client->mlsensor_hw->light_hw_turn_onoff(false);
		if(ret < 0){
			err("light_hw_turn_onoff(false) ERROR. \n");
		}
		/*Disbale INT*/
		ret = g_ALSPS_hw_client->mlsensor_hw->light_hw_interrupt_onoff(false);
		if(ret < 0){
			err("light_hw_interrupt_onoff(false) ERROR. \n");
			return ret;
		}

		g_als_data->Device_switch_on = false;
		/*enable IRQ when proximity sensor is ON*/
		if (g_ps_data->HAL_switch_on == true) {
			dbg("[IRQ] Enable irq !! \n");
			enable_irq(ALSPS_SENSOR_IRQ);
		}
		g_ALSPS_hw_client->mlsensor_hw->light_hw_reset_ALS_dynamic_status();
		g_als_data->dynamic_sensitive = g_ALSPS_hw_client->mlsensor_hw->light_hw_get_current_sensitive();
		g_als_data->dynamic_IT = g_ALSPS_hw_client->mlsensor_hw->light_hw_get_current_IT();
	}

	return ret;
}

static int light_get_lux(int adc)
{
	int lux = 0;

	if(adc < 0) {
		err("Light Sensor get Lux ERROR. (adc < 0)\n");
		return 0;
	}
	
	lux = (adc * g_als_data->g_als_accuracy_gain) / LIGHT_GAIN_ACCURACY_CALVALUE;
	
	//if(lux > LIGHT_MAX_LUX)
	//	lux = LIGHT_MAX_LUX;
	
	return lux;
}

static int light_get_accuracy_gain(void)
{
	int cal = 0;
	int gainvalue = 0;

	/* Light Sensor Read Calibration*/
	cal = lsensor_factory_read(LSENSOR_CALIBRATION_FILE);

#if defined LONG_LENGTH_LOGN_IT_NON_PERF || defined LONG_LENGTH_LOGN_IT_PERF
	if(0 == g_als_data->selection){
		cal = lsensor_factory_read_50ms(LSENSOR_400MS_CALIBRATION_FILE);
		log("Light Sensor read calibration 400ms: %d", cal);
	}else if(1 == g_als_data->selection){
		cal = lsensor_factory_read_100ms(LSENSOR_200MS_CALIBRATION_FILE);
		log("Light Sensor read calibration 200ms: %d", cal);
	}
#else
	if(0 == g_als_data->selection){
		cal = lsensor_factory_read_50ms(LSENSOR_50MS_CALIBRATION_FILE);
		log("Light Sensor read calibration 50ms: %d", cal);
	}else if(1 == g_als_data->selection){
		cal = lsensor_factory_read_100ms(LSENSOR_100MS_CALIBRATION_FILE);
		log("Light Sensor read calibration 100ms: %d", cal);
	}
#endif
	else
		err("INVALID selection : %d\n", g_als_data->selection);
	
	if(cal > 0 ){
		g_als_data->g_als_calvalue = cal;
	}
	gainvalue = (1000*LIGHT_GAIN_ACCURACY_CALVALUE)/
				(g_als_data->g_als_calvalue);
	dbg("%d, %d", g_als_data->g_als_calvalue, gainvalue);
	g_als_data->g_als_accuracy_gain = gainvalue;
	
	return gainvalue;
}

static void light_polling_lux(struct work_struct *work)
{
	int adc = 0;
	int lux = 0;
	static int limit_count = 3;
	static int log_count = 0;
	static unsigned int polling_time = 50;
	/* Check Hardware Support First */
	if(g_ALSPS_hw_client->mlsensor_hw->light_hw_get_adc == NULL) {
		err("light_hw_get_adc NOT SUPPORT. \n");
	}

mutex_lock(&g_alsps_lock);

	if(g_als_data->HAL_switch_on == true) {
		/* Light Sensor Report the first real event*/
		adc = g_ALSPS_hw_client->mlsensor_hw->light_hw_get_adc();
		if ((0 == adc) && (g_als_data->g_als_retry_count < limit_count)) {
			g_als_data->g_als_retry_count++;
			if(g_als_data->g_als_retry_count == limit_count){
				log("[Polling1] Light Sensor retry for get adc\n");
			}
			log_count = limit_count;
		}else if (adc < 0){
			log("[Polling] read adc < 0 (%d), do nothing", adc);
		} else {
			lux = light_get_lux(adc) * g_als_data->dynamic_sensitive;
			/* ASUS BSP Clay: shift lux to mitigate psensor noise when psensor on and lux < offset  +++ */
			lux = light_lux_check_psensor_noise(lux);
			/* ASUS BSP Clay: shift lux to mitigate psensor noise when psensor on and lux < offset --- */
			if(ALS_check_event_rest_time(lux)==1){
				psensor_onoff_recovery(true);
				if(g_als_last_lux != lux){
					if(lux < 100 && (log_count >= (limit_count/2))){
						log("[Polling2] Light Sensor Report lux : %d (adc = %d), last_lux=%d, count=%d, poll_t=%u\n"
							, lux, adc, g_als_last_lux, g_als_data->g_als_retry_count, polling_time);
						log_count = 0;
					}else if (log_count >= limit_count){
						log("[Polling2] Light Sensor Report lux : %d (adc = %d), last_lux=%d, count=%d, poll_t=%u\n"
							, lux, adc, g_als_last_lux, g_als_data->g_als_retry_count, polling_time);
						log_count = 0;
					}else{
						log_count++;
					}
				}
				if((g_als_last_lux <= 1000 && lux > 1000) ||
					(g_als_last_lux > 1000 && lux <= 1000)){
					ALS_dynamic_ctl_check(lux);
				}
				g_als_last_lux = lux;
			}
		}
		/* dynamic sensitive case */
		switch(g_als_data->dynamic_IT){
			case CS_IT_400MS:
				limit_count = 5;
				polling_time = 400;
				break;
			case CS_IT_50MS:
				limit_count = 5;
				polling_time = 400;
				break;
			case CS_IT_100MS:
				limit_count = 10;
				polling_time = 11;
				break;
			default:
				break;
		}
		cancel_delayed_work(&light_polling_lux_work);
		queue_delayed_work(ALSPS_delay_workqueue, &light_polling_lux_work, msecs_to_jiffies(polling_time));
	}
mutex_unlock(&g_alsps_lock);
}

/********************************/
/*Proximity sensor Info Type*/
/*******************************/
static psensor_info_type mpsensor_info_type = {{0}};

/***************************/
/*Light sensor Info Type*/
/***************************/
static lsensor_info_type mlsensor_info_type = {{0}};

	
/**********************/
/*Calibration Function*/
/*********************/
static int mproximity_show_calibration_hi(void)
{
	int calvalue;
	int ret = 0;

	calvalue = psensor_factory_read_high(PSENSOR_HI_CALIBRATION_FILE);

	/*For transition period from 3/5 to 2/4 +++*/
	if(0 == g_ps_data->selection)
		calvalue = psensor_factory_read_2cm(PSENSOR_2CM_CALIBRATION_FILE);
	else if(1 == g_ps_data->selection)
		calvalue = psensor_factory_read_3cm(PSENSOR_3CM_CALIBRATION_FILE);
	else
		err("INVALID selection : %d\n", g_ps_data->selection);
	/*For transition period from 3/5 to 2/4 ---*/
	
	if(calvalue > 0) {
		g_ps_data->g_ps_calvalue_hi = calvalue;
		log("Proximity read High Calibration : %d\n", g_ps_data->g_ps_calvalue_hi);
	}else{
		err("Proximity read DEFAULT High Calibration : %d\n", g_ps_data->g_ps_calvalue_hi);
	}

	ret = g_ps_data->g_ps_calvalue_hi;
	dbg("Proximity show High Calibration: %d\n", ret);
	return ret;
}

static int mproximity_store_calibration_hi(int calvalue)
{
	if(calvalue <= 0) {
		err("Proximity store High Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;
	}
	log("Proximity store High Calibration: %d\n", calvalue);
	psensor_factory_write_high(calvalue, PSENSOR_HI_CALIBRATION_FILE);

	/*For transition period from 3/5 to 2/4 +++*/
	if(0 == g_ps_data->selection)
		psensor_factory_write_2cm(calvalue, PSENSOR_2CM_CALIBRATION_FILE);
	else if(1 == g_ps_data->selection)
		psensor_factory_write_3cm(calvalue, PSENSOR_3CM_CALIBRATION_FILE);
	else
		err("INVALID selection : %d\n", g_ps_data->selection);
	/*For transition period from 3/5 to 2/4 ---*/

	proximity_set_threshold();

	return 0;
}

static int mproximity_show_calibration_lo(void)
{
	int calvalue;
	int ret = 0;

	calvalue = psensor_factory_read_low(PSENSOR_LOW_CALIBRATION_FILE);

	/*For transition period from 3/5 to 2/4 +++*/
	if(0 == g_ps_data->selection)
		calvalue = psensor_factory_read_4cm(PSENSOR_4CM_CALIBRATION_FILE);
	else if(1 == g_ps_data->selection)
		calvalue = psensor_factory_read_5cm(PSENSOR_5CM_CALIBRATION_FILE);
	else
		err("INVALID selection : %d\n", g_ps_data->selection);
	/*For transition period from 3/5 to 2/4 ---*/
	
	if(calvalue > 0) {
	    	g_ps_data->g_ps_calvalue_lo = calvalue;
		log("Proximity read Low Calibration : %d\n", g_ps_data->g_ps_calvalue_lo);
	}else{
		err("Proximity read DEFAULT Low Calibration : %d\n", g_ps_data->g_ps_calvalue_lo);
	}

	ret = g_ps_data->g_ps_calvalue_lo;
	dbg("Proximity show Low Calibration: %d\n", ret);
	return ret;
}

static int mproximity_store_calibration_lo(int calvalue)
{
	if(calvalue <= 0) {
		err("Proximity store Low Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;
	}
	log("Proximity store Low Calibration: %d\n", calvalue);
	psensor_factory_write_low(calvalue, PSENSOR_LOW_CALIBRATION_FILE);

	/*For transition period from 3/5 to 2/4 +++*/
	if(0 == g_ps_data->selection)
		psensor_factory_write_4cm(calvalue, PSENSOR_4CM_CALIBRATION_FILE);
	else if(1 == g_ps_data->selection)
		psensor_factory_write_5cm(calvalue, PSENSOR_5CM_CALIBRATION_FILE);
	else
		err("INVALID selection : %d\n", g_ps_data->selection);
	/*For transition period from 3/5 to 2/4 ---*/

	proximity_set_threshold();

	return 0;
}

static int mproximity_show_calibration_inf(void)
{
	int calvalue;
	calvalue = psensor_factory_read_inf(PSENSOR_INF_CALIBRATION_FILE);
	dbg("Proximity show Inf Calibration: %d\n", calvalue);
	return calvalue;
}

static int mproximity_store_calibration_inf(int calvalue)
{
	if(calvalue <= 0) {
		err("Proximity store Inf Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;
	}
	log("Proximity store Inf Calibration: %d\n", calvalue);
	psensor_factory_write_inf(calvalue, PSENSOR_INF_CALIBRATION_FILE);
	
	return 0;
}

static int mproximity_show_adc(void)
{
	int adc = 0;
	int ret;
	
	if(g_ALSPS_hw_client->mpsensor_hw->proximity_hw_get_adc == NULL){
		err("proximity_hw_get_adc NOT SUPPORT. \n");
		return -EINVAL;
	}

	mutex_lock(&g_alsps_lock);

	if(g_ps_data->Device_switch_on == false){
		ret = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);
		if(ret < 0){
			err("proximity_hw_turn_onoff(true) ERROR\n");
			return ret;
		}
		msleep(PROXIMITY_TURNON_DELAY_TIME);
	}

	adc = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_get_adc();
	dbg("mproximity_show_adc : %d \n", adc);
	
	if(g_ps_data->HAL_switch_on == false){
		ret = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);
		if(ret < 0){
			err("proximity_hw_turn_onoff(false) ERROR\n");
			return ret;
		}
	}
	
	mutex_unlock(&g_alsps_lock);
	
	return adc;
}

static int mlight_show_calibration(void)
{
	int calvalue;
	int ret = 0;

	calvalue = lsensor_factory_read(LSENSOR_CALIBRATION_FILE);
#if defined LONG_LENGTH_LOGN_IT_NON_PERF || defined LONG_LENGTH_LOGN_IT_PERF
	if(0 == g_als_data->selection){
		calvalue = lsensor_factory_read_50ms(LSENSOR_400MS_CALIBRATION_FILE);
		log("Light Sensor read calibration(show) 400ms: %d", calvalue);
	}else if(1 == g_als_data->selection){
		calvalue = lsensor_factory_read_100ms(LSENSOR_200MS_CALIBRATION_FILE);
		log("Light Sensor read calibration(show) 200ms: %d", calvalue);
	}
#else
	if(0 == g_als_data->selection){
		calvalue = lsensor_factory_read_50ms(LSENSOR_50MS_CALIBRATION_FILE);
		log("Light Sensor read calibration(show) 50ms: %d", calvalue);
	}else if(1 == g_als_data->selection){
		calvalue = lsensor_factory_read_100ms(LSENSOR_100MS_CALIBRATION_FILE);
		log("Light Sensor read calibration(show) 100ms: %d", calvalue);
	}
#endif
	else
		err("INVALID selection : %d\n", g_als_data->selection);
	
	if(calvalue > 0 ){
		g_als_data->g_als_calvalue = calvalue;
	}
	ret = g_als_data->g_als_calvalue;
	dbg("Light Sensor show Calibration: %d\n", ret);
	return ret;
}

static int mlight_store_calibration(int calvalue)
{
	if(calvalue <= 0) {
		err("Light Sensor store Calibration with NON-POSITIVE value. (%d) \n", calvalue);
		return -EINVAL;
	}
	log("Light Sensor store Calibration: %d\n", calvalue);
	lsensor_factory_write(calvalue, LSENSOR_CALIBRATION_FILE);

#if defined LONG_LENGTH_LOGN_IT_NON_PERF || defined LONG_LENGTH_LOGN_IT_PERF
	if(0 == g_als_data->selection){
		lsensor_factory_write_50ms(calvalue, LSENSOR_400MS_CALIBRATION_FILE);
		log("Light Sensor write calibration 400ms: %d", calvalue);
	}else if(1 == g_als_data->selection){
		lsensor_factory_write_100ms(calvalue, LSENSOR_200MS_CALIBRATION_FILE);
		log("Light Sensor write calibration 200ms: %d", calvalue);
	}
#else
	if(0 == g_als_data->selection){
		lsensor_factory_write_50ms(calvalue, LSENSOR_50MS_CALIBRATION_FILE);
		log("Light Sensor write calibration 50ms: %d", calvalue);
	}else if(1 == g_als_data->selection){
		lsensor_factory_write_100ms(calvalue, LSENSOR_100MS_CALIBRATION_FILE);
		log("Light Sensor write calibration 100ms: %d", calvalue);
	}
#endif
	else
		err("INVALID selection : %d\n", g_als_data->selection);
	
	if(calvalue > 0 ){
		g_als_data->g_als_calvalue = calvalue;
	}
	light_get_accuracy_gain();

	return 0;
}

static int mlight_show_adc(void)
{
	int adc = 0;
	if(g_ALSPS_hw_client->mlsensor_hw->light_hw_get_adc == NULL) {
		err("light_hw_get_adc NOT SUPPORT. \n");
		return -EINVAL;
	}

	mutex_lock(&g_alsps_lock);

	if (!g_als_data->Device_switch_on) {
		light_turn_onoff(true);
#if defined LONG_LENGTH_LOGN_IT_NON_PERF || defined LONG_LENGTH_LOGN_IT_PERF
		msleep(LIGHT_TURNON_DELAY_TIME*50);
#else
		msleep(LIGHT_TURNON_DELAY_TIME*10);
#endif
	}

	adc = g_ALSPS_hw_client->mlsensor_hw->light_hw_get_adc();
	adc *=g_als_data->dynamic_sensitive;	//if IT= 100, adc should *4 to simulate adc value based on 400 IT time
	/* ASUS BSP Clay: shift lux to mitigate psensor noise when psensor on and lux < offset +++ */
	adc = light_adc_check_psensor_noise(adc);
	/* ASUS BSP Clay: shift lux to mitigate psensor noise when psensor on and lux < offset --- */
	dbg("mlight_show_adc : %d \n", adc);

	if (!g_als_data->HAL_switch_on) {
		light_turn_onoff(false);
	}
	
	mutex_unlock(&g_alsps_lock);
	return adc;
}

static int mlight_show_gain(void)
{
	int gainvalue;
	gainvalue = light_get_accuracy_gain();
	dbg("Light Sensor show Gain Calibration: %d.%d\n",
		gainvalue/LIGHT_GAIN_ACCURACY_CALVALUE, gainvalue%LIGHT_GAIN_ACCURACY_CALVALUE);

	return gainvalue;
}

static psensor_ATTR_Calibration mpsensor_ATTR_Calibration = {
	.proximity_show_calibration_hi = mproximity_show_calibration_hi,
	.proximity_store_calibration_hi = mproximity_store_calibration_hi,
	.proximity_show_calibration_lo = mproximity_show_calibration_lo,
	.proximity_store_calibration_lo = mproximity_store_calibration_lo,
	.proximity_show_calibration_inf = mproximity_show_calibration_inf ,
	.proximity_store_calibration_inf  = mproximity_store_calibration_inf ,
	.proximity_show_adc = mproximity_show_adc,
};

static lsensor_ATTR_Calibration mlsensor_ATTR_Calibration = {
	.light_show_calibration = mlight_show_calibration,
	.light_store_calibration = mlight_store_calibration,
	.light_show_gain = mlight_show_gain,
	.light_show_adc = mlight_show_adc,
};

/******************/
/*BMMI Function*/
/****************/
static bool mproximity_show_atd_test(void)
{
	int ret=0;
	int round=0;

	ret = g_ALSPS_hw_client->ALSPS_hw_check_ID();
	if(ret < 0){
		err("Proximity ATD test check ID ERROR\n");
		goto proximity_atd_test_fail;
	}

	if(g_ps_data->Device_switch_on == false){
		ret = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);		
		if(ret < 0){
			err("Proximity ATD test turn on ERROR\n");
			goto proximity_atd_test_fail;
		}
	}
	
	for(;round<5; round++){
		ret = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_get_adc();
		if(ret < 0){
			err("Proximity ATD test get adc ERROR\n");
			goto proximity_atd_test_fail;
		}
		msleep(100);
	}

	if(g_ps_data->HAL_switch_on == false){
		ret = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);	
		if(ret < 0){
			err("Proximity ATD test turn off ERROR\n");
			goto proximity_atd_test_fail;
		}
	}

	return true;
proximity_atd_test_fail:
	return false;
}

static bool mlight_show_atd_test(void)
{
	int ret=0;
	int round=0;

	ret = g_ALSPS_hw_client->ALSPS_hw_check_ID();
	if(ret < 0){
		err("Light Sensor ATD test check ID ERROR\n");
		goto light_atd_test_fail;
	}

	if (!g_als_data->Device_switch_on) {
		ret = light_turn_onoff(true);
		if(ret < 0){
			err("Light Sensor ATD test turn on ERROR\n");
			goto light_atd_test_fail;
		}
	}
	
	for(; round<5; round++){
		ret = g_ALSPS_hw_client->mlsensor_hw->light_hw_get_adc();
		if(ret < 0){
			err("Light Sensor ATD test get adc ERROR\n");
			goto light_atd_test_fail;
		}
		msleep(100);
	}
	
	if (!g_als_data->HAL_switch_on) {
		ret = light_turn_onoff(false);
		if(ret < 0){
			err("Light Sensor ATD test turn off ERROR\n");
			goto light_atd_test_fail;
		}
	}

	return true;
light_atd_test_fail:
	return false;

}

static psensor_ATTR_BMMI mpsensor_ATTR_BMMI = {
	.proximity_show_atd_test = mproximity_show_atd_test,
};

static lsensor_ATTR_BMMI mlsensor_ATTR_BMMI = {
	.light_show_atd_test = mlight_show_atd_test,
};

/*********************/
/*Hardware Function*/
/********************/
static int mproximity_show_reg(uint8_t addr)
{
	int value;
	if(g_ALSPS_hw_client->ALSPS_hw_get_register == NULL) {
		err("proximity_hw_get_register NOT SUPPORT. \n");
		return -EINVAL;
	}

	value = g_ALSPS_hw_client->ALSPS_hw_get_register(addr);
	log("mproximity_show_reg, addr=%02X, value=%02X.\n", addr, value);
	return value;
}

static int mproximity_store_reg(uint8_t addr, int value)
{
	if(g_ALSPS_hw_client->ALSPS_hw_set_register == NULL) {
		err("proximity_hw_set_register NOT SUPPORT. \n");
		return -EINVAL;
	}

	g_ALSPS_hw_client->ALSPS_hw_set_register(addr, value);
	log("mproximity_store_reg, addr=%02X, value=%02X.\n", addr, value);
	return 0;
}

static int mlight_show_reg(uint8_t addr)
{
	int value;
	if(g_ALSPS_hw_client->ALSPS_hw_get_register == NULL) {
		err("light_hw_get_register NOT SUPPORT. \n");
		return -EINVAL;
	}

	value = g_ALSPS_hw_client->ALSPS_hw_get_register(addr);
	log("mlight_show_reg, addr=%02X, value=%02X.\n", addr, value);
	return value;
}

static int mlight_store_reg(uint8_t addr, int value)
{
	if(g_ALSPS_hw_client->ALSPS_hw_set_register == NULL) {
		err("light_hw_set_register NOT SUPPORT. \n");
		return -EINVAL;
	}

	g_ALSPS_hw_client->ALSPS_hw_set_register(addr, value);
	log("mlight_store_reg, addr=%02X, value=%02X.\n", addr, value);
	return 0;
}


static psensor_ATTR_Hardware mpsensor_ATTR_Hardware = {
	.proximity_show_reg = mproximity_show_reg,
	.proximity_store_reg = mproximity_store_reg,
};

static lsensor_ATTR_Hardware mlsensor_ATTR_Hardware = {
	.light_show_reg = mlight_show_reg,
	.light_store_reg = mlight_store_reg,
};

/****************/
/*HAL Function*/
/***************/
static bool mproximity_show_switch_onoff(void)
{
	return g_ps_data->Device_switch_on;
}

extern bool g_Psensor_load_cal_status;
static int mproximity_store_switch_onoff(bool bOn)
{
	mutex_lock(&g_alsps_lock);
	log("Proximity switch = %d.\n", bOn);
	if ((g_ps_data->Device_switch_on != bOn)){
		if (bOn == true)	{
			/* Turn on Proxomity */
			g_ps_data->HAL_switch_on = true;
			if(g_Psensor_load_cal_status){
				log("load_cal = %d, load calibration data", g_Psensor_load_cal_status);
				mproximity_store_load_calibration_data();
				g_Psensor_load_cal_status = 0;
			}
			proximity_turn_on_check();
		} else	{
			/* Turn off Proxomity */
			g_ps_data->HAL_switch_on = false;
			proximity_turn_onoff(false);
			log("Proximity Report final Away\n");
			g_ps_data->g_ps_int_status = ALSPS_INT_PS_INIT;
			psensor_report_abs(-1);
			//ftxxxx_disable_touch(false);
		}
	}else{
		log("Proximity is already %s", bOn?"ON":"OFF");
	}
	mutex_unlock(&g_alsps_lock);
	
	return 0;
}

static bool mlight_show_switch_onoff(void)
{
	return g_als_data->Device_switch_on;
}

static int mlight_store_switch_onoff(bool bOn)
{
	mutex_lock(&g_alsps_lock);
	dbg("Light Sensor switch1 = %d.\n", bOn);
	if((g_als_data->Device_switch_on != bOn)){
		if(bOn == true){
			/* Turn on Light Sensor */
			g_als_data->HAL_switch_on = true;
			psensor_onoff_recovery(false);
			light_turn_onoff(true);
			/* ASUS BSP Clay: average 5 lux for offset behavior to mitigate the low lux gap +++ */
			light_sensor_reset_status();
			/* ASUS BSP Clay: average 5 lux for offset behavior to mitigate the low lux gap --- */
			/*light sensor polling the first real event after delayed time. */
			g_als_data->g_als_retry_count = 0;
			cancel_delayed_work(&light_polling_lux_work);
			queue_delayed_work(ALSPS_delay_workqueue, &light_polling_lux_work, msecs_to_jiffies(LIGHT_TURNON_DELAY_TIME));
		}else{
			/* Turn off Light Sensor */
			g_als_data->HAL_switch_on = false;
			g_ALSPS_hw_client->mlsensor_hw->light_hw_reset_ALS_dynamic_status();
			g_als_data->dynamic_IT = g_ALSPS_hw_client->mlsensor_hw->light_hw_get_current_IT();
			log("Before Light Sensor turn off, reset IT time\n");
			g_ALSPS_hw_client->mlsensor_hw->light_hw_set_integration(g_als_data->dynamic_IT);
			light_turn_onoff(false);
			/* Report lux=-1 when turn off */
			lsensor_report_lux(-1);
			cancel_delayed_work(&light_polling_lux_work);
			psensor_onoff_recovery(true);
		}
	}else{
		log("Light Sensor is already %s", bOn?"ON":"OFF");
		cancel_delayed_work(&light_polling_lux_work);
		queue_delayed_work(ALSPS_delay_workqueue, &light_polling_lux_work, msecs_to_jiffies(LIGHT_TURNON_DELAY_TIME));
	}
	mutex_unlock(&g_alsps_lock);
	return 0;
}

static int mlight_show_lux(void)
{
	int adc = 0;
	int lux = 0;

	mutex_lock(&g_alsps_lock);

	if (!g_als_data->Device_switch_on) {
		light_turn_onoff(true);
	}
	
	msleep(LIGHT_TURNON_DELAY_TIME);

	adc = g_ALSPS_hw_client->mlsensor_hw->light_hw_get_adc();

	lux = light_get_lux(adc) * g_als_data->dynamic_sensitive; //if IT= 100, adc should *4 to simulate lux value based on 400 IT time

	/* ASUS BSP Clay: shift lux to mitigate psensor noise when psensor on and lux < offset  +++ */
	lux = light_lux_check_psensor_noise(lux);
	/* ASUS BSP Clay: shift lux to mitigate psensor noise when psensor on and lux < offset--- */

	dbg("mlight_show_lux : %d \n", lux);

	if (!g_als_data->HAL_switch_on) {
		light_turn_onoff(false);
	}
	
	mutex_unlock(&g_alsps_lock);
	
	return lux;	
}

static psensor_ATTR_HAL mpsensor_ATTR_HAL = {
	.proximity_show_switch_onoff = mproximity_show_switch_onoff,
	.proximity_store_switch_onoff = mproximity_store_switch_onoff,
	.proximity_show_status = proximityStatus,
};

static lsensor_ATTR_HAL mlsensor_ATTR_HAL = {
	.light_show_switch_onoff = mlight_show_switch_onoff,
	.light_store_switch_onoff = mlight_store_switch_onoff,
	.light_show_lux = mlight_show_lux,
};

/*********************/
/*Extension Function*/
/********************/
static bool mproximity_show_allreg(void)
{
	if(g_ALSPS_hw_client->ALSPS_hw_show_allreg == NULL) {
		err("IRsensor_hw_show_allreg NOT SUPPORT. \n");
		return false;
	}
	g_ALSPS_hw_client->ALSPS_hw_show_allreg();
	return true;
}

static bool mproximity_show_polling_mode(void)
{
	return g_ps_data->polling_mode;
}

static int mproximity_store_polling_mode(bool bOn)
{
	g_ps_data->polling_mode = bOn;
	return 0;
}

static bool mproximity_show_autok(void)
{
	return g_ps_data->autok;
}

static int mproximity_store_autok(bool bOn)
{
	g_ps_data->autok = bOn;	
	return 0;
}

static int mproximity_show_int_count(void)
{
	return g_ps_data->int_counter;
}

static int mproximity_show_event_count(void)
{
	return g_ps_data->event_counter;
}

static int mproximity_show_autokmin(void)
{
	return g_ps_data->g_ps_autok_min;
}

static int mproximity_store_autokmin(int autokmin)
{
	g_ps_data->g_ps_autok_min = autokmin;
	log("Proximity store autokmin: %d\n", autokmin);	
	return 0;
}

static int mproximity_show_autokmax(void)
{
	return g_ps_data->g_ps_autok_max;
}

static int mproximity_store_autokmax(int autokmax)
{
	g_ps_data->g_ps_autok_max = autokmax;
	log("Proximity store autokmax: %d\n", autokmax);	
	return 0;
}

static int mproximity_show_error_mesg(char *error_mesg)
{
	memcpy(error_mesg, g_error_mesg, strlen(g_error_mesg)+1);
	return 0;
}

static int mproximity_show_selection(void)
{
	return g_ps_data->selection;
}

static int mproximity_store_selection(int selection)
{
	int ret=0;
	g_ps_data->selection= selection;
	log("Proximity store selection: %d\n", selection);

	ret = proximity_set_threshold();
	if (ret < 0) {
		err("proximity_set_threshold ERROR\n");
		return ret;
	}

	return 0;
}

/*For load calibration data*/
static int mproximity_store_load_calibration_data(void)
{
	int ret=0;

	ret = psensor_factory_read_inf(PSENSOR_INF_CALIBRATION_FILE);
	if(ret >= 0) {
	    	g_ps_data->g_ps_calvalue_inf= ret;
		log("Proximity read INF Calibration : %d\n", g_ps_data->g_ps_calvalue_inf);
	}else{
		err("Proximity read DEFAULT INF Calibration : %d\n", g_ps_data->g_ps_calvalue_inf);
	}

	ret = psensor_factory_read_1cm(PSENSOR_1CM_CALIBRATION_FILE);
	if(ret > 0){
		g_pocket_mode_threshold = ret;
		log("Proximity read Pocket Mode Calibration : %d\n", g_pocket_mode_threshold);
	}else{
		err("Proximity read DEFAULT Pocket Mode Calibration : %d\n", g_pocket_mode_threshold);
	}

	/*For transition period from 3/5 to 2/4 +++*/
	if(0 == g_ps_data->selection)
		ret = psensor_factory_read_2cm(PSENSOR_2CM_CALIBRATION_FILE);
	else if(1 == g_ps_data->selection)
		ret = psensor_factory_read_3cm(PSENSOR_3CM_CALIBRATION_FILE);
	else
		err("INVALID selection : %d\n", g_ps_data->selection);
	/*For transition period from 3/5 to 2/4 ---*/
	if(ret > 0) {
		g_ps_data->g_ps_calvalue_hi = ret;
		g_ps_data->g_ps_factory_cal_hi = ret;
		log("Proximity read High Calibration : %d\n", g_ps_data->g_ps_factory_cal_hi);
	}else{
		err("Proximity read DEFAULT High Calibration : %d\n", g_ps_data->g_ps_factory_cal_hi);
	}

	/*For transition period from 3/5 to 2/4 +++*/
	if(0 == g_ps_data->selection)
		ret = psensor_factory_read_4cm(PSENSOR_4CM_CALIBRATION_FILE);
	else if(1 == g_ps_data->selection)
		ret = psensor_factory_read_5cm(PSENSOR_5CM_CALIBRATION_FILE);
	else
		err("INVALID selection : %d\n", g_ps_data->selection);
	/*For transition period from 3/5 to 2/4 ---*/

	if(ret > 0) {
		g_ps_data->g_ps_calvalue_lo = ret;
		g_ps_data->g_ps_factory_cal_lo = ret;
		log("Proximity read Low Calibration : %d\n", g_ps_data->g_ps_factory_cal_lo);
	}else{
		err("Proximity read DEFAULT Low Calibration : %d\n", g_ps_data->g_ps_factory_cal_lo);
	}

	log("Proximity load factory Calibration done!\n");
	return ret;
}

/*For enable anti-oil workaround*/
static int mproximity_show_anti_oil_enable(void)
{
	return anti_oil_enable;
}

static int mproximity_store_anti_oil_enable(bool enable)
{
	int ret=0;
	
	if(enable){
		anti_oil_enable = 1;
		log("Proximity store enable anti-oil: %d\n", anti_oil_enable);
	} else {
		anti_oil_enable = 0;
		log("Proximity store disable anti-oil: %d\n", anti_oil_enable);
	}
	
	return ret;
}

static bool mlight_show_allreg(void)
{
	if(g_ALSPS_hw_client->ALSPS_hw_show_allreg == NULL) {
		err("IRsensor_hw_show_allreg NOT SUPPORT. \n");
		return false;
	}
	g_ALSPS_hw_client->ALSPS_hw_show_allreg();
	return true;
}

static int mlight_show_sensitivity(void)
{
	return g_als_data->g_als_change_sensitivity;
}

static int mlight_store_sensitivity(int sensitivity)
{
	g_als_data->g_als_change_sensitivity = sensitivity;
	log("Light Sensor store Sensitivity: %d\n", sensitivity);
	
	return 0;
}

static int mlight_show_log_threshold(void)
{
	return g_als_data->g_als_log_threshold;
}

static int mlight_store_log_threshold(int log_threshold)
{
	g_als_data->g_als_log_threshold = log_threshold;
	log("Light Sensor store Log Threshold: %d\n", log_threshold);
	
	return 0;
}

static int mlight_show_int_count(void)
{
	return g_als_data->int_counter;
}

static int mlight_show_event_count(void)
{
	return g_als_data->event_counter;
}

static int mlight_show_error_mesg(char *error_mesg)
{
	memcpy(error_mesg, g_error_mesg, strlen(g_error_mesg)+1);
	return 0;
}

static int mlight_show_selection(void)
{
	return g_als_data->selection;
}

static int mlight_store_selection(int selection)
{
	bool turnoff_flag=false;
	mutex_lock(&g_alsps_lock);
	g_als_data->selection= selection;

	if(g_als_data->Device_switch_on){
		light_turn_onoff(false);
		turnoff_flag = true;	
	}	
	if(true == turnoff_flag){
		mdelay(LIGHT_TURNON_DELAY_TIME);
	}
	
	/*For transition period from 100ms to 50ms*/
	if(0 == selection){
		g_ALSPS_hw_client->mlsensor_hw->light_hw_set_integration(0);
	}else if(1 == selection){
		g_ALSPS_hw_client->mlsensor_hw->light_hw_set_integration(1);
	}else{
		err("INVALID selection : %d\n", selection);
	}

	if(g_als_data->HAL_switch_on){
		light_turn_onoff(true);
	}
	
	//log("Light Sensor store selection: %d\n ", selection);
	light_get_accuracy_gain();

	mutex_unlock(&g_alsps_lock);
	return 0;
}

static psensor_ATTR_Extension mpsensor_ATTR_Extension = {
	.proximity_show_allreg = mproximity_show_allreg,
	.proximity_show_polling_mode = mproximity_show_polling_mode,
	.proximity_store_polling_mode = mproximity_store_polling_mode,
	.proximity_show_autok = mproximity_show_autok,
	.proximity_store_autok = mproximity_store_autok,
	.proximity_show_int_count = mproximity_show_int_count,
	.proximity_show_event_count = mproximity_show_event_count,
	.proximity_show_autokmin = mproximity_show_autokmin,
	.proximity_store_autokmin = mproximity_store_autokmin,
	.proximity_show_autokmax = mproximity_show_autokmax,
	.proximity_store_autokmax = mproximity_store_autokmax,
	.proximity_show_error_mesg = mproximity_show_error_mesg,
	.proximity_show_selection = mproximity_show_selection,
	.proximity_store_selection = mproximity_store_selection,
	/*For load calibration data*/
	.proximity_store_load_calibration_data = mproximity_store_load_calibration_data,
	/*For enable anti-oil workaround*/
	.proximity_show_anti_oil_enable = mproximity_show_anti_oil_enable,
	.proximity_store_anti_oil_enable = mproximity_store_anti_oil_enable,
};

static lsensor_ATTR_Extension mlsensor_ATTR_Extension = {
	.light_show_allreg = mlight_show_allreg,
	.light_show_sensitivity = mlight_show_sensitivity,
	.light_store_sensitivity = mlight_store_sensitivity,
	.light_show_log_threshold = mlight_show_log_threshold,
	.light_store_log_threshold = mlight_store_log_threshold,
	.light_show_int_count = mlight_show_int_count,
	.light_show_event_count = mlight_show_event_count,
	.light_show_error_mesg = mlight_show_error_mesg,
	.light_show_selection = mlight_show_selection,
	.light_store_selection = mlight_store_selection
};

static psensor_ATTR mpsensor_ATTR = {
	.info_type = &mpsensor_info_type,
	.ATTR_Calibration = &mpsensor_ATTR_Calibration,
	.ATTR_BMMI = &mpsensor_ATTR_BMMI,
	.ATTR_Hardware = &mpsensor_ATTR_Hardware,
	.ATTR_HAL = &mpsensor_ATTR_HAL,
	.ATTR_Extension = &mpsensor_ATTR_Extension,
};

static lsensor_ATTR mlsensor_ATTR = {
	.info_type = &mlsensor_info_type,
	.ATTR_Calibration = &mlsensor_ATTR_Calibration,
	.ATTR_BMMI = &mlsensor_ATTR_BMMI,
	.ATTR_Hardware = &mlsensor_ATTR_Hardware,
	.ATTR_HAL = &mlsensor_ATTR_HAL,
	.ATTR_Extension = &mlsensor_ATTR_Extension,
};

/*================================
 *|| Interrupt Service Routine Part ||
 *================================
 */
static void proximity_work(int state)
{
	int adc = 0;
	//int audio_mode = 0;

	/* Get Proximity adc value */
	adc= g_ALSPS_hw_client->mpsensor_hw->proximity_hw_get_adc();
	if(adc < 0){
		err("[ISR] Proximity get adc ERROR\n");	
		return;
	}

	/* Ignore the interrupt when Switch off */
	if(g_ps_data->HAL_switch_on == true)
	{
		/* Check proximity close or away. */
		if(ALSPS_INT_PS_AWAY == state) {
			log("[ISR] Proximity Detect Object Away. (adc = %d)\n", adc);
			psensor_report_abs(PSENSOR_REPORT_PS_AWAY);
			g_ps_data->g_ps_int_status = ALSPS_INT_PS_AWAY;
			g_ps_data->event_counter++;	/* --- For stress test debug --- */
			//audio_mode = get_audiomode();
			//if (0 == audio_mode || 2 == audio_mode || 3 == audio_mode) {
			//	ftxxxx_disable_touch(false);
			//}
		} else if (ALSPS_INT_PS_CLOSE == state) {
			if(g_pocket_mode_threshold > 0 && adc >= g_pocket_mode_threshold){
				log("[ISR] Proximity Detect Object Close. (adc = %d, distance <= 1cm)\n", adc);
				psensor_report_abs(PSENSOR_REPORT_PS_POCKET);
				g_ps_data->g_ps_int_status = ALSPS_INT_PS_POCKET;
			}else{
				log("[ISR] Proximity Detect Object Close. (adc = %d)\n", adc);
				psensor_report_abs(PSENSOR_REPORT_PS_CLOSE);
				g_ps_data->g_ps_int_status = ALSPS_INT_PS_CLOSE;
			}
			g_ps_data->event_counter++;	/* --- For stress test debug --- */
			//audio_mode = get_audiomode();
			//if (2 == audio_mode || 3 == audio_mode) {
			//	ftxxxx_disable_touch(true);
			//}
		} else {
			err("[ISR] Proximity Detect Object ERROR. (adc = %d)\n", adc);
		}
	}
	
}

static void light_work(void)
{
	int low_threshold = 0;
	int high_threshold = 0;
	int adc = 0;
	int lux = 0;
	int ret = 0;
	int light_change_sensitivity = 0;
	int light_log_threshold = 0;

	/* Ignore the interrupt when Switch off */
	if(g_als_data->HAL_switch_on == true)
	{
		adc = g_ALSPS_hw_client->mlsensor_hw->light_hw_get_adc();

		dbg("[ISR] Light Sensor Get adc : %d\n", adc);
		if(adc < 0){
			err("light_hw_get_adc ERROR\n");
			return;
		}

		/* Set the default sensitivity (3rd priority)*/
		if(adc >= g_als_data->g_als_calvalue / g_als_data->dynamic_sensitive) {
			light_change_sensitivity = LIGHT_CHANGE_LOW_SENSITIVITY;
		}else {
			light_change_sensitivity = LIGHT_CHANGE_HI_SENSITIVITY;
		}

		/* Set the factory sensitivity (2nd priority) */
#ifdef ASUS_FTM_BUILD
		light_change_sensitivity = LIGHT_CHANGE_FACTORY_SENSITIVITY;
#endif

		/* Set the interface sensitivity (1st priority) */
		if(g_als_data->g_als_change_sensitivity >= 0)
			light_change_sensitivity = g_als_data->g_als_change_sensitivity;
		
		dbg("[ISR] Light Sensor Set Sensitivity. (light_change_sensitivity:%d)\n", light_change_sensitivity);

		/* Light Sensor Low Threshold */	
		low_threshold = adc * (100 - light_change_sensitivity) / 100;

		/* Light Sensor High Threshold */
		high_threshold = (adc * (100 + light_change_sensitivity) / 100) + 1;
		if (high_threshold > g_ALSPS_hw_client->mlsensor_hw->light_max_threshold)
			high_threshold = g_ALSPS_hw_client->mlsensor_hw->light_max_threshold;

		ret = g_ALSPS_hw_client->mlsensor_hw->light_hw_set_hi_threshold(high_threshold);
		if(ret < 0) {
			err("[ISR] Light Sensor Set High Threshold ERROR. (High:%d)\n", high_threshold);
		}
		dbg("[ISR] Light Sensor Set High Threshold. (High:%d)\n", high_threshold);	

		ret = g_ALSPS_hw_client->mlsensor_hw->light_hw_set_lo_threshold(low_threshold);
		if(ret < 0) {
			err("[ISR] Light Sensor Set Low Threshold ERROR. (Low:%d)\n", low_threshold);
		}
		dbg("[ISR] Light Sensor Set Low Threshold. (Low:%d)\n", low_threshold);	
		
		/* Light Sensor Report input event*/
		lux = light_get_lux(adc);

		if(lux > 100){
			light_log_threshold = LIGHT_LOG_THRESHOLD;
		}else{
			light_log_threshold = LIGHT_LOG_LOW_LUX_THRESHOLD;
		}
		
		/* Set the interface log threshold (1st priority) */
		if(g_als_data->g_als_log_threshold >= 0)
			light_log_threshold = g_als_data->g_als_log_threshold;

		/* report lux before dynamic ctl change */
		lux = lux * g_als_data->dynamic_sensitive;

		/* ASUS BSP Clay: shift lux to mitigate psensor noise when psensor on and lux < offset  +++ */
		lux = light_lux_check_psensor_noise(lux);
		/* ASUS BSP Clay: shift lux to mitigate psensor noise when psensor on and lux < offset--- */

		if(g_als_data->g_als_log_first_event){
			/* report directly */
			log("[ISR] Light Sensor First Report lux : %d (adc = %d), IT_id=%d\n", lux, adc, g_als_data->dynamic_IT);
			lsensor_report_lux(lux);
			g_als_data->event_counter++;	/* --- For stress test debug --- */
			g_als_last_lux = lux;
			/* dynamic control check after first report */
			g_als_data->g_als_log_first_event = false;
			ALS_dynamic_ctl_check(lux);
			psensor_onoff_recovery(true);
		}else{
			if(ALS_check_event_rest_time(lux)==1){
				if(abs(g_als_last_lux - lux) > light_log_threshold){
					log("[ISR] Light Sensor Report lux : %d (adc = %d), last_lux=%d\n", lux, adc, g_als_last_lux);
				}else if(0 == lux && g_als_last_lux != 0){
					log("[ISR] Light Sensor Report lux zero : %d (adc = %d), last_lux=%d\n", lux, adc, g_als_last_lux);
				}
				if((g_als_last_lux <= 1000 && lux > 1000) ||
					(g_als_last_lux > 1000 && lux <= 1000)){
					ALS_dynamic_ctl_check(lux);
				}
				g_als_last_lux = lux;
			}
		}
		cancel_delayed_work(&light_polling_lux_work);
		queue_delayed_work(ALSPS_delay_workqueue, &light_polling_lux_work, msecs_to_jiffies(LIGHT_POLLING_TIME));
	}
}

static int ALSPS_IST_LOG_COUNT=0;
static int ALSPS_IST_LOG_LIMIT=1000;
static void ALSPS_ist(struct work_struct *work)
{
	int alsps_int_ps, alsps_int_als;
	
mutex_lock(&g_alsps_lock);
	if(g_als_data->Device_switch_on == false && g_ps_data->Device_switch_on == false) {
		if(ALSPS_IST_LOG_COUNT==0){
			err("ALSPS are disabled and ignore IST.\n");
			ALSPS_IST_LOG_COUNT++;
		}else{
			ALSPS_IST_LOG_COUNT++;
			if(ALSPS_IST_LOG_COUNT==ALSPS_IST_LOG_LIMIT){
				ALSPS_IST_LOG_COUNT = 0;
			}
		}
		goto ist_err;
	}
	dbg("ALSPS ist +++ \n");
	if(g_ALSPS_hw_client == NULL){
		dbg("g_ALSPS_hw_client is NULL \n");
		goto ist_err;
	}

	/* Wait the i2c driver resume finish */
	if(g_alsps_power_status == ALSPS_SUSPEND){
		mdelay(WAIT_I2C_DELAY);
		dbg("Wait i2c driver ready(%d ms)", WAIT_I2C_DELAY);
	}

	/* Read INT_FLAG will clean the interrupt */
	ALSPS_SENSOR_INT = g_ALSPS_hw_client->ALSPS_hw_get_interrupt();
	if(ALSPS_SENSOR_INT <0){
//		err("ALSPS_hw_get_interrupt ERROR\n");
		goto ist_err;
	}

	/* Check Proximity Interrupt */
	alsps_int_ps = ALSPS_SENSOR_INT&ALSPS_INT_PS_MASK;
	if(alsps_int_ps == ALSPS_INT_PS_CLOSE || alsps_int_ps == ALSPS_INT_PS_AWAY) 
	{
		dbg("Proximity ist \n");
		if(g_ps_data->HAL_switch_on == true)
			g_ps_data->int_counter++;	/* --- For stress test debug --- */
		
		if (alsps_int_ps == ALSPS_INT_PS_AWAY) {
			proximity_work(ALSPS_INT_PS_AWAY);
		}
		if (alsps_int_ps == ALSPS_INT_PS_CLOSE) {
			proximity_work(ALSPS_INT_PS_CLOSE);
		}
	}

	/* Check Light Sensor Interrupt */
	alsps_int_als = ALSPS_SENSOR_INT&ALSPS_INT_ALS_MASK;
	if (alsps_int_als == ALSPS_INT_ALS) {
		dbg("Light Sensor ist \n");
		if(g_als_data->HAL_switch_on == true)
			g_als_data->int_counter++;	/* --- For stress test debug --- */

		light_work();
	}
	dbg("ALSPS ist --- \n");
ist_err:	
	__pm_relax(g_alsps_wake_lock);
	dbg("[IRQ] Enable irq !! \n");
	enable_irq(ALSPS_SENSOR_IRQ);
mutex_unlock(&g_alsps_lock);

}

static void ALSPS_irq_handler(void)
{
	dbg("[IRQ] Disable irq !! \n");
	disable_irq_nosync(ALSPS_SENSOR_IRQ);
	
	if(g_ALSPS_hw_client->ALSPS_hw_get_interrupt == NULL) {
		err("ALSPS_hw_get_interrupt NOT SUPPORT. \n");
		goto irq_err;
	}

	/*Queue work will enbale IRQ and unlock wake_lock*/
	queue_work(ALSPS_workqueue, &ALSPS_ist_work);
	__pm_stay_awake(g_alsps_wake_lock);
	return;
irq_err:
	dbg("[IRQ] Enable irq !! \n");
	enable_irq(ALSPS_SENSOR_IRQ);
}

static ALSPSsensor_GPIO mALSPSsensor_GPIO = {
	.ALSPSsensor_isr = ALSPS_irq_handler,
};

/*============================
 *|| For ALSPS check onoff ||
 *============================
 */
static int check_ALSP_onoff(void){
	if(g_ps_data->Device_switch_on || g_ps_data->HAL_switch_on ||
		g_als_data->Device_switch_on || g_als_data->HAL_switch_on){
		return 0;
	}else
		return 1;
}


/*=====================================
 *|| For ALSPS check event rest time ||
 *=====================================
 */
static int ALS_check_event_rest_time(int lux){
	struct timespec ts;
	u64 l_current_time_ns;
	//&ts = ktime_to_timespec(ktime_get_boottime());
	getnstimeofday(&ts);
	l_current_time_ns = timespec_to_ns(&ts);
	if(g_als_data->evt_skip_time_ns != 0){
		if(l_current_time_ns > g_als_data->evt_skip_time_ns){
			log("Light sensor report Lux=%d,  current_t %llx, rest_t %llx", lux, l_current_time_ns, g_als_data->evt_skip_time_ns);
			g_als_data->evt_skip_time_ns = 0;
			g_als_data->ts.tv_nsec = 0;
			lsensor_report_lux(lux);
			g_als_data->event_counter++;	/* --- For stress test debug --- */
			return 1;
		}else{
			dbg("skip_report, current %llx, start %llx ", l_current_time_ns, g_als_data->evt_skip_time_ns);
			return 0;
		}
	}else{
		lsensor_report_lux(lux);
		g_als_data->event_counter++;	/* --- For stress test debug --- */
		return 1;
	}
}

/*===================================
 *|| For ALS dynamic control check ||
 *===================================
 */
static int ALS_dynamic_ctl_check(int lux){
	if(g_ALSPS_hw_client->mlsensor_hw->light_hw_dynamic_check(lux)==1){
		g_als_data->dynamic_sensitive = g_ALSPS_hw_client->mlsensor_hw->light_hw_get_current_sensitive();
		g_als_data->dynamic_IT = g_ALSPS_hw_client->mlsensor_hw->light_hw_get_current_IT();

		/* stanby bit off*/
		g_ALSPS_hw_client->mlsensor_hw->light_hw_turn_onoff(false);
			
		/*Disable INT*/
		g_ALSPS_hw_client->mlsensor_hw->light_hw_interrupt_onoff(false);

		/* stanby bit on*/
		g_ALSPS_hw_client->mlsensor_hw->light_hw_turn_onoff(true);
			
		/*Enable INT*/
		g_ALSPS_hw_client->mlsensor_hw->light_hw_interrupt_onoff(true);
		
		g_ALSPS_hw_client->mlsensor_hw->light_hw_set_integration(g_als_data->dynamic_IT);
		
		//assign rest time enable
		//&g_als_data->ts = ktime_to_timespec(ktime_get_boottime());//
		getnstimeofday(&g_als_data->ts); 
		g_als_data->evt_skip_time_ns = timespec_to_ns(&g_als_data->ts) + g_ALSPS_hw_client->mlsensor_hw->light_hw_get_evt_skip_time_ns();
		return 1;
	}else
		return 0;
}

/*============================
 *|| For Proximity check status ||
 *============================
 */
bool proximityStatus(void)
{
	int adc_value = 0;
	bool status = false;
	int ret=0;
	int threshold_high = 0;

	__pm_stay_awake(g_alsps_wake_lock);
	/* check probe status */
	if(g_ALSPS_hw_client == NULL)
		goto ERROR_HANDLE;

	mutex_lock(&g_alsps_lock);

	/*Set Proximity Threshold(reset to factory)*/
	if(g_ps_data->Device_switch_on == false){
#if 0
		ret = proximity_set_threshold();
		if (ret < 0) {	
			err("proximity_set_threshold ERROR\n");
			goto ERROR_HANDLE;
		}
#endif
		ret = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_turn_onoff(true);
		if(ret < 0){
			err("proximity_hw_turn_onoff(true) ERROR\n");
			goto ERROR_HANDLE;
		}
	}

	msleep(PROXIMITY_TURNON_DELAY_TIME);

	adc_value = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_get_adc();

	//ASUS BSP Clay +++: use previous autok crosstalk
	if(g_ps_data->crosstalk_diff > g_ps_data->g_ps_autok_max){
		threshold_high = g_ps_data->g_ps_factory_cal_hi + g_ps_data->g_ps_autok_max;
	}else {
		threshold_high = g_ps_data->g_ps_factory_cal_hi + g_ps_data->crosstalk_diff;
	}
	//ASUS BSP Clay ---

	if (adc_value >= threshold_high) {
		status = true;
	}else{ 
		status = false;
	}
	log("proximityStatus : %s , (adc, hi_cal+prev_autoK)=(%d, %d)\n", 
		status?"Close":"Away", adc_value, threshold_high);

	if(g_ps_data->Device_switch_on == false){
		ret = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_turn_onoff(false);
		if(ret < 0){
			err("proximity_hw_turn_onoff(false) ERROR\n");
		}
	}
	
ERROR_HANDLE:
	
	mutex_unlock(&g_alsps_lock);
	__pm_relax(g_alsps_wake_lock);
	return status;
}
EXPORT_SYMBOL(proximityStatus);

/*===========================
 *|| Proximity Auto Calibration Part ||
 *============================
 */
static int proximity_check_minCT(void)
{
	int adc_value = 0;
	int crosstalk_diff;
	int crosstalk_min = 9999;
	int round, count = 0, delay = 0, crosstalk_limit = 0;

	/*check the crosstalk calibration value*/
	/*
	ret = psensor_factory_read_inf(PSENSOR_INF_CALIBRATION_FILE);
	if(ret >= 0) {
		g_ps_data->g_ps_calvalue_inf= ret;
		log("Proximity read INF Calibration : %d\n", g_ps_data->g_ps_calvalue_inf);
	}else{
		err("Proximity read DEFAULT INF Calibration : %d\n", g_ps_data->g_ps_calvalue_inf);
	}
	*/

	log("Proximity INF Calibration : %d\n", g_ps_data->g_ps_calvalue_inf);

	/* ASUS BSP+++ Clay: dynamic proxmity period due to psensor noise effect */
	if(g_ps_data->cur_period == PROXIMITY_NOISE_PERIOD &&
			PROXIMITY_NOISE_PERIOD != PROXIMITY_PERIOD){
		count = PROXIMITY_NOISE_AUTOK_COUNT;
		delay = PROXIMITY_NOISE_AUTOK_DELAY;
	}else{
		count = PROXIMITY_AUTOK_COUNT;
		delay = PROXIMITY_AUTOK_DELAY;
	}
	log("count=%d, delay=%d, period=%d\n", count, delay, g_ps_data->cur_period);
	/* ASUS BSP--- */
	
	/*update the min crosstalk value*/
	for(round=0; round<count; round++){	
		mdelay(delay);
		adc_value = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_get_adc();
		log("proximity auto calibration adc : %d\n", adc_value);
		if(adc_value < crosstalk_min ){
			crosstalk_min = adc_value;
			log("Update the min for crosstalk : %d\n", crosstalk_min);
		}
	}

	/*update the diff crosstalk value*/
	crosstalk_diff = crosstalk_min -g_ps_data->g_ps_calvalue_inf;
	if(crosstalk_diff>g_ps_data->g_ps_autok_min && crosstalk_diff<g_ps_data->g_ps_autok_max){
		log("Update the diff for crosstalk : %d\n", crosstalk_diff);
		
		//ASUS BSP Clay +++: prevent near > (pocket-500) after autok
		crosstalk_limit = g_pocket_mode_threshold - 500 - g_ps_data->g_ps_factory_cal_hi;
		if(crosstalk_diff > crosstalk_limit){
			log("crosstalk_diff(%d) > pocket-500-cal_hi(%d)", crosstalk_diff, crosstalk_limit);
			crosstalk_diff = crosstalk_limit;
		}
		//ASUS BSP Clay ---

		
		g_ps_data->crosstalk_diff = crosstalk_diff;

		if(g_ALSPS_hw_client->mpsensor_hw->proximity_hw_set_autoK == NULL) {
			err("proximity_hw_set_autoK NOT SUPPORT. \n");
			return -1;
		}
		g_ALSPS_hw_client->mpsensor_hw->proximity_hw_set_autoK(crosstalk_diff);
		g_ps_data->g_ps_calvalue_hi += crosstalk_diff;
		g_ps_data->g_ps_calvalue_lo += crosstalk_diff;
		log("Update the diff for crosstalk : %d, hi: %d, low: %d, START\n", 
			g_ps_data->crosstalk_diff, g_ps_data->g_ps_calvalue_hi, g_ps_data->g_ps_calvalue_lo);
	}else if(crosstalk_diff>=g_ps_data->g_ps_autok_max){
		log("crosstalk diff(%d) >= proximity autok max(%d)\n", crosstalk_diff, g_ps_data->g_ps_autok_max);
		g_ps_data->crosstalk_diff = crosstalk_diff;
	}else{
		log("crosstalk diff(%d) <= proximity autok min(%d)\n", crosstalk_diff, g_ps_data->g_ps_autok_min);
		g_ps_data->crosstalk_diff = 0;
	}
	
	return 0;
}

static void proximity_autok(struct work_struct *work)
{
	int adc_value;
	int crosstalk_diff, crosstalk_limit;

	/* proximity sensor go to suspend, cancel autok timer */
	if(g_alsps_power_status == ALSPS_SUSPEND){
		hrtimer_cancel(&g_alsps_timer);
		log("proximity has suspended, cancel proximity autoK polling!!");
		return;
	}

	if(g_ALSPS_hw_client->mpsensor_hw->proximity_hw_set_autoK == NULL) {
		err("proximity_hw_set_autoK NOT SUPPORT. \n");
		return;
	}

	adc_value = g_ALSPS_hw_client->mpsensor_hw->proximity_hw_get_adc();
	if(adc_value < 0){
		return;
	} else {
		dbg("auto calibration polling : %d\n", adc_value);
	}

	crosstalk_diff = adc_value - g_ps_data->g_ps_calvalue_inf;

	if((crosstalk_diff<g_ps_data->crosstalk_diff) &&( g_ps_data->crosstalk_diff!=0)){
		/*last diff of crosstalk does not set to HW, should reset the value to 0.*/
		if(g_ps_data->crosstalk_diff >= g_ps_data->g_ps_autok_max){
			g_ps_data->crosstalk_diff = 0;
		}

		if(crosstalk_diff<=g_ps_data->g_ps_autok_min){
			g_ALSPS_hw_client->mpsensor_hw->proximity_hw_set_autoK(0-g_ps_data->crosstalk_diff);
			g_ps_data->g_ps_calvalue_hi += (0-g_ps_data->crosstalk_diff);
			g_ps_data->g_ps_calvalue_lo += (0-g_ps_data->crosstalk_diff);
			g_ps_data->crosstalk_diff = 0;
			log("Update the diff for crosstalk : %d, hi: %d, low: %d, END\n", 
				g_ps_data->crosstalk_diff, g_ps_data->g_ps_calvalue_hi, g_ps_data->g_ps_calvalue_lo);
		}else if((crosstalk_diff>g_ps_data->g_ps_autok_min) && (crosstalk_diff<g_ps_data->g_ps_autok_max)){

			//ASUS BSP Clay +++: prevent near > (pocket-500) after autok
			crosstalk_limit = g_pocket_mode_threshold - 500 - g_ps_data->g_ps_factory_cal_hi;
			if(crosstalk_diff > crosstalk_limit){
				log("crosstalk_diff(%d) > pocket-500-cal_hi(%d)", crosstalk_diff, crosstalk_limit);
				crosstalk_diff = crosstalk_limit;
			}
			//ASUS BSP Clay ---

			g_ALSPS_hw_client->mpsensor_hw->proximity_hw_set_autoK(crosstalk_diff-g_ps_data->crosstalk_diff);
			g_ps_data->g_ps_calvalue_hi += (crosstalk_diff-g_ps_data->crosstalk_diff);
			g_ps_data->g_ps_calvalue_lo += (crosstalk_diff-g_ps_data->crosstalk_diff);
			g_ps_data->crosstalk_diff = crosstalk_diff;
			log("Update the diff for crosstalk : %d, hi: %d, low: %d\n", 
				g_ps_data->crosstalk_diff, g_ps_data->g_ps_calvalue_hi, g_ps_data->g_ps_calvalue_lo);
		}else{
			log("over the autok_max : (adc, inf) = %d(%d, %d) > %d\n", 
				crosstalk_diff, adc_value, g_ps_data->g_ps_calvalue_inf, g_ps_data->g_ps_autok_max);
			g_ps_data->crosstalk_diff = crosstalk_diff;
		}
	}
}

static enum hrtimer_restart proximity_timer_function(struct hrtimer *timer)
{
	ktime_t autok_delay;
	
	dbg("proximity_timer_function\n");
	queue_work(ALSPS_workqueue, &proximity_autok_work);

	if(0 == g_ps_data->crosstalk_diff){
		return HRTIMER_NORESTART;
	}else{
		/*needs to be reset in the callback function*/
		autok_delay = ns_to_ktime(PROXIMITY_AUTOK_POLLING * NSEC_PER_MSEC);
		hrtimer_forward_now(&g_alsps_timer, autok_delay);
	}
	return HRTIMER_RESTART;
}

/*====================
 *|| I2C mutex lock ||
 *====================
 */
#if 0
void lock_i2c_bus6(void) {
	mutex_lock(&g_i2c_lock);
}
EXPORT_SYMBOL(lock_i2c_bus6);

void unlock_i2c_bus6(void) {
	mutex_unlock(&g_i2c_lock);
}
EXPORT_SYMBOL(unlock_i2c_bus6);
#endif

/*====================
 *|| IO Control Part ||
 *====================
 */
#if ENABLE_PROXIMITY_IOCTL_LIB
static int proxSensor_miscOpen(struct inode *inode, struct file *file)
{
	int ret = 0;
	if(prox_open_count == 0){
		ret = mproximity_store_switch_onoff(true);
		dbg("%s: %d\n", __func__, ret);
		if (ret < 0) {
			err("proximity_hw_turn_onoff(true) ERROR\n");
		} else {
			prox_open_count++;
		}
	} else if(prox_open_count > 0) {
		prox_open_count++;
		log("proximity sensor has been opened(count=%d)\n", prox_open_count);
	}
	return ret;
}

static int proxSensor_miscRelease(struct inode *inode, struct file *file)
{
	int ret = 0;
	prox_open_count--;
	if(prox_open_count == 0){
		ret = mproximity_store_switch_onoff(false);
		dbg("%s: %d\n", __func__, ret);
		if (ret < 0) {
			err("proximity_hw_turn_onoff(false) ERROR\n");
			prox_open_count++;
		}
	} else if(prox_open_count < 0){
		prox_open_count = 0;
		log("proximity sensor has been closed, do nothing(count=%d)\n", prox_open_count);
	}
	return ret;
}

static struct file_operations prox_fops = {
	.owner = THIS_MODULE,
	.open = proxSensor_miscOpen,
	.release = proxSensor_miscRelease
};

static struct miscdevice prox_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "asusProxSensor",
	.fops = &prox_fops
};

static int proxSensor_miscRegister(void)
{
	int rtn = 0;
	rtn = misc_register(&prox_misc);
	if (rtn < 0) {
		log("[%s] Unable to register prox misc deive\n", __func__);
		misc_deregister(&prox_misc);
	}
	return rtn;
}
#endif

#if ENABLE_LIGHT_IOCTL_LIB
static int lightSensor_miscOpen(struct inode *inode, struct file *file)
{
	int ret = 0;
	if(light_open_count == 0){
		ret = mlight_store_switch_onoff(true);
		dbg("%s: %d\n", __func__, ret);
		if (ret < 0) {
			err("light_hw_turn_onoff(true) ERROR\n");
		} else {
			light_open_count++;
		}
	} else if(light_open_count > 0) {
		light_open_count++;
		log("light sensor has been opened(count=%d)\n", light_open_count);
	}
	return ret;
}

static int lightSensor_miscRelease(struct inode *inode, struct file *file)
{
	int ret = 0;
	light_open_count--;
	if(light_open_count == 0){
		ret = mlight_store_switch_onoff(false);
		dbg("%s: %d\n", __func__, ret);
		if (ret < 0) {
			err("light_hw_turn_onoff(false) ERROR\n");
			light_open_count++;
		}
	} else if(light_open_count < 0){
		light_open_count = 0;
		log("light sensor has been closed, do nothing(count=%d)\n", light_open_count);
	}
	return ret;
}

static struct file_operations light_fops = {
	.owner = THIS_MODULE,
	.open = lightSensor_miscOpen,
	.release = lightSensor_miscRelease
};

static struct miscdevice light_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "asusLightSensor",
	.fops = &light_fops
};

static int lightSensor_miscRegister(void)
{
	int rtn = 0;
	rtn = misc_register(&light_misc);
	if (rtn < 0) {
		log("[%s] Unable to register light misc deive\n", __func__);
		misc_deregister(&light_misc);
	}
	return rtn;
}
#endif

/*====================
 *|| Initialization Part ||
 *====================
 */
static int init_data(void)
{
	int ret = 0;
	/* Reset ASUS_proximity_sensor_data */
	g_ps_data = kmalloc(sizeof(struct psensor_data), GFP_KERNEL);
	if (!g_ps_data) {
		err("g_ps_data kmalloc ERROR\n");
		ret = -ENOMEM;
		goto init_data_err;
	}
	memset(g_ps_data, 0, sizeof(struct psensor_data));
	g_ps_data->Device_switch_on = false;
	g_ps_data->HAL_switch_on =    false;	
	g_ps_data->polling_mode =     true;
	g_ps_data->autok =            true;
	
	g_ps_data->g_ps_calvalue_hi = g_ALSPS_hw_client->mpsensor_hw->proximity_hi_threshold_default;
	g_ps_data->g_ps_calvalue_lo = g_ALSPS_hw_client->mpsensor_hw->proximity_low_threshold_default;	
	g_ps_data->g_ps_calvalue_inf = g_ALSPS_hw_client->mpsensor_hw->proximity_crosstalk_default;	
	g_ps_data->g_ps_factory_cal_hi = g_ALSPS_hw_client->mpsensor_hw->proximity_hi_threshold_default;
	g_ps_data->g_ps_factory_cal_lo = g_ALSPS_hw_client->mpsensor_hw->proximity_low_threshold_default;
	g_pocket_mode_threshold = PROXIMITY_POCKET_DEFAULT;

	g_ps_data->g_ps_autok_min= g_ALSPS_hw_client->mpsensor_hw->proximity_autok_min;	
	g_ps_data->g_ps_autok_max = g_ALSPS_hw_client->mpsensor_hw->proximity_autok_max;	

	g_ps_data->int_counter = 0;
	g_ps_data->event_counter = 0;
	g_ps_data->crosstalk_diff = 0;
#ifdef PSENSOR_CAL24
	g_ps_data->selection = 0;
#else
	g_ps_data->selection = 1;
#endif
	g_ps_data->cur_period = PROXIMITY_PERIOD;

	/* Reset ASUS_light_sensor_data */
	g_als_data = kmalloc(sizeof(struct lsensor_data), GFP_KERNEL);
	if (!g_als_data){
		err("g_als_data kmalloc ERROR\n");
		ret = -ENOMEM;
		goto init_data_err;
	}
	memset(g_als_data, 0, sizeof(struct lsensor_data));
	g_als_data->Device_switch_on = false;
	g_als_data->HAL_switch_on = 	false;	
	g_als_data->g_als_calvalue = g_ALSPS_hw_client->mlsensor_hw->light_calibration_default;

	g_als_data->g_als_accuracy_gain = ALSPS_DEFAULT_VALUE;
	g_als_data->g_als_change_sensitivity = ALSPS_DEFAULT_VALUE;
	g_als_data->g_als_log_threshold = ALSPS_DEFAULT_VALUE;
	g_als_data->g_als_log_first_event = true;

	g_als_data->int_counter = 0;
	g_als_data->event_counter = 0;
	g_als_data->dynamic_sensitive = g_ALSPS_hw_client->mlsensor_hw->light_hw_get_current_sensitive();
	g_als_data->dynamic_IT = g_ALSPS_hw_client->mlsensor_hw->light_hw_get_current_IT();
	g_als_data->selection = 0;
	g_als_data->ts.tv_sec = 0;
	g_als_data->ts.tv_nsec = 0;
	g_als_data->evt_skip_time_ns = 0;
	/* ASUS BSP Clay: shift lux to mitigate psensor noise when psensor on and lux < offset +++ */
	g_als_data->offset_adc = LIGHT_LOW_LUX_NOISE_OFFSET;
	g_als_data->offset_lux = 0;
	/* ASUS BSP Clay: shift lux to mitigate psensor noise when psensor on and lux < offset--- */
	g_als_data->freeze_psensor = false;

	/* ASUS BSP Clay: average 5 lux for offset behavior to mitigate the low lux gap +++ */
	light_sensor_reset_status();
	/* ASUS BSP Clay: average 5 lux for offset behavior to mitigate the low lux gap --- */

	/* Reset ASUS P/L sensor power status */
	g_alsps_power_status = ALSPS_RESUME;

	/* Reset ASUS sensor polling cancel flag */
	g_psensor_polling_cancel_flag = false;

	return 0;
init_data_err:
	err("Init Data ERROR\n");
	return ret;
}
 
static void mALSPS_algo_probe(struct i2c_client *client)
{
	int ret;
	log("Driver PROBE +++\n");

	/*check i2c client*/
	if (client == NULL) {
		err("i2c Client is NUll\n");
		goto probe_err;
	}

	/*link driver data to i2c client*/
	strlcpy(client->name, SENSOR_TYPE_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, g_als_data);
	i2c_set_clientdata(client, g_ps_data);	

	/* i2c client */
	g_i2c_client = client;
	if (ALSPS_SENSOR_IRQ < 0)
		goto probe_err;

	/* I2c stress test */
#ifdef CONFIG_I2C_STRESS_TEST
	i2c_add_test_case(client, "ALSPS Test", ARRAY_AND_SIZE(ALSPS_TestCaseInfo));
#endif

	g_ALSPS_hw_client = ALSPS_hw_getHardware();
	if(g_ALSPS_hw_client == NULL){
		goto probe_err;
	}

	/* driver data structure initialize */
	ret = init_data();
	if (ret < 0)
		goto probe_err;

	/* string copy the character of vendor and module number */
	strcpy(mpsensor_ATTR.info_type->vendor, g_ALSPS_hw_client->vendor);
	strcpy(mpsensor_ATTR.info_type->module_number, g_ALSPS_hw_client->module_number);
	strcpy(mlsensor_ATTR.info_type->vendor, g_ALSPS_hw_client->vendor);
	strcpy(mlsensor_ATTR.info_type->module_number, g_ALSPS_hw_client->module_number);

	/* Attribute */
	ret = psensor_ATTR_register(&mpsensor_ATTR);
	if (ret < 0)
		goto probe_err;
	ret = lsensor_ATTR_register(&mlsensor_ATTR);
	if (ret < 0)
		goto probe_err;

	/* Input Device */
#if ENABLE_PROXIMITY_IOCTL_LIB
	ret = proxSensor_miscRegister();
	if (ret < 0)
		goto probe_err;
#endif
	ret = psensor_report_register();
	if (ret < 0)
		goto probe_err;
#if ENABLE_LIGHT_IOCTL_LIB
	ret = lightSensor_miscRegister();
	if (ret < 0)
		goto probe_err;
#endif
	ret = lsensor_report_register();
	if (ret < 0)
		goto probe_err;

	ALSPS_SENSOR_IRQ = ALSPSsensor_gpio_register(g_i2c_client, &mALSPSsensor_GPIO);
	if (ALSPS_SENSOR_IRQ < 0)
		goto probe_err;

	/*To avoid LUX can NOT report when reboot in LUX=0*/
	log("[INIT] Light sensor report -1)\n");
	lsensor_report_lux(-1);
	log("[INIT] Proximity Report Away\n");
	g_ps_data->g_ps_int_status = ALSPS_INT_PS_INIT;
	psensor_report_abs(-1);

	log("Driver PROBE ---\n");
	g_alsps_probe_status = true;
	return ;
probe_err:
	g_alsps_probe_status = false;
	err("Driver PROBE ERROR ---\n");
	return;

}

static void mALSPS_algo_remove(void)
{
	log("Driver REMOVE +++\n");

	ALSPSsensor_gpio_unregister(ALSPS_SENSOR_IRQ);

	log("Driver REMOVE ---\n");

	return;
}

static void mALSPS_algo_shutdown(void)
{
	log("Driver SHUTDOWN +++\n");
	if(g_als_data != NULL){
		/* Disable sensor */
		if (g_als_data->Device_switch_on)
			light_turn_onoff(false);
		if (g_ps_data->Device_switch_on)
			proximity_turn_onoff(false);

		log("Driver SHUTDOWN ---\n");
	}
	return;
}

static void mALSPS_algo_suspend(void)
{
	if(false == g_alsps_probe_status){
		return;
	}
	
	log("Driver SUSPEND +++\n");

mutex_lock(&g_alsps_lock);

	g_alsps_power_status = ALSPS_SUSPEND;

	/* For keep Proximity can wake_up system */
	if (g_ps_data->Device_switch_on)
		enable_irq_wake(ALSPS_SENSOR_IRQ);

	/* For make sure Light sensor mush be switch off when system suspend */
	if (g_als_data->Device_switch_on){
		light_suspend_turn_off(false);
		cancel_delayed_work(&light_polling_lux_work);
	}

	cancel_delayed_work(&proximity_polling_adc_work);
	g_psensor_polling_cancel_flag = true;

	log("Driver SUSPEND ---\n");

mutex_unlock(&g_alsps_lock);

	return;
}

static void mALSPS_algo_resume(void)
{
	ktime_t autok_delay;

	if(false == g_alsps_probe_status){
		return;
	}

	log("Driver RESUME +++\n");

	g_alsps_power_status = ALSPS_RESUME;

	if (g_als_data->Device_switch_on == 0 && g_als_data->HAL_switch_on == 1){
		resume_flag=1;
		light_turn_onoff(true);
		/* ASUS BSP Clay: average for offset behavior to mitigate the low lux gap +++ */
		light_sensor_reset_status();
		/* ASUS BSP Clay: average for offset behavior to mitigate the low lux gap ---*/
		g_als_data->g_als_retry_count = 0;
		queue_delayed_work(ALSPS_delay_workqueue, &light_polling_lux_work, msecs_to_jiffies(LIGHT_TURNON_DELAY_TIME));
	}
	
	/* If the proximity sensor has been opened and autok has been enabled, 
	 * start autoK work queue. */
	if (g_ps_data->Device_switch_on) {
		if(true == g_ps_data->autok && g_ps_data->crosstalk_diff != 0){
			autok_delay = ns_to_ktime(PROXIMITY_AUTOK_POLLING * NSEC_PER_MSEC);
			hrtimer_start(&g_alsps_timer, autok_delay, HRTIMER_MODE_REL);
		}
		/* If the proximity sensor has been opened and proximity polling thread has been canceled, 
		 * restart it. */
		if(g_psensor_polling_cancel_flag){
			queue_delayed_work(ALSPS_delay_workqueue, &proximity_polling_adc_work, msecs_to_jiffies(100));
			g_psensor_polling_cancel_flag = false;
		}
	}
	
	log("Driver RESUME ---\n");
	
	return;
}

static ALSPS_I2C mALSPS_I2C = {
	.ALSPS_probe = mALSPS_algo_probe,
	.ALSPS_remove = mALSPS_algo_remove,
	.ALSPS_shutdown = mALSPS_algo_shutdown,
	.ALSPS_suspend = mALSPS_algo_suspend,
	.ALSPS_resume = mALSPS_algo_resume,
};

extern int ALSPS_i2c_add_driver(void);
static int __init ALSPS_init(void)
{
	int ret = 0;
	log("Driver INIT +++\n");

	/*Record the error message*/
	g_error_mesg = kzalloc(sizeof(char [ERROR_MESG_SIZE]), GFP_KERNEL);
	
	/* Work Queue */
	ALSPS_workqueue = create_singlethread_workqueue(SENSOR_TYPE_NAME"_wq");
	ALSPS_delay_workqueue = create_singlethread_workqueue(SENSOR_TYPE_NAME"_delay_wq");

	/* Initialize the Mutex */
	mutex_init(&g_alsps_lock);
	mutex_init(&g_i2c_lock);

	/* Initialize the wake lock */
	//wake_lock_init(&g_alsps_wake_lock, &g_i2c_client->dev, "ALSPS_wake_lock");
	g_alsps_wake_lock = wakeup_source_register(NULL, "ALSPS_wake_lock");

	/*Initialize high resolution timer*/
	hrtimer_init(&g_alsps_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_alsps_timer.function = proximity_timer_function;
	
	/* i2c Registration for probe/suspend/resume */
	ret = ALSPS_i2c_register(&mALSPS_I2C);
	if (ret < 0)
		goto init_err;
	
	/* Hardware Register Initialization */
	ret = ALSPS_i2c_add_driver();
	if (ret < 0)
		goto init_err;
	
	log("Driver INIT ---\n");
	return 0;

init_err:
	err("Driver INIT ERROR ---\n");
	return ret;
}

static void __exit ALSPS_exit(void)
{
	log("Driver EXIT +++\n");
	/* i2c Unregistration */	
	ALSPS_i2c_unregister();
	
	/*Report Unregistration*/
	if(g_alsps_probe_status){
		psensor_report_unregister();
		lsensor_report_unregister();
	}
	
	/*ATTR Unregistration*/
	psensor_ATTR_unregister();
	lsensor_ATTR_unregister();
	
	wakeup_source_unregister(g_alsps_wake_lock);
	mutex_destroy(&g_alsps_lock);
	mutex_destroy(&g_i2c_lock);
	if(g_ps_data!=NULL){
		kfree(g_ps_data);
	}
	if(g_als_data!=NULL){
		kfree(g_als_data);
	}

	destroy_workqueue(ALSPS_workqueue);
	destroy_workqueue(ALSPS_delay_workqueue);
	
	log("Driver EXIT ---\n");
}

module_init(ALSPS_init);
module_exit(ALSPS_exit);

MODULE_AUTHOR("Clay_Wang <Clay_Wang@asus.com>");
MODULE_DESCRIPTION("Proximity and Ambient Light Sensor");
MODULE_LICENSE("GPL");

