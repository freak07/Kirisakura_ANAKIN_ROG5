#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#define CONFIG_I2C_MODULE
#include <linux/i2c.h>
#include "device.h"
#include "memory.h"
#include "serial_bus.h"
#include "main.h"
#include "event.h"
#include "hardware.h"
#include "sysfs.h"
#include "utils.h"
#include "config.h"
#include "debug.h"
#include "sonacomm.h"
#include "workqueue.h"
#include "irq.h"

#include <linux/proc_fs.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include "locking.h"
#include "customize.h"

#include <linux/timer.h>

static int SntSensor_miscOpen(struct inode *inode, struct file *file);
static int SntSensor_miscRelease(struct inode *inode, struct file *file);
static long SntSensor_miscIoctl(struct file *file, unsigned int cmd, unsigned long arg);

void grip_enable_func_noLock(int val);

void Wait_Wake_For_RegW(void);
void Into_DeepSleep_fun(void);
void grip_dump_status_func(struct work_struct *work);
void Reset_Func(struct work_struct *work);
static int Health_Check_Enable_No_Delay(int en);
//static void Enable_tap_sensitive(const char *buf, size_t count);
//Check grip all gesture status, 1: enable, 0:disable
static int grip_all_gesture_status(void);

//Check grip game gesture (tap/slide/swipe) status, 1: enable, 0:disable
int grip_game_gesture_status(void);
static int grip_tap_gesture_status(void);
static int grip_swipe_gesture_status(void);
static int grip_slide_gesture_status(void);

//vibrator en/disable func
extern int aw8697_trig_control(int num, bool enable);

//report event to sensor hal when chip reset occurs
extern void grip_input_event_report(int g_id, int len, int trk_id, int bar_id, int force, int fr_nr, int center);

/* ASUS BSP Clay: setting vibrator trig1 & trig2 register for different tap 
modes +++ */
extern int G_grip_tap_vib1_reg;
extern int G_grip_tap_vib2_reg;
static void gripVibratorSetting(int val, int tap_id);
/* ASUS BSP Clay--- */

#ifdef DYNAMIC_PWR_CTL
extern int snt_activity_request(void);
#endif


/* Workaround for stucked semaphore */
void check_stuck_semaphore(struct work_struct *work);
void Power_Control(int en);
extern void snt_set_pinctrl(struct device *dev, char *str);
struct delayed_work check_stuck_wake;
struct workqueue_struct *asus_wq;
extern struct delayed_work event_wq;

#ifdef ASUS_ZS673KS_PROJECT
/* Read config bank */
int cust_write_registers(void *dev, int reg, int num, void *value);
int cust_read_registers(void *dev, int reg, int num, void *value);
#endif

/* 
	param. fw_loading_status:
	0: when charger/recorver or the other mode, grip fw will load fail 
	1: load fw success
int fw_loading_status = 0;
*/

/* init 1V2_2V8 power status */
static int g_snt_power_state = 1;

static uint32_t ms_start=0, ms_end=0;

struct file_operations sentons_snt_fops = {
	.owner = THIS_MODULE,
	.open = SntSensor_miscOpen,
	.release = SntSensor_miscRelease,
	.unlocked_ioctl = SntSensor_miscIoctl,
	.compat_ioctl = SntSensor_miscIoctl
};
struct miscdevice sentons_snt_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sentons_grip_sensor",
	.fops = &sentons_snt_fops
};

/* init record helath check result */
uint16_t FPC_value = 0;

static uint16_t Tap_sense_data1[3] = { 
		0x0003,
		0x0000,
		0x8062 
};
static uint16_t Tap_sense_data2[3] = { 
		0x0014,
		0x0000,
		0x804b 
};

static uint16_t Tap_sense_data3[3] = { 
		0x0028,
		0x0000,
		0x8053 
};

static uint16_t Tap_sense_reset_data1[3] = { 
		0x0032,
		0x0000,
		0x804b 
};
static uint16_t Tap_sense_reset_data2[3] = { 
		0x003c,
		0x0000,
		0x8053 
};

static uint16_t Slide_Swipe_sense_data[3] = { 
		0x0028,
		0x0000,
		0x804b 
};

static uint16_t Slide_Swipe_sense_reset_data[3] = { 
		0x0032,
		0x0000,
		0x804b 
};

static uint16_t Swipe_sense_data2[3] = { 
		0x8000,
		0x0000,
		0x8057 
};

static uint16_t Swipe_sense_reset_data2[3] = { 
		0x0400,
		0x0000,
		0x8057 
};

static uint16_t sys_param_addr[3] = { 
		0x42, 
		0x43, 
		0x44 
};
extern int track_report_count;


/* after chip reset, recovery according to status which records from 
property */
struct delayed_work rst_recovery_wk;

/* reset chip when i2c error or chip can't be waked up */
struct delayed_work rst_gpio_wk;

static int Grip_Check_FW_Status(void){
	if(snt8100fsr_g->chip_reset_flag != GRIP_FW_DL_END){
		mutex_unlock(&snt8100fsr_g->ap_lock);
		snt8100fsr_g->Recovery_Flag = 1;
		return 1;
	}else{
		return 0;
	}
}

void Grip_Chip_IRQ_EN(bool flag){
	uint16_t irq_val = 0;
	if(flag){
		irq_val = 0x1;
		PRINT_INFO("Enable Chip iRQ, Reg 0x1 = 0x%x", irq_val);
		write_register(snt8100fsr_g,
				REGISTER_ENABLE,
				&irq_val);
	}else{
		irq_val = 0x0;
		PRINT_INFO("Disable Chip IRQ, Reg 0x1 = 0x%x", irq_val);
		write_register(snt8100fsr_g,
				REGISTER_ENABLE,
				&irq_val);
	}
}
void Grip_Driver_IRQ_EN(bool flag){
	static bool irq_en = false;
	if(irq_en != flag){
		irq_en = flag;
		if(irq_en == true){
			PRINT_INFO("Enable Driver IRQ, %d", snt8100fsr_g->hostirq_gpio);
			enable_irq(gpio_to_irq(snt8100fsr_g->hostirq_gpio));
		}else{
			PRINT_INFO("Disable Driver IRQ");
			disable_irq_nosync(gpio_to_irq(snt8100fsr_g->hostirq_gpio));
		}
	}
}

/* for squeeze/tap cancel missing when grip reset */
extern int grip_write_fail_count;
void Reset_Func(struct work_struct *work){
	PRINT_INFO("status: %d, failed_count=%d", 
	snt8100fsr_g->grip_fw_loading_status, snt8100fsr_g->fw_failed_count);
	if(snt8100fsr_g->grip_fw_loading_status == false && FW_FAIL_REDOWNLOAD_LIMIT <= snt8100fsr_g->fw_failed_count){
		PRINT_INFO("1. load continue or 2. fw load fail at init, chip broken case, don't retry");
		Power_Control(0);
		return;
	}
	if(snt8100fsr_g->chip_reset_flag == GRIP_RST_FW_DL){
		PRINT_INFO("Chip reset is ongoing, skip request");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	if(snt8100fsr_g->chip_reset_flag == GRIP_FW_DL_END){
		PRINT_INFO("Starting chip reset");
		ASUSEvtlog("[Grip] Workaround : reset chip\n");
		snt8100fsr_g->chip_reset_flag = GRIP_RST_FW_DL;
		snt8100fsr_g->grip_fw_loading_status = false; /* reset fw_loading status */
		snt8100fsr_g->fw_failed_count++;
		grip_write_fail_count = 0; /* reset i2c write failed count */
		snt8100fsr_g->snt_state = GRIP_WAKEUP;
		Grip_Driver_IRQ_EN(1);
		Power_Control(1);
	}
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void check_stuck_semaphore(struct work_struct *work){
	int ret;
	/* prevent chip reset fail */
	if(snt8100fsr_g->chip_reset_flag != GRIP_FW_DL_END){
		workqueue_cancel_work(&check_stuck_wake);
		PRINT_INFO("Don't wake chip during chip reset & long fw");
		snt8100fsr_g->stuck_retry_count = 0;
		return;
	}
	if(snt8100fsr_g->stuck_flag == true){
		PRINT_INFO("Used to solve wailting semaphore!!! retry times = %d", 
		snt8100fsr_g->stuck_retry_count);
		/* when retry occurs, check listened gpio status */
		PRINT_INFO("GPIO: %d=%d", IRQ_GPIO, gpio_get_value(IRQ_GPIO));
		MUTEX_LOCK(&snt8100fsr_g->sb_lock);
 		ret = sb_wake_device(snt8100fsr_g);
		mutex_unlock(&snt8100fsr_g->sb_lock);
		if (ret) {
			PRINT_CRIT("sb_wake_device() failed");
		}
		//retry check
		if(snt8100fsr_g->stuck_retry_count < snt8100fsr_g->stuck_retry_limit){
			workqueue_cancel_work(&check_stuck_wake);
			workqueue_queue_work(&check_stuck_wake, 200);
			snt8100fsr_g->stuck_retry_count++;
		}else{
			ASUSEvtlog("[Grip] driver is failed to wait semaphore due to non-wakeable chip\n"); 
			up(&snt8100fsr_g->wake_rsp);
			if (down_trylock(&snt8100fsr_g->wake_req)){
				PRINT_INFO("Wake Req alread consumed");
				if(down_trylock(&snt8100fsr_g->wake_rsp)){
					PRINT_INFO("Wake Rsq alread consumed");
				}
			}
			queue_delayed_work(asus_wq, &rst_gpio_wk, msecs_to_jiffies(0));
		}
	}else{
		PRINT_INFO("None used");
		snt8100fsr_g->stuck_retry_count = 0;
		workqueue_cancel_work(&check_stuck_wake);
	}
}
static int Health_Check_Enable_No_Delay(int en){
	uint16_t En_fpc;
	int ret;
	ret = read_register(snt8100fsr_g, 0x003d, &En_fpc);
	if(ret < 0) {
		PRINT_ERR("Read Reg 3d Failed");
		return -1;	
	}
	//PRINT_INFO("0x003d:%x ", En_fpc);
	if(en == 0){
		En_fpc = En_fpc | 0x0004;
	}else if(en ==1){
		En_fpc = En_fpc & 0xfffb;
	}else{
		PRINT_ERR("en=%d, out of 1 or 2 ", en);
		return -1;
	}
	
	ret = write_register(snt8100fsr_g, 0x003d, &En_fpc);
	if(ret < 0) {
		PRINT_ERR("Read Reg 3d Failed");
		return -1;
	}
	PRINT_INFO("Health Check EN=%d", en);
	return 0;
}


int Health_Check_Enable(int en){
	uint16_t En_fpc;
	int ret;
	ret = read_register(snt8100fsr_g, 0x003d, &En_fpc);
	if(ret < 0) {
		PRINT_ERR("Read Reg 3d Failed");
		return -1;	
	}
	//PRINT_INFO("0x003d:%x ", En_fpc);
	if(en == 0){
		En_fpc = En_fpc | 0x0004;
	}else if(en ==1){
		En_fpc = En_fpc & 0xfffb;
	}else{
		PRINT_ERR("en=%d, out of 1 or 2 ", en);
		return -1;
	}
	
	ret = write_register(snt8100fsr_g, 0x003d, &En_fpc);
	if(ret < 0) {
		PRINT_ERR("Read Reg 3d Failed");
		return -1;
	}
	msleep(100);
	ret = read_register(snt8100fsr_g, 0x003d, &En_fpc);
	if(ret < 0) {
		PRINT_ERR("Read Reg 3d Failed");
		return -1;
	}
	PRINT_INFO("Health Check EN=%d", en);
	return 0;
}
int Health_Check(uint16_t val){
	int ret;
	uint16_t FPC_status;
	//Enable Health Check
	ret = Health_Check_Enable(0);
	ret = Health_Check_Enable(1);
	
	ret = read_register(snt8100fsr_g, REGISTER_PHY_STAT_LSB, &FPC_status);
	if(ret < 0) {
		PRINT_ERR("Read 0x03 Fail");
		Health_Check_Enable(0);
		return -1;
	}
	PRINT_INFO("0x03: 0x%x, expect: 0x%x", FPC_status, FPC_status |val);
	FPC_value = FPC_status;
	if (FPC_status != (FPC_status | val)) {
		PRINT_INFO("Health Check Fail!!!");
		Health_Check_Enable(0);
		return -1;
	}
	ret = Health_Check_Enable(0);
	return 0;
}
/*---BSP Clay proc asusGripDebug Interface---*/
static void print_current_report(int i){
	PRINT_INFO("%u %u %u %u %u %u %u\n",
				snt8100fsr_g->track_reports_frame,
				snt8100fsr_g->track_reports[i].bar_id,
				snt8100fsr_g->track_reports[i].trk_id,
				snt8100fsr_g->track_reports[i].force_lvl,
				snt8100fsr_g->track_reports[i].top,
				snt8100fsr_g->track_reports[i].center,
				snt8100fsr_g->track_reports[i].bottom);
}
static int check_report_force(int force, int force_tolerance, int force_test){
	int force_top, force_floor;
	if(force_tolerance == 100){
		return 1;
	}else if((force_tolerance > 0) && (force_tolerance < 100)){
		force_top = force_test * (100 + force_tolerance) /100;
		force_floor = force_test * (100 - force_tolerance) /100;
	}else{
		force_top = force_test;
		force_floor = force_test;
	}
	PRINT_INFO("force check: force = %d, threshould = %d, tolerance = %d percent, top = %d, floor= %d", 
				force, force_test, force_tolerance, force_top, force_floor);
	if(force_test > 0){
		if(force >= force_floor && force <= force_top){
			return 1;
		}else{
			return 0;
		}
	} else {
		return 1;
	}
}

static int check_report_force_trans(int force, int force_test, int hi_tolerance, int lo_tolerance){
	int force_top, force_floor;
	if(force_test > 255){
		force_test = 255;
	}
	if(hi_tolerance == 100 && lo_tolerance == 100){
		return 1;
	}else if((hi_tolerance >= 0) && (hi_tolerance <= 100) && 
	(lo_tolerance > 0) && (lo_tolerance < 100)){
		force_top = force_test * (100 + hi_tolerance) /100;
		force_floor = force_test * (100 - lo_tolerance) /100;
	}else{
		force_top = force_test;
		force_floor = force_test;
	}
	PRINT_INFO("force check: force = %d, threshold=%d, top = %d, floor= %d, tolerance:hi= %d, lo=%d percent", 
				force, force_test, force_top, force_floor, hi_tolerance, lo_tolerance);
	if(force > 0){
		if(force >= force_floor && force <= force_top){
			return 1;
		}else{
			return 0;
		}
	} else {
		return 1;
	}
}

void Into_DeepSleep_fun(void){
	int ret = 0;
	int frame_rate=65535;
	//int frame_rate=20;
	if(snt8100fsr_g->chip_reset_flag != GRIP_FW_DL_END){
		PRINT_INFO("skip during fw download");
	}else if(grip_status_g->G_EN==0 || grip_status_g->G_EN==-1){
		//Disable irq when driver requests chip into deep sleep mode
		//Grip_Chip_IRQ_EN(0);
		MUTEX_LOCK(&snt8100fsr_g->IRQ_WAKE_SLEEP_LOCK);
		snt8100fsr_g->snt_state = GRIP_DEEPSLEEP;
		workqueue_cancel_work(&event_wq);
		ret = write_register(snt8100fsr_g,
				REGISTER_FRAME_RATE,
				&frame_rate);
		if(ret < 0) {
			PRINT_ERR("Grip register_enable write fail");
		}else{
			PRINT_INFO("Grip_EN = %d, Grip_Frame = %d", grip_status_g->G_EN, frame_rate);
		}
		//msleep(10);
		Grip_Driver_IRQ_EN(0);
		ms_start = get_time_in_ms();
		mutex_unlock(&snt8100fsr_g->IRQ_WAKE_SLEEP_LOCK);
	}
}

/*
void Wake_device_func(void){
	int ret;
	MUTEX_LOCK(&snt8100fsr_g->sb_lock);
	ret = sb_wake_device(snt8100fsr_g);
	if (ret) {
		PRINT_CRIT("sb_wake_device() failed");
		mutex_unlock(&snt8100fsr_g->sb_lock);
		return;
	}
	mutex_unlock(&snt8100fsr_g->sb_lock);
	msleep(100);
}
*/
void Wait_Wake_For_RegW(void){
#ifdef DYNAMIC_PWR_CTL
	//sb_wake_device(snt8100fsr_g);
	if(snt8100fsr_g->grip_fw_loading_status == true){
		if (snt_activity_request() != 0) {
			PRINT_CRIT("snt_activity_request() failed");
		}
	}else{
		PRINT_INFO("Load FW Fail, skip wakeup request");
	}
#endif
}
/* write DPC function */
void DPC_write_func(int flag){
#ifdef DYNAMIC_PWR_CTL
	int ret;
	grip_status_g->G_DPC_STATUS = flag;
	if(flag == 1){
		if(grip_game_gesture_status()){
			PRINT_INFO("Don't Enable DPC since tap/slide/swipe enable");
		}else{
			PRINT_INFO("Enable DPC, write 0x%x, 0x%x, 0x%x", 
				Grip_DPC_status_g->High, Grip_DPC_status_g->Low, Grip_DPC_status_g->Condition);
			ret = write_register(snt8100fsr_g, REGISTER_DPC_HIGH_FRAME, &Grip_DPC_status_g->High);
			if (ret) {
				PRINT_CRIT("set DPC 0x%x failed", REGISTER_DPC_HIGH_FRAME);
			}
			ret = write_register(snt8100fsr_g, REGISTER_DPC_LOW_FRAME, &Grip_DPC_status_g->Low);
			if (ret) {
				PRINT_CRIT("set DPC 0x%x failed", REGISTER_DPC_LOW_FRAME);
			}
			ret = write_register(snt8100fsr_g, REGISTER_DPC_CONDITION, &Grip_DPC_status_g->Condition);
			if (ret) {
				PRINT_CRIT("set DPC 0x%x failed", REGISTER_DPC_CONDITION);
			}
		}
	}else{
		PRINT_INFO("Disable DPC");
		ret = write_register(snt8100fsr_g, REGISTER_DPC_CONDITION, &flag);
		if (ret) {
			PRINT_CRIT("set DPC 0x%x failed", REGISTER_DPC_CONDITION);
		}
		ret = write_register(snt8100fsr_g, REGISTER_DPC_HIGH_FRAME, &flag);
		if (ret) {
			PRINT_CRIT("set DPC 0x%x failed", REGISTER_DPC_HIGH_FRAME);
		}
		ret = write_register(snt8100fsr_g, REGISTER_DPC_LOW_FRAME, &flag);
		if (ret) {
			PRINT_CRIT("set DPC 0x%x failed", REGISTER_DPC_LOW_FRAME);
		}
	}
#endif
}
void Check_Tap_sense_val(void){
	int i=0;
	uint32_t RegRead_t;
	msleep(50);
	for(i = 0; i < 3; i++){
		read_register(snt8100fsr_g, sys_param_addr[i], &RegRead_t);
		PRINT_INFO("Reg: 0x%x, Val: 0x%x", sys_param_addr[i], RegRead_t);
	}
}
void Tap_sense_write_func(int flag){
	int i = 0;
	if(grip_status_g->G_TAP_SENSE_EN != flag){
		grip_status_g->G_TAP_SENSE_EN = flag;
	}else{
		PRINT_INFO("TAP SENSE=%d, Already set before=======", flag);
		return;
	}
	if(flag == 1){
		PRINT_INFO("[Enable] Tap Sense data");
		for(i = 0; i < 3; i++){
			write_register(snt8100fsr_g, sys_param_addr[i], &Tap_sense_data1[i]);
		}
		Check_Tap_sense_val();
		for(i = 0; i < 3; i++){
			write_register(snt8100fsr_g, sys_param_addr[i], &Tap_sense_data2[i]);
		}
		Check_Tap_sense_val();
		
		for(i = 0; i < 3; i++){
			write_register(snt8100fsr_g, sys_param_addr[i], &Tap_sense_data3[i]);
		}
		Check_Tap_sense_val();
		
	}else{
		PRINT_INFO("[Disable] Tap Sense data");
		for(i = 0; i < 3; i++){
			write_register(snt8100fsr_g, sys_param_addr[i], &Tap_sense_reset_data1[i]);
		}
		Check_Tap_sense_val();
		for(i = 0; i < 3; i++){
			write_register(snt8100fsr_g, sys_param_addr[i], &Tap_sense_reset_data2[i]);
		}
		Check_Tap_sense_val();
	}
}

//Used in Tap function
static void grip_check_DPC_and_sensitivity_func(void){
	int ret = 0;
	
	/********* DPC part **********/
	if(grip_game_gesture_status()){
		//there exist tap which is on
		//Disable DPC
		if(grip_status_g->G_DPC_STATUS==1){
			snt8100fsr_g->frame_rate = 100;
			
			PRINT_INFO("game gesture enable, DPC turn off from on state and set frame_rate = %d", 
			snt8100fsr_g->frame_rate);
			DPC_write_func(0);
			ret = write_register(snt8100fsr_g, REGISTER_FRAME_RATE, &snt8100fsr_g->frame_rate);
			if (ret) {
				PRINT_CRIT("write_register(REGISTER_FRAME_RATE) failed");
			}
		}else{
			PRINT_INFO("DPC already off");
		}
	}else{ 
		if(grip_status_g->G_DPC_STATUS==0){
			PRINT_INFO("Enable DPC when all taps disable");
			DPC_write_func(1);
		}
	}

	/********* sensitivity part **********/	
	if(grip_status_g->G_TAP_EN[0] <= 0 && grip_status_g->G_TAP_EN[1] <= 0){
		//Disable tap sense
		if(grip_status_g->G_TAP_SENSE_SET == 1 && grip_status_g->G_TAP_SENSE_EN == 1){
			Tap_sense_write_func(0);
		}else{
			//Do nothing
		}
	}else if(grip_status_g->G_TAP_EN[0] == 1 && grip_status_g->G_TAP_EN[1] == 1){
		if(grip_status_g->G_TAP_SENSE_SET == 1 && grip_status_g->G_TAP_SENSE_EN <= 0){
			Tap_sense_write_func(1);
		}else{
			Tap_sense_write_func(0);
		}
	}else{
		//Do nothing
	}
}

static void grip_set_sys_param(const char *buf){
	int l = (int)strlen(buf) + 1;
	uint32_t val;
	uint32_t id;
	int status;
	int ret;
	PRINT_INFO("buf=%s, size=%d", buf, l);
	
	status  = ReadNumFromBuf((const uint8_t**)&buf, &l, &id);
	if (status != 0) {
		PRINT_CRIT("Could not parse param_id %d", status);
		goto errexit;
	}

	status = ReadNumFromBuf((const uint8_t**)&buf, &l, &val);
	if (status != 0) {
		PRINT_CRIT("Could not parse param_val %d", status);
		goto errexit;
	}
	if (enable_set_sys_param(snt8100fsr_g, id, val) != 0) {
		PRINT_DEBUG("send of set sys param failed");
		goto errexit;
	}

	// wait for response from driver irpt thread
	PRINT_DEBUG("SetSysParam Rsp -- wait");
	do {
		//ret= down_timeout(&snt8100fsr_g->sc_wf_rsp, msecs_to_jiffies(3*MSEC_PER_SEC);
		ret = down_interruptible(&snt8100fsr_g->sc_wf_rsp);
		PRINT_DEBUG("SetSysParam Rsp -- acquired %d", ret);
	} while (ret == -EINTR);

errexit:
    PRINT_DEBUG("done.");
    return;
}

/* ASUS BSP Clay: +++ swipe/slide used sense setting*/
static void grip_sense1_setting(bool flag){
	int i = 0;
	static bool status = false;
	if(status == flag){
		/* do nothing */
		return;
	}else{
		status = flag;
	}
	
	if(true == flag){
		for(i = 0; i < 3; i++){
			write_register(snt8100fsr_g, sys_param_addr[i], &Slide_Swipe_sense_data[i]);
		}
		PRINT_INFO("write 0x%x=0x%x, 0x%x, 0x%x", 
			sys_param_addr[0], Slide_Swipe_sense_data[0],
			Slide_Swipe_sense_data[1], Slide_Swipe_sense_data[2]);
	}else{
		for(i = 0; i < 3; i++){
			write_register(snt8100fsr_g, sys_param_addr[i], &Slide_Swipe_sense_reset_data[i]);
		}
		PRINT_INFO("write 0x%x=0x%x, 0x%x, 0x%x", 
			sys_param_addr[0], Slide_Swipe_sense_reset_data[0],
			Slide_Swipe_sense_reset_data[1], Slide_Swipe_sense_reset_data[2]);
	}
}
/* ASUS BSP Clay: ---*/
/* ASUS BSP Clay: +++ swipe used sense setting*/
static void grip_sense2_setting(bool flag){
	int i = 0;
	static bool status = false;
	if(status == flag){
		/* do nothing */
		return;
	}else{
		status = flag;
	}
	
	if(true == flag){
		for(i = 0; i < 3; i++){
			write_register(snt8100fsr_g, sys_param_addr[i], &Swipe_sense_data2[i]);
		}
		PRINT_INFO("write 0x%x=0x%x, 0x%x, 0x%x", 
			sys_param_addr[0], Swipe_sense_data2[0],
			Swipe_sense_data2[1], Swipe_sense_data2[2]);
	}else{
		for(i = 0; i < 3; i++){
			write_register(snt8100fsr_g, sys_param_addr[i], &Swipe_sense_reset_data2[i]);
		}
		PRINT_INFO("write 0x%x=0x%x, 0x%x, 0x%x", 
			sys_param_addr[0], Swipe_sense_reset_data2[0],
			Swipe_sense_reset_data2[1], Swipe_sense_reset_data2[2]);
	}
}
/* ASUS BSP Clay: ---*/

static void grip_set_game_gesture_sysp(bool flag){
	const char *buf_on = "110 6\n";
	const char *buf_off = "110 0\n";
	const char *buf_swipe_on = "110 4\n";
	static bool status = false;
	if(status == false || status != flag){
		if(flag == false){
			if(!grip_tap_gesture_status()){
				status = flag;
			}

			if(!grip_game_gesture_status()){ //all off
				msleep(50);
				grip_set_sys_param(buf_off);
				status = flag;
			}else if(grip_slide_gesture_status() == 0 && 
			grip_swipe_gesture_status()){ //swipe on, slide off
				msleep(50);
				grip_set_sys_param(buf_swipe_on);
				status = flag;
			}
		}else if(grip_game_gesture_status() == 1 && flag == true){
			msleep(50);
			grip_set_sys_param(buf_on);
			status = flag;
		}
	}
}

static void grip_slide_swipe_status_check(void){
	uint16_t enable_sensitive_boost = 0x84;
	uint16_t disable_sensitive_boost = 0x4;
	uint16_t boost_addr = 0x3d;
	static int swipe_status = 0, gesture_status = 0, slide_status = 0;
	const char *swipe_buf_on = "110 4\n";
	const char *slide_buf_on = "110 6\n";
	const char *buf_off = "110 0\n";
	
	if(grip_status_g->G_SLIDE_EN[0] == 1 || grip_status_g->G_SLIDE_EN[1] == 1
		|| grip_status_g->G_SWIPE_EN[0] == 1 || grip_status_g->G_SWIPE_EN[1] == 1){
		if(gesture_status != 1){
			gesture_status = 1;
		}
		grip_sense1_setting(true);
		if(grip_status_g->G_SLIDE_EN[0] == 1 || grip_status_g->G_SLIDE_EN[1] == 1){
			if(0 == slide_status){ //slide off => on
				slide_status = 1;
				/* set swipe buf off when slide has higher priority */
				write_register(snt8100fsr_g, boost_addr, &disable_sensitive_boost);
				msleep(50);
				grip_set_sys_param(slide_buf_on);
			}
			if(grip_status_g->G_SWIPE_EN[0] == 1 || grip_status_g->G_SWIPE_EN[1] == 1){
				if(0 == swipe_status){ //slide on, swipe off => on
					swipe_status = 1;
					grip_sense2_setting(true);
				}
			}else{
				if(1 == swipe_status){ //slide on, swipe on => off
					swipe_status = 0;
					grip_sense2_setting(false);
				}
			}
		}else{ //swipe enable, slide off
			if(1 == slide_status){ //swipe on, slide on => off
				slide_status = 0;
				write_register(snt8100fsr_g, boost_addr, &enable_sensitive_boost);
				msleep(50);
				if(!grip_tap_gesture_status()){
					grip_set_sys_param(swipe_buf_on);
				}
				grip_sense2_setting(true);
			}
			if(0 == swipe_status){ //slide off, swipe off=> on
				swipe_status = 1;
				write_register(snt8100fsr_g, boost_addr, &enable_sensitive_boost);
				msleep(50);
				if(!grip_tap_gesture_status()){
					grip_set_sys_param(swipe_buf_on);
				}
				grip_sense2_setting(true);
			}
		}
	}else{
		if(gesture_status != 0){
			gesture_status = 0;
			grip_sense1_setting(false);
			swipe_status = 0;
			slide_status = 0;
			write_register(snt8100fsr_g, boost_addr, &disable_sensitive_boost);
			msleep(50);
			if(grip_game_gesture_status() == 0){
				grip_set_sys_param(buf_off);
			}
			grip_sense2_setting(false);
		}
	}
}
/**************** +++ wrtie DPC & Frame function +++ **************/
void grip_frame_rate_func(int val){
	int ret;
	uint16_t RegRead_t = 0;
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	Wait_Wake_For_RegW();
	if(val == 10 || val == 20 || val == 25 || val == 40 || val == 50 || val == 80 || val == 100){
		PRINT_DEBUG("val = %d", val);
		if(grip_status_g->G_DPC_STATUS==1){
			Grip_DPC_status_g->High = val;
			RegRead_t = val;
			snt8100fsr_g->frame_rate = val;
			ret = write_register(snt8100fsr_g, REGISTER_DPC_HIGH_FRAME, &RegRead_t);
			if(ret < 0) {
				PRINT_ERR("Write reg 0x%X faill", REGISTER_DPC_HIGH_FRAME);	
			}else{
				PRINT_INFO("Write DPC High: 0x%x", RegRead_t);
			}
		}else{
			snt8100fsr_g->frame_rate = val;
			ret = write_register(snt8100fsr_g, REGISTER_FRAME_RATE, &snt8100fsr_g->frame_rate);
			if(ret < 0) {
				PRINT_ERR("Write reg 0x%X faill", REGISTER_FRAME_RATE);	
			}else{
				PRINT_INFO("Write frame rate: 0x%x", RegRead_t);
			}
		}
		mutex_unlock(&snt8100fsr_g->ap_lock);
	}else{
		PRINT_INFO("Not in defined frame rate range, skip val = %d", val);
		mutex_unlock(&snt8100fsr_g->ap_lock);
		return;
	}
}

/**************** ---wrtie DPC & Frame function --- **************/

/**************** +++ wrtie gesture function +++ **************/
void grip_enable_func_noLock(int val){

	if(g_snt_power_state == 0){
		grip_status_g->G_EN= 0;
		PRINT_INFO("Grip Sensor Power off, skip grip enable function");
		return;
	}
//	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	if(snt8100fsr_g->chip_reset_flag != GRIP_FW_DL_END){ //reseting
		//mutex_unlock(&snt8100fsr_g->ap_lock);
		grip_status_g->G_EN= val;
		PRINT_INFO("chip_reset_flag = %d", snt8100fsr_g->chip_reset_flag);
		return;
	}

	if(snt8100fsr_g->grip_fw_loading_status == false){
		grip_status_g->G_EN= 0;
		PRINT_INFO("Load fw fail, skip grip enable function");
		return;
	}
	if(val == 1){ //turn on
		if(grip_status_g->G_EN <= 0){ //Need turn on
			// We mutex lock here since we're calling sb_wake_device which never locks
			Grip_Driver_IRQ_EN(1);

			/* Check Time before wake up */
			ms_end = get_time_in_ms();
			if((ms_end-ms_start)< snt8100fsr_g->sleep_ms_time){
				PRINT_INFO("ms_start=%u, ms_end=%u", ms_start, ms_end);
				msleep(snt8100fsr_g->sleep_ms_time-(ms_end-ms_start));
			}

			
			Wait_Wake_For_RegW();
			//Grip_Chip_IRQ_EN(1);
			Grip_DPC_status_g->Low = 5;
			DPC_write_func(1);
			write_register(snt8100fsr_g, REGISTER_FRAME_RATE, &snt8100fsr_g->frame_rate);
			PRINT_INFO("Grip_EN = %d , Grip_Frame = %d", grip_status_g->G_EN, snt8100fsr_g->frame_rate);
		}
	}else{
		if(grip_status_g->G_EN == 1){
			grip_status_g->G_EN= val;
			Wait_Wake_For_RegW();
			DPC_write_func(0);
			Into_DeepSleep_fun();
		}
	}
	grip_status_g->G_EN= val;
	//mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_raw_enable_func(int val){
	int ret;
	uint16_t RegRead_t = 0;
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	if(grip_status_g->G_RAW_EN == val){
		mutex_unlock(&snt8100fsr_g->ap_lock);
		return;
	}
	grip_status_g->G_RAW_EN = val;
	if(Grip_Check_FW_Status()){ return; }
	
	Wait_Wake_For_RegW();
	if(val == 0){
		val = 1;
	}else{
		val = 0;
	}
	RegRead_t = val << 3;
	ret = write_register(snt8100fsr_g, REGISTR_RAW_DATA, &RegRead_t);
	if(ret < 0) {
		PRINT_ERR("Write reg 0x%X faill", REGISTR_RAW_DATA);	
	}
	track_report_count = 0;
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

static int grip_all_gesture_status(){
	if(grip_status_g->G_SQUEEZE_EN[0] == 1 || grip_status_g->G_SQUEEZE_EN[1] == 1
		|| grip_status_g->G_TAP_EN[0] == 1 || grip_status_g->G_TAP_EN[1] == 1
		|| grip_status_g->G_TAP_EN[2] == 1 || grip_status_g->G_TAP_EN[3] == 1
		|| grip_status_g->G_SWIPE_EN[0] == 1 || grip_status_g->G_SWIPE_EN[1] == 1
		|| grip_status_g->G_SLIDE_EN[0] == 1 || grip_status_g->G_SLIDE_EN[1] == 1){
		return 1;
	}else{ /* No gesture or raw enable */
		return 0;
	}
}

int grip_game_gesture_status(){
	if(grip_tap_gesture_status() || grip_slide_gesture_status() ||
	grip_swipe_gesture_status()){
		return 1;
	}else{ /* No gesture or raw enable */
		return 0;
	}
}

static int grip_tap_gesture_status(){
	if(grip_status_g->G_TAP_EN[0] == 1 || grip_status_g->G_TAP_EN[1] == 1
		|| grip_status_g->G_TAP_EN[2] == 1 || grip_status_g->G_TAP_EN[3] == 1){
		return 1;
	}else{ /* No gesture or raw enable */
		return 0;
	}
}

static int grip_swipe_gesture_status(){
	if(grip_status_g->G_SWIPE_EN[0] == 1 || grip_status_g->G_SWIPE_EN[1] == 1){
		return 1;
	}else{ /* No gesture or raw enable */
		return 0;
	}
}

static int grip_slide_gesture_status(){
	if(grip_status_g->G_SLIDE_EN[0] == 1 || grip_status_g->G_SLIDE_EN[1] == 1){
		return 1;
	}else{ /* No gesture or raw enable */
		return 0;
	}
}

static void grip_checkToLowPower_noLock(){
	if(grip_all_gesture_status()){
		/* Do nothing */
	}else{ /* No gesture or raw enable */
		grip_enable_func_noLock(0);
	}
}

int write_registers_fifo(int reg, int num, void *value) {
	int ret=0;
	mutex_lock(&snt8100fsr_g->sb_lock);
	ret = sb_write_fifo(snt8100fsr_g, reg, num*2, value);
	if (ret) {
		PRINT_CRIT("write_registers_fifo() failed (%d)", ret);
	}
	mutex_unlock(&snt8100fsr_g->sb_lock);
	return ret;
}

int read_registers_fifo(int reg, int num, void *value) {
	int ret=0;
	mutex_lock(&snt8100fsr_g->sb_lock);
	ret = sb_read_fifo(snt8100fsr_g, reg, num*2, value);
	if (ret) {
		PRINT_CRIT("cust_read_registers() failed (%d)", ret);
	} else {
	}
	mutex_unlock(&snt8100fsr_g->sb_lock);
	return ret;
}

int tap_reg_num = 16;
int squeeze_reg_num = 18;
int slide_reg_num = 12;
int swipe_reg_num = 10;

//Tap part start===========================================
void get_tap_gesture(uint16_t tap_id, uint16_t reg_val, int index, int len){
	//int ret=0;
	//int bytes=0;
	int i=0;
	uint16_t cfg_bank_write[3] = { 0, 0, 0x0801};
	uint16_t buf[255];
	write_registers_fifo(0x2c, 3, cfg_bank_write);
	read_registers_fifo(0x200, len, buf);

	for(i = 0; i < len; i++){
		//if(i%8==1 || i%8==2){
		PRINT_INFO("reg_val=0x%x", buf[i]);
		//}
	}
}


void set_tap_gesture(uint16_t tap_id, uint16_t reg_val, int index){
	uint16_t cfg_bank_write[3] = { tap_id*tap_reg_num + index*2, 2, 0x0802};
	uint16_t cfg_bank_commit[3] = { 0, 0, 0x0803};
	write_registers_fifo(0x2c, 3, cfg_bank_write);
	write_registers_fifo(0x200, 1, &reg_val);
	write_registers_fifo(0x2c, 3, cfg_bank_commit);
}

void grip_tap_enable_func(int tap_id, int val, uint16_t* reg ) {
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);

	if( val==1 ) {
		grip_enable_func_noLock(1);
	}
	grip_status_g->G_TAP_EN[tap_id] = val;
	if(grip_status_g->G_EN == -1)
		grip_status_g->G_EN = 0;
		
	/* when G_EN=0 which means that all gestures close, don't wakeup chip and do setting */
	if(grip_status_g->G_EN == 1){
		if(Grip_Check_FW_Status()){ return; }
		
		Wait_Wake_For_RegW();
		*reg = (*reg & 0xFFFE) | ( val & 0x0001);
		set_tap_gesture(tap_id, *reg, 0);

		/* ASUS BSP Clay: setting vibrator trig1 & trig2 register for different tap modes +++ */
		gripVibratorSetting(val, tap_id);
		/* ASUS BSP Clay:--- */

		//get_tap_gesture(0, *reg, 0, tap_id*8 +9);
		grip_check_DPC_and_sensitivity_func();
		if(val == 1){
			grip_set_game_gesture_sysp(true);
		}else{
			grip_set_game_gesture_sysp(false);
		}

		if( val==0) { //turn off, check if (all off) goto low power
			grip_checkToLowPower_noLock();
		}
	}else{
		PRINT_INFO("No gesture enable, skip it");
	}
	mutex_unlock(&snt8100fsr_g->ap_lock);
}
void grip_tapX_enable_func(int val, int id, uint16_t *reg_val ){
	
	//int id = 0;
	grip_tap_enable_func(id, val, reg_val);
	PRINT_INFO("Write tap id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_tap_force_func(int tap_id, int val, uint16_t* reg ) {
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_TAP_FORCE[tap_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	val = val << 8;
	*reg = (val & 0xFF00) | ( *reg & 0x00FF);
	set_tap_gesture(tap_id, *reg, 0);
	//get_tap_gesture(0, *reg, 0, tap_id*8 +9);
	/* check bar scan behavior */
	mutex_unlock(&snt8100fsr_g->ap_lock);	
}

void grip_tapX_force_func(int val, int id, uint16_t *reg_val ){
	
	//int id = 0;
	grip_tap_force_func(id, val, reg_val);
	PRINT_INFO("Write tap id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_tap_min_position_func(int tap_id, int val, uint16_t* reg ) {
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_TAP_MIN_POS[tap_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	*reg = val;
	set_tap_gesture(tap_id, *reg, 2);
	//get_tap_gesture(0, *reg, 0, tap_id*8 +9);
	/* check bar scan behavior */
	mutex_unlock(&snt8100fsr_g->ap_lock);	
}

void grip_tapX_min_position_func(int val, int id, uint16_t *reg_val ){
	
	//int id = 0;
	grip_tap_min_position_func(id, val, reg_val);
	PRINT_INFO("Write tap id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_tap_max_position_func(int tap_id, int val, uint16_t* reg ) {
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_TAP_MAX_POS[tap_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	*reg = val;
	set_tap_gesture(tap_id, *reg, 3);
	//get_tap_gesture(0, *reg, 0, tap_id*8 +9);
	mutex_unlock(&snt8100fsr_g->ap_lock);	
}

void grip_tapX_max_position_func(int val, int id, uint16_t *reg_val ){
	
	//int id = 0;
	grip_tap_max_position_func(id, val, reg_val);
	PRINT_INFO("Write tap id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_tap_sense_enable_func(int val){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_TAP_SENSE_SET= val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	grip_check_DPC_and_sensitivity_func();
	mutex_unlock(&snt8100fsr_g->ap_lock);
}
void grip_tap_slope_window_func(int tap_id, int val, uint16_t* reg ) {
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_TAP_SLOPE_WINDOW[tap_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	*reg = val | (*reg & 0xFF00);
	set_tap_gesture(tap_id, *reg, 1);
	//get_tap_gesture(0, *reg, 0, tap_id*8 +9);
	mutex_unlock(&snt8100fsr_g->ap_lock);	
}

void grip_tapX_slope_window_func(int val, int id, uint16_t *reg_val ){
	
	//int id = 0;
	grip_tap_slope_window_func(id, val, reg_val);
	PRINT_INFO("Write tap id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_tap_slope_release_force_func(int tap_id, int val, uint16_t* reg ) {
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_TAP_SLOPE_RELEASE_FORCE[tap_id] = val;

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val<<8 | (*reg & 0x00FF);
	set_tap_gesture(tap_id, *reg, 5);
	//get_tap_gesture(0, *reg, 0, tap_id*8 +9);
	mutex_unlock(&snt8100fsr_g->ap_lock);	
}

void grip_tapX_slope_release_force_func(int val, int id, uint16_t *reg_val ){
	
	//int id = 0;
	grip_tap_slope_release_force_func(id, val, reg_val);
	PRINT_INFO("Write tap id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_tap_slope_tap_force_func(int tap_id, int val, uint16_t* reg ) {
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_TAP_SLOPE_TAP_FORCE[tap_id] = val;
	

	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val | (*reg & 0xFF00);
	set_tap_gesture(tap_id, *reg, 5);
	//get_tap_gesture(0, *reg, 0, tap_id*8 +9);
	mutex_unlock(&snt8100fsr_g->ap_lock);	
}

void grip_tapX_slope_tap_force_func(int val, int id, uint16_t *reg_val ){
	
	//int id = 0;
	grip_tap_slope_tap_force_func(id, val, reg_val);
	PRINT_INFO("Write tap id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_tap_delta_tap_force_func(int tap_id, int val, uint16_t* reg ) {
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_TAP_DELTA_TAP_FORCE[tap_id] = val;
	
	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val | (*reg & 0xFF00);
	set_tap_gesture(tap_id, *reg, 4);
	//get_tap_gesture(0, *reg, 0, tap_id*8 +9);
	mutex_unlock(&snt8100fsr_g->ap_lock);	
}

void grip_tapX_delta_tap_force_func(int val, int id, uint16_t *reg_val ){
	
	//int id = 0;
	grip_tap_delta_tap_force_func(id, val, reg_val);
	PRINT_INFO("Write tap id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_tap_delta_release_force_func(int tap_id, int val, uint16_t* reg ) {
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_TAP_DELTA_RELEASE_FORCE[tap_id] = val;
	
	if(Grip_Check_FW_Status()){return;}

	Wait_Wake_For_RegW();
	*reg = val<<8 | (*reg & 0x00FF);
	set_tap_gesture(tap_id, *reg, 4);
	//get_tap_gesture(0, *reg, 0, tap_id*8 +9);
	mutex_unlock(&snt8100fsr_g->ap_lock);	
}

void grip_tapX_delta_release_force_func(int val, int id, uint16_t *reg_val ){
	
	//int id = 0;
	grip_tap_delta_release_force_func(id, val, reg_val);
	 PRINT_INFO("Write tap id=%d, val=%d, reg: %x", id, val, *reg_val);
}

/* ASUS BSP Clay: setting vibrator trig1 & trig2 register for different tap modes +++ */
static void gripVibratorSetting(int val, int tap_id){
	static uint16_t reg_val[4] = { 0x1, 0x2, 0x8, 0x20 };
	PRINT_INFO("Tap_id=%d, EN=%d, VIB_EN=%d, 0x%x, 0x%x", 
		tap_id,	grip_status_g->G_TAP_EN[tap_id],
		grip_status_g->G_TAP_VIB_EN[tap_id],
		G_grip_tap_vib1_reg, G_grip_tap_vib2_reg);
	
	if(g_ASUS_hwID >= HW_REV_PR){
		if(val && grip_status_g->G_TAP_EN[tap_id]==1 && grip_status_g->G_TAP_VIB_EN[tap_id]==1){
			if(tap_id == 1){
				if(grip_status_g->G_TAP_EN[tap_id]==1 && grip_status_g->G_TAP_EN[tap_id+2]==1){
					PRINT_INFO("Double tap case");
					/* tap2 */
					G_grip_tap_vib1_reg = G_grip_tap_vib1_reg & (0xffff - reg_val[tap_id]);
					G_grip_tap_vib2_reg = G_grip_tap_vib2_reg | reg_val[tap_id];
				}else{
					G_grip_tap_vib1_reg = G_grip_tap_vib1_reg | reg_val[tap_id];
					G_grip_tap_vib2_reg = G_grip_tap_vib2_reg & (0xffff - reg_val[tap_id]);
				}
			}else if(tap_id == 3){
				if(grip_status_g->G_TAP_EN[tap_id-2]==1	&& grip_status_g->G_TAP_VIB_EN[tap_id-2]==1){
					PRINT_INFO("Double tap case");
					/* tap2 */
					G_grip_tap_vib1_reg = G_grip_tap_vib1_reg & (0xffff - reg_val[tap_id-2]);
					G_grip_tap_vib2_reg = G_grip_tap_vib2_reg | reg_val[tap_id-2];
				}
				G_grip_tap_vib1_reg = G_grip_tap_vib1_reg | reg_val[tap_id];
			}else if(tap_id == 2){
				G_grip_tap_vib2_reg = G_grip_tap_vib2_reg | reg_val[tap_id];
			}else{
				G_grip_tap_vib1_reg = G_grip_tap_vib1_reg | reg_val[tap_id];
			}			
		}else{
			G_grip_tap_vib1_reg = G_grip_tap_vib1_reg & (0xffff - reg_val[tap_id]);
			G_grip_tap_vib2_reg = G_grip_tap_vib2_reg & (0xffff - reg_val[tap_id]);
			//tap3 off or tap3_vib off
			//recovery case: tap2 on and tap2_vib on 
			if(tap_id==3 && grip_status_g->G_TAP_EN[tap_id-2]==1 
				&& grip_status_g->G_TAP_VIB_EN[tap_id-2]==1){
				PRINT_INFO("Double tap to One tap");
				G_grip_tap_vib1_reg = G_grip_tap_vib1_reg | reg_val[tap_id-2];
				G_grip_tap_vib2_reg = G_grip_tap_vib2_reg & (0xffff - reg_val[tap_id-2]);
			}
		}
	}else{
		if(val && grip_status_g->G_TAP_EN[tap_id]==1 && grip_status_g->G_TAP_VIB_EN[tap_id]==1){
			G_grip_tap_vib1_reg = G_grip_tap_vib1_reg | reg_val[tap_id];
		}else{
			G_grip_tap_vib1_reg = G_grip_tap_vib1_reg & (0xffff - reg_val[tap_id]);
		}
	}
	Wait_Wake_For_RegW();
	write_register(snt8100fsr_g, REGISTER_TIRGGER_LINK1, &G_grip_tap_vib1_reg);
	write_register(snt8100fsr_g, REGISTER_TIRGGER_LINK2, &G_grip_tap_vib2_reg);
	PRINT_INFO("[0x%x] = 0x%x, [0x%x] = 0x%x", 
		REGISTER_TIRGGER_LINK1, G_grip_tap_vib1_reg,
		REGISTER_TIRGGER_LINK2, G_grip_tap_vib2_reg);
}
/* ASUS BSP--- */
void grip_tapX_vibrator_enable_func(int val, int tap_id){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_INFO("val = %d, tap_id=%d", val, tap_id);
	grip_status_g->G_TAP_VIB_EN[tap_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	/* ASUS BSP Clay: setting vibrator trig1 & trig2 register for different tap modes +++ */
	gripVibratorSetting(val, tap_id);
	/* ASUS BSP--- */
	mutex_unlock(&snt8100fsr_g->ap_lock);
}
//Tap part down===========================================

//Squeeze part start===========================================
void get_sq_gesture(uint16_t tap_id, uint16_t reg_val, int index, int len){
	//int ret=0;
	//int bytes=0;
	int i=0;
	uint16_t cfg_bank_read[3] = { 0, 0, 0x0a01};
	uint16_t buf[255];
	write_registers_fifo(0x2c, 3, cfg_bank_read);
	read_registers_fifo(0x200, len, buf);

	for(i = 0; i < len; i++){
		PRINT_INFO("reg_val=0x%x", buf[i]);
	}
}

void set_sq_gesture(uint16_t sq_id, uint16_t reg_val, int index){
	uint16_t cfg_bank_write[3] = { sq_id*squeeze_reg_num + index * 2, 2, 0x0a02};
	uint16_t cfg_bank_commit[3] = { 0, 0, 0x0a03};
	write_registers_fifo(0x2c, 3, cfg_bank_write);
	write_registers_fifo(0x200, 1, &reg_val);
	write_registers_fifo(0x2c, 3, cfg_bank_commit);
}
void grip_squeeze_enable_func(int sq_id, int val, uint16_t* reg){
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_INFO("id=%d, val = %d", sq_id, val);

	if( val==1) {
		grip_enable_func_noLock(1);
	}
	
	grip_status_g->G_SQUEEZE_EN[sq_id] = val;
	if(grip_status_g->G_EN == -1)
		grip_status_g->G_EN = 0;
	
	/* when G_EN=0 which means that all gestures close, don't wakeup chip and do setting */
	if(grip_status_g->G_EN == 1){
		
		if(Grip_Check_FW_Status()){return;}
		
		Wait_Wake_For_RegW();
		val = val << 15;
		* reg = (val & 0x8000) | (* reg & 0x7FFF);
		set_sq_gesture(sq_id, * reg, 0);
		//get_sq_gesture(0, * reg, 0, 2);

		if(val == 1){
			grip_set_game_gesture_sysp(false);
		}

		if( val==0) { //turn off, check if (all off) goto low power
			grip_checkToLowPower_noLock();
		}
	}else{
		PRINT_INFO("No gesture enable, skip it");
	}
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_squeezeX_enable_func(int val, int id, uint16_t *reg_val ){
	grip_squeeze_enable_func(id, val, reg_val);
	PRINT_INFO("Write sq id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_squeeze1_enable_func(int val){
	int id=0;
	grip_squeeze_enable_func(id, val, &SQ1_BIT0);
	PRINT_INFO("Write sq1 reg: %x", SQ1_BIT0);
}

void grip_squeeze2_enable_func(int val){
	int id=1;
	grip_squeeze_enable_func(id, val, &SQ2_BIT0);
	PRINT_INFO("Write sq2 reg: %x", SQ2_BIT0);
}
	
void grip_squeeze_force_func(int sq_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	
	grip_status_g->G_SQUEEZE_FORCE[sq_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	*reg = (val & 0x00FF) | (*reg & 0xFF00);
	set_sq_gesture(sq_id, * reg, 0);
	//get_sq_gesture(0, * reg, 0, 2);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_squeezeX_force_func(int val, int id, uint16_t *reg_val ){
	grip_squeeze_force_func(id, val, reg_val);
	PRINT_INFO("Write sq id=%d, val=%d, reg: %x", id, val, *reg_val);
}

bool G_Skip_Sq1_Long = 0;
bool G_Skip_Sq2_Long = 0;
	
void grip_squeeze_short_dur_func(int sq_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	
	grip_status_g->G_SQUEEZE_SHORT[sq_id]= val;
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	*reg = ((val/20) << 8) | (*reg & 0x00FF);
	set_sq_gesture(sq_id, *reg, 5);
	//get_sq_gesture(0, * reg, 0, 2);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_squeezeX_short_dur_func(int val, int id, uint16_t *reg_val ){
	grip_squeeze_short_dur_func(id, val, reg_val);
	PRINT_INFO("Write sq id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_squeeze_long_dur_func(int sq_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	
	if(sq_id == 0){
		grip_status_g->G_SQUEEZE_LONG[sq_id]= val;
		if(val == 0){
			G_Skip_Sq1_Long = 1;
			mutex_unlock(&snt8100fsr_g->ap_lock);
			return;
		}else
			G_Skip_Sq1_Long = 0;
	}else if(sq_id ==1){
		grip_status_g->G_SQUEEZE_LONG[sq_id] = val;
		if(val == 0){
			G_Skip_Sq2_Long = 1;
			mutex_unlock(&snt8100fsr_g->ap_lock);
			return;
		}else
			G_Skip_Sq2_Long = 0;
	}
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	*reg = (val/20) | (*reg & 0xFF00);
	set_sq_gesture(sq_id, *reg, 5);
	//get_sq_gesture(0, *reg, 0, 6);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_squeezeX_long_dur_func(int val, int id, uint16_t *reg_val ){
	grip_squeeze_long_dur_func(id, val, reg_val);
	PRINT_INFO("Write sq id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_squeeze_up_rate_func(int sq_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SQUEEZE_UP_RATE[sq_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	*reg = val<<8 | (*reg & 0x00FF);
	set_sq_gesture(sq_id, *reg, 6);
	PRINT_INFO("Write Squeeze_up_rate: 0x%x", * reg);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_squeezeX_up_rate_func(int val, int id, uint16_t *reg_val ){
	grip_squeeze_up_rate_func(id, val, reg_val);
	PRINT_INFO("Write sq id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_squeeze_up_total_func(int sq_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SQUEEZE_UP_TOTAL[sq_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	*reg = val | (*reg & 0xFF00);
	set_sq_gesture(sq_id, *reg, 6);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_squeezeX_up_total_func(int val, int id, uint16_t *reg_val ){
	grip_squeeze_up_total_func(id, val, reg_val);
	PRINT_INFO("Write sq id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_squeeze_drop_rate_func(int sq_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SQUEEZE_DROP_RATE[sq_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	*reg = val<<8 | (*reg & 0x00FF);
	set_sq_gesture(sq_id, *reg, 7);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_squeezeX_drop_rate_func(int val, int id, uint16_t *reg_val ){
	grip_squeeze_drop_rate_func(id, val, reg_val);
	PRINT_INFO("Write sq id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_squeeze_drop_total_func(int sq_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SQUEEZE_DROP_TOTAL[sq_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	*reg = val | (*reg & 0xFF00);
	set_sq_gesture(sq_id, *reg, 7);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_squeezeX_drop_total_func(int val, int id, uint16_t *reg_val ){
	grip_squeeze_drop_total_func(id, val, reg_val);
	PRINT_INFO("Write sq id=%d, val=%d, reg: %x", id, val, *reg_val);
}

//Squeeze part down===========================================

/****************** Slide ********************************/
void get_slide_gesture(uint16_t tap_id, uint16_t reg_val, int index, int len){
	//int ret=0;
	//int bytes=0;
	int i=0;
	uint16_t cfg_bank_read[3] = { 0, 0, 0x0b01};
	uint16_t buf[255];
	write_registers_fifo(0x2c, 3, cfg_bank_read);
	read_registers_fifo(0x200, len, buf);

	for(i = 0; i < len; i++){
		PRINT_INFO("reg_val=0x%x", buf[i]);
	}
}
void set_slide_gesture(uint16_t slide_id, uint16_t reg_val, int index){
	uint16_t cfg_bank_write[3] = { slide_id*slide_reg_num + index * 2, 2, 0x0b02};
	uint16_t cfg_bank_commit[3] = { 0, 0, 0x0b03};
	write_registers_fifo(0x2c, 3, cfg_bank_write);
	write_registers_fifo(0x200, 1, &reg_val);
	write_registers_fifo(0x2c, 3, cfg_bank_commit);
}

void grip_slide_enable_func(int slide_id, int val, uint16_t* reg){
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);

	if( val==1) {
		grip_enable_func_noLock(1);
	}
	
	grip_status_g->G_SLIDE_EN[slide_id] = val;
	if(grip_status_g->G_EN == -1)
		grip_status_g->G_EN = 0;
	
	/* when G_EN=0 which means that all gestures close, don't wakeup chip and do setting */
	if(grip_status_g->G_EN == 1){		
		
		if(Grip_Check_FW_Status()){return;}
		
		Wait_Wake_For_RegW();
		val = val << 15;
		* reg = (val & 0x8000) | (* reg & 0x7FFF);
		set_slide_gesture(slide_id, *reg, 0);
		grip_check_DPC_and_sensitivity_func();
		grip_slide_swipe_status_check();

		if( val==0) { //turn off, check if (all off) goto low power
			grip_checkToLowPower_noLock();
		}
	}else{
		PRINT_INFO("No gesture enable, skip it");
	}
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_slideX_enable_func(int val, int id, uint16_t *reg_val){
	grip_slide_enable_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_slide_dist_func(int slide_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SLIDE_DIST[slide_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	val = val << 8;
	*reg = (val & 0xFF00) | (*reg & 0x00FF);
	set_slide_gesture(slide_id, *reg, 3);
	PRINT_INFO("Write Slide1_Dist: 0x%x", *reg);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_slideX_dist_func(int val, int id, uint16_t *reg_val){
	grip_slide_dist_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_slide_force_func(int slide_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SLIDE_FORCE[slide_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	*reg = (val & 0x00FF) | (*reg & 0xFF00);
	set_slide_gesture(slide_id, *reg, 3);
	PRINT_INFO("Write Slide1_Force: 0x%x", *reg);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_slideX_force_func(int val, int id, uint16_t *reg_val){
	grip_slide_force_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_slide_2nd_dist_func(int slide_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SLIDE_2ND_DIST[slide_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	*reg = (val & 0x00FF) | (*reg & 0xFF00);
	set_slide_gesture(slide_id, *reg, 4);
	PRINT_INFO("Write Slide1_Force: 0x%x", *reg);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_slideX_2nd_dist_func(int val, int id, uint16_t *reg_val){
	grip_slide_2nd_dist_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_slide_vibrator_enable_func(int slide_id, int val){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SLIDE_VIB_EN[slide_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	PRINT_INFO("SLIDE1 Do nothing");
	mutex_unlock(&snt8100fsr_g->ap_lock);	
}

void grip_slideX_vibrator_enable_func(int val, int id){
	grip_slide_vibrator_enable_func(id, val);
	PRINT_INFO("Write slide id=%d, val=%d", id, val);
}

void grip_slide_tap_priority_func(int slide_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SLIDE_TAP_PRIORITY[slide_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	val = val << 14;
	*reg = (val & 0x4000) | (*reg & 0xBFFF);
	set_slide_gesture(slide_id, *reg, 0);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_slideX_tap_priority_func(int val, int id, uint16_t* reg_val){
	grip_slide_tap_priority_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_slide_min_position_func(int slide_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SLIDE_MIN_POS[slide_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	*reg = val;
	set_slide_gesture(slide_id, *reg, 1);
	PRINT_INFO("Write Slide1_Force: 0x%x", *reg);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_slideX_min_position_func(int val, int id, uint16_t *reg_val){
	grip_slide_min_position_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_slide_max_position_func(int slide_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SLIDE_MAX_POS[slide_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	*reg = val;
	set_slide_gesture(slide_id, *reg, 2);
	PRINT_INFO("Write Slide1_Force: 0x%x", *reg);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_slideX_max_position_func(int val, int id, uint16_t *reg_val){
	grip_slide_max_position_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

/****************** SWIPE ********************************/
void get_swipe_gesture(uint16_t swipe_id, uint16_t reg_val, int index, int len){
	//int ret=0;
	//int bytes=0;
	int i=0;
	uint16_t cfg_bank_read[3] = { 0, 0, 0x0901};
	uint16_t buf[255];
	write_registers_fifo(0x2c, 3, cfg_bank_read);
	read_registers_fifo(0x200, len, buf);

	for(i = 0; i < len; i++){
		PRINT_INFO("reg_val=0x%x", buf[i]);
	}
}

void set_swipe_gesture(uint16_t swipe_id, uint16_t reg_val, int index){
	uint16_t cfg_bank_write[3] = { swipe_id*swipe_reg_num + index * 2, 2, 0x0902};
	uint16_t cfg_bank_commit[3] = { 0, 0, 0x0903};
	write_registers_fifo(0x2c, 3, cfg_bank_write);
	write_registers_fifo(0x200, 1, &reg_val);
	write_registers_fifo(0x2c, 3, cfg_bank_commit);
}

void grip_swipe_enable_func(int swipe_id, int val, uint16_t* reg){
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	
	if( val==1) {
		grip_enable_func_noLock(1);
	}
	
	grip_status_g->G_SWIPE_EN[swipe_id] = val;
	if(grip_status_g->G_EN == -1)
		grip_status_g->G_EN = 0;
	
	/* when G_EN=0 which means that all gestures close, don't wakeup chip and do setting */
	if(grip_status_g->G_EN == 1){
		
		if(Grip_Check_FW_Status()){return;}
		
		Wait_Wake_For_RegW();
		val = val << 15;
		* reg = (val & 0x8000) | (* reg & 0x7FFF);
		set_swipe_gesture(swipe_id, *reg, 0);
		grip_check_DPC_and_sensitivity_func();
		grip_slide_swipe_status_check();

		if( val==0) { //turn off, check if (all off) goto low power
			grip_checkToLowPower_noLock();
		}
	}else{
		PRINT_INFO("No gesture enable, skip it");
	}
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_swipeX_enable_func(int val, int id, uint16_t *reg_val){
	grip_swipe_enable_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_swipe_velocity_func(int swipe_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SWIPE_VELOCITY[swipe_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	* reg = (val & 0x00FF) | (* reg & 0xFF00);
	set_swipe_gesture(swipe_id, * reg, 0);
	 PRINT_INFO("Write Swipe_Velocity[%d]: 0x%x", swipe_id, *reg);
	 mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_swipeX_velocity_func(int val, int id, uint16_t *reg_val){
	grip_swipe_velocity_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_swipe_len_func(int swipe_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SWIPE_LEN[swipe_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	*reg = val;
	set_swipe_gesture(swipe_id, *reg, 3);
	PRINT_INFO("Write Swipe_Len[%d]: 0x%x", swipe_id, *reg);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_swipeX_len_func(int val, int id, uint16_t *reg_val){
	grip_swipe_len_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_swipe_min_position_func(int swipe_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SWIPE_MIN_POS[swipe_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	*reg = val;
	set_swipe_gesture(swipe_id, *reg, 1);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_swipeX_min_position_func(int val, int id, uint16_t *reg_val){
	grip_swipe_min_position_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}

void grip_swipe_max_position_func(int swipe_id, int val, uint16_t* reg){
	if(grip_status_g->G_EN <= 0){
		PRINT_INFO("Skip setting when grip off");
		return;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_DEBUG("val = %d", val);
	grip_status_g->G_SWIPE_MAX_POS[swipe_id] = val;
	
	if(Grip_Check_FW_Status()){return;}
	
	Wait_Wake_For_RegW();
	*reg = val;
	set_swipe_gesture(swipe_id, *reg, 2);
	mutex_unlock(&snt8100fsr_g->ap_lock);
}

void grip_swipeX_max_position_func(int val, int id, uint16_t *reg_val){
	grip_swipe_max_position_func(id, val, reg_val);
	PRINT_INFO("Write slide id=%d, val=%d, reg: %x", id, val, *reg_val);
}
/****************** SWIPE ********************************/
/* When K data is applied, squeeze should re-enable to make K data work */
static void Grip_ReEnable_Squeeze_Check(void){
	int count = 0;
	int sq_num = 2;
	/* apply K data to squeeze algorithm */
	for(count=0;count < sq_num;count++){
		if(grip_status_g->G_SQUEEZE_EN[count] > 0){
			grip_squeezeX_enable_func(grip_status_g->G_SQUEEZE_EN[count], count, &SQ_BIT0[count]);
		}
	}
}

static void Grip_K_data_recovery(void){
	/* do nothing */
}

/* Recovery status after reset */
void grip_dump_status_func(struct work_struct *work){
	int count=0;
	int sq_num=2, tap_num=4, swipe_num=2, slide_num=2;
	int need_en=0;
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	PRINT_INFO("framework setting recovery");
	PRINT_INFO("EN:%d, RAW_EN:%d, DPC_EN:%d",
		grip_status_g->G_EN, grip_status_g->G_RAW_EN, 
		grip_status_g->G_DPC_STATUS);
	for(count=0;count < sq_num;count++){
		PRINT_INFO("SQ[%d], EN:%d, Force:%d, Short:%d, Long:%d",
			count, grip_status_g->G_SQUEEZE_EN[count], grip_status_g->G_SQUEEZE_FORCE[count], 
			grip_status_g->G_SQUEEZE_SHORT[count], grip_status_g->G_SQUEEZE_LONG[count]);
	}
	for(count=0;count < tap_num;count++){
		PRINT_INFO("TAP[%d], EN:%d, Force:%d, min_pos:%d, max_pos:%d",
			count, grip_status_g->G_TAP_EN[count], grip_status_g->G_TAP_FORCE[count], 
			grip_status_g->G_TAP_MIN_POS[count], grip_status_g->G_TAP_MAX_POS[count]);
		
		PRINT_INFO("slope_window:%d, slope_tap:%d, slope_release:%d, delta_tap:%d, delta_release:%d",
			grip_status_g->G_TAP_SLOPE_WINDOW[count],
			grip_status_g->G_TAP_SLOPE_TAP_FORCE[count], grip_status_g->G_TAP_SLOPE_RELEASE_FORCE[count], 
			grip_status_g->G_TAP_DELTA_TAP_FORCE[count], grip_status_g->G_TAP_DELTA_RELEASE_FORCE[count]);
	}
	for(count=0;count < slide_num;count++){
		PRINT_INFO("SLIDE[%d], EN:%d, DIST:%d, Force:%d",
			count, grip_status_g->G_SLIDE_EN[count], grip_status_g->G_SLIDE_DIST[count], 
			grip_status_g->G_SLIDE_FORCE[count]);
	}
	for(count=0;count < swipe_num;count++){
		PRINT_INFO("SWIPE[%d], EN:%d, Velocity:%d, LEN:%d",
			count, grip_status_g->G_SWIPE_EN[count], grip_status_g->G_SWIPE_VELOCITY[count], 
			grip_status_g->G_SWIPE_LEN[count]);
	}
	mutex_unlock(&snt8100fsr_g->ap_lock);

	Wait_Wake_For_RegW();
	
	if(grip_status_g->G_EN > 0){
		grip_enable_func_noLock(grip_status_g->G_EN);
	}else{ //when en = 0, enable grip to prevent register recovery fail, and disable grip at final
		need_en = 1;
	}
	if(need_en==1){
		PRINT_INFO("Enable grip at first due to G_EN != 1");
		grip_enable_func_noLock(1);
	}
	if(grip_status_g->G_RAW_EN > 0){
		grip_raw_enable_func(grip_status_g->G_RAW_EN);
	}

	/* Squeeze Part */
	for(count=0;count < sq_num;count++){
		if(grip_status_g->G_SQUEEZE_EN[count] > 0){
			grip_squeezeX_enable_func(grip_status_g->G_SQUEEZE_EN[count], count, &SQ_BIT0[count]);
		}
		if(grip_status_g->G_SQUEEZE_SHORT[count] > 0){
			grip_squeezeX_short_dur_func(grip_status_g->G_SQUEEZE_SHORT[count], count, &SQ_BIT5[count]);
		}
		if(grip_status_g->G_SQUEEZE_LONG[count] > 0){
			grip_squeezeX_long_dur_func(grip_status_g->G_SQUEEZE_LONG[count], count, &SQ_BIT5[count]);
		}
		if(grip_status_g->G_SQUEEZE_FORCE[count] > 0){
			grip_squeezeX_force_func(grip_status_g->G_SQUEEZE_FORCE[count], count, &SQ_BIT0[count]);
		}

		if(grip_status_g->G_SQUEEZE_UP_RATE[count] > 0){
			grip_squeezeX_up_rate_func(grip_status_g->G_SQUEEZE_UP_RATE[count], count, &SQ_BIT6[count]);
		}
		if(grip_status_g->G_SQUEEZE_UP_TOTAL[count] > 0){
			grip_squeezeX_up_total_func(grip_status_g->G_SQUEEZE_UP_TOTAL[count], count, &SQ_BIT6[count]);
		}
		if(grip_status_g->G_SQUEEZE_DROP_RATE[count] > 0){
			grip_squeezeX_drop_rate_func(grip_status_g->G_SQUEEZE_DROP_RATE[count], count, &SQ_BIT7[count]);
		}
		if(grip_status_g->G_SQUEEZE_DROP_TOTAL[count] > 0){
			grip_squeezeX_drop_total_func(grip_status_g->G_SQUEEZE_DROP_TOTAL[count], count, &SQ_BIT7[count]);
		}			
	}

	
	/* Tap Part */
	for(count=0;count < tap_num;count++){
		if(grip_status_g->G_TAP_EN[count] > 0){
			grip_tapX_enable_func(grip_status_g->G_TAP_EN[count], count, &TAP_BIT0[count]);
		}
		if(grip_status_g->G_TAP_FORCE[count] > 0){
			grip_tapX_force_func(grip_status_g->G_TAP_FORCE[count], count, &TAP_BIT0[count]);
		}
		if(grip_status_g->G_TAP_MIN_POS[count] > 0){
			grip_tapX_min_position_func(grip_status_g->G_TAP_MIN_POS[count], count, &TAP_BIT2[count]);
		}
		if(grip_status_g->G_TAP_MAX_POS[count] > 0){
			grip_tapX_max_position_func(grip_status_g->G_TAP_MAX_POS[count], count, &TAP_BIT3[count]);
		}
		if(grip_status_g->G_TAP_DELTA_RELEASE_FORCE[count] > 0)
			grip_tapX_delta_release_force_func(grip_status_g->G_TAP_DELTA_RELEASE_FORCE[count], count, &TAP_BIT4[count]);
		if(grip_status_g->G_TAP_DELTA_TAP_FORCE[count] > 0)
			grip_tapX_delta_tap_force_func(grip_status_g->G_TAP_DELTA_TAP_FORCE[count], count, &TAP_BIT4[count]);
		if(grip_status_g->G_TAP_SLOPE_RELEASE_FORCE[count] > 0)
			grip_tapX_slope_release_force_func(grip_status_g->G_TAP_SLOPE_RELEASE_FORCE[count], count, &TAP_BIT5[count]);
		if(grip_status_g->G_TAP_SLOPE_TAP_FORCE[count] > 0)
			grip_tapX_slope_tap_force_func(grip_status_g->G_TAP_SLOPE_TAP_FORCE[count], count, &TAP_BIT5[count]);
		if(grip_status_g->G_TAP_SLOPE_WINDOW[count] > 0)
			grip_tapX_slope_window_func(grip_status_g->G_TAP_SLOPE_WINDOW[count], count, &TAP_BIT1[count]);
	}
	
	if(grip_status_g->G_TAP_SENSE_SET > 0){
		grip_tap_sense_enable_func(0);
		grip_tap_sense_enable_func(1);
	}

	/* Slide Part */
	for(count=0;count < slide_num;count++){
		if(grip_status_g->G_SLIDE_EN[count] > 0)
			grip_slideX_enable_func(grip_status_g->G_SLIDE_EN[count], count, &SLIDE_BIT0[count]);
		if(grip_status_g->G_SLIDE_DIST[count] > 0)
			grip_slideX_dist_func(grip_status_g->G_SLIDE_DIST[count], count, &SLIDE_BIT3[count]);
		if(grip_status_g->G_SLIDE_FORCE[count] > 0)
			grip_slideX_force_func(grip_status_g->G_SLIDE_FORCE[count], count, &SLIDE_BIT3[count]);
	}
	
	/* Swipe Part */
	for(count=0;count < swipe_num;count++){
		if(grip_status_g->G_SWIPE_EN[count] > 0)
			grip_swipeX_enable_func(grip_status_g->G_SWIPE_EN[count], count, &SWIPE_BIT0[count]);
		if(grip_status_g->G_SWIPE_VELOCITY[count] > 0)
			grip_swipeX_velocity_func(grip_status_g->G_SWIPE_VELOCITY[count], count, &SWIPE_BIT0[count]);
		if(grip_status_g->G_SWIPE_LEN[count] > 0)
			grip_swipeX_len_func(grip_status_g->G_SWIPE_LEN[count], count, &SWIPE_BIT3[count]);
	}
	
	/* Bar control, Health check and tap status*/
	PRINT_INFO("Reset check: Bar control, Health check and tap status ");
	Health_Check_Enable_No_Delay(0);
	grip_check_DPC_and_sensitivity_func();
	Grip_K_data_recovery();
	Grip_ReEnable_Squeeze_Check();
	
	if(need_en==1){
		PRINT_INFO("Disable grip at final due to G_EN != 1");
		grip_status_g->G_EN=0;
	}
	Into_DeepSleep_fun();
	grip_input_event_report(65535, 0, 0, 0, 0, 0, 0);
}

void Power_Control(int en){
	int gpio_req;
	gpio_req = of_get_named_gpio(snt8100fsr_g->dev->of_node, SNT_RST_NAME, 0);
	if(en == 0){
		g_snt_power_state = 0;
		PRINT_INFO("Set pinctl: 1V2 2V8 down");
		snt8100fsr_g->mgrip_2v8_asus_func->grip_regulator_disable();
		snt8100fsr_g->mgrip_1v2_asus_func->grip_regulator_disable();
		snt8100fsr_g->mgrip_1v8_asus_func->grip_regulator_disable();
		PRINT_INFO("Set pinctl: RST down");
		gpio_direction_output(gpio_req, 0); //output low
		msleep(50);
	}else if(en == 1){
		//disable vibrator trig pin
		if(aw8697_trig_control(1, 0)==0)
			PRINT_INFO("Disable vib trig1");
		else
			PRINT_INFO("Failed to disable vib trig1");
		if(aw8697_trig_control(2, 0)==0)
			PRINT_INFO("Disable vib trig2");
		else
			PRINT_INFO("Failed to disable vib trig2");
		
		PRINT_INFO("Set pinctl: RST down");
		gpio_request(gpio_req, "snt_rst_gpio");
		gpio_direction_output(gpio_req, 0); //output low
		msleep(5);
		if(0 == g_snt_power_state){
			snt8100fsr_g->mgrip_1v8_asus_func->grip_regulator_enable();
			snt8100fsr_g->mgrip_1v2_asus_func->grip_regulator_enable();
			snt8100fsr_g->mgrip_2v8_asus_func->grip_regulator_enable();
			msleep(5);
		}
		PRINT_INFO("Set pinctl: RST up");
		gpio_direction_output(gpio_req, 1); //output high
		g_snt_power_state = 1;
	}
}

/*************** ASUS BSP Clay: ioctl +++ *******************/
#define ASUS_GRIP_SENSOR_DATA_SIZE 3
#define ASUS_GRIP_SENSOR_D1TEST_DATA_SIZE	784
#define ASUS_GRIP_SENSOR_NAME_SIZE 32
#define ASUS_GRIP_SENSOR_IOC_MAGIC                      ('L')///< Grip sensor ioctl magic number 
#define ASUS_GRIP_SENSOR_IOCTL_ONOFF           _IOR(ASUS_GRIP_SENSOR_IOC_MAGIC, 1, int)	///< Grip sensor ioctl command - Set on/off
#define ASUS_GRIP_SENSOR_IOCTL_SET_FRAM_RATE           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 2, int)	///< Grip sensor ioctl command - Set frame rate
#define ASUS_GRIP_SENSOR_IOCTL_SET_PRESSURE_THRESHOLD           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 3, int)	///< Grip sensor ioctl command - Set pressure threshold
#define ASUS_GRIP_SENSOR_IOCTL_GET_DEBUG_MODE           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 4, int)	///< Grip sensor ioctl command - Set Debug Mode
#define ASUS_GRIP_SENSOR_IOCTL_DATA_READ           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 5, int[ASUS_GRIP_SENSOR_DATA_SIZE])	///< Grip sensor ioctl command - Data Read
#define ASUS_GRIP_SENSOR_IOCTL_MODULE_NAME           _IOR(ASUS_GRIP_SENSOR_IOC_MAGIC, 6, char[ASUS_GRIP_SENSOR_NAME_SIZE])	///< GRIP sensor ioctl command - Get module name
#define ASUS_GRIP_SENSOR_IOCTL_D1TEST_DATA_READ           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 7, unsigned char[ASUS_GRIP_SENSOR_D1TEST_DATA_SIZE])	///< Grip sensor ioctl command - D1Test Data Read
#define ASUS_GRIP_SENSOR_IOCTL_BAR0_TEST           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 8, int[2])	///< Grip sensor ioctl command - Bar Test
#define ASUS_GRIP_SENSOR_IOCTL_BAR1_TEST           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 9, int[2])	///< Grip sensor ioctl command - Bar Test
#define ASUS_GRIP_SENSOR_IOCTL_I2C_TEST           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 10, bool)	///< Grip sensor ioctl command - I2C Test
#define ASUS_GRIP_SENSOR_IOCTL_BAR0_STATUS           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 11, bool)	///< Grip sensor ioctl command - Bar Status
#define ASUS_GRIP_SENSOR_IOCTL_BAR1_STATUS           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 12, bool)	///< Grip sensor ioctl command - Bar Status
#define ASUS_GRIP_SENSOR_IOCTL_BAR_TEST_FORCE           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 13, int)	///< Grip sensor ioctl command - Bar Test Force value
#define ASUS_GRIP_SENSOR_IOCTL_BAR_TEST_TOLERANCE           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 14, int)	///< Grip sensor ioctl command - Bar Test tolerance % 
#define ASUS_GRIP_SENSOR_IOCTL_SWITCH_ONOFF           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 15, int)	///< Grip sensor ioctl command - Bar Test Force value
#define ASUS_GRIP_SENSOR_IOCTL_GET_ONOFF           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 16, bool)	///< Grip sensor ioctl command - Bar Test tolerance % 
#define ASUS_GRIP_SENSOR_IOCTL_BAR_TEST_HI_TOLERANCE           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 17, int)	///< Grip sensor ioctl command - Bar Test Hi tolerance % 
#define ASUS_GRIP_SENSOR_IOCTL_BAR_TEST_LO_TOLERANCE           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 18, int)	///< Grip sensor ioctl command - Bar Test Lo tolerance % 
#define ASUS_GRIP_SENSOR_IOCTL_BAR0_DYNAMIC_TEST           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 19, int[2])	///< Grip sensor ioctl command - Bar0 Test
#define ASUS_GRIP_SENSOR_IOCTL_BAR1_DYNAMIC_TEST           _IOW(ASUS_GRIP_SENSOR_IOC_MAGIC, 20, int[2])	///< Grip sensor ioctl command - Bar1 Test

static int SntSensor_miscOpen(struct inode *inode, struct file *file)
{
	PRINT_INFO("misc test");
#if 0
	int ret;
	if(!snt8100fsr_g){
		PRINT_CRIT("%s: null pointer, probe might not finish!", __func__);
	}
	PRINT_FUNC();
	// We don't mutex lock here, due to write_register locking
	PRINT_DEBUG("Setting frame rate to %d",
				snt8100fsr_g->suspended_frame_rate);
	ret = write_register(snt8100fsr_g,
		 			MAX_REGISTER_ADDRESS,
		 			&snt8100fsr_g->suspended_frame_rate);
	if (ret) {
		PRINT_CRIT("write_register(REGISTER_FRAME_RATE) failed");
	}

	PRINT_DEBUG("done");
#endif
	return 0;
}

static int SntSensor_miscRelease(struct inode *inode, struct file *file)
{
	//int ret;
	if(!snt8100fsr_g){
		PRINT_CRIT("%s: null pointer, probe might not finish!", __func__);
	}
	PRINT_FUNC();

	PRINT_DEBUG("done");
	return 0;
}

static int d1test_size = 784;
extern struct sc_command *sc_cmd;
extern int snt_read_sc_rsp(struct snt8100fsr *snt8100fsr);
static long SntSensor_miscIoctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0, i = 0;
	int snt_frame_rate = 0;
	int pressure_threshold = 0;
	int bar_test_result[2];
	int cur_frame_rate=50, i2c_flag = 1, grip_en_status = 0, switch_en;
	int fpc_status = 1;
	static int force_tolerance=0, force_hi_tolerance=0, force_lo_tolerance=0;
	static int force_test = 0;
	static uint16_t RegRead_t = 0;
	unsigned char d1test_ioctl[784];
	int dataSNT[ASUS_GRIP_SENSOR_DATA_SIZE];
	char nameSNT[ASUS_GRIP_SENSOR_NAME_SIZE];
	switch (cmd) {
		case ASUS_GRIP_SENSOR_IOCTL_SET_FRAM_RATE:
			ret = copy_from_user(&snt_frame_rate, (int __user*)arg, sizeof(snt_frame_rate));
			if( ret < 0) {
				PRINT_CRIT("%s: cmd = SET_FRAM_RATE, copy_from_user error(%d)\n", __func__, snt8100fsr_g->frame_rate);
				goto end;
			}
			snt8100fsr_g->frame_rate = snt_frame_rate;
			break;
		case ASUS_GRIP_SENSOR_IOCTL_SET_PRESSURE_THRESHOLD:
			ret = copy_from_user(&pressure_threshold, (int __user*)arg, sizeof(pressure_threshold));
			if( ret < 0) {
				PRINT_CRIT("%s: cmd = SET_PRESSURE_THRESHOLD, copy_from_user error(%d)\n", __func__, snt8100fsr_g->frame_rate);
				goto end;
			}
			snt8100fsr_g->pressure_threshold = pressure_threshold;
			break;
		case ASUS_GRIP_SENSOR_IOCTL_DATA_READ:
			dataSNT[0] = snt8100fsr_g->frame_rate;
			dataSNT[1] = snt8100fsr_g->pressure_threshold;
			dataSNT[2] = snt8100fsr_g->suspended_frame_rate;
			PRINT_INFO("%s: cmd = DATA_READ, data[0] = %d, data[1] = %d, data[2] = %d\n"
					 , __func__, dataSNT[0], dataSNT[1], dataSNT[2]);
			ret = copy_to_user((int __user*)arg, &dataSNT, sizeof(dataSNT));
			break;
		case ASUS_GRIP_SENSOR_IOCTL_D1TEST_DATA_READ:
			MUTEX_LOCK(&snt8100fsr_g->sb_lock);
			ret = snt_read_sc_rsp(snt8100fsr_g);
			if(log_d1test_file != NULL) {
				//Clear char array
				memset(d1test_ioctl, 0, sizeof(d1test_ioctl));
				//each sc_cmd->data[] is 4bytes
				for(i = 0; i < (d1test_size/4); i++){
					strcat(&d1test_ioctl[4*i], (unsigned char*)&sc_cmd->data[i]);
					PRINT_DEBUG("IOCTL: data[%d]: %s", i, (unsigned char*)&sc_cmd->data[i]);
				}
			}
			mutex_unlock(&snt8100fsr_g->sb_lock);
			
			ret = copy_to_user((int __user*)arg, &d1test_ioctl, sizeof(d1test_ioctl));
			PRINT_DEBUG("IOCTL: done");
			break;
		case ASUS_GRIP_SENSOR_IOCTL_MODULE_NAME:
			snprintf(nameSNT, sizeof(nameSNT), "%s", SYSFS_NAME);
			PRINT_INFO("%s: cmd = MODULE_NAME, name = %s\n", __func__, nameSNT);
			ret = copy_to_user((int __user*)arg, &nameSNT, sizeof(nameSNT));
			break;
		case ASUS_GRIP_SENSOR_IOCTL_BAR0_TEST:
			bar_test_result[0] = 0;
			bar_test_result[1] = 0;
			ret = read_register(snt8100fsr_g, REGISTER_SQUEEZE_FORCECALIB, &RegRead_t);
			RegRead_t = RegRead_t & 0x00FF;
			force_test = (int)RegRead_t;
			for (i = 0; i < snt8100fsr_g->track_reports_count; i++) {
				if(snt8100fsr_g->track_reports[i].bar_id == 0 && snt8100fsr_g->track_reports[i].force_lvl != 0){
					//center 56~166
					if(snt8100fsr_g->track_reports[i].center > 0 && snt8100fsr_g->track_reports[i].center < 9999 ){
						bar_test_result[1] = snt8100fsr_g->track_reports[i].force_lvl;
						bar_test_result[0] = check_report_force(bar_test_result[1], force_tolerance, force_test);
						print_current_report(i);
						break;
					}
				}
			}
			PRINT_INFO("Bar_0 Result = %d, Force = %d", bar_test_result[0], bar_test_result[1]);
			ret = copy_to_user((int __user*)arg, &bar_test_result, sizeof(bar_test_result));
			break;
		case ASUS_GRIP_SENSOR_IOCTL_BAR1_TEST:
			bar_test_result[0] = 0;
			bar_test_result[1] = 0;
			ret = read_register(snt8100fsr_g, REGISTER_SQUEEZE_FORCECALIB, &RegRead_t);
			RegRead_t = RegRead_t >> 8;
			force_test = (int)RegRead_t;
			for (i = 0; i < snt8100fsr_g->track_reports_count; i++) {
				if(snt8100fsr_g->track_reports[i].bar_id == 1 && snt8100fsr_g->track_reports[i].force_lvl != 0){
					//center 92~604
					if(snt8100fsr_g->track_reports[i].center > 0 && snt8100fsr_g->track_reports[i].center < 9999 ){
						bar_test_result[1] = snt8100fsr_g->track_reports[i].force_lvl;
						bar_test_result[0] = check_report_force(bar_test_result[1], force_tolerance, force_test);
						print_current_report(i);
						break;
					}
				}
			}
			PRINT_INFO("Bar_1 Result = %d, Force = %d", bar_test_result[0], bar_test_result[1]);
			ret = copy_to_user((int __user*)arg, &bar_test_result, sizeof(bar_test_result));
			break;
		case ASUS_GRIP_SENSOR_IOCTL_BAR_TEST_FORCE:
			ret = copy_from_user(&force_test, (int __user*)arg, sizeof(force_test));
			if( ret < 0) {
				PRINT_CRIT("%s: cmd = TEST_FORCE, copy_from_user error(%d)\n", __func__, force_test);
				goto end;
			}
			PRINT_INFO("set bar_test force = %d", force_test);
			break;
		case ASUS_GRIP_SENSOR_IOCTL_BAR_TEST_TOLERANCE:
			ret = copy_from_user(&force_tolerance, (int __user*)arg, sizeof(force_tolerance));
			if( ret < 0) {
				PRINT_CRIT("%s: cmd = ASUS_GRIP_SENSOR_IOCTL_BAR_TEST_TOLERANCE, copy_from_user error(%d)\n", __func__, force_tolerance);
				goto end;
			}
			PRINT_INFO("set bar_test tolerance = %d", force_tolerance);
			break;
		case ASUS_GRIP_SENSOR_IOCTL_BAR_TEST_HI_TOLERANCE:
			ret = copy_from_user(&force_hi_tolerance, (int __user*)arg, sizeof(force_hi_tolerance));
			if( ret < 0) {
				PRINT_CRIT("%s: cmd = HI_TOLERANCE, copy_from_user error(%d)\n", __func__, force_hi_tolerance);
				goto end;
			}
			PRINT_INFO("set bar_test hi tolerance = %d", force_hi_tolerance);
			break;
		case ASUS_GRIP_SENSOR_IOCTL_BAR_TEST_LO_TOLERANCE:
			ret = copy_from_user(&force_lo_tolerance, (int __user*)arg, sizeof(force_lo_tolerance));
			if( ret < 0) {
				PRINT_CRIT("%s: cmd = LO_TOLERANCE, copy_from_user error(%d)\n", __func__, force_lo_tolerance);
				goto end;
			}
			PRINT_INFO("set bar_test lo tolerance = %d", force_lo_tolerance);
			break;
			
		case ASUS_GRIP_SENSOR_IOCTL_BAR0_DYNAMIC_TEST:
			bar_test_result[0] = 0;
			bar_test_result[1] = 0;
			for (i = 0; i < snt8100fsr_g->track_reports_count; i++) {
				if(snt8100fsr_g->track_reports[i].bar_id == 0 && snt8100fsr_g->track_reports[i].force_lvl != 0){
					if(snt8100fsr_g->track_reports[i].center > 0 && snt8100fsr_g->track_reports[i].center < 9999 ){
						bar_test_result[1] = snt8100fsr_g->track_reports[i].force_lvl;
						if(bar_test_result[1]>255){
							bar_test_result[1] = 255;
						}
						bar_test_result[0] = check_report_force_trans(bar_test_result[1], force_test, force_hi_tolerance, force_lo_tolerance);
						print_current_report(i);
						break;
					}
				}
			}
			PRINT_INFO("Bar_0 Result = %d, Force = %d", bar_test_result[0], bar_test_result[1]);
			ret = copy_to_user((int __user*)arg, &bar_test_result, sizeof(bar_test_result));
			break;
		case ASUS_GRIP_SENSOR_IOCTL_BAR1_DYNAMIC_TEST:
			bar_test_result[0] = 0;
			bar_test_result[1] = 0;
			for (i = 0; i < snt8100fsr_g->track_reports_count; i++) {
				if(snt8100fsr_g->track_reports[i].bar_id == 1 && snt8100fsr_g->track_reports[i].force_lvl != 0){
					if(snt8100fsr_g->track_reports[i].center > 0 && snt8100fsr_g->track_reports[i].center < 9999 ){
						bar_test_result[1] = snt8100fsr_g->track_reports[i].force_lvl * 255;
						if(bar_test_result[1]>255){
							bar_test_result[1] = 255;
						}
						bar_test_result[0] = check_report_force_trans(bar_test_result[1], force_test, force_hi_tolerance, force_lo_tolerance);
						print_current_report(i);
						break;
					}
				}
			}
			PRINT_INFO("Bar_1 Result = %d, Force = %d", bar_test_result[0], bar_test_result[1]);
			ret = copy_to_user((int __user*)arg, &bar_test_result, sizeof(bar_test_result));
			break;
		case ASUS_GRIP_SENSOR_IOCTL_I2C_TEST:
				Wait_Wake_For_RegW();
			if(snt8100fsr_g->grip_fw_loading_status == true){
				ret = read_register (snt8100fsr_g,
					REGISTER_FRAME_RATE,
					&cur_frame_rate);
				if(ret < 0) {
					PRINT_ERR("Grip I2c no ack");	
					i2c_flag = 0;
				}
			}else{
				i2c_flag = 0;
			}
			PRINT_INFO("I2C status = %d, frame_rate=%d", i2c_flag, cur_frame_rate);
			ret = copy_to_user((int __user*)arg, &i2c_flag, sizeof(i2c_flag));
			break;
		case ASUS_GRIP_SENSOR_IOCTL_GET_ONOFF:
			grip_en_status = g_snt_power_state;
			ret = copy_to_user((int __user*)arg, &grip_en_status, sizeof(i2c_flag));
			break;
		case ASUS_GRIP_SENSOR_IOCTL_SWITCH_ONOFF:
			ret = copy_from_user(&switch_en, (int __user*)arg, sizeof(switch_en));
			if( ret < 0) {
				PRINT_CRIT("%s: cmd = ASUS_GRIP_SENSOR_IOCTL_SWITCH_ONOFF, copy_from_user error(%d)\n", __func__, switch_en);
							goto end;
			}
			if(switch_en == 0 && g_snt_power_state == 1){
				Wait_Wake_For_RegW();
			}
			Power_Control(switch_en);
			PRINT_INFO("set power = %d", switch_en);
			break;
		case ASUS_GRIP_SENSOR_IOCTL_BAR0_STATUS:	
			Wait_Wake_For_RegW();
				if(Health_Check(0x0003)!=0){
				fpc_status = 0;
				}
			ret = copy_to_user((int __user*)arg, &fpc_status, sizeof(fpc_status));
			break;
		case ASUS_GRIP_SENSOR_IOCTL_BAR1_STATUS:	
			Wait_Wake_For_RegW();
				if(Health_Check(0x003C)!=0){
				fpc_status = 0;
				}
			ret = copy_to_user((int __user*)arg, &fpc_status, sizeof(fpc_status));
			break;
		default:
			ret = -1;
			PRINT_INFO("%s: default\n", __func__);
	}
 end:
	return ret;
}

int sntSensor_miscRegister(void)
{
	int rtn = 0;
	/* in sys/class/misc/ */
	rtn = misc_register(&sentons_snt_misc);
	if (rtn < 0) {
		PRINT_CRIT("[%s] Unable to register misc deive\n", __func__);
		misc_deregister(&sentons_snt_misc);
	}
	return rtn;
}
/*************** ASUS BSP Clay: ioctl --- *******************/

/*************** ASUS BSP Clay: proc file +++ *******************/
/*+++BSP Clay proc asusGripDebug Interface+++*/
static int g_debugMode=0;
int asusGripDebug_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", g_debugMode);
	return 0;
}
int asusGripDebug_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, asusGripDebug_proc_read, NULL);
}

ssize_t asusGripDebug_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int count=0, val[2] = { -1, -1};
	char *token = (char *) kmalloc(sizeof(char), GFP_KERNEL);
	char *s = (char *) kmalloc(sizeof(char), GFP_KERNEL);
	
	Wait_Wake_For_RegW();
	if (len > 256) {
		len = 256;
	}
	
	if (copy_from_user(s, buff, len)) {
		return -EFAULT;
	}
	//strlcpy(s, messages, sizeof(len));

	do {
		token = strsep(&s, " ");
		if(token!= NULL)
			val[count] = (int)simple_strtol(token, NULL, 10);
		else
			break;
		count++;
	}while(token != NULL);

	if(count ==2){
		PRINT_INFO("val=%d, %d", val[0], val[1]);
	}else{
		PRINT_INFO("count = %d, Do nothing!", count);
	}
	
	if(token != NULL)
		kfree(token);
	if(s != NULL)
		kfree(s);
	return len;
}
void create_asusGripDebug_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = asusGripDebug_proc_open,
		.write = asusGripDebug_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/Grip_Debug_Flag", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/*---BSP Clay proc asusGripDebug Interface---*/


/*Calibration File read operation */
static int Grip_Calibration_raw_data_proc_read(struct seq_file *buf, void *v)
{
	int ret, i;

	
	MUTEX_LOCK(&snt8100fsr_g->sb_lock);

	ret = snt_read_sc_rsp(snt8100fsr_g);
	if(log_d1test_file != NULL) {
		//each sc_cmd->data[] is 4bytes
		for(i = 0; i < (d1test_size/4); i++){
			PRINT_INFO("IOCTL: data[%d]: %s", i, (unsigned char*)&sc_cmd->data[i]);	
			seq_printf(buf, "%s", (unsigned char*)&sc_cmd->data[i]);
		}
	}
	mutex_unlock(&snt8100fsr_g->sb_lock);
		
	PRINT_INFO("proc_data: done");
	
	return 0;
}

static int Grip_Calibration_raw_data_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Calibration_raw_data_proc_read, NULL);
}

void create_Grip_Calibration_raw_data_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Calibration_raw_data_proc_open,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/Grip_D1test", 0666, NULL, &proc_fops);
	if (!proc_file) {
		PRINT_ERR("%s failed!\n", __func__);
	}
	return;
}
/*Calibration File read operation*/

/* +++ BSP Clay proc i2c check +++ */
static int Grip_I2c_Check_proc_read(struct seq_file *buf, void *v)
{
	int ret, i2c_status;
	bool flag = 1;
	Wait_Wake_For_RegW();
	ret = read_register (snt8100fsr_g, REGISTER_FRAME_RATE, &i2c_status);
	if(ret < 0) {
		PRINT_ERR("Grip I2c no ack");	
		flag = 0;
		goto Report;
	}
		
Report:
	if(flag == 1){
		seq_printf(buf, "%d\n", flag);
	}else{
		seq_printf(buf, "%d\n", flag);
	}
	return 0;
}
static int Grip_I2c_Check_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_I2c_Check_proc_read, NULL);
}
void create_Grip_I2c_Check_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_I2c_Check_proc_open,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/Grip_I2c_Check", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/* --- BSP Clay proc i2c check --- */
static ssize_t Grip_ReadK_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	
	int val;
	char messages[2048];
	/*
	if(g_Charger_mode){
		PRINT_INFO("Charger mode, do nothing");
		return len;
	}
	*/
	if (len > 2048) {
		len = 2048;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);
	
	if(aw8697_trig_control(1, 1)==0)
		PRINT_INFO("Enable vib trig1");
	else
		PRINT_INFO("Failed to enable vib trig1");
	
	if(aw8697_trig_control(2, 1)==0)
		PRINT_INFO("Enable vib trig2");
	else
		PRINT_INFO("Failed to enable vib trig2");
	
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	Wait_Wake_For_RegW();
	//Grip_Chip_IRQ_EN(1);
		
	//PRINT_INFO("Str: %s, %zu bytes", messages, len);
	enable_boot_init_reg_req(snt8100fsr_g, messages, len);
	Into_DeepSleep_fun();
	mutex_unlock(&snt8100fsr_g->ap_lock);
	Grip_ReEnable_Squeeze_Check();
	return len;
}

static int Grip_ReadK_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "0\n");
	return 0;
}
static int Grip_ReadK_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_ReadK_proc_read, NULL);
}
void create_Grip_ReadK_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_ReadK_proc_open,
		.write = Grip_ReadK_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/Grip_ReadK", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

/* +++ BSP Clay proc FPC check --- */
/* +++ BSP Clay Disable wake_lock and grip event +++ */
bool wake_lock_disable_flag = 0;;
static int Grip_Disable_WakeLock_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "wake_lock_evt_flag: %d\n", wake_lock_disable_flag);
	return 0;
}

static ssize_t Grip_Disable_WakeLock_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val, ret, reg_en = 0;
	char messages[256];
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);
	wake_lock_disable_flag = val;
	if(wake_lock_disable_flag == 0){
		reg_en = 1;
	}else{
		__pm_relax(snt8100fsr_g->snt_wakelock);
	}
	Wait_Wake_For_RegW();
	ret = read_register(snt8100fsr_g, REGISTER_ENABLE, &reg_en);
	if(ret < 0) {
		PRINT_ERR("Grip register_enable write fail");
	}else{
		PRINT_INFO("reg_en = %d", reg_en);
	}	

	return len;
}

static int Grip_Disable_WakeLock_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Disable_WakeLock_proc_read, NULL);
}

void create_Grip_Disable_WakeLock_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Disable_WakeLock_proc_open,
		.write = Grip_Disable_WakeLock_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/Grip_Disable_WakeLock", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/* --- BSP Clay R/W Temp register value --- */
/* +++ BSP Clay Frame Rate setting +++ */
static int Grip_frame_proc_read(struct seq_file *buf, void *v)
{
	if(grip_status_g->G_DPC_STATUS==1)
		seq_printf(buf, "%d\n", Grip_DPC_status_g->High);
	else
		seq_printf(buf, "%d\n", snt8100fsr_g->frame_rate);
	return 0;
}
static ssize_t Grip_frame_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;
	char messages[256];
	
	memset(messages, 0, sizeof(messages));
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);
	grip_frame_rate_func(val);
	return len;
}

static int Grip_frame_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_frame_proc_read, NULL);
}

void create_Grip_frame_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_frame_proc_open,
		.write = Grip_frame_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_frame_rate", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

/* --- BSP Clay Frame Rate setting --- */
//==============Enable Interface=============//
static int Grip_raw_en_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", grip_status_g->G_RAW_EN);
	return 0;
}
static ssize_t Grip_raw_en_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;
	char messages[256];
	
	memset(messages, 0, sizeof(messages));
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);
	
	if(grip_status_g->G_RAW_EN==val){
		PRINT_INFO("repeat, skip it");
		return len;
	}
		grip_raw_enable_func(val);
	return len;
}

static int Grip_raw_en_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_raw_en_proc_read, NULL);
}

void create_Grip_raw_en_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_raw_en_proc_open,
		.write = Grip_raw_en_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_raw_en", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

/*+++ Sensor Grip Gesture +++ */
static int Grip_en_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", grip_status_g->G_EN);
	return 0;
}
static ssize_t Grip_en_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	/*
	int val;
	char messages[256];
	
	memset(messages, 0, sizeof(messages));
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);
	grip_enable_func_noLock(val);
	*/
	PRINT_INFO("Do nothing");
	return len;
}

static int Grip_en_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_en_proc_read, NULL);
}

void create_Grip_en_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_en_proc_open,
		.write = Grip_en_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_en", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

//Tap Sense Enable Interface
int Grip_Tap_Sense_En_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", grip_status_g->G_TAP_SENSE_SET);
	return 0;
}

ssize_t Grip_Tap_Sense_En_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;
	char messages[256];

	memset(messages, 0, sizeof(messages));
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);
	if(grip_status_g->G_TAP_SENSE_SET ==val){
		PRINT_INFO("repeat, skip it");
		return len;
	}
	grip_tap_sense_enable_func(val);
	return len;
}

int Grip_Tap_Sense_En_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_Sense_En_proc_read, NULL);
}

void create_Grip_Tap_Sense_En_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_Sense_En_proc_open,
		.write = Grip_Tap_Sense_En_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_sense_en", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
/* --- BSP Clay Grip Gesture --- */

//=======================================================
//=======================================================
//=======================================================
static int parse_result[2] = {-1};
static char *parse_token = NULL;
static char *string_temp = NULL;
static int Grip_node_parse_string(const char __user *buff, size_t len, int *reg_val){
	int count=0, val[2] = { -1, -1};
	int result=-1;
	char message[256];
	
	memset(message, 0, sizeof(message));
	if(parse_token == NULL){
		PRINT_DEBUG("parse_token KMALLOC MEMORY!!!!");
		parse_token = (char *) kmalloc(sizeof(char), GFP_KERNEL);
	}
	if(string_temp == NULL){
		PRINT_DEBUG("string_temp KMALLOC MEMORY!!!!");
		parse_token = (char *) kmalloc(sizeof(char), GFP_KERNEL);
	}
	
	memset(parse_result, -1, sizeof(*parse_result));
		
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(message, buff, len)) {
		result = -EFAULT;
		goto Final;
	}
	string_temp = message;
	
	do {
		if(count > 0){
			if(parse_token[0] > 57 || parse_token[0] < 48){
				PRINT_ERR("1. count=%d, parse_token=%s, string_temp=%s", count, parse_token, string_temp);
				break;
			}
		}
		
		parse_token = strsep(&string_temp, " ");
		val[count] = (int)simple_strtol(parse_token, NULL, 10);
		PRINT_DEBUG("count=%d, val[%d]=%s, parse_token=%s, string_temp=%s", count, val[count], parse_token, string_temp);
		count++;
		if(string_temp == NULL){
			PRINT_DEBUG("string_temp == NULL");
			break;
		}else if(string_temp[0] > 57 || string_temp[0] < 48){
			PRINT_ERR("2. count=%d, parse_token=%s, string_temp=%s", count, parse_token, string_temp);
			break;
		}
	}while(parse_token != NULL);
	
	if(count ==2){
		if(reg_val[val[0]]==val[1]){
			PRINT_DEBUG("repeat, skip it, input1=%d, input2=%d", val[0], val[1]);
			goto Final;
		}else
			result = 1;
		
		PRINT_DEBUG("val=%d, %d", val[0], val[1]);
		parse_result[0] = val[0];
		parse_result[1] = val[1];
	}else{
		PRINT_ERR("Error input count = %d, Do nothing!", count);
	}

Final:
	PRINT_DEBUG("Done");
	return result;
}
int Grip_Tap_En_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n", 
		grip_status_g->G_TAP_EN[0], 
		grip_status_g->G_TAP_EN[1], 
		grip_status_g->G_TAP_EN[2], 
		grip_status_g->G_TAP_EN[3]);
	return 0;
}

ssize_t Grip_Tap_En_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_EN) > 0)
		grip_tapX_enable_func(parse_result[1], parse_result[0], &TAP_BIT0[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	
	return len;
}

int Grip_Tap_En_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_En_proc_read, NULL);
}

void create_Grip_Tap_En_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_En_proc_open,
		.write = Grip_Tap_En_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_en", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Tap_Force_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n", 
		grip_status_g->G_TAP_FORCE[0], 
		grip_status_g->G_TAP_FORCE[1], 
		grip_status_g->G_TAP_FORCE[2], 
		grip_status_g->G_TAP_FORCE[3]);
	return 0;
}

ssize_t Grip_Tap_Force_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_FORCE) > 0)
		grip_tapX_force_func(parse_result[1], parse_result[0], &TAP_BIT0[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Tap_Force_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_Force_proc_read, NULL);
}

void create_Grip_Tap_Force_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_Force_proc_open,
		.write = Grip_Tap_Force_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_force", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}


int Grip_Tap_min_pos_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n", 
		grip_status_g->G_TAP_MIN_POS[0], 
		grip_status_g->G_TAP_MIN_POS[1], 
		grip_status_g->G_TAP_MIN_POS[2], 
		grip_status_g->G_TAP_MIN_POS[3]);
	return 0;
}

ssize_t Grip_Tap_min_pos_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_MIN_POS) > 0)
		grip_tapX_min_position_func(parse_result[1], parse_result[0], &TAP_BIT2[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Tap_min_pos_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_min_pos_proc_read, NULL);
}

void create_Grip_Tap_min_pos_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_min_pos_proc_open,
		.write = Grip_Tap_min_pos_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_min_position", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Tap_max_pos_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n", 
		grip_status_g->G_TAP_MAX_POS[0], 
		grip_status_g->G_TAP_MAX_POS[1], 
		grip_status_g->G_TAP_MAX_POS[2], 
		grip_status_g->G_TAP_MAX_POS[3]);
	return 0;
}

ssize_t Grip_Tap_max_pos_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_MAX_POS) > 0)
		grip_tapX_max_position_func(parse_result[1], parse_result[0], &TAP_BIT3[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Tap_max_pos_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_max_pos_proc_read, NULL);
}

void create_Grip_Tap_max_pos_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_max_pos_proc_open,
		.write = Grip_Tap_max_pos_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_max_position", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Tap_slope_window_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n", 
		grip_status_g->G_TAP_SLOPE_WINDOW[0],
		grip_status_g->G_TAP_SLOPE_WINDOW[1],
		grip_status_g->G_TAP_SLOPE_WINDOW[2],
		grip_status_g->G_TAP_SLOPE_WINDOW[3]);
	return 0;
}

ssize_t Grip_Tap_slope_window_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_SLOPE_WINDOW) > 0)
		grip_tapX_slope_window_func(parse_result[1], parse_result[0], &TAP_BIT1[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Tap_slope_window_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_slope_window_proc_read, NULL);
}

void create_Grip_Tap_slope_window_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_slope_window_proc_open,
		.write = Grip_Tap_slope_window_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_slope_window", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Tap_slope_tap_force_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n", 
		grip_status_g->G_TAP_SLOPE_TAP_FORCE[0],
		grip_status_g->G_TAP_SLOPE_TAP_FORCE[1],
		grip_status_g->G_TAP_SLOPE_TAP_FORCE[2],
		grip_status_g->G_TAP_SLOPE_TAP_FORCE[3]);
	return 0;
}

ssize_t Grip_Tap_slope_tap_force_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_SLOPE_TAP_FORCE) > 0)
		grip_tapX_slope_tap_force_func(parse_result[1], parse_result[0], &TAP_BIT5[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Tap_slope_tap_force_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_slope_tap_force_proc_read, NULL);
}

void create_Grip_Tap_slope_tap_force_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_slope_tap_force_proc_open,
		.write = Grip_Tap_slope_tap_force_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_slope_tap_force", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Tap_slope_release_force_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n", 
		grip_status_g->G_TAP_SLOPE_RELEASE_FORCE[0],
		grip_status_g->G_TAP_SLOPE_RELEASE_FORCE[1],
		grip_status_g->G_TAP_SLOPE_RELEASE_FORCE[2],
		grip_status_g->G_TAP_SLOPE_RELEASE_FORCE[3]);
	return 0;
}

ssize_t Grip_Tap_slope_release_force_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_SLOPE_RELEASE_FORCE) > 0)
		grip_tapX_slope_release_force_func(parse_result[1], parse_result[0], &TAP_BIT5[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Tap_slope_release_force_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_slope_release_force_proc_read, NULL);
}

void create_Grip_Tap_slope_release_force_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_slope_release_force_proc_open,
		.write = Grip_Tap_slope_release_force_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_slope_release_force", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Tap_delta_tap_force_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n", 
		grip_status_g->G_TAP_DELTA_TAP_FORCE[0], 
		grip_status_g->G_TAP_DELTA_TAP_FORCE[1], 
		grip_status_g->G_TAP_DELTA_TAP_FORCE[2], 
		grip_status_g->G_TAP_DELTA_TAP_FORCE[3]);
	return 0;
}

ssize_t Grip_Tap_delta_tap_force_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_DELTA_TAP_FORCE) > 0)
		grip_tapX_delta_tap_force_func(parse_result[1], parse_result[0], &TAP_BIT4[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Tap_delta_tap_force_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_delta_tap_force_proc_read, NULL);
}

void create_Grip_Tap_delta_tap_force_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_delta_tap_force_proc_open,
		.write = Grip_Tap_delta_tap_force_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_delta_tap_force", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Tap_delta_release_force_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n", 
		grip_status_g->G_TAP_DELTA_RELEASE_FORCE[0],
		grip_status_g->G_TAP_DELTA_RELEASE_FORCE[1],
		grip_status_g->G_TAP_DELTA_RELEASE_FORCE[2],
		grip_status_g->G_TAP_DELTA_RELEASE_FORCE[3]);
	return 0;
}

ssize_t Grip_Tap_delta_release_force_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_DELTA_RELEASE_FORCE) > 0)
		grip_tapX_delta_release_force_func(parse_result[1], parse_result[0], &TAP_BIT4[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Tap_delta_release_force_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_delta_release_force_proc_read, NULL);
}

void create_Grip_Tap_delta_release_force_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_delta_release_force_proc_open,
		.write = Grip_Tap_delta_release_force_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_delta_release_force", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Tap_vib_en_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n", 
		grip_status_g->G_TAP_VIB_EN[0],
		grip_status_g->G_TAP_VIB_EN[1], 
		grip_status_g->G_TAP_VIB_EN[2],
		grip_status_g->G_TAP_VIB_EN[3]);
	return 0;
}

ssize_t Grip_Tap_vib_en_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_TAP_VIB_EN) > 0)
		grip_tapX_vibrator_enable_func(parse_result[1], parse_result[0]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Tap_vib_en_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Tap_vib_en_proc_read, NULL);
}

void create_Grip_Tap_vib_en_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Tap_vib_en_proc_open,
		.write = Grip_Tap_vib_en_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_tap_vib_en", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

//============================================================
//=======================TAP Part Done!!!==========================
//============================================================
int Grip_Squeeze_en_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n", 
		grip_status_g->G_SQUEEZE_EN[0],
		grip_status_g->G_SQUEEZE_EN[1]);
	return 0;
}

ssize_t Grip_Squeeze_en_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SQUEEZE_EN) > 0)
		grip_squeezeX_enable_func(parse_result[1], parse_result[0], &SQ_BIT0[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Squeeze_en_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Squeeze_en_proc_read, NULL);
}

void create_Grip_Squeeze_en_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Squeeze_en_proc_open,
		.write = Grip_Squeeze_en_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_squeeze_en", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Squeeze_force_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n", 
		grip_status_g->G_SQUEEZE_FORCE[0],
		grip_status_g->G_SQUEEZE_FORCE[1]);
	return 0;
}

ssize_t Grip_Squeeze_force_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SQUEEZE_FORCE) > 0)
		grip_squeezeX_force_func(parse_result[1], parse_result[0], &SQ_BIT0[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Squeeze_force_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Squeeze_force_proc_read, NULL);
}

void create_Grip_Squeeze_force_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Squeeze_force_proc_open,
		.write = Grip_Squeeze_force_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_squeeze_force", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Squeeze_short_dur_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n", 
		grip_status_g->G_SQUEEZE_SHORT[0],
		grip_status_g->G_SQUEEZE_SHORT[1]);
	return 0;
}

ssize_t Grip_Squeeze_short_dur_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SQUEEZE_SHORT) > 0)
		grip_squeezeX_short_dur_func(parse_result[1], parse_result[0], &SQ_BIT5[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Squeeze_short_dur_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Squeeze_short_dur_proc_read, NULL);
}

void create_Grip_Squeeze_short_dur_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Squeeze_short_dur_proc_open,
		.write = Grip_Squeeze_short_dur_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_squeeze_short_dur", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Squeeze_long_dur_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n", 
		grip_status_g->G_SQUEEZE_LONG[0],
		grip_status_g->G_SQUEEZE_LONG[1]);
	return 0;
}

ssize_t Grip_Squeeze_long_dur_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SQUEEZE_LONG) > 0)
		grip_squeezeX_long_dur_func(parse_result[1], parse_result[0], &SQ_BIT5[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Squeeze_long_dur_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Squeeze_long_dur_proc_read, NULL);
}

void create_Grip_Squeeze_long_dur_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Squeeze_long_dur_proc_open,
		.write = Grip_Squeeze_long_dur_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_squeeze_long_dur", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Squeeze_drop_rate_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n", 
		grip_status_g->G_SQUEEZE_DROP_RATE[0],
		grip_status_g->G_SQUEEZE_DROP_RATE[1]);
	return 0;
}

ssize_t Grip_Squeeze_drop_rate_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SQUEEZE_DROP_RATE) > 0)
		grip_squeezeX_drop_rate_func(parse_result[1], parse_result[0], &SQ_BIT7[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Squeeze_drop_rate_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Squeeze_drop_rate_proc_read, NULL);
}

void create_Grip_Squeeze_drop_rate_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Squeeze_drop_rate_proc_open,
		.write = Grip_Squeeze_drop_rate_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_squeeze_drop_rate", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Squeeze_drop_total_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n", 
		grip_status_g->G_SQUEEZE_DROP_TOTAL[0],
		grip_status_g->G_SQUEEZE_DROP_TOTAL[1]);
	return 0;
}

ssize_t Grip_Squeeze_drop_total_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SQUEEZE_DROP_TOTAL) > 0)
		grip_squeezeX_drop_total_func(parse_result[1], parse_result[0], &SQ_BIT7[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Squeeze_drop_total_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Squeeze_drop_total_proc_read, NULL);
}

void create_Grip_Squeeze_drop_total_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Squeeze_drop_total_proc_open,
		.write = Grip_Squeeze_drop_total_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_squeeze_drop_total", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Squeeze_up_rate_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n", 
		grip_status_g->G_SQUEEZE_UP_RATE[0],
		grip_status_g->G_SQUEEZE_UP_RATE[1]);
	return 0;
}

ssize_t Grip_Squeeze_up_rate_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SQUEEZE_UP_RATE) > 0)
		grip_squeezeX_up_rate_func(parse_result[1], parse_result[0], &SQ_BIT6[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Squeeze_up_rate_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Squeeze_up_rate_proc_read, NULL);
}

void create_Grip_Squeeze_up_rate_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Squeeze_up_rate_proc_open,
		.write = Grip_Squeeze_up_rate_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_squeeze_up_rate", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Squeeze_up_total_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n", 
		grip_status_g->G_SQUEEZE_UP_TOTAL[0],
		grip_status_g->G_SQUEEZE_UP_TOTAL[1]);
	return 0;
}

ssize_t Grip_Squeeze_up_total_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SQUEEZE_UP_TOTAL) > 0)
		grip_squeezeX_up_total_func(parse_result[1], parse_result[0], &SQ_BIT6[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Squeeze_up_total_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Squeeze_up_total_proc_read, NULL);
}

void create_Grip_Squeeze_up_total_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Squeeze_up_total_proc_open,
		.write = Grip_Squeeze_up_total_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_squeeze_up_total", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
//============================================================
//=======================SQUEEZE Part Done!!!==========================
//============================================================
int Grip_Slide_en_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n", 
		grip_status_g->G_SLIDE_EN[0],
		grip_status_g->G_SLIDE_EN[1]);
	return 0;
}

ssize_t Grip_Slide_en_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SLIDE_EN) > 0)
		grip_slideX_enable_func(parse_result[1], parse_result[0], &SLIDE_BIT0[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Slide_en_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Slide_en_proc_read, NULL);
}

void create_Grip_Slide_en_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Slide_en_proc_open,
		.write = Grip_Slide_en_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_slide_en", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Slide_dist_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n", 
		grip_status_g->G_SLIDE_DIST[0],
		grip_status_g->G_SLIDE_DIST[1]);
	return 0;
}

ssize_t Grip_Slide_dist_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SLIDE_DIST) > 0)
		grip_slideX_dist_func(parse_result[1], parse_result[0], &SLIDE_BIT3[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	
	return len;
}

int Grip_Slide_dist_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Slide_dist_proc_read, NULL);
}

void create_Grip_Slide_dist_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Slide_dist_proc_open,
		.write = Grip_Slide_dist_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_slide_dist", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Slide_2nd_dist_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n", 
		grip_status_g->G_SLIDE_2ND_DIST[0],
		grip_status_g->G_SLIDE_2ND_DIST[1]);
	return 0;
}

ssize_t Grip_Slide_2nd_dist_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SLIDE_2ND_DIST) > 0)
		grip_slideX_2nd_dist_func(parse_result[1], parse_result[0], &SLIDE_BIT4[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	
	return len;
}

int Grip_Slide_2nd_dist_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Slide_2nd_dist_proc_read, NULL);
}

void create_Grip_Slide_2nd_dist_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Slide_2nd_dist_proc_open,
		.write = Grip_Slide_2nd_dist_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_slide_2nd_dist", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Slide_force_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n", 
		grip_status_g->G_SLIDE_FORCE[0],
		grip_status_g->G_SLIDE_FORCE[1]);
	return 0;
}

ssize_t Grip_Slide_force_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SLIDE_FORCE) > 0)
		grip_slideX_force_func(parse_result[1], parse_result[0], &SLIDE_BIT3[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	
	return len;
}

int Grip_Slide_force_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Slide_force_proc_read, NULL);
}

void create_Grip_Slide_force_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Slide_force_proc_open,
		.write = Grip_Slide_force_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_slide_force", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Slide_min_pos_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n", 
		grip_status_g->G_SLIDE_MIN_POS[0],
		grip_status_g->G_SLIDE_MIN_POS[1]);
	return 0;
}

ssize_t Grip_Slide_min_pos_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SLIDE_MIN_POS) > 0)
		grip_slideX_min_position_func(parse_result[1], parse_result[0], &SLIDE_BIT1[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	
	return len;
}

int Grip_Slide_min_pos_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Slide_min_pos_proc_read, NULL);
}

void create_Grip_Slide_min_pos_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Slide_min_pos_proc_open,
		.write = Grip_Slide_min_pos_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_slide_min_pos", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Slide_max_pos_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n", 
		grip_status_g->G_SLIDE_MAX_POS[0],
		grip_status_g->G_SLIDE_MAX_POS[1]);
	return 0;
}

ssize_t Grip_Slide_max_pos_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SLIDE_MAX_POS) > 0)
		grip_slideX_max_position_func(parse_result[1], parse_result[0], &SLIDE_BIT2[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	
	return len;
}

int Grip_Slide_max_pos_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Slide_max_pos_proc_read, NULL);
}

void create_Grip_Slide_max_pos_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Slide_max_pos_proc_open,
		.write = Grip_Slide_max_pos_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_slide_max_pos", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Slide_vib_en_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n", 
		grip_status_g->G_SLIDE_VIB_EN[0],
		grip_status_g->G_SLIDE_VIB_EN[1]);
	return 0;
}

ssize_t Grip_Slide_vib_en_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SLIDE_VIB_EN) > 0)
		grip_slideX_vibrator_enable_func(parse_result[1], parse_result[0]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Slide_vib_en_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Slide_vib_en_proc_read, NULL);
}

void create_Grip_Slide_vib_en_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Slide_vib_en_proc_open,
		.write = Grip_Slide_vib_en_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_slide_vib_en", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Slide_tap_priority_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d,%d,%d\n", 
		grip_status_g->G_SLIDE_TAP_PRIORITY[0],
		grip_status_g->G_SLIDE_TAP_PRIORITY[1]);
	return 0;
}

ssize_t Grip_Slide_tap_priority_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SLIDE_TAP_PRIORITY) > 0)
		grip_slideX_tap_priority_func(parse_result[1], parse_result[0], &SLIDE_BIT0[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Slide_tap_priority_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Slide_tap_priority_proc_read, NULL);
}

void create_Grip_Slide_tap_priority_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Slide_tap_priority_proc_open,
		.write = Grip_Slide_tap_priority_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_slide_tap_priority", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

//============================================================
//=======================SLIDE Part Done!!!==========================
//============================================================

int Grip_Swipe_en_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n", 
		grip_status_g->G_SWIPE_EN[0],
		grip_status_g->G_SWIPE_EN[1]);
	return 0;
}

ssize_t Grip_Swipe_en_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SWIPE_EN) > 0)
		grip_swipeX_enable_func(parse_result[1], parse_result[0], &SWIPE_BIT0[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Swipe_en_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Swipe_en_proc_read, NULL);
}

void create_Grip_Swipe_en_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Swipe_en_proc_open,
		.write = Grip_Swipe_en_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_swipe_en", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
int Grip_Swipe_velocity_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n", 
		grip_status_g->G_SWIPE_VELOCITY[0],
		grip_status_g->G_SWIPE_VELOCITY[1]);
	return 0;
}

ssize_t Grip_Swipe_velocity_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SWIPE_VELOCITY) > 0)
		grip_swipeX_velocity_func(parse_result[1], parse_result[0], &SWIPE_BIT0[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Swipe_velocity_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Swipe_velocity_proc_read, NULL);
}

void create_Grip_Swipe_velocity_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Swipe_velocity_proc_open,
		.write = Grip_Swipe_velocity_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_swipe_v", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Swipe_len_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n", 
		grip_status_g->G_SWIPE_LEN[0],
		grip_status_g->G_SWIPE_LEN[1]);
	return 0;
}

ssize_t Grip_Swipe_len_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SWIPE_LEN) > 0)
		grip_swipeX_len_func(parse_result[1], parse_result[0], &SWIPE_BIT3[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Swipe_len_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Swipe_len_proc_read, NULL);
}

void create_Grip_Swipe_len_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Swipe_len_proc_open,
		.write = Grip_Swipe_len_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_swipe_len", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Swipe_min_pos_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n", 
		grip_status_g->G_SWIPE_MIN_POS[0],
		grip_status_g->G_SWIPE_MIN_POS[1]);
	return 0;
}

ssize_t Grip_Swipe_min_pos_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SWIPE_MIN_POS) > 0)
		grip_swipeX_min_position_func(parse_result[1], parse_result[0], &SWIPE_BIT1[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Swipe_min_pos_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Swipe_min_pos_proc_read, NULL);
}

void create_Grip_Swipe_min_pos_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Swipe_min_pos_proc_open,
		.write = Grip_Swipe_min_pos_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_swipe_min_pos", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

int Grip_Swipe_max_pos_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d,%d\n", 
		grip_status_g->G_SWIPE_MAX_POS[0],
		grip_status_g->G_SWIPE_MAX_POS[1]);
	return 0;
}

ssize_t Grip_Swipe_max_pos_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	if(Grip_node_parse_string(buff, len, grip_status_g->G_SWIPE_MAX_POS) > 0)
		grip_swipeX_max_position_func(parse_result[1], parse_result[0], &SWIPE_BIT2[parse_result[0]]);
	else
		PRINT_DEBUG("Do Nothing");
	return len;
}

int Grip_Swipe_max_pos_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Swipe_max_pos_proc_read, NULL);
}

void create_Grip_Swipe_max_pos_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Swipe_max_pos_proc_open,
		.write = Grip_Swipe_max_pos_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/grip_swipe_max_pos", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
//============================================================
//=======================SWIPE Part Done!!!==========================
//============================================================

//==========Gesture Thershold Interface=======//
int fw_version = 0;
static char *product_string;
static int Grip_FW_RESULT_proc_read(struct seq_file *buf, void *v)
{
	if(snt8100fsr_g->grip_fw_loading_status == true){
		seq_printf(buf, "0xffff\n");
	}else{
		seq_printf(buf, "0x0\n");
	}
	return 0;
}

static ssize_t Grip_FW_RESULT_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;
	char messages[256];
	
	memset(messages, 0, sizeof(messages));
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);
	return len;
}

static int Grip_FW_RESULT_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_FW_RESULT_proc_read, NULL);
}

void create_Grip_FW_RESULT_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_FW_RESULT_proc_open,
		.write = Grip_FW_RESULT_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = 
	proc_create("driver/grip_fw_result", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
static int Grip_FW_VER_proc_read(struct seq_file *buf, void *v)
{
	int ret=0;
	if(snt8100fsr_g->grip_fw_loading_status == false){
		seq_printf(buf, "0x0\n");
		return -1;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	if(snt8100fsr_g->grip_fw_loading_status == true){
		Wait_Wake_For_RegW();
		//Grip_Chip_IRQ_EN(1);
		
		product_string = memory_allocate(PRODUCT_CONFIG_MAX_LEN, 0);
		if (product_string == NULL) {
			PRINT_CRIT("memory_allocate(PRODUCT_CONFIG_MAX_LEN) failed");
			mutex_unlock(&snt8100fsr_g->ap_lock);
			return -1;
		}

		ret = read_product_config(snt8100fsr_g, product_string);
		seq_printf(buf, "%s\n",product_string);
		memory_free(product_string);
		if (ret) {
			PRINT_WARN("Unable to read product config");
		}
		Into_DeepSleep_fun();	
	}else{
		seq_printf(buf, "0x0\n");
	}
	mutex_unlock(&snt8100fsr_g->ap_lock);
	return 0;
}

static ssize_t Grip_FW_VER_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;
	char messages[256];
	
	memset(messages, 0, sizeof(messages));
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);
	fw_version = val;
	return len;
}

static int Grip_FW_VER_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_FW_VER_proc_read, NULL);
}

void create_Grip_FW_VER_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_FW_VER_proc_open,
		.write = Grip_FW_VER_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = 
	proc_create("driver/grip_fw_ver", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}

static int Grip_set_power_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "Grip 1V2_2V8 status: %d\n", g_snt_power_state);
	return 0;
}

static ssize_t Grip_set_power_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;
	char messages[256];
	
	memset(messages, 0, sizeof(messages));
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);
	if(val == 1){
		snt8100fsr_g->mgrip_2v8_asus_func->grip_regulator_disable();
	}else{
		snt8100fsr_g->mgrip_2v8_asus_func->grip_regulator_enable();
		
	}
	PRINT_INFO("set power = %d", val);
	return len;
}

static int Grip_set_power_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_set_power_proc_read, NULL);
}

void create_Grip_set_power_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_set_power_proc_open,
		.write = Grip_set_power_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = 
	proc_create("driver/grip_set_power", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
#ifdef ASUS_ZS673KS_PROJECT
static void Grip_Read_Calibration_Data(struct seq_file *buf, int sleep_time){
	uint16_t cfg_bank_sync1[3] = {0x0000,0x0004,0x0e02};
	uint16_t cfg_bank_dataA[2] = {0x011a,0x0000};
	uint16_t cfg_bank_dataB[2] = {0x011b,0x0000};
	uint16_t cfg_bank_dataC[2] = {0x011c,0x0000};
	uint16_t cfg_bank_sync2[3] = {0x0000,0x0000,0x0e04};
	uint16_t cfg_bank_read[3] = {0x0000,0x0000,0x0e01};
	int block_len = 3;
	int partition_len = 2;
	int read_dataA_len = 16, read_dataB_len = 9, read_dataC_len = 9;
	int i = 0;
	uint16_t string[20];
	uint16_t string1[20];
	uint16_t string2[20];
	memset(string, 0, sizeof(string));
	cust_write_registers(snt8100fsr_g,0x2c, block_len, cfg_bank_sync1);
	msleep(sleep_time);
	cust_write_registers(snt8100fsr_g,0x200, partition_len, cfg_bank_dataA);
	msleep(sleep_time);
	cust_write_registers(snt8100fsr_g,0x2c, block_len, cfg_bank_sync2);
	msleep(sleep_time);

	cust_write_registers(snt8100fsr_g,0x2c, block_len, cfg_bank_read);
	msleep(sleep_time);
	cust_read_registers(snt8100fsr_g,0x200, 1, string);
	msleep(sleep_time);
	cust_read_registers(snt8100fsr_g,0x200, read_dataA_len, string);
	msleep(sleep_time);
	
	seq_printf(buf, "11a:");
	for(i = 0; i < read_dataA_len; i++){
		seq_printf(buf, "0x%x,",string[i]);
		PRINT_INFO("reg_val=0x%x", string[i]);
		if(i == (read_dataA_len/2)){
			seq_printf(buf, "&");
		}
	}
	seq_printf(buf, "&11c");
	memset(string1, 0, sizeof(string1));
	cust_write_registers(snt8100fsr_g,0x2c, block_len, cfg_bank_sync1);
	msleep(sleep_time);
	cust_write_registers(snt8100fsr_g,0x200, partition_len, cfg_bank_dataC);
	msleep(sleep_time);
	cust_write_registers(snt8100fsr_g,0x2c, block_len, cfg_bank_sync2);
	msleep(sleep_time);
	
	cust_write_registers(snt8100fsr_g,0x2c, block_len, cfg_bank_read);
	msleep(sleep_time);
	cust_read_registers(snt8100fsr_g,0x200, 1, string1);
	msleep(sleep_time);
	cust_read_registers(snt8100fsr_g,0x200, read_dataC_len, string1);
	msleep(sleep_time);

	for(i = 0; i < read_dataB_len; i++){
		seq_printf(buf, "0x%x,",string1[i]);
		PRINT_INFO("reg_val=0x%x", string1[i]);
	}
	seq_printf(buf, "&11b");

	memset(string2, 0, sizeof(string2));
	cust_write_registers(snt8100fsr_g,0x2c, block_len, cfg_bank_sync1);
	msleep(sleep_time);
	cust_write_registers(snt8100fsr_g,0x200, partition_len, cfg_bank_dataB);
	msleep(sleep_time);
	cust_write_registers(snt8100fsr_g,0x2c, block_len, cfg_bank_sync2);
	msleep(sleep_time);
	
	cust_write_registers(snt8100fsr_g,0x2c, block_len, cfg_bank_read);
	msleep(sleep_time);
	cust_read_registers(snt8100fsr_g,0x200, 1, string2);
	msleep(sleep_time);
	cust_read_registers(snt8100fsr_g,0x200, read_dataB_len, string2);
	msleep(sleep_time);

	for(i = 0; i < read_dataC_len; i++){
		seq_printf(buf, "0x%x,",string2[i]);
		PRINT_INFO("reg_val=0x%x", string2[i]);
	}
}

static int Grip_Cal_Read_proc_read(struct seq_file *buf, void *v)
{
	if(snt8100fsr_g->grip_fw_loading_status == false){
		seq_printf(buf, "0x0\n");
		return -1;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	if(snt8100fsr_g->grip_fw_loading_status == true){
		Wait_Wake_For_RegW();
		PRINT_INFO("call sleep1");
		Grip_Read_Calibration_Data(buf,1);
		Into_DeepSleep_fun();
	}else{
		seq_printf(buf, "fw_loading fail\n");
	}
	mutex_unlock(&snt8100fsr_g->ap_lock);
	return 0;
}

static int Grip_Cal_Read_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, Grip_Cal_Read_proc_read, NULL);
}

void create_Grip_Cal_Read_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open = Grip_Cal_Read_proc_open,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = 
	proc_create("driver/grip_cal_read", 0666, NULL, &proc_fops);

	if (!proc_file) {
		PRINT_CRIT("[Proc]%s failed!\n", __FUNCTION__);
	}
	return;
}
#endif
/*************** ASUS BSP Clay: proc file --- *******************/

