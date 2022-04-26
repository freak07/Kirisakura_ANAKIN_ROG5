/*****************************************************************************
* File: event.c
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
*****************************************************************************/
#include <linux/kernel.h>

#include "config.h"
#include "serial_bus.h"
#include "workqueue.h"
#include "hardware.h"
#include "irq.h"
#include "memory.h"
#include "file.h"
#include "event.h"
#include "track_report.h"
#include "customize.h"
#include "sonacomm.h"
#include "utils.h"
#include "device.h"
#include "debug.h"
#include "crc.h"
#include "file_control.h"
#include "locking.h"

#include <linux/timer.h>
#include <linux/delay.h>
/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/
#define MAX_FRAME_BUFFER_SIZE    (16 * 1024)
#define FRAME_LENGTH_WIDTH       (4)
#define FRAME_BUFFER_SIZE        (FRAME_LENGTH_WIDTH + MAX_FRAME_BUFFER_SIZE)

// I2C FWUpdate
#define FW_BUFFER_SIZE         256
#define FWImageTrailerSize      16


/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/
//static struct delayed_work g_dwork;
struct event_work {
	struct delayed_work my_work;
	struct snt8100fsr *snt8100fsr;
	bool context_fwd;
};

#if USE_TRIG_IRQ
struct trig_work_s {
	struct delayed_work my_work;
	struct snt8100fsr *snt8100fsr;
	int id;
};
#endif

/*==========================================================================*/
/* LOCAL PROTOTYPES                                                         */
/*==========================================================================*/
static irqreturn_t irq_handler_top(int irq, void *dev);
#if USE_TRIG0_IRQ
static irqreturn_t trig_handler_top0(int irq, void *dev);
static void trig_wq_func0(struct work_struct *work);
#endif
#if USE_TRIG1_IRQ
static irqreturn_t trig_handler_top1(int irq, void *dev);
static void trig_wq_func1(struct work_struct *work);
#endif
#if USE_TRIG2_IRQ
static irqreturn_t trig_handler_top2(int irq, void *dev);
static void trig_wq_func2(struct work_struct *work);
#endif
#if USE_TRIG3_IRQ
static irqreturn_t trig_handler_top3(int irq, void *dev);
static void trig_wq_func3(struct work_struct *work);
#endif
static void event_wq_func(struct work_struct *lwork);
static void read_track_reports(struct snt8100fsr *sb_dev);
static void log_track_report(uint16_t frame, struct track_report *tr, size_t count);
static void log_track_report_bin(uint16_t frame, struct track_report *tr, size_t count);
static void log_no_touch_frame(struct snt8100fsr *snt8100fsr);
static void log_d1test(struct snt8100fsr *snt8100fsr);
static void log_frame_cmd(struct snt8100fsr *snt8100fsr);
static void log_frame_stream(struct snt8100fsr *snt8100fsr);
static void set_sys_param_rsp(struct snt8100fsr *snt8100fsr);
static void get_sys_param_rsp(struct snt8100fsr *snt8100fsr);

#ifdef SUPPORT_FLASH
static void fwupdate_rsp_ind(struct snt8100fsr *snt8100fsr);
#endif
static void write_flash_reg_part_rsp(struct snt8100fsr *snt8100fsr);
static void spurious_cmd_event(struct snt8100fsr *snt8100fsr);

static int init_after_reset(struct snt8100fsr *snt8100fsr);
static void Grip_check_K_and_frame(void);
void sc_flush(struct snt8100fsr *snt8100fsr);

/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/
static struct event_work *work_g;

struct file* log_track_reports_file = NULL;
static int log_track_reports_file_offset;
struct file* log_track_reports_bin_file = NULL;
static int log_track_reports_bin_file_offset;
struct file* log_d1test_file = NULL;
static int log_d1test_file_offset;
static int log_d1test_num_frames;
struct file* log_frame_file = NULL;
static int log_frame_file_offset;
struct file* log_no_touch_frame_file = NULL;
static int log_no_touch_frame_file_offset;
struct sc_command *sc_cmd;
static uint8_t* frame_buffer;
static uint32_t frame_size;

/*==========================================================================*/
/* ASUS METHODS                                                                  */
/*==========================================================================*/
static int finish_boot = 0;
int G_grip_tap_vib1_reg = 0x300; //Trigger 3
int G_grip_tap_vib2_reg = 0x500; //Trigger 5
int GRIP_TAP_LINK_VIBRATOR = 0x303; //Trigger 1 & 2 => Trigger 3
extern int Health_Check(uint16_t val);
void asus_init_state(void){
	int ret;
	struct register_enable reg_enable;
	uint16_t Raw_data; 
	uint16_t i2c_addr = 0x4C2C;
	
	int j=0, tap_algo_num=4, slide_algo_num=2, swipe_algo_num=2, squeeze_algo_num=2;
	uint16_t trig3_puls_dur = 0xb; //the pulse width bigger than 1us
	uint16_t trig3_control = 0x0200; //magic number from sentons
	uint16_t trig5_puls_dur = 0xb; //the pulse width bigger than 1us
	uint16_t trig5_control = 0x0002; //magic number from sentons
	
	reg_enable.padding = 0;
	Health_Check(0xffff);

	reg_enable.enable = 1;
	Raw_data = 8;
	
	PRINT_INFO("Set tap gesture ");
	write_register(snt8100fsr_g, REGISTER_TIRGGER3_PULSE_DUR, &trig3_puls_dur);
	write_register(snt8100fsr_g, REGISTER_TIRGGER3_CONTROL, &trig3_control);
	write_register(snt8100fsr_g, REGISTER_TIRGGER_LINK1, &G_grip_tap_vib1_reg);
	write_register(snt8100fsr_g, REGISTER_TIRGGER5_PULSE_DUR, &trig5_puls_dur);
	write_register(snt8100fsr_g, REGISTER_TIRGGER5_CONTROL, &trig5_control);
	write_register(snt8100fsr_g, REGISTER_TIRGGER_LINK2, &G_grip_tap_vib2_reg);
	
	for(j=0;j<tap_algo_num;j++){
		set_tap_gesture(j, TAP_BIT0[j], 0);
		set_tap_gesture(j, TAP_BIT1[j], 1);
		set_tap_gesture(j, TAP_BIT2[j], 2);
		set_tap_gesture(j, TAP_BIT3[j], 3);
		set_tap_gesture(j, TAP_BIT4[j], 4);
		set_tap_gesture(j, TAP_BIT5[j], 5);
		set_tap_gesture(j, TAP_BIT6[j], 6);
		set_tap_gesture(j, TAP_BIT7[j], 7);
	}
	/*
	set_tap_gesture(2, TAP2_BIT0, 0);
	set_tap_gesture(2, TAP2_BIT1, 1);
	set_tap_gesture(2, TAP2_BIT2, 2);
	set_tap_gesture(2, TAP2_BIT3, 3);
	set_tap_gesture(2, TAP2_BIT4, 4);
	set_tap_gesture(2, TAP2_BIT5, 5);
	set_tap_gesture(2, TAP2_BIT6, 6);
	set_tap_gesture(2, TAP2_BIT7, 7);
	*/
	//get_tap_gesture(0, TAP0_BIT0, 0, 17);

	PRINT_INFO("Set sq gesture ");

	for(j=0;j<squeeze_algo_num;j++){
		set_sq_gesture(j, SQ_BIT0[j], 0);
		set_sq_gesture(j, SQ_BIT1[j], 1);
		set_sq_gesture(j, SQ_BIT2[j], 2);
		set_sq_gesture(j, SQ_BIT3[j], 3);
		set_sq_gesture(j, SQ_BIT4[j], 4);
		set_sq_gesture(j, SQ_BIT5[j], 5);
		set_sq_gesture(j, SQ_BIT6[j], 6);
		set_sq_gesture(j, SQ_BIT7[j], 7);
		set_sq_gesture(j, SQ_BIT8[j], 8);
	}
	//get_sq_gesture(0, SQ1_BIT0, 0, 10);
	//get_sq_gesture(0, SQ1_BIT0, 0, 20);
	
	PRINT_INFO("Set slide gesture ");
	for(j=0;j<slide_algo_num;j++){
		set_slide_gesture(j, SLIDE_BIT0[j], 0);
		set_slide_gesture(j, SLIDE_BIT1[j], 1);
		set_slide_gesture(j, SLIDE_BIT2[j], 2);
		set_slide_gesture(j, SLIDE_BIT3[j], 3);
		set_slide_gesture(j, SLIDE_BIT4[j], 4);
		set_slide_gesture(j, SLIDE_BIT5[j], 5);
	}

	PRINT_INFO("Set swipe gesture ");
	for(j=0;j<swipe_algo_num;j++){
		set_swipe_gesture(j, SWIPE_BIT0[j], 0);
		set_swipe_gesture(j, SWIPE_BIT1[j], 1);
		set_swipe_gesture(j, SWIPE_BIT2[j], 2);
		set_swipe_gesture(j, SWIPE_BIT3[j], 3);
		set_swipe_gesture(j, SWIPE_BIT4[j], 4);
	}
	
	PRINT_INFO("Touch status:0x%x ", reg_enable.enable);
	ret = write_register(snt8100fsr_g, REGISTER_ENABLE, &reg_enable);
	if (ret) {
		PRINT_CRIT("sb_write_register() failed");
		return;
	}
	PRINT_INFO("set raw data = 0x%x", Raw_data);
	ret = write_register(snt8100fsr_g, REGISTR_RAW_DATA, &Raw_data);
	PRINT_INFO("select i2c wake addr=%x, Slave I2C addr=%x", i2c_addr>>8, i2c_addr&0x00FF);
	ret = write_register(snt8100fsr_g, REGISTER_I2C_CFG, &i2c_addr);
	if (ret) {
		PRINT_CRIT("sb_write_register() failed");
		return;
	}
}
/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/
int stop_event_processing(struct snt8100fsr *snt8100fsr) {
	struct register_enable reg_enable;
	int ret;
	PRINT_FUNC();
	// Disable IRQ processing
	irq_handler_unregister(&snt_irq_db);

	#if USE_TRIG0_IRQ
	irq_handler_unregister(&trig0_irq_db);
	#endif

	#if USE_TRIG1_IRQ
	irq_handler_unregister(&trig1_irq_db);
	#endif

	#if USE_TRIG2_IRQ
	irq_handler_unregister(&trig2_irq_db);
	#endif

	#if USE_TRIG3_IRQ
	irq_handler_unregister(&trig3_irq_db);
	#endif

	// Clear the enable bit to stop processing touch reports
	reg_enable.enable = 0;
	reg_enable.padding = 0;
	ret = write_register(snt8100fsr, REGISTER_ENABLE, &reg_enable);
	if (ret) {
		PRINT_CRIT("write_register() failed");
	}

	// Free work object
	if (work_g) {
		PRINT_INFO("freeing IRQ workq");
		workqueue_free_work(work_g);
		work_g = NULL;
	}

	#if USE_TRIG0_IRQ
	if (trig0_irq_db.work) {
		PRINT_INFO("freeing TRIG0 workq");
		workqueue_free_work(trig0_irq_db.work);
		trig0_irq_db.work = NULL;
	}
	#endif

	#if USE_TRIG1_IRQ
	if (trig1_irq_db.work) {
		PRINT_INFO("freeing TRIG1 workq");
		workqueue_free_work(trig1_irq_db.work);
		trig1_irq_db.work = NULL;
	}
	#endif

	#if USE_TRIG2_IRQ
	if (trig2_irq_db.work) {
		PRINT_INFO("freeing TRIG2 workq");
		workqueue_free_work(trig2_irq_db.work);
		trig2_irq_db.work = NULL;
	}
	#endif

	#if USE_TRIG3_IRQ
	if (trig3_irq_db.work) {
		PRINT_INFO("freeing TRIG3_workq");
		workqueue_free_work(trig3_irq_db.work);
		trig3_irq_db.work = NULL;
	}
	#endif

	// Free memory for saving last received track reports
	if (snt8100fsr->track_reports) {
		memory_free(snt8100fsr->track_reports);
		snt8100fsr->track_reports = NULL;
	}

	return 0;
}

struct delayed_work event_wq;
struct delayed_work irq_wq0;
struct delayed_work irq_wq1;
struct delayed_work irq_wq2;
struct delayed_work irq_wq3;

int retry_cal_read = 1;
int start_event_processing(struct snt8100fsr *snt8100fsr) {
	char *product_string;
	struct register_event reg_event;
	// [dy] Always declare register_enable
	struct register_enable reg_enable;

	int ret;

	snt8100fsr->active_sc_cmd = mc_no_command;

	
	/*
	 *  Allocate memory for our track reports. No DMA needed as the underlying
	 *  SPI/I2C routines will use a DMA enabled block of memory for transfers
	 */
	snt8100fsr->track_reports = memory_allocate(TRACK_REPORTS_MAX_LEN, 0);

	// Allocate a work object for queuing from IRQ handler
	INIT_DELAYED_WORK(&event_wq, event_wq_func);
#if USE_TRIG0_IRQ
	INIT_DELAYED_WORK(&irq_wq0, trig_wq_func0);
#endif
#if USE_TRIG1_IRQ
	INIT_DELAYED_WORK(&irq_wq1, trig_wq_func1);
#endif
#if USE_TRIG2_IRQ
	INIT_DELAYED_WORK(&irq_wq2, trig_wq_func2);
#endif
#if USE_TRIG3_IRQ
	INIT_DELAYED_WORK(&irq_wq3, trig_wq_func3);
#endif
	work_g = (void *)workqueue_alloc_work(sizeof(struct event_work),
				event_wq_func);
	if (!work_g) {
		PRINT_CRIT(OOM_STRING);
		return -ENOMEM;
	}
	
#if USE_TRIG0_IRQ
	{struct trig_work_s *w = (struct trig_work_s *)workqueue_alloc_work(sizeof(struct trig_work_s),
							trig_wq_func0);
	if (!w) {
		PRINT_CRIT(OOM_STRING);
		return -ENOMEM;
	}
	w->snt8100fsr = snt8100fsr;
	w->id = 0;
	trig0_irq_db.work = (void*)w;
	}
#endif

#if USE_TRIG1_IRQ
	{struct trig_work_s *w = (struct trig_work_s *)workqueue_alloc_work(sizeof(struct trig_work_s),
							trig_wq_func1);
	if (!w) {
		PRINT_CRIT(OOM_STRING);
		return -ENOMEM;
	}
	w->snt8100fsr = snt8100fsr;
	w->id = 1;
	trig1_irq_db.work = (void*)w;
	}
#endif

#if USE_TRIG2_IRQ
	{struct trig_work_s *w = (struct trig_work_s *)workqueue_alloc_work(sizeof(struct trig_work_s),
							trig_wq_func2);
	if (!w) {
		PRINT_CRIT(OOM_STRING);
		return -ENOMEM;
	}
	w->snt8100fsr = snt8100fsr;
	w->id = 2;
	trig2_irq_db.work = (void*)w;
	}
#endif

#if USE_TRIG3_IRQ
	{struct trig_work_s *w = (struct trig_work_s *)workqueue_alloc_work(sizeof(struct trig_work_s),
							trig_wq_func3);
	if (!w) {
		PRINT_CRIT(OOM_STRING);
		return -ENOMEM;
	}
	w->snt8100fsr = snt8100fsr;
	w->id = 3;
	trig2_irq_db.work = (void*)w;
	}
#endif
	work_g->snt8100fsr = snt8100fsr;
	// [dy] Event queue initializes firmware download context false
	work_g -> context_fwd = false;
	// Read the event register
	ret = read_register(snt8100fsr, REGISTER_EVENT, &reg_event);
	if (ret) {
		PRINT_CRIT("sb_read_register() failed");
		mutex_unlock(&snt8100fsr_g->ap_lock);
	}

	if (reg_event.boot == 0) {
		PRINT_INFO("System already booted");
	} else {
		PRINT_INFO("System booted");
	}
	
	// Get the product configuration string
	product_string = memory_allocate(PRODUCT_CONFIG_MAX_LEN, 0);
	if (product_string == NULL) {
		PRINT_CRIT("memory_allocate(PRODUCT_CONFIG_MAX_LEN) failed");
		mutex_unlock(&snt8100fsr_g->ap_lock);
	}

	ret = read_product_config(snt8100fsr, product_string);
	memory_free(product_string);
	if (ret) {
		PRINT_WARN("Unable to read product config");
	}

	// Set the frame rate
	PRINT_INFO("Setting frame rate to: %dhz", snt8100fsr->frame_rate);
	ret = write_register(snt8100fsr, REGISTER_FRAME_RATE, &snt8100fsr->frame_rate);
	if (ret) {
		PRINT_CRIT("Unable to set frame rate");
		mutex_unlock(&snt8100fsr_g->ap_lock);
	}

	// Enable IRQ processing
	PRINT_INFO("registering IRQ (num=%d, gpio=%d)", snt8100fsr_g->hostirq_gpio, IRQ_GPIO);
	snt_irq_db.top = irq_handler_top;
	snt_irq_db.irq_num = gpio_to_irq(snt8100fsr_g->hostirq_gpio);
	irq_handler_register(&snt_irq_db);
	enable_irq_wake(snt_irq_db.irq_num);
	
	#if USE_TRIG0_IRQ
	PRINT_INFO("registering TRIG0 (num=%d, gpio=%d)", snt8100fsr_g->snt_trig0_num, TRIG0_GPIO);
	trig0_irq_db.top = trig_handler_top0;
	trig0_irq_db.irq_num = gpio_to_irq(snt8100fsr_g->snt_trig0_num);
	irq_handler_register(&trig0_irq_db);
	#endif

	#if USE_TRIG1_IRQ
	PRINT_INFO("registering TRIG1 (num=%d, gpio=%d)", snt8100fsr_g->snt_trig1_num, TRIG1_GPIO);
	trig1_irq_db.top = trig_handler_top1;
	trig1_irq_db.irq_num = gpio_to_irq(snt8100fsr_g->snt_trig1_num);
	irq_handler_register(&trig1_irq_db);
	#endif

	#if USE_TRIG2_IRQ
	PRINT_INFO("registering TRIG2 (num=%d, gpio=%d)", snt8100fsr_g->snt_trig2_num, TRIG2_GPIO);
	trig2_irq_db.top = trig_handler_top2;
	trig2_irq_db.irq_num = gpio_to_irq(snt8100fsr_g->snt_trig2_num);
	irq_handler_register(&trig2_irq_db);
	#endif

	#if USE_TRIG3_IRQ
	PRINT_INFO("registering TRIG3 (num=%d, gpio=%d)", snt8100fsr_g->snt_trig3_num, TRIG3_GPIO);
	trig3_irq_db.top = trig_handler_top3;
	trig3_irq_db.irq_num = gpio_to_irq(snt8100fsr_g->snt_trig3_num);
	irq_handler_register(&trig3_irq_db);
	#endif
	// [dy] 2017-09-01 Touch Enable/Disable was not ifdef'd properly
	// Read the enable bit first
	ret = read_register(snt8100fsr, REGISTER_ENABLE, &reg_enable);
	if (ret) {
		PRINT_CRIT("Unable to read register enable");
		mutex_unlock(&snt8100fsr_g->ap_lock);
	}
	PRINT_INFO("Touch processing currently set to %d", reg_enable.enable);
	reg_enable.padding = 0;	
#ifdef TOUCH_ENABLED_AT_STARTUP
	// Set the Enable bit to process touch reports
	reg_enable.enable = 1;
#else
	reg_enable.enable = 0;
#endif	
	ret = write_register(snt8100fsr, REGISTER_ENABLE, &reg_enable);
	if (ret) {
		PRINT_CRIT("sb_read_register() failed");
		mutex_unlock(&snt8100fsr_g->ap_lock);
	}
	ret = read_register(snt8100fsr, REGISTER_ENABLE, &reg_enable);
	if (ret) {
		PRINT_CRIT("Unable to read register enable");
		mutex_unlock(&snt8100fsr_g->ap_lock);
	}

	if (reg_enable.enable == 1) {
		PRINT_INFO("Enabled hardware touch processing");
	} else {
#ifdef TOUCH_ENABLED_AT_STARTUP		
		PRINT_CRIT("Unable to enable hardware touch processing");
		mutex_unlock(&snt8100fsr_g->ap_lock);
#else
		PRINT_INFO("Hardware touch processing is disabled");
#endif
	}

	/* Calibration write data +++ */
	// Restore from boot_init_reg file
	// (such as calibration data)
	ret = enable_boot_init_reg_with_file(BOOT_INIT_REG_LOCATION);
	if (ret) {
		PRINT_WARN("Not calibrated: no file %s found", BOOT_INIT_REG_LOCATION);
	}
	/* Calibration write data --- */
	
	// Load any register values on the disk
	//load_registers_from_disk(snt8100fsr);

	asus_init_state();
	Grip_check_K_and_frame();
	PRINT_DEBUG("done");
	return 0;
}

static int init_after_reset( struct snt8100fsr *snt8100fsr) {
	struct register_event reg_event;	
	// [dy] Always declare register_enable
	struct register_enable reg_enable;
	int ret;
	PRINT_FUNC();
	if (is_irq_handler_registered(&snt_irq_db)) {
		PRINT_INFO("irq_handler is registered.");   
	} else {
		PRINT_CRIT("irq_handler not registered.");
	}

	#if USE_TRIG0_IRQ
	if (is_irq_handler_registered(&trig0_irq_db)) {
		PRINT_INFO("trig0_handler is registered.");   
	} else {
		PRINT_CRIT("trig0_handler not registered.");
	}
	#endif

	#if USE_TRIG1_IRQ
	if (is_irq_handler_registered(&trig1_irq_db)) {
		PRINT_INFO("trig1_handler is registered.");   
	} else {
		PRINT_CRIT("trig1_handler not registered.");
	}
	#endif


	#if USE_TRIG2_IRQ
	if (is_irq_handler_registered(&trig2_irq_db)) {
		PRINT_INFO("trig2_handler is registered.");   
	} else {
		PRINT_CRIT("trig2_handler not registered.");
	}
	#endif


	#if USE_TRIG3_IRQ
	if (is_irq_handler_registered(&trig3_irq_db)) {
		PRINT_INFO("trig3_handler is registered.");   
	} else {
		PRINT_CRIT("trig3_handler not registered.");
	}
	#endif
	
	// Read the event register
	ret = read_register(snt8100fsr, REGISTER_EVENT, &reg_event);
	if (ret) {
		PRINT_CRIT("sb_read_register() failed");
		return -1;
	}

	if (reg_event.boot == 0) {
		PRINT_INFO("System already booted");
	} else {
		PRINT_INFO("System booted");
	}

	snt8100fsr->active_sc_cmd = mc_no_command;

	/* Calibration write data +++ */
	// Restore from boot_init_reg file
	// (such as calibration data)
	ret = enable_boot_init_reg_with_file(BOOT_INIT_REG_LOCATION);
	if (ret) {
		PRINT_WARN("Not calibrated: no file %s found", BOOT_INIT_REG_LOCATION);
	}
	/* Calibration write data --- */

	ret = read_register(snt8100fsr, REGISTER_ENABLE, &reg_enable);
	if (ret) {
		PRINT_CRIT("Unable to read register enable");
	}

	PRINT_INFO("register enable: 0x%X", reg_enable.enable);
	return 0;
}

int set_context_fwd_done(struct snt8100fsr *snt8100fsr) {
	PRINT_FUNC();
	if (work_g -> context_fwd) {
		work_g -> context_fwd = false;
		PRINT_DEBUG("context_fwd set to false");
	}
	else {
		PRINT_DEBUG("context_fwd already false");
	}
	return 0;
}	

int ASUS_Handle_Reset(struct snt8100fsr *snt8100fsr){
	init_after_reset( snt8100fsr);
	asus_init_state();
	Grip_check_K_and_frame();
	if(finish_boot != 0){
		queue_delayed_work(asus_wq, &rst_recovery_wk, msecs_to_jiffies(0));
	}
	return 0;
}

extern bool wake_lock_disable_flag;
static irqreturn_t irq_handler_top(int irq, void *dev) {
	//PRINT_FUNC();
	/* [dy] If context of firmware download, firmware work queue
	 */
	if (work_g -> context_fwd) {
		PRINT_INFO("Chip reset: call irq_handler_fwd()");
		irq_handler_fwd();
	} else {
			// HZ = #_of_ticks/second, so (1 * HZ) means 1 second
		// wake_lock_timeout(&(work_g->snt8100fsr->snt_wakelock), 5*HZ);
		
		if(wake_lock_disable_flag==0){
			__pm_stay_awake(work_g->snt8100fsr->snt_wakelock);
			PRINT_DEBUG("wake_clock");
		}else{
			/* don't set wake_lock */
			PRINT_DEBUG("no wake_clock");
		}
		if(snt8100fsr_g->suspend_status == true){
			workqueue_queue_work(&event_wq, 10);
		} else {
		workqueue_queue_work(&event_wq, 0);
		}
	}
	PRINT_DEBUG("done");
	return IRQ_HANDLED;
}

#if USE_TRIG_IRQ
/*
static irqreturn_t trig_handler_top(int irq, void *dev) {
	p_irq_db_t p_irq = (p_irq_db_t) dev;
	PRINT_INFO("trig_handler_top");
	PRINT_FUNC();
	if (dev==NULL) {
		PRINT_CRIT("NULL dev ptr");
	} else {
		workqueue_queue_work(p_irq->work, 0);
	}
	PRINT_DEBUG("done");
	return IRQ_HANDLED;
}
*/
#endif

#if USE_TRIG0_IRQ
static irqreturn_t trig_handler_top0(int irq, void *dev) {
	PRINT_FUNC();
	if (dev==NULL) {
		PRINT_CRIT("NULL dev ptr");
	} else {
		workqueue_queue_work(&irq_wq0, 0);
	}
	PRINT_DEBUG("done");
	return IRQ_HANDLED;
}
#endif

#if USE_TRIG1_IRQ
static irqreturn_t trig_handler_top1(int irq, void *dev) {
	PRINT_FUNC();
	if (dev==NULL) {
		PRINT_CRIT("NULL dev ptr");
	} else {
		workqueue_queue_work(&irq_wq1, 0);
	}
	PRINT_DEBUG("done");
	return IRQ_HANDLED;
}
#endif

#if USE_TRIG2_IRQ
static irqreturn_t trig_handler_top2(int irq, void *dev) {
	PRINT_FUNC();
	if (dev==NULL) {
		PRINT_CRIT("NULL dev ptr");
	} else {
		workqueue_queue_work(&irq_wq2, 0);
	}
	PRINT_DEBUG("done");
	return IRQ_HANDLED;
}
#endif

#if USE_TRIG3_IRQ
static irqreturn_t trig_handler_top3(int irq, void *dev) {
	PRINT_FUNC();
	if (dev==NULL) {
		PRINT_CRIT("NULL dev ptr");
	} else {
		workqueue_queue_work(&irq_wq3, 0);
	}
	PRINT_DEBUG("done");
	return IRQ_HANDLED;
}
#endif

#if USE_TRIG0_IRQ
static void trig_wq_func0(struct work_struct *work) {
	 //struct trig_work_s *w = (struct trig_work_s*) work;
	 int gpio_val = 0;
	 gpio_val = gpio_get_value(snt8100fsr_g->snt_trig0_num);
	 PRINT_FUNC();
	 MUTEX_LOCK(&snt8100fsr_g->tap_lock);
	 process_trigger(0, gpio_val);
	 mutex_unlock(&snt8100fsr_g->tap_lock);
	PRINT_DEBUG("done.");
}
#endif
	 
#if USE_TRIG1_IRQ
static void trig_wq_func1(struct work_struct *work) {
	 //struct trig_work_s *w = (struct trig_work_s*) work;
	 int gpio_val = 0;
	 gpio_val = gpio_get_value(snt8100fsr_g->snt_trig1_num);
	 PRINT_FUNC();
	 MUTEX_LOCK(&snt8100fsr_g->tap_lock);
	 process_trigger(1, gpio_val);
	 mutex_unlock(&snt8100fsr_g->tap_lock);
	 PRINT_DEBUG("done.");
}
#endif
	 
#if USE_TRIG2_IRQ
static void trig_wq_func2(struct work_struct *work) {
	 //struct trig_work_s *w = (struct trig_work_s*) work;
	 int gpio_val = 0;
	 gpio_val = gpio_get_value(snt8100fsr_g->snt_trig2_num);
	 PRINT_FUNC();
	 MUTEX_LOCK(&snt8100fsr_g->tap_lock);
	 process_trigger(2, gpio_val);
	 mutex_unlock(&snt8100fsr_g->tap_lock);
	 PRINT_DEBUG("done.");
 }
#endif

#if USE_TRIG3_IRQ
static void trig_wq_func3(struct work_struct *work) {
	 //struct trig_work_s *w = (struct trig_work_s*) work;
	 int gpio_val = 0;
	 gpio_val = gpio_get_value(snt8100fsr_g->snt_trig3_num);
	 PRINT_FUNC();
	 MUTEX_LOCK(&snt8100fsr_g->tap_lock);
	 process_trigger(3, gpio_val);
	 mutex_unlock(&snt8100fsr_g->tap_lock);
	 PRINT_DEBUG("done.");
 }
#endif

static void event_wq_func(struct work_struct *lwork) {
	//struct register_event reg_event = {0};
	struct register_event reg_event;
	//struct event_work *w = (struct event_work *)work;
	struct event_work *w = work_g;
	struct snt8100fsr *snt8100fsr = w->snt8100fsr;
	int ret;
	MUTEX_LOCK(&snt8100fsr_g->IRQ_WAKE_SLEEP_LOCK);
	reg_event.stream = 0;
	reg_event.fwd = 0;
	reg_event.touch = 0;
	//PRINT_INFO("%u, %u, %u, %u, %u", reg_event.boot,  reg_event.touch, reg_event.command, reg_event.stream, reg_event.host);
	//PRINT_INFO("%u, %u, %u, %u, %u",  reg_event.test,  reg_event.rst, reg_event.fwd, reg_event.padding, reg_event.fault);
/*
	reg_event.boot = 0;
	reg_event.touch = 0;
	reg_event.command = 0;
	reg_event.stream = 0;
	reg_event.host = 0;
	reg_event.test = 0;
	reg_event.rst = 0;
	reg_event.fwd = 0;
	reg_event.padding = 0;
	reg_event.fault = 0;
*/
	PRINT_FUNC();
	
	if(snt8100fsr_g->suspend_status == true){
		PRINT_INFO("Waiting for snt_resume");
		workqueue_queue_work(&event_wq, 2);
		goto err_read_event_reg;
	}

	if(snt8100fsr_g->snt_state == GRIP_DEEPSLEEP){
		PRINT_INFO("Warning! Unexpeted Grip Irq, Skip it, %d", snt8100fsr_g->snt_state);
		goto err_read_event_reg;
	}else{
	//PRINT_INFO("Normal expeted Irq");
	}
	do {
		ret = read_register(snt8100fsr, REGISTER_EVENT, &reg_event);
		if (ret) {
		//PRINT_CRIT("Unable to read event register, reg_event=%u", reg_event);
			PRINT_CRIT("Unable to read event register");
			goto err_read_event_reg;
		}
		// Check if need to re-try event register
		if (1 == reg_event.stream && snt8100fsr->active_sc_cmd != mc_frame_dump) {
			PRINT_DEBUG("Spurious event reg 0x%x, re-acquire", *(uint16_t*)&reg_event);
			ret = read_register(snt8100fsr, REGISTER_EVENT, &reg_event);
			if (ret) {
				PRINT_CRIT("Unable to read event register");
				goto err_read_event_reg;
			}
		}

		// Check if firmware download requested
		if (1 == reg_event.fwd) {
			PRINT_INFO("Firmware download requested");
#ifdef UPLOAD_FIRMWARE
#ifdef DYNAMIC_PWR_CTL
			// back out pending Activity Requests
			snt8100fsr_g->wake_rsp_result = 1;  // FAIL
			while(down_trylock(&snt8100fsr->wake_req)==0) {
				PRINT_INFO("ActivityRsp FAIL:Fw res=%d", snt8100fsr_g->wake_rsp_result);
				up(&snt8100fsr_g->wake_rsp);
			}
#endif /* DYNAMIC_PWR_CTL */
			// back out sys param requests
			while(down_trylock(&snt8100fsr->sc_wf_rsp_req)==0) {
				PRINT_DEBUG("SysParam Rsp");
				up(&snt8100fsr_g->sc_wf_rsp);
			}
			// [dy] Set event context firmware download
			if(work_g -> context_fwd != true){
				work_g -> context_fwd = true;
				upload_firmware_fwd(snt8100fsr, FW_PATH);
			}else{
				PRINT_INFO("Avoid second redownload firmware!");
			}
#endif /* UPLOAD_FIRMWARE */

	}else { //Normal Event Processing

		// Do we have any track reports to handle?
		if (reg_event.touch) {
			read_track_reports(snt8100fsr);
		}

		// Any command completions?
#ifdef DYNAMIC_PWR_CTL
		//PRINT_INFO("event_wq_func::do-while: %x, %x, %d",
		//		reg_event.boot, reg_event.host, snt8100fsr->active_sc_cmd);
		if(reg_event.host==1){
			PRINT_DEBUG("===Wake Up, receive reg_event.host===");
		}
#endif /* DYNAMIC_PWR_CTL */
		if (reg_event.command) {
			switch (snt8100fsr->active_sc_cmd) {
				case mc_d1_test: log_d1test(snt8100fsr); break;
				case mc_frame_dump: log_frame_cmd(snt8100fsr); break;
				case mc_no_touch: log_no_touch_frame(snt8100fsr); break;
				case mc_set_sys_param: set_sys_param_rsp(snt8100fsr); break;
				case mc_get_sys_param: get_sys_param_rsp(snt8100fsr); break;
#ifdef SUPPORT_FLASH
				case mc_fw_update:
				case mc_flash_update: fwupdate_rsp_ind(snt8100fsr); break;
				case mc_update_regs:
#endif
				case mc_reg_script: write_flash_reg_part_rsp(snt8100fsr); break;
				default: spurious_cmd_event(snt8100fsr); break;
				}
			}
		}

		if (reg_event.stream) {
			log_frame_stream(snt8100fsr);
		}

		/*
		* Don't loop on reg_event.stream because too much data, we'll never
		* leave our while loop and the system may reboot due to watchdog timer
		*/
	} while (!reg_event.fwd && (reg_event.touch || (reg_event.command && (log_d1test_file == NULL))));

	if (!reg_event.fwd) {
#ifndef DYNAMIC_PWR_CTL
		//When Grip on and no DPC
		if(grip_status_g->G_EN==1 && grip_status_g->G_DPC_STATUS>=0){
			/*
			 * If the host bit is set, we are waking up from a sleep mode. We need
			 * to set the frame rate back to the last known setting. Least significant byte.
			 */
			if (reg_event.host) {
				PRINT_INFO("receive host, set frame_rate = %d", snt8100fsr->frame_rate);
				PRINT_DEBUG("Host event bit set, restoring frame rate to %dhz", snt8100fsr->frame_rate);
				ret = write_register(snt8100fsr, REGISTER_FRAME_RATE, &snt8100fsr->frame_rate);
				if (ret) {
					PRINT_CRIT("write_register(REGISTER_FRAME_RATE) failed");
				}
			}
		}
#endif /* DYNAMIC_PWR_CTL */
#ifdef DYNAMIC_PWR_CTL
		snt8100fsr_g->wake_rsp_result = 0;	// succsess!
		while(down_trylock(&snt8100fsr->wake_req)==0) {
			PRINT_DEBUG("ActivityRsp host=%d", reg_event.host);
			up(&snt8100fsr_g->wake_rsp);
		}
#endif /* DYNAMIC_PWR_CTL */
	}
	
err_read_event_reg:
	mutex_unlock(&snt8100fsr_g->IRQ_WAKE_SLEEP_LOCK);
	if(wake_lock_disable_flag==0){
			__pm_relax(work_g->snt8100fsr->snt_wakelock);
		PRINT_DEBUG("wake_unclock");
	}else{
		PRINT_DEBUG("no wake_unclock");
		/* don't use wake_lock */
	}
	return;
}

struct __attribute__((__packed__)) length_and_frame {
	uint16_t len; // length, in bytes, of FIFO message (including frame field)
	uint16_t frame; // frame number
};
//uint16_t reg_print[69];
static void read_track_reports(struct snt8100fsr *snt8100fsr) {
	static const uint16_t tr_size = sizeof(struct track_report);
	struct length_and_frame landf;
	int ret;

	//PRINT_FUNC();

	MUTEX_LOCK(&snt8100fsr->sb_lock);
	
	// Read the 4 byte Track Report Header 
	ret = sb_read_fifo(snt8100fsr, REGISTER_FIFO_TOUCH, 
					TOUCH_FIFO_LENGTH_WIDTH + TOUCH_FIFO_FRAME_WIDTH, &landf);
	if (ret) {
		PRINT_CRIT("sb_read_fifo() track reports length failed");
		goto cleanup_sb;
	}

	// If the length is greater than our max size, we have a problem
	if (landf.len > TRACK_REPORTS_MAX_LEN) {
		PRINT_CRIT("Track reports length greater than max %lu (%d)",
		   TRACK_REPORTS_MAX_LEN, landf.len);
		goto cleanup_sb;
	}

	// Remove frame field width from length to get size of track reports part
	landf.len -= TOUCH_FIFO_FRAME_WIDTH;

	// Fatal if length is not divisble by size of our track report struct
	if (landf.len % tr_size) {
		PRINT_CRIT("Length not multiple of %d (%d)", tr_size, landf.len);
		goto cleanup_sb;
	}

	/*
	 * Lock to protect track_reports and track_reports_count for the
	 * SysFS interface which uses these fields
	 */
	MUTEX_LOCK(&snt8100fsr->track_report_sysfs_lock);
	// Calculate the number of track reports
	snt8100fsr->track_reports_count = landf.len / tr_size;
	snt8100fsr->track_reports_frame = landf.frame;
	PRINT_DEBUG("[%d, %d]", landf.len, snt8100fsr->track_reports_count);

	// Read the track reports
	ret = sb_read_fifo(snt8100fsr, REGISTER_FIFO_TOUCH, landf.len, snt8100fsr->track_reports);
	if (ret) {
		PRINT_CRIT("sb_read_fifo() track reports failed");
		goto cleanup_sysfs_and_sb;
	}

	// Pass the reports to the processing routine
	process_track_reports(snt8100fsr->track_reports_frame, snt8100fsr->track_reports, snt8100fsr->track_reports_count);

	// If we are logging track reports, save to the file
	if(log_track_reports_file != NULL) {
		log_track_report(snt8100fsr->track_reports_frame, snt8100fsr->track_reports, snt8100fsr->track_reports_count);
	}
	if (log_track_reports_bin_file != NULL) {
		log_track_report_bin(snt8100fsr->track_reports_frame, snt8100fsr->track_reports, snt8100fsr->track_reports_count);
	}

	//PRINT_DEBUG("done");

cleanup_sysfs_and_sb:
	mutex_unlock(&snt8100fsr->track_report_sysfs_lock);
cleanup_sb:
	mutex_unlock(&snt8100fsr->sb_lock);
}

int read_product_config(struct snt8100fsr *snt8100fsr, char *product_string) {
	uint16_t len;
	int ret;

	MUTEX_LOCK(&snt8100fsr->sb_lock);

	// Read the length
	ret = sb_read_fifo(snt8100fsr, REGISTER_FIFO_CONFIG, TOUCH_FIFO_LENGTH_WIDTH, &len);
	if (ret) {
		PRINT_CRIT("sb_read_fifo() product config length failed");
		mutex_unlock(&snt8100fsr->sb_lock);
		return ret;
	}

	// If the length is greater than our max size, we have a problem
	if (len > PRODUCT_CONFIG_MAX_LEN) {
		PRINT_CRIT("Product config length greater than max %d (%d)", PRODUCT_CONFIG_MAX_LEN, len);
		mutex_unlock(&snt8100fsr->sb_lock);
		return -1;
	}

	// Read the product config string
	ret = sb_read_fifo(snt8100fsr, REGISTER_FIFO_CONFIG, len, product_string);
	if (ret) {
		PRINT_CRIT("sb_read_fifo() track reports failed");
		mutex_unlock(&snt8100fsr->sb_lock);
		return ret;
	}

	PRINT_NOTICE("Product Info: %s", product_string);
	mutex_unlock(&snt8100fsr->sb_lock);
	return 0;
}

/*==========================================================================*/
/* TRACK REPORT LOGGING                                                     */
/*==========================================================================*/
int enable_track_report_logging(bool enable, int ftype) {
	int ret;
	char log_header[] = "Timestamp, Frame No, Bar, Track ID, Force, Pos0, Pos1, Pos2\n";
	mutex_lock(&snt8100fsr_g->track_report_sysfs_lock);
	if (ftype == 1) {   // bin file
		if (enable) {
			if (log_track_reports_bin_file == NULL) {
				PRINT_DEBUG("Creating track report bin log file: %s", TRACK_REPORT_BIN_LOG_LOCATION);
				log_track_reports_bin_file_offset = 0;
				ret = file_open(TRACK_REPORT_BIN_LOG_LOCATION, O_WRONLY|O_CREAT|O_TRUNC, 0777,
					&log_track_reports_bin_file);
				if(ret) {
					PRINT_DEBUG("Unable to create file '%s', error %d", TRACK_REPORT_BIN_LOG_LOCATION, ret);
				mutex_unlock(&snt8100fsr_g->track_report_sysfs_lock);
				return -1;
			}
		} else {
			PRINT_NOTICE("Track report bin logging already enabled");
		}
	} else {
		if (log_track_reports_bin_file != NULL) {
			PRINT_DEBUG("Closing track report log file: %s", TRACK_REPORT_BIN_LOG_LOCATION);
			file_close(log_track_reports_bin_file);
			log_track_reports_bin_file = NULL;
			} else {
				PRINT_NOTICE("Track report bin logging already disabled");
			}
		}
		mutex_unlock(&snt8100fsr_g->track_report_sysfs_lock);
		return 0;
	}
	else {
		if (enable) {
			if (log_track_reports_file == NULL) {
				PRINT_DEBUG("Creating track report log file: %s",
							TRACK_REPORT_LOG_LOCATION);
			log_track_reports_file_offset = 0;
				ret = file_open(TRACK_REPORT_LOG_LOCATION,
								O_WRONLY|O_CREAT|O_TRUNC, 0777,
								&log_track_reports_file);
				if(ret) {
					PRINT_DEBUG("Unable to create file '%s', error %d",
								TRACK_REPORT_LOG_LOCATION, ret);
					mutex_unlock(&snt8100fsr_g->track_report_sysfs_lock);
					return -1;
				}

				// Write a text header so that the log file can be interpreted
				file_write(log_track_reports_file,
						log_track_reports_file_offset,
						log_header,
						sizeof(log_header)-1);
				log_track_reports_file_offset += sizeof(log_header)-1;
			} else {
				PRINT_NOTICE("Track report logging already enabled");
			}
		} else {
			if (log_track_reports_file != NULL) {
				PRINT_DEBUG("Closing track report log file: %s",
							TRACK_REPORT_LOG_LOCATION);
				file_close(log_track_reports_file);
				log_track_reports_file = NULL;
			} else {
				PRINT_NOTICE("Track report logging already disabled");
			}
		}
	}
	mutex_unlock(&snt8100fsr_g->track_report_sysfs_lock);
	return 0;
}

static int build_tr_diag_report_001(p_tr_diag_vers_001_t p_diag, uint8_t *log_buf, int max_buf)
{
	int i;
	int size = snprintf(log_buf, max_buf,
					"vers=%d, gpio=0x%02x, atc=%u, ntm.stress=%u, ntm.nt=%u\n",
					(unsigned int)p_diag->vers,
					(unsigned int)p_diag->gpio,
					(unsigned int)p_diag->atc,
					(unsigned int)p_diag->ntm>>4,
					(unsigned int)p_diag->ntm&0xf);
	size += snprintf(log_buf+size, max_buf-size, "mpa : ");
	for (i=0; i < TR_DIAG_MAX_MPATH; i++) {
		size += snprintf(log_buf+size, max_buf-size, "%u ",(unsigned int)p_diag->mpa[i]);
	}
	size += snprintf(log_buf+size, max_buf-size, "\nd_mp: ");
	for (i=0; i < TR_DIAG_MAX_MPATH; i++) {
		size += snprintf(log_buf+size, max_buf-size, "%u ",(unsigned int)p_diag->d_mp[i]);
	}
	size += snprintf(log_buf+size, max_buf-size, "\n\n");
	return size;
}

static int build_tr_diag_report_002(p_tr_diag_vers_002_t p_diag, uint8_t *log_buf, int max_buf)
{
	int i;
	int size = snprintf(log_buf, max_buf,
					"vers=%d, frame_rate=%d, trig=0x%02x, gpio=0x%02x, atc=%u, ntm.stress=%u, ntm.nt=%u\n",
					(unsigned int)p_diag->vers,
					(unsigned int)p_diag->frame_rate,
					(unsigned int)p_diag->trig,
					(unsigned int)p_diag->gpio,
					(unsigned int)p_diag->atc,
					(unsigned int)p_diag->ntm>>4,
					(unsigned int)p_diag->ntm&0xf);
	size += snprintf(log_buf+size, max_buf-size, "mpa : ");
	for (i=0; i < TR_DIAG_MAX_MPATH; i++) {
		size += snprintf(log_buf+size, max_buf-size, "%u ",(unsigned int)p_diag->mpa[i]);
	}
	size += snprintf(log_buf+size, max_buf-size, "\nd_mp: ");
	for (i=0; i < TR_DIAG_MAX_MPATH; i++) {
		size += snprintf(log_buf+size, max_buf-size, "%u ",(unsigned int)p_diag->d_mp[i]);
	}
	size += snprintf(log_buf+size, max_buf-size, "\n\n");
	return size;
}

static int build_tr_diag_report(uint8_t* p_diag, uint8_t *log_buf, int max_buf)
{
	if (p_diag) {
		switch (p_diag[1]) {
			case 1: return build_tr_diag_report_001((p_tr_diag_vers_001_t)p_diag, log_buf, max_buf); break;
			case 2: return build_tr_diag_report_002((p_tr_diag_vers_002_t)p_diag, log_buf, max_buf); break;
			default: PRINT_CRIT("Unkown tr_diag version %d", p_diag[1]); return(0); break;
		}
	}
	return(0);
}

static int build_gs_report(uint8_t* p_buf, uint8_t *log_buf, int max_buf)
{
	int size;
	PRINT_FUNC();

	size = 0;
	if (GS_IS_HDR_REPORT(p_buf)) {
		p_gs_hdr_rec_t p = (p_gs_hdr_rec_t) p_buf;
		size += snprintf(log_buf, max_buf - size, 
					"0x%x, %d, %d, %d, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u\n", p->escape1,
					p->swipe0_velocity, p->swipe1_velocity, p->swipe2_velocity,
					GS_GET_SQUEEZE_START(p->squeeze), GS_GET_SQUEEZE_CANCEL(p->squeeze),
					GS_GET_SQUEEZE_SHORT(p->squeeze), GS_GET_SQUEEZE_LONG(p->squeeze),
					GS_GET_SQUEEZE_END(p->squeeze),
					GS_GET_TAP_START(p->tap,0), GS_GET_TAP_START(p->tap,1),
					GS_GET_TAP_START(p->tap,2), GS_GET_TAP_START(p->tap,3),
					GS_GET_TAP_STOP(p->tap,0),  GS_GET_TAP_STOP(p->tap,1),
					GS_GET_TAP_STOP(p->tap,2), GS_GET_TAP_STOP(p->tap,3));
	} else {
		p_gs_slider_rec_t p = (p_gs_slider_rec_t) p_buf;
		if (p->escape0 == GS_RPT_TYPE_SLIDE) {
			uint8_t id0 = GS_GET_SLIDER_ID0(p->slider_finger_id);
			uint8_t id1 = GS_GET_SLIDER_ID1(p->slider_finger_id);
			uint8_t fid0 = GS_GET_SLIDER_FID0(p->slider_finger_id);
			uint8_t fid1 = GS_GET_SLIDER_FID1(p->slider_finger_id);
			if (fid0) {
				size += snprintf(log_buf + size, max_buf - size,
				"0x%x, %u, %u, %u, %u\n", p->escape0,
				id0, fid0,
				p->slider_force0, p->slider_pos0);
			}
			if (fid1) {
				size += snprintf(log_buf + size, max_buf - size,
					"0x%x, %u, %u, %u, %u\n", p->escape0,
					id1, fid1,
					p->slider_force1, p->slider_pos1);
			}

		} else if (p->escape0 == GS_RPT_TYPE_SQUEEZE) {
			int sqIdx;
			p_gs_squeeze_rec_t p_sq = (p_gs_squeeze_rec_t) p;
			size += snprintf(log_buf + size, max_buf - size,"0x%x",p_sq->escape0);
			for (sqIdx = 0; sqIdx < SNT_GS_SQUEEZE_MAX_NUM; sqIdx++) {
				size += snprintf(log_buf + size, max_buf - size,
					", %u, %u, %u, %u, %u, %u", sqIdx,
					GS_GET_SQUEEZE_START(p_sq->squeeze[sqIdx]),
					GS_GET_SQUEEZE_CANCEL(p_sq->squeeze[sqIdx]),
					GS_GET_SQUEEZE_SHORT(p_sq->squeeze[sqIdx]),
					GS_GET_SQUEEZE_LONG(p_sq->squeeze[sqIdx]), 
					GS_GET_SQUEEZE_END(p_sq->squeeze[sqIdx]) );
			}
			size += snprintf(log_buf + size, max_buf - size,"\n");
		}
	}
	return size;
}

static void log_track_report_bin(uint16_t frame,
                      struct track_report *tr,
                      size_t count) {

    // record is tot_len(2) ts(4) reclen(2) frame(2) records(8*x)
    static const uint16_t tr_size = sizeof(struct track_report);
    uint32_t ms = get_time_in_ms();
    uint16_t hdr[5];
    hdr[0] = count*tr_size + 2*sizeof(uint16_t) + sizeof(uint32_t);
    hdr[1] = (uint16_t)(ms&0xffff);
    hdr[2] = (uint16_t)(ms>>16);
    hdr[3] = count*tr_size + sizeof(uint16_t);
    hdr[4] = frame;

    PRINT_FUNC();

    file_write(log_track_reports_bin_file,
                log_track_reports_bin_file_offset,
                (void*)hdr,
                sizeof(hdr));
    log_track_reports_bin_file_offset += sizeof(hdr);
    file_write(log_track_reports_bin_file,
                log_track_reports_bin_file_offset,
                (void*)tr,
                count*tr_size);
    log_track_reports_bin_file_offset += count*tr_size;
}


static void log_track_report(uint16_t frame,
                      struct track_report *tr,
                      size_t count) {
	char log_buf[256];
	uint32_t ms;
	int i, size;
	int in_gs = 0;
	int done = 0;   // set when get tr_diag report as it uses rest of report

	PRINT_FUNC();

	ms = get_time_in_ms();

	for(i = 0; i < count && !done; i++) {
		struct track_report *p_tr = &tr[i];
		// start new line of data.
		size = snprintf(log_buf, sizeof(log_buf), "%u, %u, ", ms, frame);
		// diag. consumes the rest of the records so forces done.
		if (TR_DIAG_IS_REPORT(p_tr)) { 
			size += build_tr_diag_report((uint8_t*)p_tr, log_buf+size, sizeof(log_buf)-size);
			done = 1;
	} else if (in_gs || GS_IS_REPORT(p_tr)) { // gestures
			size += build_gs_report((uint8_t*)p_tr, log_buf+size, sizeof(log_buf)-size);
			in_gs = 1;
		} else { // track reports
			if (IS_STG_TRACK_REPORT(tr[i].bar_id)) {
				struct stg_track_report *tg = (struct stg_track_report*) tr;
				// Timestamp, Frame, Bar, Frc0, Frc1, Frc2, Frc3, Frc4, Frc5, Frce6
				size += snprintf(log_buf+size, sizeof(log_buf)-size,
					"%u, %u, %u, %u, %u, %u, %u, %u\n",
					tg[i].bar_id,
					tg[i].force_lvl[0],
					tg[i].force_lvl[1],
					tg[i].force_lvl[2],
					tg[i].force_lvl[3],
					tg[i].force_lvl[4],
					tg[i].force_lvl[5],
					tg[i].force_lvl[6]
					);
			} else {
				// Timestamp, Frame No, Bar, Track ID, Force, Pos0, Pos1, Pos2
				size += snprintf(log_buf+size, sizeof(log_buf)-size,
					"%u, %u, %u, %u, %u, %u\n",
					tr[i].bar_id,
					tr[i].trk_id,
					tr[i].force_lvl,
					tr[i].center,
					tr[i].bottom,
					tr[i].top);
		}
	}
	if (size > 0) {
		file_write(log_track_reports_file,	log_track_reports_file_offset, log_buf, size);
		log_track_reports_file_offset += size;
		}
	}
}

int enable_event_logging(struct snt8100fsr *snt8100fsr) {
	int ret=0;

	PRINT_FUNC();

	/*
	 * We must mutex lock before using sc_cmd to protect it from other
	 * commands which use the same structure */
	MUTEX_LOCK(&snt8100fsr->sb_lock);

	if (snt8100fsr->event_log_file == NULL) {
		PRINT_DEBUG("Creating event log file: %s", EVENT_LOG_LOCATION);
		ret = file_open(EVENT_LOG_LOCATION,
						O_WRONLY|O_CREAT|O_TRUNC, 0777,
						&snt8100fsr->event_log_file);
		if(ret) {
			PRINT_DEBUG("Unable to create file '%s', error %d",
						EVENT_LOG_LOCATION, ret);
			goto errexit;
		}
	} else {
		PRINT_NOTICE("Event logging already enabled");
		goto cleanup;
	}

	if (snt8100fsr->active_sc_cmd != mc_no_command) {
		PRINT_CRIT("Cannot retrieve event log. Command (%d) already active", snt8100fsr->active_sc_cmd);
		ret = -1;
		goto errexit;
	}

	if (sc_cmd == NULL) {
		sc_cmd = memory_allocate(sizeof(*sc_cmd), GFP_DMA);
	}
	
	if (sc_cmd == NULL) {
		PRINT_CRIT("Unable to allocate sc cmd for event logging");
		goto errexit;
	}

	// request write of flash register init partition
	sc_cmd->magic = SC_MAGIC_NUMBER;
	sc_cmd->address = get_sys_param_event_log;
	sc_cmd->length = 0;
	sc_cmd->major = mc_get_sys_param;
	sc_cmd->minor = min_ok;
	sc_cmd->trans_id = get_time_in_ms(); // unique number

	ret = sb_write_fifo(snt8100fsr,
						REGISTER_FIFO_COMMAND,
						SC_COMMAND_HEADER_SIZE + sc_cmd->length,
						sc_cmd);

	if (ret) {
		PRINT_CRIT("Unable to write to command fifo: %d", ret);
		goto errexit;
	}
	// success! cache active cmd and prepare for rsp
	snt8100fsr->active_sc_cmd = mc_get_sys_param;
	PRINT_DEBUG("GetSysParam Req - event_log");
	up(&snt8100fsr->sc_wf_rsp_req); // post req for rsp event

cleanup:
	mutex_unlock(&snt8100fsr->sb_lock);
	PRINT_DEBUG("done %d", ret);
	return ret;

 errexit:
	if (snt8100fsr->event_log_file != NULL) {
		file_close(snt8100fsr->event_log_file);
		snt8100fsr->event_log_file = NULL;
	}
	if (snt8100fsr->active_sc_cmd == mc_no_command && sc_cmd) {
		memory_free(sc_cmd);
		sc_cmd = NULL;
	}
	goto cleanup;
}

void event_logging_cleanup(struct snt8100fsr *snt8100fsr)
{
	if (snt8100fsr->event_log_file != NULL) {
		file_close(snt8100fsr->event_log_file);
		snt8100fsr->event_log_file = NULL;
	}
}


/*==========================================================================*/
/* Read SC Response                                                          */
/*==========================================================================*/


int snt_read_sc_rsp(struct snt8100fsr *snt8100fsr) {

	int ret = -1;

	if (sc_cmd == NULL) {
		PRINT_CRIT("sc_cmd not initialized!");
		goto cleanup;
	}

	memset(sc_cmd, 0, sizeof(*sc_cmd));
	// Read the entire sonacomm header
	ret = sb_read_fifo(snt8100fsr,
					REGISTER_FIFO_COMMAND,
					SC_OVERHEAD, // 16 bytes
					sc_cmd);

	if (ret) {
		PRINT_CRIT("Unable to read command fifo length: %x", ret);
		goto cleanup;
	}

	if (sc_cmd->length > SC_DATA_PAYLOAD_BYTES) {
		PRINT_CRIT("Command fifo length is too large: %d", sc_cmd->length);
		ret = -1;
		goto cleanup;
	}

	PRINT_DEBUG("sc_cmd->length = %d bytes", sc_cmd->length);

	// Read the rest of the sonacomm data
	ret = sb_read_fifo(snt8100fsr,
					REGISTER_FIFO_COMMAND,
					sc_cmd->length,
					&sc_cmd->data[0]);

	if (ret) {
		PRINT_CRIT("Unable to read command fifo data: %x", ret);
		goto cleanup;
	}

cleanup:
	return ret;
}


/*==========================================================================*/
/* D1Test LOGGING                                                           */
/*==========================================================================*/

static void d1test_cleanup(struct snt8100fsr *snt8100fsr) {

	PRINT_FUNC();

	if (sc_cmd != NULL) {
		memory_free(sc_cmd);
		sc_cmd = NULL;
	}

	if (log_d1test_file != NULL) {
		PRINT_DEBUG("Closing d1test log file: %s", D1TEST_DATA_LOG_LOCATION);
		file_close(log_d1test_file);
		log_d1test_file = NULL;
	} else {
		PRINT_NOTICE("d1test logging already disabled");
	}
	log_d1test_num_frames = 0;
	if (snt8100fsr->active_sc_cmd == mc_d1_test){
		snt8100fsr->active_sc_cmd = mc_no_command;
	}
	PRINT_DEBUG("done. active_sc_cmd = %d", snt8100fsr->active_sc_cmd);
}



int enable_d1test_logging(struct snt8100fsr *snt8100fsr, int num_frames) {
	int ret;

	PRINT_FUNC();

	/*
	* We must mutex lock before using sc_cmd to protect it from other
	* commands which use the same structure
	*/
	MUTEX_LOCK(&snt8100fsr->sb_lock);

	log_d1test_num_frames = num_frames;

	if (log_d1test_num_frames > 0) {

		if (snt8100fsr->active_sc_cmd != mc_no_command) {
			PRINT_ERR("Cannot start d1Test. SC Cmd %d Active", snt8100fsr->active_sc_cmd);
			mutex_unlock(&snt8100fsr->sb_lock);
			return -1;
		}

		if (log_d1test_file == NULL) {
			PRINT_DEBUG("Creating d1test log file: %s",
						D1TEST_DATA_LOG_LOCATION);
			log_d1test_file_offset = 0;
			ret = file_open(D1TEST_DATA_LOG_LOCATION,
							O_WRONLY|O_CREAT|O_TRUNC,
							0777,
							&log_d1test_file);
			if(ret) {
				PRINT_DEBUG("Unable to create file '%s', error %d",
							D1TEST_DATA_LOG_LOCATION, ret);
				mutex_unlock(&snt8100fsr->sb_lock);
				return -1;
			}
		} else {
			PRINT_NOTICE("d1test logging already enabled");
			mutex_unlock(&snt8100fsr->sb_lock);
			return 0;
		}
	}

	if (sc_cmd == NULL) {
		sc_cmd = memory_allocate(sizeof(*sc_cmd), GFP_DMA);
	}
	
	if (sc_cmd == NULL) {
		PRINT_CRIT("Unable to allocate sc cmd for d1test");
		mutex_unlock(&snt8100fsr->sb_lock);
		return -1;
	}
	
	sc_cmd->magic = SC_MAGIC_NUMBER;
	sc_cmd->address = 0;
	sc_cmd->major = mc_d1_test;
	sc_cmd->trans_id = get_time_in_ms();
	if (log_d1test_num_frames > 0) {
		sc_cmd->data[0] = log_d1test_num_frames;
		sc_cmd->minor = min_ok;
		sc_cmd->length = sizeof(uint32_t);
	} else {
		sc_cmd->minor = min_data_last;
		sc_cmd->length = 0;
	}

	snt8100fsr->active_sc_cmd = mc_d1_test;
	ret = sb_write_fifo(snt8100fsr,
						REGISTER_FIFO_COMMAND,
						SC_COMMAND_HEADER_SIZE + sc_cmd->length,
						sc_cmd);

	if (ret) {
		PRINT_CRIT("Unable to write to command fifo: %d", ret);
		d1test_cleanup(snt8100fsr);
		mutex_unlock(&snt8100fsr->sb_lock);
		return -1;
	}

	if (log_d1test_num_frames == 0) {
		d1test_cleanup(snt8100fsr);
	}

	mutex_unlock(&snt8100fsr->sb_lock);
	PRINT_DEBUG("done");
	return 0;
}

static void log_d1test(struct snt8100fsr *snt8100fsr) {
	int ret;

	PRINT_FUNC();

	/*
	* We must mutex lock before using sc_cmd to protect it from other
	* commands which use the same structure
	*/
	MUTEX_LOCK(&snt8100fsr->sb_lock);

	ret = snt_read_sc_rsp(snt8100fsr);
	if (ret) {
		PRINT_CRIT("Read cmd Fifo rsp failed");
		snt8100fsr->active_sc_cmd = mc_no_command;
		goto cleanup;
	}

	// Write the data portion to our log file
	if(log_d1test_file != NULL) {
		file_write(log_d1test_file,
				log_d1test_file_offset,
				(unsigned char*)&sc_cmd->data[0],
				sc_cmd->length);
		log_d1test_file_offset += sc_cmd->length;
	} else {
		PRINT_NOTICE("d1test data received with no open log file");
	}

	log_d1test_num_frames--;
	if (log_d1test_num_frames <= 0) {
		d1test_cleanup(snt8100fsr);
	}

cleanup:
	mutex_unlock(&snt8100fsr->sb_lock);
	PRINT_DEBUG("done");
}

/*==========================================================================*/
/* FRAME DATA LOGGING                                                       */
/*==========================================================================*/

void frame_logging_cleanup(struct snt8100fsr *snt8100fsr)
{
	PRINT_FUNC();

	if (frame_buffer != NULL) {
		memory_free(frame_buffer);
		frame_buffer = NULL;
	}

	if (sc_cmd != NULL) {
		memory_free(sc_cmd);
		sc_cmd = NULL;
	}

	snt8100fsr->active_sc_cmd = mc_no_command;
	snt8100fsr->frame_stream_count = 0;


	if (log_frame_file != NULL) {
		PRINT_DEBUG("Closing frame data log file: %s",
					FRAME_DATA_LOG_LOCATION);
		file_close(log_frame_file);
		log_frame_file = NULL;
	}

	PRINT_DEBUG("done");
}

int enable_frame_logging(struct snt8100fsr *snt8100fsr,
                         uint32_t frame_count) {
	int ret;

	// Enable or disable capture based on frame count
	bool enable = (frame_count > 0);

	PRINT_FUNC("%d frames", frame_count);

	/*
	* We must mutex lock before using sc_cmd to protect it from other
	* commands which use the same structure.
	*/
	MUTEX_LOCK(&snt8100fsr->sb_lock);

	if (enable) {
		if (snt8100fsr->active_sc_cmd != mc_no_command) {
			PRINT_CRIT("Cannot perform Frame Logging. Command (%d) already active", snt8100fsr->active_sc_cmd);
			mutex_unlock(&snt8100fsr->sb_lock);
			return -1;
		}
		if (log_frame_file == NULL) {
			PRINT_DEBUG("Creating frame data log file: %s",
						FRAME_DATA_LOG_LOCATION);
			log_frame_file_offset = 0;
			ret = file_open(FRAME_DATA_LOG_LOCATION,
							O_WRONLY|O_CREAT|O_TRUNC,
							0777,
							&log_frame_file);
			if(ret) {
				PRINT_DEBUG("Unable to create file '%s', error %d",
							FRAME_DATA_LOG_LOCATION, ret);
				mutex_unlock(&snt8100fsr->sb_lock);
				return -1;
			}

			/*
			* Until we get our command response with the frame size on stream fifo,
			* set the frame_size to 0
			*/
			frame_size = 0;
		} else {
			PRINT_NOTICE("Frame data logging already enabled");
			mutex_unlock(&snt8100fsr->sb_lock);
			return 0;
		}

		if (sc_cmd == NULL) {
			sc_cmd = memory_allocate(sizeof(*sc_cmd), GFP_DMA);
		}

	if (sc_cmd == NULL) {
		PRINT_CRIT("Unable to allocate sc cmd for frame logging");
		mutex_unlock(&snt8100fsr->sb_lock);
		return -1;
	}

		if (sc_cmd == NULL) {
			PRINT_CRIT("Unable to allocate sc cmd for frame logging");
			mutex_unlock(&snt8100fsr->sb_lock);
			return -1;
		}

		if (frame_buffer == NULL) {
			frame_buffer = memory_allocate(FRAME_BUFFER_SIZE, GFP_DMA);
		}
	}

	// Capture this many frames
	snt8100fsr->frame_stream_count = frame_count;

	if(sc_cmd){
		// Enable the frame data logging
		sc_cmd->magic = SC_MAGIC_NUMBER;
		sc_cmd->address = 0;
		sc_cmd->length = 0;
		sc_cmd->major = mc_frame_dump;
		sc_cmd->minor = enable ? min_ok : min_data_last;
		sc_cmd->trans_id = get_time_in_ms();
		sc_cmd->length = sizeof(sc_cmd->data[0]);
		sc_cmd->data[0] = frame_count;

		ret = sb_write_fifo(snt8100fsr,
							REGISTER_FIFO_COMMAND,
							SC_COMMAND_HEADER_SIZE + sizeof(sc_cmd->data[0]),
							sc_cmd);

		if (ret) {
			mutex_unlock(&snt8100fsr->sb_lock);
			PRINT_CRIT("Unable to write to command fifo: %d", ret);
			return -1;
		}
	}
	snt8100fsr->active_sc_cmd = mc_frame_dump;

	if (!enable) {
		/* Temporary fix: don't free the memory
		if (frame_buffer != NULL) {
			memory_free(frame_buffer);
			frame_buffer = NULL;
		}

		if (sc_cmd != NULL) {
			memory_free(sc_cmd);
			sc_cmd = NULL;
		}*/

		snt8100fsr->active_sc_cmd = mc_no_command;

		if (log_frame_file != NULL) {
			PRINT_DEBUG("Closing frame data log file: %s",
						FRAME_DATA_LOG_LOCATION);
			file_close(log_frame_file);
			log_frame_file = NULL;
		} else {
			PRINT_NOTICE("Frame data logging already disabled");
		}
	}

	mutex_unlock(&snt8100fsr->sb_lock);
	PRINT_DEBUG("done");
	return 0;
}

static void log_frame_cmd(struct snt8100fsr *snt8100fsr) {
	int ret;

	PRINT_FUNC();

	/*
 	* We must mutex lock before using sc_cmd to protect it from other
 	* commands which use the same structure
 	*/
	MUTEX_LOCK(&snt8100fsr->sb_lock);

	ret = snt_read_sc_rsp(snt8100fsr);
	if (ret) {
		PRINT_CRIT("Read cmd Fifo rsp failed");
		goto cleanup;
	}

	// sc_cmd->data[0] is the length of the frame to read from stream fifo
	frame_size = sc_cmd->data[0];

	if (frame_size > 16384) {
		PRINT_CRIT("frame_size too large (max %d): %d bytes", 16384, frame_size);
		goto cleanup;
	}

	PRINT_INFO("Frame data size %d bytes in stream fifo", frame_size);

cleanup:
	mutex_unlock(&snt8100fsr->sb_lock);
	PRINT_DEBUG("done");
}

static void log_frame_stream(struct snt8100fsr *snt8100fsr) {
	int ret;

	PRINT_FUNC();

	MUTEX_LOCK(&snt8100fsr->sb_lock);

	if (frame_size == 0) {
		PRINT_CRIT("Frame size not received on command fifo!");
		goto cleanup;
	}

	if (frame_buffer) {
		// Read the length and frame size
		ret = sb_read_fifo(snt8100fsr,
						REGISTER_FIFO_STREAM,
						FRAME_LENGTH_WIDTH + frame_size,
						frame_buffer);

		if (ret) {
			PRINT_CRIT("Unable to read stream fifo length: %d", ret);
			goto cleanup;
		}

		// Write the frame buffer to our log file, skipping the LENGTH header
		if(log_frame_file != NULL) {
			file_write(log_frame_file,
					log_frame_file_offset,
					&frame_buffer[FRAME_LENGTH_WIDTH],
					frame_size);
			log_frame_file_offset += frame_size;
		} else {
			PRINT_NOTICE("Frame data received with no open log file");
		}

		snt8100fsr->frame_stream_count--;
	} else {
		sc_flush(snt8100fsr);
		PRINT_DEBUG("Cleaning up spurious streamed frame");
		frame_logging_cleanup(snt8100fsr);
		goto cleanup;
	}

	if (snt8100fsr->frame_stream_count < 1) {
		snt8100fsr->active_sc_cmd = mc_no_command;
		PRINT_NOTICE("All frame stream data received, closing log file");
		mutex_unlock(&snt8100fsr->sb_lock);
		enable_frame_logging(snt8100fsr, 0);
		PRINT_DEBUG("done");
		return;
	}

cleanup:
	mutex_unlock(&snt8100fsr->sb_lock);
	PRINT_DEBUG("done");
}

/*==========================================================================*/
/* NO TOUCH FRAME DUMP                                                      */
/*==========================================================================*/

void no_touch_logging_cleanup(struct snt8100fsr *snt8100fsr)
{
	if (log_no_touch_frame_file) {
		file_close(log_no_touch_frame_file);
		log_no_touch_frame_file = NULL;
	}
	snt8100fsr->active_sc_cmd = mc_no_command;

	if (sc_cmd != NULL) {
		memory_free(sc_cmd);
		sc_cmd = NULL;
	}
}

int enable_no_touch_logging(struct snt8100fsr *snt8100fsr) {
	/*
	* This method must not be called from our workerqueue. It will generally
	* be called from sysfs_no_touch_show(). This method will wait for
	* interrupts and use the workerqueue to gather the data, then we will
	* respond with the final result once we have it
	*/
	int ret;

	PRINT_FUNC();
	
	/*
	* We must mutex lock before using sc_cmd to protect it from other
	* commands which use the same structure */
	MUTEX_LOCK(&snt8100fsr->sb_lock);

	if (snt8100fsr->active_sc_cmd != mc_no_command) {
		PRINT_CRIT("Cannot perform NoTouch Logging. Command (%d) already active", snt8100fsr->active_sc_cmd);
		mutex_unlock(&snt8100fsr->sb_lock);
		return -1;
	}

	if (log_no_touch_frame_file == NULL) {
		PRINT_DEBUG("Creating no_touch data log file: %s",
					NO_TOUCH_FRAME_DATA_LOG_LOCATION);
		log_no_touch_frame_file_offset = 0;
		ret = file_open(NO_TOUCH_FRAME_DATA_LOG_LOCATION,
						O_WRONLY|O_CREAT|O_TRUNC,
						0777,
						&log_no_touch_frame_file);
		if(ret) {
			PRINT_DEBUG("Unable to create file '%s', error %d",
						NO_TOUCH_FRAME_DATA_LOG_LOCATION, ret);
			mutex_unlock(&snt8100fsr->sb_lock);
			return -1;
		}
	} else {
		PRINT_NOTICE("no_touch frame logging already enabled");
	}

	if (sc_cmd == NULL) {
		sc_cmd = memory_allocate(sizeof(*sc_cmd), GFP_DMA);
	}

	if (sc_cmd == NULL) {
		PRINT_CRIT("Unable to allocate sc cmd for notouch logging");
		mutex_unlock(&snt8100fsr->sb_lock);
		return -1;
	}

	// Enable the no_touch sensor frame logging
	sc_cmd->magic = SC_MAGIC_NUMBER;
	sc_cmd->address = 0;
	sc_cmd->length = 0;
	sc_cmd->major = mc_no_touch;
	sc_cmd->minor = min_ok;
	sc_cmd->trans_id = get_time_in_ms();
	sc_cmd->length = 0;

	ret = sb_write_fifo(snt8100fsr,
						REGISTER_FIFO_COMMAND,
						SC_COMMAND_HEADER_SIZE,
						sc_cmd);

	if (ret) {
		mutex_unlock(&snt8100fsr->sb_lock);
		PRINT_CRIT("Unable to write to command fifo: %d", ret);
		return -1;
	}
	snt8100fsr->active_sc_cmd = mc_no_touch;

	mutex_unlock(&snt8100fsr->sb_lock);
	PRINT_DEBUG("done");
	return 0;
}

static void log_no_touch_frame(struct snt8100fsr *snt8100fsr) {
	int ret;

	PRINT_FUNC();

	/*
	* We must mutex lock before using sc_cmd to protect it from other
	* commands which use the same structure
	*/
	MUTEX_LOCK(&snt8100fsr->sb_lock);

	ret = snt_read_sc_rsp(snt8100fsr);
	if (ret) {
		PRINT_CRIT("Read cmd Fifo rsp failed");
		goto errexit;
	}

	// Write the data portion to our log file
	if(log_no_touch_frame_file != NULL) {
		file_write(log_no_touch_frame_file,
					log_no_touch_frame_file_offset,
					(unsigned char*)&sc_cmd->data[0],
					sc_cmd->length);
		log_no_touch_frame_file_offset += sc_cmd->length;
	} else {
		PRINT_WARN("no_touch frame data received with no open log file");
	}

	// Check if we are done receiving data
	if (sc_cmd->minor == min_data_last) {
		PRINT_DEBUG("Received last no_touch frame data");
		no_touch_logging_cleanup(snt8100fsr);
	}

cleanup:
	mutex_unlock(&snt8100fsr->sb_lock);
	PRINT_DEBUG("done");
	return;

errexit:
	no_touch_logging_cleanup(snt8100fsr);
	goto cleanup;
}

#define DIGIT1 0
#define DIGIT2 1
#define FIRSTHEX 2
#define READNUM 3
#define COMMENT 4


int CvtAscii2Digit(uint8_t byte, int radix)
{
	int digit = -1;
	if (byte >= '0' && byte <= '9') {
		digit = byte - '0';
	} else if (radix == 16 && byte >= 'a' && byte <= 'f') {
		digit = byte - 'a' + 10;
	} else if (radix == 16 && byte >= 'A' && byte <= 'F') {
		digit = byte - 'A' + 10;
	}
	return digit;
}

////////////////////////////////////////////////
//
// Reads number of the form: 1234 or 0x12ab or -53
// hex numbers must start with 0x (not 0X)
// can have comments "#"
// RETURNS:
// 0 - success
// 1 - EOB
// 2 - Parse error
///////////////////////////////////////////////
int ReadNumFromBuf(const uint8_t **p_in, int *count, uint32_t *ret_val)
{
	uint8_t byte;
	int state = DIGIT1;
	int radix = 10;
	int comment_state = COMMENT;
	int val = 0;
	int sign = 1;

	//PRINT_FUNC("%d bytes", *count);

	while (*count > 0) {
		byte = *(*p_in);
		*p_in += 1; // change the reference value
		*count -= 1;

		if (byte == '#') {
			comment_state = state;
			//PRINT_DEBUG("Start Comment\n");
			state = COMMENT;
		}

		//PRINT_DEBUG("state=%d, val=0x%08x, byte=%x, radix=%d\n", state, (uint32_t)val, byte, radix);

		switch (state) {

			case DIGIT1: {
				if (byte == '-') {
					sign = -1;
					state = READNUM;
				} else {
					int digit = CvtAscii2Digit(byte, radix);
					if (digit >= 0) {
					val = digit;
					state = DIGIT2;
					}
				}
			}
			break;

			case DIGIT2: {
				if (byte == 'x') {
					if (val != 0) {
						PRINT_ERR("x Embedded in non-Hex number %d", val);
						return -2;
					}
					radix = 16;
					state = FIRSTHEX;
					break;
				} else {
					state = READNUM;
				}
			}

			/* FALLTHROUGH */

			case READNUM: {
				int digit = CvtAscii2Digit(byte, radix);
				if (digit >= 0) {
					val = val*radix + sign*digit;
				} else {
					// done. number found
					*ret_val = (uint32_t) val;
					//PRINT_DEBUG("done. val=%d (0x%x), count=%d", val, *ret_val, *count);
					return 0;
				}
			}
			break;

			case FIRSTHEX: {
				int digit = CvtAscii2Digit(byte, radix);
				if (digit >= 0) {
					val = digit;
					state = READNUM;
				} else {
					PRINT_ERR("No digits following 0x\n");
					return(-2);
				}
			}
			break;

			case COMMENT: {
				if (byte == 0x0a || byte == 0x0d)  {
					//PRINT_DEBUG("END COMMENT\n");
					state = comment_state;
				}
			}
			break;

			default:
				PRINT_CRIT("Unknown State %d\n", state);
				return -2;
		}
	}
	//PRINT_DEBUG("done. EOB count=%d", *count);
	return -1;
}
#undef DIGIT1
#undef DIGIT2
#undef FIRSTHEX
#undef READNUM
#undef COMMENT

/* Calibration write data +++*/
//////////////////////////////////////////////////////////////////////////////////////
//
// Program SPI Flash register init partition


#define STATE_REGBASE 0
#define STATE_NUMREG 1
#define STATE_REGVALS 2
#define STATE_ERROR 3

//////////////////////////////////////////////////////////////////////////////
//
// RETURNS
// Number of bytes rounded up to closest multiple of 4 (sizeof uint32_t)
//////////////////////////////////////////////////////////////////////////////
int ReadRegList(const uint8_t *p_in, int in_buf_len, uint8_t *p_out, int out_buf_len)
{
	int len = 0;
	uint32_t num_regs = 0;
	uint32_t val;
	int state = STATE_REGBASE;
	int ret;

	PRINT_FUNC("in len = %d, out_len = %d", in_buf_len, out_buf_len);

	if (out_buf_len < 2 || (out_buf_len % sizeof(uint32_t)) != 0) {
		PRINT_ERR("Bad buffer length for Register Update (%d)\n", out_buf_len);
		state = STATE_ERROR;
		goto ReadRegListDone;
	}
	out_buf_len -= 2; // remove 2 bytes for "end of file" marker

	while (len < out_buf_len) {
		ret = ReadNumFromBuf(&p_in, &in_buf_len, &val);

		if (ret == -1) { // end of buffer reached
			PRINT_DEBUG("EOB");
			goto ReadRegListDone;
		}
		if (ret == -2) {  	// parse error
			PRINT_ERR("Error processing SPI Register Initialization data\n");
			state = STATE_ERROR;
			goto ReadRegListDone;
		}

		switch (state) {
		case STATE_REGBASE: {
			if (val == 0x200) val = 0x82;
			if (val >= 0x000000ff) {
				PRINT_ERR("reg_base out of bounds (%d: 0x%x)\n", len, val);
				state = STATE_ERROR;
				goto ReadRegListDone;
			}
			if (len >= out_buf_len){
				PRINT_ERR("outbuf exceeded");
				state = STATE_ERROR;
				goto ReadRegListDone;
			}
			*p_out++ = (uint8_t)(val&0x000000ff);
			len++;
			state = STATE_NUMREG;
			}
			break;

		case STATE_NUMREG: {
			if (val > 0x000000ff) {
				PRINT_ERR("num_regs out of bounds (%d: 0x%x)\n", len, val);
				state = STATE_ERROR;
				goto ReadRegListDone;
			}
			if (len >= out_buf_len){
				PRINT_ERR("outbuf exceeded");
				state = STATE_ERROR;
				goto ReadRegListDone;
			}
			*p_out++ = (uint8_t)(val&0x000000ff);
			len++;
			if (val == 0) {// no regs to read
				state = STATE_REGBASE;
			} else {
				num_regs = val;
				state = STATE_REGVALS;
			}
			}
			break;

		case STATE_REGVALS: {
			if (val & 0x80000000) val = val & 0x0000ffff; 	// negative num trucate to uint16_t
			if (val > 0x0000ffff) {
				PRINT_ERR("reg_val %d is out of bounds (%d: 0x%x)\n", num_regs, len, val);
				state=STATE_ERROR;
				goto ReadRegListDone;
			}
			if ((len+2) > out_buf_len){
				PRINT_ERR("outbuf exceeded");
				state=STATE_ERROR;
				goto ReadRegListDone;
			}
			*p_out++ = (uint8_t)(val & 0xff);
			*p_out++ = (uint8_t)((val>>8) & 0xff);
			len += 2;
			if (--num_regs == 0) {
				state = STATE_REGBASE;
			}
			}
			break;

		default: PRINT_ERR("Unknown state in (%d)\n", state);
				state=STATE_ERROR;
				goto ReadRegListDone;
		}
	}

ReadRegListDone:
	if (state == STATE_ERROR) {
		PRINT_ERR("Formatting Error\n");
		return(-1);
	}
	if (state != STATE_REGBASE) {
		PRINT_ERR("Premature End of File (state = %d)\n", state);
		return(0);
	}

	// Put End of Data marker
	*p_out++ = 0;// reg_base == 0
	*p_out++ = 0;// num_regs == 0
	len += 2;
	PRINT_DEBUG("done. len=%d, len32=%ld", len, ((len+(sizeof(uint32_t)-1))/sizeof(uint32_t))*sizeof(uint32_t));
	return ((len+(sizeof(uint32_t)-1))/sizeof(uint32_t))*sizeof(uint32_t); // ceil(number of uint32_t) bytes
}
/* Calibration write data ---*/

#ifdef SUPPORT_FLASH
/*==========================================================================*/
/* FWUPDATE                                                                 */
/*==========================================================================*/

void fwupdate_cleanup(struct snt8100fsr *snt8100fsr) {
	PRINT_FUNC();
	if (sc_cmd != NULL) {
		memory_free(sc_cmd);
		sc_cmd = NULL;
	}
	if (snt8100fsr->fwupdate_file != NULL) {
		PRINT_DEBUG("Closing FWUpdate file");
		file_close(snt8100fsr->fwupdate_file);
		snt8100fsr->fwupdate_file = NULL;
	}
	snt8100fsr->fwupdate_tx_mtu = 0;
	snt8100fsr->active_sc_cmd = mc_no_command;
	return;
}

static inline int fwupdate_get_method(const char * s) {
	const char *s_base = s;
	const char *s_limit = s + PAGE_SIZE;
	int  ret = mc_fw_update;
	if (s == NULL) return ret;

	while (s < s_limit && *s != '\0' ) {
		s++;
	}

	if ((int)(s - s_base) >= 6 &&
		*s  == '\0' &&
		*(s-1) == 'h' &&
		*(s-2) == 's' &&
		*(s-3) == 'a' &&
		*(s-4) == 'l' &&
		*(s-5) == 'f' &&
		*(s-6) == '.') {
		PRINT_DEBUG("flash image");
		ret = mc_flash_update;
	} else {
		PRINT_DEBUG("update image");
	}
	return ret;
}


int fwupdate_send_mtu(struct snt8100fsr *snt8100fsr) {
	int ret = 0;

	PRINT_FUNC("tx_mtu %d", snt8100fsr->fwupdate_tx_mtu);

	if (snt8100fsr->fwupdate_tx_mtu) {
		int mtu_bytes = file_read(snt8100fsr->fwupdate_file,
						snt8100fsr->fwupdate_address,
						(uint8_t*) &sc_cmd->data[0],
						FW_BUFFER_SIZE);
		PRINT_DEBUG("fwupdate read address 0x%x, bytes %d", snt8100fsr->fwupdate_address, mtu_bytes);
		if (mtu_bytes < 0) {
			PRINT_CRIT("read of FWUpdate file failed %d", mtu_bytes);
			return mtu_bytes;
		}
		if (mtu_bytes && (mtu_bytes < FW_BUFFER_SIZE))
			mtu_bytes -= FWImageTrailerSize;

		if (mtu_bytes) {
			uint32_t checksum = snt_crc_checksum_buf((uint8_t*)&sc_cmd->data[0], mtu_bytes);
			int  	checksum_i = mtu_bytes/sizeof(uint32_t);
			PRINT_DEBUG("crc 0x%08x", checksum);
			sc_cmd->data[checksum_i] = checksum;
			sc_cmd->length = mtu_bytes + sizeof(checksum);
		} else {
			PRINT_DEBUG("Eof reached");
			sc_cmd->length = 0;
		}
		sc_cmd->minor = (snt8100fsr->fwupdate_tx_mtu == 1) ? min_data_last : min_data_frag;
		sc_cmd->address = snt8100fsr->fwupdate_address;
		PRINT_DEBUG("maj=%d, min=%d, addr=%x, len=%d",
					sc_cmd->major, sc_cmd->minor, snt8100fsr->fwupdate_address, sc_cmd->length);
		ret = sb_write_fifo(snt8100fsr,
							REGISTER_FIFO_COMMAND,
							SC_COMMAND_HEADER_SIZE + sc_cmd->length,
							sc_cmd);

		snt8100fsr->fwupdate_address += mtu_bytes;
		if (ret) {
			PRINT_CRIT("Unable to write to command fifo: %d", ret);
			snt8100fsr->active_sc_cmd = mc_no_command;
			return -1;
		}
	} else {
		PRINT_DEBUG("FWUpdate Complete");
		fwupdate_cleanup(snt8100fsr);
	}
	PRINT_DEBUG("done");
	return ret;
}


int enable_fwupdate(struct snt8100fsr *snt8100fsr, const char *fname) {
	/*
	* This method must not be called from our workerqueue. It will generally
	* be called from sysfs_fwupdate_store(). This method will wait for
	* interrupts and use the workerqueue to trigger when to send update frames.
	*/
	int ret = -1;

	PRINT_FUNC();

	/*
	* We must mutex lock before using sc_cmd to protect it from other
	* commands which use the same structure */
	MUTEX_LOCK(&snt8100fsr->sb_lock);

	if (snt8100fsr->active_sc_cmd != mc_no_command) {
		PRINT_CRIT("Cannot perform firmware/flash update. Command (%d) already active", snt8100fsr->active_sc_cmd);
		snt8100fsr->fwupdate_status = FWUPDATE_STATUS_DRIVER_ERR;
		goto cleanup;
	}

	if (snt8100fsr->fwupdate_file == NULL) {
		snt8100fsr->fwupdate_status = FWUPDATE_STATUS_OKAY;
		snt8100fsr->fwupdate_tot_mtu = 0;
		snt8100fsr->fwupdate_major = fwupdate_get_method(fname);
		snt8100fsr->fwupdate_address = 0;
		PRINT_DEBUG("Opening fwupdate file %s", fname);
		ret = file_open(fname, O_RDONLY, 0, &snt8100fsr->fwupdate_file);
		if (ret) {
			PRINT_DEBUG("Unable to open file %s, error %d", fname, ret);
			snt8100fsr->fwupdate_status = FWUPDATE_STATUS_FILE_ERR;
			goto cleanup;
		}
		ret = file_size(snt8100fsr->fwupdate_file, &snt8100fsr->fwupdate_size);
		if (ret) {
			PRINT_DEBUG("Unable to get file size error %d", ret);
			snt8100fsr->fwupdate_status = FWUPDATE_STATUS_FILE_ERR;
			goto cleanup;
		}
		// num tx packets = ceil(filesize-Trailer/BUFF_SIZE) + 1. Last one is for "done" indicator
		snt8100fsr->fwupdate_tx_mtu = ((snt8100fsr->fwupdate_size-FWImageTrailerSize+FW_BUFFER_SIZE-1) / FW_BUFFER_SIZE) + 1;
		snt8100fsr->fwupdate_tot_mtu = snt8100fsr->fwupdate_tx_mtu;
		PRINT_DEBUG("fwupdate file size %d, num mtu %d", snt8100fsr->fwupdate_size, snt8100fsr->fwupdate_tx_mtu);

	} else {
		PRINT_NOTICE("FWUpdate already enabled");
		goto cleanup;
	}

	if (sc_cmd == NULL) {
		sc_cmd = memory_allocate(sizeof(*sc_cmd), GFP_DMA);
	}
	if (sc_cmd == NULL) {
		PRINT_DEBUG("Unable to allocate sc cmd for fwupdate");
		snt8100fsr->fwupdate_status = FWUPDATE_STATUS_DRIVER_ERR;
		goto cleanup;
	}

	sc_cmd->magic = SC_MAGIC_NUMBER;
	sc_cmd->major = snt8100fsr->fwupdate_major;
	sc_cmd->address = snt8100fsr->fwupdate_address;
	sc_cmd->length = 0;
	sc_cmd->trans_id = get_time_in_ms();

	snt8100fsr->active_sc_cmd = snt8100fsr->fwupdate_major;
	ret = fwupdate_send_mtu(snt8100fsr);
	if (ret) {
		PRINT_CRIT("FWUpdate send failed %d", ret);
		snt8100fsr->fwupdate_status = FWUPDATE_STATUS_ABORT;
		fwupdate_cleanup(snt8100fsr);
	}

cleanup:
	PRINT_DEBUG("done");
	mutex_unlock(&snt8100fsr->sb_lock);
	return ret;
}


static void fwupdate_rsp_ind(struct snt8100fsr *snt8100fsr) {
	int ret;

	PRINT_FUNC("Enter");

	/*
	* We must mutex lock before using sc_cmd to protect it from other
	* commands which use the same structure
	*/
	MUTEX_LOCK(&snt8100fsr->sb_lock);

	ret = snt_read_sc_rsp(snt8100fsr);
	if (ret) {
		PRINT_CRIT("Read cmd Fifo rsp failed");
		snt8100fsr->fwupdate_status = FWUPDATE_STATUS_ABORT;
		goto err_exit;
	}

	PRINT_DEBUG("FWUpdate mtu rsp maj=%d, min=%d, crc=0x%08x, address=0x%08x",
					sc_cmd->major, sc_cmd->minor, sc_cmd->data[0], sc_cmd->address);

	if (sc_cmd->major != snt8100fsr->fwupdate_major) {
		PRINT_CRIT("Unexpected cmd response %d", sc_cmd->major);
		snt8100fsr->fwupdate_status = FWUPDATE_STATUS_DATA_BAD;
		goto err_exit;
	}
	if (sc_cmd->minor == min_data_bad) {
		PRINT_CRIT("Invalid file format");
		snt8100fsr->fwupdate_status = sc_cmd->minor;
		goto err_exit;
	}
	if (sc_cmd->minor == min_bad_part1) {
		PRINT_CRIT("Flash partition error");
		snt8100fsr->fwupdate_status = sc_cmd->minor;
		goto err_exit;
	}
	if (sc_cmd->minor == min_not_supported) {
		PRINT_CRIT("Command not supported by firmware");
		snt8100fsr->fwupdate_status = sc_cmd->minor;
		goto err_exit;
	}

	snt8100fsr->fwupdate_tx_mtu--;
	ret = fwupdate_send_mtu(snt8100fsr);
	if (ret) {
		PRINT_CRIT("FWUpdate send error %d", ret);
		snt8100fsr->fwupdate_status = FWUPDATE_STATUS_ABORT;
		goto err_exit;
	}

cleanup:
	mutex_unlock(&snt8100fsr->sb_lock);
	PRINT_DEBUG("done");
	return;

err_exit:
	fwupdate_cleanup(snt8100fsr);
	goto cleanup;
}
#endif // SUPPORT_FLASH

#undef STATE_REGBASE
#undef STATE_NUMREG
#undef STATE_REGVALS
#undef STATE_ERROR

static int write_flash_reg_part_send_pkt(struct snt8100fsr *snt8100fsr) {
	int ret;

	if (snt8100fsr->frp_cur_size) {

		int pkt_len = (snt8100fsr->frp_cur_size < (int)sizeof(sc_cmd->data)) 
						? snt8100fsr->frp_cur_size : (int)sizeof(sc_cmd->data);
		snt8100fsr->frp_cur_size -= pkt_len;

		if (!sc_cmd) {
			PRINT_CRIT("sc_cmd NULL");
			goto errexit;
		}

		if (snt8100fsr->regs_cmd_id != mc_update_regs && 
			snt8100fsr->regs_cmd_id != mc_reg_script) {
			PRINT_CRIT("invalid reg script cmd %d", snt8100fsr->regs_cmd_id);
			goto errexit;
		}

		PRINT_DEBUG("Sending %d byte Frp packet", pkt_len);

		memcpy(sc_cmd->data, 
				((uint8_t*)snt8100fsr_g->reg_part_buf) + snt8100fsr->frp_tx_offset,
				pkt_len);

		// request write of flash register init partition
		sc_cmd->magic = SC_MAGIC_NUMBER;
		sc_cmd->address = snt8100fsr->frp_tx_offset;
		sc_cmd->length = pkt_len;
		sc_cmd->major = snt8100fsr->regs_cmd_id;
		sc_cmd->minor = (snt8100fsr->frp_cur_size) ? min_data_frag : min_data_last;
		sc_cmd->trans_id = get_time_in_ms();// unique number

		ret = sb_write_fifo(snt8100fsr,
							REGISTER_FIFO_COMMAND,
								SC_COMMAND_HEADER_SIZE + sc_cmd->length,
							sc_cmd);

		if (ret) {
			PRINT_CRIT("Unable to write to command fifo: %d", ret);
			goto errexit;
		}
		snt8100fsr->active_sc_cmd = mc_update_regs;
		snt8100fsr->frp_tx_offset += pkt_len;
	} else {
		if (sc_cmd) {
			memory_free(sc_cmd);
			sc_cmd = NULL;
		}
		if (snt8100fsr_g->reg_part_buf) {
			memory_free(snt8100fsr_g->reg_part_buf);
			snt8100fsr_g->reg_part_buf = NULL;
		}
		snt8100fsr->active_sc_cmd = mc_no_command;
	}
	return(0);
errexit:
	return(-1);
}


static void write_flash_reg_part_rsp(struct snt8100fsr *snt8100fsr) {
	int ret;

	PRINT_FUNC();

	/*
	* We must mutex lock before using sc_cmd to protect it from other
	* commands which use the same structure
	*/
	MUTEX_LOCK(&snt8100fsr->sb_lock);

	ret = snt_read_sc_rsp(snt8100fsr);
	if (ret) {
		PRINT_CRIT("Read cmd Fifo rsp failed");
		goto errexit;
	}

	ret = write_flash_reg_part_send_pkt(snt8100fsr);
	if (ret < 0) {
		PRINT_CRIT("ERROR while processing write FRP");
		goto errexit;
	}

cleanup:
	mutex_unlock(&snt8100fsr->sb_lock);
	PRINT_DEBUG("done");
	return;

errexit:
	if (sc_cmd) {
		memory_free(sc_cmd);
		sc_cmd = NULL;
	}
	if (snt8100fsr_g->reg_part_buf) {
		memory_free(snt8100fsr_g->reg_part_buf);
		snt8100fsr_g->reg_part_buf = NULL;
	}

	snt8100fsr->active_sc_cmd = mc_no_command;
	goto cleanup;
}

void enable_write_flash_reg_part_req(struct snt8100fsr *snt8100fsr,
                                       const char *buf,
                                       size_t count,
                                       int sc_maj_id) {
	int ret;
	uint16_t reg;

	PRINT_FUNC("count = %zu", count);

	/*
	* We must mutex lock before using sc_cmd to protect it from other
	* commands which use the same structure */
	MUTEX_LOCK(&snt8100fsr->sb_lock);

	if (snt8100fsr->active_sc_cmd != mc_no_command) {
		PRINT_CRIT("Cannot perform firmware/flash update. Command (%d) already active", snt8100fsr->active_sc_cmd);
		goto errexit;
	}
	
	/** check max size of flash regsiter partition  */
	snt8100fsr->frp_max_size = SC_DATA_PAYLOAD_BYTES;
	ret = sb_read_fifo(snt8100fsr,
					REGISTER_FRP_MAX_SIZE,
					2,
					&reg);
	if (ret) {
		PRINT_DEBUG("sb_read_fifo() max frp size failed");
	} else {
		if (reg != 0)
			snt8100fsr->frp_max_size = reg;
	}
	PRINT_DEBUG("Max FRP size = %d", snt8100fsr->frp_max_size);

	// allocate buffer to get Flash Register Partition
	if (snt8100fsr_g->reg_part_buf != NULL) {
		memory_free(snt8100fsr_g->reg_part_buf);
	}
	snt8100fsr_g->reg_part_buf = (uint16_t*)memory_allocate(REG_PART_MAX_LEN, GFP_DMA);
	if (snt8100fsr_g->reg_part_buf == NULL) {
		PRINT_CRIT("memory_allocate(%d) failed", REG_PART_MAX_LEN);
		goto errexit;
	}

	if (sc_cmd == NULL) {
		sc_cmd = memory_allocate(sizeof(*sc_cmd), GFP_DMA);
	}

	if (sc_cmd == NULL) {
		PRINT_CRIT("Unable to allocate sc cmd for flash reg part req");
		goto errexit;
	}

	if (sc_cmd == NULL) {
		PRINT_CRIT("Unable to allocate sc cmd for flash reg part req");
		goto errexit;
	}
	/* cache which flavor of regscript cmd being done */
	snt8100fsr->regs_cmd_id = sc_maj_id;

	snt8100fsr->frp_cur_size = ReadRegList(buf, count, (uint8_t*) snt8100fsr_g->reg_part_buf, snt8100fsr->frp_max_size);
	if (snt8100fsr->frp_cur_size <= 0) {
		PRINT_CRIT("Frp exceeds maximum allowed");
		goto errexit;
	}
	PRINT_DEBUG("frp total size = %d", snt8100fsr->frp_cur_size);
	snt8100fsr->frp_tx_offset = 0;

	ret = write_flash_reg_part_send_pkt(snt8100fsr);
	if (ret < 0) {
		PRINT_CRIT("ERROR while processing write FRP");
		goto errexit;
	}

cleanup:
	mutex_unlock(&snt8100fsr->sb_lock);
	PRINT_DEBUG("done");
	return;

 errexit:
	if (snt8100fsr->active_sc_cmd == mc_no_command && sc_cmd) {
		memory_free(sc_cmd);
		sc_cmd = NULL;
	}
	if (snt8100fsr_g->reg_part_buf) {
		memory_free(snt8100fsr_g->reg_part_buf);
		snt8100fsr_g->reg_part_buf = NULL;
	}

	goto cleanup;
}

/* Calibration write data +++*/
int enable_boot_init_reg_with_file(char *fname)
{
	struct file* boot_init_file;
	char buffer[BOOT_INIT_BUFFER_SIZE];
	int ret, numRead, boot_init_file_size;

	PRINT_FUNC();

	ret = file_open(fname, O_RDONLY, 0, &boot_init_file);
	if(ret) {
		PRINT_INFO("Unable to open file %s, error %d",
						BOOT_INIT_REG_LOCATION, ret);
		return -1;
	}

	ret = file_size(boot_init_file, &boot_init_file_size);
	if (ret) {
		PRINT_INFO("Unable to get file size error %d", ret);
		return -1;
	}

	memset(buffer, 0, sizeof(buffer));
	numRead = file_read(boot_init_file, 0, buffer, boot_init_file_size);
	if (numRead <= 0) {
		PRINT_INFO("Bad read from boot init file");
		return -1;
	}
	enable_boot_init_reg_req(snt8100fsr_g, buffer, (size_t)numRead);

	PRINT_DEBUG("done.");
	return 0;
}

#define OFFSET_REG_ID 0
#define OFFSET_REG_LEN 1
#define OFFSET_LSB 2
#define OFFSET_MSB 3
void enable_boot_init_reg_req(struct snt8100fsr *snt8100fsr,
                                       const char *buf,
                                       size_t count) {
	int ret;
	uint8_t buf_data[2048] = {0};
	uint16_t reg_data[128] = {0};
	int reg_id, len, dataIdx, next_reg_idx = 0;
	int times=0;
	int timeout = 100; /* 100 ms */
	struct register_enable reg_enable;

	PRINT_FUNC();

	ret = ReadRegList(buf, count, (uint8_t*) &buf_data, sizeof(buf_data));
	PRINT_INFO("buf = %s, count = %zu", buf, count);

/*
	for (i=0; i < sizeof(buf_data); i++) {
	PRINT_INFO("[austin] 0x%X", buf_data[i]);
	}
*/
	reg_id = buf_data[OFFSET_REG_ID];
	len	= buf_data[OFFSET_REG_LEN];
	
	MUTEX_LOCK(&snt8100fsr->sb_lock);

	/* Parsing buf_data to reg_id/len/reg_data only when {reg_id} and {len}
	* are not equal to zero, maybe there is better way to stop the while-loop */
	while ((reg_id != 0x00) && (len != 0x00)) {
		PRINT_INFO("Enter while: reg_id = %d, len = %d, times = %d", reg_id, len, times);
		for(dataIdx = 0; dataIdx < len; dataIdx++) {
			reg_data[dataIdx] = (buf_data[next_reg_idx+OFFSET_MSB+dataIdx*2] << 8) |
								buf_data[next_reg_idx+OFFSET_LSB+dataIdx*2];

			PRINT_DEBUG("{%d} (%d / %d) [%x]",
					reg_id, dataIdx, len, reg_data[dataIdx]);
		}
		//PRINT_INFO("Before sb_write_fifo, reg_id = %d, len = %d, times = %d", reg_id, len, times);
		snt8100fsr_g->get_reg_id  = 0x2a;
		snt8100fsr_g->get_reg_buf = (uint16_t*)memory_allocate(2, GFP_DMA);
		if (snt8100fsr_g->get_reg_buf == NULL) {
			PRINT_CRIT("memory_allocate(%d) failed", len);
			return;
		}
		
		ret = sb_read_fifo(snt8100fsr_g,
			snt8100fsr_g->get_reg_id,
			2,
			snt8100fsr_g->get_reg_buf);
		while((snt8100fsr_g->get_reg_buf[0] & (1<<15)) != 0) {
			ret = sb_read_fifo(snt8100fsr_g,
				snt8100fsr_g->get_reg_id,
				2,
				snt8100fsr_g->get_reg_buf);
			if (ret) {
				PRINT_CRIT("Unable to read from registers: %d", ret);
				if (snt8100fsr_g->get_reg_buf != NULL)
					memory_free(snt8100fsr_g->get_reg_buf);
				snt8100fsr_g->get_reg_buf = NULL;
				snt8100fsr_g->get_reg_num = 0;
				return;
			}
			if(times < timeout){
				if(times%10 == 0){
					PRINT_INFO("Wait Busy config bank fail!");
				}
				times++;
				msleep(1);
			}else{
				PRINT_INFO("Wait Busy config bank fail!");
				break;
			}
		}
		ret = sb_write_fifo(snt8100fsr_g,
				reg_id,
				len*2,
				reg_data);
		if (ret) {
			PRINT_CRIT("Unable to write to registers: %d, reg: 0x%02X", ret, (uint16_t)reg_id);
			break;
		}
		next_reg_idx += (2 + len*2);
		reg_id = buf_data[next_reg_idx+OFFSET_REG_ID];
		len = buf_data[next_reg_idx+OFFSET_REG_LEN];
		//PRINT_INFO("End while, reg_id = %d, len = %d, times = %d", reg_id, len, times);
		times++;
	}

	mutex_unlock(&snt8100fsr->sb_lock);

	PRINT_DEBUG("write 0x1=1 to prevent cal data bug");
	reg_enable.enable = 1;
	ret = write_register(snt8100fsr, REGISTER_ENABLE, &reg_enable);
	if (ret) {
		PRINT_CRIT("sb_read_register() failed");
	}
	PRINT_INFO("done");
	
	return;
}

extern int aw8697_trig_control(int num, bool enable);
static void Grip_check_K_and_frame(){
	int ret;
	uint16_t disable_K = 1; //enable_bang = 1;

	/* OPEN 3E for No Scaling Bar */
	ret = write_register (snt8100fsr_g,
		0x3e,
		 &disable_K);

	/* fw version 200.11.22 will set 0x38 to 3 by default, but gesture sensitive will be degraded */
	/* After 200.11.20 fw version, default 0x38 = 0x3
	Therefore, cancel the behavior that set 0x38 to 1 at initialization
		PRINT_INFO("Enable Bang: 0x38 1");
		ret = write_register (snt8100fsr_g,
				0x38,
				&enable_bang);
	*/
	snt8100fsr_g->frame_rate = 100;
	Into_DeepSleep_fun();
	PRINT_INFO("Grip Initial Done! fw_loading_status=%d", snt8100fsr_g->grip_fw_loading_status);
	if(aw8697_trig_control(1, 1)==0)
		PRINT_INFO("Enable vib trig1");
	else
		PRINT_INFO("Failed to enable vib trig1");
	
	if(aw8697_trig_control(2, 1)==0)
		PRINT_INFO("Enable vib trig2");
	else
		PRINT_INFO("Failed to enable vib trig2");
	finish_boot = 1;
}
int enable_set_sys_param(struct snt8100fsr *snt8100fsr, int id, int val) {
	int ret = -1;

	PRINT_FUNC("%d %d", id, val);

	/*
	* We must mutex lock before using sc_cmd to protect it from other
	* commands which use the same structure */
	MUTEX_LOCK(&snt8100fsr->sb_lock);

	snt8100fsr->set_sys_param_id = id;
	snt8100fsr->set_sys_param_val = val;
	snt8100fsr->set_sys_param_status= -1;

	if (snt8100fsr->active_sc_cmd != mc_no_command) {
		PRINT_CRIT("Cannot perform SetSysParam. Command (%d) already active", snt8100fsr->active_sc_cmd);
		goto errexit;
	}

	if (sc_cmd == NULL) {
		sc_cmd = memory_allocate(sizeof(*sc_cmd), GFP_DMA);
	}

	if (sc_cmd == NULL) {
		PRINT_CRIT("Unable to allocate sc cmd for set_sys_param");
		goto errexit;
	}

	PRINT_INFO("enable_set_sys_param: set active_sc_cmd to mc_set_sys_param = %d", mc_set_sys_param);
	// request write of flash register init partition
	sc_cmd->magic = SC_MAGIC_NUMBER;
	sc_cmd->address = snt8100fsr_g->set_sys_param_id;
	sc_cmd->length = 4;
	sc_cmd->major = mc_set_sys_param;
	sc_cmd->minor = min_ok;
	sc_cmd->trans_id = get_time_in_ms();	// unique number
	sc_cmd->data[0] = snt8100fsr_g->set_sys_param_val;

	ret = sb_write_fifo(snt8100fsr,
						REGISTER_FIFO_COMMAND,
						SC_COMMAND_HEADER_SIZE + sc_cmd->length,
						sc_cmd);

	if (ret) {
		PRINT_CRIT("Unable to write to command fifo: %d", ret);
		goto errexit;
	}
	// SUCESS! update state and prime for rsp indication
	ret = 0;
	snt8100fsr->active_sc_cmd = mc_set_sys_param;
	PRINT_DEBUG("SetSysParam Req");
	up(&snt8100fsr->sc_wf_rsp_req); // post req for rsp event	

cleanup:
	mutex_unlock(&snt8100fsr->sb_lock);
	PRINT_DEBUG("done");
	return ret;

 errexit:
	if (snt8100fsr->active_sc_cmd == mc_no_command && sc_cmd) {
		memory_free(sc_cmd);
		sc_cmd = NULL;
	}

	goto cleanup;
}

static void set_sys_param_rsp(struct snt8100fsr *snt8100fsr) {
	int ret;

	PRINT_FUNC();

	/*
	* We must mutex lock before using sc_cmd to protect it from other
	* commands which use the same structure
	*/
	MUTEX_LOCK(&snt8100fsr->sb_lock);

	ret = snt_read_sc_rsp(snt8100fsr);
	if (ret) {
		PRINT_CRIT("Read cmd Fifo rsp failed");
		goto cleanup;
	}

	// Verify return
	if (sc_cmd->minor != min_ok) {
		PRINT_ERR("ERROR! return %d on update_regs_rsp", sc_cmd->minor);
		goto cleanup;
	}

	snt8100fsr->set_sys_param_status = sc_cmd->data[0];


cleanup:
	if (sc_cmd) {
		memory_free(sc_cmd);
		sc_cmd = NULL;
	}
	snt8100fsr->active_sc_cmd = mc_no_command;
	
	// respond to requestor
	while(down_trylock(&snt8100fsr->sc_wf_rsp_req)==0) {
		PRINT_DEBUG("SetSysParam Rsp");
		up(&snt8100fsr_g->sc_wf_rsp);
	}
	mutex_unlock(&snt8100fsr->sb_lock);
	PRINT_DEBUG("done");
	return;
}




void enable_get_sys_param(struct snt8100fsr *snt8100fsr, int val) {
	int ret;

	PRINT_FUNC("%d", val);

	/*
	* We must mutex lock before using sc_cmd to protect it from other
	* commands which use the same structure */
	MUTEX_LOCK(&snt8100fsr->sb_lock);

	snt8100fsr->get_sys_param_id = val;
	snt8100fsr->get_sys_param_status = -1;

	if (snt8100fsr->active_sc_cmd != mc_no_command) {
		PRINT_CRIT("Cannot perform GetSysParam. Command (%d) already active", snt8100fsr->active_sc_cmd);
		goto errexit;
	}

	if (snt8100fsr->get_sys_param_cmd != NULL) {
		memory_free(snt8100fsr->get_sys_param_cmd);
		snt8100fsr->get_sys_param_cmd = NULL;
	}

	if (sc_cmd == NULL) {
		sc_cmd = memory_allocate(sizeof(*sc_cmd), GFP_DMA);
	}

	if (sc_cmd == NULL) {
		PRINT_CRIT("Unable to allocate sc cmd for get_sys_param");
		goto errexit;
	}

	// request write of flash register init partition
	sc_cmd->magic = SC_MAGIC_NUMBER;
	sc_cmd->address = snt8100fsr_g->get_sys_param_id;
	sc_cmd->length = 0;
	sc_cmd->major = mc_get_sys_param;
	sc_cmd->minor = min_ok;
	sc_cmd->trans_id = get_time_in_ms();	// unique number

	ret = sb_write_fifo(snt8100fsr, REGISTER_FIFO_COMMAND, SC_COMMAND_HEADER_SIZE + sc_cmd->length, sc_cmd);

	if (ret) {
		PRINT_CRIT("Unable to write to command fifo: %d", ret);
		goto errexit;
	}
	// success! update status and prime for response
	snt8100fsr->active_sc_cmd = mc_get_sys_param;
	PRINT_DEBUG("GetSysParam Req");
	up(&snt8100fsr->sc_wf_rsp_req); // post req for rsp event

cleanup:
	mutex_unlock(&snt8100fsr->sb_lock);
	PRINT_DEBUG("done");
	return;

errexit:
	if (snt8100fsr->active_sc_cmd == mc_no_command && sc_cmd) {
		memory_free(sc_cmd);
		sc_cmd = NULL;
	}

	goto cleanup;
}

static void get_sys_param_rsp(struct snt8100fsr *snt8100fsr) {
	int ret;

	PRINT_FUNC();

	/*
	* We must mutex lock before using sc_cmd to protect it from other
	* commands which use the same structure
	*/
	MUTEX_LOCK(&snt8100fsr->sb_lock);

	ret = snt_read_sc_rsp(snt8100fsr);
	if (ret) {
		PRINT_CRIT("Read cmd Fifo rsp failed");
		goto cleanup;
	}

	// Verify return
	if (sc_cmd->minor != min_ok) {
		PRINT_ERR("ERROR! return %d on update_regs_rsp", sc_cmd->minor);
		goto cleanup;
	}

	// event log is special case
	if (sc_cmd->address == get_sys_param_event_log) {
		if (snt8100fsr->event_log_file != NULL){
			file_write(snt8100fsr->event_log_file,
			0,
			(uint8_t*)sc_cmd->data,
			sc_cmd->length);
		}
		goto cleanup;
	} else {
		snt8100fsr->get_sys_param_status = sc_cmd->data[0];
		snt8100fsr->get_sys_param_val = sc_cmd->data[1];

		// deal with Gets that are greater than 4 bytes.
		if (sc_cmd->length > 8) {
			snt8100fsr->get_sys_param_cmd = sc_cmd;
			sc_cmd = NULL;
		}
	}

cleanup:
	if (snt8100fsr->event_log_file != NULL) {
		file_close(snt8100fsr->event_log_file);
		snt8100fsr->event_log_file = NULL;
	}
	if (sc_cmd) {
		memory_free(sc_cmd);
		sc_cmd = NULL;
	}
	snt8100fsr->active_sc_cmd = mc_no_command;

	// respond to requestor
	while(down_trylock(&snt8100fsr->sc_wf_rsp_req)==0) {
		PRINT_DEBUG("GetSysParam Rsp");
		up(&snt8100fsr_g->sc_wf_rsp);
	}
	mutex_unlock(&snt8100fsr->sb_lock);
	PRINT_DEBUG("done");
	return;
}

static void spurious_cmd_event(struct snt8100fsr *snt8100fsr) {
	int ret;

	PRINT_FUNC();

	/*
	* We must mutex lock before using sc_cmd to protect it from other
	* commands which use the same structure
	*/
	MUTEX_LOCK(&snt8100fsr->sb_lock);

	if (sc_cmd == NULL) {
		sc_cmd = memory_allocate(sizeof(*sc_cmd), GFP_DMA);
	}

	if (sc_cmd == NULL) {
		PRINT_CRIT("Unable to allocate sc cmd for spurious cmd");
		goto cleanup;
	}

	ret = snt_read_sc_rsp(snt8100fsr);
	if (ret) {
		PRINT_CRIT("Read cmd Fifo rsp failed");
		goto cleanup;
	}

cleanup:
	if (sc_cmd) {
		memory_free(sc_cmd);
		sc_cmd = NULL;
	}
	mutex_unlock(&snt8100fsr->sb_lock);
	PRINT_DEBUG("done");
	return;
}

void sc_flush(struct snt8100fsr *snt8100fsr)
{
	struct register_actions reg_actions;
	int ret;

	PRINT_FUNC();
	// reset snt8100fsr sc state machines
	memset((void*)&reg_actions, 0, sizeof(reg_actions));
	reg_actions.fcmd = 1;
	reg_actions.fstr = 1;
	ret = sb_write_fifo(snt8100fsr, REGISTER_ACTIONS, sizeof(reg_actions), &reg_actions);
	if (ret) {
		PRINT_CRIT("write_register() failed");
	}
	PRINT_DEBUG("done");
}

void sc_cleanup(struct snt8100fsr *snt8100fsr)
{
	PRINT_FUNC();

	MUTEX_LOCK(&snt8100fsr->sb_lock);

	sc_flush(snt8100fsr);

	d1test_cleanup(snt8100fsr);
	no_touch_logging_cleanup(snt8100fsr);

	#ifdef SUPPORT_FLASH
	fwupdate_cleanup(snt8100fsr);
	#endif

	frame_logging_cleanup(snt8100fsr);

	event_logging_cleanup(snt8100fsr);

	mutex_unlock(&snt8100fsr->sb_lock);
	PRINT_DEBUG("done");
	return;
}
