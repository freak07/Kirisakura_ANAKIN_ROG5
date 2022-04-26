/*****************************************************************************
* File: device.h
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
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/semaphore.h>

#ifndef DEVICE_H
#define DEVICE_H

/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/
static const char * const pctl_names[] = {
    "snt_hostirq_active",
    "snt_reset_reset",
    "snt_reset_active",
};

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/
enum {
  BUS_TYPE_NONE = 0,
  BUS_TYPE_SPI  = 1,
  BUS_TYPE_I2C  = 2,
  BUS_TYPE_MAX,
};

enum{
  GRIP_I2C_PROBE = 0,
  GRIP_CREATE_FIRM_WQ  = 1,
  GRIP_START_LOAD_FIRM  = 2,
  GRIP_LOAD_FIRM_FAIL  = 3,
  GRIP_LOAD_FIRM_DONE  = 4,
  GRIP_START_EVENT_MONITOR = 5,
  GRIP_I2C_PROBE_DONE = 6,
};

typedef struct grip_asus_struct {
	int (*grip_regulator_init)(void);
	int (*grip_regulator_enable)(void);
	int (*grip_regulator_disable)(void);
}grip_asus_struct;

struct snt8100fsr {
    struct device           *dev;
    struct spi_device       *spi;
    struct i2c_client       *i2c;
    struct pinctrl          *snt_pinctrl;
    struct pinctrl_state    *pinctrl_state[ARRAY_SIZE(pctl_names)];
    int                     hostirq_gpio;
    int                     rst_gpio;
    int                     bus_type;

    // Rate to sample frames from the sensor
    int                     frame_rate;

    //Rate when device suspend (e.g. probe) is called
    int                     suspended_frame_rate;

    //// true if this i2c dev is to wake up the chip
    bool                    wake_i2c_device;
    struct class            *class;
    struct device           *device;

    // SysFS lock for sysfs calls
    struct mutex            track_report_sysfs_lock;

    // Serial bus lock (i2c/spi)
    struct mutex            sb_lock;
    u32                     spi_freq_khz;
    struct track_report     *track_reports;
    uint16_t                track_reports_count;
    uint16_t                track_reports_frame;

    // Frame streams left to receive from the hardware device for logging
    uint32_t                frame_stream_count;

    uint32_t                get_sys_param_id;
    uint32_t                get_sys_param_val;
    uint32_t                get_sys_param_status;
    struct   sc_command *   get_sys_param_cmd;
    uint32_t                set_sys_param_id;
    uint32_t                set_sys_param_val;
    uint32_t                set_sys_param_status;
    struct semaphore        sc_wf_rsp_req;
    struct semaphore        sc_wf_rsp;

    int                     op_profile;

    // fwupdate state variables
    struct file*    fwupdate_file;
    int             fwupdate_size;
    int             fwupdate_tx_mtu;
    int             fwupdate_tot_mtu;
    int             fwupdate_major;
    int             fwupdate_address;
    int             fwupdate_status;

    // read_flash_reg_partr variables
    uint16_t    *   reg_part_buf;
    int			 frp_max_size;;
    int			 frp_cur_size;;
    int			 frp_tx_offset;
	
    // update_regs/reg_script variables
    int             regs_cmd_id;

    // event log variables
    struct file *   event_log_file;

    // get_reg variables
    uint16_t   *    get_reg_buf;
    int             get_reg_id;
    int             get_reg_num;

    int             active_sc_cmd;
	
    // DYNAMIC_PWR_CTL state variables
    struct semaphore wake_req;
    struct semaphore wake_rsp;
    int              wake_rsp_result;
    int              enable_dpc_flag;
	
    //threshold when apply in IMS's defined mode
    int                     pressure_threshold;

    //use to enable  touch report
    int			    en_demo;
	
    //use to enable  touch report
    int			    en_sensor_evt;
    struct mutex            ap_lock;
    struct mutex            tap_lock;
    struct mutex            IRQ_WAKE_SLEEP_LOCK;
	
    // Wake lock
    struct wakeup_source         *snt_wakelock;

    // Driver State
    int         snt_state;

    // deep sleep time
    int	sleep_ms_time;

    //Trigger pin
    int snt_trig0_num;
    int snt_trig1_num;
    int snt_trig2_num;
    int snt_trig3_num;

    int snt_id_num;
    bool Recovery_Flag;          /* Recovery when grip is access by IMS before firmware loading done */

	/* use to probe power */
	grip_asus_struct *mgrip_2v8_asus_func;
	grip_asus_struct *mgrip_1v8_asus_func;
	grip_asus_struct *mgrip_1v2_asus_func;
	
	/* 
	-1: first download firmware
    1: set 1 when reset start
    0: when firmware download finished, set flag to 0
	*/
	int chip_reset_flag;

	bool suspend_status;
	
	bool grip_fw_loading_status;

	int stuck_retry_count;
	bool stuck_flag;
	int stuck_retry_limit;
	int fw_failed_count;
};

enum{
  GRIP_SHUTDOWN = 0,   //Befor Load firmware
  GRIP_WAKEUP  = 1,
  GRIP_DEEPSLEEP  = 2,
  GRIP_RESET = 3,
};

enum{
  GRIP_TAP_L1 = 1,   
  GRIP_TAP_R1  = 2,
  GRIP_TAP_L2 = 3,
  GRIP_TAP_R2  = 4,
  GRIP_SWIPE_L  = 5,
  GRIP_SWIPE_R  = 6,
  GRIP_SQUEEZE_SHORT  = 7,
  GRIP_SQUEEZE_LONG  = 8,
  GRIP_SQUEEZE_START  = 9,
  GRIP_SQUEEZE_CANCEL  = 10,
  GRIP_SQUEEZE_END  = 11,
  GRIP_SLIDE_L  = 12,
  GRIP_SLIDE_R  = 13,
};

enum{
  GRIP_FST_FW_DL = -1,   //Befor Load firmware
  GRIP_RST_FW_DL  = 0,
  GRIP_FW_DL_END  = 1
};

struct grip_status {
	/* Properties of type `int' */
	int G_EN;
	int G_DPC_STATUS;
	int G_RAW_EN;
	int G_SQUEEZE_EN[2];
	int G_SQUEEZE_FORCE[2];
	int G_SQUEEZE_SHORT[2];
	int G_SQUEEZE_LONG[2];
	int G_SQUEEZE_UP_RATE[2];
	int G_SQUEEZE_UP_TOTAL[2];
	int G_SQUEEZE_DROP_RATE[2];
	int G_SQUEEZE_DROP_TOTAL[2];
	int G_SQUEEZE1_EN;
	int G_SQUEEZE1_FORCE;
	int G_SQUEEZE1_SHORT;
	int G_SQUEEZE1_LONG;
	int G_SQUEEZE1_UP_RATE;
	int G_SQUEEZE1_UP_TOTAL;
	int G_SQUEEZE1_DROP_RATE;
	int G_SQUEEZE1_DROP_TOTAL;
	int G_SQUEEZE2_EN;
	int G_SQUEEZE2_FORCE;
	int G_SQUEEZE2_SHORT;
	int G_SQUEEZE2_LONG;
	int G_SQUEEZE2_UP_RATE;
	int G_SQUEEZE2_UP_TOTAL;
	int G_SQUEEZE2_DROP_RATE;
	int G_SQUEEZE2_DROP_TOTAL;
	int G_TAP_EN[4];
	int G_TAP_FORCE[4];
	int G_TAP_VIB_EN[4];
	int G_TAP_REST_EN[4];
	int G_TAP_MIN_POS[4];
	int G_TAP_MAX_POS[4];
	int G_TAP_SLOPE_WINDOW[4];
	int G_TAP_SLOPE_RELEASE_FORCE[4];
	int G_TAP_SLOPE_TAP_FORCE[4];
	int G_TAP_DELTA_RELEASE_FORCE[4];
	int G_TAP_DELTA_TAP_FORCE[4];
	int G_TAP1_EN;
	int G_TAP1_FORCE;
	int G_TAP1_VIB_EN;
	int G_TAP1_REST_EN;
	int G_TAP1_MIN_POS;
	int G_TAP1_MAX_POS;
	int G_TAP1_SLOPE_WINDOW;
	int G_TAP1_SLOPE_RELEASE_FORCE;
	int G_TAP1_SLOPE_TAP_FORCE;
	int G_TAP1_DELTA_RELEASE_FORCE;
	int G_TAP1_DELTA_TAP_FORCE;
	int G_TAP2_EN;
	int G_TAP2_FORCE;
	int G_TAP2_VIB_EN;
	int G_TAP2_REST_EN;
	int G_TAP2_MIN_POS;
	int G_TAP2_MAX_POS;
	int G_TAP2_SLOPE_WINDOW;
	int G_TAP2_SLOPE_RELEASE_FORCE;
	int G_TAP2_SLOPE_TAP_FORCE;
	int G_TAP2_DELTA_RELEASE_FORCE;
	int G_TAP2_DELTA_TAP_FORCE;
	int G_TAP3_EN;
	int G_TAP3_FORCE;
	int G_TAP3_MIN_POS;
	int G_TAP3_MAX_POS;
	int G_TAP3_SLOPE_WINDOW;
	int G_TAP3_SLOPE_RELEASE_FORCE;
	int G_TAP3_SLOPE_TAP_FORCE;
	int G_TAP3_DELTA_RELEASE_FORCE;
	int G_TAP3_DELTA_TAP_FORCE;
	int G_TAP4_EN;
	int G_TAP4_FORCE;
	int G_TAP4_MIN_POS;
	int G_TAP4_MAX_POS;
	int G_TAP4_SLOPE_WINDOW;
	int G_TAP4_SLOPE_RELEASE_FORCE;
	int G_TAP4_SLOPE_TAP_FORCE;
	int G_TAP4_DELTA_RELEASE_FORCE;
	int G_TAP4_DELTA_TAP_FORCE;
	int G_SLIDE_EN[2];
	int G_SLIDE_DIST[2];
	int G_SLIDE_FORCE[2];
	int G_SLIDE_VIB_EN[2];
	int G_SLIDE_TAP_PRIORITY[2];
	int G_SLIDE_MIN_POS[2];
	int G_SLIDE_MAX_POS[2];
	int G_SLIDE_2ND_DIST[2];
	int G_SLIDE1_EN;
	int G_SLIDE2_EN;
	int G_SLIDE1_DIST;
	int G_SLIDE2_DIST;
	int G_SLIDE1_FORCE;
	int G_SLIDE2_FORCE;
	int G_SLIDE1_VIB_EN;
	int G_SLIDE2_VIB_EN;
	int G_SWIPE_EN[2];
	int G_SWIPE_VELOCITY[2];
	int G_SWIPE_LEN[2];
	int G_SWIPE_MIN_POS[2];
	int G_SWIPE_MAX_POS[2];
	int G_SWIPE1_EN;
	int G_SWIPE1_VELOCITY;
	int G_SWIPE1_LEN;
	int G_SWIPE2_EN;
	int G_SWIPE2_VELOCITY;
	int G_SWIPE2_LEN;
	int G_TAP_SENSE_SET;
	int G_TAP_SENSE_EN;
};

struct Grip_DPC_status{
	uint16_t High;
	uint16_t Low;
	uint16_t Condition;
};
/*==========================================================================*/
/* EXTERNS                                                                  */
/*==========================================================================*/
// The currently enabled SysFS device
extern struct snt8100fsr *snt8100fsr_g;

// The i2c device to wake the snt8100fsr
extern struct snt8100fsr *snt8100fsr_wake_i2c_g;

extern struct grip_status *grip_status_g;

extern struct Grip_DPC_status *Grip_DPC_status_g;

/*==========================================================================*/
/* PROTOTYPES                                                               */
/*==========================================================================*/
int snt_spi_device_init(struct spi_device *spi,
                        struct snt8100fsr *snt8100fsr);

int snt_i2c_device_init(struct i2c_client *i2c,
                        struct snt8100fsr *snt8100fsr);

int snt_suspend(struct device *dev);
int snt_resume(struct device *dev);

#endif // PROTOTYPE_H
