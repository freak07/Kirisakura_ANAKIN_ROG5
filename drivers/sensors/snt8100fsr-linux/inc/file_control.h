#include <linux/proc_fs.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

extern int sntSensor_miscRegister(void);

extern void create_Grip_I2c_Check_proc_file(void);
extern void create_Grip_Calibration_raw_data_proc_file(void);
extern void create_Grip_Disable_WakeLock_proc_file(void);

//Enable interface
extern void create_Grip_frame_proc_file(void);
extern void create_Grip_raw_en_proc_file(void);
extern void create_Grip_en_proc_file(void);
	
//Gesture proc
extern void create_Grip_Tap_En_proc_file(void);
extern void create_Grip_Tap_Force_proc_file(void);
extern void create_Grip_Tap_min_pos_proc_file(void);
extern void create_Grip_Tap_max_pos_proc_file(void);
extern void create_Grip_Tap_slope_window_proc_file(void);
extern void create_Grip_Tap_slope_tap_force_proc_file(void);
extern void create_Grip_Tap_slope_release_force_proc_file(void);
extern void create_Grip_Tap_delta_tap_force_proc_file(void);
extern void create_Grip_Tap_delta_release_force_proc_file(void);
extern void create_Grip_Tap_vib_en_proc_file(void);

extern void create_Grip_Squeeze_en_proc_file(void);
extern void create_Grip_Squeeze_force_proc_file(void);
extern void create_Grip_Squeeze_force_proc_file(void);
extern void create_Grip_Squeeze_short_dur_proc_file(void);
extern void create_Grip_Squeeze_long_dur_proc_file(void);
extern void create_Grip_Squeeze_drop_rate_proc_file(void);
extern void create_Grip_Squeeze_drop_total_proc_file(void);
extern void create_Grip_Squeeze_up_rate_proc_file(void);
extern void create_Grip_Squeeze_up_total_proc_file(void);

extern void create_Grip_Slide_en_proc_file(void);
extern void create_Grip_Slide_dist_proc_file(void);
extern void create_Grip_Slide_2nd_dist_proc_file(void);
extern void create_Grip_Slide_force_proc_file(void);
extern void create_Grip_Slide_min_pos_proc_file(void);
extern void create_Grip_Slide_max_pos_proc_file(void);
extern void create_Grip_Slide_vib_en_proc_file(void);
void create_Grip_Slide_tap_priority_proc_file(void);

extern void create_Grip_Swipe_en_proc_file(void);
extern void create_Grip_Swipe_velocity_proc_file(void);
extern void create_Grip_Swipe_len_proc_file(void);
extern void create_Grip_Swipe_min_pos_proc_file(void);
extern void create_Grip_Swipe_max_pos_proc_file(void);
extern void create_Grip_Tap_Sense_En_proc_file(void);
#ifdef ASUS_ZS673KS_PROJECT
extern void create_Grip_Cal_Read_proc_file(void);
#endif


//Function: DPC wake from low power mode
extern void Wait_Wake_For_RegW(void);
extern void DPC_write_func(int flag);

// Gesture enable func
extern void grip_raw_enable_func(int val);
extern void grip_enable_func_noLock(int val);
extern void grip_tap_sense_enable_func(int val);


#ifdef DYNAMIC_PWR_CTL
extern int snt_activity_request(void);
#endif

extern int Health_Check_Enable(int en);
extern void Into_DeepSleep_fun(void);
/******* Dynamic Loading FW ********/
extern int fw_version;
extern void create_Grip_FW_RESULT_proc_file(void);
extern void create_Grip_FW_VER_proc_file(void);
extern void create_Grip_set_power_proc_file(void);
extern void create_Grip_SQ_Bar0_factory_proc_file(void);
extern void create_Grip_SQ_Bar1_factory_proc_file(void);
extern void create_Grip_ReadK_proc_file(void);

/* used to record health check value */
extern uint16_t FPC_value;

extern bool G_Skip_Sq1_Long;
extern bool G_Skip_Sq2_Long;

/* Workaround for stucked semaphore */
extern struct delayed_work check_stuck_wake;
extern void check_stuck_semaphore(struct work_struct *work);


extern struct delayed_work rst_recovery_wk;
extern struct delayed_work rst_gpio_wk;
extern void Reset_Func(struct work_struct *work);
extern void grip_dump_status_func(struct work_struct *work);
extern struct workqueue_struct *asus_wq;


extern void set_sq_gesture(uint16_t slide_id, uint16_t reg_val, int index);
extern void set_tap_gesture(uint16_t slide_id, uint16_t reg_val, int index);
extern void set_slide_gesture(uint16_t slide_id, uint16_t reg_val, int index);
extern void set_swipe_gesture(uint16_t slide_id, uint16_t reg_val, int index);

extern void get_sq_gesture(uint16_t tap_id, uint16_t reg_val, int index, int len);
/* Enable/disable Grip Sensor Power 1V2_2V8 */
extern void Power_Control(int en); 

extern void Grip_Driver_IRQ_EN(bool flag);
extern void Grip_Chip_IRQ_EN(bool flag);

extern int grip_game_gesture_status(void);
void Grip_Driver_IRQ_EN(bool flag);


//extern enum DEVICE_PROJID g_ASUS_prjID;
//extern enum DEVICE_HWID g_ASUS_hwID;
