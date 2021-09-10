#include <linux/leds.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/syscalls.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include "aura_wakelock.h"

#define FAN_ADD 1

struct ms51_platform_data {

	u8 fw_version;
	u32 hw_stage;

	struct regulator *regulator_vdd;
	int regulator_vdd_vmin;
	int regulator_vdd_vmax;
	int regulator_current;

	int logo_5p0_en;
	u32 logo_5p0_en_flags;

	int front_led_en;
	u32 front_led_en_flags;

	bool remove_aura_3p3_en;
	int aura_3p3_en;
	u32 aura_3p3_en_flags;

	int pogo_sleep;
	u32 pogo_sleep_flags;

	int pogo_det;
	u32 pogo_det_flags;

	// Calibration
	u32 RED_MAX;
	u32 GREEN_MAX;
	u32 BLUE_MAX;

	u8 current_mode;
	bool suspend_state;

	struct pinctrl *pinctrl;
	struct pinctrl_state *lgf_gpio_active;
	struct pinctrl_state *lgf_gpio_release;

	struct mutex ms51_mutex;

	//struct device dev;;
	struct led_classdev led;	/* LED control */
	char led_name[32];

// For emulated FW pattern
	struct workqueue_struct		*resume_workqueue;
	struct work_struct			resume_work;

	struct workqueue_struct		*init_workqueue;
	struct work_struct			init_work;

	struct i2c_client *client;

	// wake lock for resume work queue
	struct wake_lock	aura_wake_lock;

	#if FAN_ADD

	struct device *hwmon_dev;
	struct mutex update_lock;
	//unsigned long last_updated;	/* In jiffies */
	//enum chip_types chip_type; /* For recording what the chip is */ 

	//u8 bank;

	//u32 has_in;    /* Enable monitor VIN or not. VCC, VSEN1~3, VCore */
	//u16 in[5][3];		/* Complete format, not raw data. [VCC, VSEN1~3, VCore][read/high/low]. Completed voltage format, now raw data */

	//u16 has_fan;		/* Enable fan1-3 */
	u16 fan[3];		   /* Completed FAN count format, not raw reg value */
	//u16 fan_min[3];	/* Completed FAN count format, not raw reg value */

	//u8 has_temp;      /* Enable monitor RTD1-3 and LTD or not */
	//u8 temp[4][4];		/* Reg raw data. [RTD1-3,LTD][current, crit, high limit, low limit] */
	//u8 temp_read_lsb[3]; /*  Reg raw data.  **LTD has NO lsb value.** The LSB value corresponded to temp[][TEMP_READ]. */
	//u8 temp_mode;		/* 0: TR mode, 1: TD mode */

	//u8 enable_dts;    
	/* Enable PECI and SB-TSI, 
	* bit 0: =1 enable, =0 disable , 
	* bit 1: =1 AMD SB-TSI, =0 Intel PECI */
	//u8 has_dts;      /* Enable monitor DTS temp: DTS1-2 */
	//u8 dts[2][4];       /*  Reg raw data. [DTS1-2][current, crit, high limit, low limit] */
	//u8 dts_read_lsb[2];  /*  Reg raw data. The LSB value corresponded to dts[][DTS_READ] */

 
	//u8 alarms[3];     /* Raw register value */

	unsigned int ms51_enable_pin;  //fan_pwr_on gpio
	//bool enable_fan;
	//int pwm_reg_value;

	//struct notifier_block notifier_block_fan;  //register framebuffer notify
	char valid;

	#endif
};

/* Scale user input to sensible values */
static inline int SENSORS_LIMIT(long value, long low, long high)
{
	if (value < low)
		return low;
	else if (value > high)
		return high;
	else
		return value;
}
