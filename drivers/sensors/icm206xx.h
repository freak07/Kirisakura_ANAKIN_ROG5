/*
 * Copyright (c) 2014, 2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* register and associated bit definition */

#ifndef __ICM206XX_H__
#define __ICM206XX_H__

#define REG_SAMPLE_RATE_DIV		0x19
#define REG_CONFIG					0x1A
#define REG_GYRO_CONFIG			0x1B
#define REG_ACCEL_CONFIG			0x1C
#define REG_ACCEL_WOM_X_THR		0x20
#define REG_ACCEL_WOM_Y_THR		0x21
#define REG_ACCEL_WOM_Z_THR		0x22
#define REG_FIFO_EN					0x23
#define REG_INT_PIN_CFG			0x37
#define REG_INT_ENABLE				0x38
#define REG_FIFO_WM_INT_STATUS	0x39
#define REG_INT_STATUS				0x3A
#define REG_RAW_ACCEL				0x3B
#define REG_TEMPERATURE			0x41
#define REG_RAW_GYRO				0x43
#define REG_EXT_SENS_DATA_00		0x49
#define REG_SIGNAL_PATH_RESET		0x68
#define REG_DETECT_CTRL			0x69
#define REG_USER_CTRL				0x6A
#define REG_PWR_MGMT_1			0x6B
#define REG_PWR_MGMT_2			0x6C
#define REG_OIS_ENABLE				0x70
#define REG_FIFO_COUNT_H			0x72
#define REG_FIFO_R_W				0x74
#define REG_WHOAMI					0x75

#define GYRO_CONFIG_FSR_SHIFT_ICM2069X	2
#define GYRO_CONFIG_FSR_SHIFT_ICM2060X	3
#define ACCL_CONFIG_FSR_SHIFT	3

#define BITS_SELF_TEST_EN	0xE0
#define FIFO_DISABLE_ALL	0x00
#define BIT_ACCEL_FIFO		0x08
#define BIT_GYRO_FIFO		0x70
#define BIT_TEMP_FIFO		0x80

#define BIT_INT_ACTIVE_LOW	0x80
#define BIT_INT_OPEN_DRAIN	0x40
#define BIT_INT_LATCH_EN	0x20
#define BIT_INT_RD_CLR		0x10
#define BIT_I2C_BYPASS_EN	0x02
#define BIT_INT_CFG_DEFAULT	(BIT_INT_LATCH_EN | BIT_INT_RD_CLR)

#define BIT_DATA_RDY_EN		0x01
#define BIT_FIFO_OVERFLOW_EN	0x10
#define BIT_WOM_Z_INT_EN	0x20
#define BIT_WOM_Y_INT_EN	0x40
#define BIT_WOM_X_INT_EN	0x80

#define BIT_FIFO_WM_INT	    0x40

#define BIT_DATA_RDY_INT	0x01
#define BIT_FIFO_OVERFLOW	0x10
#define BIT_WOM_Z_INT	    0x20
#define BIT_WOM_Y_INT	    0x40
#define BIT_WOM_X_INT	    0x80

#define MOT_DET_DELAY_SHIFT	4

#define BIT_FIFO_EN		    0x40
#define BIT_FIFO_RESET		0x04

#define BIT_H_RESET		0x80
#define BIT_SLEEP		0x40
#define BIT_CYCLE		0x20
#define BIT_CLK_MASK		0x07
#define BIT_RESET_ALL		0xCF
#define BIT_WAKEUP_AFTER_RESET	0x00

#define BIT_PWR_ACCEL_STBY_MASK	0x38
#define BIT_PWR_GYRO_STBY_MASK	0x07
#define BITS_PWR_ALL_AXIS_STBY	(BIT_PWR_ACCEL_STBY_MASK |\
				BIT_PWR_GYRO_STBY_MASK)

#define SAMPLE_DIV_MAX		0xFF
#define ODR_DLPF_DIS		8000
#define ODR_DLPF_ENA		1000

/* Min delay = MSEC_PER_SEC/ODR_DLPF_ENA */
/* Max delay = MSEC_PER_SEC/(ODR_DLPF_ENA/SAMPLE_DIV_MAX+1) */
#define DELAY_MS_MIN_DLPF	1
#define DELAY_MS_MAX_DLPF	256

/* Min delay = MSEC_PER_SEC/ODR_DLPF_DIS and round up to 1*/
/* Max delay = MSEC_PER_SEC/(ODR_DLPF_DIS/SAMPLE_DIV_MAX+1) */
#define DELAY_MS_MIN_NODLPF	1
#define DELAY_MS_MAX_NODLPF	32

/* device bootup time in millisecond */
#define POWER_UP_TIME_MS	2
/* delay to wait gyro engine stable in millisecond */
#define SENSOR_UP_TIME_MS	5
/* delay between power operation in microsecond */
#define POWER_EN_DELAY_US	10

/* initial configure */
#define INIT_FIFO_RATE		1000//200

#define DEFAULT_MOT_THR		0x08// 1
#define DEFAULT_MOT_DET_EN	0xC0

/* chip reset wait */
#define ICM_RESET_RETRY_CNT	10
#define ICM_RESET_WAIT_MS	100
#define ICM_RESET_SLEEP_US	10

/* FIFO related constant */
#define ICM_FIFO_SIZE_BYTE	1024
#define ICM_FIFO_CNT_SIZE	2

#define ASUS_2ND_ACCEL_SENSOR_DATA_SIZE	3
#define ASUS_2ND_ACCEL_SENSOR_IOC_MAGIC                      ('C')	///< icm206xx accel ioctl magic number 
#define ASUS_2ND_ACCEL_SENSOR_IOCTL_DATA_READ           _IOR(ASUS_2ND_ACCEL_SENSOR_IOC_MAGIC, 1, int[ASUS_2ND_ACCEL_SENSOR_DATA_SIZE])	///< icm206xx accel ioctl command - Read data xyz
#define ASUS_2ND_ACCEL_SENSOR_IOCTL_DEBUG_MODE           _IOW(ASUS_2ND_ACCEL_SENSOR_IOC_MAGIC, 2, int)	///< RGB sensor ioctl command - Get debug mode
#define ASUS_2ND_ACCEL_SENSOR_IOCTL_UPDATE_CALIBRATION           _IOW(ASUS_2ND_ACCEL_SENSOR_IOC_MAGIC, 3, int)	///< 2nd accel sensor ioctl command - Info update calibration

#define ASUS_2ND_GYRO_SENSOR_DATA_SIZE	3
#define ASUS_2ND_GYRO_SENSOR_IOC_MAGIC                      ('C')	///< icm206xx accel ioctl magic number 
#define ASUS_2ND_GYRO_SENSOR_IOCTL_DATA_READ           _IOR(ASUS_2ND_GYRO_SENSOR_IOC_MAGIC, 1, int[ASUS_2ND_GYRO_SENSOR_DATA_SIZE])	///< icm206xx gyro ioctl command - Read data xyz
#define ASUS_2ND_GYRO_SENSOR_IOCTL_DEBUG_MODE           _IOW(ASUS_2ND_GYRO_SENSOR_IOC_MAGIC, 2, int)	///< 2nd gyro sensor ioctl command - Get debug mode
#define ASUS_2ND_GYRO_SENSOR_IOCTL_UPDATE_CALIBRATION           _IOW(ASUS_2ND_GYRO_SENSOR_IOC_MAGIC, 3, int)	///< icm206xx gyro ioctl command - Info update calibration

#define ICM_DEBUG_NODE

#define INVN_TAG    "ICM206XX"
#define ICM_SENSOR_NAME "icm20690"

#define IS_ODD_NUMBER(x)	(x & 1UL)

/* VDD 2.375V-3.46V VLOGIC 1.8V +-5% */
#define ICM_VDD_MIN_UV	2500000
#define ICM_VDD_MAX_UV	3400000
#define ICM_VLOGIC_MIN_UV	1800000
#define ICM_VLOGIC_MAX_UV	1800000
#define ICM_VI2C_MIN_UV	1750000
#define ICM_VI2C_MAX_UV	1950000

#define ICM_ACCEL_MIN_VALUE	-40000
#define ICM_ACCEL_MAX_VALUE	32767
#define ICM_GYRO_MIN_VALUE	-40000
#define ICM_GYRO_MAX_VALUE	32767

#define ICM_MAX_EVENT_CNT	170
/* Limit mininum delay to 2ms as we do not need higher rate so far */
#define ICM_ACCEL_MIN_POLL_INTERVAL_MS	2
#define ICM_ACCEL_MAX_POLL_INTERVAL_MS	5000
#define ICM_ACCEL_DEFAULT_POLL_INTERVAL_MS	10//200
#define ICM_ACCEL_INT_MAX_DELAY			19

#define ICM_GYRO_MIN_POLL_INTERVAL_MS	2
#define ICM_GYRO_MAX_POLL_INTERVAL_MS	5000
#define ICM_GYRO_DEFAULT_POLL_INTERVAL_MS	10//200
#define ICM_GYRO_INT_MAX_DELAY		18

#define ICM_RAW_ACCEL_DATA_LEN	6
#define ICM_RAW_GYRO_DATA_LEN	6

#define ICM_DEV_NAME_ACCEL	INVN_TAG"-accel"
#define ICM_DEV_NAME_GYRO	INVN_TAG"-gyro"

#define ICM_PINCTRL_DEFAULT	"cam_default"
#define ICM_PINCTRL_SUSPEND	"cam_suspend"

#define CAL_SKIP_COUNT	5
#define ICM_ACC_CAL_COUNT	15
#define ICM_ACC_CAL_NUM	(ICM_ACC_CAL_COUNT - CAL_SKIP_COUNT)
#define ICM_ACC_CAL_BUF_SIZE	22
#define RAW_TO_1G	16384
#define ACCELDATAUNIT	9807
#define ICM_ACC_CAL_DELAY	100	/* ms */
#define POLL_MS_100HZ	10
#define SNS_TYPE_GYRO	0
#define SNS_TYPE_ACCEL	1

#define INV_SPI_READ 0x80

enum icm_device_id {
    ICM20602_ID = 0x12,
    ICM20626_ID = 0x2D,
    ICM20690_ID = 0x20,
};

enum icm_fsr {
	ICM_FSR_250DPS = 0,
	ICM_FSR_500DPS,
	ICM_FSR_1000DPS,
	ICM_FSR_2000DPS,
	NUM_FSR
};

enum icm_filter {
	ICM_DLPF_256HZ_NOLPF2 = 0,
	ICM_DLPF_188HZ,
	ICM_DLPF_98HZ,
	ICM_DLPF_42HZ,
	ICM_DLPF_20HZ,
	ICM_DLPF_10HZ,
	ICM_DLPF_5HZ,
	ICM_DLPF_RESERVED,
	NUM_FILTER
};

enum icm_clock_source {
	ICM_CLK_INTERNAL = 0,
	ICM_CLK_PLL_X,
	NUM_CLK
};

enum icm_accl_fs {
	ACCEL_FS_02G = 0,
	ACCEL_FS_04G,
	ACCEL_FS_08G,
	ACCEL_FS_16G,
	NUM_ACCL_FSR
};

/* Sensitivity Scale Factor
 * Sensor HAL will take 1024 LSB/g
 */
enum icm_accel_fs_shift {
	ACCEL_SCALE_SHIFT_02G = 0,
	ACCEL_SCALE_SHIFT_04G = 1,
	ACCEL_SCALE_SHIFT_08G = 2,
	ACCEL_SCALE_SHIFT_16G = 3
};

enum icm_gyro_fs_shift {
	GYRO_SCALE_SHIFT_FS0 = 3,
	GYRO_SCALE_SHIFT_FS1 = 2,
	GYRO_SCALE_SHIFT_FS2 = 1,
	GYRO_SCALE_SHIFT_FS3 = 0
};

/* device enum */
enum inv_devices {
    INV_ICM20602,
    INV_ICM20626,
    INV_ICM20690,
    INV_NUM_PARTS
};

enum icm_place {
	ICM_PLACE_PU = 0,
	ICM_PLACE_PR = 1,
	ICM_PLACE_LD = 2,
	ICM_PLACE_LL = 3,
	ICM_PLACE_PU_BACK = 4,
	ICM_PLACE_PR_BACK = 5,
	ICM_PLACE_LD_BACK = 6,
	ICM_PLACE_LL_BACK = 7,
	ICM_PLACE_UNKNOWN = 8,
	ICM_AXIS_REMAP_TAB_SZ = 8
};

enum {
	ACCEL2_POLL_DELAY = 0,
	GYRO2_POLL_DELAY = 1,
	ICM206XX_GET_ICM_STATUS = 2,
};
/*
 *  struct icm_reg_map_s - Notable slave registers.
 *  @sample_rate_div:	Divider applied to gyro output rate.
 *  @lpf:		Configures internal LPF.
 *  @fifo_en:	Determines which data will appear in FIFO.
 *  @gyro_config:	gyro config register.
 *  @accel_config:	accel config register
 *  @mot_thr:	Motion detection threshold.
 *  @fifo_count_h:	Upper byte of FIFO count.
 *  @fifo_r_w:	FIFO register.
 *  @raw_gyro:	Address of first gyro register.
 *  @raw_accl:	Address of first accel register.
 *  @temperature:	temperature register.
 *  @int_pin_cfg:	Interrupt pin and I2C bypass configuration.
 *  @int_enable:	Interrupt enable register.
 *  @int_status:	Interrupt flags.
 *  @user_ctrl:	User control.
 *  @pwr_mgmt_1:	Controls chip's power state and clock source.
 *  @pwr_mgmt_2:	Controls power state of individual sensors.
 */
struct icm_reg_map {
	u8 sample_rate_div;
	u8 lpf;
	u8 fifo_en;
	u8 gyro_config;
	u8 accel_config;
	u8 fifo_count_h;
	u8 mot_thr;
	u8 mot_ctrl;
	u8 fifo_r_w;
	u8 raw_gyro;
	u8 raw_accel;
	u8 temperature;
	u8 int_pin_cfg;
	u8 int_enable;
	u8 int_status;
	u8 user_ctrl;
	u8 pwr_mgmt_1;
	u8 pwr_mgmt_2;
	u8 signal_path_reset;
	u8 ois_enable;
};

/*
 *  struct icm_chip_config - Cached chip configuration data.
 *  @fsr:		Full scale range.
 *  @lpf:		Digital low pass filter frequency.
 *  @accl_fs:		accel full scale range.
 *  @enable:		master enable to enable output
 *  @accel_enable:		enable accel functionality
 *  @accel_fifo_enable:	enable accel data output
 *  @gyro_enable:		enable gyro functionality
 *  @gyro_fifo_enable:	enable gyro data output
 *  @is_asleep:		1 if chip is powered down.
 *  @lpa_mode:		low power mode.
 *  @tap_on:		tap on/off.
 *  @flick_int_on:		flick interrupt on/off.
 *  @int_enabled:		interrupt is enabled.
 *  @mot_det_on:		motion detection wakeup enabled.
 *  @cfg_fifo_en:		FIFO R/W is enabled in USER_CTRL register.
 *  @int_pin_cfg:		interrupt pin configuration.
 *  @lpa_freq:		frequency of low power accelerometer.
 *  @rate_div:		Sampling rate divider.
 */
struct icm_chip_config {
	u32 fsr:2;
	u32 lpf:3;
	u32 accel_fs:2;
	u32 enable:1;
	u32 accel_enable:1;
	u32 accel_fifo_enable:1;
	u32 gyro_enable:1;
	u32 gyro_fifo_enable:1;
	u32 is_asleep:1;
	u32 lpa_mode:1;
	u32 tap_on:1;
	u32 flick_int_on:1;
	u32 int_enabled:1;
	u32 mot_det_on:1;
	u32 cfg_fifo_en:1;
	u8 int_pin_cfg;
	u16 lpa_freq;
	u16 rate_div;
};

/*
 *  struct icm_spi_data - device platform dependent data.
 *  @gpio_en:		enable GPIO.
 *  @gpio_int:		interrupt GPIO.
 *  @int_flags:		interrupt pin control flags.
 *  @use_int:		use interrupt mode instead of polling data.
 *  @place:			sensor place number.
 */
struct icm_spi_data {
	int gpio_en;
	int gpio_int;
	u32 int_flags;
	bool use_int;
	u8 place;
	int id;
};

struct icm_place_name {
	char name[32];
	enum icm_place place;
};

struct axis_data {
	s16 x;
	s16 y;
	s16 z;
	s16 rx;
	s16 ry;
	s16 rz;
};

struct icm_init_status {
	bool timer_inited;
	bool misc_inited;
	bool class_inited;
	bool gpio_inited;
};
/*
 *  struct icm_sensor - Cached chip configuration data
 *  @client:		I2C client
 *  @dev:		device structure
 *  @accel_dev:		accelerometer input device structure
 *  @gyro_dev:		gyroscope input device structure
 *  @accel_cdev:		sensor class device structure for accelerometer
 *  @gyro_cdev:		sensor class device structure for gyroscope
 *  @pdata:	device platform dependent data
 *  @op_lock:	device operation mutex
 *  @chip_type:	sensor hardware model
 *  @fifo_flush_work:	work structure to flush sensor fifo
 *  @reg:		notable slave registers
 *  @cfg:		cached chip configuration data
 *  @axis:	axis data reading
 *  @gyro_poll_ms:	gyroscope polling delay
 *  @accel_poll_ms:	accelerometer polling delay
 *  @accel_latency_ms:	max latency for accelerometer batching
 *  @gyro_latency_ms:	max latency for gyroscope batching
 *  @accel_en:	accelerometer enabling flag
 *  @gyro_en:	gyroscope enabling flag
 *  @use_poll:		use polling mode instead of  interrupt mode
 *  @motion_det_en:	motion detection wakeup is enabled
 *  @batch_accel:	accelerometer is working on batch mode
 *  @batch_gyro:	gyroscope is working on batch mode
 *  @vdd:	regulator data for Vdd
 *  @power_enabled:	flag of device power state
 *  @pinctrl:	pinctrl struct for interrupt pin
 *  @pin_default:	pinctrl default state
 *  @pin_sleep:	pinctrl sleep state
 *  @fifo_start_ns:		timestamp of first fifo data
 */
struct icm_sensor {						
	struct spi_device *pdev;
	struct regulator *power_supply;
	struct spi_device *spi;
	struct device *dev;
	struct hrtimer gyro_timer;
	struct hrtimer accel_timer;
	struct input_dev *accel_dev;
	struct input_dev *gyro_dev;
	struct input_dev *input_dev_icm206xx;
	struct proc_dir_entry *proc_polling;
	struct proc_dir_entry *proc_debug;
	struct icm_spi_data pdata;
	struct mutex op_lock;
	struct mutex bus_lock;
	enum inv_devices chip_type;
	struct workqueue_struct *data_wq;
	struct work_struct resume_work;
	struct delayed_work fifo_flush_work;
	struct icm_reg_map reg;
	struct icm_chip_config cfg;
	struct axis_data axis;
	struct icm_init_status init_status;
	struct wakeup_source icm206xx_wakeup_source;
	u32 deviceid;
	u32 gyro_poll_ms;
	u32 accel_poll_ms;
	u32 accel_latency_ms;
	u32 gyro_latency_ms;
	int irq;
	atomic_t accel_en;
	atomic_t gyro_en;
	bool use_poll;
	bool motion_det_en;
	bool batch_accel;
	bool batch_gyro;

	/* power control */
	struct regulator *vdd;
	bool power_enabled;

	/* pinctrl */
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;

	u64 fifo_start_ns;
	int gyro_wkp_flag;
	int accel_wkp_flag;
	struct task_struct *gyr_task;
	struct task_struct *accel_task;
	bool gyro_delay_change;
	bool accel_delay_change;
	wait_queue_head_t	gyro_wq;
	wait_queue_head_t	accel_wq;
};

struct sensor_axis_remap {
	/* src means which source will be mapped to target x, y, z axis
	 * if an target OS axis is remapped from (-)x,
	 * src is 0, sign_* is (-)1
	 * if an target OS axis is remapped from (-)y,
	 * src is 1, sign_* is (-)1
	 * if an target OS axis is remapped from (-)z,
	 * src is 2, sign_* is (-)1
	 */
	int src_x:3;
	int src_y:3;
	int src_z:3;

	int sign_x:2;
	int sign_y:2;
	int sign_z:2;
};

#endif /* __ICM206XX_H__ */
