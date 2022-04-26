/*
 * ICM 6-axis gyroscope + accelerometer driver
 *
 * Copyright (c) 2014-2015, 2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/msm-geni-se.h>
#include "icm206xx.h"

static const struct sensor_axis_remap
icm_accel_axis_remap_tab[ICM_AXIS_REMAP_TAB_SZ] = {
	/* src_x src_y src_z  sign_x  sign_y  sign_z */
	{  0,    1,    2,     1,      1,      1 }, /* P0 */
	{  1,    0,    2,     1,     -1,      1 }, /* P1 */
	{  0,    1,    2,    -1,     -1,      1 }, /* P2 */
	{  1,    0,    2,    -1,      1,      1 }, /* P3 */

	{  0,    1,    2,    -1,      1,     -1 }, /* P4 */
	{  1,    0,    2,    -1,     -1,     -1 }, /* P5 */
	{  0,    1,    2,     1,     -1,     -1 }, /* P6 */
	{  1,    0,    2,     1,      1,     -1 }, /* P7 */
};

static const struct sensor_axis_remap
icm_gyro_axis_remap_tab[ICM_AXIS_REMAP_TAB_SZ] = {
	/* src_x src_y src_z  sign_x  sign_y  sign_z */
	{  0,    1,    2,     1,      1,      1 }, /* P0 */
	{  1,    0,    2,     1,     -1,      1 }, /* P1 */
	{  0,    1,    2,    -1,     -1,      1 }, /* P2 */
	{  1,    0,    2,    -1,      1,      1 }, /* P3 */

	{  0,    1,    2,    -1,      1,     -1 }, /* P4 */
	{  1,    0,    2,    -1,     -1,     -1 }, /* P5 */
	{  0,    1,    2,     1,     -1,     -1 }, /* P6 */
	{  1,    0,    2,     1,      1,     -1 }, /* P7 */
};

static const struct icm_place_name
icm_place_name2num[ICM_AXIS_REMAP_TAB_SZ] = {
	{"Portrait Up", ICM_PLACE_PU},
	{"Landscape Right", ICM_PLACE_PR},
	{"Portrait Down", ICM_PLACE_LD},
	{"Landscape Left", ICM_PLACE_LL},
	{"Portrait Up Back Side", ICM_PLACE_PU_BACK},
	{"Landscape Right Back Side", ICM_PLACE_PR_BACK},
	{"Portrait Down Back Side", ICM_PLACE_LD_BACK},
	{"Landscape Left Back Side", ICM_PLACE_LL_BACK},
};

/* Map gyro measurement range setting to number of bit to shift */
static const u8 icm_gyro_fs_shift[NUM_FSR] = {
	GYRO_SCALE_SHIFT_FS0, /* ICM_FSR_250DPS */
	GYRO_SCALE_SHIFT_FS1, /* ICM_FSR_500DPS */
	GYRO_SCALE_SHIFT_FS2, /* ICM_FSR_1000DPS */
	GYRO_SCALE_SHIFT_FS3, /* ICM_FSR_2000DPS */
};

/* Map accel measurement range setting to number of bit to shift */
static const u8 icm_accel_fs_shift[NUM_ACCL_FSR] = {
	ACCEL_SCALE_SHIFT_02G, /* ACCEL_FS_02G */
	ACCEL_SCALE_SHIFT_04G, /* ACCEL_FS_04G */
	ACCEL_SCALE_SHIFT_08G, /* ACCEL_FS_08G */
	ACCEL_SCALE_SHIFT_16G, /* ACCEL_FS_16G */
};

/* Function declarations */
static int icm_set_interrupt(struct icm_sensor *sensor, const u8 mask, bool on);
static int icm_set_fifo(struct icm_sensor *sensor, bool en_accel, bool en_gyro);
static void icm_flush_fifo(struct icm_sensor *sensor);
static int icm_config_sample_rate(struct icm_sensor *sensor);
static int icm_acc_data_process(struct icm_sensor *sensor);
static void icm_resume_work_fn(struct work_struct *work);
static s32 icm_read_byte_data(struct icm_sensor *sensor, u8 command);
static s32 icm_do_write_byte_data(struct icm_sensor *sensor, u8 command, u8 value);
static s32 icm_write_byte_data(struct icm_sensor *sensor, u8 command, u8 value);
static s32 icm_mask_write_byte_data(struct icm_sensor *sensor, u8 command, u8 mask, u8 value);
static int icm_power_ctl(struct icm_sensor *sensor, bool enable);
static int icm_power_up(struct icm_sensor *);
static int icm_power_down(struct icm_sensor *);

static int g_icm206xx_status = 0;
static bool g_icm_enable_debug = true;		// set if verbose message is needed.
static struct icm_sensor *g_icm206xx_sensor = NULL;
static bool g_icm_debugMode = false;
static bool g_icm_skip_first_data = false;
static bool gyro_data_ready = true;
static u64 gyro_last_enable_time_ns = 0;
static struct timespec g_icm_timestamp;
static struct delayed_work g_icm_work_report;
static unsigned long g_icm_next_report_time_s = 2;
static struct delayed_work g_icm_data_retry_work;
static unsigned long g_icm_next_retry_time_ms = 100;
static u64 g_icm_irq_counter = 0;

#define icm_errmsg(str, args...) \
	printk("[icm206xx][error]%s: " str, __func__, ##args)

#define icm_dbgmsg(str, args...) \
	if (g_icm_enable_debug) \
		printk("[icm206xx]%s: " str, __func__, ##args)

/*
 * Timespec interfaces utilizing the ktime based ones
 */
static inline void icm_get_monotonic_boottime(struct timespec *ts)
{
	*ts = ktime_to_timespec(ktime_get_boottime());
}
static inline void icm_set_fifo_start_time(struct icm_sensor *sensor)
{
	struct timespec ts;

	icm_get_monotonic_boottime(&ts);
	sensor->fifo_start_ns = timespec_to_ns(&ts);
}

/*
 * icm_read_reg() - read multiple register data
 * @start_addr: register address read from
 * @buffer: provide register addr and get register
 * @length: length of register
 *
 * Reads the register values in one transaction or returns a negative
 * error code on failure.
 */
static int icm_read_reg(struct icm_sensor *sensor, u8 reg, u8 *data, int len)
{
	struct spi_message msg;
	int res;
	u8 d[1];
	struct spi_transfer xfers[] = {
		{
		 .tx_buf = d,
		 .bits_per_word = 8,
		 .len = 1,
		 },
		{
		 .rx_buf = data,
		 .bits_per_word = 8,
		 .len = len,
		 }
	};

	if (!data)
		return -EINVAL;

	mutex_lock(&sensor->bus_lock);
	d[0] = (reg | INV_SPI_READ);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);
	res = spi_sync(sensor->pdev, &msg);
	mutex_unlock(&sensor->bus_lock);

	return res;

}


/*
 * icm_read_accel_data() - get accelerometer data from device
 * @sensor: sensor device instance
 * @data: axis data to update
 *
 * Return the converted X Y and Z data from the sensor device
 */
static int icm_read_accel_data(struct icm_sensor *sensor,
			     struct axis_data *data)
{
	int ret = 0;
	u16 buffer[3];

	ret = icm_read_reg(sensor, sensor->reg.raw_accel,
		(u8 *)buffer, ICM_RAW_ACCEL_DATA_LEN);
	if (!ret) {
		data->x = be16_to_cpu(buffer[0]);
		data->y = be16_to_cpu(buffer[1]);
		data->z = be16_to_cpu(buffer[2]);
	}
	return ret;
}

/*
 * icm_read_gyro_data() - get gyro data from device
 * @sensor: sensor device instance
 * @data: axis data to update
 *
 * Return the converted RX RY and RZ data from the sensor device
 */
static int icm_read_gyro_data(struct icm_sensor *sensor,
			     struct axis_data *data)
{
	int ret = 0;
	u16 buffer[3];

	ret = icm_read_reg(sensor, sensor->reg.raw_gyro,
		(u8 *)buffer, ICM_RAW_GYRO_DATA_LEN);
	if (!ret) {
		data->rx = be16_to_cpu(buffer[0]);
		data->ry = be16_to_cpu(buffer[1]);
		data->rz = be16_to_cpu(buffer[2]);
	}
	return ret;
}

/*
 * icm_remap_accel_data() - remap accelerometer raw data to axis data
 * @data: data needs remap
 * @place: sensor position
 */
static void icm_remap_accel_data(struct axis_data *data, int place)
{
	const struct sensor_axis_remap *remap;
	s16 tmp[3];
	/* sensor with place 0 needs not to be remapped */
	if ((place <= 0) || (place >= ICM_AXIS_REMAP_TAB_SZ))
		return;

	remap = &icm_accel_axis_remap_tab[place];

	tmp[0] = data->x;
	tmp[1] = data->y;
	tmp[2] = data->z;
	data->x = tmp[remap->src_x] * remap->sign_x;
	data->y = tmp[remap->src_y] * remap->sign_y;
	data->z = tmp[remap->src_z] * remap->sign_z;
}

/*
 * icm_remap_gyro_data() - remap gyroscope raw data to axis data
 * @data: data to remap
 * @place: sensor position
 */
static void icm_remap_gyro_data(struct axis_data *data, int place)
{
	const struct sensor_axis_remap *remap;
	s16 tmp[3];
	/* sensor with place 0 needs not to be remapped */
	if ((place <= 0) || (place >= ICM_AXIS_REMAP_TAB_SZ))
		return;

	remap = &icm_gyro_axis_remap_tab[place];
	tmp[0] = data->rx;
	tmp[1] = data->ry;
	tmp[2] = data->rz;
	data->rx = tmp[remap->src_x] * remap->sign_x;
	data->ry = tmp[remap->src_y] * remap->sign_y;
	data->rz = tmp[remap->src_z] * remap->sign_z;
}
/*ASUS BSP: this function check if waiting enough time (80ms) since gyro sensor enabled
			there is a counter used to prevent data always not ready*/
static bool icm_is_gyro_data_ready()
{
	struct timespec ts;
	static int l_counter = 0;
	u64 l_current_time_ns;
	if (!gyro_data_ready) {
		l_counter++;
		icm_get_monotonic_boottime(&ts);
		l_current_time_ns = timespec_to_ns(&ts);
		if (l_current_time_ns - gyro_last_enable_time_ns > 80000000) {
			icm_dbgmsg("done, skipped %d data\n", l_counter - 1);
			gyro_data_ready = true;
			l_counter = 0;
		}
		if (l_counter > 50) {
			icm_errmsg("skipped more than 50 data! enabled time = %lld, current time = %lld\n", gyro_last_enable_time_ns, l_current_time_ns);
			gyro_data_ready = true;
			l_counter = 0;
		}
	}
	return gyro_data_ready;
}

/*
 * icm_read_single_event() - handle one sensor event.
 * @sensor: sensor device instance
 *
 * It only reads one sensor event from sensor register and send it to
 * sensor HAL, FIFO overflow and motion detection interrupt should be
 * handle by separate function.
 */
static int icm_read_single_event(struct icm_sensor *sensor, struct timespec timestamp1)
{
	int ret = 0;
	u32 shift;

	if (sensor->cfg.accel_enable) {
		ret = icm_acc_data_process(sensor);
		if (ret) {
			return ret;
		}
		/*ASUS_BSP: skip first accelerometer data since it's wrong*/
		if (!g_icm_skip_first_data) {
			shift = icm_accel_fs_shift[sensor->cfg.accel_fs];
			input_report_abs(sensor->accel_dev, ABS_X,
				(sensor->axis.x << shift));
			input_report_abs(sensor->accel_dev, ABS_Y,
				(sensor->axis.y << shift));
			input_report_abs(sensor->accel_dev, ABS_Z,
				(sensor->axis.z << shift));
			input_report_abs(sensor->accel_dev, ABS_WHEEL,
				timestamp1.tv_sec);
			input_report_abs(sensor->accel_dev, ABS_GAS,
				timestamp1.tv_nsec);
			input_sync(sensor->accel_dev);
		} else{
			g_icm_skip_first_data = false;
			icm_dbgmsg("skip first g sensor data\n");
		}
	}

	if (sensor->cfg.gyro_enable) {
		ret = icm_read_gyro_data(sensor, &sensor->axis);
		if (ret) {
			return ret;
		}
		icm_remap_gyro_data(&sensor->axis,
			(sensor->pdata).place);
		if (icm_is_gyro_data_ready()) {
			shift = icm_gyro_fs_shift[sensor->cfg.fsr];
			input_report_abs(sensor->gyro_dev, ABS_RX,
				(sensor->axis.rx >> shift));
			input_report_abs(sensor->gyro_dev, ABS_RY,
				(sensor->axis.ry >> shift));
			input_report_abs(sensor->gyro_dev, ABS_RZ,
				(sensor->axis.rz >> shift));
			input_report_abs(sensor->gyro_dev, ABS_WHEEL,
				timestamp1.tv_sec);
			input_report_abs(sensor->gyro_dev, ABS_GAS,
				timestamp1.tv_nsec);
			input_sync(sensor->gyro_dev);
		}
	}
	return 0;
}
static void icm_data_retry_wq(struct work_struct *work)
{
	int ret = 0;
	int l_gpio_value = 0;
	struct timespec l_timestamp;
	mutex_lock(&g_icm206xx_sensor->op_lock);
	l_gpio_value = gpio_get_value((g_icm206xx_sensor->pdata).gpio_int);

	/*ASUS BSP: if any of below conditions found, don't need to retry
		1. l_gpio_value = 0
			=> data has handled
		2. g_icm_next_retry_time_ms = 0
			=> irq has triggered
		3. sensor->power_enabled = 0
			=> sensor has disabled
	*/
	if (l_gpio_value && g_icm_next_retry_time_ms && g_icm206xx_sensor->power_enabled) {
		l_timestamp = ktime_to_timespec(ktime_get_boottime());
		ret = icm_read_single_event(g_icm206xx_sensor, l_timestamp);
	}
	if (!l_gpio_value || !g_icm_next_retry_time_ms || !g_icm206xx_sensor->power_enabled || !ret) {
		icm_dbgmsg("stop retry work with l_gpio_value = %d, retry_time_ms = %lu, power_enabled = %d, ret = %d\n",
			l_gpio_value,
			g_icm_next_retry_time_ms,
			g_icm206xx_sensor->power_enabled ? 1: 0,
			ret);
		g_icm_next_retry_time_ms = 0;
	} else{
		/*ASUS BSP: double retry time until it reach 6s*/
		if (g_icm_next_retry_time_ms < 6000) {
			g_icm_next_retry_time_ms *= 2;
		}
		/*ASUS BSP: check the boundary of retry time*/
		if (g_icm_next_retry_time_ms < 100) {
			g_icm_next_retry_time_ms = 100;
		} else if (g_icm_next_retry_time_ms > 6000) {
			g_icm_next_retry_time_ms = 6000;
		}
		schedule_delayed_work(&g_icm_data_retry_work, HZ * g_icm_next_retry_time_ms / 1000);
	}
	mutex_unlock(&g_icm206xx_sensor->op_lock);
}
/*
 * icm_interrupt_thread() - handle an IRQ
 * @irq: interrupt number
 * @data: the sensor device
 *
 * Called by the kernel single threaded after an interrupt occurs. Read
 * the sensor data and generate an input event for it.
 */
static irqreturn_t icm_interrupt_routine(int irq, void *data)
{
	g_icm_timestamp = ktime_to_timespec(ktime_get_boottime());
	return IRQ_WAKE_THREAD;
}
void icm_reset_ois_channel()
{
	int ret;
	icm_dbgmsg("E\n");
	ret = icm_write_byte_data(g_icm206xx_sensor, g_icm206xx_sensor->reg.signal_path_reset, 0x00);
	if (ret < 0) {
		icm_errmsg("write signal_path_reset failed.\n");
	}
	msleep(50);
	ret = icm_write_byte_data(g_icm206xx_sensor, g_icm206xx_sensor->reg.signal_path_reset, 0x08);
	if (ret < 0) {
		icm_dbgmsg("write signal_path_reset failed.\n");
	}
	icm_dbgmsg("X\n");
}
static irqreturn_t icm_interrupt_thread(int irq, void *data)
{
	int ret = 0;
	struct icm_sensor *sensor = data;
	g_icm_irq_counter++;
	mutex_lock(&sensor->op_lock);
	/*ASUS BSP: stop retry work when irq triggered*/
	g_icm_next_retry_time_ms = 0;
	if (!sensor->power_enabled) {
		icm_errmsg("power_enabled = false, just exit\n");
		goto exit;
	}
	ret = icm_read_single_event(sensor, g_icm_timestamp);
	if (ret) {
		icm_errmsg("data read failed, start retry work!\n");
		g_icm_next_retry_time_ms = 100;
		schedule_delayed_work(&g_icm_data_retry_work, HZ * g_icm_next_retry_time_ms / 1000);
	}
	

exit:
	mutex_unlock(&sensor->op_lock);

	return IRQ_HANDLED;
}

static void icm_sche_next_flush(struct icm_sensor *sensor)
{
	u32 latency;

	if ((sensor->batch_accel) && (sensor->batch_gyro)) {
		if (sensor->gyro_latency_ms < sensor->accel_latency_ms)
			latency = sensor->gyro_latency_ms;
		else
			latency = sensor->accel_latency_ms;
	} else if (sensor->batch_accel)
		latency = sensor->accel_latency_ms;
	else if (sensor->batch_gyro)
		latency = sensor->gyro_latency_ms;
	else
		latency = 0;

	if (latency != 0)
		queue_delayed_work(sensor->data_wq,
			&sensor->fifo_flush_work,
			msecs_to_jiffies(latency));
	else
		icm_errmsg("unknown error, accel: en=%d latency=%d gyro: en=%d latency=%d\n",
			sensor->batch_accel,
			sensor->accel_latency_ms,
			sensor->batch_gyro,
			sensor->gyro_latency_ms);
}

/*
 * icm_fifo_flush_fn() - flush shared sensor FIFO
 * @work: the work struct
 */
static void icm_fifo_flush_fn(struct work_struct *work)
{
	struct icm_sensor *sensor = container_of(
				(struct delayed_work *)work,
				struct icm_sensor, fifo_flush_work);

	icm_flush_fifo(sensor);
	icm_sche_next_flush(sensor);
}

static int icm_manage_polling(int sns_type, struct icm_sensor *sensor)
{
	ktime_t ktime;
	int ret = 0;

	switch (sns_type) {
	case SNS_TYPE_GYRO:
		if (atomic_read(&sensor->gyro_en)) {
			ktime = ktime_set(0,
				sensor->gyro_poll_ms * NSEC_PER_MSEC);
			/*ret = */hrtimer_start(&sensor->gyro_timer,
					ktime,
					HRTIMER_MODE_REL);
		} else
			ret = hrtimer_try_to_cancel(&sensor->gyro_timer);
		break;

	case SNS_TYPE_ACCEL:
		if (atomic_read(&sensor->accel_en)) {
			ktime = ktime_set(0,
				sensor->accel_poll_ms * NSEC_PER_MSEC);
			/*ret = */hrtimer_start(&sensor->accel_timer,
					ktime,
					HRTIMER_MODE_REL);
		} else
			ret = hrtimer_try_to_cancel(&sensor->accel_timer);
		break;

	default:
		icm_errmsg("Invalid sensor type\n");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum hrtimer_restart gyro_timer_handle(struct hrtimer *hrtimer)
{
	struct icm_sensor *sensor;

	sensor = container_of(hrtimer, struct icm_sensor, gyro_timer);
	sensor->gyro_wkp_flag = 1;
	wake_up_interruptible(&sensor->gyro_wq);
	if (icm_manage_polling(SNS_TYPE_GYRO, sensor) < 0)
		icm_errmsg("gyr: failed to start/cancel timer\n");

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart accel_timer_handle(struct hrtimer *hrtimer)
{
	struct icm_sensor *sensor;

	sensor = container_of(hrtimer, struct icm_sensor, accel_timer);
	sensor->accel_wkp_flag = 1;
	wake_up_interruptible(&sensor->accel_wq);
	if (icm_manage_polling(SNS_TYPE_ACCEL, sensor) < 0)
		icm_errmsg("acc: failed to start/cancel timer\n");

	return HRTIMER_NORESTART;
}

static int gyro_poll_thread(void *data)
{
	struct icm_sensor *sensor = data;
	u32 shift;
	ktime_t timestamp;

	while (1) {
		wait_event_interruptible(sensor->gyro_wq,
			((sensor->gyro_wkp_flag != 0) ||
				kthread_should_stop()));
		sensor->gyro_wkp_flag = 0;

		if (kthread_should_stop())
			break;

		mutex_lock(&sensor->op_lock);
		if (sensor->gyro_delay_change) {
			if (sensor->gyro_poll_ms <= POLL_MS_100HZ)
				set_wake_up_idle(true);
			else
				set_wake_up_idle(false);
			sensor->gyro_delay_change = false;
		}
		mutex_unlock(&sensor->op_lock);

		timestamp = ktime_get_boottime();
		icm_read_gyro_data(sensor, &sensor->axis);
		icm_remap_gyro_data(&sensor->axis, (sensor->pdata).place);
		shift = icm_gyro_fs_shift[sensor->cfg.fsr];
		input_report_abs(sensor->gyro_dev, ABS_RX,
			(sensor->axis.rx >> shift));
		input_report_abs(sensor->gyro_dev, ABS_RY,
			(sensor->axis.ry >> shift));
		input_report_abs(sensor->gyro_dev, ABS_RZ,
			(sensor->axis.rz >> shift));
		input_report_abs(sensor->gyro_dev, ABS_WHEEL,
			ktime_to_timespec(timestamp).tv_sec);
		input_report_abs(sensor->gyro_dev, ABS_GAS,
			ktime_to_timespec(timestamp).tv_nsec);
		input_sync(sensor->gyro_dev);
	}

	return 0;
}

static int accel_poll_thread(void *data)
{
	struct icm_sensor *sensor = data;
	u32 shift;
	ktime_t timestamp;

	while (1) {
		wait_event_interruptible(sensor->accel_wq,
			((sensor->accel_wkp_flag != 0) ||
				kthread_should_stop()));
		sensor->accel_wkp_flag = 0;

		if (kthread_should_stop())
			break;

		mutex_lock(&sensor->op_lock);
		if (sensor->accel_delay_change) {
			if (sensor->accel_poll_ms <= POLL_MS_100HZ)
				set_wake_up_idle(true);
			else
				set_wake_up_idle(false);
			sensor->accel_delay_change = false;
		}
		mutex_unlock(&sensor->op_lock);

		timestamp = ktime_get_boottime();
		icm_acc_data_process(sensor);
		shift = icm_accel_fs_shift[sensor->cfg.accel_fs];
		input_report_abs(sensor->accel_dev, ABS_X,
			(sensor->axis.x << shift));
		input_report_abs(sensor->accel_dev, ABS_Y,
			(sensor->axis.y << shift));
		input_report_abs(sensor->accel_dev, ABS_Z,
			(sensor->axis.z << shift));
		input_report_abs(sensor->accel_dev, ABS_WHEEL,
			ktime_to_timespec(timestamp).tv_sec);
		input_report_abs(sensor->accel_dev, ABS_GAS,
			ktime_to_timespec(timestamp).tv_nsec);
		input_sync(sensor->accel_dev);
	}

	return 0;
}

static int icm_switch_engine(struct icm_sensor *sensor,
				bool en, u32 mask)
{
	struct icm_reg_map *reg;
	int ret;

	reg = &sensor->reg;
	/*
	 * switch clock needs to be careful. Only when gyro is on, can
	 * clock source be switched to gyro. Otherwise, it must be set to
	 * internal clock
	 */
	if ((BIT_PWR_GYRO_STBY_MASK == mask) && (!en)) {
		/*
		 * turning off gyro requires switch to internal clock first.
		 * Then turn off gyro engine
		 */
		ret = icm_mask_write_byte_data(sensor,
			reg->pwr_mgmt_1, BIT_CLK_MASK, ICM_CLK_INTERNAL);
		if (ret < 0)
			goto error;
	}

	ret = icm_mask_write_byte_data(sensor,
			reg->pwr_mgmt_2, mask, en ? 0 : mask);
	if (ret < 0)
		goto error;

	if ((BIT_PWR_GYRO_STBY_MASK == mask) && en) {
		/* wait gyro stable */
		msleep(SENSOR_UP_TIME_MS);
		/* after gyro is on & stable, switch internal clock to PLL */
		ret = icm_mask_write_byte_data(sensor,
				reg->pwr_mgmt_1, BIT_CLK_MASK, ICM_CLK_PLL_X);
		if (ret < 0)
			goto error;
	}

	return 0;

error:
	icm_errmsg("Fail to switch ICM engine\n");

	return ret;
}

static int icm_init_engine(struct icm_sensor *sensor)
{
	int ret;
	icm_dbgmsg("++\n");
	ret = icm_switch_engine(sensor, false, BIT_PWR_GYRO_STBY_MASK);
	if (ret)
		return ret;

	ret = icm_switch_engine(sensor, false, BIT_PWR_ACCEL_STBY_MASK);
	if (ret)
		return ret;

	icm_dbgmsg("--\n");
	return 0;
}

/*
 * icm_set_power_mode() - set the power mode
 * @sensor: sensor data structure
 * @power_on: value to switch on/off of power, 1: normal power,
 *    0: low power
 *
 * Put device to normal-power mode or low-power mode.
 */
static int icm_set_power_mode(struct icm_sensor *sensor,
					bool power_on)
{
	s32 ret;
	u8 val;

	ret = icm_read_byte_data(sensor, sensor->reg.pwr_mgmt_1);
	if (ret < 0) {
		icm_errmsg("Fail to read power mode, ret=%d\n", ret);
		return ret;
	}

	if (power_on)
		val = (u8)ret & ~BIT_SLEEP;
	else
		val = (u8)ret | BIT_SLEEP;
	ret = icm_write_byte_data(sensor, sensor->reg.pwr_mgmt_1, val);
	if (ret < 0) {
		icm_errmsg("Fail to write power mode, ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static int icm_gyro_enable(struct icm_sensor *sensor, bool on)
{
	int ret;

	if (sensor->cfg.is_asleep) {
		icm_errmsg("Fail to set gyro state, device is asleep.\n");
		return -EINVAL;
	}

	if (on) {
		ret = icm_mask_write_byte_data(sensor,
				sensor->reg.pwr_mgmt_1, BIT_SLEEP, 0);
		if (ret < 0) {
			icm_errmsg("Fail to set sensor power state, ret=%d\n", ret);
			return ret;
		}

		ret = icm_switch_engine(sensor, true,
			BIT_PWR_GYRO_STBY_MASK);
		if (ret)
			return ret;

		sensor->cfg.gyro_enable = 1;
		sensor->cfg.enable = 1;
	} else {
		if (work_pending(&sensor->resume_work))
			cancel_work_sync(&sensor->resume_work);
		ret = icm_switch_engine(sensor, false,
			BIT_PWR_GYRO_STBY_MASK);
		if (ret)
			return ret;
		sensor->cfg.gyro_enable = 0;
		if (!sensor->cfg.accel_enable) {
			ret = icm_mask_write_byte_data(sensor,
					sensor->reg.pwr_mgmt_1, BIT_SLEEP, BIT_SLEEP);
			if (ret < 0) {
				icm_errmsg("Fail to set sensor power state, ret=%d\n", ret);
				return ret;
			}
			sensor->cfg.enable = 0;
		}
	}
	return 0;
}

/*
 * icm_restore_context() - update the sensor register context
 */
static int icm_restore_context(struct icm_sensor *sensor, bool isInit)
{
	struct icm_reg_map *reg;
	int ret;
	u8 data, pwr_ctrl;
	reg = &sensor->reg;

	if (!isInit) {
		/* Save power state and wakeup device from sleep */
		ret = icm_read_byte_data(sensor, reg->pwr_mgmt_1);
		if (ret < 0) {
			icm_errmsg("read power ctrl failed.\n");
			return ret;
		}
		pwr_ctrl = (u8)ret;
	} else{
		icm_dbgmsg("++\n");
	}

	/* Wake up from sleep */
	ret = icm_write_byte_data(sensor, reg->pwr_mgmt_1,
		BIT_WAKEUP_AFTER_RESET);
	if (ret < 0) {
		icm_errmsg("wakeup sensor failed.\n");
		return ret;
	}

	/* Gyro full scale range configure */
	if (sensor->chip_type == INV_ICM20690) {
		ret = icm_write_byte_data(sensor, reg->gyro_config,
			sensor->cfg.fsr << GYRO_CONFIG_FSR_SHIFT_ICM2069X);
	} else {
		ret = icm_write_byte_data(sensor, reg->gyro_config,
			sensor->cfg.fsr << GYRO_CONFIG_FSR_SHIFT_ICM2060X);
	}
	if (ret < 0) {
		icm_errmsg("update fsr failed.\n");
		return ret;
	}

	ret = icm_write_byte_data(sensor, reg->lpf, sensor->cfg.lpf);
	if (ret < 0) {
		icm_errmsg("update lpf failed.\n");
		return ret;
	}

	/*ASUS BSP:	write AFS_SEL to 2g
				write AFS_SEL_OIS to 8g*/
	ret = icm_write_byte_data(sensor, reg->accel_config,
			(sensor->cfg.accel_fs << ACCL_CONFIG_FSR_SHIFT) | 0x02);
	if (ret < 0) {
		icm_errmsg("update accel_fs failed.\n");
		return ret;
	}
	/*ASUS BSP: write FCHOICE_OIS_B to 1*/
 	ret = icm_write_byte_data(sensor, reg->signal_path_reset, 0x08);
	if (ret < 0) {
		icm_errmsg("write signal_path_reset failed.\n");
		return ret;
	}
	/*ASUS BSP: write ACCEL_FCHOICE_OIS_B to 1*/
 	ret = icm_write_byte_data(sensor, reg->mot_ctrl, 0x10);
	if (ret < 0) {
		icm_errmsg("write mot_ctrl failed.\n");
		return ret;
	}
	/*ASUS BSP: write OIS_ENABLE to 1*/
 	ret = icm_write_byte_data(sensor, reg->ois_enable, 0x02);
	if (ret < 0) {
		icm_errmsg("write ois_enable failed.\n");
		return ret;
	}

	ret = icm_write_byte_data(sensor, reg->sample_rate_div, sensor->cfg.rate_div);
	if (ret < 0) {
		icm_errmsg("set sample_rate_div failed.\n");
		return ret;
	}

	if (!isInit) {
		ret = icm_read_byte_data(sensor, reg->fifo_en);
		if (ret < 0) {
			icm_errmsg("read fifo_en failed.\n");
			return ret;
		}

		data = (u8)ret;

		if (sensor->cfg.accel_fifo_enable)
			data |= BIT_ACCEL_FIFO;

		if (sensor->cfg.gyro_fifo_enable)
			data |= BIT_GYRO_FIFO;

		if (sensor->cfg.accel_fifo_enable || sensor->cfg.gyro_fifo_enable) {
			ret = icm_write_byte_data(sensor, reg->fifo_en, data);
			if (ret < 0) {
				icm_errmsg("write fifo_en failed.\n");
				return ret;
			}
		}

		if (sensor->cfg.cfg_fifo_en) {
			/* Assume DMP and external I2C is not in use*/
			ret = icm_write_byte_data(sensor, reg->user_ctrl,
					BIT_FIFO_EN);
			if (ret < 0) {
				icm_errmsg("enable FIFO R/W failed.\n");
				return ret;
			}
		}

		/* Accel and Gyro should set to standby by default */
		ret = icm_write_byte_data(sensor, reg->pwr_mgmt_2,
				BITS_PWR_ALL_AXIS_STBY);
		if (ret < 0) {
			icm_errmsg("set pwr_mgmt_2 failed.\n");
			return ret;
		}
	}

	ret = icm_write_byte_data(sensor, reg->int_pin_cfg, sensor->cfg.int_pin_cfg);
	if (ret < 0) {
		icm_errmsg("set int_pin_cfg failed.\n");
		return ret;
	}

	if (!isInit) {
		ret = icm_write_byte_data(sensor, reg->pwr_mgmt_1, pwr_ctrl);
		if (ret < 0) {
			icm_errmsg("write saved power state failed.\n");
			return ret;
		}
	} else{
		/* Put sensor into sleep mode */
		ret = icm_mask_write_byte_data(sensor, sensor->reg.pwr_mgmt_1, BIT_SLEEP, BIT_SLEEP);
		if (ret < 0) {
			icm_errmsg("write sleep mode failed.\n");
			return ret;
		}
		icm_dbgmsg("--\n");
	}

	return ret;
}

/*
 * icm_reset_chip() - reset chip to default state
 */
static void icm_reset_chip(struct icm_sensor *sensor)
{
	int ret;

	ret = icm_do_write_byte_data(sensor, sensor->reg.pwr_mgmt_1,
			/*BIT_RESET_ALL*/BIT_H_RESET);
	if (ret < 0) {
		icm_errmsg("Reset chip fail!\n");
		goto exit;
	}

	mdelay(ICM_RESET_WAIT_MS);	// use mdelay instead of udelay, because linux MAX_UDELAY_MS of udealy is set at 2ms.

exit:
	return;
}

static int icm_gyro_batching_enable(struct icm_sensor *sensor)
{
	int ret = 0;
	u32 latency;

	if (!sensor->batch_accel) {
		latency = sensor->gyro_latency_ms;
	} else {
		cancel_delayed_work_sync(&sensor->fifo_flush_work);
		if (sensor->accel_latency_ms < sensor->gyro_latency_ms)
			latency = sensor->accel_latency_ms;
		else
			latency = sensor->gyro_latency_ms;
	}
	ret = icm_set_fifo(sensor, sensor->cfg.accel_enable, true);
	if (ret < 0) {
		icm_errmsg("Fail to enable FIFO for gyro, ret=%d\n", ret);
		return ret;
	}

	if (sensor->use_poll) {
		queue_delayed_work(sensor->data_wq,
			&sensor->fifo_flush_work,
			msecs_to_jiffies(latency));
	} else if (!sensor->cfg.int_enabled) {
		icm_set_interrupt(sensor, BIT_FIFO_OVERFLOW, true);
		sensor->cfg.int_enabled = true;
	}

	return ret;
}

static int icm_gyro_batching_disable(struct icm_sensor *sensor)
{
	int ret = 0;
	u32 latency;

	ret = icm_set_fifo(sensor, sensor->cfg.accel_enable, false);
	if (ret < 0) {
		icm_errmsg("Fail to disable FIFO for accel, ret=%d\n", ret);
		return ret;
	}
	if (!sensor->use_poll) {
		if (sensor->cfg.int_enabled && !sensor->cfg.accel_enable) {
			icm_set_interrupt(sensor,
				BIT_FIFO_OVERFLOW, false);
			sensor->cfg.int_enabled = false;
		}
	} else {
		if (!sensor->batch_accel) {
			cancel_delayed_work_sync(&sensor->fifo_flush_work);
		} else if (sensor->gyro_latency_ms <
				sensor->accel_latency_ms) {
			cancel_delayed_work_sync(&sensor->fifo_flush_work);
			latency = sensor->accel_latency_ms;
			queue_delayed_work(sensor->data_wq,
				&sensor->fifo_flush_work,
				msecs_to_jiffies(latency));
		}
	}
	sensor->batch_gyro = false;

	return ret;
}

static int icm_gyro_do_enable(struct icm_sensor *sensor, bool enable)
{
	int ret = 0;
	struct timespec ts;

	mutex_lock(&sensor->op_lock);
	if (enable) {
		ret = icm_gyro_enable(sensor, true);
		if (ret) {
			icm_errmsg("Fail to enable gyro engine ret=%d\n", ret);
			ret = -EBUSY;
			goto exit;
		}
		atomic_set(&sensor->gyro_en, 1);

		ret = icm_config_sample_rate(sensor);
		if (ret < 0)
			icm_errmsg("Unable to update sampling rate! ret=%d\n", ret);

		if (sensor->batch_gyro) {
			ret = icm_gyro_batching_enable(sensor);
			if (ret) {
				icm_errmsg("Fail to enable gyro batching =%d\n", ret);
				ret = -EBUSY;
				goto exit;
			}
		} else {
			if (sensor->use_poll) {
			ktime_t ktime;

			ktime = ktime_set(0,
					sensor->gyro_poll_ms * NSEC_PER_MSEC);
			hrtimer_start(&sensor->gyro_timer, ktime,
					HRTIMER_MODE_REL);
			} else {
				icm_set_interrupt(sensor, BIT_DATA_RDY_EN, true);
				sensor->cfg.int_enabled = true;

				}
		}
		icm_get_monotonic_boottime(&ts);
		gyro_last_enable_time_ns = timespec_to_ns(&ts);
		gyro_data_ready = false;
	} else {
		atomic_set(&sensor->gyro_en, 0);
		if (sensor->batch_gyro) {
			ret = icm_gyro_batching_disable(sensor);
			if (ret) {
				icm_errmsg("Fail to enable gyro batching =%d\n", ret);
				ret = -EBUSY;
				goto exit;
			}
		} else {
			ret = hrtimer_try_to_cancel(&sensor->gyro_timer);
		}
		ret = icm_gyro_enable(sensor, false);
		if (ret) {
			icm_errmsg("Fail to disable gyro engine ret=%d\n", ret);
			ret = -EBUSY;
			goto exit;
		}

	}

exit:
	mutex_unlock(&sensor->op_lock);

	icm_dbgmsg("done\n");
	return ret;
}
/*+++ASUS BSP: check if has enabled before.+++*/
static int icm_gyro_set_enable(struct icm_sensor *sensor, bool enable)
{
	int ret = 0;
	static int l_count = 0;

	icm_dbgmsg("enable = %d, l_count = %d\n", enable ? 1 : 0, l_count);
	if ((enable && l_count == 0) || (!enable && l_count == 1)) {
		if (g_icm206xx_sensor && enable) {
			icm_power_ctl(g_icm206xx_sensor, true);
		}
		ret = icm_gyro_do_enable(sensor, enable);
		if (g_icm206xx_sensor && !enable) {
			icm_power_ctl(g_icm206xx_sensor, false);
		}
	}

	if (enable) {
		l_count++;
	} else{
		l_count--;
	}
	return ret;
}
/*---ASUS BSP: check if has enabled before.---*/

/*
 * Set interrupt enabling bits to enable/disable specific type of interrupt.
 */
static int icm_set_interrupt(struct icm_sensor *sensor,
		const u8 mask, bool on)
{
	int ret;
	u8 data;

	if (sensor->cfg.is_asleep)
		return -EINVAL;

	ret = icm_read_byte_data(sensor,
				sensor->reg.int_enable);
	if (ret < 0) {
		icm_errmsg("Fail read interrupt mode. ret=%d\n", ret);
		return ret;
	}

	if (on) {
		data = (u8)ret;
		data |= mask;
	} else {
		data = (u8)ret;
		data &= ~mask;
	}

	ret = icm_write_byte_data(sensor,
			sensor->reg.int_enable, data);
	if (ret < 0) {
		icm_errmsg("Fail to set interrupt. ret=%d\n", ret);
		return ret;
	}

	return 0;
}

/*
 * Enable/disable motion detection interrupt.
 */
static int icm_set_motion_det(struct icm_sensor *sensor, bool on)
{
	int ret;

	if (on) {
		ret = icm_write_byte_data(sensor,
				sensor->reg.mot_thr, DEFAULT_MOT_THR);
		if (ret < 0)
			goto err_exit;
		ret = icm_write_byte_data(sensor,
				sensor->reg.mot_thr+1, DEFAULT_MOT_THR);
		if (ret < 0)
			goto err_exit;
		ret = icm_write_byte_data(sensor,
				sensor->reg.mot_thr+2, DEFAULT_MOT_THR);
		if (ret < 0)
			goto err_exit;

		ret = icm_write_byte_data(sensor,
				sensor->reg.mot_ctrl, DEFAULT_MOT_DET_EN);
		if (ret < 0)
			goto err_exit;

	} else {
		ret = icm_write_byte_data(sensor,
				sensor->reg.mot_ctrl, 0x00);
		if (ret < 0)
			goto err_exit;
        
    }

	ret = icm_set_interrupt(sensor, BIT_WOM_X_INT_EN|BIT_WOM_Y_INT_EN|BIT_WOM_Z_INT_EN, on);
	if (ret < 0)
		goto err_exit;

	sensor->cfg.mot_det_on = on;
	/* Use default motion detection delay 4ms */

	return 0;

err_exit:
	icm_errmsg("Fail to set motion detection. ret=%d\n", ret);

	return ret;
}

/* Update sensor sample rate divider upon accel and gyro polling rate. */
static int icm_config_sample_rate(struct icm_sensor *sensor)
{
	int ret;
	u32 delay_ms;
	u8 div, saved_pwr;

	if (sensor->cfg.is_asleep)
		return -EINVAL;

	if (!atomic_read(&sensor->gyro_en)) {
		delay_ms = sensor->accel_poll_ms;
	} else if (!atomic_read(&sensor->accel_en)) {
		delay_ms = sensor->gyro_poll_ms;
	} else if (sensor->accel_poll_ms <= sensor->gyro_poll_ms) {
		delay_ms = sensor->accel_poll_ms;
	} else{
		delay_ms = sensor->gyro_poll_ms;
	}

	/* Sample_rate = internal_ODR/(1+SMPLRT_DIV) */
	if ((sensor->cfg.lpf != ICM_DLPF_256HZ_NOLPF2) &&
		(sensor->cfg.lpf != ICM_DLPF_RESERVED)) {
		if (delay_ms > DELAY_MS_MAX_DLPF)
			delay_ms = DELAY_MS_MAX_DLPF;
		if (delay_ms < DELAY_MS_MIN_DLPF)
			delay_ms = DELAY_MS_MIN_DLPF;

		div = (u8)(((ODR_DLPF_ENA * delay_ms) / MSEC_PER_SEC) - 1);
	} else {
		if (delay_ms > DELAY_MS_MAX_NODLPF)
			delay_ms = DELAY_MS_MAX_NODLPF;
		if (delay_ms < DELAY_MS_MIN_NODLPF)
			delay_ms = DELAY_MS_MIN_NODLPF;
		div = (u8)(((ODR_DLPF_DIS * delay_ms) / MSEC_PER_SEC) - 1);
	}

	icm_dbgmsg("previous div: %u, current div: %u\n", sensor->cfg.rate_div, div);
	if (sensor->cfg.rate_div == div)
		return 0;

	ret = icm_read_byte_data(sensor, sensor->reg.pwr_mgmt_1);
	if (ret < 0)
		goto err_exit;

	saved_pwr = (u8)ret;

	ret = icm_write_byte_data(sensor, sensor->reg.pwr_mgmt_1,
		(saved_pwr & ~BIT_SLEEP));
	if (ret < 0)
		goto err_exit;

	ret = icm_write_byte_data(sensor,
		sensor->reg.sample_rate_div, div);
	if (ret < 0)
		goto err_exit;

	ret = icm_write_byte_data(sensor, sensor->reg.pwr_mgmt_1,
		saved_pwr);
	if (ret < 0)
		goto err_exit;

	sensor->cfg.rate_div = div;

	return 0;
err_exit:
	icm_errmsg("update sample div failed, div=%d, ret=%d\n", div, ret);

	return ret;
}

/*
 * Calculate sample interval according to sample rate.
 * Return sample interval in millisecond.
 */
static inline u64 icm_get_sample_interval(struct icm_sensor *sensor)
{
	u64 interval_ns;

	if ((sensor->cfg.lpf == ICM_DLPF_256HZ_NOLPF2) ||
		(sensor->cfg.lpf == ICM_DLPF_RESERVED)) {
		interval_ns = (sensor->cfg.rate_div + 1) * NSEC_PER_MSEC;
		interval_ns /= 8;
	} else {
		interval_ns = (sensor->cfg.rate_div + 1) * NSEC_PER_MSEC;
	}

	return interval_ns;
}

/*
 * icm_flush_fifo() - flush fifo and send sensor event
 * @sensor: sensor device instance
 * Return 0 on success and returns a negative error code on failure.
 *
 * This function assumes only accel and gyro data will be stored into FIFO
 * and does not check FIFO enabling bits, if other sensor data is stored into
 * FIFO, it will cause confusion.
 */
static void icm_flush_fifo(struct icm_sensor *sensor)
{
	u64 interval_ns, ts_ns, sec;
	int ret, i, ns;
	u16 *buf, cnt;
	u8 shift;

	ret = icm_read_reg(sensor, sensor->reg.fifo_count_h,
			(u8 *)&cnt, ICM_FIFO_CNT_SIZE);
	if (ret < 0) {
		icm_errmsg("read FIFO count failed, ret=%d\n", ret);
		return;
	}

	cnt = be16_to_cpu(cnt);
	if (cnt == 0)
		return;
	if (cnt > ICM_FIFO_SIZE_BYTE || IS_ODD_NUMBER(cnt)) {
		icm_errmsg("Invalid FIFO count number %d\n", cnt);
		return;
	}

	interval_ns = icm_get_sample_interval(sensor);
	ts_ns = sensor->fifo_start_ns + interval_ns;
	icm_set_fifo_start_time(sensor);

	buf = kmalloc(cnt, GFP_KERNEL);
	if (!buf)
		return;

	ret = icm_read_reg(sensor, sensor->reg.fifo_r_w,
			(u8 *)buf, cnt);
	if (ret < 0) {
		icm_errmsg("Read FIFO data error!\n");
		goto exit;
	}

	for (i = 0; i < (cnt >> 1); ts_ns += interval_ns) {
		if (sensor->cfg.accel_fifo_enable) {
			sensor->axis.x = be16_to_cpu(buf[i++]);
			sensor->axis.y = be16_to_cpu(buf[i++]);
			sensor->axis.z = be16_to_cpu(buf[i++]);
			sec = ts_ns;
			ns = do_div(sec, NSEC_PER_SEC);

			icm_remap_accel_data(&sensor->axis,
				(sensor->pdata).place);

			shift = icm_accel_fs_shift[sensor->cfg.accel_fs];
			input_report_abs(sensor->accel_dev, ABS_X,
				(sensor->axis.x << shift));
			input_report_abs(sensor->accel_dev, ABS_Y,
				(sensor->axis.y << shift));
			input_report_abs(sensor->accel_dev, ABS_Z,
				(sensor->axis.z << shift));
			input_report_abs(sensor->accel_dev, ABS_WHEEL,
				(int)sec);
			input_report_abs(sensor->accel_dev, ABS_GAS,
				(int)ns);
			input_sync(sensor->accel_dev);
		}

		if (sensor->cfg.gyro_fifo_enable) {
			sensor->axis.rx = be16_to_cpu(buf[i++]);
			sensor->axis.ry = be16_to_cpu(buf[i++]);
			sensor->axis.rz = be16_to_cpu(buf[i++]);
			sec = ts_ns;
			ns = do_div(sec, NSEC_PER_SEC);

			icm_remap_gyro_data(&sensor->axis,
				(sensor->pdata).place);

			shift = icm_gyro_fs_shift[sensor->cfg.fsr];
			input_report_abs(sensor->gyro_dev, ABS_RX,
				(sensor->axis.rx >> shift));
			input_report_abs(sensor->gyro_dev, ABS_RY,
				(sensor->axis.ry >> shift));
			input_report_abs(sensor->gyro_dev, ABS_RZ,
				(sensor->axis.rz >> shift));
			input_report_abs(sensor->gyro_dev, ABS_WHEEL,
				(int)sec);
			input_report_abs(sensor->gyro_dev, ABS_GAS,
				(int)ns);
			input_sync(sensor->gyro_dev);
		}
	}

exit:
	kfree(buf);
}

/*
 * icm_set_fifo() - Configure and enable sensor FIFO
 * @sensor: sensor device instance
 * @en_accel: buffer accel event to fifo
 * @en_gyro: buffer gyro event to fifo
 * Return 0 on success and returns a negative error code on failure.
 *
 * This function will remove all existing FIFO setting and flush FIFO data,
 * new FIFO setting will be applied after that.
 */
static int icm_set_fifo(struct icm_sensor *sensor,
					bool en_accel, bool en_gyro)
{
	struct icm_reg_map *reg = &sensor->reg;
	int ret;
	u8 en, user_ctl;

	en = FIFO_DISABLE_ALL;
	ret = icm_write_byte_data(sensor,
			reg->fifo_en, en);
	if (ret < 0)
		goto err_exit;

	icm_flush_fifo(sensor);

	/* Enable sensor output to FIFO */
	if (en_accel)
		en |= BIT_ACCEL_FIFO;

	if (en_gyro)
		en |= BIT_GYRO_FIFO;

	ret = icm_write_byte_data(sensor,
			reg->fifo_en, en);
	if (ret < 0)
		goto err_exit;

	/* Enable/Disable FIFO RW*/
	ret = icm_read_byte_data(sensor,
			reg->user_ctrl);
	if (ret < 0)
		goto err_exit;

	user_ctl = (u8)ret;
	if (en_accel | en_gyro) {
		user_ctl |= BIT_FIFO_EN;
		sensor->cfg.cfg_fifo_en = true;
	} else {
		user_ctl &= ~BIT_FIFO_EN;
		sensor->cfg.cfg_fifo_en = false;
	}

	ret = icm_write_byte_data(sensor,
			reg->user_ctrl, user_ctl);
	if (ret < 0)
		goto err_exit;

	icm_set_fifo_start_time(sensor);
	sensor->cfg.accel_fifo_enable = en_accel;
	sensor->cfg.gyro_fifo_enable = en_gyro;

	return 0;

err_exit:
	icm_errmsg("Set fifo failed, ret=%d\n", ret);

	return ret;
}

static int icm_gyro_set_poll_delay(struct icm_sensor *sensor,
					unsigned long delay)
{
	int ret = 0;

	if (delay < ICM_GYRO_MIN_POLL_INTERVAL_MS)
		delay = ICM_GYRO_MIN_POLL_INTERVAL_MS;
	if (delay > ICM_GYRO_MAX_POLL_INTERVAL_MS)
		delay = ICM_GYRO_MAX_POLL_INTERVAL_MS;

	mutex_lock(&sensor->op_lock);
	if (sensor->gyro_poll_ms == delay)
		goto exit;

	sensor->gyro_delay_change = true;
	sensor->gyro_poll_ms = delay;

	if (!atomic_read(&sensor->gyro_en))
		goto exit;

	if (sensor->use_poll) {
		ktime_t ktime;

		ret = hrtimer_try_to_cancel(&sensor->gyro_timer);
		ktime = ktime_set(0,
				sensor->gyro_poll_ms * NSEC_PER_MSEC);
		hrtimer_start(&sensor->gyro_timer, ktime, HRTIMER_MODE_REL);

	} else {
		ret = icm_config_sample_rate(sensor);
		if (ret < 0)
			icm_errmsg("Unable to set polling delay for gyro!\n");
	}

exit:
	mutex_unlock(&sensor->op_lock);

	return ret;
}

/*
 * icm_gyro_attr_get_polling_delay() - get the sampling rate
 */
static ssize_t icm_gyro_attr_get_polling_delay(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int val;
	struct icm_sensor *sensor = dev_get_drvdata(dev);

	val = sensor ? sensor->gyro_poll_ms : 0;

	return snprintf(buf, 8, "%d\n", val);
}

/*
 * icm_gyro_attr_set_polling_delay() - set the sampling rate
 */
static ssize_t icm_gyro_attr_set_polling_delay(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct icm_sensor *sensor = dev_get_drvdata(dev);
	unsigned long interval_ms;
	int ret;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;

	ret = icm_gyro_set_poll_delay(sensor, interval_ms);

	return ret ? -EBUSY : size;
}

static ssize_t icm_gyro_attr_get_enable(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct icm_sensor *sensor = dev_get_drvdata(dev);

	return snprintf(buf, 4, "%d\n", sensor->cfg.gyro_enable);
}

/*
 * icm_gyro_attr_set_enable() -
 *    Set/get enable function is just needed by sensor HAL.
 */
static ssize_t icm_gyro_attr_set_enable(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct icm_sensor *sensor = dev_get_drvdata(dev);
	unsigned long enable;
	int ret;

	if (kstrtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		ret = icm_gyro_set_enable(sensor, true);
	else
		ret = icm_gyro_set_enable(sensor, false);

	return ret ? -EBUSY : count;
}

static struct device_attribute gyro_attr[] = {
	__ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		icm_gyro_attr_get_polling_delay,
		icm_gyro_attr_set_polling_delay),
	__ATTR(enable, S_IRUGO | S_IWUSR,
		icm_gyro_attr_get_enable,
		icm_gyro_attr_set_enable),
};

static int create_gyro_sysfs_interfaces(struct device *dev)
{
	int i;
	int err;

	for (i = 0; i < ARRAY_SIZE(gyro_attr); i++) {
		err = device_create_file(dev, gyro_attr + i);
		if (err)
			goto error;
	}

	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, gyro_attr + i);
	icm_errmsg("Unable to create interface\n");

	return err;
}

static int icm_accel_enable(struct icm_sensor *sensor, bool on)
{
	int ret;

	if (sensor->cfg.is_asleep)
		return -EINVAL;

	if (on) {
		ret = icm_mask_write_byte_data(sensor,
				sensor->reg.pwr_mgmt_1, BIT_SLEEP, 0);
		if (ret < 0) {
			icm_errmsg("Fail to set sensor power state, ret=%d\n", ret);
			return ret;
		}

		ret = icm_switch_engine(sensor, true,
			BIT_PWR_ACCEL_STBY_MASK);
		if (ret)
			return ret;
		sensor->cfg.accel_enable = 1;
		sensor->cfg.enable = 1;
	} else {
		if (work_pending(&sensor->resume_work))
			cancel_work_sync(&sensor->resume_work);
		ret = icm_switch_engine(sensor, false,
			BIT_PWR_ACCEL_STBY_MASK);
		if (ret)
			return ret;
		sensor->cfg.accel_enable = 0;

		if (!sensor->cfg.gyro_enable) {
			ret = icm_mask_write_byte_data(sensor,
					sensor->reg.pwr_mgmt_1, BIT_SLEEP, BIT_SLEEP);
			if (ret < 0) {
				icm_errmsg("Fail to set sensor power state for accel, ret=%d\n", ret);
				return ret;
			}
			sensor->cfg.enable = 0;
		}
	}

	return 0;
}

static int icm_accel_batching_enable(struct icm_sensor *sensor)
{
	int ret = 0;
	u32 latency;

	if (!sensor->batch_gyro) {
		latency = sensor->accel_latency_ms;
	} else {
		cancel_delayed_work_sync(&sensor->fifo_flush_work);
		if (sensor->accel_latency_ms < sensor->gyro_latency_ms)
			latency = sensor->accel_latency_ms;
		else
			latency = sensor->gyro_latency_ms;
	}

	ret = icm_set_fifo(sensor, true, sensor->cfg.gyro_enable);
	if (ret < 0) {
		icm_errmsg("Fail to enable FIFO for accel, ret=%d\n", ret);
		return ret;
	}

	if (sensor->use_poll) {
		queue_delayed_work(sensor->data_wq,
			&sensor->fifo_flush_work,
			msecs_to_jiffies(latency));
	} else if (!sensor->cfg.int_enabled) {
		icm_set_interrupt(sensor, BIT_FIFO_OVERFLOW, true);
		sensor->cfg.int_enabled = true;
	}

	return ret;
}

static int icm_accel_batching_disable(struct icm_sensor *sensor)
{
	int ret = 0;
	u32 latency;

	ret = icm_set_fifo(sensor, false, sensor->cfg.gyro_enable);
	if (ret < 0) {
		icm_errmsg("Fail to disable FIFO for accel, ret=%d\n", ret);
		return ret;
	}
	if (!sensor->use_poll) {
		if (sensor->cfg.int_enabled && !sensor->cfg.gyro_enable) {
			icm_set_interrupt(sensor,
				BIT_FIFO_OVERFLOW, false);
			sensor->cfg.int_enabled = false;
		}
	} else {
		if (!sensor->batch_gyro) {
			cancel_delayed_work_sync(&sensor->fifo_flush_work);
		} else if (sensor->accel_latency_ms <
				sensor->gyro_latency_ms) {
			cancel_delayed_work_sync(&sensor->fifo_flush_work);
			latency = sensor->gyro_latency_ms;
			queue_delayed_work(sensor->data_wq,
				&sensor->fifo_flush_work,
				msecs_to_jiffies(latency));
		}
	}
	sensor->batch_accel = false;

	return ret;
}
static int icm_accel_do_enable(struct icm_sensor *sensor, bool enable)
{
	int ret = 0;

	mutex_lock(&sensor->op_lock);
	if (enable) {
		ret = icm_accel_enable(sensor, true);
		if (ret) {
			icm_errmsg("Fail to enable accel engine ret=%d\n", ret);
			goto exit;
		}
		atomic_set(&sensor->accel_en, 1);

		ret = icm_config_sample_rate(sensor);
		if (ret < 0) {
			icm_errmsg("Unable to update sampling rate! ret=%d\n", ret);
		}

		if (sensor->batch_accel) {
			ret = icm_accel_batching_enable(sensor);
			if (ret) {
				icm_errmsg("Fail to enable accel batching =%d\n", ret);
				goto exit;
			}
		} else {
			if (sensor->use_poll) {
				ktime_t ktime;
				ktime = ktime_set(0,
					sensor->accel_poll_ms * NSEC_PER_MSEC);
				hrtimer_start(&sensor->accel_timer, ktime,
					HRTIMER_MODE_REL);
			} else {
				icm_set_interrupt(sensor, BIT_DATA_RDY_EN, true);
				sensor->cfg.int_enabled = true;

			}
		}
		g_icm_skip_first_data = true;
	} else {
		atomic_set(&sensor->accel_en, 0);
		if (sensor->batch_accel) {
			ret = icm_accel_batching_disable(sensor);
			if (ret) {
				icm_errmsg("Fail to disable accel batching =%d\n", ret);
				goto exit;
			}
		} else {
			if (sensor->use_poll) {
				ret = hrtimer_try_to_cancel(&sensor->accel_timer);
			} else {
				sensor->cfg.int_enabled = false;

			}
		}

		ret = icm_accel_enable(sensor, false);
		if (ret) {
			icm_errmsg("Fail to disable accel engine ret=%d\n", ret);
			goto exit;
		}

	}

exit:
	mutex_unlock(&sensor->op_lock);
	return ret;
}

/*+++ASUS BSP: check if has enabled before.+++*/
static int icm_accel_set_enable(struct icm_sensor *sensor, bool enable)
{
	int ret = 0;
	static int l_count = 0;

	icm_dbgmsg("enable = %d, l_count = %d\n", enable ? 1 : 0, l_count);
	if ((enable && l_count == 0) || (!enable && l_count == 1)) {
		if (g_icm206xx_sensor && enable) {
			icm_power_ctl(g_icm206xx_sensor, true);
		}
		ret = icm_accel_do_enable(sensor, enable);
		if (g_icm206xx_sensor && !enable) {
			icm_power_ctl(g_icm206xx_sensor, false);
		}
	}

	if (enable) {
		l_count++;
	} else{
		l_count--;
	}
	return ret;
}
/*---ASUS BSP: check if has enabled before.---*/
static int icm_accel_set_poll_delay(struct icm_sensor *sensor,
					unsigned long delay)
{
	int ret = 0;

	if (delay < ICM_ACCEL_MIN_POLL_INTERVAL_MS)
		delay = ICM_ACCEL_MIN_POLL_INTERVAL_MS;
	if (delay > ICM_ACCEL_MAX_POLL_INTERVAL_MS)
		delay = ICM_ACCEL_MAX_POLL_INTERVAL_MS;

	mutex_lock(&sensor->op_lock);
	if (sensor->accel_poll_ms == delay)
		goto exit;

	sensor->accel_delay_change = true;
	sensor->accel_poll_ms = delay;

	if (!atomic_read(&sensor->accel_en))
		goto exit;


	if (sensor->use_poll) {
		ktime_t ktime;

		ret = hrtimer_try_to_cancel(&sensor->accel_timer);
		ktime = ktime_set(0,
				sensor->accel_poll_ms * NSEC_PER_MSEC);
		hrtimer_start(&sensor->accel_timer, ktime, HRTIMER_MODE_REL);
	} else {
		ret = icm_config_sample_rate(sensor);
		if (ret < 0)
			icm_errmsg("Unable to set polling delay for accel!\n");
	}

exit:
	mutex_unlock(&sensor->op_lock);

	return ret;
}

static int icm_acc_data_process(struct icm_sensor *sensor)
{
	int ret = 0;
	ret = icm_read_accel_data(sensor, &sensor->axis);
	if (!ret) {
		icm_remap_accel_data(&sensor->axis, (sensor->pdata).place);
		sensor->axis.x = sensor->axis.x * ACCELDATAUNIT / RAW_TO_1G;
		sensor->axis.y = sensor->axis.y * ACCELDATAUNIT / RAW_TO_1G;
		sensor->axis.z = sensor->axis.z * ACCELDATAUNIT / RAW_TO_1G;
	}
	return ret;
}

/*
 * icm_accel_attr_get_polling_delay() - get the sampling rate
 */
static ssize_t icm_accel_attr_get_polling_delay(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int val;
	struct icm_sensor *sensor = dev_get_drvdata(dev);

	val = sensor ? sensor->accel_poll_ms : 0;

	return snprintf(buf, 8, "%d\n", val);
}

/*
 * icm_accel_attr_set_polling_delay() - set the sampling rate
 */
static ssize_t icm_accel_attr_set_polling_delay(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct icm_sensor *sensor = dev_get_drvdata(dev);
	unsigned long interval_ms;
	int ret;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;

	ret = icm_accel_set_poll_delay(sensor, interval_ms);

	return ret ? -EBUSY : size;
}

static ssize_t icm_accel_attr_get_enable(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct icm_sensor *sensor = dev_get_drvdata(dev);
	
	return snprintf(buf, 4, "%d\n", sensor->cfg.accel_enable);
}

/*
 * icm_accel_attr_set_enable() -
 *    Set/get enable function is just needed by sensor HAL.
 */

static ssize_t icm_accel_attr_set_enable(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct icm_sensor *sensor = dev_get_drvdata(dev);
	unsigned long enable;
	int ret;

	if (kstrtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		ret = icm_accel_set_enable(sensor, true);
	else
		ret = icm_accel_set_enable(sensor, false);

	return ret ? -EBUSY : count;
}

#ifdef ICM_DEBUG_NODE
static u8 icm_address;
static u8 icm_data;

static ssize_t icm_accel_attr_get_reg_addr(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, 8, "%d\n", icm_address);
}

static ssize_t icm_accel_attr_set_reg_addr(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long addr;

	if (kstrtoul(buf, 10, &addr))
		return -EINVAL;
	if ((addr < 0) || (addr > 255))
		return -EINVAL;

	icm_address = addr;
	icm_dbgmsg("icm_address =%d\n", icm_address);

	return size;
}

static ssize_t icm_accel_attr_get_data(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct icm_sensor *sensor = dev_get_drvdata(dev);
	int ret;

	ret = icm_read_byte_data(sensor, icm_address);
	icm_dbgmsg("read addr(0x%x)=0x%x\n", icm_address, ret);
	if (ret >= 0 && ret <= 255)
		icm_data = ret;

	return snprintf(buf, 8, "0x%x\n", ret);
}

static ssize_t icm_accel_attr_set_data(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long reg_data;

	if (kstrtoul(buf, 10, &reg_data))
		return -EINVAL;
	if ((reg_data < 0) || (reg_data > 255))
		return -EINVAL;

	icm_data = reg_data;
	icm_dbgmsg("set icm_data =0x%x\n", icm_data);

	return size;
}
static ssize_t icm_accel_attr_reg_write(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct icm_sensor *sensor = dev_get_drvdata(dev);
	int ret;

	ret = icm_write_byte_data(sensor,
		icm_address, icm_data);
	icm_dbgmsg("write addr(0x%x)<-0x%x ret=%d\n",
		icm_address, icm_data, ret);

	return size;
}

#endif

static struct device_attribute accel_attr[] = {
	__ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		icm_accel_attr_get_polling_delay,
		icm_accel_attr_set_polling_delay),
	__ATTR(enable, S_IRUGO | S_IWUSR,
		icm_accel_attr_get_enable,
		icm_accel_attr_set_enable),
#ifdef ICM_DEBUG_NODE
	__ATTR(addr, S_IRUSR | S_IWUSR,
		icm_accel_attr_get_reg_addr,
		icm_accel_attr_set_reg_addr),
	__ATTR(reg, S_IRUSR | S_IWUSR,
		icm_accel_attr_get_data,
		icm_accel_attr_set_data),
	__ATTR(write, S_IWUSR,
		NULL,
		icm_accel_attr_reg_write),
#endif
};

static int create_accel_sysfs_interfaces(struct device *dev)
{
	int i;
	int err;

	for (i = 0; i < ARRAY_SIZE(accel_attr); i++) {
		err = device_create_file(dev, accel_attr + i);
		if (err)
			goto error;
	}

	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, accel_attr + i);
	icm_errmsg("Unable to create interface\n");

	return err;
}

static void setup_icm_reg(struct icm_reg_map *reg)
{
	reg->sample_rate_div	= REG_SAMPLE_RATE_DIV;
	reg->lpf		= REG_CONFIG;
	reg->fifo_en		= REG_FIFO_EN;
	reg->gyro_config	= REG_GYRO_CONFIG;
	reg->accel_config	= REG_ACCEL_CONFIG;
	reg->mot_thr		= REG_ACCEL_WOM_X_THR;
	reg->fifo_count_h	= REG_FIFO_COUNT_H;
	reg->fifo_r_w		= REG_FIFO_R_W;
	reg->raw_gyro		= REG_RAW_GYRO;
	reg->raw_accel		= REG_RAW_ACCEL;
	reg->temperature	= REG_TEMPERATURE;
	reg->int_pin_cfg	= REG_INT_PIN_CFG;
	reg->int_enable		= REG_INT_ENABLE;
	reg->int_status		= REG_INT_STATUS;
	reg->signal_path_reset		= REG_SIGNAL_PATH_RESET;
	reg->mot_ctrl		= REG_DETECT_CTRL;
	reg->user_ctrl		= REG_USER_CTRL;
	reg->pwr_mgmt_1		= REG_PWR_MGMT_1;
	reg->pwr_mgmt_2		= REG_PWR_MGMT_2;
	reg->ois_enable		= REG_OIS_ENABLE;
};

/*
 * icm_check_chip_type() - check and setup chip type.
 */
static int icm_check_chip_type(struct icm_sensor *sensor)
{
	struct icm_reg_map *reg;
	s32 ret;

	reg = &sensor->reg;
	setup_icm_reg(reg);
	ret = icm_set_power_mode(sensor, false);
	if (ret) {
		return ret;
	}
	ret = icm_set_power_mode(sensor, true);
	if (ret) {
		return ret;
	}

	ret = icm_read_byte_data(sensor,
		REG_WHOAMI);
	if (ret < 0)
		return 0;

	sensor->deviceid = ret;
	sensor->chip_type = INV_ICM20690;
	icm_dbgmsg("WHOAMI=0x%x", ret);

	if (sensor->deviceid != ICM20690_ID) {
		icm_errmsg("Invalid chip ID %d\n", sensor->deviceid);
		return 0;
	} else{
		return 1;
	}
}

/*
 *  icm_init_config() - Initialize hardware, disable FIFO.
 *  @indio_dev:	Device driver instance.
 *  Initial configuration:
 *  FSR: +/- 2000DPS
 *  DLPF: 188Hz
 *  FIFO rate: 1000Hz
 *  AFS: 2G
 */
static void icm_init_config(struct icm_sensor *sensor)
{
	memset(&sensor->cfg, 0, sizeof(struct icm_chip_config));
	sensor->cfg.fsr = ICM_FSR_2000DPS;
	sensor->cfg.lpf = ICM_DLPF_188HZ;
	sensor->cfg.rate_div = (u8)(ODR_DLPF_ENA / INIT_FIFO_RATE - 1);
	sensor->cfg.accel_fs = ACCEL_FS_02G;
	if (((sensor->pdata).int_flags & IRQF_TRIGGER_FALLING) || ((sensor->pdata).int_flags & IRQF_TRIGGER_LOW)) {
		sensor->cfg.int_pin_cfg = BIT_INT_CFG_DEFAULT | BIT_INT_ACTIVE_LOW;
	} else{
		sensor->cfg.int_pin_cfg = BIT_INT_CFG_DEFAULT;
	}
	sensor->cfg.gyro_enable = 0;
	sensor->cfg.gyro_fifo_enable = 0;
	sensor->cfg.accel_enable = 0;
	sensor->cfg.accel_fifo_enable = 0;
}
static int icm_pinctrl_init(struct icm_sensor *sensor)
{
	sensor->pinctrl = devm_pinctrl_get(sensor->dev);
	if (IS_ERR_OR_NULL(sensor->pinctrl)) {
		icm_errmsg("Failed to get pinctrl\n");
		return PTR_ERR(sensor->pinctrl);
	}

	sensor->pin_default =
		pinctrl_lookup_state(sensor->pinctrl, ICM_PINCTRL_DEFAULT);
	if (IS_ERR_OR_NULL(sensor->pin_default))
		icm_errmsg("Failed to look up default state\n");

	sensor->pin_sleep =
		pinctrl_lookup_state(sensor->pinctrl, ICM_PINCTRL_SUSPEND);
	if (IS_ERR_OR_NULL(sensor->pin_sleep))
		icm_errmsg("Failed to look up sleep state\n");

	return 0;
}

static int icm_pinctrl_state(struct icm_sensor *sensor,
			bool active)
{
	int ret = 0;

	if (active) {
		if (!IS_ERR_OR_NULL(sensor->pin_default)) {
			ret = pinctrl_select_state(sensor->pinctrl,
				sensor->pin_default);
			if (ret)
				icm_errmsg("Error pinctrl_select_state(%s) err:%d\n",
					ICM_PINCTRL_DEFAULT, ret);
		}
	} else {
		if (!IS_ERR_OR_NULL(sensor->pin_sleep)) {
			ret = pinctrl_select_state(sensor->pinctrl,
				sensor->pin_sleep);
			if (ret)
				icm_errmsg("Error pinctrl_select_state(%s) err:%d\n",
					ICM_PINCTRL_SUSPEND, ret);
		}
	}
	return ret;
}

static int icm_dt_get_place(struct device *dev,
			struct icm_spi_data *pdata)
{
	const char *place_name;
	int rc;
	int i;

	rc = of_property_read_string(dev->of_node, "invn,place", &place_name);
	if (rc) {
		icm_errmsg("Cannot get place configuration!\n");
		return -EINVAL;
	}

	icm_dbgmsg("place_name=[%s]\n", place_name);
	
	for (i = 0; i < ICM_AXIS_REMAP_TAB_SZ; i++) {
		if (!strcmp(place_name, icm_place_name2num[i].name)) {
			pdata->place = icm_place_name2num[i].place;
			break;
		}
	}
	if (i >= ICM_AXIS_REMAP_TAB_SZ) {
		icm_errmsg("Invalid place parameter, use default value 0\n");
		pdata->place = 0;
	}

	return 0;
}

static int icm_parse_dt(struct device *dev,
			struct icm_spi_data *pdata)
{
	int rc;

	rc = icm_dt_get_place(dev, pdata);
	if (rc) {
		return rc;
	}

	pdata->gpio_int = of_get_named_gpio_flags(dev->of_node,
				"invn,gpio-int", 0, &pdata->int_flags);

	pdata->gpio_en = of_get_named_gpio_flags(dev->of_node,
				"invn,gpio-en", 0, NULL);

	pdata->use_int = of_property_read_bool(dev->of_node,
				"invn,use-interrupt");
	return 0;
}

/*+++ASUS BSP proc asusIcm206xxPolling Interface+++*/
static int asusIcm206xxPolling_proc_read(struct seq_file *buf, void *v)
{
	int result = 0;
	if (g_icm206xx_sensor) {
		if (g_icm206xx_sensor->use_poll) {
			result = 1;
		} else{
			result = 0;
		}
	} else{
		icm_errmsg("null icm ctrl!");
		result = -1;
	}
	seq_printf(buf, "%d\n", result);
	return 0;
}
static int asusIcm206xxPolling_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, asusIcm206xxPolling_proc_read, NULL);
}

static ssize_t asusIcm206xxPolling_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;
	char messages[256];
	if (!g_icm206xx_sensor) {
		icm_errmsg("null icm_sensor!");
		return len;
	}

	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);
	if (val == 0) {
		g_icm206xx_sensor->use_poll = false;
	} else{
		g_icm206xx_sensor->use_poll = true;
	}
	icm_dbgmsg("%d\n", val);
	return len;
}
static struct proc_dir_entry * create_asusIcm206xxPolling_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  asusIcm206xxPolling_proc_open,
		.write = asusIcm206xxPolling_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/asusIcm206xxPolling", 0664, NULL, &proc_fops);

	if (!proc_file) {
		icm_errmsg("failed!\n");
	}
	return proc_file;
}
/*---ASUS BSP proc asusIcm206xxPolling Interface---*/
static ssize_t accel2_poll_delay_show(struct class *c, struct class_attribute *attr,
			char *ubuf)
{
	icm_dbgmsg("%s\n", ubuf);
	if (!g_icm206xx_sensor) {
		icm_errmsg("null icm_sensor!");
		return snprintf(ubuf, PAGE_SIZE, "%d\n", 0);
	}
	return snprintf(ubuf, PAGE_SIZE, "%u\n", g_icm206xx_sensor->accel_poll_ms);
}

static ssize_t accel2_poll_delay_store(struct class *c, struct class_attribute *attr,
			const char *ubuf, size_t count)
{
	unsigned long interval_ms;
	icm_dbgmsg("%s ms\n", ubuf);
	if (!g_icm206xx_sensor) {
		icm_errmsg("null icm sensor!");
		return count;
	}
	if (!kstrtoul(ubuf, 10, &interval_ms)) {
		icm_accel_set_poll_delay(g_icm206xx_sensor, interval_ms);
	}
	return count;
}
static CLASS_ATTR_RW(accel2_poll_delay);

static ssize_t gyro2_poll_delay_show(struct class *c, struct class_attribute *attr,
			char *ubuf)
{
	icm_dbgmsg("%s\n", ubuf);
	if (!g_icm206xx_sensor) {
		icm_errmsg("null icm sensor!");
		return snprintf(ubuf, PAGE_SIZE, "%d\n", 0);
	}
	return snprintf(ubuf, PAGE_SIZE, "%u\n", g_icm206xx_sensor->gyro_poll_ms);
}

static ssize_t gyro2_poll_delay_store(struct class *c, struct class_attribute *attr,
			const char *ubuf, size_t count)
{
	unsigned long interval_ms;
	icm_dbgmsg("%s ms\n", ubuf);
	if (!g_icm206xx_sensor) {
		icm_errmsg("null icm ctrl!");
		return count;
	}
	if (!kstrtoul(ubuf, 10, &interval_ms)) {
		icm_gyro_set_poll_delay(g_icm206xx_sensor, interval_ms);
	}
	return count;
}
static CLASS_ATTR_RW(gyro2_poll_delay);
static ssize_t icm206xx_get_icm_status_show(struct class *c, struct class_attribute *attr,
			char *ubuf)
{
	s32 ret = 0;
	u64 l_irq_counter = g_icm_irq_counter;
	int i = 0;
	if (!g_icm206xx_sensor) {
		icm_errmsg("null icm sensor!");
	} else{
		icm_accel_set_enable(g_icm206xx_sensor, true);
		if (ICM20690_ID == icm_read_byte_data(g_icm206xx_sensor, REG_WHOAMI)) {
			for (i = 0; i < 10; i++) {
				if (g_icm_irq_counter > l_irq_counter + 1) {
					ret = 1;
					break;
				}
				msleep(100);
			}
		}
		icm_accel_set_enable(g_icm206xx_sensor, false);
	}
	return snprintf(ubuf, PAGE_SIZE, "%d\n", ret);
}
static CLASS_ATTR_RO(icm206xx_get_icm_status);

static struct attribute *icm206xx_class_attrs[] = {
	[ACCEL2_POLL_DELAY]	= &class_attr_accel2_poll_delay.attr,
	[GYRO2_POLL_DELAY]	= &class_attr_gyro2_poll_delay.attr,
	[ICM206XX_GET_ICM_STATUS]	= &class_attr_icm206xx_get_icm_status.attr,
	NULL,
};
ATTRIBUTE_GROUPS(icm206xx_class);
struct class icm206xx_class = {
	.name = "icm206xx",
	.owner = THIS_MODULE,
	.class_groups = icm206xx_class_groups
};
/*+++ASUS BSP proc asusIcm206xxDebug Interface+++*/
static int asusIcm206xxDebug_proc_read(struct seq_file *buf, void *v)
{
	int result = 0;
	if (g_icm_debugMode) {
		result = 1;
	} else{
		result = 0;
	}
	seq_printf(buf, "%d\n", result);
	return 0;
}
static int asusIcm206xxDebug_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, asusIcm206xxDebug_proc_read, NULL);
}

static ssize_t asusIcm206xxDebug_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;
	char messages[256];
	if (len > 256) {
		len = 256;
	}
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);
	if (val == 0) {
		g_icm_debugMode = false;
	} else{
		g_icm_debugMode = true;
	}
	icm_dbgmsg("%d\n", val);
	return len;
}
static struct proc_dir_entry * create_asusIcm206xxDebug_proc_file(void)
{
	static const struct file_operations proc_fops = {
		.owner = THIS_MODULE,
		.open =  asusIcm206xxDebug_proc_open,
		.write = asusIcm206xxDebug_proc_write,
		.read = seq_read,
		.release = single_release,
	};
	struct proc_dir_entry *proc_file = proc_create("driver/asusIcm206xxDebug", 0664, NULL, &proc_fops);

	if (!proc_file) {
		icm_errmsg("failed!\n");
	}
	return proc_file;
}
/*---ASUS BSP proc asusIcm206xxDebug Interface---*/
static int icm206xx_accel_miscOpen(struct inode *inode, struct file *file)
{
	int ret = 0;
	if (!g_icm206xx_sensor) {
		icm_errmsg("null icm sensor!");
		return -1;
	}
	ret = icm_accel_set_enable(g_icm206xx_sensor, true);
	icm_dbgmsg("ret = %d\n", ret);
	if (ret < 0) {
		icm_accel_set_enable(g_icm206xx_sensor, false);
	}
	return ret;
}

static int icm206xx_accel_miscRelease(struct inode *inode, struct file *file)
{
	int ret = 0;
	if (!g_icm206xx_sensor) {
		icm_errmsg("null icm ctrl!");
		return -1;
	}
	ret = icm_accel_set_enable(g_icm206xx_sensor, false);
	icm_dbgmsg("ret = %d\n", ret);
	return ret;
}
static long icm206xx_accel_miscIoctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int dataI[ASUS_2ND_ACCEL_SENSOR_DATA_SIZE];
	u8 shift;
	static int l_counter = 0;
	if (!g_icm206xx_sensor) {
		icm_errmsg("null icm ctrl!");
		ret = -1;
		goto end;
	}
	switch (cmd) {
		case ASUS_2ND_ACCEL_SENSOR_IOCTL_DATA_READ:
			icm_acc_data_process(g_icm206xx_sensor);
			shift = icm_accel_fs_shift[g_icm206xx_sensor->cfg.accel_fs];
			dataI[0] = g_icm206xx_sensor->axis.x << shift;
			dataI[1] = g_icm206xx_sensor->axis.y << shift;
			dataI[2] = g_icm206xx_sensor->axis.z << shift;
			icm_dbgmsg("cmd = DATA_READ, data[0] = %d, data[1] = %d, data[2] = %d\n", dataI[0], dataI[1], dataI[2]);
			ret = copy_to_user((int __user*)arg, &dataI, sizeof(dataI));
			break;
		case ASUS_2ND_ACCEL_SENSOR_IOCTL_UPDATE_CALIBRATION:
			icm_dbgmsg("cmd = UPDATE_CALIBRATION\n");
			input_report_abs(g_icm206xx_sensor->accel_dev, ABS_BRAKE, ++l_counter);
			input_sync(g_icm206xx_sensor->accel_dev);
			break;
		default:
			ret = -1;
			icm_errmsg("default\n", __func__);
	}
end:
	return ret;
}
static struct file_operations icm206xx_accel_fops = {
  .owner = THIS_MODULE,
  .open = icm206xx_accel_miscOpen,
  .release = icm206xx_accel_miscRelease,
  .unlocked_ioctl = icm206xx_accel_miscIoctl,
  .compat_ioctl = icm206xx_accel_miscIoctl
};
struct miscdevice icm206xx_accel_misc = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = "asus2ndAccelSensor",
  .fops = &icm206xx_accel_fops
};
static int icm206xx_gyro_miscOpen(struct inode *inode, struct file *file)
{
	int ret = 0;
	if (!g_icm206xx_sensor) {
		icm_errmsg("null icm ctrl!");
		return -1;
	}
	ret = icm_gyro_set_enable(g_icm206xx_sensor, true);
	icm_dbgmsg("ret = %d\n", ret);
	if (ret < 0) {
		icm_gyro_set_enable(g_icm206xx_sensor, false);
	}
	return ret;
}

static int icm206xx_gyro_miscRelease(struct inode *inode, struct file *file)
{
	int ret = 0;
	if (!g_icm206xx_sensor) {
		icm_errmsg("null icm ctrl!");
		return -1;
	}
	ret = icm_gyro_set_enable(g_icm206xx_sensor, false);
	icm_dbgmsg("ret = %d\n", ret);
	return ret;
}
static long icm206xx_gyro_miscIoctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int dataI[ASUS_2ND_GYRO_SENSOR_DATA_SIZE];
	u8 shift;
	static int l_counter = 0;
	switch (cmd) {
		case ASUS_2ND_GYRO_SENSOR_IOCTL_DATA_READ:
			icm_read_gyro_data(g_icm206xx_sensor, &g_icm206xx_sensor->axis);
			icm_remap_gyro_data(&g_icm206xx_sensor->axis,
				g_icm206xx_sensor->pdata.place);
			shift = icm_gyro_fs_shift[g_icm206xx_sensor->cfg.fsr];
			dataI[0] = g_icm206xx_sensor->axis.rx >> shift;
			dataI[1] = g_icm206xx_sensor->axis.ry >> shift;
			dataI[2] = g_icm206xx_sensor->axis.rz >> shift;
			icm_dbgmsg("cmd = DATA_READ, data[0] = %d, data[1] = %d, data[2] = %d\n", dataI[0], dataI[1], dataI[2]);
			ret = copy_to_user((int __user*)arg, &dataI, sizeof(dataI));
			break;
		case ASUS_2ND_GYRO_SENSOR_IOCTL_UPDATE_CALIBRATION:
			icm_dbgmsg("cmd = UPDATE_CALIBRATION\n");
			input_report_abs(g_icm206xx_sensor->gyro_dev, ABS_BRAKE, ++l_counter);
			input_sync(g_icm206xx_sensor->gyro_dev);
			break;
		default:
			ret = -1;
			icm_errmsg("default\n", __func__);
	}
	return ret;
}
static struct file_operations icm206xx_gyro_fops = {
  .owner = THIS_MODULE,
  .open = icm206xx_gyro_miscOpen,
  .release = icm206xx_gyro_miscRelease,
  .unlocked_ioctl = icm206xx_gyro_miscIoctl,
  .compat_ioctl = icm206xx_gyro_miscIoctl
};
struct miscdevice icm206xx_gyro_misc = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = "asus2ndGyroSensor",
  .fops = &icm206xx_gyro_fops
};
static int icm206xx_miscRegister(void)
{
	int rtn = 0;
	rtn = misc_register(&icm206xx_accel_misc);
	if (rtn < 0) {
		icm_errmsg("Unable to register misc deive\n");
		return rtn;
	}
	rtn = misc_register(&icm206xx_gyro_misc);
	if (rtn < 0) {
		icm_errmsg("Unable to register misc deive\n");
		misc_deregister(&icm206xx_accel_misc);
	}
	return rtn;
}

static ssize_t accel_status_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_icm206xx_status);
}

static ssize_t gyro_status_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_icm206xx_status);
}

static struct device_attribute dev_attr_accel_status =
__ATTR(accel_status, 0664, accel_status_show, NULL);

static struct device_attribute dev_attr_gyro_status =
__ATTR(gyro_status, 0664, gyro_status_show, NULL);

static struct attribute *icm206xx_attributes[] = {
	&dev_attr_accel_status.attr,
	&dev_attr_gyro_status.attr,
	NULL
};

static struct attribute_group icm206xx_attr_group = {
	.attrs = icm206xx_attributes,
};
static int icm206xx_input_setup_BMMI(struct icm_sensor *sensor)
{
	int rc = 0;
	struct input_dev *idev;

	idev = input_allocate_device();
	if (idev == NULL) {
		return -ENOMEM;
	}

	idev->name = "Invensense ICM206xx Accel_Gyro";
	rc = input_register_device(idev);
	if (rc) {
		icm_errmsg("icm206xx register input device fail rc = %d\n", rc);
		input_free_device(idev);
		return rc;
	}

	input_set_drvdata(idev, sensor);

	rc = sysfs_create_group(&idev->dev.kobj,
				&icm206xx_attr_group);
	if (rc) {
		icm_errmsg("icm206xx create sysfs fail\n");
		input_unregister_device(idev);
		return rc;
	}
	sensor->input_dev_icm206xx = idev;
	return 0;
}

static struct input_dev* icm206xx_input_setup(struct icm_sensor *sensor, int sensor_type)
{
	int ret = 0;
	struct input_dev *l_dev;

	l_dev = devm_input_allocate_device(sensor->dev);
	if (!l_dev) {
		icm_errmsg("Failed to allocate accelerometer input device\n");
		ret = -ENOMEM;
		return NULL;
	}

	l_dev->id.bustype = BUS_SPI;
	l_dev->dev.parent = sensor->dev;
	
	input_set_capability(l_dev, EV_ABS, ABS_WHEEL); //sec
	input_set_capability(l_dev, EV_ABS, ABS_GAS); //nsec
	input_set_capability(l_dev, EV_ABS, ABS_MISC);
	input_set_capability(l_dev, EV_ABS, ABS_BRAKE);
	input_set_drvdata(l_dev, sensor);

	if (sensor_type == 0) {
		l_dev->name = ICM_DEV_NAME_ACCEL;
		input_set_abs_params(l_dev, ABS_X,
			ICM_ACCEL_MIN_VALUE, ICM_ACCEL_MAX_VALUE,
			0, 0);
		input_set_abs_params(l_dev, ABS_Y,
			ICM_ACCEL_MIN_VALUE, ICM_ACCEL_MAX_VALUE,
			0, 0);
		input_set_abs_params(l_dev, ABS_Z,
			ICM_ACCEL_MIN_VALUE, ICM_ACCEL_MAX_VALUE,
			0, 0);
	} else{
		l_dev->name = ICM_DEV_NAME_GYRO;
		input_set_abs_params(l_dev, ABS_RX,
			ICM_GYRO_MIN_VALUE, ICM_GYRO_MAX_VALUE,
			0, 0);
		input_set_abs_params(l_dev, ABS_RY,
			ICM_GYRO_MIN_VALUE, ICM_GYRO_MAX_VALUE,
			0, 0);
		input_set_abs_params(l_dev, ABS_RZ,
			ICM_GYRO_MIN_VALUE, ICM_GYRO_MAX_VALUE,
			0, 0);
	}
	
	ret = input_register_device(l_dev);
	if (ret) {
		icm_errmsg("Failed to register input device\n");
		input_free_device(l_dev);
		return NULL;
	}

	if (sensor_type == 0) {
		ret = create_accel_sysfs_interfaces(&l_dev->dev);
	} else{
		ret = create_gyro_sysfs_interfaces(&l_dev->dev);
	}
	if (ret < 0) {
		icm_errmsg("failed to create sysfs\n");
		input_unregister_device(l_dev);
		return NULL;
	}

	return l_dev;
}
static int icm206xx_check_probe_status(struct device *dev)
{
	if (IS_ERR(regulator_get(dev, "icm206xx"))) {
		return -EINVAL;
	} else{
		return 0;
	}
}
static int icm206xx_init_regulator(struct icm_sensor *sensor)
{
	int ret = 0;
	if (!sensor || !sensor->dev) {
		icm_errmsg("there is no icm_sensor\n");
		ret = -EINVAL;
		goto exit;
	}

	sensor->power_supply = regulator_get(sensor->dev, "icm206xx");
	if (IS_ERR(sensor->power_supply)) {
		icm_errmsg("there is no power_supply\n");
		sensor->power_supply = NULL;
		ret = -EINVAL;
		goto exit;
	} else{
		if (regulator_count_voltages(sensor->power_supply) > 0) {
			ret = regulator_set_voltage(sensor->power_supply, 2900000, 3450000);
		}
	}
#ifdef ASUS_DXO
	sensor->vdd = regulator_get(sensor->dev, "icm206xx-cam");
	if (IS_ERR(sensor->vdd)) {
		sensor->vdd = NULL;
		icm_dbgmsg("there is no regulator icm206xx-cam\n");
	}
#else
	sensor->vdd = regulator_get(sensor->dev, "icm206xx-vdd");
	if (IS_ERR(sensor->vdd)) {
		sensor->vdd = NULL;
		icm_dbgmsg("there is no regulator icm206xx-vdd\n");
	} else{
		if (regulator_count_voltages(sensor->vdd) > 0) {
			regulator_set_voltage(sensor->vdd, 1710000, 3450000);
		}
	}
#endif
exit:
	return ret;
}


static void icm_init_status_function(struct icm_sensor *sensor)
{
	struct icm_init_status *l_init_status = &sensor->init_status;
	l_init_status->timer_inited = false;
	l_init_status->misc_inited = false;
	l_init_status->class_inited = false;
	l_init_status->gpio_inited = false;
	sensor->irq = 0;
	sensor->accel_dev = NULL;
	sensor->gyro_dev = NULL;
	sensor->input_dev_icm206xx = NULL;
	sensor->data_wq = NULL;
	sensor->gyr_task = NULL;
	sensor->accel_task = NULL;
	sensor->proc_polling = NULL;
	sensor->proc_debug = NULL;
}
static void icm_deinit()
{
	struct icm_init_status *l_init_status;
	if (g_icm206xx_sensor) {
		l_init_status = &g_icm206xx_sensor->init_status;

		/*power off here*/
		if (g_icm206xx_sensor->power_enabled) {
			icm_set_power_mode(g_icm206xx_sensor, false);
			icm_power_down(g_icm206xx_sensor);
		}

		if (l_init_status->misc_inited) {
			misc_deregister(&icm206xx_accel_misc);
			misc_deregister(&icm206xx_gyro_misc);
			l_init_status->misc_inited = false;
		}
		if (l_init_status->class_inited) {
			class_unregister(&icm206xx_class);
			l_init_status->class_inited = false;
		}
		if (g_icm206xx_sensor->proc_debug) {
			proc_remove(g_icm206xx_sensor->proc_debug);
			g_icm206xx_sensor->proc_debug = NULL;
		}
		if (g_icm206xx_sensor->proc_polling) {
			proc_remove(g_icm206xx_sensor->proc_polling);
			g_icm206xx_sensor->proc_polling = NULL;
		}
		if (g_icm206xx_sensor->gyro_dev) {
			input_unregister_device(g_icm206xx_sensor->gyro_dev);
			g_icm206xx_sensor->gyro_dev = NULL;
		}
		if (g_icm206xx_sensor->accel_dev) {
			input_unregister_device(g_icm206xx_sensor->accel_dev);
			g_icm206xx_sensor->accel_dev = NULL;
		}
		if (g_icm206xx_sensor->input_dev_icm206xx) {
			input_unregister_device(g_icm206xx_sensor->input_dev_icm206xx);
			g_icm206xx_sensor->input_dev_icm206xx = NULL;
		}
		if (g_icm206xx_sensor->gyr_task) {
			kthread_stop(g_icm206xx_sensor->gyr_task);
			g_icm206xx_sensor->gyr_task = NULL;
		}
		if (g_icm206xx_sensor->accel_task) {
			kthread_stop(g_icm206xx_sensor->accel_task);
			g_icm206xx_sensor->accel_task = NULL;
		}
		if (l_init_status->timer_inited) {
			hrtimer_try_to_cancel(&g_icm206xx_sensor->gyro_timer);
			hrtimer_try_to_cancel(&g_icm206xx_sensor->accel_timer);
			l_init_status->timer_inited = false;
		}
		if (g_icm206xx_sensor->data_wq) {
			destroy_workqueue(g_icm206xx_sensor->data_wq);
			g_icm206xx_sensor->data_wq = NULL;
		}
		if (g_icm206xx_sensor->irq > 0) {
			free_irq(g_icm206xx_sensor->irq, g_icm206xx_sensor);
			g_icm206xx_sensor->irq = 0;
		}
		if (l_init_status->gpio_inited) {
			gpio_free((g_icm206xx_sensor->pdata).gpio_int);
			l_init_status->gpio_inited = false;
		}
		devm_kfree(g_icm206xx_sensor->dev, g_icm206xx_sensor);
		g_icm206xx_sensor = NULL;
	}
}
static u8 icm206xx_dumpReg_CMD_Table[] = {
	REG_SAMPLE_RATE_DIV,
	REG_CONFIG,
	REG_GYRO_CONFIG,
	REG_ACCEL_CONFIG,
	REG_INT_PIN_CFG,
	REG_INT_ENABLE,
	REG_INT_STATUS,
	REG_SIGNAL_PATH_RESET,
	REG_DETECT_CTRL,
	REG_PWR_MGMT_1,
	REG_PWR_MGMT_2,
	REG_OIS_ENABLE
};
static void dump_sensor_status(void)
{
	char sensorInfo[256] = "";
	s32 reg_value = -1;
	u8 shift;
	int i;
	for (i = 0; i < ARRAY_SIZE(icm206xx_dumpReg_CMD_Table); i++) {
		reg_value = icm_read_byte_data(g_icm206xx_sensor, icm206xx_dumpReg_CMD_Table[i]);
		snprintf(sensorInfo, 256, "%s0x%02x=%02x, ", sensorInfo, icm206xx_dumpReg_CMD_Table[i], reg_value);
	}
	icm_interrupt_routine(g_icm206xx_sensor->irq, g_icm206xx_sensor);
	icm_interrupt_thread(g_icm206xx_sensor->irq, g_icm206xx_sensor);
	shift = icm_accel_fs_shift[g_icm206xx_sensor->cfg.accel_fs];
	snprintf(sensorInfo, 256, "%saccelX=%d, ", sensorInfo, g_icm206xx_sensor->axis.x >> shift);
	snprintf(sensorInfo, 256, "%saccelY=%d, ", sensorInfo, g_icm206xx_sensor->axis.y >> shift);
	snprintf(sensorInfo, 256, "%saccelZ=%d, ", sensorInfo, g_icm206xx_sensor->axis.z >> shift);

	shift = icm_gyro_fs_shift[g_icm206xx_sensor->cfg.fsr];
	snprintf(sensorInfo, 256, "%sgyroX=%d, ", sensorInfo, g_icm206xx_sensor->axis.rx >> shift);
	snprintf(sensorInfo, 256, "%sgyroY=%d, ", sensorInfo, g_icm206xx_sensor->axis.ry >> shift);
	snprintf(sensorInfo, 256, "%sgyroZ=%d", sensorInfo, g_icm206xx_sensor->axis.rz >> shift);

	icm_dbgmsg("%s\n", sensorInfo);
}
static void print_sensor_status(void)
{
	char sensorInfo[256];

	snprintf(sensorInfo, sizeof(sensorInfo), "irq_counter = %llu, gpio_int = %d, accel_enable = %u, gyro_enable = %u, sample_rate = %u ms",
		g_icm_irq_counter,
		gpio_get_value((g_icm206xx_sensor->pdata).gpio_int),
		g_icm206xx_sensor->cfg.accel_enable,
		g_icm206xx_sensor->cfg.gyro_enable,
		g_icm206xx_sensor->cfg.rate_div + 1);
	icm_dbgmsg("%s\n", sensorInfo);
	if (g_icm_debugMode) {
		dump_sensor_status();
	}
}
void report_wq(struct work_struct *work)
{
	int l_maxTime = 600;
	if (g_icm_debugMode) {
		l_maxTime = 10;
	}
	print_sensor_status();

	/*ASUS_BSP: calculate report time, the report interval(s) will be: 0, 4, 8, 16, 32, 64, 128, 256, 512, 600*/
	if (g_icm_next_report_time_s != l_maxTime) {
		g_icm_next_report_time_s *= 2;
	}
	if (g_icm_next_report_time_s > l_maxTime) {
		g_icm_next_report_time_s = l_maxTime;
	}
	schedule_delayed_work(&g_icm_work_report, HZ * g_icm_next_report_time_s);
}
extern int asus_get_se_proto(struct spi_device *spi);
static bool icm_spi_check_bus(struct spi_device *spi)
{
	int proto = asus_get_se_proto(spi);
	icm_dbgmsg("proto = %d", proto);
	if (unlikely(proto != SPI)) {
		return false;
	}
	return true;
}
static void icm_irq_ctl(int irq, bool enable)
{
	static bool enabled = true;
	if (enabled != enable) {
		if (enable) {
			enable_irq(irq);
		} else{
			disable_irq(irq);
		}
		enabled = enable;
	}
}
/*
 * icm_probe() - device detection callback
 * @client: i2c client of found device
 * @id: id match information
 *
 * The I2C layer calls us when it believes a sensor is present at this
 * address. Probe to see if this is correct and to validate the device.
 *
 * If present install the relevant sysfs interfaces and input device.
 */
static int icm_spi_probe(struct spi_device *pdev)
{
	struct icm_sensor *sensor;
	struct icm_spi_data *pdata;
	static int l_retryCount = 0;
	int ret;

	icm_dbgmsg("+\n");
	if (!icm_spi_check_bus(pdev)) {
		icm_dbgmsg("invalid bus! might need to modify trustzone.");
		return -1;
	}
	if (icm206xx_check_probe_status(&pdev->dev)) {
		l_retryCount++;
		icm_errmsg("icm206xx_check_probe_status failed, defer probe, count = %d\n", l_retryCount);
		if (l_retryCount < 5) {
			return -EPROBE_DEFER;
		} else{
			return -1;
		}
	}
	sensor = devm_kzalloc(&pdev->dev, sizeof(struct icm_sensor),
			GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->pdev = pdev;
	sensor->dev = &pdev->dev;

	icm_init_status_function(sensor);

	dev_set_drvdata(&pdev->dev, sensor);
	g_icm206xx_sensor = sensor;
	pdata = &sensor->pdata;
	device_init_wakeup(sensor->dev, true);

	if (pdev->dev.of_node) {
		ret = icm_parse_dt(&pdev->dev, pdata);
		icm_dbgmsg("gpio_int=%d, int_flags=%d, gpio_en=%d, use_int=%d\n", pdata->gpio_int, pdata->int_flags, pdata->gpio_en, pdata->use_int ? 1 : 0);
		if (ret) {
			icm_errmsg("Failed to parse device tree\n");
			ret = -EINVAL;
			goto probe_failed;
		}
	} else {
		icm_errmsg("Failed to parse device tree\n");
		ret = -EINVAL;
		goto probe_failed;
	}

	mutex_init(&sensor->op_lock);
	mutex_init(&sensor->bus_lock);
	ret = icm206xx_init_regulator(sensor);
	if (ret) {
		icm_errmsg("icm206xx_init_regulator failed\n");
		goto probe_failed;
	}

	ret = icm_pinctrl_init(sensor);
	if (ret) {
		icm_errmsg("Can't initialize pinctrl\n");
		goto probe_failed;
	}
	ret = icm_pinctrl_state(sensor, true);
	if (ret) {
		icm_errmsg("Failed to power on device\n");
		goto probe_failed;
	}
	if ((pdata->use_int) &&
		gpio_is_valid(pdata->gpio_int)) {
		sensor->use_poll = 0;

		/* configure interrupt gpio */
		ret = gpio_request(pdata->gpio_int, "icm_gpio_int");
		if (ret) {
			icm_errmsg(	"Unable to request interrupt gpio %d\n", pdata->gpio_int);
			goto probe_failed;
		}
		(sensor->init_status).gpio_inited = true;

		ret = gpio_direction_input(pdata->gpio_int);
		if (ret) {
			icm_errmsg("Unable to set direction for gpio %d\n", pdata->gpio_int);
			goto probe_failed;
		}
		sensor->irq = gpio_to_irq(pdata->gpio_int);

		ret = request_threaded_irq(sensor->irq,
				icm_interrupt_routine, icm_interrupt_thread,
				pdata->int_flags | IRQF_ONESHOT,
				"icm", sensor);
		icm_irq_ctl(sensor->irq, false);
		if (ret) {
			icm_errmsg("Can't get IRQ %d, error %d\n",
				sensor->irq, ret);
			sensor->irq = 0;
			goto probe_failed;
		}

	} else {
		sensor->use_poll = 1;
	}
	ret = icm_power_up(sensor);
	if (ret) {
		icm_errmsg("icm206xx power_up error ret %d\n", ret);
		goto probe_failed;
	}

	sensor->cfg.is_asleep = false;
	atomic_set(&sensor->accel_en, 0);
	atomic_set(&sensor->gyro_en, 0);
	icm_init_config(sensor);
	sensor->accel_poll_ms = ICM_ACCEL_DEFAULT_POLL_INTERVAL_MS;
	sensor->gyro_poll_ms = ICM_GYRO_DEFAULT_POLL_INTERVAL_MS;

	sensor->data_wq = create_freezable_workqueue("icm_data_work");
	if (!sensor->data_wq) {
		icm_errmsg("Cannot create workqueue!\n");
		goto probe_failed;
	}

	INIT_DELAYED_WORK(&g_icm_data_retry_work, icm_data_retry_wq);
	INIT_DELAYED_WORK(&g_icm_work_report, report_wq);
	INIT_DELAYED_WORK(&sensor->fifo_flush_work, icm_fifo_flush_fn);
	INIT_WORK(&sensor->resume_work, icm_resume_work_fn);

	init_waitqueue_head(&sensor->gyro_wq);
	init_waitqueue_head(&sensor->accel_wq);

	hrtimer_init(&sensor->gyro_timer, CLOCK_BOOTTIME, HRTIMER_MODE_REL);
	sensor->gyro_timer.function = gyro_timer_handle;
	hrtimer_init(&sensor->accel_timer, CLOCK_BOOTTIME, HRTIMER_MODE_REL);
	sensor->accel_timer.function = accel_timer_handle;
	(sensor->init_status).timer_inited = true;

	sensor->gyro_wkp_flag = 0;
	sensor->accel_wkp_flag = 0;

	sensor->gyr_task = kthread_run(gyro_poll_thread, sensor, "sns_gyro");
	sensor->accel_task = kthread_run(accel_poll_thread, sensor, "sns_accel");

	g_icm206xx_status = icm_check_chip_type(sensor);

	if (g_icm206xx_status) {
		ret = icm_init_engine(sensor);
		if (ret) {
			icm_errmsg("Failed to init chip engine\n");
			goto probe_failed;
		}
		icm_reset_chip(sensor);
		ret = icm_restore_context(sensor, true);
		if (ret) {
			icm_errmsg("Failed to set default config\n");
			goto probe_failed;
		}
	}

	sensor->accel_dev = icm206xx_input_setup(sensor, 0);
	if (!sensor->accel_dev) {
		icm_errmsg("Failed to register accel input device\n");
		ret = -1;
		goto probe_failed;
	}
	sensor->gyro_dev = icm206xx_input_setup(sensor, 1);
	if (!sensor->gyro_dev) {
		icm_errmsg("Failed to register gyro input device\n");
		ret = -1;
		goto probe_failed;
	}
	ret = icm206xx_input_setup_BMMI(sensor);
	if (ret) {
		icm_errmsg("Failed to register input device - BMMI\n");
		goto probe_failed;
	}

	sensor->proc_polling = create_asusIcm206xxPolling_proc_file();
	if (!sensor->proc_polling) {
		icm_errmsg("Failed to register proc_polling\n");
		ret = -1;
		goto probe_failed;
	}
	sensor->proc_debug = create_asusIcm206xxDebug_proc_file();
	if (!sensor->proc_debug) {
		icm_errmsg("Failed to register proc_debug\n");
		ret = -1;
		goto probe_failed;
	}
	if (class_register(&icm206xx_class)) {
		icm_errmsg("Failed to register class\n");
		ret = -1;
		goto probe_failed;
	}
	(sensor->init_status).class_inited = true;
	if (icm206xx_miscRegister()) {
		icm_errmsg("Failed to register misc\n");
		ret = -1;
		goto probe_failed;
	}
	(sensor->init_status).misc_inited = true;

	icm_power_down(sensor);
	icm_dbgmsg("-\n");
	return 0;

probe_failed:
	icm_deinit();
	icm_errmsg("Probe device return error%d\n", ret);
	return ret;
}


/*
 * icm_remove() - remove a sensor
 * @client: i2c client of sensor being removed
 *
 * Our sensor is going away, clean up the resources.
 */
static int icm_spi_remove(struct spi_device *pdev)
{
	icm_dbgmsg("+\n");
	icm_deinit();
	icm_dbgmsg("-\n");
	return 0;
}

static void icm_resume_work_fn(struct work_struct *work)
{
	struct icm_sensor *sensor;
	int ret = 0;

	icm_dbgmsg("\n");
	sensor = container_of(work,
			struct icm_sensor, resume_work);

	mutex_lock(&sensor->op_lock);
	if ((sensor->batch_accel) || (sensor->batch_gyro)) {
		icm_set_interrupt(sensor,
				BIT_FIFO_OVERFLOW, true);
		icm_sche_next_flush(sensor);
	}

	if (sensor->cfg.mot_det_on) {
		/* keep accel on and config motion detection wakeup */
		irq_set_irq_wake(sensor->irq, 0);
		icm_set_motion_det(sensor, false);
		icm_set_interrupt(sensor, BIT_DATA_RDY_EN, true);
		goto exit;
	}

	/* Reset sensor to recovery from unexpected state */
	icm_reset_chip(sensor);

	ret = icm_restore_context(sensor, false);
	if (ret < 0) {
		icm_errmsg("Failed to restore context\n");
		goto exit;
	}

	/* Enter sleep mode if both accel and gyro are not enabled */
	ret = icm_set_power_mode(sensor, sensor->cfg.enable);
	if (ret < 0) {
		icm_errmsg("Failed to set power mode enable=%d\n",
				sensor->cfg.enable);
		goto exit;
	}

	if (sensor->cfg.gyro_enable) {
		ret = icm_gyro_enable(sensor, true);
		if (ret < 0) {
			icm_errmsg("Failed to enable gyro\n");
			goto exit;
		}

		if (sensor->use_poll) {
			ktime_t ktime;

			ktime = ktime_set(0, sensor->gyro_poll_ms * NSEC_PER_MSEC);
			hrtimer_start(&sensor->gyro_timer, ktime, HRTIMER_MODE_REL);
		} else{
			icm_set_interrupt(sensor, BIT_DATA_RDY_EN, true);
		}
	}

	if (sensor->cfg.accel_enable) {
		ret = icm_accel_enable(sensor, true);
		if (ret < 0) {
			icm_errmsg("Failed to enable accel\n");
			goto exit;
		}

		if (sensor->use_poll) {
			ktime_t ktime;

			ktime = ktime_set(0,
					sensor->accel_poll_ms * NSEC_PER_MSEC);
			hrtimer_start(&sensor->accel_timer, ktime,
					HRTIMER_MODE_REL);
		} else{
			icm_set_interrupt(sensor, BIT_DATA_RDY_EN, true);
		}
	}

exit:
	mutex_unlock(&sensor->op_lock);
}

#ifdef CONFIG_PM
/*
 * icm_suspend() - called on device suspend
 * @dev: device being suspended
 *
 * Put the device into sleep mode before we suspend the machine.
 */
static int icm_suspend(struct device *dev)
{
	struct icm_sensor *sensor = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&sensor->op_lock);
	if (!(sensor->cfg.gyro_enable) && !(sensor->cfg.accel_enable))
		goto exit;

	if ((sensor->batch_accel) || (sensor->batch_gyro)) {
		icm_set_interrupt(sensor,
				BIT_FIFO_OVERFLOW, false);
		cancel_delayed_work_sync(&sensor->fifo_flush_work);
		goto exit;
	}
	if (sensor->motion_det_en) {
		/* keep accel on and config motion detection wakeup */
		ret = icm_set_interrupt(sensor, BIT_DATA_RDY_EN, false);
		if (ret == 0) {
			ret = icm_set_motion_det(sensor, true);
		}
		if (ret == 0) {
			irq_set_irq_wake(sensor->irq, 1);
			goto exit;
		}
		/* if motion detection config does not success,
		 * not exit suspend and sensor will be power off.
		 */
	}

	if (sensor->use_poll) {
		if (sensor->cfg.gyro_enable) {
			ret = hrtimer_try_to_cancel(&sensor->gyro_timer);
		}

		if (sensor->cfg.accel_enable) {
			ret = hrtimer_try_to_cancel(&sensor->accel_timer);
		}
	}

	icm_set_power_mode(sensor, false);

exit:
	mutex_unlock(&sensor->op_lock);

	return ret;
}

/*
 * icm_resume() - called on device resume
 * @dev: device being resumed
 *
 * Put the device into powered mode on resume.
 */
static int icm_resume(struct device *dev)
{
	struct icm_sensor *sensor = dev_get_drvdata(dev);
	
	mutex_lock(&sensor->op_lock);

	if (sensor->cfg.gyro_enable || sensor->cfg.accel_enable) {
		queue_work(sensor->data_wq, &sensor->resume_work);
	}

	mutex_unlock(&sensor->op_lock);

	return 0;
}

static UNIVERSAL_DEV_PM_OPS(icm_pm, icm_suspend, icm_resume, NULL);
#endif

static int icm_power_ctl(struct icm_sensor *sensor, bool enable)
{
	int ret = 0;
	static int l_count = 0;

	icm_dbgmsg("enable = %d, l_count = %d\n", enable ? 1 : 0, l_count);
	if ((enable && l_count == 0) || (!enable && l_count == 1)) {
		if (enable) {
			ret = icm_power_up(sensor);
			ret |= icm_restore_context(g_icm206xx_sensor, false);
			/*ASUS_BSP: initial the report time, and continuously print some information when sensor is enabled*/
			g_icm_next_report_time_s = 2;
			schedule_delayed_work(&g_icm_work_report, 0);
		} else{
			ret = icm_power_down(sensor);
			/*ASUS_BSP: cancel report worker when sensor is disabled*/
			cancel_delayed_work(&g_icm_work_report);
		}
	}

	if (enable) {
		l_count++;
	} else{
		l_count--;
	}
	return ret;
}
static int icm_power_up(struct icm_sensor *sensor)
{
	int rc = 0;

	if (!sensor) {
		icm_errmsg( "stmvl53l1_power_up failed %d\n", __LINE__);
		return -EINVAL;
	}
	/* turn on power */
	if (sensor->vdd) {
		regulator_enable(sensor->vdd);
	}
	if (sensor->power_supply) {
		rc = regulator_enable(sensor->power_supply);
		if (rc) {
			icm_errmsg("fail to turn on icm206xx regulator\n");
			goto exit;
		}
		msleep(POWER_UP_TIME_MS);
	}
	g_icm206xx_sensor->power_enabled = true;
	if (g_icm206xx_sensor->irq) {
		icm_irq_ctl(g_icm206xx_sensor->irq, true);
	} else{
		icm_errmsg("invalid irq");
	}
	pm_stay_awake(g_icm206xx_sensor->dev);
exit:
	return rc;
}

static int icm_power_down(struct icm_sensor *sensor)
{
	int rc = 0;

	if (g_icm206xx_sensor->irq) {
		icm_irq_ctl(g_icm206xx_sensor->irq, false);
	} else{
		icm_errmsg("invalid irq");
	}
	g_icm206xx_sensor->power_enabled = false;
	if (!sensor) {
		icm_errmsg("icm_power_down failed %d\n", __LINE__);
		return -EINVAL;
	}

	/* turn off power */
	if (sensor->power_supply) {
		rc = regulator_disable(sensor->power_supply);
		if (rc) {
			icm_errmsg("reg disable failed. rc=%d\n", rc);
		}
	}
	if (sensor->vdd) {
		regulator_disable(sensor->vdd);
	}
	pm_relax(g_icm206xx_sensor->dev);
	return rc;
}

static s32 icm_read_byte_data(struct icm_sensor *sensor, u8 reg)
{
	uint8_t data;
	struct spi_message msg;
	int res;
	u8 d[1];
	struct spi_transfer xfers[] = {
		{
		 .tx_buf = d,
		 .bits_per_word = 8,
		 .len = 1,
		 },
		{
		 .rx_buf = &data,
		 .bits_per_word = 8,
		 .len = 1,
		 }
	};

	mutex_lock(&sensor->bus_lock);
	d[0] = (reg | INV_SPI_READ);

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);
	res = spi_sync(sensor->pdev, &msg);
	mutex_unlock(&sensor->bus_lock);

	return (res == 0 ? data : res);

}

static s32 icm_write_byte_data(struct icm_sensor *sensor, u8 reg, u8 data)
{
	int res = -1;
	int read_data;
	int count = 5;
	u8 i = 0;
	for (i = 0; i < count; i++) {
		res = icm_do_write_byte_data(sensor, reg, data);
		read_data = icm_read_byte_data(sensor, reg);
		if (read_data != data) {
			icm_errmsg("i = %u/%u, addr = %u, write data = %u, read data = %d, result = %d\n", i, count, reg, data, read_data, res);
		} else{
			break;
		}
	}
	return res;
}
static s32 icm_do_write_byte_data(struct icm_sensor *sensor, u8 reg, u8 data)
{
	struct spi_message msg;
	int res;
	u8 d[2];
	struct spi_transfer xfers = {
		.tx_buf = d,
		.bits_per_word = 8,
		.len = 2,
	};

	mutex_lock(&sensor->bus_lock);
	d[0] = reg;
	d[1] = data;
	spi_message_init(&msg);
	spi_message_add_tail(&xfers, &msg);
	res = spi_sync(sensor->pdev, &msg);
	if (g_icm_debugMode) {
		icm_dbgmsg("%02x: %02x\n", reg, data);
	}
	mutex_unlock(&sensor->bus_lock);

	return res;
}

static s32 icm_mask_write_byte_data(struct icm_sensor *sensor, u8 command, u8 mask, u8 value)
{
	s32 rc = -1, read_data;
 	u8 write_data;

	read_data = icm_read_byte_data(sensor, command);
	if (read_data < 0) {
		icm_errmsg("Failed to read reg 0x%02x, read_data=%d\n", command, read_data);
		goto out;
	}
	write_data = (u8)read_data;
	write_data &= ~mask;
	write_data |= value & mask;
	rc = icm_write_byte_data(sensor, command, write_data);
	if (rc) {
		icm_errmsg("Failed to write reg 0x%02x, rc=%d\n", command, rc);
	}
out:
	return rc;
}
static const struct spi_device_id icm_spi_id[] = {
	{ICM_SENSOR_NAME, 0},
	{},
};
static const struct of_device_id icm_of_match[] = {
	{ .compatible = "invn,icm20690", },
	{ },
};
MODULE_DEVICE_TABLE(of, icm_of_match);

static struct spi_driver icm_spi_driver = {
	.id_table = icm_spi_id,
	.probe          = icm_spi_probe,
	.remove         = icm_spi_remove,
	.driver = {
		.name   = ICM_SENSOR_NAME,
		.owner  = THIS_MODULE,
		.pm     = &icm_pm,
		.of_match_table = icm_of_match,
	},
};

module_spi_driver(icm_spi_driver);

MODULE_DESCRIPTION("ICM Tri-axis gyroscope driver");
MODULE_LICENSE("GPL v2");
