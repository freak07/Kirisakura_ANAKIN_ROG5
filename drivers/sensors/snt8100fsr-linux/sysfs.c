/*****************************************************************************
* File: sysfs.c
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
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/time.h>

#include "config.h"
#include "hardware.h"
#include "memory.h"
#include "file.h"
#include "event.h"

#include "sonacomm.h"
#include "utils.h"
#include "device.h"
#include "debug.h"
#include "workqueue.h"
#include "file_control.h"
#include "locking.h"
#include "customize.h"
#include "asus_init.h"

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/

/*==========================================================================*/
/* LOCAL PROTOTYPES                                                         */
/*==========================================================================*/
//int get_attrs_array_size(struct device_attribute *list[]);
//struct device_attribute *alloc_sysfs_attrs(void);
//void free_sysfs_attrs(struct attribute **attrs);
int dump_deep_trace_to_file(void);

/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/
//static struct   kobject *sysfs_kobj_g;

/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/

#ifdef DYNAMIC_PWR_CTL
/*
 * Utility function to perform Activity Request and wait for completion.
 *
 * set Request sema.
 * perform Activity Request
 * wait for ActivityRsp sema.
 */
int snt_activity_request_force(int force)
{
	int ret = 0;
	MUTEX_LOCK(&snt8100fsr_g->IRQ_WAKE_SLEEP_LOCK);
	MUTEX_LOCK(&snt8100fsr_g->sb_lock);
	if (snt8100fsr_g->enable_dpc_flag ||force) {
		up(&snt8100fsr_g->wake_req);

		/* Check wake device for 3.4.0 */
		ret = sb_wake_device(snt8100fsr_g);
		PRINT_DEBUG("sb_wake_device, snt_state=%d", snt8100fsr_g->snt_state);
		snt8100fsr_g->snt_state = GRIP_WAKEUP;
		mutex_unlock(&snt8100fsr_g->IRQ_WAKE_SLEEP_LOCK);
		if (ret) {
			PRINT_CRIT("sb_wake_device() failed");

			/* attempt to back out request */
			if (down_trylock(&snt8100fsr_g->wake_req)){
				PRINT_DEBUG("Wake Req alread consumed");
				mutex_unlock(&snt8100fsr_g->sb_lock);
				if(down_trylock(&snt8100fsr_g->wake_rsp)){
					PRINT_DEBUG("Wake Req alread consumed");
				}
			}else{
				mutex_unlock(&snt8100fsr_g->sb_lock);
			}
			return ret;
		}
		mutex_unlock(&snt8100fsr_g->sb_lock);
		
		snt8100fsr_g->stuck_retry_count = 0;
		workqueue_cancel_work(&check_stuck_wake);
		workqueue_queue_work(&check_stuck_wake, 300);
		do{
			snt8100fsr_g->stuck_flag = true;
			ret = down_interruptible(&snt8100fsr_g->wake_rsp);
			PRINT_DEBUG("recv wake rsp, ret=%d, res=%d", ret, snt8100fsr_g->wake_rsp_result);
			if(ret == -EINTR)
				msleep(5);
			//PRINT_INFO("wake-rsp wait down");
		}while(ret == -EINTR);
		snt8100fsr_g->stuck_flag = false;
		snt8100fsr_g->stuck_retry_count = 0;
		workqueue_cancel_work(&check_stuck_wake);
		if (ret==0){
			ret = snt8100fsr_g->wake_rsp_result;
		}
	}else{
		mutex_unlock(&snt8100fsr_g->sb_lock);
		mutex_unlock(&snt8100fsr_g->IRQ_WAKE_SLEEP_LOCK);
	}
	
	PRINT_FUNC("done:%d", ret);

	return ret;
}

int snt_activity_request(void)
{
	Grip_Driver_IRQ_EN(1);
	return snt_activity_request_force(0 /* No Force */ );
}


ssize_t sysfs_enable_dynamic_pwr_ctl_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {
	PRINT_FUNC("%lu bytes", count);
	if (count != 2) {
		PRINT_CRIT("Only single digit number '1' or '0' allowed");
		return -1;
	}

	MUTEX_LOCK(&snt8100fsr_g->sb_lock);

	snt8100fsr_g->enable_dpc_flag = (*buf == '0') ? 0 : 1;

	mutex_unlock(&snt8100fsr_g->sb_lock);

	PRINT_DEBUG("done. enable_dpc = %d", snt8100fsr_g->enable_dpc_flag);
	return count;
}

static ssize_t sysfs_enable_dynamic_pwr_ctl_show(struct device *dev,
                            struct device_attribute *attr,
                            char *buf) {
	int ret;
	PRINT_FUNC();
	ret = snprintf(buf, PAGE_SIZE, "%d\n", snt8100fsr_g->enable_dpc_flag);
	PRINT_DEBUG("done (%d)", ret);
	return ret;
}

#endif /* DYNAMIC_PWR_CTL */

/*
 * SysFS interface for showing and storing generic registers
 */
static ssize_t sysfs_register_show(struct device *dev,
                            struct device_attribute *attr,
                            char *buf) {
	uint16_t reg_value;
	char *reg_name;
	int ret;
	int reg;

#ifdef DYNAMIC_PWR_CTL
	if (snt_activity_request() != 0) {
		PRINT_CRIT("snt_activity_request() failed");
		return -1;
	}
#endif

	// Figure out which register we are to show
	reg_name = (char *)&attr->attr.name[strlen(REGISTER_PREFIX)];
	PRINT_INFO("%s", reg_name);

	reg = register_number_for_key(reg_name);

	// Read the register
	ret = read_register(snt8100fsr_g, reg, &reg_value);
	if (ret) {
		PRINT_CRIT("Unable to read register %s: %d", reg_name, ret);
		return -1;
	}

	ret = snprintf(buf, PAGE_SIZE, "%u\n", reg_value);
	PRINT_DEBUG("done");
	return ret;
}

static ssize_t sysfs_register_store(struct device *dev,
                             struct device_attribute *attr,
                             const char *buf,
                             size_t count) {
	uint16_t reg_value;
	uint32_t value;
	char *reg_name;
	int ret;
	int reg;
	uint16_t frame_rate;

#ifdef DYNAMIC_PWR_CTL
	if (snt_activity_request() != 0) {
		PRINT_CRIT("snt_activity_request() failed");
		return -1;
	}
#endif

	// Figure out which register we are to store
	reg_name = (char *)&attr->attr.name[strlen(REGISTER_PREFIX)];

	reg = register_number_for_key(reg_name);

	// Convert the input string to a numerical value
	if(string_to_uint32(buf, &value) == 0) {
		reg_value = (uint16_t)value;
		PRINT_FUNC("%s, 0x%02x", reg_name, reg_value);
		ret = write_register(snt8100fsr_g, reg, &reg_value);
		if (ret) {
			PRINT_CRIT("Unable to write register %s: %d", reg_name, ret);
			return -1;
		}

		if (reg == REGISTER_FRAME_RATE) {
			/*
			* Only save fully working frame rates, at and below 5hz and 
			* the value of 0xFF for deep sleep, are special temporary rates.
 			*/
			frame_rate = reg_value & 0xff; // frame rate in LSB, decimation in MSB
			if (frame_rate > DEFAULT_SUSPENDED_FRAME_RATE && frame_rate != DEEP_SLEEP_FRAME_RATE) {
				snt8100fsr_g->frame_rate = reg_value;
			}
		}
	} else {
		PRINT_FUNC("%s, %zu bytes", reg_name, count);
		PRINT_NOTICE("Invalid input: %s", buf);
		return -1;
	}

	PRINT_DEBUG("done");
	return count;
}

/*
 * SysFS interface for reading and writing a generic 32 bit value
 */
static ssize_t sysfs_uint32_show(struct device *dev,
                                 struct device_attribute *attr,
                                 uint32_t *value,
                                 char *buf) {
	return snprintf(buf, PAGE_SIZE, "%d\n", *value);
}

static ssize_t sysfs_uint32_store(struct device *dev,
                                  struct device_attribute *attr,
                                  uint32_t *result,
                                  const char *buf,
                                  size_t count) {
	long value;

	PRINT_FUNC("%zu bytes", count);

	if (count < 2) {
		return -1;
	}

	if(kstrtol(buf, 10, &value) == 0) {
		PRINT_DEBUG("Storing value %d into result", (uint32_t)value);
		*result = (uint32_t)value;
	} else {
		PRINT_NOTICE("Invalid SysFS Value: %s", buf);
		return -1;
	}

	return count;
}

/*
 * SysFS interface for logging the no touch frame report
 */
ssize_t sysfs_log_no_touch_frame_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf) {
	return snprintf(buf, PAGE_SIZE, "%d\n", log_no_touch_frame_file != NULL ? 1 : 0);
}

ssize_t sysfs_log_no_touch_frame_store(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf,
                                       size_t count) {
	int ret;
	PRINT_FUNC("%zu bytes", count);

	if (count > 2) {
		return -1;
	}

	if (*buf == '0') {
		PRINT_WARN("Disable no_touch logging not supported. It will auto"
				"-disable upon completion.");
		return -1;
	} else if (*buf == '1') {
	    
#ifdef DYNAMIC_PWR_CTL
	if (snt_activity_request() != 0) {
		PRINT_CRIT("snt_activity_request() failed");
		return -1;
	}
#endif

		ret = enable_no_touch_logging(snt8100fsr_g);
		if (ret) {
			PRINT_CRIT("Enable no_touch logging failed: %d", ret);
			return -1;
		}
	} else {
		PRINT_NOTICE("Invalid SysFS Value: 0x%X", *buf);
		return -1;
	}

	return count;
}

ssize_t sysfs_sys_track_report_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf,
                                size_t count) {
	int ret;
	PRINT_FUNC("%zu bytes", count);

	if (count != 2) {
		PRINT_CRIT("Only single digit number '1' or '0' allowed");
		return -1;
	}

	if (*buf == '1') {
		PRINT_DEBUG("Waking device now");
		MUTEX_LOCK(&snt8100fsr_g->sb_lock);

		ret = sb_wake_device(snt8100fsr_g);

		if (ret) {
			PRINT_CRIT("sb_wake_device() failed");
			mutex_unlock(&snt8100fsr_g->sb_lock);
			return ret;
		}

		mutex_unlock(&snt8100fsr_g->sb_lock);
	} else if (*buf == '0') {
		PRINT_DEBUG("Do nothing since '0' received");
	} else {
		PRINT_CRIT("Only single digit number '1' or '0' allowed");
		return -1;
	}

	PRINT_DEBUG("done");
	return count;
}
/*
 * SysFS interface to get a copy of the latest track report
 */
ssize_t sysfs_track_report_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf) {
	int i = 0;
	ssize_t ret = 0;

	MUTEX_LOCK(&snt8100fsr_g->track_report_sysfs_lock);
	for (i = 0; i < snt8100fsr_g->track_reports_count; i++) {
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "%u %u %u %u %u %u %u\n",
					snt8100fsr_g->track_reports_frame,
					snt8100fsr_g->track_reports[i].bar_id,
					snt8100fsr_g->track_reports[i].trk_id,
					snt8100fsr_g->track_reports[i].force_lvl,
					snt8100fsr_g->track_reports[i].top,
					snt8100fsr_g->track_reports[i].center,
					snt8100fsr_g->track_reports[i].bottom);
	}

	snt8100fsr_g->track_reports_count = 0;

	mutex_unlock(&snt8100fsr_g->track_report_sysfs_lock);
	return ret;
}

ssize_t sysfs_track_report_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf,
                              size_t count) {
	long value;
	PRINT_FUNC("%zu bytes", count);

	if (count > 10) {
		return -1;
	}
	
	if(kstrtol(buf, 10, &value) == 0) {
		snt8100fsr_g->en_demo = value;
		PRINT_DEBUG("en_demo = %d", snt8100fsr_g->en_demo);
		return count;
	} 

	PRINT_NOTICE("Invalid SysFS Value: 0x%X", *buf);
	return -1;
}

/*
 * SysFS interface for starting and stopping the logging of track reports
 */
ssize_t sysfs_log_track_report_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n",
                    log_track_reports_file != NULL ? 1 : 0);
}

ssize_t sysfs_log_track_report_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
	int ret;

	PRINT_FUNC("%zu bytes", count);

	if (count > 2) {
		return -1;
	}
	
#ifdef DYNAMIC_PWR_CTL
		if (snt_activity_request() != 0) {
			PRINT_CRIT("snt_activity_request() failed");
			return -1;
		}
#endif

	if (*buf == '0') {
		ret = enable_track_report_logging(false, 0/*txt*/);
		if (ret) {
			PRINT_CRIT("Unable to disable track report logging: %d", ret);
			return -1;
		}
	} else if (*buf == '1') {
		ret = enable_track_report_logging(true, 0/*txt*/);
		if (ret) {
			PRINT_CRIT("Unable to enable track report logging: %d", ret);
			return -1;
		}
	} else {
		PRINT_NOTICE("Invalid SysFS Value: 0x%X", *buf);
		return -1;
	}

	return count;
}


/*
 * SysFS interface for retrieving the event log
 */
ssize_t sysfs_event_log_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n",
                    snt8100fsr_g->event_log_file != NULL ? 1 : 0);
}

ssize_t sysfs_event_log_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
	int ret;

	PRINT_FUNC("%zu bytes", count);

	if (count > 2) {
		return -1;
	}
	
#ifdef DYNAMIC_PWR_CTL
		if (snt_activity_request() != 0) {
			PRINT_CRIT("snt_activity_request() failed");
			return -1;
		}
#endif

	if (*buf == '1') {
		ret = enable_event_logging(snt8100fsr_g);
		if (ret) {
			PRINT_CRIT("Unable to enable event logging: %d", ret);
			return -1;
		}
	} else {
		PRINT_NOTICE("Invalid SysFS Value: 0x%X", *buf);
		return -1;
	}

	// wait for response from driver irpt thread
	PRINT_DEBUG("EventLog Rsp -- wait");
	do {
		ret = down_interruptible(&snt8100fsr_g->sc_wf_rsp);
		PRINT_DEBUG("EventLog Rsp -- acquired %d", ret);
	} while (ret == -EINTR);

	return count;
}


ssize_t sysfs_log_track_report_bin_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", log_track_reports_bin_file != NULL ? 1 : 0);
}

ssize_t sysfs_log_track_report_bin_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
	int ret;

	PRINT_FUNC("%zu bytes", count);

	if (count > 2) {
		return -1;
	}
	
#ifdef DYNAMIC_PWR_CTL
		if (snt_activity_request() != 0) {
			PRINT_CRIT("snt_activity_request() failed");
			return -1;
		}
#endif

	if (*buf == '0') {
		ret = enable_track_report_logging(false, 1/*bin*/);
		if (ret) {
			PRINT_CRIT("Unable to disable track report logging: %d", ret);
			return -1;
		}
	} else if (*buf == '1') {
		ret = enable_track_report_logging(true, 1/*bin*/);
		if (ret) {
			PRINT_CRIT("Unable to enable track report logging: %d", ret);
			return -1;
		}
	} else {
		PRINT_NOTICE("Invalid SysFS Value: 0x%X", *buf);
		return -1;
	}

	return count;
}


ssize_t sysfs_deep_trace_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
	int ret;

	PRINT_FUNC("%zu bytes", count);

	if (count > 2) {
		PRINT_NOTICE("ERROR. Only one byte allowed %zu", count);
		return -1;
	}
	
#ifdef DYNAMIC_PWR_CTL
	if (snt_activity_request() != 0) {
		PRINT_CRIT("snt_activity_request() failed");
		return -1;
	}
#endif

	if (*buf == '1') {
		ret = dump_deep_trace_to_file();
		if (ret) {
			PRINT_CRIT("Unable to get deep trace: %d", ret);
			return -1;
		}
	} else {
		PRINT_NOTICE("Invalid SysFS Value: 0x%X", *buf);
		return -1;
	}

	return count;
}


/*
 * SysFS interface for starting and stopping the logging of d1test data
 */
ssize_t sysfs_log_d1test_show(struct device *dev,
                              struct device_attribute *attr,
                              char *buf) {
    return snprintf(buf, PAGE_SIZE, "%d\n",
                    log_d1test_file != NULL ? 1 : 0);
}

ssize_t sysfs_log_d1test_store(struct device *dev,
                               struct device_attribute *attr,
                               const char *buf,
                               size_t count) {
	int ret;
	long value;
	PRINT_FUNC("%zu bytes", count);

	if (count > 10) {
		PRINT_CRIT("bad argument size");
		return -1;
	}

#ifdef DYNAMIC_PWR_CTL
	if (snt_activity_request() != 0) {
		PRINT_CRIT("snt_activity_request() failed");
		return -1;
	}
#endif

	if(kstrtol(buf, 10, &value) == 0) {
		PRINT_DEBUG("d1test frames %d", (int)value);
		if (value == 1) {
			value = 64*1024;
		}
		ret = enable_d1test_logging(snt8100fsr_g, (int)value);
	} else {
		PRINT_CRIT("Bad Arguement %s", buf);
		return -1;
	}

	PRINT_FUNC("done");
	return count;
}

/*
 * SysFS interface for starting and stopping the logging of frame data
 */
ssize_t sysfs_log_frames_show(struct device *dev,
                             struct device_attribute *attr,
                             char *buf) {
    return snprintf(buf, PAGE_SIZE, "%d\n",
                    log_frame_file != NULL ? 1 : 0);
}

ssize_t sysfs_log_frames_store(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf,
                              size_t count) {
	int ret;
	long value;
	PRINT_FUNC("%zu bytes", count);

	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	if (count > 10) {
		goto error_state;
	}
	
#ifdef DYNAMIC_PWR_CTL
		if (snt_activity_request() != 0) {
			PRINT_CRIT("snt_activity_request() failed");
			goto error_state;
		}
#endif
	
	if(kstrtol(buf, 10, &value) == 0) {
		if (value == 0) {
			ret = enable_frame_logging(snt8100fsr_g, 0);
			if (ret) {
				PRINT_CRIT("Disable frame logging failed: %d", ret);
				goto error_state;
			}
			mutex_unlock(&snt8100fsr_g->ap_lock);
			return count;
		} else {
			// -1 means log as long as possible.
			if (value == -1) {
				value = 0x7FFFFFFF;
			}

			ret = enable_frame_logging(snt8100fsr_g, (uint32_t)value);
			if (ret) {
				PRINT_CRIT("Enable frame logging failed: %d", ret);
				goto error_state;
			}
			mutex_unlock(&snt8100fsr_g->ap_lock);
			return count;
		}
	}

	PRINT_NOTICE("Invalid SysFS Value: 0x%X", *buf);

error_state:
	PRINT_INFO("unlock");
	mutex_unlock(&snt8100fsr_g->ap_lock);
	return -1;
}

/*
 * SysFS interface for product config string
 */
ssize_t sysfs_product_config_show(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf) {
	char *product_string;
	int ret;
	    
	PRINT_FUNC();
#ifdef DYNAMIC_PWR_CTL
		if (snt_activity_request() != 0) {
			PRINT_CRIT("snt_activity_request() failed");
			return -1;
		}
#endif
	product_string = memory_allocate(PRODUCT_CONFIG_MAX_LEN, 0);
	if (product_string == NULL) {
		PRINT_CRIT("memory_allocate(PRODUCT_CONFIG_MAX_LEN) failed");
		return -1;
	}

	ret = read_product_config(snt8100fsr_g, product_string);
	if (ret) {
		memory_free(product_string);
		PRINT_WARN("Unable to read product config");
		return ret;
	}

	ret = snprintf(buf, PAGE_SIZE, "%s\n", product_string);
	memory_free(product_string);
	PRINT_FUNC("done");
	return ret;
}

/*
 * SysFS interface for version string
 */
ssize_t sysfs_version_show(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf) {

	return snprintf(buf, PAGE_SIZE, "%s\n", SNT_VERSION);
}

/*
 * SysFS interface for setting the suspended frame rate.
 * This frame rate will be set when i2c_suspend() or spi_suspend() is called.
 */
ssize_t sysfs_suspended_frame_rate_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf) {
	int ret;
	PRINT_FUNC();
	ret = sysfs_uint32_show(dev, attr, &snt8100fsr_g->suspended_frame_rate, buf);
	PRINT_DEBUG("done");
	return ret;
}

ssize_t sysfs_suspended_frame_rate_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf,
                                         size_t count) {
	int ret;
	PRINT_FUNC();
	ret = sysfs_uint32_store(dev, attr, &snt8100fsr_g->suspended_frame_rate, buf, count);
	PRINT_DEBUG("done");
	return ret;
}

/*
 * SysFS interface for waking up the device
 */
ssize_t sysfs_wake_device_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf,
                                size_t count) {
	int ret = -1;
	PRINT_FUNC("%zu bytes", count);

	if (count != 2) {
		PRINT_CRIT("Only single digit number '1' or '0' allowed");
		return ret;
	}

	if (*buf == '1') {
		PRINT_DEBUG("Waking device now");
	Grip_Driver_IRQ_EN(1);
#ifdef DYNAMIC_PWR_CTL
	if (snt_activity_request_force(1  /* force */) != 0) {
		PRINT_CRIT("snt_activity_request() failed");
		return -1;
	}
#else
		MUTEX_LOCK(&snt8100fsr_g->sb_lock);
		ret = sb_wake_device(snt8100fsr_g);
		if (ret) {
			PRINT_CRIT("sb_wake_device() failed");
			mutex_unlock(&snt8100fsr_g->sb_lock);
			return -1;
		}
		mutex_unlock(&snt8100fsr_g->sb_lock);
#endif
	//Grip_Chip_IRQ_EN(1);
	} else if (*buf == '0') {
		PRINT_DEBUG("Do nothing since '0' received");
	} else {
		PRINT_CRIT("Only single digit number '1' or '0' allowed");
		return -1;
	}

	PRINT_DEBUG("done");
	return count;
}

/*
 * SysFS interface for re-loading the registers from disk
 */
ssize_t sysfs_load_registers_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {
	PRINT_FUNC("%zu bytes", count);

	if (count != 2) {
		PRINT_CRIT("Only single digit number '1' or '0' allowed");
		return -1;
	}

	if (*buf == '1') {
		PRINT_DEBUG("Reloading registers now");

#ifdef DYNAMIC_PWR_CTL
	if (snt_activity_request() != 0) {
		PRINT_CRIT("snt_activity_request() failed");
		return -1;
	}
#endif

		load_registers_from_disk(snt8100fsr_g);
	} else if (*buf == '0') {
		PRINT_DEBUG("Do nothing since '0' received");
	} else {
		PRINT_CRIT("Only single digit number '1' or '0' allowed");
		return -1;
	}

	PRINT_DEBUG("done");
	return count;
}

/*
 * SysFS interface for showing a list of our available hardware registers
 * and corresponding numerical address.
 */
ssize_t sysfs_list_registers_show(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf) {
	int ret = 0;
	int i = 0;
	char *name = register_dict_g[i].str;

	while (name) {
		ret += snprintf(buf+ret, PAGE_SIZE-ret, "%s%s 0x%x (%d)\n",
				REGISTER_PREFIX, name, register_dict_g[i].n,
				register_dict_g[i].n);

		name = register_dict_g[++i].str;
	}
	return ret;
}

ssize_t do_write_reg_script(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count,
                                           int sc_maj_id) {
	char *p_buf = NULL;
	char *fname = NULL;
	struct file *p_file = NULL;
	int ret;
	int frp_size;
	int bytes;

	PRINT_FUNC("count = %lu", count);

	if (count < 2) {
		PRINT_DEBUG("no file name supplied");
		goto cleanup;
	}
	fname = memory_allocate(count,0);
	if (fname == NULL) {
		PRINT_CRIT("could not allocate memory for fname");
		goto cleanup;
	}
	memcpy(fname, buf, count);
	fname[count-1] = '\0';
	PRINT_DEBUG("opening frp file %s", fname);
	ret = file_open(fname, O_RDONLY, 0, &p_file);
	if (ret) {
		PRINT_CRIT("Unable to open file %s, error %d", fname, ret);
		goto cleanup;
	}
	ret = file_size(p_file, &frp_size);
	if (ret) {
		PRINT_CRIT("Unable to get file size error %d", ret);
		goto cleanup;
	}
	PRINT_DEBUG("Frp file size = %d", frp_size);
	p_buf = memory_allocate(frp_size,0);
	if (p_buf == NULL) {
		PRINT_CRIT("Failed to get Frp input buffer of size %d", frp_size);
		goto cleanup;
	}

	bytes = file_read(p_file, 0, p_buf, frp_size);
	if (bytes < 0) {
		PRINT_CRIT("read of Frp file failed %d", bytes);
		goto cleanup;
	}
	PRINT_DEBUG("Finished read of Frp file");

#ifdef DYNAMIC_PWR_CTL
	if (snt_activity_request() != 0) {
		PRINT_CRIT("snt_activity_request() failed");
		return -1;
	}
#endif

	enable_write_flash_reg_part_req(snt8100fsr_g, p_buf, frp_size, sc_maj_id);

cleanup:
	if (fname != NULL) {
		memory_free(fname);
	}
	if (p_file != NULL) {
		file_close(p_file);
	}
	if (p_buf != NULL) {
		memory_free(p_buf);
	}
	PRINT_DEBUG("done.");
	return count;
}

#ifdef SUPPORT_FLASH

/*
 * SysFS interface for showing status of fwupdate
 */
ssize_t sysfs_fwupdate_show(struct device *dev,
                            struct device_attribute *attr,
                            char *buf)
{
    int ret;
    PRINT_FUNC("tx_mtu=%d, status=%d", snt8100fsr_g->fwupdate_tx_mtu, snt8100fsr_g->fwupdate_status);
    ret  = snprintf(buf, PAGE_SIZE, "num_pkts = %d\n", snt8100fsr_g->fwupdate_tx_mtu);
    ret += snprintf(buf+ret, PAGE_SIZE, "tot_pkts = %d\n", snt8100fsr_g->fwupdate_tot_mtu);
    ret += snprintf(buf+ret, PAGE_SIZE, "status   = %d\n", snt8100fsr_g->fwupdate_status);
    PRINT_DEBUG("done %d", ret);
    return ret;
}

/*
 * SysFS interface updating the firmware image on system flash
 */
ssize_t sysfs_fwupdate_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {
	char *p_buf;
	PRINT_FUNC("%zu bytes", count);
	    
#ifdef DYNAMIC_PWR_CTL
	if (snt_activity_request() != 0) {
		PRINT_CRIT("snt_activity_request() failed");
		return -1;
	}
#endif

	p_buf = memory_allocate(count, 0);
	memcpy(p_buf, buf, count);
	p_buf[count-1] = '\0'; // remove \n

	enable_fwupdate(snt8100fsr_g, p_buf);
	memory_free(p_buf);

	PRINT_DEBUG("done.");
	return count;
}

ssize_t sysfs_write_flash_reg_part_file_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count) {
                                           
#ifdef DYNAMIC_PWR_CTL
	if (snt_activity_request() != 0) {
		PRINT_CRIT("snt_activity_request() failed");
		return -1;
	}
#endif
	return do_write_reg_script( dev, attr, buf, count, mc_update_regs);
}

ssize_t sysfs_write_flash_reg_part_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count) {
	PRINT_FUNC("%zu bytes", count);   

#ifdef DYNAMIC_PWR_CTL
	if (snt_activity_request() != 0) {
		PRINT_CRIT("snt_activity_request() failed");
		return -1;
	}
#endif


	enable_write_flash_reg_part_req(snt8100fsr_g, buf, count, mc_update_regs);
	PRINT_DEBUG("done.");
	return count;
}


ssize_t sysfs_read_flash_reg_part_store (struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count) {
	int ret;
	struct file *frp_file;
	int frp_file_offset = 0;
	int pcnt;
	int len;
	int i=0;
	int j;
	uint16_t reg_id;
	uint8_t num_reg;
	uint8_t *out_buf=NULL;

	PRINT_FUNC("%lu bytes", count);

#ifdef DYNAMIC_PWR_CTL
		if (snt_activity_request() != 0) {
			PRINT_CRIT("snt_activity_request() failed");
			goto errexit;
		}
#endif


	MUTEX_LOCK(&snt8100fsr_g->sb_lock);

	PRINT_DEBUG("Creating Frp capture file: %s", FRP_CAPTURE_FILE_LOCATION);

	ret = file_open(FRP_CAPTURE_FILE_LOCATION,
					O_WRONLY|O_CREAT|O_TRUNC, 0777,
					&frp_file);
	if(ret) {
		PRINT_DEBUG("Unable to create file '%s', error %d",
					FRP_CAPTURE_FILE_LOCATION, ret);
		goto errexit;
	}

	// allocate temp buf for writes
	out_buf = (uint8_t*) memory_allocate( PAGE_SIZE, GFP_DMA);
	if (out_buf == NULL) {
		PRINT_CRIT("unable to allocate out_buf");
		goto errexit;
	}

	// allocate buffer to get Flash Register Partition
	if (snt8100fsr_g->reg_part_buf != NULL) {
		memory_free(snt8100fsr_g->reg_part_buf);
	}
	snt8100fsr_g->reg_part_buf = (uint16_t*)memory_allocate(REG_PART_MAX_LEN, GFP_DMA);
	if (snt8100fsr_g->reg_part_buf == NULL) {
		PRINT_CRIT("memory_allocate(%d) failed", REG_PART_MAX_LEN);
		goto errexit;
	}

	/* set up CfgBank regs for read partition 1 */
	snt8100fsr_g->reg_part_buf[0] = 0;// offset = 0
	snt8100fsr_g->reg_part_buf[1] = 0;// length = 0
	snt8100fsr_g->reg_part_buf[2] = 0x0101; // READ partition 1
	ret = sb_write_fifo(snt8100fsr_g, REGISTER_BNK_CFG_OFFSET, 3*sizeof(uint16_t), snt8100fsr_g->reg_part_buf);
	if (ret) {
		PRINT_CRIT("Unable to write to registers: %d", ret);
		goto errexit;
	}

	// Read FIFO Length and data
	ret = sb_read_fifo(snt8100fsr_g, REGISTER_FIFO_CONFIG, sizeof(uint16_t), snt8100fsr_g->reg_part_buf);

	if (ret) {
		PRINT_CRIT("Unable to read Config FIFO length: %d", ret);
		goto errexit;
	}
	len = snt8100fsr_g->reg_part_buf[0];
	PRINT_DEBUG("Flash Register Partition Length = %d", len);
	if (len > REG_PART_MAX_LEN) {
		PRINT_CRIT("Flash Register Partition length greater than max %d (%d)",
				REG_PART_MAX_LEN, len);
		goto errexit;
	}
	if (len&1) {
		PRINT_CRIT("Flash Register Partition length must be even number: %d", len);
		goto errexit;
	}
	if (len == 0) {
		PRINT_DEBUG("Flash Register Partition length == 0.");
		// empty partition is not an error
		goto exit;
	}
	ret = sb_read_fifo(snt8100fsr_g, REGISTER_FIFO_CONFIG, len, snt8100fsr_g->reg_part_buf);

	if (ret) {
		PRINT_CRIT("Unable to read Config FIFO length: %d", ret);
		goto errexit;
	}
	// Process output
	pcnt = snprintf(out_buf, PAGE_SIZE, "\n");
	file_write(frp_file, frp_file_offset, out_buf, pcnt);
	frp_file_offset += pcnt;
	reg_id = snt8100fsr_g->reg_part_buf[i]&0x00ff;// LSB
	num_reg = (snt8100fsr_g->reg_part_buf[i]>>8) & 0x00ff;// MSB
	PRINT_DEBUG("reg_id=0x%x, num_reg=%d", reg_id, num_reg);
	while (reg_id != 0 && num_reg != 0 && i < len/2) {
		i++;
		if (num_reg == 0) {
			PRINT_CRIT("num_reg in Flash Register Partition is 0 at pos %d", i);
			goto errexit;
		}
		if (num_reg+i >= len/2) {
			PRINT_CRIT("num_reg exceeds register list size: (%d, %d, %d)", num_reg, i, len/2);
			goto errexit;
		}
		if (reg_id == 0x82) reg_id = 0x200;
		
		pcnt = snprintf(out_buf, PAGE_SIZE, "0x%x %d", reg_id, num_reg);
		file_write(frp_file, frp_file_offset, out_buf, pcnt);
		frp_file_offset += pcnt;
		for (j=0; j < num_reg; j++) {
			if (j % 8 == 0) {
				pcnt = snprintf(out_buf, PAGE_SIZE, "\n");
				file_write(frp_file, frp_file_offset, out_buf, pcnt);
				frp_file_offset += pcnt;
			}
			pcnt = snprintf(out_buf, PAGE_SIZE, " 0x%04x", snt8100fsr_g->reg_part_buf[i+j]);
			file_write(frp_file, frp_file_offset, out_buf, pcnt);
			frp_file_offset += pcnt;
		}
		i += j;
		pcnt = snprintf(out_buf, PAGE_SIZE, "\n");
		file_write(frp_file, frp_file_offset, out_buf, pcnt);
		frp_file_offset += pcnt;
		reg_id = snt8100fsr_g->reg_part_buf[i]&0x00ff;// LSB
		num_reg = (snt8100fsr_g->reg_part_buf[i]>>8) & 0x00ff;// MSB
		PRINT_DEBUG("reg_id=0x%x, num_reg=%d", reg_id, num_reg);
	}
exit:
	if (snt8100fsr_g->reg_part_buf != NULL) {
		memory_free(snt8100fsr_g->reg_part_buf);
		snt8100fsr_g->reg_part_buf = NULL;
	}
	if (out_buf != NULL) {
		memory_free(out_buf);
	}
	if (frp_file) {
		file_close(frp_file);
	}
	mutex_unlock(&snt8100fsr_g->sb_lock);
	PRINT_DEBUG("done %d", frp_file_offset);
	return count;

errexit:
	count = -1;
	goto exit;
}

/*
 * SysFS interface to enable reporting to demo app
 */
 
ssize_t sysfs_enable_demo_app_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf) {
    return snprintf(buf, PAGE_SIZE, "%d\n", snt8100fsr_g->en_demo);
}

ssize_t sysfs_enable_demo_app_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t count) {
	long value;
	PRINT_FUNC("%zu bytes", count);

	if (count > 10) {
		return -1;
	}

	if(kstrtol(buf, 10, &value) == 0) {
		snt8100fsr_g->en_demo = value;
		PRINT_DEBUG("en_demo = %d", snt8100fsr_g->en_demo);
		return count;
	}

	PRINT_NOTICE("Invalid SysFS Value: 0x%X", *buf);
	return -1;
}

/*
 * SysFS interface for read the Flash Register Partition
 */
ssize_t sysfs_read_flash_reg_part_show(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf)
{
	int ret;
	int count=0;
	int len;
	int i=0;
	int j;
	uint16_t reg_id;
	uint8_t num_reg;

	PRINT_FUNC("Enter.");

#ifdef DYNAMIC_PWR_CTL
			if (snt_activity_request() != 0) {
				PRINT_CRIT("snt_activity_request() failed");
				return -1;
			}
#endif


	MUTEX_LOCK(&snt8100fsr_g->sb_lock);

	// allocate buffer to get Flash Register Partition
	if (snt8100fsr_g->reg_part_buf != NULL) {
		memory_free(snt8100fsr_g->reg_part_buf);
	}
	snt8100fsr_g->reg_part_buf = (uint16_t*)memory_allocate(REG_PART_MAX_LEN, GFP_DMA);
	if (snt8100fsr_g->reg_part_buf == NULL) {
		PRINT_CRIT("memory_allocate(%d) failed", REG_PART_MAX_LEN);
		goto errexit;
	}

	/* set up CfgBank regs for read partition 1 */
	snt8100fsr_g->reg_part_buf[0] = 0;// offset = 0
	snt8100fsr_g->reg_part_buf[1] = 0;// length = 0
	snt8100fsr_g->reg_part_buf[2] = 0x0101; // READ partition 1
	ret = sb_write_fifo(snt8100fsr_g, REGISTER_BNK_CFG_OFFSET, 3*sizeof(uint16_t), snt8100fsr_g->reg_part_buf);
	if (ret) {
		PRINT_CRIT("Unable to write to registers: %d", ret);
		goto errexit;
	}

	// Read FIFO Length and data
	ret = sb_read_fifo(snt8100fsr_g, REGISTER_FIFO_CONFIG, sizeof(uint16_t), snt8100fsr_g->reg_part_buf);

	if (ret) {
		PRINT_CRIT("Unable to read Config FIFO length: %d", ret);
		goto errexit;
	}
	len = snt8100fsr_g->reg_part_buf[0];
	PRINT_DEBUG("Flash Register Partition Length = %d", len);
	if (len > REG_PART_MAX_LEN) {
		PRINT_CRIT("Flash Register Partition length greater than max %d (%d)", REG_PART_MAX_LEN, len);
		goto errexit;
	}
	if (len&1) {
		PRINT_CRIT("Flash Register Partition length must be even number: %d", len);
		goto errexit;
	}
	if (len == 0) {
		PRINT_DEBUG("Flash Register Partition length == 0.");
		// empty partition is not an error
		goto exit;
	}
	ret = sb_read_fifo(snt8100fsr_g, REGISTER_FIFO_CONFIG, len, snt8100fsr_g->reg_part_buf);

	if (ret) {
		PRINT_CRIT("Unable to read Config FIFO length: %d", ret);
		goto errexit;
	}
	// Process output
	count += snprintf(buf+count, PAGE_SIZE, "\n");
	reg_id = snt8100fsr_g->reg_part_buf[i]&0x00ff;// LSB
	num_reg = (snt8100fsr_g->reg_part_buf[i]>>8) & 0x00ff;// MSB
	PRINT_DEBUG("reg_id=0x%x, num_reg=%d", reg_id, num_reg);
	while (reg_id != 0 && num_reg != 0 && i < len/2) {
		i++;
		if (num_reg == 0) {
			PRINT_CRIT("num_reg in Flash Register Partition is 0 at pos %d", i);
			goto errexit;
		}
		if (num_reg+i >= len/2) {
			PRINT_CRIT("num_reg exceeds register list size: (%d, %d, %d)", num_reg, i, len/2);
			goto errexit;
		}
		if (reg_id == 0x82) reg_id = 0x200;
		count += snprintf(buf+count, PAGE_SIZE, "0x%x %d", reg_id, num_reg);
		for (j=0; j < num_reg; j++) {
			if (j % 8 == 0) count += snprintf(buf+count, PAGE_SIZE, "\n");
			count += snprintf(buf+count, PAGE_SIZE, " 0x%04x", snt8100fsr_g->reg_part_buf[i+j]);
		}
		i += j;
		count += snprintf(buf+count, PAGE_SIZE, "\n");
		reg_id = snt8100fsr_g->reg_part_buf[i]&0x00ff;// LSB
		num_reg = (snt8100fsr_g->reg_part_buf[i]>>8) & 0x00ff;  // MSB
		PRINT_DEBUG("reg_id=0x%x, num_reg=%d", reg_id, num_reg);
   }
exit:
	if (snt8100fsr_g->reg_part_buf != NULL) {
		memory_free(snt8100fsr_g->reg_part_buf);
		snt8100fsr_g->reg_part_buf = NULL;
	}
	mutex_unlock(&snt8100fsr_g->sb_lock);
	PRINT_DEBUG("done %lu", count);
	return count;

errexit:
	count = -1;
	goto exit;
}
#endif
// SUPPORT_FLASH

ssize_t sysfs_reg_script_file_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count) {
                                           
#ifdef DYNAMIC_PWR_CTL
		if (snt_activity_request() != 0) {
			PRINT_CRIT("snt_activity_request() failed");
			return -1;
		}
#endif


    return do_write_reg_script( dev, 
                                attr, 
                                buf, 
                                count, 
                                mc_reg_script);
}

/* Calibration write data +++ */
ssize_t sysfs_boot_init_reg_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count) {
	PRINT_FUNC("%zu bytes", count);
	PRINT_INFO("Enter!!!!");
	Wait_Wake_For_RegW();
	enable_boot_init_reg_req(snt8100fsr_g, buf, count);
	Into_DeepSleep_fun();

	PRINT_DEBUG("done.");
	return count;
}

extern int grip_write_fail_count;
ssize_t sysfs_chip_reset_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count) {
	PRINT_FUNC("%zu bytes", count);
	if(snt8100fsr_g->chip_reset_flag == GRIP_RST_FW_DL){
		PRINT_INFO("Chip reset is ongoing, skip request");
	}else {
		snt8100fsr_g->snt_state = GRIP_RESET;
		Grip_Driver_IRQ_EN(1);
		PRINT_INFO("call chip reset");
		grip_write_fail_count = 0;
		Power_Control(1);
	}
	return count;
}
/* Calibration write data --- */

/*
 * SysFS interface for --SetSysParam
 */
ssize_t sysfs_set_sys_param_show(struct device *dev,
                            struct device_attribute *attr,
                            char *buf)
{
	int ret;
	PRINT_FUNC("status %d", snt8100fsr_g->set_sys_param_status);
	ret = sysfs_uint32_show(dev, attr, &snt8100fsr_g->set_sys_param_status, buf);
	PRINT_DEBUG("done %d", ret);
	return ret;
}

/*
 *
 */
ssize_t sysfs_set_sys_param_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {

	//size_t l = count;
	int l = (int)count;
	uint32_t val;
	uint32_t id;
	int status;
	int ret;

	PRINT_INFO("%zu bytes, buf=%s", count, buf);
	
#ifdef DYNAMIC_PWR_CTL
			if (snt_activity_request() != 0) {
				PRINT_CRIT("snt_activity_request() failed");
				return -1;
			}
#endif


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
		ret = down_interruptible(&snt8100fsr_g->sc_wf_rsp);
		PRINT_DEBUG("SetSysParam Rsp -- acquired %d", ret);
	} while (ret == -EINTR);

errexit:
	PRINT_DEBUG("done.");
	return count;
}

/*
 * SysFS Interface for profile
 */
ssize_t sysfs_profile_show(struct device *dev,
                            struct device_attribute *attr,
                            char *buf) {
	int ret = 0;

	PRINT_FUNC();
	mutex_lock(&snt8100fsr_g->sb_lock);

	ret += snprintf(buf+ret, PAGE_SIZE, "%d\n", snt8100fsr_g->op_profile);

	mutex_unlock(&snt8100fsr_g->sb_lock);
	PRINT_DEBUG("done.");
	return ret;
}

ssize_t sysfs_profile_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {
	long profile;

	PRINT_FUNC("%d bytes", count);

	if (count > 10) {
		return -1;
	}

#ifdef DYNAMIC_PWR_CTL
	if (snt_activity_request() != 0) {
		PRINT_CRIT("snt_activity_request() failed");
		return -1;
	}
#endif

	if(kstrtol(buf, 10, &profile) == 0) {
		snt8100fsr_g->op_profile = (int)profile;
		set_operational_profile(snt8100fsr_g, snt8100fsr_g->op_profile);
	} else {
		PRINT_CRIT("bad parameter");
		return -1;
	}

	PRINT_DEBUG("done.");
	return count;
}

/*
 * SysFS interface for --GetSysParam
 */
ssize_t sysfs_get_sys_param_show(struct device *dev,
                            struct device_attribute *attr,
                            char *buf)
{
	int ret=0;
	int bytes=0;

	PRINT_FUNC("get_sys_param");

	MUTEX_LOCK(&snt8100fsr_g->sb_lock);

	// deal with gets greater than 4 bytes
	if (snt8100fsr_g->get_sys_param_cmd && snt8100fsr_g->get_sys_param_cmd->length > 8) {
		int len = snt8100fsr_g->get_sys_param_cmd->length-4;
		int i;
		uint8_t *p = (uint8_t*) &snt8100fsr_g->get_sys_param_cmd->data[1];
		PRINT_DEBUG("id = %d, status=%d, len=%d", snt8100fsr_g->get_sys_param_id,
						snt8100fsr_g->get_sys_param_status, len);
		ret = snprintf(buf, PAGE_SIZE, "%d", snt8100fsr_g->get_sys_param_status);
		for (i=0; i < len; i++) {
			bytes = snprintf(buf+ret, PAGE_SIZE, " %d", p[i]);
			ret += bytes;
		}
		snprintf(buf+ret, PAGE_SIZE, "\n");

		memory_free(snt8100fsr_g->get_sys_param_cmd);
		snt8100fsr_g->get_sys_param_cmd = NULL;

	} else {
		PRINT_DEBUG("id = %d, status=%d, val=%d", snt8100fsr_g->get_sys_param_id,snt8100fsr_g->get_sys_param_status,
						snt8100fsr_g->get_sys_param_val);

		ret= snprintf(buf, PAGE_SIZE, "%d %u\n", snt8100fsr_g->get_sys_param_status, snt8100fsr_g->get_sys_param_val);
	}

	if (snt8100fsr_g->get_sys_param_cmd) {
		memory_free(snt8100fsr_g->get_sys_param_cmd);
		snt8100fsr_g->get_sys_param_cmd = NULL;
	}
	mutex_unlock(&snt8100fsr_g->sb_lock);
	PRINT_DEBUG("done %d", ret);
	return ret;
}

/*
 *
 */
ssize_t sysfs_get_sys_param_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {
	//size_t l = count;
	int l = (int)count;
	uint32_t val;
	int status;
    int ret;

    PRINT_FUNC("%zu bytes", count);
	
#ifdef DYNAMIC_PWR_CTL
			if (snt_activity_request() != 0) {
				PRINT_CRIT("snt_activity_request() failed");
				return -1;
			}
#endif


	status  = ReadNumFromBuf((const uint8_t**)&buf, &l, &val);
	if (status != 0) {
		PRINT_CRIT("Could not parse param_id %d", status);
		goto errexit;
	}
	enable_get_sys_param(snt8100fsr_g, val);

	// wait for response from driver irpt thread
	PRINT_DEBUG("GetSysParam Rsp -- wait");
	do {
		ret = down_interruptible(&snt8100fsr_g->sc_wf_rsp);
		PRINT_DEBUG("GetSysParam Rsp -- acquired %d", ret);
	} while (ret == -EINTR);


errexit:
	PRINT_DEBUG("done.");
	return count;
}

/*
 * SysFS interface for read/write of registers
 */
ssize_t sysfs_get_reg_show(struct device *dev,
                            struct device_attribute *attr,
                            char *buf)
{
	int ret=0;
	int bytes=0;

	PRINT_FUNC("");
	MUTEX_LOCK(&snt8100fsr_g->sb_lock);

	if (NULL != snt8100fsr_g->get_reg_buf) {
		int i;
		PRINT_INFO("id = %d, len=%d", snt8100fsr_g->get_reg_id, snt8100fsr_g->get_reg_num);
		for (i=0; i < snt8100fsr_g->get_reg_num; i++) {
			PRINT_INFO("reg_id=%d, val=0x%x, ret=%d", snt8100fsr_g->get_reg_id+i, snt8100fsr_g->get_reg_buf[i], ret );
			bytes = snprintf(buf+ret, PAGE_SIZE, " 0x%04x", snt8100fsr_g->get_reg_buf[i]);
			ret += bytes;
		}

		ret += snprintf(buf+ret, PAGE_SIZE, "\n");;

		if (NULL != snt8100fsr_g->get_reg_buf) memory_free(snt8100fsr_g->get_reg_buf);
		snt8100fsr_g->get_reg_buf = NULL;

	} else {
		PRINT_INFO("Empty get_reg_buf");
	}

	mutex_unlock(&snt8100fsr_g->sb_lock);
	PRINT_DEBUG("done %d", ret);
	return ret;
}


ssize_t sysfs_get_reg_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {
	int len;
	int ret;   
	//size_t l = count;
	int l = (int)count;
	uint32_t val;
	int status;

	PRINT_FUNC("%zu bytes", count);
	
#ifdef DYNAMIC_PWR_CTL
			if (snt_activity_request() != 0) {
				PRINT_CRIT("snt_activity_request() failed");
				return -1;
			}
#endif

	MUTEX_LOCK(&snt8100fsr_g->sb_lock);

	status  = ReadNumFromBuf((const uint8_t**)&buf, &l, &val);
	if (status != 0) {
		PRINT_CRIT("Could not parse reg_id %d", status);
		goto errexit;
	}
	snt8100fsr_g->get_reg_id  = val;

	status = ReadNumFromBuf((const uint8_t**)&buf, &l, &val);
	if (status != 0) {
		PRINT_CRIT("Could not parse num regs %d", status);
		goto errexit;
	}
	snt8100fsr_g->get_reg_num = val;

	len = snt8100fsr_g->get_reg_num * 2;// cvt from num_reg to byte len
	PRINT_DEBUG("reg_id=%d, num_reg=%d", snt8100fsr_g->get_reg_id, snt8100fsr_g->get_reg_num);

	if (snt8100fsr_g->get_reg_buf != NULL) {
		memory_free(snt8100fsr_g->get_reg_buf);
	}
	snt8100fsr_g->get_reg_buf = (uint16_t*)memory_allocate(len, GFP_DMA);
	if (snt8100fsr_g->get_reg_buf == NULL) {
		PRINT_CRIT("memory_allocate(%d) failed", len); 
		mutex_unlock(&snt8100fsr_g->sb_lock);
		count = -1;
		goto errexit;	
	}

	ret = sb_read_fifo(snt8100fsr_g, snt8100fsr_g->get_reg_id, len, snt8100fsr_g->get_reg_buf);

	if (ret) {
		PRINT_CRIT("Unable to read from registers: %d", ret);
		if (snt8100fsr_g->get_reg_buf != NULL) memory_free(snt8100fsr_g->get_reg_buf);
		snt8100fsr_g->get_reg_buf = NULL;
		snt8100fsr_g->get_reg_num = 0;
		count = -1;
	}

errexit:
	mutex_unlock(&snt8100fsr_g->sb_lock);
	PRINT_DEBUG("done.");
	return count;
}
extern struct   sc_command *sc_cmd;
ssize_t sysfs_sc_reset_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {
	PRINT_FUNC("%zu bytes", count);
	PRINT_INFO("sc_cmd type: %d", snt8100fsr_g->active_sc_cmd);
	if (count != 2) {
		PRINT_CRIT("Only single digit number '1' or '0' allowed");
		return -1;
	}
	
#ifdef DYNAMIC_PWR_CTL
			if (snt_activity_request() != 0) {
				PRINT_CRIT("snt_activity_request() failed");
				return -1;
			}
#endif


	if (*buf == '1') {
		sc_cleanup(snt8100fsr_g);
	if(sc_cmd !=NULL){
		MUTEX_LOCK(&snt8100fsr_g->sb_lock);
		memory_free(sc_cmd);
		mutex_unlock(&snt8100fsr_g->sb_lock);
		sc_cmd = NULL;
	}
	} else {
		PRINT_DEBUG("only '1' allowed");
	}

	PRINT_INFO("sc_cmd type: %d", snt8100fsr_g->active_sc_cmd);
	PRINT_DEBUG("done.");
	return count;
}

ssize_t sysfs_set_reg_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {
	//size_t l = count;
	int l = (int)count;
	uint32_t val;
	int reg_id;
	uint16_t * reg_buf = NULL;
	int len = 0;
	int ret;
	int status;
	PRINT_INFO("buf = %s, count = %d", buf, count);
	PRINT_FUNC("%zu bytes", count);

	if(snt8100fsr_g->chip_reset_flag == GRIP_RST_FW_DL){
		PRINT_INFO("skip register set request during chip reset");
		return -1;
	}
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	
#ifdef DYNAMIC_PWR_CTL
	if (snt_activity_request() != 0) {
		PRINT_CRIT("snt_activity_request() failed");
		mutex_unlock(&snt8100fsr_g->ap_lock);
		return -1;
	}
#endif


	status = ReadNumFromBuf((const uint8_t**)&buf, &l, &val);
	if (status != 0) {
		PRINT_DEBUG("Could not parse reg_id %d", (int)status);
		goto errexit;
	}
	reg_id = val;
	PRINT_DEBUG("reg_id=%d", reg_id);

	reg_buf = (uint16_t*)memory_allocate(count, GFP_DMA);
	if (reg_buf == NULL) {
		PRINT_CRIT("memory_allocate(%zu) failed",count);
		mutex_unlock(&snt8100fsr_g->ap_lock);
		return -1;
	}

	status = ReadNumFromBuf((const uint8_t**)&buf, &l, &val);
	while (status == 0) {
		if (0x80000000 & val) val &= 0x0000ffff;// to 16 bit neg num
		if ( val > 0x0000ffff) {
			PRINT_DEBUG("invalid register value %x", val);
			goto errexit;
		}
		reg_buf[len++] = val;
		status = ReadNumFromBuf((const uint8_t**)&buf, &l, &val);
	}
	len *= 2; // cvt to from num reg to num bytes

	MUTEX_LOCK(&snt8100fsr_g->sb_lock);
	ret = sb_write_fifo(snt8100fsr_g, reg_id, len, reg_buf);
	mutex_unlock(&snt8100fsr_g->sb_lock);
	if (ret) {
		PRINT_CRIT("Unable to write to registers: %d", ret);
		//when count = 0, return count will make recursive problem
		//count = 0;
		goto errexit;
	}
	else { //cache register_frame_rate if it was set
		if (reg_id < 3  && (reg_id + count) > 2) {
			snt8100fsr_g->frame_rate = reg_buf[2-reg_id];
		}
	}

errexit:
	if (reg_buf) {
		memory_free(reg_buf);
	}
	mutex_unlock(&snt8100fsr_g->ap_lock);
	PRINT_INFO("done. %d", count);
	return count;
}

/*
 * SysFS interface to enable reporting to sensor app
 */
 
ssize_t sysfs_enable_sensor_evt_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf) {
    return snprintf(buf, PAGE_SIZE, "%d\n", snt8100fsr_g->en_sensor_evt);
}

ssize_t sysfs_enable_sensor_evt_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t count) {
	long value;
	PRINT_FUNC("%zu bytes", count);

	if (count > 10) {
		return -1;
	}

	if(kstrtol(buf, 10, &value) == 0) {
		snt8100fsr_g->en_sensor_evt = value;
		PRINT_DEBUG("en_sensor_evt = %d", snt8100fsr_g->en_sensor_evt);
		return count;
	}

	PRINT_NOTICE("Invalid SysFS Value: 0x%X", *buf);
	return -1;
}

/*
 * SysFS interface to enable reporting to sensor app
 */
 
ssize_t sysfs_reset_fw_failed_count_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf) {
    return snprintf(buf, PAGE_SIZE, "%d, should < %d\n", 
    snt8100fsr_g->fw_failed_count, FW_FAIL_REDOWNLOAD_LIMIT);
}

ssize_t sysfs_reset_fw_failed_count_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t count) {
	long value;
	PRINT_FUNC("%zu bytes", count);

	if (count > 10) {
		return -1;
	}

	if(kstrtol(buf, 10, &value) == 0) {
		snt8100fsr_g->fw_failed_count = value;
		PRINT_DEBUG("fw_failed_count = %d", snt8100fsr_g->fw_failed_count);
		return count;
	}

	PRINT_NOTICE("Invalid SysFS Value: 0x%X", *buf);
	return -1;
}


/*==========================================================================*/
/* SysFS Setup                                                              */
/*==========================================================================*/
// SYSFS_PERM_SHOW_STORE
/*
static DEVICE_ATTR(log_track_report,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_log_track_report_show,
                   sysfs_log_track_report_store);

static DEVICE_ATTR(log_track_report_bin,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_log_track_report_bin_show,
                   sysfs_log_track_report_bin_store);

static DEVICE_ATTR(event_log,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_event_log_show,
                   sysfs_event_log_store);

static DEVICE_ATTR(log_d1test,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_log_d1test_show,
                   sysfs_log_d1test_store);

static DEVICE_ATTR(log_frames,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_log_frames_show,
                   sysfs_log_frames_store);

static DEVICE_ATTR(log_no_touch_frame,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_log_no_touch_frame_show,
                   sysfs_log_no_touch_frame_store);

static DEVICE_ATTR(suspended_frame_rate,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_suspended_frame_rate_show,
                   sysfs_suspended_frame_rate_store);

static DEVICE_ATTR(set_sys_param,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_set_sys_param_show,
                   sysfs_set_sys_param_store);

static DEVICE_ATTR(get_sys_param,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_get_sys_param_show,
                   sysfs_get_sys_param_store);

static DEVICE_ATTR(profile,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_profile_show,
                   sysfs_profile_store);

static DEVICE_ATTR(get_reg,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_get_reg_show,
                   sysfs_get_reg_store);

#ifdef SUPPORT_FLASH
static DEVICE_ATTR(fwupdate,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_fwupdate_show,
                   sysfs_fwupdate_store);
static DEVICE_ATTR(read_flash_reg_part,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_read_flash_reg_part_show,
                   sysfs_read_flash_reg_part_store);
#endif

static DEVICE_ATTR(enable_sensor_evt,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_enable_sensor_evt_show,
                   sysfs_enable_sensor_evt_store);

#ifdef DYNAMIC_PWR_CTL
static DEVICE_ATTR(enable_dynamic_pwr_ctl,
                   SYSFS_PERM_SHOW_STORE,
                   sysfs_enable_dynamic_pwr_ctl_show,
                   sysfs_enable_dynamic_pwr_ctl_store);
#endif

// SYSFS_PERM_STORE
static DEVICE_ATTR(wake_device,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_wake_device_store);

static DEVICE_ATTR(load_registers,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_load_registers_store);

static DEVICE_ATTR(set_reg,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_set_reg_store);

static DEVICE_ATTR(deep_trace,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_deep_trace_store);

static DEVICE_ATTR(sc_reset,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_sc_reset_store);

#ifdef SUPPORT_FLASH
static DEVICE_ATTR(write_flash_reg_part,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_write_flash_reg_part_store);
static DEVICE_ATTR(write_flash_reg_part_file,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_write_flash_reg_part_file_store);
#endif

static DEVICE_ATTR(reg_script_file,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_reg_script_file_store);
 
// SYSFS_PERM_SHOW
static DEVICE_ATTR(track_report,
                   SYSFS_PERM_SHOW,
                   sysfs_track_report_show,
                   NULL);

static DEVICE_ATTR(product_config,
                   SYSFS_PERM_SHOW,
                   sysfs_product_config_show,
                   NULL);

static DEVICE_ATTR(list_registers,
                   SYSFS_PERM_SHOW,
                   sysfs_list_registers_show,
                   NULL);

static DEVICE_ATTR(version,
                   SYSFS_PERM_SHOW,
                   sysfs_version_show,
                   NULL);

// Calibration write data +++
static DEVICE_ATTR(boot_init_reg,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_boot_init_reg_store);
// force_transfer_en
static DEVICE_ATTR(force_transfer_en,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_force_transfer_en_store);

static DEVICE_ATTR(chip_reset,
                   SYSFS_PERM_STORE,
                   NULL,
                   sysfs_chip_reset_store);
*/
// Calibration write data --
/*
static struct attribute *sysfs_attrs_static[] = {
    &dev_attr_track_report.attr,
    &dev_attr_log_track_report.attr,
    &dev_attr_log_track_report_bin.attr,
    &dev_attr_event_log.attr,
    &dev_attr_log_no_touch_frame.attr,
    &dev_attr_log_d1test.attr,
    &dev_attr_log_frames.attr,
    &dev_attr_suspended_frame_rate.attr,
    &dev_attr_product_config.attr,
    &dev_attr_wake_device.attr,
    &dev_attr_load_registers.attr,
    &dev_attr_list_registers.attr,
    &dev_attr_version.attr,
    &dev_attr_profile.attr,
    &dev_attr_set_sys_param.attr,
    &dev_attr_get_sys_param.attr,
    &dev_attr_get_reg.attr,
    &dev_attr_set_reg.attr,
    &dev_attr_sc_reset.attr,
    &dev_attr_deep_trace.attr,
    &dev_attr_reg_script_file.attr,
#ifdef SUPPORT_FLASH
    &dev_attr_fwupdate.attr,
    &dev_attr_write_flash_reg_part.attr,
    &dev_attr_write_flash_reg_part_file.attr,
    &dev_attr_read_flash_reg_part.attr,
#endif
// Calibration write data +++ 
    &dev_attr_boot_init_reg.attr,
    &dev_attr_chip_reset.attr,
// Calibration write data --- 
    &dev_attr_enable_sensor_evt.attr,
#ifdef DYNAMIC_PWR_CTL
    &dev_attr_enable_dynamic_pwr_ctl.attr,
#endif
    NULL
};
*/

//=============================================================
//=============================================================
//=============================================================
//static struct device_attribute *sysfs_attrs_g;
static struct device_attribute grip_property_attrs[] = {
	/*read only*/
	__ATTR(chip_reset, 0220, NULL, sysfs_chip_reset_store),
	__ATTR(enable_sensor_evt, 0664, sysfs_enable_sensor_evt_show, sysfs_enable_sensor_evt_store),
	__ATTR(log_track_report, 0664, sysfs_log_track_report_show, sysfs_log_track_report_store),
	__ATTR(log_track_report_bin, 0664, sysfs_log_track_report_show, sysfs_log_track_report_bin_store),
	__ATTR(event_log, 0664, sysfs_event_log_show, sysfs_event_log_store),
	__ATTR(log_d1test, 0664, sysfs_log_d1test_show, sysfs_log_d1test_store),
	__ATTR(log_frames, 0664, sysfs_log_frames_show, sysfs_log_frames_store),
	__ATTR(log_no_touch_frame, 0664, sysfs_log_no_touch_frame_show, sysfs_log_no_touch_frame_store),
	__ATTR(suspended_frame_rate, 0664, sysfs_suspended_frame_rate_show, sysfs_suspended_frame_rate_store),
	__ATTR(set_sys_param, 0664, sysfs_set_sys_param_show, sysfs_set_sys_param_store),
	__ATTR(get_sys_param, 0664, sysfs_get_sys_param_show, sysfs_get_sys_param_store),
	__ATTR(profile, 0664, sysfs_profile_show, sysfs_profile_store),
	__ATTR(get_reg, 0664, sysfs_get_reg_show, sysfs_get_reg_store),
	__ATTR(wake_device, 0220, NULL, sysfs_wake_device_store),
	__ATTR(load_registers, 0220, NULL, sysfs_load_registers_store),
	__ATTR(set_reg, 0220, NULL, sysfs_set_reg_store),
	__ATTR(deep_trace, 0220, NULL, sysfs_deep_trace_store),
	__ATTR(sc_reset, 0220, NULL, sysfs_sc_reset_store),
	__ATTR(track_report, 0444, sysfs_track_report_show, NULL),
	__ATTR(product_config, 0444, sysfs_product_config_show, NULL),
	__ATTR(list_registers, 0444, sysfs_list_registers_show, NULL),
	__ATTR(version, 0444, sysfs_version_show, NULL),
	__ATTR(boot_init_reg, 0220, NULL, sysfs_boot_init_reg_store),
	__ATTR(register_event, 0664, sysfs_register_show, sysfs_register_store),
	__ATTR(register_enable, 0664, sysfs_register_show, sysfs_register_store),
	__ATTR(register_frame_rate, 0664, sysfs_register_show, sysfs_register_store),
	__ATTR(register_phy_stat_lsb, 0664, sysfs_register_show, sysfs_register_store),
	__ATTR(register_phy_stat_msb, 0664, sysfs_register_show, sysfs_register_store),
	__ATTR(register_actions, 0664, sysfs_register_show, sysfs_register_store),
	__ATTR(register_irpt_cfg, 0664, sysfs_register_show, sysfs_register_store),
	__ATTR(register_i2c_cfg, 0664, sysfs_register_show, sysfs_register_store),
	__ATTR(register_loopback, 0664, sysfs_register_show, sysfs_register_store),
	__ATTR(register_chipid_lsb, 0664, sysfs_register_show, sysfs_register_store),
	__ATTR(register_chipid_msb, 0664, sysfs_register_show, sysfs_register_store),
	__ATTR(register_bar0_fsf, 0664, sysfs_register_show, sysfs_register_store),
	__ATTR(register_bar1_fsf, 0664, sysfs_register_show, sysfs_register_store),
	__ATTR(register_bar2_fsf, 0664, sysfs_register_show, sysfs_register_store),
	__ATTR(register_bar3_fsf, 0664, sysfs_register_show, sysfs_register_store),
	__ATTR(register_bypass, 0664, sysfs_register_show, sysfs_register_store),
	__ATTR(register_bar_ctrl, 0664, sysfs_register_show, sysfs_register_store),
	__ATTR(reset_fw_fail_count, 0664, sysfs_reset_fw_failed_count_show, sysfs_reset_fw_failed_count_store),
#ifdef SUPPORT_FLASH
	__ATTR(fwupdate, 0664, sysfs_fwupdate_show, sysfs_fwupdate_store),
	__ATTR(read_flash_reg_part, 0664, sysfs_read_flash_reg_part_show, sysfs_read_flash_reg_part_store),
#endif
#ifdef DYNAMIC_PWR_CTL
	__ATTR(enable_dynamic_pwr_ctl, 0664, sysfs_enable_dynamic_pwr_ctl_show, sysfs_enable_dynamic_pwr_ctl_store),
#endif
#ifdef SUPPORT_FLASH
	__ATTR(write_flash_reg_part, 0220, NULL, sysfs_write_flash_reg_part_store),
	__ATTR(write_flash_reg_part_file, 0220, NULL, sysfs_write_flash_reg_part_file_store),
#endif
};

static int g_devMajor = -1;
static struct class *g_property_class = NULL;
//static struct class *g_snt_property_class = NULL;
static struct device *g_gripsensor_dev;

static int Grip_class_open(struct inode * inode, struct file * file)
{
    return 0;
}

static const struct file_operations Grip_class_fops = {
    .owner      = THIS_MODULE,
    .open       = Grip_class_open,
};

static int create_Grip_chrdev(void)
{
	/*create character device*/
	g_devMajor = register_chrdev(0, "grip_sensor", &Grip_class_fops);
	if (g_devMajor < 0) {
		PRINT_ERR("%s: could not get major number\n", __FUNCTION__);
		return g_devMajor;
	}
	
	return 0;
}

static int create_Grip_class(void)
{
	/* create sys/class file node*/
	g_property_class = class_create(THIS_MODULE, "grip_sensor");
	if (IS_ERR(g_property_class)) {
		PRINT_ERR("%s: class_create ERROR.\n", __FUNCTION__);
		return PTR_ERR(g_property_class);
	}
	
	return 0;
}

static struct device *Grip_ATTR_device_create(int type)
{
	int ret = 0;
	dev_t dev;
	struct device *sensor_dev=NULL;
	
	/*prepare character device and return for error handling*/
	if(g_devMajor < 0){
		ret=create_Grip_chrdev();
		if(ret < 0)
			return NULL;
	}

	/*prepare sys/class file node and return for error handling*/
	if(g_property_class == NULL || IS_ERR(g_property_class) ){
		ret=create_Grip_class();
		if(ret < 0)
			return NULL;
	}

	dev = MKDEV(g_devMajor, type);
	sensor_dev = device_create(g_property_class, NULL, dev, NULL, "%s", "snt8100fsr");
	
	if (IS_ERR(sensor_dev)) {
		ret = PTR_ERR(sensor_dev);
		PRINT_ERR("%s: sensor_dev pointer is ERROR(%d). \n", __FUNCTION__, ret);		
		return NULL;
	}	
	
	return sensor_dev;
}

static int GripSensor_ATTR_register(void){
	int ret = 0;
	int ATTR_index;
	
	/* psensor device */
	g_gripsensor_dev = Grip_ATTR_device_create(10);
	if (IS_ERR(g_gripsensor_dev) || g_gripsensor_dev == NULL) {
		ret = PTR_ERR(g_gripsensor_dev);
		PRINT_ERR("%s: psensor create ERROR.\n", __FUNCTION__);
		return ret;
	}	
	for (ATTR_index=0; ATTR_index < ARRAY_SIZE(grip_property_attrs); ATTR_index++) {
		ret = device_create_file(g_gripsensor_dev, &grip_property_attrs[ATTR_index]);
		if (ret)
			return ret;
	}
	/*
	for (ATTR_index=0; ATTR_index < get_attrs_array_size(&sysfs_attrs_g); ATTR_index++) {
		ret = device_create_file(g_gripsensor_dev, &sysfs_attrs_g[ATTR_index]);
		if (ret)
			return ret;
	}
	*/
	return 0;
}

//static struct attribute_group attr_group_g;
int get_register_dict_size(void) {
    int i = 0;
    char *name = register_dict_g[i].str;
    while (name) {
        name = register_dict_g[++i].str;
    }
    return i;
}

int snt_sysfs_init(struct snt8100fsr *snt8100fsr, bool enable) {
    PRINT_FUNC("0x%p, %s for \"%s\"", snt8100fsr, enable ? "true" : "false", SYSFS_NAME);
/*
    if (!enable) {
        sysfs_remove_link(sysfs_kobj_g, SYSFS_NAME);
        sysfs_remove_group(sysfs_kobj_g, &attr_group_g);
        kobject_del(sysfs_kobj_g);
        
        free_sysfs_attrs(sysfs_attrs_g);

        if (snt8100fsr_g && snt8100fsr_g->get_sys_param_cmd) {
            memory_free(snt8100fsr_g->get_sys_param_cmd);
            snt8100fsr_g->get_sys_param_cmd = NULL;
        }
        if (snt8100fsr_g && snt8100fsr_g->get_reg_buf) {
            memory_free(snt8100fsr_g->get_reg_buf);
            snt8100fsr_g->get_reg_buf = NULL;
        }
        snt8100fsr_g = NULL;
        dev_info(snt8100fsr->dev, "%s: sysfs attributes removed\n", __func__);
    } else {

        sysfs_attrs_g = alloc_sysfs_attrs();
	snt8100fsr_g->class = class_create(THIS_MODULE, "snt8100fsr");
	if (IS_ERR(snt8100fsr_g->class)) {
		PRINT_ERR("%s: class_create ERROR.\n", __FUNCTION__);
		return PTR_ERR(snt8100fsr_g->class);
	}
	snt8100fsr_g->dev->class = snt8100fsr_g->class;
	dev_set_name(snt8100fsr_g->dev, "snt8100fsr111");
	if(device_register(snt8100fsr_g->dev)){
		PRINT_ERR("driver_register failed\n");
	}
        // Save our attributes to the attribute group
        attr_group_g.attrs = sysfs_attrs_g;
        sysfs_kobj_g = kobject_create_and_add(SYSFS_NAME, NULL);
        if (sysfs_kobj_g == NULL) {
            PRINT_ERR("%s: subsystem_register failed\n", __func__);
            return -ENOMEM;
        }
        if (snt8100fsr->spi) {
            if (sysfs_create_link(sysfs_kobj_g, &snt8100fsr->spi->dev.kobj, "spi") < 0) {
                PRINT_ERR("%s: failed to create link\n", __func__);
                return -ENOMEM;
            }
        }
        if (snt8100fsr->i2c) {
            if (sysfs_create_link(sysfs_kobj_g, &snt8100fsr->i2c->dev.kobj, "i2c") < 0) {
                PRINT_ERR("%s: failed to create link\n", __func__);
                return -ENOMEM;
            }
        }

        if (sysfs_create_group(sysfs_kobj_g, &attr_group_g) < 0) {
            PRINT_ERR("%s: Failed to create sysfs attributes\n", __func__);
            return -ENODEV;
        }
    }
    */
    GripSensor_ATTR_register();
    return 0;
}
/*
void free_sysfs_attrs(struct attribute **attrs) {

     // Free memory for our dynamically allocated register sysFS entries,
     // starting from after the static portion of the list which we don't
     // perform memory_free operations on.
     
     
    int i = get_attrs_array_size(sysfs_attrs_static);
    struct attribute *attr = attrs[i];
    while (attr) {
        PRINT_DEBUG("sysFS[%d]: 0x%p", i, attr);
        memory_free((void *)attr->name);
        memory_free(attr);
        attr = attrs[++i];
    }
    memory_free(attrs);
}
*/
/*
struct device_attribute *alloc_sysfs_attrs(void) {
    int i;
    //struct device_attribute *da;
    int reg_prefix_size;
    char *reg_name;
    char *full_name;
    int register_entries;
    int sysfs_entries;
    int total_entries;
    int total_size;
    int pos;
    struct device_attribute *attrs;

    register_entries = get_register_dict_size();

    
     // Calculate how many sysFS entries we need room for, plus trailing
     // null inside sysfs_attrs_list.
     
    sysfs_entries = get_attrs_array_size((struct device_attribute **)&grip_property_attrs);
    total_entries = sysfs_entries + register_entries;

    // Calcualte the size of these entries in bytes, plus room for null;
    total_size = (total_entries + 1) * sizeof(struct attribute *);

    PRINT_DEBUG("%d hardware registers found", register_entries);
    PRINT_DEBUG("%d static sysFS entries found", sysfs_entries);
    PRINT_DEBUG("Allocating %d bytes for %d sysFS entries", total_size, total_entries);

    attrs = memory_allocate(total_size, 0);
    memset(attrs, 0, total_size);
    // Firstly, copy the static sysfs attributes
    memcpy(attrs, grip_property_attrs, sizeof(grip_property_attrs));
    pos = sysfs_entries;

    // Secondly, copy the hardware registers as attributes
    i = 0;
    reg_name = register_dict_g[i].str;
    reg_prefix_size = strlen(REGISTER_PREFIX);
    while (reg_name) {
        int size = reg_prefix_size + strlen(reg_name) + 1;
        full_name = memory_allocate(size, 0);
        snprintf(full_name, size, "%s%s",
                 REGISTER_PREFIX, reg_name);

        PRINT_DEBUG("Creating sysFS %d entry for: %s", pos, full_name);
        da = memory_allocate(sizeof(struct device_attribute), 0);
        da->attr.name = full_name;
        da->attr.mode = SYSFS_PERM_SHOW_STORE;
        da->show = sysfs_register_show;
        da->store = sysfs_register_store;
        &attrs[pos++] = da;
        
        //attrs[pos] = memory_allocate(sizeof(struct device_attribute), 0);
        attrs[pos].attr.name = full_name;
        attrs[pos].attr.mode = SYSFS_PERM_SHOW_STORE;
        attrs[pos].show = sysfs_register_show;
        attrs[pos].store = sysfs_register_store;
        pos++;

        reg_name = register_dict_g[++i].str;
    }

    return attrs;
}
int get_attrs_array_size(struct device_attribute *list[]) {
    int i = 0;
    struct device_attribute *attr = list[i];
    while (attr) {
        attr = list[++i];
    }
    return i;
}

*/


/*==========================================================================*/
/* DEEP TRACE LOGGING                                                       */
/*==========================================================================*/
int dump_deep_trace_to_file(void) {
    int             ret;
    int             err_ret = 0;
    struct file*    deep_trace_file = NULL;
    int             deep_trace_file_offset = 0;
    uint16_t    *   deep_trace_buf = NULL;
    int             len;

    PRINT_FUNC("Enter.");

    MUTEX_LOCK(&snt8100fsr_g->sb_lock);

    PRINT_DEBUG("Creating deep trace file: %s",  DEEP_TRACE_LOCATION);
    ret = file_open(DEEP_TRACE_LOCATION,
                    O_WRONLY|O_CREAT|O_TRUNC, 0777,
                    &deep_trace_file);
    if(ret) {
        PRINT_DEBUG("Unable to create file '%s', error %d",
                    DEEP_TRACE_LOCATION, ret);
        goto errexit;
    }

    // allocate buffer to get Deep Trace table
    deep_trace_buf = (uint16_t*)memory_allocate(DEEP_TRACE_BUF_LEN, GFP_DMA);
    if (deep_trace_buf == NULL) {
        PRINT_CRIT("memory_allocate(%d) failed", DEEP_TRACE_BUF_LEN);
        goto errexit;
    }

    /* set up CfgBank regs for read partition 7 */
    deep_trace_buf[0] = 0;      // offset = 0
    deep_trace_buf[1] = 0;      // length = 0
    deep_trace_buf[2] = 0x0701; // READ partition 7
    ret = sb_write_fifo(snt8100fsr_g,
                    REGISTER_BNK_CFG_OFFSET,
                    3*sizeof(uint16_t),
                    deep_trace_buf);
    if (ret) {
        PRINT_CRIT("Unable to write to cfgbank registers: %d", ret);
        goto errexit;
    }

    // Read and write FIFO Length
    ret = sb_read_fifo(snt8100fsr_g,
                    REGISTER_FIFO_CONFIG,
                    sizeof(uint16_t),
                    deep_trace_buf);

    if (ret) {
        PRINT_CRIT("Unable to read Config FIFO length: %d", ret);
        goto errexit;
    }
    len = deep_trace_buf[0];
    PRINT_DEBUG("Deep Trace Length = %d", len);
    if (len&1) {
        PRINT_CRIT("Deep Trace length must be even number: %d", len);
        goto errexit;
    }
    if (len == 0) {
        PRINT_DEBUG("Deep Trace length == 0.");
        // empty partition is not an error
        goto exit;
    }
    file_write(deep_trace_file,
                deep_trace_file_offset,
                (uint8_t*)deep_trace_buf,
                sizeof(uint16_t));
    deep_trace_file_offset += sizeof(uint16_t);

    // Read and write Deep Trace Table
    while (len > 0) {
        int block_len = min(len, DEEP_TRACE_BUF_LEN);
        ret = sb_read_fifo(snt8100fsr_g,
                        REGISTER_FIFO_CONFIG,
                        block_len,
                        deep_trace_buf);

        if (ret) {
            PRINT_CRIT("Unable to read deep trace buffer: %d", ret);
            goto errexit;
        }
        len -= block_len;
        file_write(deep_trace_file,
                    deep_trace_file_offset,
                    (uint8_t*)deep_trace_buf,
                    block_len);
        deep_trace_file_offset += block_len;
    }

exit:
    if (deep_trace_file != NULL) {
        file_close(deep_trace_file);
    }
    if (deep_trace_buf != NULL) {
        memory_free(deep_trace_buf);
    }
    mutex_unlock(&snt8100fsr_g->sb_lock);
    PRINT_DEBUG("done");

    return err_ret;

errexit:
    err_ret = -1;
    goto exit;
}
