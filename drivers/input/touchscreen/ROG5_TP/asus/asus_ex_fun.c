#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
#include <linux/soc/qcom/battery_charger_asus.h>
#endif
#include "focaltech_core.h"
#include "asus_tp.h"

/*****************************************************************************
* 1.Static variables
*****************************************************************************/
#define GLOVE                  "driver/glove"
#define FPXY                   "driver/fp_xy"
#define REAL_TIME              "driver/real_time"
#define PERF_TIME              "driver/perf_time"
/*****************************************************************************
* 2.Global variable or extern global variabls/functions
*****************************************************************************/
enum _asus_ex_mode {
    MODE_GLOVE = 0,
    MODE_CHARGER,
};
/*****************************************************************************
* 3.Static function prototypes
*******************************************************************************/
static int asus_ex_mode_switch(enum _asus_ex_mode mode,u8 value)
{
    int ret = 0;
    u8 m_val = 0;

    if (value)
        m_val = 0x01;
    else
        m_val = 0x00;
    
    switch (mode) {
    case MODE_GLOVE:
        ret = fts_write_reg(FTS_REG_GLOVE_MODE_EN, m_val);
        if (ret < 0) {
            FTS_ERROR("MODE_GLOVE switch to %d fail", m_val);
        }
        break;
    case MODE_CHARGER:
        ret = fts_write_reg(FTS_REG_CHARGER_MODE_EN, m_val);
        if (ret < 0) {
            FTS_ERROR("MODE_CHARGER switch to %d fail", m_val);
        }
        break;
    default:
        FTS_ERROR("mode(%d) unsupport", mode);
        ret = -EINVAL;
        break;
    }

    return ret;
}

void asus_tp_charge_mode (bool enable)
{
    struct fts_ts_data *ts_data = fts_data;
    int ret ;
    if (enable) {
	if (ts_data->charger_mode != ENABLE) {
	    FTS_DEBUG("enter charger mode");
	    ret = asus_ex_mode_switch(MODE_CHARGER, ENABLE);
	    if (ret >= 0) {
		ts_data->charger_mode = ENABLE;
	    }
	}
    } else {
        if (ts_data->charger_mode == ENABLE) {
	    FTS_DEBUG("exit charger mode");
	    ret = asus_ex_mode_switch(MODE_CHARGER, DISABLE);
	    if (ret >= 0) {
		ts_data->charger_mode = DISABLE;
	    }
	}      
    }
}

#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
static int charge_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
    
    FTS_INFO("USB event:%lu", event);
    switch (event) {
    case QTI_POWER_SUPPLY_CHARGED:
	FTS_INFO("QTI_POWER_SUPPLY_CHARGED");
	asus_tp_charge_mode(true);
	fts_data->charge_notify_charge = true;
        break;
    case QTI_POWER_SUPPLY_UNCHARGED:
	FTS_INFO("QTI_POWER_SUPPLY_UNCHARGED");
	asus_tp_charge_mode(false);
	fts_data->charge_notify_charge = false;
        break;
    }
    
    return 0;
}
#endif

void fts_ex_fun_recovery(struct fts_ts_data *ts_data) 
{
  int ret;
  
  //check glove MODE_CHARGER
  if (ts_data->glove_mode == ENABLE) {
      FTS_INFO("Glove enabled, restore glove mode");
      ret = asus_ex_mode_switch(MODE_GLOVE,ENABLE);
      if (ret >= 0) {
	  ts_data->glove_mode = ENABLE;
      }
  }
  
  //check charge mode
  if (ts_data->charge_notify_charge) {
      FTS_INFO("Charge mode enabled, receive usb plug message when suspend, restore charge mode");
      ret = asus_ex_mode_switch(MODE_CHARGER, ENABLE);
      if (ret >= 0) {
	  ts_data->charger_mode = ENABLE;
      }
  }
}

/*
 * Factory related
 */
static ssize_t fts_touch_status_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    struct input_dev *input_dev = fts_data->input_dev;

    ssize_t num_read_chars = 0;
    u8 fwver = 0;

    if(fts_data->init_success == 0) {
        num_read_chars = snprintf(buf, PAGE_SIZE,"9\n");
        return num_read_chars;  
    }
    mutex_lock(&input_dev->mutex);

    if (fts_read_reg(FTS_REG_FW_VER, &fwver) < 0)
    {
        num_read_chars = snprintf(buf, PAGE_SIZE,"0\n");
    }
    else
    {
        num_read_chars = snprintf(buf, PAGE_SIZE,"1\n");
    }

    mutex_unlock(&input_dev->mutex);

    return num_read_chars;
}

/*
 * FingerPrinter related
 */
static ssize_t fts_fp_mode_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
   
    count = snprintf(buf + count, PAGE_SIZE, "Notify FP:%s\n",
                     fts_data->fp_enable ? "On" : "Off");

    return count;
}

static ssize_t fts_fp_mode_store(
    struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    // listen persist.vendor.asus.fp.wakeup
    if (FTS_SYSFS_ECHO_ON(buf)) {
        if (!fts_data->fp_enable) {
            FTS_DEBUG("Notify FP enable");
            fts_data->fp_enable = ENABLE;
        }
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        if (fts_data->fp_enable) {
            FTS_DEBUG("Notify FP disable");
            fts_data->fp_enable = DISABLE;
	    fts_data->next_resume_isaod = false;
        }
    }

    FTS_DEBUG("Notify FP:%d", fts_data->fp_enable);
    return count;
}

static ssize_t fts_aod_mode_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
   
    count = snprintf(buf + count, PAGE_SIZE, "AOD:%s\n",
                     fts_data->aod_enable ? "On" : "Off");

    return count;
}

static ssize_t fts_aod_mode_store(
    struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    // listen persist.vendor.asus.fp.wakeup
    if (FTS_SYSFS_ECHO_ON(buf)) {
        if (!fts_data->aod_enable) {
            FTS_DEBUG("Notify AOD enable");
            fts_data->aod_enable = ENABLE;
        }
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        if (fts_data->aod_enable) {
            FTS_DEBUG("Notify AOD disable");
            fts_data->aod_enable = DISABLE;
	    fts_data->next_resume_isaod = false;
        }
    }

    FTS_DEBUG("Notify AOD:%d", fts_data->aod_enable);
    return count;
}

static ssize_t fts_fp_ctrl_mode_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d \n", fts_data->fp_report_type);
}

static ssize_t fts_fp_ctrl_mode_store(
    struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
  // listen vendor.asus.touch_control_fod
	if (buf[0] == '0') {
		fts_data->fp_report_type = 0;
		FTS_DEBUG("fp_report_type = 0");
		fts_data->next_resume_isaod = false;
		if ((fts_data->display_state == 1) && fts_data->suspended) {
		    FTS_DEBUG("display on , touch not resume , resume touch");
		    resume_touch(true);		    
		}
	} else if (buf[0] == '1') {
		fts_data->fp_report_type = 1;
		FTS_DEBUG("fp_report_type = 1");
	} else if (buf[0] == '2') {
		fts_data->fp_report_type = 2;
		FTS_DEBUG("fp_report_type = 2");
		if (fts_data->next_resume_isaod) {
		    fts_data->next_resume_isaod = false;
		    FTS_DEBUG("FOD speedup, finger not leave panel");
		    if (fts_data->suspended) {
		      FTS_DEBUG("touch not resume , resume touch");
		      resume_touch(true);
		    }
		    if (fts_data->auth_complete == 1) {
			FTS_DEBUG("FP already auth success , reset fp_report_type to 0");
			fts_data->fp_report_type = 0;
			msleep(100);
		        FTS_DEBUG("Notify FP finger up, KEY_U");
			input_report_key(fts_data->input_dev, KEY_U,1);
			input_sync(fts_data->input_dev);
			input_report_key(fts_data->input_dev, KEY_U,0);
			input_sync(fts_data->input_dev);
			fts_release_all_finger();
		    }
		} else { 
		    if (fts_data->display_state == 1) {
			FTS_DEBUG("FOD speedup ,FOD servicee request touch resume");
			fts_data->next_resume_isaod = false;
			if (fts_data->suspended) {
			  FTS_DEBUG("touch not resume , resume touch");
			  resume_touch(true);
			}
		    }
		}
	}

	return count;
}

static ssize_t fp_auth_status_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d \n", fts_data->auth_complete);
}

static ssize_t fp_auth_status_store(
    struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

    fts_data->auth_complete = buf[0]-'0';
    
    FTS_DEBUG("FP:auth_complete %d", fts_data->auth_complete);
    return count;
}

static ssize_t fp_area_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d \n", fts_data->fp_mini);
}

static ssize_t fp_area_store(
    struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    FTS_INFO("buf[0] %d",buf[0]);
	
    fts_data->fp_mini = buf[0]-'0';

    FTS_DEBUG("FP:mini area %d", fts_data->fp_mini);
    return count;
}

/*
 * Gesture related
 */
static ssize_t fts_phonecall_state_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    struct fts_ts_data *ts_data = fts_data;
    struct input_dev *input_dev = ts_data->input_dev;

    mutex_lock(&input_dev->mutex);
    
    count = snprintf(buf + count, PAGE_SIZE, "Phone call :%s\n",
                     ts_data->phone_call_state ? "On" : "Off");
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_phonecall_state_store(
    struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct fts_ts_data *ts_data = fts_data;

    if (FTS_SYSFS_ECHO_ON(buf)) {
        ts_data->phone_call_state = ENABLE;
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        ts_data->phone_call_state = DISABLE;
    }

    FTS_DEBUG("phone state:%d", ts_data->phone_call_state);
    return count;
}

static ssize_t asus_ex_proc_fpxy_read(struct file *file, char __user *buf, size_t count, loff_t *ppos) 
{
	char *buff = NULL;
        int len = 0;
	ssize_t ret = 0;
	struct fts_ts_data *ts_data = fts_data;
	
	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += snprintf(buff + len, PAGE_SIZE, "%d,%d\n",ts_data->fp_x,ts_data->fp_y);
	
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);
	
	return ret;  
}

static ssize_t asus_ex_proc_glove_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int len = 0;
	u8 val = 0;
	ssize_t ret = 0;
	char *buff = NULL;
	struct fts_ts_data *ts_data = fts_data;
	struct input_dev *input_dev = ts_data->input_dev;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	mutex_lock(&input_dev->mutex);
	fts_read_reg(FTS_REG_GLOVE_MODE_EN, &val);
	len += snprintf(buff + len, PAGE_SIZE, "%d\n", val);
	mutex_unlock(&input_dev->mutex);

	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static ssize_t asus_ex_proc_glove_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	struct fts_ts_data *ts_data = fts_data;
	int ret = 0;
	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strncmp(messages, "0", 1) == 0) {
		if (ts_data->glove_mode) {
			FTS_DEBUG("exit glove mode");
			ret = asus_ex_mode_switch(MODE_GLOVE,DISABLE);
			if (ret >= 0) {
				ts_data->glove_mode = DISABLE;
			}
		} else {
			FTS_DEBUG("not in glove mode");
		}
	} else {
		if (!ts_data->glove_mode) {
			FTS_DEBUG("enter glove mode");
			ret = asus_ex_mode_switch(MODE_GLOVE,ENABLE);
			if (ret >= 0) {
				ts_data->glove_mode = ENABLE;
			}
		} else {
			FTS_DEBUG("already in glove mode");
		}
	}
	return len;
}

static ssize_t asus_ex_proc_realtime_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff = NULL;
	struct fts_ts_data *ts_data = fts_data;
	struct input_dev *input_dev = ts_data->input_dev;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	mutex_lock(&input_dev->mutex);
	
	len += snprintf(buff + len, PAGE_SIZE, "%d\n", ts_data->realtime);
	mutex_unlock(&input_dev->mutex);

	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static ssize_t asus_ex_proc_realtime_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	struct fts_ts_data *ts_data = fts_data;
	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strncmp(messages, "0", 1) == 0) {
	  ts_data->realtime = 0;
	} else {
	  ts_data->realtime = 1;
	}
	return len;
}

static ssize_t asus_ex_proc_perftime_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff = NULL;
	struct fts_ts_data *ts_data = fts_data;
	struct input_dev *input_dev = ts_data->input_dev;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	mutex_lock(&input_dev->mutex);
	
	len += snprintf(buff + len, PAGE_SIZE, "%d\n", ts_data->perftime);
	mutex_unlock(&input_dev->mutex);

	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static ssize_t asus_ex_proc_perftime_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	struct fts_ts_data *ts_data = fts_data;
	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strncmp(messages, "0", 1) == 0) {
	  ts_data->perftime = 0;
	} else {
	  ts_data->perftime = 1;
	}
	return len;
}


static struct file_operations asus_ex_proc_glove_ops = {
	.write = asus_ex_proc_glove_write,
	.read  = asus_ex_proc_glove_read,
};

static struct file_operations asus_ex_proc_fpxy_ops = {
	.read  = asus_ex_proc_fpxy_read,
};

static struct file_operations asus_ex_proc_realtime_ops = {
	.write = asus_ex_proc_realtime_write,
	.read  = asus_ex_proc_realtime_read,
};

static struct file_operations asus_ex_proc_perftime_ops = {
	.write = asus_ex_proc_perftime_write,
	.read  = asus_ex_proc_perftime_read,
};

static DEVICE_ATTR(touch_status, S_IRUGO | S_IWUSR, fts_touch_status_show, NULL);
static DEVICE_ATTR(fts_fp_mode, S_IRUGO | S_IWUSR, fts_fp_mode_show, fts_fp_mode_store);
static DEVICE_ATTR(fts_fp_ctrl_mode, S_IRUGO | S_IWUSR, fts_fp_ctrl_mode_show, fts_fp_ctrl_mode_store);
static DEVICE_ATTR(fts_aod_ctrl_mode, S_IRUGO | S_IWUSR, fts_aod_mode_show, fts_aod_mode_store);
static DEVICE_ATTR(fts_phone_state, S_IRUGO | S_IWUSR, fts_phonecall_state_show, fts_phonecall_state_store);
static DEVICE_ATTR(fp_auth_status, S_IRUGO | S_IWUSR, fp_auth_status_show, fp_auth_status_store);
static DEVICE_ATTR(fp_area, S_IRUGO | S_IWUSR, fp_area_show, fp_area_store);

/* add your attr in here*/
static struct attribute *fts_attributes[] = {
    &dev_attr_touch_status.attr,
    &dev_attr_fts_fp_mode.attr,
    &dev_attr_fts_fp_ctrl_mode.attr,
    &dev_attr_fts_aod_ctrl_mode.attr,
    &dev_attr_fts_phone_state.attr,
    &dev_attr_fp_auth_status.attr,
    &dev_attr_fp_area.attr,
    NULL
};

static struct attribute_group asus_attribute_group = {
    .attrs = fts_attributes
};

int asus_create_sysfs(struct fts_ts_data *ts_data)
{
    int ret = 0;

    ret = sysfs_create_group(&ts_data->dev->kobj, &asus_attribute_group);
    if (ret) {
        FTS_ERROR("[EX]: asus_create_group() failed!!");
        sysfs_remove_group(&ts_data->dev->kobj, &asus_attribute_group);
        return -ENOMEM;
    } else {
        FTS_INFO("[EX]: asus_create_group() succeeded!!");
    }

    proc_create(GLOVE, 0666, NULL, &asus_ex_proc_glove_ops);
    proc_create(FPXY, 0777, NULL, &asus_ex_proc_fpxy_ops);
    proc_create(REAL_TIME, 0666, NULL, &asus_ex_proc_realtime_ops);
    proc_create(PERF_TIME, 0666, NULL, &asus_ex_proc_perftime_ops);
    
    ts_data->fp_enable = 0;
    ts_data->fp_report_type = 0;
    fts_data->auth_complete = 1;
    ts_data->phone_call_state = false;
    ts_data->perftime = DISABLE;
    ts_data->realtime = DISABLE;
    
#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
    ts_data->charge_notify.notifier_call = charge_notifier_callback;
    qti_charge_register_notify(&ts_data->charge_notify);
#endif

    return ret;
}

int asus_remove_sysfs(struct fts_ts_data *ts_data)
{
    sysfs_remove_group(&ts_data->dev->kobj, &asus_attribute_group);
#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
    qti_charge_unregister_notify(&ts_data->charge_notify);
#endif	
    return 0;
}
