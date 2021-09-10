#include "focaltech_core.h"
#include "asus_tp.h"
/*****************************************************************************
* 1.Static variables
*****************************************************************************/
#define BKTP_GESTURE                            "driver/bktp_en_gesture"
#define BKTP_GESTURE_LEN                        "driver/bktp_vaild_len"
#define INIT_LENGTH                             0x21 //33  , 0.99mm
#define LENGTH_1                                0x64 //100 , 3.00mm
#define LENGTH_2                                0xA7 //168 , 5.01mm
#define LENGTH_3                                0xE9 //233 , 6.99mm


/*****************************************************************************
* 2.Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* 3. Global variable or extern global variabls/functions
*****************************************************************************/
/*static u16 shex_to_u16(const char *hex_buf, int size)
{
    int i;
    int base = 1;
    int value = 0;
    char single;

    for (i = size - 1; i >= 0; i--) {
        single = hex_buf[i];

        if ((single >= '0') && (single <= '9')) {
            value += (single - '0') * base;
        } else if ((single >= 'a') && (single <= 'z')) {
            value += (single - 'a' + 10) * base;
        } else if ((single >= 'A') && (single <= 'Z')) {
            value += (single - 'A' + 10) * base;
        } else {
            return -EINVAL;
        }
	base *= 10;
    }

    return value;
}
*/
void set_vaild_length(int level)
{
  int ret = 0;

  FTS_INFO("set vaild length %d",level);
  
  if (level == 0)
      ret = fts_write_reg(SET_GESTURE_LEN_REG,INIT_LENGTH);
  if (level == 1)
      ret = fts_write_reg(SET_GESTURE_LEN_REG,LENGTH_1);
  if (level == 2)
      ret = fts_write_reg(SET_GESTURE_LEN_REG,LENGTH_2);
  if (level == 3)
      ret = fts_write_reg(SET_GESTURE_LEN_REG,LENGTH_3);

  if (ret< 0 ) {
    FTS_INFO("set vaild length fai;");
    return;
  }    
}

/*****************************************************************************
* 4.Static function prototypes
*******************************************************************************/
static ssize_t asus_ex_proc_bktp_gesture_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff = NULL;
	struct fts_ts_data *ts_data = fts_data;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += snprintf(buff + len, PAGE_SIZE, "%d\n", ts_data->asus_gesture_en);

	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static ssize_t asus_ex_proc_bktp_gesture_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[10];
	memset(messages, 0, sizeof(messages));

	if (len > 10)
		len = 10;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strncmp(messages, "0", 1) == 0) {
	    FTS_DEBUG("Back touch gesture mode disable");
	    fts_data->asus_gesture_en = DISABLE;
	    fts_ts_suspend_resume(true); // suspend to power off
	} else {
	    FTS_DEBUG("Back touch gesture mode enable");
	    fts_data->asus_gesture_en = ENABLE;
	    asus_gesture_resume(true); //resume into active mode
	    set_vaild_length(fts_data->vaild_len);
	}
	return len;
}

static ssize_t asus_ex_proc_bktp_gesture_len_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
  
  	int len = 0;
	ssize_t ret = 0;
	char *buff = NULL;
	u8 val = 0x0;
	int ret0 = 0;
	
	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
	  return -ENOMEM;

	ret0 = fts_read_reg(SET_GESTURE_LEN_REG, &val);

	len += snprintf(buff + len, PAGE_SIZE, "level set %d read 0x%X\n", fts_data->vaild_len,val);

	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static ssize_t asus_ex_proc_bktp_gesture_len_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
  	char messages[10];
	memset(messages, 0, sizeof(messages));

	if (len > 4)
	  return -EFAULT;
	
	if (copy_from_user(messages, buff, len))
		return -EFAULT;
	
	FTS_INFO("read messages %s",messages);
	
	if (strncmp(messages, "0", 0) == 0) {
	  fts_data->vaild_len = 0;
	}
	
	if (strncmp(messages, "1", 1) == 0) {
	  fts_data->vaild_len = 1;
	}

	if (strncmp(messages, "2", 2) == 0) {
	  fts_data->vaild_len = 2;
	}

	if (strncmp(messages, "3", 3) == 0) {
	  fts_data->vaild_len = 3;
	}
	
	if (fts_data->asus_gesture_en == ENABLE)
	    set_vaild_length(fts_data->vaild_len);

	FTS_DEBUG("vaild length level set to :%d",fts_data->vaild_len);
	
	return len;
}

static struct file_operations asus_ex_proc_bktp_gesture_ops = {
	.write = asus_ex_proc_bktp_gesture_write,
	.read  = asus_ex_proc_bktp_gesture_read,
};

static struct file_operations asus_ex_proc_bktp_gesture_len_ops = {
	.write = asus_ex_proc_bktp_gesture_len_write,
	.read  = asus_ex_proc_bktp_gesture_len_read,
};


int asus_create_sysfs(struct fts_ts_data *ts_data)
{
    int ret = 0;

    proc_create(BKTP_GESTURE, 0777, NULL, &asus_ex_proc_bktp_gesture_ops);
    proc_create(BKTP_GESTURE_LEN, 0777, NULL, &asus_ex_proc_bktp_gesture_len_ops);    
    
    ts_data->asus_gesture_en = DISABLE;
    ts_data->vaild_len = 1;
    
    return ret;
}

int asus_remove_sysfs(struct fts_ts_data *ts_data)
{
    return 0;
}
