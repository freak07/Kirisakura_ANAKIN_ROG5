#include "focaltech_core.h"
#include "asus_tp.h"

#define KEY_GESTURE_LEFT_LEFT                        KEY_F13
#define KEY_GESTURE_LEFT_RIGHT                       KEY_F14
#define KEY_GESTURE_RIGHT_LEFT                       KEY_F15
#define KEY_GESTURE_RIGHT_RIGHT                      KEY_F16

#define GESTURE_LEFT_LEFT                            0x11
#define GESTURE_LEFT_RIGHT                           0x12
#define GESTURE_RIGHT_LEFT                           0x13
#define GESTURE_RIGHT_RIGHT                          0x14

#define BKTP_GESTURE                                 "driver/bktp_gesture_id"

void asus_gesture_report(struct fts_ts_data *ts_data, int gesture_id)
{
    int gesture = -1;
    struct input_dev *input_dev = ts_data->input_dev;
    
    switch (gesture_id) {
      case GESTURE_LEFT_LEFT:
	   gesture = KEY_GESTURE_LEFT_LEFT;
	   break;
      case GESTURE_LEFT_RIGHT:
	   gesture = KEY_GESTURE_LEFT_RIGHT;
	   break;
      case GESTURE_RIGHT_LEFT:
	   gesture = KEY_GESTURE_RIGHT_LEFT;
	   break;	
      case GESTURE_RIGHT_RIGHT:
	   gesture = KEY_GESTURE_RIGHT_RIGHT;
	   break;	
      default:
	break;	
    }
    
    if (gesture != -1) {
        FTS_DEBUG("Gesture Code=%d", gesture);
        input_report_key(input_dev, gesture, 1);
        input_sync(input_dev);
        input_report_key(input_dev, gesture, 0);
        input_sync(input_dev);
    }
}

int asus_gesture_readdata(struct fts_ts_data *ts_data, u8 *data)
{
    int ret = 0;
    u8 fig_num;
    u8 gesture_id_1 = 0;
    u8 gesture_id_2 = 0;

    ret =fts_read_reg(READ_FIG_NUM_REG, &fig_num);
    if (ret < 0 || fig_num < 0) {
      return 1;
    }

//    if (fig_num > 0)
//	FTS_INFO("fig number %d",fig_num);
            
    ret =fts_read_reg(READ_GESTURE_1_REG, &gesture_id_1);
    if (ret < 0) {
      FTS_INFO("failed to read gesture id 1");
      return 1;
    }

    if (gesture_id_1!=0x00)
      FTS_INFO("gesture_id 1 =0x%2x ",gesture_id_1);
    
    asus_gesture_report(ts_data,gesture_id_1);
	
    if (fig_num == 2) {
	ret =fts_read_reg(READ_GESTURE_2_REG, &gesture_id_2);
	if (ret < 0) {
	    FTS_INFO("failed to read gesture id 2");
	    return 1;
	}
        if (gesture_id_1!=0x00)
	    FTS_INFO("gesture_id 2 =0x%x ",gesture_id_2);
	
	asus_gesture_report(ts_data,gesture_id_2);
    }
        
    return 0;  
}

static ssize_t asus_ex_proc_bktp_gesture_read(struct file *file, char __user *buf, size_t count, loff_t *ppos) 
{
	char *buff = NULL;
        int len = 0;
	ssize_t ret = 0;
	int ret0 = 0;
	u8 fig_num;
	u8 gesture_id_1 = 0;
	u8 gesture_id_2 = 0;
	
	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;
	
        ret0 =fts_read_reg(READ_FIG_NUM_REG, &fig_num);
        FTS_INFO("fig number %d",fig_num);
            
        ret0 =fts_read_reg(READ_GESTURE_1_REG, &gesture_id_1);
        FTS_INFO("gesture_id 1 =0x%x ",gesture_id_1);
        if (fig_num == 2) {
	  ret0 =fts_read_reg(READ_GESTURE_2_REG, &gesture_id_2);
	  FTS_INFO("gesture_id 2 =0x%x ",gesture_id_2);
        }

	len += snprintf(buff + len, PAGE_SIZE, "0x%x,0x%x\n",gesture_id_1,gesture_id_2);
	
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);
	
	return ret;  
}


static struct file_operations asus_ex_proc_bktp_gesture_ops = {
	.read  = asus_ex_proc_bktp_gesture_read,
};

int asus_gesture_init(struct fts_ts_data *ts_data)
{
    struct input_dev *input_dev = ts_data->input_dev;

//    FTS_FUNC_ENTER();
    
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_LEFT_LEFT);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_LEFT_RIGHT);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_RIGHT_LEFT);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_RIGHT_RIGHT);

    
    __set_bit(KEY_GESTURE_LEFT_LEFT, input_dev->keybit);
    __set_bit(KEY_GESTURE_LEFT_RIGHT, input_dev->keybit);
    __set_bit(KEY_GESTURE_RIGHT_LEFT, input_dev->keybit);
    __set_bit(KEY_GESTURE_RIGHT_RIGHT, input_dev->keybit);
    
    proc_create(BKTP_GESTURE, 0777, NULL, &asus_ex_proc_bktp_gesture_ops);

    ts_data->asus_gesture_en = DISABLE;
    
//    FTS_FUNC_EXIT();
    return 0;
}