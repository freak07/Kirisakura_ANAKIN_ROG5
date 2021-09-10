#include "focaltech_core.h"
#include "asus_tp.h"
/* ASUS BSP Display +++ */
#include <drm/drm_anakin.h>

/**********************************************************
report key code
**********************************************************/
//FOD action
#define KEY_GESTURE_O                           KEY_O
#define KEY_GESTURE_F                           KEY_F
#define KEY_GESTURE_U                           KEY_U
#define KEY_GESTURE_L                           KEY_L
//media
#define KEY_GESTURE_PAUSE                       KEY_PLAYPAUSE
#define KEY_GESTURE_REWIND                      KEY_REWIND
#define KEY_GESTURE_FORWARD                     KEY_FORWARD
//Zenmotion
#define KEY_GESTURE_UP                          KEY_UP
#define KEY_GESTURE_E                           KEY_E
#define KEY_GESTURE_M                           KEY_M
#define KEY_GESTURE_W                           KEY_W
#define KEY_GESTURE_S                           KEY_S
#define KEY_GESTURE_V                           KEY_V
#define KEY_GESTURE_Z                           KEY_Z
/**********************************************************
gesture id
**********************************************************/
#define GESTURE_O                               0x2A
#define GESTURE_L                               0x27
#define GESTURE_F                               0x28
#define GESTURE_U                               0x29

#define MUSIC_PAUSE				0x26
#define MUSIC_REWIND				0x51
#define MUSIC_FORWARD				0x52

#define GESTURE_UP                              0x22
#define GESTURE_DOUBLECLICK                     0x24
#define GESTURE_W                               0x31
#define GESTURE_M                               0x32
#define GESTURE_E                               0x33
#define GESTURE_S                               0x46
#define GESTURE_V                               0x54
#define GESTURE_C                               0x34
#define GESTURE_Z                               0x65

#define GESTURE_TYPE             "driver/gesture_type"
#define DCLICK                   "driver/dclick"
#define SWIPEUP                  "driver/swipeup"
/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
extern bool proximityStatus(void);

u8 reg_D1 = 0x00;
u8 reg_D2 = 0x00;
u8 reg_D5 = 0x00;
u8 reg_D6 = 0x00;
u8 reg_D7 = 0x00;
/*****************************************************************************
* Static function prototypes
*****************************************************************************/   
void write_fp_xy(struct fts_ts_data *ts_data) 
{
    u8 fp_x_h;
    u8 fp_x_l;
    u8 fp_y_h;
    u8 fp_y_l;
    int ret = 0;
    ret =fts_read_reg(FTS_FOD_X_H_POS_REG, &fp_x_h);
    ret =fts_read_reg(FTS_FOD_X_L_POS_REG, &fp_x_l);
    ret =fts_read_reg(FTS_FOD_Y_H_POS_REG, &fp_y_h);
    ret =fts_read_reg(FTS_FOD_Y_L_POS_REG, &fp_y_l);
    ts_data->fp_x = (u16)(((fp_x_h & 0x0F) << 8) + fp_x_l);
    ts_data->fp_y = (u16)(((fp_y_h & 0x0F) << 8) + fp_y_l);
    FTS_DEBUG("x %d y %d",ts_data->fp_x,ts_data->fp_y);      
}

void asus_gesture_report(struct fts_ts_data *ts_data, int gesture_id)
{
    int gesture = -1;
    struct input_dev *input_dev = ts_data->input_dev;

    if (ts_data->phone_call_state) {
	if (proximityStatus() == true){
	    FTS_INFO("in call state , ignore whole gesture event");
	    return;
	}
    }
    
    FTS_DEBUG("gesture_id:0x%x", gesture_id);
    switch (gesture_id) {
    case GESTURE_UP:
	if (ts_data->swipeup_mode == 1) {
	  gesture = KEY_GESTURE_UP;
	  FTS_INFO("key swipeup");
	}
        break;
    case GESTURE_DOUBLECLICK:
        if (ts_data->dclick_mode == 1) {
	    gesture = KEY_POWER;
	    FTS_INFO("key double click");
	}
        break;
    case GESTURE_W:
	if (ts_data->gesture_type & 1 << 1) { // W
	  gesture = KEY_GESTURE_W;
	  FTS_INFO("key W");
	}
        break;
    case GESTURE_M:
	if (ts_data->gesture_type & 1 << 4){ // M
	  gesture = KEY_GESTURE_M;
	  FTS_INFO("key M");
	}
        break;
    case GESTURE_E:
	if (fts_data->gesture_type & 1 << 3){ // e
	  gesture = KEY_GESTURE_E;
	  FTS_INFO("key E");
	}
        break;
    case GESTURE_S:
        if (ts_data->gesture_type & 1 << 2) { // S
	  gesture = KEY_GESTURE_S;
	  FTS_INFO("key S");
	}
        break;
    case GESTURE_V:
	if (fts_data->gesture_type & 1 << 6){ // V
	  gesture = KEY_GESTURE_V;
	  FTS_INFO("key V");
	}
        break;
    case GESTURE_Z:
        if (fts_data->gesture_type & 1 << 5) { // Z
	  gesture = KEY_GESTURE_Z;
	  FTS_INFO("key Z");
	}
        break;
    case GESTURE_O:
	if (ts_data->fp_enable == 1) {
	  gesture = KEY_GESTURE_O;
	  ts_data->next_resume_isaod = true;
	  FTS_INFO("key O");
	  write_fp_xy(ts_data);	  
	}
        break;
    case GESTURE_F:
	if (ts_data->fp_enable == 1) {
	  FTS_INFO("key F");
	  /* ASUS BSP Display +++ */
	  //anakin_drm_notify(ASUS_NOTIFY_FOD_TOUCHED, 1);
	  ts_data->next_resume_isaod = true;
	  gesture = KEY_GESTURE_F;
	  write_fp_xy(ts_data);
	}
        break;
    case GESTURE_U:
        if (ts_data->fp_enable == 1) {
	  FTS_INFO("key U");
	  gesture = KEY_GESTURE_U;
	  ts_data->next_resume_isaod = false;
	}
	break;
    case GESTURE_L:
        if (ts_data->fp_enable == 1) {
	  FTS_INFO("key L");
	  gesture = KEY_GESTURE_L;
	}
	break;
    case MUSIC_PAUSE:
	if (ts_data->gesture_type & 1 << 7) { // music_control
	  FTS_INFO("key MUSIC_PAUSE");
	  gesture = KEY_GESTURE_PAUSE;
	}
	break;
    case MUSIC_REWIND:
      	if (ts_data->gesture_type & 1 << 7) { // music_control
	  FTS_INFO("key MUSIC_REWIND");
	  gesture = KEY_GESTURE_REWIND;
	}
	break;
    case MUSIC_FORWARD:
      	if (ts_data->gesture_type & 1 << 7) { // music_control
	  FTS_INFO("key MUSIC_FORWARD");
	  gesture = KEY_GESTURE_FORWARD;
	}
	break;
    default:
        gesture = -1;
        break;
    }
    /* report event key */
    if (gesture != -1) {
        FTS_DEBUG("Gesture Code=%d", gesture);
        input_report_key(input_dev, gesture, 1);
        input_sync(input_dev);
        input_report_key(input_dev, gesture, 0);
        input_sync(input_dev);
    }
}

int set_gesture_register (struct fts_ts_data *ts_data) 
{
    int i = 0;
    int ret = -1; //register set result
    u8 state = 0xFF;
    
    u8 fod_bit = 0x80;//D1
    u8 music_pause_bit = 0x40;//D1
    u8 dclick_bit = 0x10;//D1
    u8 swiepup_bit= 0x04;//D1
    u8 e_bit = 0x08;//D2
    u8 m_bit = 0x04;//D2
    u8 w_bit = 0x02;//D2
    u8 s_bit = 0x40;//D5
    u8 v_bit = 0x10;//D6
    u8 music_rewind_bit = 0x04;//D6
    u8 music_forward_bit = 0x02;//D6
    u8 z_bit = 0x20;//D7
    
    reg_D1 = 0x00;
    reg_D2 = 0x00;
    reg_D5 = 0x00;
    reg_D6 = 0x00;
    reg_D7 = 0x00;
    
    if (ts_data->fp_enable){
        reg_D1 = reg_D1|fod_bit;
    }
    if (ts_data->dclick_mode == 1){
	reg_D1 = reg_D1|dclick_bit;
    }
    if (ts_data->swipeup_mode == 1) {
	reg_D1 = reg_D1|swiepup_bit;
    }
    if (ts_data->music_control== 1) {
       	reg_D1 = reg_D1|music_pause_bit;
	reg_D6 = reg_D6|music_rewind_bit;
	reg_D6 = reg_D6|music_forward_bit;
    }

    if (ts_data->gesture_mode_enable == 1){
	if (ts_data->gesture_type & 1 << 1)
	    reg_D2 = reg_D2|w_bit;
	if (ts_data->gesture_type & 1 << 2)
	    reg_D5 = reg_D5|s_bit;
	if (ts_data->gesture_type & 1 << 3)
	    reg_D2 = reg_D2|e_bit;
	if (ts_data->gesture_type & 1 << 4)
	    reg_D2 = reg_D2|m_bit;
	if (ts_data->gesture_type & 1 << 5)
	    reg_D7 = reg_D7|z_bit;
	if (ts_data->gesture_type & 1 << 6)
	    reg_D6 = reg_D6|v_bit;

    }
    
    for (i = 0; i < 5; i++) {
        fts_write_reg(0xD1, reg_D1);
        fts_write_reg(0xD2, reg_D2);
        fts_write_reg(0xD5, reg_D5);
        fts_write_reg(0xD6, reg_D6);
        fts_write_reg(0xD7, reg_D7);
        fts_write_reg(FTS_REG_GESTURE_EN, ENABLE);
        msleep(1);
        fts_read_reg(FTS_REG_GESTURE_EN, &state);
        if (state == ENABLE)
            break;
    }
    
    if (i >= 5)
        FTS_ERROR("make IC enter into gesture(suspend) fail,state:%x", state);
    else {
        FTS_INFO("Enter into gesture(suspend) successfully");
	ret = ENABLE;
    }

    FTS_INFO("reg D1:0x%2X D2:0x%2X D5:0x%2X D6:0x%2X D7:0x%2X",reg_D1,reg_D2,reg_D5,reg_D6,reg_D7);
    return ret;
}

int is_enter_gesture_mode (struct fts_ts_data *ts_data) 
{
    int enable_gesture = 0;
    
    if (ts_data->fp_enable == 1) {
        enable_gesture = 1;
	FTS_INFO("FP auth enable , enter gesture mode");
    }
    
    if (ts_data->gesture_mode_enable == 1) {
        enable_gesture = 1;
	FTS_INFO("Zenmotion enable , enter gesture mode");
    }
    
    if (ts_data->dclick_mode == 1) {
       enable_gesture = 1;
       FTS_INFO("Double click enable , enter gesture mode");
    }
    
    if (ts_data->swipeup_mode == 1) {
       enable_gesture = 1;  
       FTS_INFO("Swipe up enable , enter gesture mode");
    }
    
    return enable_gesture;
}

static ssize_t asus_gesture_proc_type_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	char *buff = NULL;
	int offset = 0;
	FTS_FUNC_ENTER();
	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	offset += sprintf(buff, "%#x \n", fts_data->gesture_type);

	ret = simple_read_from_buffer(buf, count, ppos, buff, offset);
	kfree(buff);

	return ret;
}

static ssize_t asus_gesture_proc_type_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	int tmp = 0;
	u8 gesture_tmp = 0;
	char gesture_buf[16];
	char gesture_type_buf[16] = {'0'};
	char messages[16];
	memset(messages, 0, sizeof(messages));
	FTS_FUNC_ENTER();
	
	if (len > 16)
		len = 16;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	memset(gesture_buf, 0, sizeof(gesture_buf));
	sprintf(gesture_buf, "%s", messages);
	gesture_buf[len] = '\0';
	FTS_INFO("fts_gesture_store %s ! length %d", gesture_buf, len);

	memset(gesture_type_buf, 0, sizeof(gesture_type_buf));
	gesture_type_buf[8] = '\0';
	for (tmp = 0; tmp < len; tmp++) {
		gesture_type_buf[tmp] = gesture_buf[len-tmp-1];
	}
	FTS_INFO("fts_gesture_store %s ! length %d", gesture_type_buf, len);
	if (gesture_type_buf[0] == '1') {
		fts_data->gesture_mode_enable = ENABLE;
		FTS_INFO("gesture_mode enable !");
	} else
		fts_data->gesture_mode_enable = DISABLE;

	if (gesture_type_buf[7] == '1') {
		fts_data->music_control = ENABLE;
		printk("[Focal][Touch] music_control enable ! \n");
	} else
		fts_data->music_control = DISABLE;

	if (fts_data->gesture_mode_enable == ENABLE) {
		for (tmp = 0; tmp < 8; tmp++) {
			if (gesture_type_buf[tmp] == '1') {
				gesture_tmp |= (1 << tmp);
			}
		}
		fts_data->gesture_type = gesture_tmp;
		FTS_INFO("gesture_mode_enable type = %#x !", fts_data->gesture_type);
	} else {
		fts_data->gesture_mode_enable = DISABLE;
		fts_data->music_control = DISABLE;
		fts_data->gesture_type = 0;
		FTS_INFO("gesture mode is disabled.");
	}

	return len;
}

static ssize_t asus_gesture_proc_dclick_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff = NULL;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += sprintf(buff, "%d\n", fts_data->dclick_mode);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static ssize_t asus_gesture_proc_dclick_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	memset(messages, 0, sizeof(messages));

	FTS_FUNC_ENTER();
	
	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strncmp(messages, "0", 1) == 0) {
		fts_data->dclick_mode = DISABLE;
		FTS_INFO("dclick_mode_disable");
	} else {
		fts_data->dclick_mode = ENABLE;
		FTS_INFO("dclick_mode_enable");
	}
	return len;
}

static ssize_t asus_gesture_proc_swipeup_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff = NULL;

	FTS_FUNC_ENTER();
	
	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += sprintf(buff, "%d\n", fts_data->swipeup_mode);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static ssize_t asus_gesture_proc_swipeup_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	memset(messages, 0, sizeof(messages));
	
	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strncmp(messages, "0", 1) == 0) {
		fts_data->swipeup_mode = DISABLE;
		FTS_INFO("swipeup_mode_disable");
	} else {
		fts_data->swipeup_mode = ENABLE;
		FTS_INFO("swipeup_mode_enable");
	}
	return len;
}

static struct file_operations asus_gesture_proc_type_ops = {
	.write = asus_gesture_proc_type_write,
	.read  = asus_gesture_proc_type_read,
};

static struct file_operations asus_gesture_proc_dclick_ops = {
	.write = asus_gesture_proc_dclick_write,
	.read  = asus_gesture_proc_dclick_read,
};

static struct file_operations asus_gesture_proc_swipeup_ops = {
	.write = asus_gesture_proc_swipeup_write,
	.read  = asus_gesture_proc_swipeup_read,
};

int asus_gesture_init(struct fts_ts_data *ts_data)
{
    struct input_dev *input_dev = ts_data->input_dev;

    FTS_FUNC_ENTER();

//FOD
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_O);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_F);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_U);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_L);

//media    
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_PAUSE);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_REWIND);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_FORWARD);

//Zenmotion
    input_set_capability(input_dev, EV_KEY, KEY_POWER);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_UP);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_E);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_M);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_W);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_S);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_V);
    input_set_capability(input_dev, EV_KEY, KEY_GESTURE_Z);
      
//FOD 
    __set_bit(KEY_GESTURE_O, input_dev->keybit);
    __set_bit(KEY_GESTURE_F, input_dev->keybit);
    __set_bit(KEY_GESTURE_U, input_dev->keybit);
    __set_bit(KEY_GESTURE_L, input_dev->keybit);

//Zenmotion
    __set_bit(KEY_GESTURE_UP, input_dev->keybit);
    __set_bit(KEY_GESTURE_E, input_dev->keybit);
    __set_bit(KEY_GESTURE_M, input_dev->keybit);
    __set_bit(KEY_GESTURE_W, input_dev->keybit);
    __set_bit(KEY_GESTURE_S, input_dev->keybit);
    __set_bit(KEY_GESTURE_V, input_dev->keybit);
    __set_bit(KEY_GESTURE_Z, input_dev->keybit);

    ts_data->gesture_mode_enable = 0;
    ts_data->dclick_mode = 0;
    ts_data->swipeup_mode = 0;
    ts_data->music_control = 0;
    
    proc_create(GESTURE_TYPE, 0666, NULL, &asus_gesture_proc_type_ops);
    proc_create(DCLICK, 0666, NULL, &asus_gesture_proc_dclick_ops);
    proc_create(SWIPEUP, 0666, NULL, &asus_gesture_proc_swipeup_ops);

    FTS_FUNC_EXIT();
    return 0;
}
