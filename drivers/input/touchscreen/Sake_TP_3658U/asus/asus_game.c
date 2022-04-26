#include "focaltech_core.h"
#include "asus_tp.h"

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
bool game_mode_active = false;
int touch_figer_slot[TOTAL_SLOT] = {0};
u16 touch_level = 0x0,leave_level = 0x0, first_filter = 0x0, normal_filter = 0x0;
u8 Rcoefleft = 0x0A, RcoefRight=0x0A, touch_timer = 0x0;

/*****************************************************************************
* 1.Static function prototypes
*******************************************************************************/
static u16 shex_to_u16(const char *hex_buf, int size)
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

int set_rotation_mode(int angle)
{
    int ret1 = 0, ret2 = 0, ret3 = 0;
    struct fts_ts_data *ts_data = fts_data;
    int ret = 0;
    u8 l_val = 0 , r_val = 0;
    if (ts_data->game_mode == ENABLE) {
	switch (angle) {
	    case 90:
	      	ret1 = fts_write_reg(FTS_REG_EDGEPALM_MODE_EN, 0x01);
		ret2 = fts_write_reg(FTS_REG_EDGEPALM_LEFT, Rcoefleft);
		ret3 = fts_write_reg(FTS_REG_EDGEPALM_RIGHT, RcoefRight);
		if (ret1 < 0 || ret2 < 0) {
		  FTS_INFO("Game mode enable,set rotation reg to 90 fail");
		  ret = -1;
		} else
                  game_mode_active = true;
	      break;
	    case 270:
	        ret1 = fts_write_reg(FTS_REG_EDGEPALM_MODE_EN, 0x02);
		ret2 = fts_write_reg(FTS_REG_EDGEPALM_LEFT, Rcoefleft);
		ret3 = fts_write_reg(FTS_REG_EDGEPALM_RIGHT, RcoefRight);
		if (ret1 < 0 || ret2 < 0) {
		    FTS_INFO("Game mode enable,set rotation reg to 270 fail ");
		    ret = -1;
		} else
		    game_mode_active =true;

	      break;
	    case 0:
	      if (game_mode_active) {
		  ret1 = fts_write_reg(FTS_REG_EDGEPALM_MODE_EN, 0x00);
		  if (ret1 < 0 || ret2 < 0) {
		      FTS_INFO("Game mode enable,set rotation reg to 0 fail ");
		      ret = -1;
		  } else 
		      game_mode_active =false;
	      }
	}
	fts_read_reg(FTS_REG_EDGEPALM_LEFT, &l_val);
	fts_read_reg(FTS_REG_EDGEPALM_RIGHT, &r_val);
	FTS_INFO("Game mode enable,set rotation reg to %d , palm range left %x right %x",angle,l_val,r_val);	
    } else {
	if (game_mode_active) {
	    ret1 = fts_write_reg(FTS_REG_EDGEPALM_MODE_EN, 0x00);
	    if (ret1 < 0 || ret2 < 0) {
		FTS_INFO("Game mode Exit ,set rotation reg to 0 fail");
		ret = -1;
	    } else {
		FTS_INFO("Game mode Exit ,set rotation reg to 0");
		game_mode_active =false;
	    }
	} else 
	    FTS_INFO("Not in game mode and never active corner protection, ignore rotation exit");
    }
    
    return ret;
}

void ATR_touch(int id,int action, int x, int y, int random)
{
	static int random_x = -5, random_y = -5, random_major = -5;
	struct input_dev *input_dev = fts_data->input_dev;
	int first_empty_slot = -1;
	int i;

	FTS_INFO("keymapping ATR_touch  id=%d, action=%d, x=%d, y=%d", id, action,  x,  y);
	mutex_lock(&input_dev->mutex);
	if(action) //press, find first slot or find last slot;
	{
		for(i = TOTAL_SLOT -1; i >= 0 ; i--)
		{
			if(first_empty_slot == -1 && touch_figer_slot[i] == 0) //find first empty slot
				first_empty_slot = i;
			if(touch_figer_slot[i] == (id + 1)) //if the last id has been pressed, keep reporting same slot
				first_empty_slot = i;
		}
		FTS_INFO("keymapping ATR_touch press found slot %d", first_empty_slot);
		if(first_empty_slot != -1) // found an available slot
		{
			if(touch_figer_slot[first_empty_slot] ==0)
				FTS_INFO("keymapping report %d down x=%d ,y=%d ",first_empty_slot,x,y);

			input_mt_slot(input_dev, first_empty_slot + 10);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 0x09 + random_major);

			if(!random)
			{
				input_report_abs(input_dev, ABS_MT_POSITION_X, x + random_x);
				input_report_abs(input_dev, ABS_MT_POSITION_Y, y + random_y);
			} else {
				input_report_abs(input_dev, ABS_MT_POSITION_X, x );
				input_report_abs(input_dev, ABS_MT_POSITION_Y, y );
			}
			FTS_INFO("slot %d", first_empty_slot+10);
			if(!fts_data->finger_press){
				FTS_INFO("no touch BTN down , atr touch down");
				fts_data->atr_press = true;
				input_report_key(input_dev, BTN_TOUCH, 1);
			} else {
				FTS_INFO("touch BTN down long pressed , ignore atr touch down");
			}
			input_sync(input_dev);

			touch_figer_slot[first_empty_slot] = id + 1; // save finger id in slot 			
		}
	} 
	else //release
	{
		for(i = TOTAL_SLOT -1; i >= 0 ; i--)
		{
			if(touch_figer_slot[i] == (id + 1)) //find the released slot
			{
				first_empty_slot = i;
				break;
			}
		}

		FTS_INFO("keymapping  release slot %d", first_empty_slot);
		if(first_empty_slot >= 0)
		{
			input_mt_slot(input_dev, first_empty_slot + 10);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
			input_sync(input_dev);
			touch_figer_slot[first_empty_slot] = 0;
		}
	}

	for(i = TOTAL_SLOT -1; i >= 0 ; i--)
	{
	    if (touch_figer_slot[i] != 0) //find the released slot
		break;
	}   
	if(i < 0) // all button up
	{
		fts_data->atr_press = false;
		if(!fts_data->finger_press)
		{
		    FTS_INFO("no touch and atr BTN down, all BTN up");
		    input_report_key(input_dev, BTN_TOUCH, 0);
		    input_sync(input_dev);
		}
	}
	
	random_x += 1; if(random_x > 5) random_x = -5;
	random_y += 1; if(random_y > 5) random_y = -5;
	random_major += 1; if(random_major > 5) random_major = -5;

	mutex_unlock(&input_dev->mutex);
}

/*
 * attribute functions
 */
static ssize_t keymapping_touch_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
    bool stat = 0;
    if (fts_data->atr_press)
	stat = true;
    else
	stat = false;

    return sprintf(buf, "%d", stat);
}

static ssize_t keymapping_touch_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	int id = 0, action = 0, x = 0, y = 0, random = 0, minus = 0;
	FTS_INFO("keymapping cmd buf: %s len=%d", buf, count);
	
	if ((count > 16) || (count < 14)){
	    FTS_INFO("Invalid cmd buffer %d", count);
	    return -EINVAL;
	}

	if (count == 14) {
		id = buf[0] - '0';
		action = buf[1] - '0';
		random = buf[2] - '0';
		
		minus = buf[3];
		x =  shex_to_u16(buf + 4, 4);
		if(minus == '-')
			x = -x;

		minus = buf[8];
		y =  shex_to_u16(buf + 9, 4);
		if(minus == '-')
			y = -y;
	}
	
	FTS_INFO("keymapping ID=%d ACTION=%d X=%d Y=%d RANDOM=%d", id, action, x, y, random);
	ATR_touch(id, action, x, y, random);

	return count;
}

static ssize_t fts_game_mode_show (struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    struct fts_ts_data *ts_data = fts_data;
    struct input_dev *input_dev = ts_data->input_dev;

    mutex_lock(&input_dev->mutex);
    
    count = snprintf(buf + count, PAGE_SIZE, "Game Mode:%s\n",
                     ts_data->game_mode ? "On" : "Off");
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_game_mode_store(
    struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct fts_ts_data *ts_data = fts_data;
    int ret = 0;
    if (FTS_SYSFS_ECHO_ON(buf)) {
        if (!ts_data->game_mode) {
            FTS_DEBUG("enter game mode");
            ts_data->game_mode = ENABLE;
	    if (ts_data->rotation_angle!=0) {
		ret = set_rotation_mode(ts_data->rotation_angle);
	    }
        }
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        if (ts_data->game_mode) {
            FTS_DEBUG("exit game mode");
            ts_data->game_mode = DISABLE;
	    ts_data->rotation_angle = 0;
	    ret = set_rotation_mode(ts_data->rotation_angle);
        }
    }

    FTS_DEBUG("game mode:%d", ts_data->game_mode);
    return count;
}

static ssize_t fts_rotation_mode_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d \n", fts_data->rotation_angle);
}

static ssize_t fts_rotation_mode_store(
    struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	if (buf[0] == '1') {
		fts_data->rotation_angle = 90;
	} else if (buf[0] == '2') {
		fts_data->rotation_angle = 270;
	} else if (buf[0] == '0') {
		fts_data->rotation_angle = 0;
	}
	
	ret = set_rotation_mode(fts_data->rotation_angle);
	FTS_INFO("set angle protect with rotation %d result %d",fts_data->rotation_angle, ret);
	return count;
}

static ssize_t game_settings_store(
    struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char game_settings[29];
	
	memset(game_settings, 0, sizeof(game_settings));
	sprintf(game_settings, "%s", buf);
	game_settings[count-1] = '\0';
	FTS_INFO("game_settings %s count %d ",game_settings,count);
	
	if(count != 28){
            return -EINVAL;
	}
	
	touch_level = (u16)shex_to_u16(game_settings +0, 3);
	leave_level = (u16)shex_to_u16(game_settings +4, 3);
	first_filter = (u16)shex_to_u16(game_settings +8, 3);
	normal_filter = (u16)shex_to_u16(game_settings +12, 3);
	Rcoefleft = (u8)shex_to_u16(game_settings +16, 3);
	RcoefRight = (u8)shex_to_u16(game_settings +20, 3);
	touch_timer = (u8)shex_to_u16(game_settings +24, 3);
	
	if (Rcoefleft > 0x0A)
	    Rcoefleft = 0x0A;
	if (RcoefRight > 0x0A)
	    RcoefRight = 0x0A;

	FTS_INFO("touch_level 0x%04X, leave_level 0x%04X first_filter 0x%04X, normal_filter 0x%04X",
		touch_level,leave_level,first_filter,normal_filter);
	FTS_INFO("Rcoefleft 0x%02X,RcoefRight 0x%02X touch_timer 0x%02X",Rcoefleft,RcoefRight,touch_timer);

	return count;
}

static DEVICE_ATTR(keymapping_touch, S_IRUGO | S_IWUSR, keymapping_touch_show, keymapping_touch_store);
static DEVICE_ATTR(fts_game_mode, S_IRUGO | S_IWUSR, fts_game_mode_show, fts_game_mode_store);
static DEVICE_ATTR(fts_rotation_mode, S_IRUGO | S_IWUSR, fts_rotation_mode_show, fts_rotation_mode_store);
static DEVICE_ATTR(game_settings, S_IRUGO | S_IWUSR, NULL, game_settings_store);
/* add your attr in here*/
static struct attribute *fts_attributes[] = {
    &dev_attr_keymapping_touch.attr,
    &dev_attr_fts_game_mode.attr,
    &dev_attr_fts_rotation_mode.attr,
    &dev_attr_game_settings.attr,
    NULL
};

static struct attribute_group asus_game_attribute_group = {
    .attrs = fts_attributes
};

void asus_game_recovery(struct fts_ts_data *ts_data) 
{
    int ret = 0 ;
    if (ts_data->game_mode == ENABLE) {
      FTS_INFO("Game mode recovery");
      ret = set_rotation_mode(ts_data->rotation_angle);
    }
  
}

int asus_game_create_sysfs(struct fts_ts_data *ts_data)
{
    int ret = 0;

    ret = sysfs_create_group(&ts_data->dev->kobj, &asus_game_attribute_group);
    if (ret) {
        FTS_ERROR("[EX]: asus_create_group() failed!!");
        sysfs_remove_group(&ts_data->dev->kobj, &asus_game_attribute_group);
        return -ENOMEM;
    } else {
        FTS_INFO("[EX]: asus_create_group() succeeded!!");
    }
    
    ts_data->game_mode = DISABLE;
    ts_data->rotation_angle = 0;
    
    return ret;
}

int asus_game_remove_sysfs(struct fts_ts_data *ts_data)
{
    sysfs_remove_group(&ts_data->dev->kobj, &asus_game_attribute_group);
    return 0;
}