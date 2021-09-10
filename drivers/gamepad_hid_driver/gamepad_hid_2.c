#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/hidraw.h>
#include <linux/usb.h>
#include <linux/time.h>

#include "gamepad_hid.h"
#define RGB_MAX 21   //for rainbow color setting

static int mode2_state=0;
static int apply_state=0;

struct gamepad_drvdata {
	struct led_classdev led;
};

static struct hidraw *gamepad_hidraw;

static u32 g_red;
static u32 g_green;
static u32 g_blue;
static u32 g_mode;
static u32 g_speed;
static u32 g_led_on;

extern int asus_usbhid_set_raw_report(struct hid_device *hid, unsigned int reportnum,
				 __u8 *buf, size_t count, unsigned char rtype);

static int asus_usb_hid_write(u8 vlaues,char asus_gp_command_id)
{
	struct hid_device *hdev;
	unsigned char report_type;
	unsigned int report_number;
	int ret = 0;
	int len = 0;
	char *buffer;

	pr_debug("[GAMEPAD_II] asus_usb_hid_write asus_gp_command_id: %d,value: %d\n", asus_gp_command_id,vlaues);

	if (gamepad_hidraw == NULL) {
		pr_err("[GAMEPAD_II] gamepad_hidraw is NULL !\n");
		return -1;
	}

	if(asus_gp_command_id < 0x80)
	{
		pr_err("[GAMEPAD_II] error asus gamepad set command : 0x%x\n", asus_gp_command_id);
		return -1;
	}
	
	buffer = kzalloc(0x08, GFP_KERNEL);

	memset(buffer,0,8);

	hdev = gamepad_hidraw->hid;

	hid_hw_power(hdev, PM_HINT_FULLON);

	report_number = ASUS_GAMEPAD_REPORT_ID;
	buffer[0] = report_number;
	buffer[1] = asus_gp_command_id; // ASUSCmdID
	buffer[2] = vlaues; 
	len = 8;
	report_type = ASUS_GAMEPAD_REPORT_TYPE;

	ret = asus_usbhid_set_raw_report(hdev,report_number,buffer,len,report_type);

	hid_hw_power(hdev, PM_HINT_NORMAL);

	kfree(buffer);

	return ret;

}

static int asus_usb_hid_read(u8 *data,char asus_gp_command_id)
{
	int ret = 0;
	struct hid_device *hdev;
	unsigned char report_type;
	unsigned char report_number;
	char *buffer;
	int count = 0;

	pr_debug("[GAMEPAD_II] asus_usb_hid_read asus_gp_command_id=%d\n",asus_gp_command_id);

	if (gamepad_hidraw == NULL) {
		pr_err("[GAMEPAD_II] gamepad_hidraw is NULL !\n");
		return -1;
	}
	
	if(asus_gp_command_id > 0x80)
	{
		pr_err("[GAMEPAD_II] error asus gamepad set command : 0x%x\n", asus_gp_command_id);
		return -1;
	}
	
	buffer = kzalloc(0x02, GFP_KERNEL);

	memset(buffer,0,2);

	hdev = gamepad_hidraw->hid;

	hid_hw_power(hdev, PM_HINT_FULLON);

	report_number = ASUS_GAMEPAD_REPORT_ID;
	count = 0x2;
	report_type = asus_gp_command_id-1;

	ret = hid_hw_raw_request(hdev, report_number, buffer, count, report_type,
				 HID_REQ_GET_REPORT);
	
	(*data) = buffer[1];

	//printk("[GAMEPAD_II] asus_usb_hid_read : %d\n", (*data));

	hid_hw_power(hdev, PM_HINT_NORMAL);

	kfree(buffer);

	return ret;
}

static ssize_t red_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	u32 reg_val;
	int err = 0;
	ssize_t ret;

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;
	//printk("[GAMEPAD_II] red reg_val %d\n", reg_val);
	pr_debug("[GAMEPAD_II] %s reg_val %d\n", __func__,reg_val);

	g_red=reg_val;

	err = asus_usb_hid_write(reg_val,ASUS_GAMEPAD_SET_RED_PWM);
	if (err < 0)
		pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);

	return count;
}

static ssize_t red_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{

	unsigned char data= 0;
	int err = 0;
	pr_debug("[GAMEPAD_II] %s\n",__func__);
	err = asus_usb_hid_read(&data,ASUS_GAMEPAD_GET_RED_PWM);
	if (err < 0)
		pr_err("[GAMEPAD_II] red_pwm_show:err %d\n", err);

	return snprintf(buf, PAGE_SIZE,"%d\n", data);
}

static ssize_t green_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	u32 reg_val;
	int err = 0;
	ssize_t ret;

	ret = kstrtou32(buf, 10, &reg_val);
	pr_debug("[GAMEPAD_II] %s reg_val %d\n", __func__,reg_val);
	
	if (ret)
		return count;
	//printk("[GAMEPAD_II] green reg_val %d\n", reg_val);
	g_green=reg_val;
	err = asus_usb_hid_write(reg_val,ASUS_GAMEPAD_SET_GREEN_PWM);
	if (err < 0)
		pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);

	return count;
}

static ssize_t green_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{

	unsigned char data= 0;
	int err = 0;
	pr_debug("[GAMEPAD_II] %s\n",__func__);

	err = asus_usb_hid_read(&data,ASUS_GAMEPAD_GET_GREEN_PWM);
	if (err < 0)
		pr_err("[GAMEPAD_II] green_pwm_show:err %d\n", err);

	return snprintf(buf, PAGE_SIZE,"%d\n", data);
}

static ssize_t blue_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	u32 reg_val;
	int err = 0;
	ssize_t ret;

	ret = kstrtou32(buf, 10, &reg_val);
	pr_debug("[GAMEPAD_II] %s reg_val %d\n", __func__,reg_val);

	if (ret)
		return count;
	//printk("[GAMEPAD_II] blue reg_val %d\n", reg_val);
	g_blue=reg_val;
	err = asus_usb_hid_write(reg_val,ASUS_GAMEPAD_SET_BLUE_PWM);
	if (err < 0)
		pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);

	return count;
}

static ssize_t blue_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{

	unsigned char data= 0;
	int err = 0;
	pr_debug("[GAMEPAD_II] %s\n",__func__);
	err = asus_usb_hid_read(&data,ASUS_GAMEPAD_GET_BLUE_PWM);
	if (err < 0)
		pr_err("[GAMEPAD_II] blue_pwm_show:err %d\n", err);

	return snprintf(buf, PAGE_SIZE,"%d\n", data);
}

static ssize_t apply_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	u32 reg_val;
	int err = 0;
	ssize_t ret;

	pr_debug("[GAMEPAD_II] apply_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret){
		apply_state=-1;
		return count;
	}
	//printk("[GAMEPAD_II] apply reg_val %d\n", reg_val);
	pr_info("[GAMEPAD_II] Send apply. RGB:%d %d %d, mode:%d, speed:%d, led_on:%d\n", g_red, g_green, g_blue, g_mode, g_speed, g_led_on);
	err = asus_usb_hid_write(reg_val,ASUS_GAMEPAD_SET_APPLY);
	if (err < 0){
		apply_state=-1;
		pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
	}

	return count;
}

static ssize_t apply_show(struct device *dev, struct device_attribute *attr,char *buf)
{

	return snprintf(buf, PAGE_SIZE,"%d\n", apply_state);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	u32 reg_val;
	int err = 0;
	ssize_t ret;

	//printk("[GAMEPAD_II] mode_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;
	pr_debug("[GAMEPAD_II] %s reg_val %d\n", __func__,reg_val);
	g_mode=reg_val;
	err = asus_usb_hid_write(reg_val,ASUS_GAMEPAD_SET_MODE);
	if (err < 0)
		pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);

	return count;
}

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,char *buf)
{

	unsigned char data = 0;
	int err = 0;
	pr_debug("[GAMEPAD_II] %s\n",__func__);
	err = asus_usb_hid_read(&data,ASUS_GAMEPAD_GET_MODE);
	if (err < 0)
		pr_err("[GAMEPAD_II] mode_show:err %d\n", err);

	return snprintf(buf, PAGE_SIZE,"%d\n",data);
}

static ssize_t fw_ver_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	unsigned char data= 0;
	int err = 0;
	pr_debug("[GAMEPAD_II] %s\n",__func__);
	err = asus_usb_hid_read(&data,ASUS_GAMEPAD_GET_FW_VERSION);
	if (err < 0)
		pr_err("[GAMEPAD_II] fw_ver_show:err %d\n", err);

	return snprintf(buf, PAGE_SIZE,"0x%x\n", data);
}

static ssize_t fw_mode_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	unsigned char data= 0;
	int err = 0;
	pr_debug("[GAMEPAD_II] %s\n",__func__);
	err = asus_usb_hid_read(&data,ASUS_GAMEPAD_GET_FWMODE);
	if (err < 0){
		pr_err("[GAMEPAD_II] fw_mode_show:err %d\n", err);
	}

	return snprintf(buf, PAGE_SIZE,"0x%x\n", data);
}


static ssize_t frame_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	u32 reg_val;
	int err = 0;
	ssize_t ret;

	//printk("[GAMEPAD_II] frame_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;
	pr_info("[GAMEPAD_II] frame_store  reg_val %d\n", reg_val);

	err = asus_usb_hid_write(reg_val,ASUS_GAMEPAD_SET_FRAME);
	if (err < 0)
		pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);

	return count;
}

static ssize_t frame_show(struct device *dev, struct device_attribute *attr,char *buf)
{

	unsigned char data= 0;
	int err = 0;
	pr_debug("[GAMEPAD_II] %s\n",__func__);
	err = asus_usb_hid_read(&data,ASUS_GAMEPAD_GET_FRAME);
	if (err < 0)
		pr_err("[GAMEPAD_II] frame_show:err %d\n", err);
	pr_info("[GAMEPAD_II] %s\n data=%d",__func__,data);
	return snprintf(buf, PAGE_SIZE,"%d\n", data);
}

static ssize_t mode2_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned char rgb[RGB_MAX] = {0};
	unsigned char rainbow_mode = 0;
	unsigned char mode2 = 0;
	int err = 0;
	int rgb_num=0;//rgb_num_t=0;
	long rgb_tmp = 0;
	int ntokens = 0;
	const char *cp = buf;
	const char *buf_tmp;
	int i = 0;
	mode2_state=0;

	sscanf(buf, "%d", &mode2);
	//printk("[GAMEPAD_II] mode2_store mode2 = 0x%x.\n",mode2);
	while ((cp = strpbrk(cp + 1, ",")))
	{
		ntokens++;  //the number of ","
	}
	pr_info("[GAMEPAD_II] mode2_store mode2 = 0x%x buf=%s ntokens=%d .\n",mode2,buf,ntokens);
	if(ntokens > 6)
	{
		pr_err("[AURA_ML51_INBOX] mode2_store,wrong input,too many ntokens\n");
		mode2_state=-1;
		return count;
	}
	cp=buf;
	while((cp = strpbrk(cp, ",")))  //goto the ",".
	{
		cp++; // go after the ','
		while(*cp != ',' && *cp != '\0' && *cp !='\n')
		{
			if(*cp==' '){
				cp++; //skip the ' '
			}else{
				buf_tmp = cp;
				rgb_tmp = 0;
				sscanf(buf_tmp, "%x",&rgb_tmp);
				rgb[rgb_num++] = (rgb_tmp >> 16)&0xFF;
				rgb[rgb_num++] = (rgb_tmp >> 8)&0xFF;
				rgb[rgb_num++] = rgb_tmp & 0xFF;
				break;
			}
		}
	}

	if(rgb_num != ntokens*3){
		pr_err("[AURA_ML51_INBOX] mode2_store,wrong input,rgb_num != ntokens*3\n");
		mode2_state=-1;
		return count;
	}

	/*for(i=0;i<rgb_num;i++)
	{
		printk("AURA_ML51_INBOX] mode2_store, rgb[%d]=0x%x \n",i,rgb[i]);
	}*/

	switch(mode2){
		case 0: //closed
			rainbow_mode = 0;
			err = asus_usb_hid_write(rainbow_mode,ASUS_GAMEPAD_SET_MODE);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			break;
		case 1: //6 color rainbow
			if(ntokens != 6){
				pr_err("[GAMEPAD_II] mode2_store,wrong input.\n");
				mode2_state=-1;
				return count;
			}
			rainbow_mode = 5;
			for(i=0;i<rgb_num;i++){
				err = asus_usb_hid_write(rgb[i],0x90+i);
				if (err < 0){
					mode2_state=-1;
					pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
				}
			}

			err = asus_usb_hid_write(rainbow_mode,ASUS_GAMEPAD_SET_MODE);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			break;
		case 10: //6 color rainbow in the different direction
			if(ntokens != 6){
				pr_err("[GAMEPAD_II] mode2_store,wrong input.\n");
				mode2_state=-1;
				return count;
			}
			rainbow_mode = 9;
			for(i=0;i<rgb_num;i++){
				err = asus_usb_hid_write(rgb[i],0x90+i);
				if (err < 0){
					mode2_state=-1;
					pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
				}
			}

			err = asus_usb_hid_write(rainbow_mode,ASUS_GAMEPAD_SET_MODE);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			break;
		case 2: //static
			if(ntokens != 1){
				pr_err("[GAMEPAD_II] mode2_store,wrong input.\n");
				mode2_state=-1;
				return count;
			}
			rainbow_mode = 1;
			err = asus_usb_hid_write(rgb[0],ASUS_GAMEPAD_SET_RED_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			err = asus_usb_hid_write(rgb[1],ASUS_GAMEPAD_SET_GREEN_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			err = asus_usb_hid_write(rgb[2],ASUS_GAMEPAD_SET_BLUE_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}

			err = asus_usb_hid_write(rainbow_mode,ASUS_GAMEPAD_SET_MODE);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			break;
		case 3: //breath at the same time
			if(ntokens != 1){
				pr_err("[GAMEPAD_II] mode2_store,wrong input.\n");
				mode2_state=-1;
				return count;
			}
			rainbow_mode = 0x2;
			err = asus_usb_hid_write(rgb[0],ASUS_GAMEPAD_SET_RED_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			err = asus_usb_hid_write(rgb[1],ASUS_GAMEPAD_SET_GREEN_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			err = asus_usb_hid_write(rgb[2],ASUS_GAMEPAD_SET_BLUE_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}

			err = asus_usb_hid_write(rainbow_mode,ASUS_GAMEPAD_SET_MODE);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			break;
		case 4: //breath at different time
			if(ntokens != 1){
				pr_err("[GAMEPAD_II] mode2_store,wrong input.\n");
				mode2_state=-1;
				return count;
			}
			rainbow_mode = 8;
			err = asus_usb_hid_write(rgb[0],ASUS_GAMEPAD_SET_RED_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			err = asus_usb_hid_write(rgb[1],ASUS_GAMEPAD_SET_GREEN_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			err = asus_usb_hid_write(rgb[2],ASUS_GAMEPAD_SET_BLUE_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}

			err = asus_usb_hid_write(rainbow_mode,ASUS_GAMEPAD_SET_MODE);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			break;
		case 5: //breath only one led
		case 11: //breath only one led
			if(ntokens != 1){
				pr_err("[GAMEPAD_II] mode2_store,wrong input.\n");
				mode2_state=-1;
				return count;
			}
			rainbow_mode = 0x2;
			err = asus_usb_hid_write(rgb[0],ASUS_GAMEPAD_SET_RED_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			err = asus_usb_hid_write(rgb[1],ASUS_GAMEPAD_SET_GREEN_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			err = asus_usb_hid_write(rgb[2],ASUS_GAMEPAD_SET_BLUE_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}

			err = asus_usb_hid_write(rainbow_mode,ASUS_GAMEPAD_SET_MODE);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			break;
		case 6: //commet
			if(ntokens != 1){
				pr_err("[GAMEPAD_II] mode2_store,wrong input.\n");
				mode2_state=-1;
				return count;
			}
			rainbow_mode = 0x6;
			err = asus_usb_hid_write(rgb[0],ASUS_GAMEPAD_SET_RED_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			err = asus_usb_hid_write(rgb[1],ASUS_GAMEPAD_SET_GREEN_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			err = asus_usb_hid_write(rgb[2],ASUS_GAMEPAD_SET_BLUE_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}

			err = asus_usb_hid_write(rainbow_mode,ASUS_GAMEPAD_SET_MODE);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			break;
		case 8: //commet
			if(ntokens != 1){
				pr_err("[GAMEPAD_II] mode2_store,wrong input.\n");
				mode2_state=-1;
				return count;
			}
			rainbow_mode = 10;
			err = asus_usb_hid_write(rgb[0],ASUS_GAMEPAD_SET_RED_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			err = asus_usb_hid_write(rgb[1],ASUS_GAMEPAD_SET_GREEN_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			err = asus_usb_hid_write(rgb[2],ASUS_GAMEPAD_SET_BLUE_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}

			err = asus_usb_hid_write(rainbow_mode,ASUS_GAMEPAD_SET_MODE);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			break;
		case 7: //flash and dash
			if(ntokens != 1){
				pr_err("[GAMEPAD_II] mode2_store,wrong input.\n");
				mode2_state=-1;
				return count;
			}
			rainbow_mode = 0x7;
			err = asus_usb_hid_write(rgb[0],ASUS_GAMEPAD_SET_RED_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			err = asus_usb_hid_write(rgb[1],ASUS_GAMEPAD_SET_GREEN_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			err = asus_usb_hid_write(rgb[2],ASUS_GAMEPAD_SET_BLUE_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}

			err = asus_usb_hid_write(rainbow_mode,ASUS_GAMEPAD_SET_MODE);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			break;
		case 9: //flash and dash
			if(ntokens != 1){
				pr_err("[GAMEPAD_II] mode2_store,wrong input.\n");
				mode2_state=-1;
				return count;
			}
			rainbow_mode = 11;
			err = asus_usb_hid_write(rgb[0],ASUS_GAMEPAD_SET_RED_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			err = asus_usb_hid_write(rgb[1],ASUS_GAMEPAD_SET_GREEN_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			err = asus_usb_hid_write(rgb[2],ASUS_GAMEPAD_SET_BLUE_PWM);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}

			err = asus_usb_hid_write(rainbow_mode,ASUS_GAMEPAD_SET_MODE);
			if (err < 0){
				mode2_state=-1;
				pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);
			}
			break;
		default:
			break;
	}
	return count;
}

static ssize_t mode2_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE,"%d\n", mode2_state);
}


static ssize_t led_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	u32 reg_val;
	int err = 0;
	ssize_t ret;

	//printk("[GAMEPAD_II] led_on_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;
	pr_debug("[GAMEPAD_II] led_on_store reg_val %d\n", reg_val);
	g_led_on=reg_val;
	err = asus_usb_hid_write(reg_val,ASUS_GAMEPAD_SET_LED_ON);
	if (err < 0)
		pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);

	return count;
}

static ssize_t led_on_show(struct device *dev, struct device_attribute *attr,char *buf)
{

	unsigned char data= 0;
	int err = 0;
	err = asus_usb_hid_read(&data,ASUS_GAMEPAD_GET_LED_ON);
	if (err < 0)
		pr_err("[GAMEPAD_II] led_on_show:err %d\n", err);

	return snprintf(buf, PAGE_SIZE,"%d\n", data);
}

static ssize_t speed_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	u32 reg_val;
	int err = 0;
	ssize_t ret;

	//printk("[GAMEPAD_II] speed_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;
	pr_debug("[GAMEPAD_II] speed_store reg_val %d\n", reg_val);
	g_speed=reg_val;
	err = asus_usb_hid_write(reg_val,ASUS_GAMEPAD_SET_SPEED);
	if (err < 0)
		pr_err("[GAMEPAD_II] asus_usb_hid_write:err %d\n", err);

	return count;
}

static ssize_t speed_show(struct device *dev, struct device_attribute *attr,char *buf)
{

	unsigned char data= 0;
	int err = 0;
	err = asus_usb_hid_read(&data,ASUS_GAMEPAD_GET_SPEED);
	if (err < 0)
		pr_err("[GAMEPAD_II] speed_show:err %d\n", err);

	return snprintf(buf, PAGE_SIZE,"%d\n", data);
}

static DEVICE_ATTR(red_pwm, 0664, red_pwm_show, red_pwm_store);
static DEVICE_ATTR(green_pwm, 0664, green_pwm_show, green_pwm_store);
static DEVICE_ATTR(blue_pwm, 0664, blue_pwm_show, blue_pwm_store);
static DEVICE_ATTR(mode, 0664, mode_show, mode_store);
static DEVICE_ATTR(apply, 0664, apply_show, apply_store);
static DEVICE_ATTR(fw_ver, 0664, fw_ver_show, NULL);
static DEVICE_ATTR(fw_mode, 0664, fw_mode_show, NULL);
static DEVICE_ATTR(frame, 0664, frame_show, frame_store);
static DEVICE_ATTR(mode2, 0664, mode2_show, mode2_store);
static DEVICE_ATTR(led_on, 0664, led_on_show, led_on_store);
static DEVICE_ATTR(speed, 0664, speed_show, speed_store);



static struct attribute *pwm_attrs[] = {
	&dev_attr_red_pwm.attr,
	&dev_attr_green_pwm.attr,
	&dev_attr_blue_pwm.attr,
	&dev_attr_mode.attr,
	&dev_attr_apply.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_fw_mode.attr,
	&dev_attr_frame.attr,
	&dev_attr_mode2.attr,
	&dev_attr_led_on.attr,
	&dev_attr_speed.attr,
	NULL
};

static const struct attribute_group pwm_attr_group = {
	.attrs = pwm_attrs,
};


static void aura_sync_set(struct led_classdev *led,
			      enum led_brightness brightness)
{
	pr_info("[GAMEPAD_II] aura_sync_set : %d.\n", brightness);
}

static enum led_brightness aura_sync_get(struct led_classdev *led_cdev)
{
	struct gamepad_drvdata *data;

	pr_info("[GAMEPAD_II] aura_sync_get.\n");
	data = container_of(led_cdev, struct gamepad_drvdata, led);

	return data->led.brightness;
}

static int aura_sync_register(struct device *dev, struct gamepad_drvdata *data)
{
	data->led.name = "aura_gamepad";

	data->led.brightness = LED_OFF;
	data->led.max_brightness = LED_HALF;
	data->led.default_trigger = "none";
	data->led.brightness_set = aura_sync_set;
	data->led.brightness_get = aura_sync_get;

	return led_classdev_register(dev, &data->led);
}
static void aura_sync_unregister(struct gamepad_drvdata *data)
{
	led_classdev_unregister(&data->led);
}


#ifdef CONFIG_PM
static int gamepad_usb_resume(struct hid_device *hdev)
{
	return 0;
}

static int gamepad_usb_suspend(struct hid_device *hdev, pm_message_t message)
{
	return 0;
}
#endif /* CONFIG_PM */

static int gamepad_usb_raw_event(struct hid_device *hdev,
		struct hid_report *report, u8 *data, int size)
{
	return 0;
}

static int gamepad_usb_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret = 0;
	unsigned int cmask = HID_CONNECT_DEFAULT;
	struct gamepad_drvdata *drvdata;

	pr_info("[GAMEPAD_II] hid->name : %s\n", hdev->name);
	pr_info("[GAMEPAD_II] hid->vendor  : 0x%x\n", hdev->vendor);
	pr_info("[GAMEPAD_II] hid->product : 0x%x\n", hdev->product);
	ASUSEvtlog("[GAMEPAD_II] GamePad connect\n");

	drvdata = devm_kzalloc(&hdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (drvdata == NULL) {
		hid_err(hdev, "[GAMEPAD_II] Can't alloc drvdata\n");
		return -ENOMEM;
	}
	hid_set_drvdata(hdev, drvdata);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "[GAMEPAD_II] parse failed\n");
		goto err_free;
	}

	ret = hid_hw_start(hdev, cmask);
	if (ret) {
		hid_err(hdev, "[GAMEPAD_II] hw start failed\n");
		goto err_free;
	}

	gamepad_hidraw = hdev->hidraw;
		
	// Register sys class  
	ret = aura_sync_register(&hdev->dev, drvdata);
	if (ret) {
		hid_err(hdev, "[GAMEPAD_II] aura_sync_register failed\n");
		goto err_free;
	}
	ret = sysfs_create_group(&drvdata->led.dev->kobj, &pwm_attr_group);
	if (ret)
		goto unregister;

	g_red=-1;
    g_green=-1;
   	g_blue=-1;
    g_mode=-1;
    g_speed=-1;
    g_led_on=-1;

	return 0;

unregister:
	aura_sync_unregister(drvdata);
err_free:
	pr_err("[GAMEPAD_II] gamepad_usb_probe fail.\n");
	hid_hw_stop(hdev);
	return ret;
}

static void gamepad_usb_remove(struct hid_device *hdev)
{
	struct gamepad_drvdata *drvdata = dev_get_drvdata(&hdev->dev);;
	pr_info("[GAMEPAD_II] gamepad_usb_remove .\n");
	ASUSEvtlog("[GAMEPAD_II] GamePad disconnect!!!\n");

	sysfs_remove_group(&drvdata->led.dev->kobj, &pwm_attr_group);
	aura_sync_unregister(drvdata);
	gamepad_hidraw = NULL;
	hid_hw_stop(hdev);
}

static struct hid_device_id gamepad_idtable[] = {
	{ HID_USB_DEVICE(0x0B05, 0x7900),
		.driver_data = 0 },
	{ HID_USB_DEVICE(0x02E2C, 0x002A),
		.driver_data = 0 },
	{ }
};
MODULE_DEVICE_TABLE(hid, gamepad_idtable);

static struct hid_driver gamepad_hid_driver = {
	.name		= "gamepad_hid_2",
	.id_table		= gamepad_idtable,
	.probe			= gamepad_usb_probe,
	.remove			= gamepad_usb_remove,
	.raw_event		= gamepad_usb_raw_event,
#ifdef CONFIG_PM
	.suspend          = gamepad_usb_suspend,
	.resume			= gamepad_usb_resume,
#endif
};

static int __init gamepad_usb_init(void)
{
	pr_info("[GAMEPAD_II] gamepad_usb_init\n");

	return hid_register_driver(&gamepad_hid_driver);
}

static void __exit gamepad_usb_exit(void)
{
	pr_info("[GAMEPAD_II] gamepad_usb_exit\n");

	hid_unregister_driver(&gamepad_hid_driver);
}

module_init(gamepad_usb_init);
module_exit(gamepad_usb_exit);


MODULE_AUTHOR("ASUS Lotta Lu");
MODULE_DESCRIPTION("GamePad II HID Interface");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("asus:gamepad II hid");
