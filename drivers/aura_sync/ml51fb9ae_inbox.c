/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
 * Edit by ASUS Deeo, deeo_ho@asus.com
 * V5
 */

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
#include "nuvoton_ml51.h"

#include <linux/hwmon.h>
#include <linux/hwmon-vid.h>
#include <linux/hwmon-sysfs.h>
#include <linux/mutex.h>
#include <linux/usb.h>
//#include <linux/msm_drm_notify.h>


#define RGB_MAX 21   //for rainbow color setting
int mode2_state=0;
int apply_state=0;
static struct ml51_platform_data *g_pdata;

static u32 g_red;
static u32 g_green;
static u32 g_blue;
static u32 g_mode;
static u32 g_speed;
static u32 g_led_on;
static u32 g_led2_on;



/*static int i2c_read_bytes(struct i2c_client *client, char *write_buf, int writelen, char *read_buf, int readlen)
{
	struct i2c_msg msgs[2];
	int ret=-1;

	//send register address
	msgs[0].flags = !I2C_M_RD;	//write
	msgs[0].addr = client->addr;
	msgs[0].len = writelen;
	msgs[0].buf = write_buf;
	
	//read data
	msgs[1].flags = I2C_M_RD;		//read
	msgs[1].addr = client->addr;
	msgs[1].len = readlen;
	msgs[1].buf = read_buf;

	ret = i2c_transfer(client->adapter,msgs, 2);
	return ret;
}*/

static int i2c_write_bytes(struct i2c_client *client, char *write_buf, int writelen)
{
	struct i2c_msg msg;
	int ret=-1;

	msg.flags = !I2C_M_RD;		//write
	msg.addr = client->addr;
	msg.len = writelen;
	msg.buf = write_buf; 

	ret = i2c_transfer(client->adapter,&msg, 1);
	return ret;
} 

static int ml51_read_bytes(struct i2c_client *client, short addr, char *data)
{
	int err = 0;
	unsigned char buf[16] = {0};
	struct i2c_msg msgs;
	buf[0] = (addr >> 8) & 0xFF;
	buf[1] = addr & 0xFF;

	pr_debug("[AURA_ML51_INBOX] ml51_read_bytes : buf[0] : 0x%x, buf[1] : 0x%x\n", buf[0], buf[1]);
	err = i2c_write_bytes(client, buf, 2);
	if (err !=1)
		printk("[AURA_ML51_INBOX] i2c_write_bytes addr: 0x%x err:%d\n", addr,err);

	/*buf[0] = 0xAA;
	err = i2c_read_bytes(client, buf, 1, data, 1);	//send read command
	if (err != 2)
		printk("[AURA_ML51_INBOX] i2c_read_bytes:err %d\n", err);*/

	msleep(1);//wait for ic
	//read data
	msgs.flags = I2C_M_RD;		//read
	msgs.addr = client->addr;
	msgs.len = 1;
	msgs.buf = data;
	err = i2c_transfer(client->adapter,&msgs, 1);

	return err;
}

static int ml51_read_words(struct i2c_client *client, short addr, char *data)
{
	int err = 0;
	unsigned char buf[16] = {0};
	struct i2c_msg msgs;

	buf[0] = (addr >> 8) & 0xFF;
	buf[1] = addr & 0xFF;


	pr_debug("[AURA_ML51_INBOX] ml51_read_words : buf[0] : 0x%x, buf[1] : 0x%x\n", buf[0], buf[1]);
	err = i2c_write_bytes(client, buf, 2);
	if (err !=1)
		printk("[AURA_ML51_INBOX] i2c_write_bytes addr:0x%x, err:%d\n", addr,err);
	msleep(1);
	/*buf[0] = 0xAA;
	err = i2c_read_bytes(client, buf, 1, data, 2);	//send read command
	if (err != 2)
		printk("[AURA_ML51_INBOX] i2c_read_bytes:err %d\n", err);*/
	//read data
	msgs.flags = I2C_M_RD;		//read
	msgs.addr = client->addr;
	msgs.len = 2;
	msgs.buf = data;
	err = i2c_transfer(client->adapter,&msgs, 1);

	return err;
}


static int ml51_write_bytes(struct i2c_client *client, short addr, char value)
{
	int err = 0;
	unsigned char buf[16] = {0};

	buf[0] = (addr >> 8) & 0xFF;
	buf[1] = addr & 0xFF;
	buf[2] = value;

	//printk("[AURA_ML51_INBOX] ml51_write_bytes : buf[0] : 0x%x, buf[1] : 0x%x, value : 0x%x\n", buf[0], buf[1], value);
	err = i2c_write_bytes(client, buf, 3);
	if (err !=1)
		printk("[AURA_ML51_INBOX] i2c_write_bytes addr:0x%x, err:%d\n", addr,err);


	return err;
}

#if FAN_ADD

static ssize_t fan_on(struct i2c_client *client)
{
	int err = 0;
	pr_debug("[AURA_ML51_INBOX] %s\n",__func__);
	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_write_bytes(client, 0x6023, 1);
	if (err !=1){
		printk("[INBOX FAN] ml51_write_bytes:0x6023 err %d\n", err);
		mutex_unlock(&g_pdata->ml51_mutex);
		return -1;
	}
	mutex_unlock(&g_pdata->ml51_mutex);
	return 0;
}

static ssize_t fan_off(struct i2c_client *client)
{
	int err = 0;
	pr_debug("[AURA_ML51_INBOX] %s\n",__func__);
	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_write_bytes(client, 0x6023, 0);
	if (err !=1){
		printk("[INBOX FAN] ml51_write_bytes:0x6023 err %d\n", err);
		mutex_unlock(&g_pdata->ml51_mutex);
		return -1;
	}
	mutex_unlock(&g_pdata->ml51_mutex);
	return 0;
}

static unsigned char get_fan_vdd(struct i2c_client *client)
{
	unsigned char data[4] = {0};
	int err = 0;
	pr_debug("[AURA_ML51_INBOX] %s\n",__func__);
	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_read_bytes(client, 0x6023, data);
	if (err != 1)
	{
		printk("[INBOX FAN] get_fan_vdd:0x6023 err %d\n", err);
		mutex_unlock(&g_pdata->ml51_mutex);
		return -1;
	}
	mutex_unlock(&g_pdata->ml51_mutex);
	return data[0];
}


static ssize_t
show_vdd(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct i2c_client *client = to_i2c_client(dev);
	unsigned char data[4] = {0};
	int err = 0;
	printk("[INBOX FAN] %s.\n",__func__);
	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_read_bytes(client, 0x6023, data);
	if (err != 1)
		printk("[INBOX FAN] show_vdd:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);

}

static ssize_t
store_vdd(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	u32 reg_val;
	ssize_t ret;

	printk("[INBOX FAN] %s.\n",__func__);

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;
	if(reg_val==0x1){
		fan_on(client);
	}else{
		fan_off(client);
	}
	return count;
}

//+++inbox user fan
static ssize_t inbox_user_fan(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int num = 99;

	struct i2c_client *client = to_i2c_client(dev);
	u32  tmp;
	int err = 0;
	mutex_lock(&g_pdata->update_lock);
	//printk("[INBOX FAN] -----%s\n",client->name);
	sscanf(buf, "%d", &num);
	printk("[INBOX FAN] %s : num=%d\n",__func__, num);
	/*if ((num != 0) && get_fan_vdd(client)!= 1) {
		printk("[INBOX FAN] %s : enable fan\n",__func__);
		err = fan_on(client);
		if(!err){
			printk("[INBOX FAN]: %s :fan on\n",__func__);
		}else{
			printk("[INBOX FAN]: %s :fan on failed, err=%d\n",__func__,err);
			return err;
		}
		msleep(30); //Wait 30ms for IC power on
	}
	else {
		printk("[INBOX FAN] %s :enable_pin is already set \n",__func__);
	}*/
	switch (num) {
		case 0:
			//printk("[INBOX FAN] %s : fan_type 0: close +++\n",__func__);
			tmp = 0;
			mutex_lock(&g_pdata->ml51_mutex);
			printk("[INBOX FAN] fan_type 0  client->addr : 0x%x, fan pwm  tmp 0x%x\n",client->addr, tmp);
			err = ml51_write_bytes(client, 0x6001 ,tmp);
			if (err !=1)
				printk("[INBOX FAN] ml51_write_bytes:err %d\n", err);
			mutex_unlock(&g_pdata->ml51_mutex);

			err = fan_off(client);
			if(!err){
				printk("[INBOX FAN]: %s :fan off\n",__func__);
			}
			break;
		case 1:
			//printk("[INBOX FAN] %s : fan_type 1: low +++\n",__func__);
			tmp = 0x7E;
			mutex_lock(&g_pdata->ml51_mutex);
			printk("[INBOX FAN] fan_type 1 client->addr : 0x%x, fan pwm  tmp 0x%x\n",client->addr, tmp);
			err = ml51_write_bytes(client, 0x6001 ,tmp);
			if (err !=1)
				printk("[INBOX FAN] ml51_write_bytes:err %d\n", err);
			mutex_unlock(&g_pdata->ml51_mutex);
			msleep(30);
			if (get_fan_vdd(client)!= 1) {
				printk("[INBOX FAN] %s : enable fan\n",__func__);
				err = fan_on(client);
				if(!err){
					printk("[INBOX FAN]: %s :fan on\n",__func__);
				}else{
					printk("[INBOX FAN]: %s :fan on failed, err=%d\n",__func__,err);
					mutex_unlock(&g_pdata->update_lock);
					return size;
				}
			}
			else {
				printk("[INBOX FAN] %s : enable_pin is already set \n",__func__);
			}
			//the first level
			break;
		case 2:
			//printk("[INBOX FAN] %s : fan_type 2: medium +++\n",__func__);
			tmp = 0x98;
			mutex_lock(&g_pdata->ml51_mutex);
			printk("[INBOX FAN] fan_type 2 client->addr : 0x%x, fan pwm  tmp 0x%x\n",client->addr, tmp);
			err = ml51_write_bytes(client, 0x6001 ,tmp);
			if (err !=1)
				printk("[INBOX FAN] ml51_write_bytes:err %d\n", err);
			mutex_unlock(&g_pdata->ml51_mutex);
			msleep(30);
			if (get_fan_vdd(client)!= 1) {
				printk("[INBOX FAN] %s : enable fan\n",__func__);
				err = fan_on(client);
				if(!err){
					printk("[INBOX FAN]: %s :fan on\n",__func__);
				}else{
					printk("[INBOX FAN]: %s :fan on failed, err=%d\n",__func__,err);
					mutex_unlock(&g_pdata->update_lock);
					return size;
				}
			}
			else {
				printk("[INBOX FAN] %s : enable_pin is already set \n",__func__);
			}

			//the second level
			break;
		case 3:
			//printk("[INBOX FAN] %s : fan_type 3: high +++\n",__func__);
			tmp = 0xA7;
			mutex_lock(&g_pdata->ml51_mutex);
			printk("[INBOX FAN] fan_type 3 client->addr : 0x%x, fan pwm  tmp 0x%x\n",client->addr, tmp);
			err = ml51_write_bytes(client, 0x6001 ,tmp);
			if (err !=1)
				printk("[INBOX FAN] ml51_write_bytes:err %d\n", err);
			mutex_unlock(&g_pdata->ml51_mutex);
			msleep(30);
			if (get_fan_vdd(client)!= 1) {
				printk("[INBOX FAN] %s : enable fan\n",__func__);
				err = fan_on(client);
				if(!err){
					printk("[INBOX FAN]: %s :fan on\n",__func__);
				}else{
					printk("[INBOX FAN]: %s :fan on failed, err=%d\n",__func__,err);
					mutex_unlock(&g_pdata->update_lock);
					return size;
				}
			}
			else {
				printk("[INBOX FAN] %s : enable_pin is already set \n",__func__);
			}
			//the third level
			break;
		case 4:
			//printk("[INBOX FAN] %s : fan_type 4: turbo +++\n",__func__);
			tmp = 0xDA;
			mutex_lock(&g_pdata->ml51_mutex);
			printk("[INBOX FAN]fan_type 4  client->addr : 0x%x, fan pwm  tmp 0x%x\n",client->addr, tmp);
			err = ml51_write_bytes(client, 0x6001 ,tmp);
			if (err !=1)
				printk("[INBOX FAN] ml51_write_bytes:err %d\n", err);
			mutex_unlock(&g_pdata->ml51_mutex);
			msleep(30);
			if (get_fan_vdd(client)!= 1) {
				printk("[INBOX FAN] %s : enable fan\n",__func__);
				err = fan_on(client);
				if(!err){
					printk("[INBOX FAN]: %s :fan on\n",__func__);
				}else{
					printk("[INBOX FAN]: %s :fan on failed, err=%d\n",__func__,err);
					mutex_unlock(&g_pdata->update_lock);
					return size;
				}
			}
			else {
				printk("[INBOX FAN] %s : enable_pin is already set \n",__func__);
			}
			//the forth level
			break;
		default:
			printk("[INBOX FAN] %s :mode isn't 0-4, unsupport\n",__func__);

	}
	msleep(500); //Wait 0.5s
	mutex_unlock(&g_pdata->update_lock);
	return size;
}

//+++inbox thermal fan
static ssize_t inbox_thermal_fan(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{

	int num = 99;

	struct i2c_client *client = to_i2c_client(dev);
	u32 tmp;
	int err = 0;

	mutex_lock(&g_pdata->update_lock);
	//printk("[INBOX FAN] -----%s\n",client->name);
	sscanf(buf, "%d", &num);
	printk("[INBOX FAN] %s : num=%d\n",__func__, num);
	/*if ((num != 0) && get_fan_vdd(client)!= 1) {
		printk("[INBOX FAN] %s : enable fan\n",__func__);
		err = fan_on(client);
		if(!err){
			printk("[INBOX FAN]: %s :fan on\n",__func__);
		}else{
			printk("[INBOX FAN]: %s :fan on failed, err=%d\n",__func__,err);
			return err;
		}
		msleep(30); //Wait 30ms for IC power on
	}
	else {
		printk("[INBOX FAN] %s : enable_pin is already set \n",__func__);
	}*/
	switch (num) {
		case 0:
		   // printk("[INBOX FAN] %s : thermal_type 0: default 1 (low) +++\n",__func__);
			tmp = 0x0;
			mutex_lock(&g_pdata->ml51_mutex);
			printk("[INBOX FAN] thermal_type 0 client->addr : 0x%x, fan pwm  tmp 0x%x\n",client->addr, tmp);
			err = ml51_write_bytes(client, 0x6001 ,tmp);
			if (err !=1)
				printk("[INBOX FAN] ml51_write_bytes:err %d\n", err);
			mutex_unlock(&g_pdata->ml51_mutex);
						err = fan_off(client);
			if(!err){
				printk("[INBOX FAN]: %s :fan off\n",__func__);
			}
			break;
		case 1:
			//printk("[INBOX FAN] %s : fan_type 1: low +++\n",__func__);
			tmp = 0x7E;
			mutex_lock(&g_pdata->ml51_mutex);
			printk("[INBOX FAN] thermal_type 1 client->addr : 0x%x, fan pwm  tmp 0x%x\n",client->addr, tmp);
			err = ml51_write_bytes(client, 0x6001 ,tmp);
			if (err !=1)
				printk("[INBOX FAN] ml51_write_bytes:err %d\n", err);
			mutex_unlock(&g_pdata->ml51_mutex);
			msleep(30);
			if (get_fan_vdd(client)!= 1) {
				printk("[INBOX FAN] %s : enable fan\n",__func__);
				err = fan_on(client);
				if(!err){
					printk("[INBOX FAN]: %s :fan on\n",__func__);
				}else{
					printk("[INBOX FAN]: %s :fan on failed, err=%d\n",__func__,err);
					mutex_unlock(&g_pdata->update_lock);
					return size;
				}
			}
			else {
				printk("[INBOX FAN] %s : enable_pin is already set \n",__func__);
			}
			//the first level
			break;
		case 2:
			//printk("[INBOX FAN] %s : fan_type 2: medium +++\n",__func__);
			tmp = 0x98;
			mutex_lock(&g_pdata->ml51_mutex);
			printk("[INBOX FAN] thermal_type 2 client->addr : 0x%x, fan pwm  tmp 0x%x\n",client->addr, tmp);
			err = ml51_write_bytes(client, 0x6001 ,tmp);
			if (err !=1)
				printk("[INBOX FAN] ml51_write_bytes:err %d\n", err);
			mutex_unlock(&g_pdata->ml51_mutex);
			msleep(30);
			if (get_fan_vdd(client)!= 1) {
				printk("[INBOX FAN] %s : enable fan\n",__func__);
				err = fan_on(client);
				if(!err){
					printk("[INBOX FAN]: %s :fan on\n",__func__);
				}else{
					printk("[INBOX FAN]: %s :fan on failed, err=%d\n",__func__,err);
					mutex_unlock(&g_pdata->update_lock);
					return size;
				}
			}
			else {
				printk("[INBOX FAN] %s : enable_pin is already set \n",__func__);
			}
			//the second level
			break;
		case 3:
			//printk("[INBOX FAN] %s : fan_type 3: high +++\n",__func__);
			tmp = 0xA7;
			mutex_lock(&g_pdata->ml51_mutex);
			printk("[INBOX FAN] thermal_type 3  client->addr : 0x%x, fan pwm  tmp 0x%x\n",client->addr, tmp);
			err = ml51_write_bytes(client, 0x6001 ,tmp);
			if (err !=1)
				printk("[INBOX FAN] ml51_write_bytes:err %d\n", err);
			mutex_unlock(&g_pdata->ml51_mutex);
			msleep(30);
			if (get_fan_vdd(client)!= 1) {
				printk("[INBOX FAN] %s : enable fan\n",__func__);
				err = fan_on(client);
				if(!err){
					printk("[INBOX FAN]: %s :fan on\n",__func__);
				}else{
					printk("[INBOX FAN]: %s :fan on failed, err=%d\n",__func__,err);
					mutex_unlock(&g_pdata->update_lock);
					return size;
				}
			}
			else {
				printk("[INBOX FAN] %s : enable_pin is already set \n",__func__);
			}
			//the third level
			break;
		case 4:
			//printk("[INBOX FAN] %s : fan_type 4: turbo +++\n",__func__);
			tmp = 0xDA;
			mutex_lock(&g_pdata->ml51_mutex);
			printk("[INBOX FAN] thermal_type 4 client->addr : 0x%x, fan pwm  tmp 0x%x\n",client->addr, tmp);
			err = ml51_write_bytes(client, 0x6001 ,tmp);
			if (err !=1)
				printk("[INBOX FAN] ml51_write_bytes:err %d\n", err);
			mutex_unlock(&g_pdata->ml51_mutex);
			msleep(30);
			if (get_fan_vdd(client)!= 1) {
				printk("[INBOX FAN] %s : enable fan\n",__func__);
				err = fan_on(client);
				if(!err){
					printk("[INBOX FAN]: %s :fan on\n",__func__);
				}else{
					printk("[INBOX FAN]: %s :fan on failed, err=%d\n",__func__,err);
					mutex_unlock(&g_pdata->update_lock);
					return size;
				}
			}
			else {
				printk("[INBOX FAN] %s : enable_pin is already set \n",__func__);
			}
			//the forth level
			break;
		default:
			printk("[INBOX FAN] %s :mode isn't 0-4, unsupport\n",__func__);
	}
	msleep(500); //Wait 0.5s
	mutex_unlock(&g_pdata->update_lock);
	return size;
}
//---inbox thermal fan


static ssize_t
show_fan(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char data[4] = {0};
	int err = 0;
	long rpm = 0;
	int raw_data = 0;

	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_read_words(client, 0x6002, data);   //RPM=24M/(data*512)
	if (err != 1)
		printk("[INBOX FAN] show_fan: 0x6002 err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	raw_data = data[0]*256+data[1];
	printk("[INBOX FAN] show_fan: raw_data = 0x%4x data[0]=0x%2x data[1]=0x%2x\n", raw_data,data[1],data[0]);
	if(raw_data != 0){
		rpm = (24 * 1000000) >> 9;
		do_div(rpm,raw_data);
		return snprintf(buf, PAGE_SIZE,"%ld\n", rpm*30);
	}else{
		return snprintf(buf, PAGE_SIZE,"%d\n", raw_data);
	}
}

static ssize_t show_reg(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	struct i2c_client *client = to_i2c_client(dev);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_read_bytes(client, 0x6001, data); //PWM
	if (err != 1)
		printk("[INBOX FAN] show_reg:0x6001 err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

//+++pwm_store
static ssize_t pwm_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned char  reg_value = 0;
	int err = 0;
	struct i2c_client *client = to_i2c_client(dev);

	//sscanf(buf, "%x", &reg_value);
	kstrtou8(buf, 0, &reg_value);
	printk("[INBOX FAN] pwm_store : pwm set to %x\n", reg_value);
	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_write_bytes(client, 0x6001 ,reg_value);
	if (err !=1)
		printk("[INBOX FAN] ml51_write_bytes:err %d\n", err);
	mutex_unlock(&g_pdata->ml51_mutex);

	return size;
}
//---pwm_store

//+++write to reg
static ssize_t write_to_reg(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int reg_addr = 0, reg_value = 0;
	int err = 0;
	struct i2c_client *client = to_i2c_client(dev);

	sscanf(buf, "%x %x", &reg_addr, &reg_value);
	printk("[INBOX FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);

	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_write_bytes(client, reg_addr, reg_value);
	if (err !=1){
		printk("[INBOX FAN] ml51_write_bytes:err %d\n", err);
		//return size;
	}
	mutex_unlock(&g_pdata->ml51_mutex);


	return size;
}
//---write to reg



#define NOT_USED			-1
#define FAN_INPUT     0
//for the node
#define SENSOR_ATTR_FAN				  		\
	SENSOR_ATTR_2(VDD, S_IRUGO|S_IWUSR, show_vdd, store_vdd, NOT_USED, 0), \
	SENSOR_ATTR_2(debug_write_to_reg, S_IWUSR, NULL, write_to_reg, NOT_USED, 0),\
	SENSOR_ATTR_2(inbox_user_type, S_IWUSR, NULL, inbox_user_fan, NOT_USED, 0), \
	SENSOR_ATTR_2(inbox_thermal_type, S_IWUSR, NULL, inbox_thermal_fan, NOT_USED, 0), \
	SENSOR_ATTR_2(PWM, S_IRUGO|S_IWUSR , show_reg, pwm_store, NOT_USED, 0), \
	SENSOR_ATTR_2(RPM, S_IRUGO, show_fan, NULL, FAN_INPUT, 0)


static struct sensor_device_attribute_2 fan_attr[] = {
	SENSOR_ATTR_FAN,
};

#endif  //end if of the FAN_ADD

static ssize_t red1_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;

	//printk("[AURA_ML51_INBOX] red1_pwm_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->RED_MAX), 255);

	mutex_lock(&g_pdata->ml51_mutex);
	//printk("[AURA_ML51_INBOX] red1_pwm_store,client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);
	//printk("[AURA_ML51_INBOX] red1 tmp %d\n", tmp);
	err = ml51_write_bytes(client, 0x8013, tmp);
	if (err !=1)
		printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	return count;
}

static ssize_t red1_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_read_bytes(client, 0x8013,data);
	if (err != 1)
		printk("[AURA_ML51_INBOX] red1_pwm_show:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t green1_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;

	//printk("[AURA_ML51_INBOX] green1_pwm_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->GREEN_MAX), 255);

	mutex_lock(&g_pdata->ml51_mutex);
	//printk("[AURA_ML51_INBOX] green1_pwm_store,client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);
	//printk("[AURA_ML51_INBOX] green1 tmp %d\n", tmp);
	err = ml51_write_bytes(client, 0x8014,tmp);
	if (err !=1)
		printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	return count;
}

static ssize_t green1_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_read_bytes(client, 0x8014, data);
	if (err != 1)
		printk("[AURA_ML51_INBOX] green1_pwm_show:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t blue1_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;

	//printk("[AURA_ML51_INBOX] blue1_pwm_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->BLUE_MAX), 255);

	mutex_lock(&g_pdata->ml51_mutex);
	//printk("[AURA_ML51_INBOX] blue1_pwm_store,client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);
	//printk("[AURA_ML51_INBOX] blue1 tmp %d\n", tmp);
	err = ml51_write_bytes(client, 0x8015, tmp);
	if (err !=1)
		printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	return count;
}

static ssize_t blue1_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_read_bytes(client, 0x8015, data);
	if (err != 1)
		printk("[AURA_ML51_INBOX] blue1_pwm_show:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}




static ssize_t red_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;


	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;
	pr_debug("[AURA_ML51_INBOX] %s reg_val=%d.\n",__func__,reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->RED_MAX), 255);

	mutex_lock(&g_pdata->ml51_mutex);
	//printk("[AURA_ML51_INBOX] red_pwm_store first pwm,client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);
	//printk("[AURA_ML51_INBOX] red tmp %d\n", tmp);
	g_red=tmp;
	err = ml51_write_bytes(client, 0x8010, tmp);
	if (err !=1)
		printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);

	//printk("[AURA_ML51_INBOX] red_pwm_store second pwm,client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);
	err = ml51_write_bytes(client, 0x8013, tmp);
	if (err !=1)
		printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);

	return count;
}

static ssize_t red_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_read_bytes(client, 0x8010, data);
	if (err != 1)
		printk("[AURA_ML51_INBOX] red_pwm_show:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t green_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;

	//printk("[AURA_ML51_INBOX] green_pwm_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;
	pr_debug("[AURA_ML51_INBOX] %s reg_val=%d.\n",__func__,reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->GREEN_MAX), 255);

	mutex_lock(&g_pdata->ml51_mutex);
	//printk("[AURA_ML51_INBOX] green_pwm_store first pwm,client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);
	//printk("[AURA_ML51_INBOX] green tmp %d\n", tmp);
	g_green=tmp;
	err = ml51_write_bytes(client, 0x8011, tmp);
	if (err !=1)
		printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);

	//printk("[AURA_ML51_INBOX] green_pwm_store second pwm,client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);
	err = ml51_write_bytes(client, 0x8014,tmp);
	if (err !=1)
		printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	return count;
}

static ssize_t green_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_read_bytes(client, 0x8011, data);
	if (err != 1)
		printk("[AURA_ML51_INBOX] green_pwm_show:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t blue_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;

	//printk("[AURA_ML51_INBOX] blue_pwm_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;
	pr_debug("[AURA_ML51_INBOX] %s reg_val=%d.\n",__func__,reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->BLUE_MAX), 255);

	mutex_lock(&g_pdata->ml51_mutex);
	//printk("[AURA_ML51_INBOX] blue_pwm_store first pwm,client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);
	//printk("[AURA_ML51_INBOX] blue tmp %d\n", tmp);
	g_blue=tmp;
	err = ml51_write_bytes(client, 0x8012, tmp);
	if (err !=1)
		printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);

	//printk("[AURA_ML51_INBOX] blue_pwm_store second pwm,client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);

	err = ml51_write_bytes(client, 0x8015, tmp);
	if (err !=1)
		printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	return count;
}

static ssize_t blue_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_read_bytes(client, 0x8012, data);
	if (err != 1)
		printk("[AURA_ML51_INBOX] blue_pwm_show:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t led_color_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	//struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	int reg[3] = {0};
	int err = 0;

	printk("[AURA_ML51_INBOX] led_color_store.\n");
	sscanf(buf, "%x %x %x", &reg[0], &reg[1],&reg[2]);

	if(((reg[0]&0xF0) != 0xD0)||((reg[0]&0x0F) < 0 || (reg[0]&0x0F) > 0xC )){
		printk("[AURA_ML51_INBOX] led_color_store: error input byte[0]=%d\n", reg[0]);
		return count;
	}
	if(reg[1] < 0 || reg[1] > 2){
		printk("[AURA_ML51_INBOX] led_color_store: error input byte[1]=%d\n", reg[1]);
		return count;
	}
	if(reg[2] < 0 || reg[2] > 255){
		printk("[AURA_ML51_INBOX] led_color_store: error input byte[2]=%d\n", reg[2]);
		return count;
	}

	mutex_lock(&g_pdata->ml51_mutex);
	printk("[AURA_ML51_INBOX] led_color_store,client->addr:0x%x,reg_value:0x%04x,reg[2]:0x%x.\n", client->addr,reg[0]*256+reg[1],reg[2]);
	err = ml51_write_bytes(client, reg[0]*256+reg[1], reg[2]);
	if (err !=1)
		printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	return count;
}

static ssize_t led_color_show(struct device *dev, struct device_attribute *attr,char *buf)
{

	return snprintf(buf, PAGE_SIZE,"%s\n", "not define led_color_show");
}



static ssize_t mode2_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	//struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	unsigned char rgb[RGB_MAX] = {0};
	//unsigned char tokenized_data[22] = {0};
	unsigned char rainbow_mode = 0;
	unsigned char mode2 = 0;
	int err = 0;
	int i = 0,rgb_num=0;//rgb_num_t=0;
	long rgb_tmp = 0;
	unsigned char reg[3] = {0};
	int n = 0;
	int ntokens = 0;
	const char *cp = buf;
	const char *buf_tmp;
	mode2_state=0;

	//printk("[AURA_ML51_INBOX] mode2_store.\n");
	sscanf(buf, "%d", &mode2);

	while ((cp = strpbrk(cp + 1, ",")))
	{
		ntokens++;  //the number of ","
		//printk("[AURA_ML51_INBOX] mode2_store %s.\n",cp);
	}
	printk("[AURA_ML51_INBOX] mode2_store mode2=%d buf=%s ntokens=%d\n",mode2,buf,ntokens);
	if(ntokens > 6) 
	{
		printk("[AURA_ML51_INBOX] mode2_store,wrong input,too many ntokens\n");
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
		printk("[AURA_ML51_INBOX] mode2_store,wrong input,rgb_num != ntokens*3\n");
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
			break;
		case 1: //6 color rainbow
			rainbow_mode = 0x7;
			break;
		case 2: //static
			rainbow_mode = 1;
			break;
		case 3: //breath at the same time
			rainbow_mode = 0x2;
			break;
		case 4: //breath at different time
			rainbow_mode = 0x11;
			break;
		case 5: //breath only one led
			rainbow_mode = 0x10;
			break;
		case 6: //commet
			rainbow_mode = 0x12;
			break;
		case 8: //commet in different direction direction
			rainbow_mode = 0x13;
			break;
		case 7: //flash and dash
			rainbow_mode = 0x14;
			break;
		case 9: //flash and dash in different direction
			rainbow_mode = 0x15;
			break;
		case 10: //6 color in different direction
			rainbow_mode = 0x8;
			break;
		case 11: //6 color in different direction
			rainbow_mode = 0xF;
			break;
	}

	switch(rainbow_mode){
		case 0:  //mode 0
			mutex_lock(&g_pdata->ml51_mutex);
			err = ml51_write_bytes(client, 0x8021, rainbow_mode);
			if (err !=1){
				printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ml51_mutex);
				mode2_state=-1;
				return count;
			}
			mutex_unlock(&g_pdata->ml51_mutex);
			break;
		case 0x1: //static
			if(ntokens != 2){
				printk("[AURA_ML51_INBOX] mode2_store,wrong input.\n");
				mode2_state=-1;
				return count;
			}
			//sscanf(buf, "%x, %x %x %x,%x %x %x", 
				//&rainbow_mode,&rgb[0],&rgb[1],&rgb[2],&rgb[3],&rgb[4],&rgb[5]);
			//printk("[AURA_ML51_INBOX] mode2_store,static two leds. mode=0x%x,client->addr:0x%x.\n", rainbow_mode,client->addr);
			mutex_lock(&g_pdata->ml51_mutex);
			for(i=0;i<6;i++){
				err = ml51_write_bytes(client, (0x8010+i),rgb[i]);
				if (err !=1){
					mode2_state=-1;
					printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);	
				}
			}
			err = ml51_write_bytes(client, 0x8021, rainbow_mode);
			if (err !=1){
				mode2_state=-1;
				printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);
			}
			mutex_unlock(&g_pdata->ml51_mutex);
			break;
		case 0x2: //breath at the same time
			if(ntokens != 2){
				printk("[AURA_ML51_INBOX] mode2_store,wrong input.\n");
				mode2_state=-1;
				return count;
			}
			//sscanf(buf, "%x, %x %x %x,%x %x %x", 
				//&rainbow_mode,&rgb[0],&rgb[1],&rgb[2],&rgb[3],&rgb[4],&rgb[5]);
			//printk("[AURA_ML51_INBOX] mode2_store,static two leds. mode=0x%x,client->addr:0x%x.\n", rainbow_mode,client->addr);
			mutex_lock(&g_pdata->ml51_mutex);
			for(i=0;i<6;i++){
				err = ml51_write_bytes(client, (0x8010+i),rgb[i]);
				if (err !=1){
					mode2_state=-1;
					printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);	
				}
			}
			err = ml51_write_bytes(client, 0x8021, rainbow_mode);
			if (err !=1){
				mode2_state=-1;
				printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);
			}
			mutex_unlock(&g_pdata->ml51_mutex);
			break;

		case 0x7:
		case 0x8://6 colors rainbow
			if(ntokens != 6){
				printk("[AURA_ML51_INBOX] mode2_store,wrong input, ntokens wrong.\n");
				mode2_state=-1;
				return count;
			}
			//printk("[AURA_ML51_INBOX] mode2_store,buf=%s\n",buf);
			//sscanf(buf, "%x, %x %x %x,%x %x %x,%x %x %x,%x %x %x,%x %x %x,%x %x %x,%x %x %x", 
				//&rainbow_mode,&rgb[0],&rgb[1],&rgb[2],&rgb[3],&rgb[4],&rgb[5],&rgb[6],&rgb[7],&rgb[8],&rgb[9],&rgb[10],&rgb[11],
				//&rgb[12],&rgb[13],&rgb[14],&rgb[15],&rgb[16],&rgb[17],&rgb[18],&rgb[19],&rgb[20]);


			//printk("[AURA_ML51_INBOX] mode2_store,6 color rainbow. mode=0x%x,client->addr:0x%x.\n", rainbow_mode,client->addr);
			mutex_lock(&g_pdata->ml51_mutex);
			for(i = 0 ;i <= 5; i++){
				reg[0] = 0xD0+i;
				for(n = 0;n <= 2; n++){
					reg[1] = n;
					reg[2] = rgb[3*i+n];
					//printk("[AURA_ML51_INBOX] mode2_store,reg[0]=0x%02x reg[1]=0x%02x reg[2]=0x%02x.\n", reg[0],reg[1],reg[2]);
					err = ml51_write_bytes(client, reg[0]*256+reg[1], reg[2]);
					if (err !=1){
						mode2_state=-1;
						printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);
					}

				}
			}
			err = ml51_write_bytes(client, 0x8021, rainbow_mode);
			if (err !=1){
				mode2_state=-1;
				printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);
			}
			mutex_unlock(&g_pdata->ml51_mutex);
			break;
		case 0x10: //breath one led
		case 0xF: //breath one led
			if(ntokens != 2){
				printk("[AURA_ML51_INBOX] mode2_store,wrong input.\n");
				mode2_state=-1;
				return count;
			}
			//sscanf(buf, "%x, %x %x %x,%x %x %x", 
				//&rainbow_mode,&rgb[0],&rgb[1],&rgb[2],&rgb[3],&rgb[4],&rgb[5]);
			//printk("[AURA_ML51_INBOX] mode2_store,breath one led. mode=0x%x,client->addr:0x%x.\n", rainbow_mode,client->addr);
			mutex_lock(&g_pdata->ml51_mutex);
			for(i=0;i<6;i++){
				err = ml51_write_bytes(client, (0x8010+i),rgb[i]);
				if (err !=1){
					mode2_state=-1;
					printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);	
				}
			}
			err = ml51_write_bytes(client, 0x8021, rainbow_mode);
			if (err !=1){
				mode2_state=-1;
				printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);
			}
			mutex_unlock(&g_pdata->ml51_mutex);
			break;
		case 0x11: //breath at the different time
			if(ntokens != 2){
				printk("[AURA_ML51_INBOX] mode2_store,wrong input.\n");
				mode2_state=-1;
				return count;
			}
			//sscanf(buf, "%x, %x %x %x,%x %x %x", 
				//&rainbow_mode,&rgb[0],&rgb[1],&rgb[2],&rgb[3],&rgb[4],&rgb[5]);
			//printk("[AURA_ML51_INBOX] mode2_store,breath at the different time. mode=0x%x,client->addr:0x%x.\n", rainbow_mode,client->addr);
			mutex_lock(&g_pdata->ml51_mutex);
			for(i=0;i<6;i++){
				err = ml51_write_bytes(client, (0x8010+i),rgb[i]);
				if (err !=1){
					mode2_state=-1;
					printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);	
				}
			}
			err = ml51_write_bytes(client, 0x8021, rainbow_mode);
			if (err !=1){
				mode2_state=-1;
				printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);
			}
			mutex_unlock(&g_pdata->ml51_mutex);
			break;

		case 0x12://comet
		case 0x14://flash and dash
			if(ntokens != 2){
				printk("[AURA_ML51_INBOX] mode2_store,wrong input.\n");
				return count;
			}
			//sscanf(buf, "%x, %x %x %x", &rainbow_mode,&rgb[0],&rgb[1],&rgb[2]);
			//printk("[AURA_ML51_INBOX] mode2_store,comet or flash and dash. mode=0x%x,client->addr:0x%x.\n", rainbow_mode,client->addr);
			mutex_lock(&g_pdata->ml51_mutex);
			
			for(i = 0 ;i <= 1; i++){
				reg[0] = 0xDB+i;
				for(n = 0;n <= 2; n++){
					reg[1] = n;
					reg[2] = rgb[3*i+n];
					//printk("[AURA_ML51_INBOX] mode2_store,reg[0]=0x%02x reg[1]=0x%02x reg[2]=0x%02x.\n", reg[0],reg[1],reg[2]);
					err = ml51_write_bytes(client, reg[0]*256+reg[1], reg[2]);
					if (err !=1){
						mode2_state=-1;
						printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);
					}
				}
			}
			err = ml51_write_bytes(client, 0x8021, rainbow_mode);
			if (err !=1){
				mode2_state=-1;
				printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);
			}
			mutex_unlock(&g_pdata->ml51_mutex);
			break;
		case 0x13://comet in different direction
		case 0x15://flash and dash in different direction
			if(ntokens != 2){
				printk("[AURA_ML51_INBOX] mode2_store,wrong input.\n");
				return count;
			}
			//sscanf(buf, "%x, %x %x %x", &rainbow_mode,&rgb[0],&rgb[1],&rgb[2]);
			//printk("[AURA_ML51_INBOX] mode2_store,comet or flash and dash. mode=0x%x,client->addr:0x%x.\n", rainbow_mode,client->addr);
			mutex_lock(&g_pdata->ml51_mutex);

			for(i = 1 ;i >= 0; i--){
				reg[0] = 0xDB+(1-i);
				for(n = 0;n <= 2; n++){
					reg[1] = n;
					reg[2] = rgb[3*i+n];
					//printk("[AURA_ML51_INBOX] mode2_store 0x13 0x15,reg[0]=0x%02x reg[1]=0x%02x reg[2]=0x%02x.\n", reg[0],reg[1],reg[2]);
					err = ml51_write_bytes(client, reg[0]*256+reg[1], reg[2]);
					if (err !=1){
						mode2_state=-1;
						printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);
					}
				}
			}
			err = ml51_write_bytes(client, 0x8021, rainbow_mode);
			if (err !=1){
				mode2_state=-1;
				printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);
			}
			mutex_unlock(&g_pdata->ml51_mutex);
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

static ssize_t apply_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	u32 val;
	int err = 0;
	ssize_t ret;
	apply_state=0;
	ret = kstrtou32(buf, 10, &val);
	if (ret){
		apply_state=-1;
		return count;
	}
	mutex_lock(&g_pdata->ml51_mutex);
	if (val > 0){
		//printk("[AURA_ML51_INBOX] Send apply cmd.\n");
		printk("[AURA_ML51_INBOX] Send apply. RGB:%d %d %d, mode:%d, speed:%d, led_on:%d, led2_on:%d\n", g_red, g_green, g_blue, g_mode, g_speed, g_led_on, g_led2_on);
		err = ml51_write_bytes(client, 0x802F, 0x1);
		if (err !=1){
			apply_state=-1;
			printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);
		}
	} else
		printk("[AURA_ML51_INBOX] No send apply cmd.\n");

	mutex_unlock(&g_pdata->ml51_mutex);
	return count;
}

static ssize_t apply_show(struct device *dev, struct device_attribute *attr,char *buf)
{

	return snprintf(buf, PAGE_SIZE,"%d\n", apply_state);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	u32 val;
	int err = 0;
	ssize_t ret;
	//printk("[AURA_ML51_INBOX] mode_store.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;
	pr_debug("[AURA_ML51_INBOX] %s reg_val=%d.\n",__func__,val);
	mutex_lock(&g_pdata->ml51_mutex);
	//printk("[AURA_ML51_INBOX] client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
	g_mode=val;
	err = ml51_write_bytes(client, 0x8021, val);
	if (err !=1){
		printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);
		mutex_unlock(&g_pdata->ml51_mutex);
		return count;
	}

	platform_data->current_mode = (u8)val;
//	ASUSEvtlog("[AURA_ML51_INBOX] current_mode : %d\n", platform_data->current_mode);

	mutex_unlock(&g_pdata->ml51_mutex);
	return count;
}

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_read_bytes(client, 0x8021, data);
	if (err != 1){
		printk("[AURA_ML51_INBOX] mode_show:err %d\n", err);
		mutex_unlock(&g_pdata->ml51_mutex);
		return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
	}

	platform_data->current_mode = data[0];

	mutex_unlock(&g_pdata->ml51_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	u32 val;
	ssize_t ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	if(val>0) {
		printk("[AURA_ML51_INBOX] reset set HIGH\n");
		
		if ( gpio_is_valid(platform_data->aura_3p3_en) ) {
			gpio_set_value(platform_data->aura_3p3_en, 1);
		}else {
			printk("[AURA_ML51_INBOX] aura_3p3_en is not vaild\n");
		}
	}else {
		printk("[AURA_ML51_INBOX] reset set LOW\n");

		if ( gpio_is_valid(platform_data->aura_3p3_en) ) {
			gpio_set_value(platform_data->aura_3p3_en, 0);
		}else {
			printk("[AURA_ML51_INBOX] aura_3p3_en is not vaild\n");
		}
	}
	return count;
}

static ssize_t reset_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	int val = 0;

	if ( gpio_is_valid(platform_data->aura_3p3_en) ) {
		val = gpio_get_value(platform_data->aura_3p3_en);
		printk("[AURA_ML51_INBOX] reset pin, aura_3p3_en[%d] :0x%x\n", platform_data->aura_3p3_en, val);
		return snprintf(buf, PAGE_SIZE,"%d\n", val);
	}else {
		printk("[AURA_ML51_INBOX] reset pin,aura_3p3_en is not valid\n");
		return snprintf(buf, PAGE_SIZE,"reset pin is not valid\n");
	}
}

/*static ssize_t led_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	u32 val;
	ssize_t ret;

	ret = kstrtou32(buf, 10, &val);

	//return for now 
	return count;
	if (ret)
		return ret;

	if(val>0) {
		printk("[AURA_ML51_INBOX] LED_EN set HIGH\n");
		
		if ( gpio_is_valid(platform_data->logo_5p0_en) )
			gpio_set_value(platform_data->logo_5p0_en, 1);
	}else {
		printk("[AURA_ML51_INBOX] LED_EN set LOW\n");

		if ( gpio_is_valid(platform_data->logo_5p0_en) )
			gpio_set_value(platform_data->logo_5p0_en, 0);
	}

	return count;
}

static ssize_t led_en_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	int val = 0;
	//return for now 
	return 0;
	if ( gpio_is_valid(platform_data->logo_5p0_en) ) {
		val = gpio_get_value(platform_data->logo_5p0_en);
		return snprintf(buf, PAGE_SIZE,"logo_5p0_en[%d] :0x%x\n", platform_data->logo_5p0_en, val);
	}else {
		return snprintf(buf, PAGE_SIZE,"No define logo_5p0_en\n");
	}
}*/

static ssize_t set_frame(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	u32 val;
	int err = 0;
	ssize_t ret;

	//printk("[AURA_ML51_INBOX] set_frame.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	if (val > 255){
		printk("[AURA_ML51_INBOX] Frame should not over 255.\n");
		return count;
	}

	mutex_lock(&g_pdata->ml51_mutex);
	printk("[AURA_ML51_INBOX] set_frame client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
	err = ml51_write_bytes(client, 0x80F2, val);
	if (err !=1)
		printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	return count;
}

static ssize_t get_frame(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_read_bytes(client, 0x80F3, data);
	if (err != 1)
		printk("[AURA_ML51_INBOX] get_frame:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	pr_info("[AURA_ML51_INBOX] %s data[0]=%d.\n",__func__,data[0]);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t set_speed(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	u32 val;
	int err = 0;
	ssize_t ret;

	//printk("[AURA_ML51_INBOX] set_speed.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;
	pr_debug("[AURA_ML51_INBOX] %s val=%d.\n",__func__,val);
	if (val != 254 && val != 255 && val != 0 && val != 1 && val != 2){
		printk("[AURA_ML51_INBOX] speed should be 254,255,0,1,2 .\n");
		return count;
	}

	mutex_lock(&g_pdata->ml51_mutex);
	//printk("[AURA_ML51_INBOX] client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
	g_speed=val;
	err = ml51_write_bytes(client, 0x8022, val);
	if (err !=1)
		printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	return count;

}

static ssize_t get_speed(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_read_bytes(client, 0x8022, data);
	if (err != 1)
		printk("[AURA_ML51_INBOX] get_speed:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t set_cali_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	int red_val = 0, green_val = 0, blue_val = 0;

	sscanf(buf, "%d %d %d", &red_val, &green_val, &blue_val);
	printk("[AURA_ML51_INBOX] set_cali_data, %d, %d, %d\n", red_val, green_val, blue_val);

	platform_data->RED_MAX = red_val;
	platform_data->GREEN_MAX = green_val;
	platform_data->BLUE_MAX = blue_val;

	return count;
}

static ssize_t get_cali_data(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ml51_platform_data *platform_data = i2c_get_clientdata(client);

	printk("[AURA_ML51_INBOX] R:%d, G:%d, B:%d\n", platform_data->RED_MAX, platform_data->GREEN_MAX, platform_data->BLUE_MAX);
	return snprintf(buf, PAGE_SIZE,"R:%d, G:%d, B:%d\n", platform_data->RED_MAX, platform_data->GREEN_MAX, platform_data->BLUE_MAX);
}

static ssize_t set_vph(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	u32 val;
	ssize_t ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;
	//return because we cann't get the gpio of pogo_sleep and pogo_det
	return count;   

	if(val>0) {
		printk("[AURA_ML51_INBOX] VPH set HIGH\n");
		
		if ( gpio_is_valid(platform_data->pogo_sleep) ) {
			gpio_set_value(platform_data->pogo_sleep, 1);
		}else {
			printk("[AURA_ML51_INBOX] pogo_sleep is not vaild\n");
		}

		if ( gpio_is_valid(platform_data->pogo_det) ) {
			gpio_set_value(platform_data->pogo_det, 1);
		}else {
			printk("[AURA_ML51_INBOX] pogo_det is not vaild\n");
		}

	}else {
		printk("[AURA_ML51_INBOX] VPH set LOW\n");

		if ( gpio_is_valid(platform_data->pogo_sleep) ) {
			gpio_set_value(platform_data->pogo_sleep, 0);
		}else {
			printk("[AURA_ML51_INBOX] pogo_sleep is not vaild\n");
		}

		if ( gpio_is_valid(platform_data->pogo_det) ) {
			gpio_set_value(platform_data->pogo_det, 0);
		}else {
			printk("[AURA_ML51_INBOX] pogo_det is not vaild\n");
		}
	}
	return count;
}

static ssize_t get_vph(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	int val1 = 0, val2 = 0;

	//return because we cann't get the gpio of pogo_sleep and pogo_det
	return 0;   

	if ( gpio_is_valid(platform_data->pogo_sleep) ) {
		val1 = gpio_get_value(platform_data->pogo_sleep);
		printk("[AURA_ML51_INBOX] pogo_sleep[%d] :0x%x\n", platform_data->pogo_sleep, val1);
		//return snprintf(buf, PAGE_SIZE,"%d\n", val);
	}else {
		printk("[AURA_ML51_INBOX] pogo_sleep is not valid\n");
		//return snprintf(buf, PAGE_SIZE,"aura_3p3_en is not valid\n");
	}

	if ( gpio_is_valid(platform_data->pogo_det) ) {
		val2 = gpio_get_value(platform_data->pogo_det);
		printk("[AURA_ML51_INBOX] pogo_det[%d] :0x%x\n", platform_data->pogo_det, val2);
		//return snprintf(buf, PAGE_SIZE,"%d\n", val);
	}else {
		printk("[AURA_ML51_INBOX] pogo_det is not valid\n");
		//return snprintf(buf, PAGE_SIZE,"aura_3p3_en is not valid\n");
	}

	return snprintf(buf, PAGE_SIZE,"pogo_sleep[%d]:%d, pogo_det[%d]:%d\n", platform_data->pogo_sleep, val1, platform_data->pogo_det, val2);
}


static ssize_t fw_ver_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;
	unsigned char ver_major = 0;
	unsigned char ver_minor = 0;

	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_read_words(client, 0xA001, data);
	if (err != 1)
		printk("[AURA_ML51_INBOX] fw_ver_show:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	ver_major = data[0];
	ver_minor = data[1];

	return snprintf(buf, PAGE_SIZE,"0x%02x%02x\n", ver_major,ver_minor);
}

static int ml51_update_write(struct i2c_client *client, char * cmd_data_buf)
{
	int err = 0;

	//printk("[AURA_ML51_INBOX] ml51_update_write \n");
	err = i2c_write_bytes(client, cmd_data_buf, 48);
	if (err !=1)
		printk("[AURA_ML51_INBOX] i2c_write_bytes:err %d\n", err);

	return err;

}
static int ml51_fw_erase(struct i2c_client *client)
{
	int err = 0;
	unsigned char buf[2] = {0};
	buf[0] = 0xA3;
	buf[1] = 0x1;

	printk("[AURA_ML51_INBOX] ml51_fw_erase \n");
	err = i2c_write_bytes(client, buf, 2);
	if (err !=1)
		printk("[AURA_ML51_INBOX] i2c_write_bytes:err %d\n", err);
	printk("[AURA_ML51_INBOX] after erase\n");
	return err;

}
static int ml51_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];

	printk("[AURA_ML51_INBOX] ml51_GetFirmwareSize.\n");
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", firmware_name);
	if (NULL == pfile) {
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}


static int ml51_ReadFirmware(char *fw_name, unsigned char *fw_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", fw_name);
	if (NULL == pfile) {
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, fw_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}

static int ml51_UpdateFirmware(struct i2c_client *client, char *fw_buf,int fwsize)
{
	int err = 0;
	//struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	unsigned char *buf;
	short addr;
	int retry=0;
	unsigned char data[4]={0};
	struct i2c_msg msg;

	buf = kmalloc(sizeof(unsigned char)*49, GFP_DMA);
	if (!buf) {
		printk("unable to allocate key input memory\n");
		return -ENOMEM;
	}

	printk("[AURA_ML51_INBOX] ml51_UpdateFirmware after kmalloc\n");
	//erase--remove this because we will send all 13kb data (add 0 to the end)
	//err = ml51_fw_erase(client);
	//if (err !=1)
		//printk("[AURA_ML51_INBOX] ml51_fw_erase :err %d\n", err);
	//msleep(500);
	//printk("[AURA_ML51_INBOX] after erase :\n");


	//flash

	//first write
	memset(buf,0,sizeof(unsigned char)*48);
	buf[0] = 0xA0;
	buf[13] = 0x34;
	memcpy(&(buf[16]),fw_buf+0,32);

	printk("[AURA_ML51_INBOX] ml51_UpdateFirmware,client->addr : 0x%x\n", client->addr);

	while(1){
		printk("[AURA_ML51_INBOX] ml51_update_write : num=0\n");
		err = ml51_update_write(client,buf);
		if (err !=1)
			printk("[AURA_ML51_INBOX] ml51_update_write :err %d\n", err);
		msleep(1000);

		msg.flags = I2C_M_RD;		//read
		msg.addr = client->addr;
		msg.len = 4;
		msg.buf = data;

		err = i2c_transfer(client->adapter,&msg, 1);

		if((data[2]==0x1F) && (data[3]==0)){
			retry=0;
			printk("[AURA_ML51_INBOX] write ok!\n");
			break;
		}else{
			retry++;
			if (retry > 10) {
				printk("[AURA_ML51_INBOX] ml51_UpdateFirmware retry too many times: %d, Force exit!!!\n", retry);
				kfree(buf);
				return -1;
			}
		}
	}
	//the follwing write
	for(addr = 32; addr < 13*1024; addr = addr+32){

		memset(buf,0,sizeof(unsigned char)*48);
		buf[0] = 0xA0;
		if(addr <= fwsize-32){
			memcpy(&(buf[16]),fw_buf+addr,32);
		}else{
			if(addr >= fwsize){
				memset(&(buf[16]),0,sizeof(unsigned char)*32);
			}else{
				memcpy(&(buf[16]),fw_buf+addr,fwsize-addr);
				//memset(&(buf[16+fwsize-addr]),0,sizeof(unsigned char)*(32-fwsize+addr));
			}
		}

		while(1){
			//printk("[AURA_ML51_INBOX] ml51_update_write : num=%d\n", addr/32);
			err = ml51_update_write(client,buf);
			msleep(10);
			if (err !=1)
				printk("[AURA_ML51_INBOX] ml51_update_write :err %d\n", err);

			if(addr/32 == 415) //the last write
			{
				printk("[AURA_ML51_INBOX] the last write, no return value. assume it writes ok!\n");
				break;
			}

			data[2]=0;
			data[3]=0;
			msg.flags = I2C_M_RD;		//read
			msg.addr = client->addr;
			msg.len = 4;
			msg.buf = data;

			err = i2c_transfer(client->adapter,&msg, 1);

			if((data[2]==((addr+31)&0xFF)) && (data[3]==(((addr+31)>>8)&0xFF))){
				retry=0;
				printk("[AURA_ML51_INBOX] write ok!\n");
				break;
			}else{
				retry++;
				if (retry > 10) {
					printk("[AURA_ML51_INBOX] ml51_UpdateFirmware retry too many times: %d, Force exit!!!\n", retry);
					kfree(buf);
					return -1;
				}
			}
		}//end while
	}//end for
	printk("[AURA_ML51_INBOX] ml51_UpdateFirmware finished.\n");
	kfree(buf);
	return 0;
}
static ssize_t fw_update_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	int err = 0;
	int fw_size;
	unsigned char *fw_buf;
	//struct ml51_platform_data *platform_data = i2c_get_clientdata(client);


	char fw_name[128];

	printk("[AURA_ML51_INBOX] fw_update_store. slave = 0x%x\n", client->addr);

	memset(fw_name, 0, sizeof(fw_name));
	sprintf(fw_name, "%s", buf);
	fw_name[count-1] = '\0';

	// get fs_size
	fw_size = ml51_GetFirmwareSize(fw_name);
	if(fw_size<=0)
	{
		printk("[AURA_ML51_INBOX] fwsize %d\n", fw_size);
		return count;
	}
	printk("[AURA_ML51_INBOX] fwsize %d\n", fw_size);

	// set fw_buf
	fw_buf = kmalloc(fw_size+1 ,GFP_ATOMIC);
	printk("[AURA_ML51_INBOX] after kmalloc\n");


	// read FW content
	if (ml51_ReadFirmware(fw_name, fw_buf)) {
		printk("[AURA_ML51_INBOX] ERROR: request_firmware failed\n");
		kfree(fw_buf);
		return count;
	}
	printk("[AURA_ML51_INBOX] after read \n");


	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_UpdateFirmware(client,fw_buf,fw_size);
	if(err)
		printk("[AURA_ML51_INBOX] ml51_UpdateFirmware, err %d\n", err);
	mutex_unlock(&g_pdata->ml51_mutex);
	kfree(fw_buf);
	return count;


}

/*static ssize_t write_reg_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *buf)
{

	return snprintf(buf, PAGE_SIZE,"%d\n", 0x11);

}
static ssize_t write_reg_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int err=-1;
	char buf_write[2]={0};
	if(buf==NULL)
	{
		printk("[AURA_ML51_INBOX]%s buf is null\n",__func__);
		return -1;
	}
	buf_write[0] = buf[0];

	err = i2c_write_bytes(client, buf_write, 1);
	if (err !=1)
		printk("[AURA_ML51_INBOX] i2c_write_bytes:err %d\n", err);

	return count;
}*/



static ssize_t fw_mode_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	//struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	unsigned char data[4] = {0};
	//int err = 0;
	unsigned char *buf_cmd;
	struct i2c_msg msgs[2];
	int ret=-1;
	//unsigned char tmp[129] = {0};
	//int  i = 0;
	buf_cmd = kmalloc(sizeof(unsigned char)*49, GFP_DMA);
	if (!buf_cmd) {
		printk("unable to allocate key input memory\n");
		return -ENOMEM;
	}

	printk("[AURA_ML51_INBOX] fw_mode_show, after kmalloc\n");
	memset(buf_cmd,0,sizeof(unsigned char)*48);
	buf_cmd[0] = 0xCA;
	/*for(i=0;i<48;i++){
		printk("[AURA_ML51_INBOX] buf[%d]= %x",i,buf[i]);
	}*/

	mutex_lock(&g_pdata->ml51_mutex);
	//send register address
	msgs[0].flags = !I2C_M_RD;	//write
	msgs[0].addr = client->addr;
	msgs[0].len = 48;
	msgs[0].buf = buf_cmd;
	ret = i2c_transfer(client->adapter,&msgs[0], 1);

	msleep(1);
	//read data
	msgs[1].flags = I2C_M_RD;		//read
	msgs[1].addr = client->addr;
	msgs[1].len = 1;
	msgs[1].buf = data;

	ret = i2c_transfer(client->adapter,&msgs[1], 1);
	mutex_unlock(&g_pdata->ml51_mutex);


	/*mutex_lock(&g_pdata->ml51_mutex);
	i2c_read_bytes(client, buf_cmd, 48, data, 1);
	mutex_unlock(&g_pdata->ml51_mutex);*/
	kfree(buf_cmd);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t ld2ap_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	u32 val;
	int err = 0;
	ssize_t ret;

	printk("[AURA_ML51_INBOX] ld2ap_store.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	mutex_lock(&g_pdata->ml51_mutex);
	printk("[AURA_ML51_INBOX] client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
	err = ml51_write_bytes(client, 0xAB00, val);
	if (err !=1)
		printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	return count;

}
static ssize_t ap2ld_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	u32 val;
	int err = 0;
	ssize_t ret;

	printk("[AURA_ML51_INBOX] ap2ld_store.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	mutex_lock(&g_pdata->ml51_mutex);
	printk("[AURA_ML51_INBOX] client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
	err = ml51_write_bytes(client, 0xA002, val);
	if (err !=1)
		printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ml51_mutex);
	return count;
}


static ssize_t erase_ap_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	u32 val;
	//int err = 0;
	ssize_t ret;

	printk("[AURA_ML51_INBOX] erase_ap_store.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	mutex_lock(&g_pdata->ml51_mutex);
	if(val==1){
		ml51_fw_erase(client);
	}
	mutex_unlock(&g_pdata->ml51_mutex);
	return count;

}

static ssize_t frontled_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);

	u32 val;
	int err = 0;
	ssize_t ret;
	//printk("[AURA_ML51_INBOX] frontled_en_store.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;
	pr_debug("[AURA_ML51_INBOX] %s val=%d \n",__func__,val);
	mutex_lock(&g_pdata->ml51_mutex);
	//printk("[AURA_ML51_INBOX] client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
	g_led2_on=val;
	err = ml51_write_bytes(client, 0x6017, val);
	if (err !=1){
		printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);
		mutex_unlock(&g_pdata->ml51_mutex);
		return count;
	}
	mutex_unlock(&g_pdata->ml51_mutex);
	return count;
}

static ssize_t frontled_en_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_read_bytes(client, 0x6017, data);
	if (err != 1){
		printk("[AURA_ML51_INBOX] frontled_en_show:err %d\n", err);
		mutex_unlock(&g_pdata->ml51_mutex);
		return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
	}
	mutex_unlock(&g_pdata->ml51_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t backled_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);

	u32 val;
	int err = 0;
	ssize_t ret;
	//printk("[AURA_ML51_INBOX] backled_en_store.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;
	pr_debug("[AURA_ML51_INBOX] %s val=%d \n",__func__,val);
	mutex_lock(&g_pdata->ml51_mutex);
	//printk("[AURA_ML51_INBOX] client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
	g_led_on=val;
	err = ml51_write_bytes(client, 0x6022, val);
	if (err !=1){
		printk("[AURA_ML51_INBOX] ml51_write_bytes:err %d\n", err);
		mutex_unlock(&g_pdata->ml51_mutex);
		return count;
	}
	mutex_unlock(&g_pdata->ml51_mutex);
	return count;
}

static ssize_t backled_en_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ml51_mutex);
	err = ml51_read_bytes(client, 0x6022, data);
	if (err != 1){
		printk("[AURA_ML51_INBOX] backled_en_show:err %d\n", err);
		mutex_unlock(&g_pdata->ml51_mutex);
		return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
	}
	mutex_unlock(&g_pdata->ml51_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t unique_id_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[12] = {0};
	int err = 0;
	unsigned char cmd[2] = {0};
	struct i2c_msg msgs;

	cmd[0] = 0xA0;
	cmd[1] = 0x03;
	pr_debug("[AURA_ML51_INBOX] %s\n",__func__);
	mutex_lock(&g_pdata->ml51_mutex);
	err = i2c_write_bytes(client, cmd, 2);
	if (err !=1)
		printk("[AURA_ML51_INBOX] i2c_write_bytes 0xA003 :err %d\n", err);

	msleep(2);

	//read data
	msgs.flags = I2C_M_RD;		//read
	msgs.addr = client->addr;
	msgs.len = 12;
	msgs.buf = data;
	err = i2c_transfer(client->adapter,&msgs, 1);
	mutex_unlock(&g_pdata->ml51_mutex);
	return snprintf(buf, PAGE_SIZE,"0x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
		data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[9],data[10],data[11]);
}


static ssize_t aura_fan_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	u32 val;
	ssize_t ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;
	pr_debug("[AURA_ML51_INBOX] %s val=%d \n",__func__,val);
	if(val>0) {
		printk("[AURA_ML51_INBOX] aura_fan_en set HIGH\n");
		if ( gpio_is_valid(platform_data->fan_enable_pin) ) {
			gpio_set_value(platform_data->fan_enable_pin, 1);
		}else {
			printk("[AURA_ML51_INBOX] aura_fan_en is not vaild\n");
		}
		msleep(350); //sleep 350ms for waiting ic power on.
	}else {
		printk("[AURA_ML51_INBOX] aura_fan_en set LOW\n");

		if ( gpio_is_valid(platform_data->fan_enable_pin) ) {
			gpio_set_value(platform_data->fan_enable_pin, 0);
		}else {
			printk("[AURA_ML51_INBOX] aura_fan_en is not vaild\n");
		}
	}
	return count;
}

static ssize_t aura_fan_en_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
	int val = 0;

	if ( gpio_is_valid(platform_data->fan_enable_pin) ) {
		val = gpio_get_value(platform_data->fan_enable_pin);
		printk("[AURA_ML51_INBOX] aura_fan_en[%d] :0x%x\n", platform_data->fan_enable_pin, val);
		return snprintf(buf, PAGE_SIZE,"%d\n", val);
	}else {
		printk("[AURA_ML51_INBOX] aura_fan_en is not valid\n");
		return snprintf(buf, PAGE_SIZE,"aura_fan_en is not valid\n");
	}
}



static DEVICE_ATTR(red_pwm, 0664, red_pwm_show, red_pwm_store);
static DEVICE_ATTR(green_pwm, 0664, green_pwm_show, green_pwm_store);
static DEVICE_ATTR(blue_pwm, 0664, blue_pwm_show, blue_pwm_store);
static DEVICE_ATTR(red1_pwm, 0664, red1_pwm_show, red1_pwm_store);
static DEVICE_ATTR(green1_pwm, 0664, green1_pwm_show, green1_pwm_store);
static DEVICE_ATTR(blue1_pwm, 0664, blue1_pwm_show, blue1_pwm_store);
static DEVICE_ATTR(apply, 0664, apply_show, apply_store);
static DEVICE_ATTR(mode, 0664, mode_show, mode_store);
static DEVICE_ATTR(reset, 0664, reset_show, reset_store);
static DEVICE_ATTR(frame, 0664, get_frame, set_frame);
static DEVICE_ATTR(speed, 0664, get_speed, set_speed);
//static DEVICE_ATTR(led_en, 0664, led_en_show, led_en_store);
static DEVICE_ATTR(Calibration, 0664, get_cali_data, set_cali_data);
static DEVICE_ATTR(VPH, 0664, get_vph, set_vph);
static DEVICE_ATTR(fw_ver, 0664, fw_ver_show, NULL);
static DEVICE_ATTR(fw_update, 0664, NULL, fw_update_store);
static DEVICE_ATTR(fw_mode, 0664, fw_mode_show,NULL);
static DEVICE_ATTR(ap2ld, 0664, NULL,ap2ld_store);
static DEVICE_ATTR(ld2ap, 0664, NULL,ld2ap_store);
static DEVICE_ATTR(erase_ap, 0664, NULL,erase_ap_store);
static DEVICE_ATTR(led2_on, 0664, frontled_en_show, frontled_en_store);
static DEVICE_ATTR(led_on, 0664, backled_en_show, backled_en_store);
static DEVICE_ATTR(unique_id, 0664, unique_id_show, NULL);
static DEVICE_ATTR(aura_fan_en, 0664, aura_fan_en_show, aura_fan_en_store);
static DEVICE_ATTR(led_color, 0664, led_color_show,led_color_store);
static DEVICE_ATTR(mode2, 0664, mode2_show,mode2_store);

static struct attribute *pwm_attrs[] = {
	&dev_attr_red_pwm.attr,
	&dev_attr_green_pwm.attr,
	&dev_attr_blue_pwm.attr,
	&dev_attr_red1_pwm.attr,
	&dev_attr_green1_pwm.attr,
	&dev_attr_blue1_pwm.attr,
	&dev_attr_apply.attr,
	&dev_attr_mode.attr,
	&dev_attr_reset.attr,
	&dev_attr_frame.attr,
	&dev_attr_speed.attr,
	//&dev_attr_led_en.attr,
	&dev_attr_Calibration.attr,
	&dev_attr_VPH.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_fw_update.attr,
	&dev_attr_fw_mode.attr,
	&dev_attr_ap2ld.attr,
	&dev_attr_ld2ap.attr,
	&dev_attr_erase_ap.attr,
	&dev_attr_led2_on.attr,
	&dev_attr_led_on.attr,
	&dev_attr_unique_id.attr,
	&dev_attr_aura_fan_en.attr,
	&dev_attr_led_color.attr,
	&dev_attr_mode2.attr,
	NULL
};

static const struct attribute_group pwm_attr_group = {
	.attrs = pwm_attrs,
};


static void aura_sync_set(struct led_classdev *led,
			      enum led_brightness brightness)
{
	printk("[AURA_ML51_INBOX] aura_sync_set : %d.\n", brightness);
}

static enum led_brightness aura_sync_get(struct led_classdev *led_cdev)
{
	struct ml51_platform_data *pdata;

	printk("[AURA_ML51_INBOX] aura_sync_get.\n");
	pdata = container_of(led_cdev, struct ml51_platform_data, led);

	return pdata->led.brightness;
}

static int aura_sync_register(struct device *dev, struct ml51_platform_data *pdata)
{
	pdata->led.name = "aura_inbox";

	pdata->led.brightness = LED_OFF;
	pdata->led.max_brightness = LED_HALF;
	pdata->led.default_trigger = "none";
	pdata->led.brightness_set = aura_sync_set;
	pdata->led.brightness_get = aura_sync_get;

	return led_classdev_register(dev, &pdata->led);
}

static void aura_sync_unregister(struct ml51_platform_data *pdata)
{
	led_classdev_unregister(&pdata->led);
}

static int ml51_parse_dt(struct device *dev, struct ml51_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
//	int retval;

	printk("[AURA_ML51_INBOX] ml51_parse_dt\n");

	pdata->aura_3p3_en = of_get_named_gpio_flags(np, "nuvoton,aura_3p3_en", 0, &pdata->aura_3p3_en_flags);
	printk("[AURA_ML51_INBOX] aura_3p3_en : %d\n", pdata->aura_3p3_en);

	/*pdata->logo_5p0_en = of_get_named_gpio_flags(np, "nuvoton,logo_5p0_en", 0, &pdata->logo_5p0_en_flags);
	printk("[AURA_ML51_INBOX] logo_5p0_en : %d\n", pdata->logo_5p0_en);

	pdata->pogo_sleep = of_get_named_gpio_flags(np, "nuvoton,pogo-sleep", 0, &pdata->pogo_sleep_flags);
	printk("[AURA_ML51_INBOX] pogo_sleep : %d\n", pdata->pogo_sleep);

	pdata->pogo_det = of_get_named_gpio_flags(np, "nuvoton,pogo-det", 0, &pdata->pogo_det_flags);
	printk("[AURA_ML51_INBOX] pogo_det : %d\n", pdata->pogo_det);*/

#if FAN_ADD
	pdata->fan_enable_pin = of_get_named_gpio_flags(np, "nuvoton,fan_pwr_on", 0, NULL);
	printk("[INBOX_FAN] pdata->fan_enable_pin=%d\n",pdata->fan_enable_pin);

#endif


/*
	printk("[AURA_ML51_INBOX] Get the pinctrl node \n");
	// Get the pinctrl node
	pdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(pdata->pinctrl)) {
	     dev_err(dev, "%s: Failed to get pinctrl\n", __func__);
	}

	printk("[AURA_ML51_INBOX] Get the active setting \n");
	// Get the active setting
	printk("[AURA_ML51_INBOX] Get default_enable \n");
	pdata->pins_active = pinctrl_lookup_state(pdata->pinctrl, "active");
	if (IS_ERR_OR_NULL(pdata->pins_active)) {
		dev_err(dev, "%s: Failed to get pinctrl state active\n", __func__);
	}

	// Set the active setting
	printk("[AURA_ML51_INBOX] set the active state\n");
	retval = pinctrl_select_state(pdata->pinctrl, pdata->pins_active);
	if (retval)
		dev_err(dev, "%s: pinctrl_select_state retval:%d\n", __func__, retval);
*/
	return 0;
}

static int ml51_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct ml51_platform_data *platform_data;

	unsigned char data[4] = {0};
	unsigned char *buf_cmd;
	struct i2c_msg msgs[2];
#if FAN_ADD
	int i = 0;
#endif 

	buf_cmd = kmalloc(sizeof(unsigned char)*49, GFP_DMA);
	if (!buf_cmd) {
		printk("unable to allocate buf_cmd memory\n");
		kfree(buf_cmd);
		return -ENOMEM;
	}

	printk("[AURA_ML51_INBOX] ml51_probe,client->addr : 0x%x\n", client->addr);

	platform_data = devm_kzalloc(&client->dev, sizeof(struct ml51_platform_data), GFP_KERNEL);
	if (!platform_data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		kfree(buf_cmd);
		return -ENOMEM;
	}

	i2c_set_clientdata(client, platform_data);
	mutex_init(&platform_data->ml51_mutex);
// Parse platform data from dtsi
	err = ml51_parse_dt(&client->dev, platform_data);
	if (err) {
		printk("[AURA_ML51_INBOX] ml51_parse_dt get fail !!!\n");
		goto parse_remove;
	}
// Set ml51 power
#if FAN_ADD
	if ( gpio_is_valid(platform_data->fan_enable_pin) ){
		err = gpio_request(platform_data->fan_enable_pin, "fan_enable");
		if (err){
			printk("[INBOX FAN] fan_enable_pin gpio_request, err %d\n", err);
			goto parse_remove;
		}

		printk("[INBOX FAN] ML51 INBOX Fan power on.\n");
		err = gpio_direction_output(platform_data->fan_enable_pin, 1);
		if (err){
			printk("[INBOX FAN] fan_enable_pin output high.err %d\n", err);
			goto fan_on_remove;
		}
	}
	printk("[INBOX FAN] ml51_inbox_probe : platform_data->fan_enable_pin=%d, value=%d  \n",platform_data->fan_enable_pin,gpio_get_value(platform_data->fan_enable_pin));

#endif

//set reset pin to low
	if ( gpio_is_valid(platform_data->aura_3p3_en) ){
		err = gpio_request(platform_data->aura_3p3_en, "aura_3p3_en");
		if (err){
			printk("[AURA_ML51_INBOX] aura_3p3_en gpio_request, err %d\n", err);
			goto fan_on_remove;
		}

		printk("[AURA_ML51_INBOX] ML51 INBOX aura power on.\n");
		err = gpio_direction_output(platform_data->aura_3p3_en, 0);  //user as reset pin from er's inbox
		if (err){
			printk("[AURA_ML51_INBOX] aura_3p3_en output high.err %d\n", err);
			goto aura_en_remove;
		}
	}
// Wait 0.35s for IC power on.
	msleep(350);

//check i2c function
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto aura_en_remove;
	} else
		printk("[AURA_ML51_INBOX] I2C function test pass\n");

	printk("[AURA_ML51_INBOX] get fw_mode in probe function.\n");
	memset(buf_cmd,0,sizeof(unsigned char)*48);
	buf_cmd[0] = 0xCA;
	/*for(i=0;i<48;i++){
		printk("[AURA_ML51_INBOX] buf[%d]= %x",i,buf[i]);
	}*/

	mutex_lock(&platform_data->ml51_mutex);
	//i2c_read_bytes(client, buf_cmd, 48, data, 1);

	//send register address
	msgs[0].flags = !I2C_M_RD;	//write
	msgs[0].addr = client->addr;
	msgs[0].len = 48;
	msgs[0].buf = buf_cmd;
	i2c_transfer(client->adapter,&msgs[0], 1);

	msleep(1);
	//read data
	msgs[1].flags = I2C_M_RD;		//read
	msgs[1].addr = client->addr;
	msgs[1].len = 1;
	msgs[1].buf = data;

	i2c_transfer(client->adapter,&msgs[1], 1);

	mutex_unlock(&platform_data->ml51_mutex);

	if(data[0]!=1 && data[0]!=2){
		printk("[AURA_ML51_INBOX] get fw_mode in probe failed for the first time, we will reset and retry\n");
		if ( gpio_is_valid(platform_data->fan_enable_pin) ) {
			gpio_set_value(platform_data->fan_enable_pin, 0);
		}else {
			printk("[AURA_ML51_INBOX] aura_fan_en is not vaild\n");
		}
		if ( gpio_is_valid(platform_data->fan_enable_pin) ) {
			gpio_set_value(platform_data->fan_enable_pin, 1);
		}else {
			printk("[AURA_ML51_INBOX] aura_fan_en is not vaild\n");
		}
		msleep(350); //sleep 350ms for waiting ic power on.
		memset(buf_cmd,0,sizeof(unsigned char)*48);
		buf_cmd[0] = 0xCA;
		mutex_lock(&platform_data->ml51_mutex);
		msgs[0].flags = !I2C_M_RD;	//write
		msgs[0].addr = client->addr;
		msgs[0].len = 48;
		msgs[0].buf = buf_cmd;
		i2c_transfer(client->adapter,&msgs[0], 1);

		msleep(1);

		msgs[1].flags = I2C_M_RD;		//read
		msgs[1].addr = client->addr;
		msgs[1].len = 1;
		msgs[1].buf = data;
		i2c_transfer(client->adapter,&msgs[1], 1);
		mutex_unlock(&platform_data->ml51_mutex);
		if(data[0]!=1 && data[0]!=2){
			printk("[AURA_ML51_INBOX] get fw_mode in probe failed for the second time, we will return\n");
			goto aura_en_remove;
		}
	}

// Register sys class  
	err = aura_sync_register(&client->dev, platform_data);
	if (err) {
		printk("[AURA_ML51_INBOX] Failed to register LED device: %d\n", err);
		goto aura_en_remove;
	}
	err = sysfs_create_group(&platform_data->led.dev->kobj, &pwm_attr_group);
	if (err)
		goto unled;


	platform_data->current_mode = 0x0;
	platform_data->suspend_state = false;

// Default Calibration Data
	platform_data->RED_MAX = 255;
	platform_data->GREEN_MAX = 255;
	platform_data->BLUE_MAX = 255;

#if FAN_ADD
	mutex_init(&platform_data->update_lock);
	//Register sys class for fan
	for (i = 0; i < ARRAY_SIZE(fan_attr); i++) {
		err = device_create_file(&client->dev, &fan_attr[i].dev_attr);
		if (err)
			goto unled;
	}

	platform_data->hwmon_dev = hwmon_device_register_with_groups(&client->dev, "Inbox_Fan", NULL, NULL); //for fan hwmon nodes
	if (IS_ERR(platform_data->hwmon_dev)) {
		err = PTR_ERR(platform_data->hwmon_dev);
		goto fan_remove;
	}

#endif

// Set global variable
	g_pdata = platform_data;
	g_red=-1;
    g_green=-1;
   	g_blue=-1;
    g_mode=-1;
    g_speed=-1;
    g_led_on=-1;
    g_led2_on=-1;

	printk("[AURA_ML51_INBOX] ml51_probe done.\n");
	return 0;

#if FAN_ADD
fan_remove:
	for (i = 0; i < ARRAY_SIZE(fan_attr); i++)
		device_remove_file(&client->dev, &fan_attr[i].dev_attr);
	mutex_destroy(&platform_data->update_lock);
#endif
unled:
	sysfs_remove_group(&platform_data->led.dev->kobj, &pwm_attr_group);
	aura_sync_unregister(platform_data);
aura_en_remove:
	printk("[AURA_ML51_INBOX] power off.\n");
	if(gpio_is_valid(platform_data->aura_3p3_en)) {
		printk("[INBOX FAN] set GPIO[%d] output low\n", platform_data->aura_3p3_en);
		err = gpio_direction_output(platform_data->aura_3p3_en, 0);
		if (err)
			printk("[INBOX FAN] fan power_gpio output low, err %d\n", err);
		printk("[INBOX FAN] Free GPIO[%d].\n", platform_data->aura_3p3_en);
		gpio_free(platform_data->aura_3p3_en);
	}
#if FAN_ADD
fan_on_remove:
	if(gpio_is_valid(platform_data->fan_enable_pin)) {
		printk("[INBOX FAN] set GPIO[%d] output low\n", platform_data->fan_enable_pin);
		err = gpio_direction_output(platform_data->fan_enable_pin, 0);
		if (err)
			printk("[INBOX FAN] fan power_gpio output low, err %d\n", err);
		printk("[INBOX FAN] Free GPIO[%d].\n", platform_data->fan_enable_pin);
		gpio_free(platform_data->fan_enable_pin);
	}
#endif
parse_remove:
	mutex_destroy(&platform_data->ml51_mutex);
	kfree(buf_cmd);
	return -1;
}

static int ml51_remove(struct i2c_client *client)
{
	int err = 0;
	struct ml51_platform_data *platform_data = i2c_get_clientdata(client);
#if FAN_ADD
	int i = 0;
	struct device *dev = &client->dev;

	//remove this for inbox have been removed already so that i2c cmd woulf fail.
	/*if (get_fan_vdd(client)== 1) {
		
		err = fan_off(client);
		if(!err){
			printk("[INBOX FAN]: %s :fan off",__func__);
		}else{
			printk("[INBOX FAN]: %s :fan off failed, err=%d\n",__func__,err);
			return err;
		}
		msleep(100); //Wait 0.1s for IC power on
	}*/

		//---sys unregister
	hwmon_device_unregister(platform_data->hwmon_dev);

	for (i = 0; i < ARRAY_SIZE(fan_attr); i++)
		device_remove_file(dev, &fan_attr[i].dev_attr);

	//--lock destroy
	mutex_destroy(&platform_data->update_lock);	
#endif
// unregister
	printk("[AURA_ML51_INBOX] sysfs_remove_group\n");
	sysfs_remove_group(&platform_data->led.dev->kobj, &pwm_attr_group);

	printk("[AURA_ML51_INBOX] AURA_ML51_unregister\n");
	aura_sync_unregister(platform_data);
//destroy ml51 lock
	mutex_destroy(&platform_data->ml51_mutex);
// reset pin
	printk("[AURA_ML51_INBOX] set GPIO[%d] output low\n", platform_data->aura_3p3_en);
	err = gpio_direction_output(platform_data->aura_3p3_en, 0);
	if (err)
		printk("[AURA_ML51_INBOX] aura_3p3_en output low, err %d\n", err);
// free gpio
	printk("[AURA_ML51_INBOX] Free GPIO[%d].\n", platform_data->aura_3p3_en);
	gpio_free(platform_data->aura_3p3_en);
//power off
#if FAN_ADD
	if(gpio_is_valid(platform_data->fan_enable_pin)) {
		printk("[FAN] set GPIO[%d] output low\n", platform_data->fan_enable_pin);
		err = gpio_direction_output(platform_data->fan_enable_pin, 0);
		if (err)
			printk("[FAN] fan power_gpio output low, err %d\n", err);
		printk("[FAN] Free GPIO[%d].\n", platform_data->fan_enable_pin);
		gpio_free(platform_data->fan_enable_pin);
	}
	

#endif
	printk("[AURA_ML51_INBOX] ml51_remove : err %d\n", err);
	return 0;
}

int ml51_inbox_suspend(struct device *dev)
{
	int err = 0;

	printk("[AURA_ML51_INBOX] ml51_inbox_suspend : current_mode : 0x%x\n", g_pdata->current_mode);

	/*if(!g_pdata->current_mode){
		//printk("[AURA_ML51_INBOX] Disable VDD.\n");
		if ( gpio_is_valid(g_pdata->aura_3p3_en) )
			gpio_set_value(g_pdata->aura_3p3_en, 0);
	}*/

	g_pdata->suspend_state = true;

	return err;
}

int ml51_inbox_resume(struct device *dev)
{
	int err = 0;

	printk("[AURA_ML51_INBOX] ml51_inbox_resume : current_mode : 0x%x\n", g_pdata->current_mode);

	/*if(!g_pdata->current_mode){
		//printk("[AURA_ML51_INBOX] Enable VDD.\n");
		if ( gpio_is_valid(g_pdata->aura_3p3_en) )
			gpio_set_value(g_pdata->aura_3p3_en, 1);
	}*/

	g_pdata->suspend_state = false;

	return err;
}

static const struct i2c_device_id ml51_id[] = {
	{ "ml51_i2c", 0},
	{},
};
//MODULE_DEVICE_TABLE(i2c, ml51_id);

static const struct dev_pm_ops ml51_pm_ops = {
	.suspend	= ml51_inbox_suspend,
	.resume	= ml51_inbox_resume,
};

#ifdef CONFIG_OF
static const struct of_device_id ml51_match_table[] = {
	{ .compatible = "ml51_inbox",},
	{ },
};
#else
#define ene_match_table NULL
#endif

static struct i2c_driver ml51_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver		= {
		.name		= "ml51_inbox",
		.owner = THIS_MODULE,
		.pm	= &ml51_pm_ops,
		.of_match_table	= ml51_match_table,
	},
	.probe		= ml51_probe,
	.remove		= ml51_remove,
	.id_table 	= ml51_id,
};

static int __init ml51_bus_init(void)
{
	int ret;

	ret = i2c_add_driver(&ml51_driver);
	if (ret)
		printk("[AURA_ML51_INBOX] NUVOTON ML51 driver int failed.\n");
	else
		printk("[AURA_ML51_INBOX] NUVOTON ML51 driver int success.\n");
	
	return ret;
}
module_init(ml51_bus_init);

static void __exit ml51_bus_exit(void)
{
	i2c_del_driver(&ml51_driver);
}
module_exit(ml51_bus_exit);

MODULE_AUTHOR("ASUS Deeo Ho");
MODULE_DESCRIPTION("Aura sync LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("asus:nuvoton-ml51");
