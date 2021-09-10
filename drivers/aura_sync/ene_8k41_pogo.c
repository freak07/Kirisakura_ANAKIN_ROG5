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
#include "ene_8k41.h"

//For HID wait for completion
#include <linux/completion.h>
//#include <linux/msm_drm_notify.h>

#define FW_PATH "/asusfw/aura_sync/ENE-8K41-aura-V7.bin"

extern int hid_suspend_vote(int);
extern int hid_vote_register(char *);
extern int hid_vote_unregister(int, char *);
extern int hid_to_gpio_set(u8 gpio, u8 value);
extern int hid_to_gpio_get(u8 gpio);
extern int hid_get_gpio_data(char *buffer, int count, int *len);
extern int asus_wait4hid (void);

extern uint8_t gDongleType;

static struct ene_8k41_platform_data *g_pdata;

static int i2c_read_bytes(struct i2c_client *client, char *write_buf, int writelen, char *read_buf, int readlen)
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
}

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

static int ene_8k41_read_bytes(struct i2c_client *client, short addr, char *data)
{
	int err = 0;
	unsigned char buf[16] = {0};

	buf[0] = 0x00;
	buf[1] = (addr >> 8) & 0xFF;
	buf[2] = addr & 0xFF;

//	printk("[AURA_POGO] ene_8k41_read_bytes : addr: 0x%x, buf[1] : 0x%x, buf[2] : 0x%x\n", (addr & 0xFFFF), buf[1], buf[2]);
	err = i2c_write_bytes(client, buf, 3);	//set register address
	if (err !=1)
		printk("[AURA_POGO] i2c_write_bytes:err %d\n", err);

	buf[0] = 0x81;
	err = i2c_read_bytes(client, buf, 1, data, 1);	//send read command
	if (err != 2)
		printk("[AURA_POGO] i2c_read_bytes:err %d\n", err);

	return err;
}

static int ene_8k41_write_bytes(struct i2c_client *client, short addr, char value)
{
	int err = 0;
	unsigned char buf[16] = {0};

	buf[0] = 0x00;
	buf[1] = (addr >> 8) & 0xFF;
	buf[2] = addr & 0xFF;

//	printk("[AURA_POGO] ene_8k41_write_bytes : addr: 0x%x, buf[1] : 0x%x, buf[2] : 0x%x, value : 0x%x\n", (addr & 0xFFFF), buf[1], buf[2], value);
	err = i2c_write_bytes(client, buf, 3);	//set register address
	if (err !=1)
		printk("[AURA_POGO] i2c_write_bytes:err %d\n", err);

	buf[0] = 0x01;
	buf[1] = value;
	
	err = i2c_write_bytes(client, buf, 2);	//set register address
	if (err !=1)
		printk("[AURA_POGO] i2c_write_bytes:err %d\n", err);
	
	return err;
}

static int ene_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];

	printk("[AURA_POGO] ene_GetFirmwareSize.\n");
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

static int ene_ReadFirmware(char *fw_name, unsigned char *fw_buf)
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

static char ene_CheckFirmwareVer(struct i2c_client *client)
{
	int err = 0;
	unsigned char data[2] = {0};

	err = ene_8k41_read_bytes(client, 0x80C0, data);
	if (err != 2){
		printk("[AURA_POGO] fw_check:err %d\n", err);
		data[0] = 0xFF;
	}

	return (u8) data[0];
}

static int ene_UpdateFirmware(struct i2c_client *client, char *fw_buf)
{
	int err = 0;
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	unsigned char data[2] = {0};
	unsigned char buf[129] = {0};
	//unsigned char tmp[129] = {0};
	//int  i = 0;
	short addr;
	int retry = 0;

	err = ene_8k41_read_bytes(client, 0xFF8F, data);
	if (err != 2)
		printk("[AURA_POGO] Check REG[0xFF8F]:err %d\n", err);

	printk("[AURA_POGO] REG[0xFF8F] : 0x%x\n", data[0]);

	if( (u8)(data[0]) <= 0x40){
		platform_data->fw_version = ene_CheckFirmwareVer(client);
		printk("[AURA_POGO] FW VER : 0x%x\n", platform_data->fw_version);
		//if ( platform_data->fw_version == 0x0) { // rember change condition.
		if (0) {
			printk("[AURA_POGO] Don't need FW update.\n");
			return 0;
		}
	} else
		printk("[ASUS_SYNC] FW is abnormal !!!");

	printk("[AURA_POGO] Start ene_UpdateFirmware\n");

	err = ene_8k41_write_bytes(client, 0xF018, 0xCE);
	if (err !=1)
		printk("[AURA_POGO] REG[F010]: write err %d\n", err);

	err = ene_8k41_read_bytes(client, 0xF018, data);
	if (err != 2)
		printk("[AURA_POGO] REG[0xF018]: read err %d\n", err);

	printk("[AURA_POGO] REG[0xF018]: 0x%x\n", data[0]);

// (1) Reset 8051
	printk("[AURA_POGO] Reset 8051 =====\n");
	err = ene_8k41_write_bytes(client, 0xF010, 0x5);
	if (err !=1)
		printk("[AURA_POGO] REG[0xF010]: write err %d\n", err);

	msleep(500);

// (2) Read & Fill OSC32M freq.
	printk("[AURA_POGO] Read & Fill OSC32M freq =====\n");
	err = ene_8k41_write_bytes(client, 0xF808, 0xF0);
	if (err !=1)
		printk("[AURA_POGO] REG[0xF808]: write err %d\n", err);
	err = ene_8k41_write_bytes(client, 0xF809, 0x01);
	if (err !=1)
		printk("[AURA_POGO] REG[0xF809]: write err %d\n", err);
	err = ene_8k41_write_bytes(client, 0xF807, 0x90);
	if (err !=1)
		printk("[AURA_POGO] REG[0xF807]: write err %d\n", err);

	err = ene_8k41_read_bytes(client, 0xF80B, data);
	if (err != 2)
		printk("[AURA_POGO] REG[0xF80B]: read err %d\n", err);

	printk("[AURA_POGO] REG[0xF80B]: 0x%x\n", data[0]);
	err = ene_8k41_write_bytes(client, 0xF806, data[0]);
	if (err != 1)
		printk("[AURA_POGO] REG[0xF806]: write err %d\n", err);

// (3) Fill program & Erase timing
	printk("[AURA_POGO] Fill program & Erase timing =====\n");
	err = ene_8k41_write_bytes(client, 0xF815, 0x10);
	if (err !=1)
		printk("[AURA_POGO] REG[0xF815]: write err %d\n", err);
	err = ene_8k41_write_bytes(client, 0xF816, 0x11);
	if (err !=1)
		printk("[AURA_POGO] REG[0xF816]: write err %d\n", err);
	err = ene_8k41_write_bytes(client, 0xF817, 0x06);
	if (err !=1)
		printk("[AURA_POGO] REG[0xF817]: write err %d\n", err);
	err = ene_8k41_write_bytes(client, 0xF818, 0x07);
	if (err !=1)
		printk("[AURA_POGO] REG[0xF818]: write err %d\n", err);

// (4) Erase
	printk("[AURA_POGO] Erase page from 0x0000 to 0x3F80 =====\n");
	for (addr = 0x0000 ; addr <= 0x3F80 ; addr+= 0x80) {

		printk("[AURA_POGO] set addr : 0x%04x\n", addr);
		err = ene_8k41_write_bytes(client, 0xF808, (addr & 0xFF));				//	Addr_L
		if (err !=1)
			printk("[AURA_POGO] REG[0xF808]: write err %d\n", err);
		err = ene_8k41_write_bytes(client, 0xF809, (addr >> 8) & 0xFF);		//	Addr_H
		if (err !=1)
			printk("[AURA_POGO] REG[0xF809]: write err %d\n", err);

		err = ene_8k41_write_bytes(client, 0xF807, 0x20);	// erase page cmd
		if (err !=1)
			printk("[AURA_POGO] REG[0xF807]: write err %d\n", err);

		while(1){	 // check cmd finished. 0xF800[7] = 1
			msleep(50);
			data[0] = 0x0;
			err = ene_8k41_read_bytes(client, 0xF800, data);
			if (err != 2) {
				printk("[AURA_POGO] REG[0xF800]: read err %d\n", err);
			}
			if ( ((data[0] >> 7) & 0x1) == 1 ){
				//printk("[AURA_POGO] check REG[0xF800] pass, 0x%02x\n", (data[0]));
				retry=0;
				break;
			}
			printk("[AURA_POGO] check REG[0xF800], 0x%02x\n", data[0]);
			retry++;
			if (retry > 10) {
				printk("[AURA_POGO] retry too many times: %d, Force exit!!!\n", retry);
				return -1;
			}
		}
	}
	printk("[AURA_POGO] erase page finished. =====\n");
	
// (5) Program
	printk("[AURA_POGO] Program FW from 0x0000 to 0x3F80 =====\n");
	for (addr = 0x0000 ; addr <= 0x3F80 ; addr = addr + 0x80) {

		printk("[AURA_POGO] set addr : 0x%04x\n", addr);
		err = ene_8k41_write_bytes(client, 0xF808, (addr & 0xFF));				//	Addr_L
		if (err !=1)
			printk("[AURA_POGO] REG[0xF808]: write err %d\n", err);
		err = ene_8k41_write_bytes(client, 0xF809, (addr >> 8) & 0xFF);		//	Addr_H
		if (err !=1)
			printk("[AURA_POGO] REG[0xF809]: write err %d\n", err);

		err = ene_8k41_write_bytes(client, 0xF807, 0x80);	// 3. Clear HVPL data
		if (err !=1)
			printk("[AURA_POGO] REG[0xF807]: write err %d\n", err);

		while(1){	 // 4. check cmd finished 0xF800[7] = 1
			msleep(50);
			data[0] = 0x0;
			err = ene_8k41_read_bytes(client, 0xF800, data);
			if (err != 2) {
				printk("[AURA_POGO] REG[0xF800]: read err %d\n", err);
			}
			if ( ((data[0] >> 7) & 0x1) == 1 ){
				//printk("[AURA_POGO] check REG[0xF800] pass, 0x%02x\n", (data[0]));
				retry = 0;
				break;
			}
			printk("[AURA_POGO] check REG[0xF800], 0x%02x\n", (data[0]));
			retry++;
			if (retry > 10) {
				printk("[AURA_POGO] retry too many times: %d, Force exit!!!\n", retry);
				return -1;
			}
		}

		// 5. Set address to 0xF80A
		buf[0] = 0x00;
		buf[1] = 0xF8;	//Addr_H
		buf[2] = 0x0A;	//Addr_L
		err = i2c_write_bytes(client, buf, 3);	//set register address
		if (err !=1)
			printk("[AURA_POGO] i2c_write_bytes:err %d\n", err);

		// 6. Send data to buffer
		buf[0] = 0x05;
		memcpy( &(buf[1]), fw_buf + addr, 128); // copy fw_buf to buf

		err = i2c_write_bytes(client, buf, 129);	 // 128 byte + 1 cmd = 129
		if (err !=1)
			printk("[AURA_POGO] i2c_write_bytes:err %d\n", err);

		// 7. Set address to 0xF807
		buf[0] = 0x00;
		buf[1] = 0xF8;	//Addr_H
		buf[2] = 0x07;	//Addr_L
		err = i2c_write_bytes(client, buf, 3);	//set register address
		if (err !=1)
			printk("[AURA_POGO] i2c_write_bytes:err %d\n", err);

		// 8. Program page
		buf[0] = 0x05;
		buf[1] = 0x70;
		err = i2c_write_bytes(client, buf, 2);
		if (err !=1)
			printk("[AURA_POGO] i2c_write_bytes:err %d\n", err);

		while(1){	 // 9. check cmd finished 0xF800[7] = 1
			msleep(50);
			data[0] = 0x0;
			err = ene_8k41_read_bytes(client, 0xF800, data);
			if (err != 2) {
				printk("[AURA_POGO] REG[0xF800]: read err %d\n", err);
			}
			if ( ((data[0] >> 7) & 0x1) == 1 ){
				//printk("[AURA_POGO] check REG[0xF800] pass, 0x%02x\n", (data[0]));
				retry = 0;
				break;
			}
			printk("[AURA_POGO] check REG[0xF800], 0x%02x\n", (data[0]));
			retry++;
			if (retry > 10) {
				printk("[AURA_POGO] retry too many times: %d, Force exit!!!\n", retry);
				return -1;
			}
		}
	}
	
////////////////////////////
/*
	printk("[AURA_POGO] read FW in 8K41 rom +++\n");
	for (addr = 0x0000 ; addr <= 0x0080 ; addr = addr + 0x80) {
		printk("[AURA_POGO] set addr : 0x%04x\n", addr);
		err = ene_8k41_write_bytes(client, 0xF808, (addr & 0xFF));				//	Addr_L
		if (err !=1)
			printk("[AURA_POGO] REG[0xF808]: write err %d\n", err);
		err = ene_8k41_write_bytes(client, 0xF809, (addr >> 8) & 0xFF);		//	Addr_H
		if (err !=1)
			printk("[AURA_POGO] REG[0xF809]: write err %d\n", err);

		// 5. Set address to 0xF80B
		buf[0] = 0x00;
		buf[1] = 0xF8;	//Addr_H
		buf[2] = 0x0B;	//Addr_L
		err = i2c_write_bytes(client, buf, 3);	//set register address
		if (err !=1)
			printk("[AURA_POGO] i2c_write_bytes:err %d\n", err);

		buf[0] = 0x06;
		err = i2c_read_bytes(client, buf, 1, tmp, 128);	//set register address
		if (err !=2)
			printk("[AURA_POGO] i2c_read_bytes:err %d\n", err);

		for (i=0; i<128 ; i++)
			printk("[AURA_POGO] tmp[%d]:0x%x\n", i, tmp[i]);
	}
	printk("[AURA_POGO] read FW in 8K41 rom ---\n");
*/
////////////////////////////

// (9) Restart 8051
	printk("[AURA_POGO] Restart 8051.\n");
	err = ene_8k41_write_bytes(client, 0xF010, 0x04);
	if (err !=1)
		printk("[AURA_POGO] REG[0xF807]: write err %d\n", err);

	msleep(500);

	err = ene_8k41_write_bytes(client, 0xF018, 0x00);
	if (err !=1)
		printk("[AURA_POGO] REG[F010]: write err %d\n", err);

	platform_data->FW_update_done = true;
	printk("[AURA_POGO] ene_UpdateFirmware finished. %d\n", platform_data->FW_update_done);

	return 0;
}

static ssize_t red_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;

	printk("[AURA_POGO] red_pwm_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return ret;

	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->RED_MAX), 255);

	mutex_lock(&g_pdata->ene_mutex);
	printk("[AURA_POGO] client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);
	printk("[AURA_POGO] red tmp %d\n", tmp);
	err = ene_8k41_write_bytes(client, 0x8010, tmp);
	if (err !=1)
		printk("[AURA_POGO] ene_8k41_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t red_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_8k41_read_bytes(client, 0x8010, data);
	if (err != 2)
		printk("[AURA_POGO] red_pwm_show:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t green_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;

	printk("[AURA_POGO] green_pwm_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return ret;

	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->GREEN_MAX), 255);

	mutex_lock(&g_pdata->ene_mutex);
	printk("[AURA_POGO] client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);
	printk("[AURA_POGO] green tmp %d\n", tmp);
	err = ene_8k41_write_bytes(client, 0x8011, tmp);
	if (err !=1)
		printk("[AURA_POGO] ene_8k41_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t green_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_8k41_read_bytes(client, 0x8011, data);
	if (err != 2)
		printk("[AURA_POGO] green_pwm_show:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t blue_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;

	printk("[AURA_POGO] blue_pwm_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return ret;

	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->BLUE_MAX), 255);

	mutex_lock(&g_pdata->ene_mutex);
	printk("[AURA_POGO] client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);
	printk("[AURA_POGO] blue tmp %d\n", tmp);
	err = ene_8k41_write_bytes(client, 0x8012, tmp);
	if (err !=1)
		printk("[AURA_POGO] ene_8k41_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t blue_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_8k41_read_bytes(client, 0x8012, data);
	if (err != 2)
		printk("[AURA_POGO] blue_pwm_show:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t apply_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	u32 val;
	int err = 0;
	ssize_t ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	mutex_lock(&g_pdata->ene_mutex);
	if (val > 0){
		printk("[AURA_POGO] Send apply cmd.\n");
		err = ene_8k41_write_bytes(client, 0x802F, 0x1);
		if (err !=1)
			printk("[AURA_POGO] ene_8k41_write_bytes:err %d\n", err);
	} else
		printk("[AURA_POGO] No send apply cmd.\n");

	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t apply_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_8k41_read_bytes(client, 0x802F, data);
	if (err != 2)
		printk("[AURA_POGO] apply_show:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	u32 val;
	int err = 0;
	ssize_t ret;

	printk("[AURA_POGO] mode_store.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	mutex_lock(&g_pdata->ene_mutex);
	printk("[AURA_POGO] client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
	err = ene_8k41_write_bytes(client, 0x8021, val);
	if (err !=1){
		printk("[AURA_POGO] ene_8k41_write_bytes:err %d\n", err);
		mutex_unlock(&g_pdata->ene_mutex);
		return count;
	}

	platform_data->current_mode = (u8)val;
//	ASUSEvtlog("[AURA_POGO] current_mode : %d\n", platform_data->current_mode);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_8k41_read_bytes(client, 0x8021, data);
	if (err != 2){
		printk("[AURA_POGO] mode_show:err %d\n", err);
		mutex_unlock(&g_pdata->ene_mutex);
		return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
	}

	platform_data->current_mode = data[0];

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t vdd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	//struct i2c_client *client = to_i2c_client(dev->parent);
	//struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	u32 val;
	//int err;
	ssize_t ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if(val>0) {
		printk("[AURA_POGO] VDD set HIGH\n");
		hid_to_gpio_set(0x3F, 1);
	}else {
		printk("[AURA_POGO] VDD set LOW\n");
		hid_to_gpio_set(0x3F, 0);
	}
	return count;
}

static ssize_t vdd_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	char buffer[4] = {0};
	int count = 2;
	int ret = 0, len = 0;

	printk("[AURA_POGO] vdd_show\n");
	hid_to_gpio_get(0x3F);
	ret = hid_get_gpio_data(buffer, count, &len);
	if (ret < 0)
		return sprintf(buf, "%s\n", "HID_not_connect");

	return snprintf(buf, PAGE_SIZE,"0x%x\n", buffer[0]);
}

static ssize_t led_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	u32 val;
	ssize_t ret;
	int err = 0;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if(val>0) {
		printk("[AURA_POGO] LED_EN set HIGH\n");
		
		if ( gpio_is_valid(platform_data->logo_5v_en) )
			gpio_set_value(platform_data->logo_5v_en, 1);

		if (gDongleType == 3){
			printk("[AURA_DT] led_en on\n");
			// set 0xF203 as 0x01 for PB0 gpio output mode enable
			err = ene_8k41_write_bytes(client, 0xF203, 0x01);
			if (err < 0)
				printk("[AURA_DT] ene_8k41_write_bytes:err %d\n", err);

			// set 0xF208 as 0x01 for PB0 gpio HIGH
			err = ene_8k41_write_bytes(client, 0xF208, 0x01);
			if (err < 0)
				printk("[AURA_DT] ene_8k41_write_bytes:err %d\n", err);
		}
	}else {
		printk("[AURA_POGO] LED_EN set LOW\n");

		if ( gpio_is_valid(platform_data->logo_5v_en) )
			gpio_set_value(platform_data->logo_5v_en, 0);

		if (gDongleType == 3){
			printk("[AURA_DT] led_en off\n");
			// set 0xF203 as 0x00 for PB0 gpio output mode enable
			err = ene_8k41_write_bytes(client, 0xF203, 0x0);
			if (err < 0)
				printk("[AURA_DT] ene_8k41_write_bytes:err %d\n", err);

			// set 0xF208 as 0x00 for PB0 gpio HIGH
			err = ene_8k41_write_bytes(client, 0xF208, 0x0);
			if (err < 0)
				printk("[AURA_DT] ene_8k41_write_bytes:err %d\n", err);
		}
	}

	return count;
}

static ssize_t led_en_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	int val = 0;
	unsigned char data[2] = {0};
	int err = 0;

	if ( gpio_is_valid(platform_data->logo_5v_en) ) {
		val = gpio_get_value(platform_data->logo_5v_en);
		return snprintf(buf, PAGE_SIZE,"logo_5v_en[%d] :0x%x\n", platform_data->logo_5v_en, val);
	}else if ( gDongleType == 3 ) {

		err = ene_8k41_read_bytes(client, 0xF208, data);
		if (err < 0)
			printk("[AURA_DT] ene_8k41_read_bytes:err %d\n", err);

		return snprintf(buf, PAGE_SIZE,"DT LED_EN : 0x%x\n", data[0]);
	}else {
		return snprintf(buf, PAGE_SIZE,"No define logo_5v_en\n");
	}
}

static ssize_t fw_update_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	int err = 0;
	char fw_name[128];
	unsigned char *fw_buf;
	int fw_size;

	memset(fw_name, 0, sizeof(fw_name));
	sprintf(fw_name, "%s", buf);
	fw_name[count-1] = '\0';
	
	printk("[AURA_POGO] fwname : %s\n", fw_name);

	// get fs_size
	fw_size = ene_GetFirmwareSize(fw_name);
	printk("[AURA_POGO] fwsize %d\n", fw_size);
	
	// set fw_buf
	fw_buf = kmalloc(fw_size+1 ,GFP_ATOMIC);

	// read FW content
	if (ene_ReadFirmware(fw_name, fw_buf)) {
		printk("[AURA_POGO] ERROR: request_firmware failed\n");
		kfree(fw_buf);
		return -EIO;
	}

	// Start update FW
	mutex_lock(&g_pdata->ene_mutex);
	err = ene_UpdateFirmware(client, fw_buf);
	if(err)
		printk("[AURA_POGO] ene_UpdateFirmware, err %d\n", err);

	// Update FW VER
	platform_data->fw_version = ene_CheckFirmwareVer(client);
	printk("[AURA_POGO] FW VER : 0x%x\n", platform_data->fw_version);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t fw_ver_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);

	err = ene_8k41_read_bytes(client, 0xF018, data);
	if (err != 2){
		printk("[AURA_POGO] fw_ver_show:err %d\n", err);
		mutex_unlock(&g_pdata->ene_mutex);
		return snprintf(buf, PAGE_SIZE,"i2c_error\n");
	}

	platform_data->fw_version = ene_CheckFirmwareVer(client);

	printk("[AURA_POGO] fw_ver_show : 0x%x\n", platform_data->fw_version);
	mutex_unlock(&g_pdata->ene_mutex);

	//if (data[0] != 0x0 && !platform_data->FW_update_done){
	if (data[0] != 0x0){
		printk("[AURA_POGO] FW Error, REG[0xF018] : 0x%x\n", data[0]);
		return snprintf(buf, PAGE_SIZE,"0x0\n");
	}

	if (platform_data->fw_version == 0xFF){
		return snprintf(buf, PAGE_SIZE,"i2c_error\n");
	}

	return snprintf(buf, PAGE_SIZE,"0x%x\n", platform_data->fw_version);
}

static ssize_t set_frame(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	u32 val;
	int err = 0;
	ssize_t ret;

	printk("[AURA_POGO] set_frame.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if (val > 255){
		printk("[AURA_POGO] Frame should not over 255.\n");
		return ret;
	}

	mutex_lock(&g_pdata->ene_mutex);
	printk("[AURA_POGO] client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
	err = ene_8k41_write_bytes(client, 0x80F2, val);
	if (err !=1)
		printk("[AURA_POGO] ene_8k41_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t get_frame(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_8k41_read_bytes(client, 0x80F3, data);
	if (err != 2)
		printk("[AURA_POGO] get_frame:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t set_speed(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	u32 val;
	int err = 0;
	ssize_t ret;

	printk("[AURA_POGO] set_speed.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if (val != 254 && val != 255 && val != 0 && val != 1 && val != 2){
		printk("[AURA_POGO] speed should be 254,255,0,1,2 .\n");
		return ret;
	}

	mutex_lock(&g_pdata->ene_mutex);
	printk("[AURA_POGO] client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
	err = ene_8k41_write_bytes(client, 0x8022, val);
	if (err !=1)
		printk("[AURA_POGO] ene_8k41_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;

}

static ssize_t get_speed(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_8k41_read_bytes(client, 0x8022, data);
	if (err != 2)
		printk("[AURA_POGO] get_speed:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t set_cali_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	int red_val = 0, green_val = 0, blue_val = 0;

	sscanf(buf, "%d %d %d", &red_val, &green_val, &blue_val);
	printk("[AURA_POGO] set_cali_data, %d, %d, %d\n", red_val, green_val, blue_val);

	platform_data->RED_MAX = red_val;
	platform_data->GREEN_MAX = green_val;
	platform_data->BLUE_MAX = blue_val;

	return count;
}

static ssize_t get_cali_data(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);

	printk("[AURA_POGO] R:%d, G:%d, B:%d\n", platform_data->RED_MAX, platform_data->GREEN_MAX, platform_data->BLUE_MAX);
	return snprintf(buf, PAGE_SIZE,"R:%d, G:%d, B:%d\n", platform_data->RED_MAX, platform_data->GREEN_MAX, platform_data->BLUE_MAX);
}

static ssize_t pogo_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE,"1\n");
}

static DEVICE_ATTR(red_pwm, 0664, red_pwm_show, red_pwm_store);
static DEVICE_ATTR(green_pwm, 0664, green_pwm_show, green_pwm_store);
static DEVICE_ATTR(blue_pwm, 0664, blue_pwm_show, blue_pwm_store);
static DEVICE_ATTR(apply, 0664, apply_show, apply_store);
static DEVICE_ATTR(mode, 0664, mode_show, mode_store);
static DEVICE_ATTR(VDD, 0664, vdd_show, vdd_store);
static DEVICE_ATTR(fw_update, 0664, NULL, fw_update_store);
static DEVICE_ATTR(fw_ver, 0664, fw_ver_show, NULL);
static DEVICE_ATTR(frame, 0664, get_frame, set_frame);
static DEVICE_ATTR(speed, 0664, get_speed, set_speed);
static DEVICE_ATTR(led_en, 0664, led_en_show, led_en_store);
static DEVICE_ATTR(Calibration, 0664, get_cali_data, set_cali_data);
static DEVICE_ATTR(pogo, 0664, pogo_show, NULL);

static struct attribute *pwm_attrs[] = {
	&dev_attr_red_pwm.attr,
	&dev_attr_green_pwm.attr,
	&dev_attr_blue_pwm.attr,
	&dev_attr_apply.attr,
	&dev_attr_mode.attr,
	&dev_attr_VDD.attr,
	&dev_attr_fw_update.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_frame.attr,
	&dev_attr_speed.attr,
	&dev_attr_led_en.attr,
	&dev_attr_Calibration.attr,
	&dev_attr_pogo.attr,
	NULL
};

static const struct attribute_group pwm_attr_group = {
	.attrs = pwm_attrs,
};

// Register FB notifier +++
static int ene_8k41_fb_callback(struct notifier_block *nb, unsigned long val, void *data)
{
	struct ene_8k41_platform_data *platform_data;
	struct msm_drm_notifier *evdata = data;
	unsigned int blank;
	int err;

	if (val != MSM_DRM_EARLY_EVENT_BLANK)
		return 0;

	if (evdata->id != 0)	// id=0 is internal display, external is 1
		return 0;

	printk("[AURA_POGO] go to the ene_8k41_fb_callback value = %d msm_drm_display_id = %d\n", (int)val, evdata->id);
	platform_data = container_of(nb, struct ene_8k41_platform_data, notifier);
	if (evdata && evdata->data && val == MSM_DRM_EARLY_EVENT_BLANK && platform_data) {
		blank = *(int *)(evdata->data);

	printk("[AURA_POGO] go to the blank value = %d\n", (int)blank);

	switch (blank) {
		case MSM_DRM_BLANK_POWERDOWN:
			printk("[AURA_POGO] ene_8k41_suspend : current_mode : 0x%x\n", g_pdata->current_mode);
			if(!platform_data->current_mode)
				hid_to_gpio_set(0x3F, 0);

			platform_data->suspend_state = true;

			err = hid_suspend_vote(platform_data->hid_suspend_id);
			if(err)
				printk("[AURA_POGO] Fail hid_suspend_vote\n");

			break;
		case MSM_DRM_BLANK_UNBLANK:
			printk("[AURA_POGO] ene_8k41_resume : current_mode : 0x%x\n", g_pdata->current_mode);
			if(!platform_data->current_mode){
				err = asus_wait4hid();
				if (err)
					printk("[AURA_POGO] Fail to wait HID\n");

				hid_to_gpio_set(0x3F, 1);
			}

			platform_data->suspend_state = false;

			break;
		default:
			break;
		}
	}

	return NOTIFY_OK;
}

static struct notifier_block ene_8k41_noti_block = {
	.notifier_call = ene_8k41_fb_callback,
};
// Register FB notifier ---

static void aura_sync_set(struct led_classdev *led,
			      enum led_brightness brightness)
{
	printk("[AURA_POGO] aura_sync_set : %d.\n", brightness);
}

static enum led_brightness aura_sync_get(struct led_classdev *led_cdev)
{
	struct ene_8k41_platform_data *pdata;

	printk("[AURA_POGO] aura_sync_get.\n");
	pdata = container_of(led_cdev, struct ene_8k41_platform_data, led);

	return pdata->led.brightness;
}

static int aura_sync_register(struct device *dev, struct ene_8k41_platform_data *pdata)
{
	//if (!pdata->dongle)
	//	pdata->led.name = "aura_sync";
	//else
		pdata->led.name = "aura_sync_station";

	pdata->led.brightness = LED_OFF;
	pdata->led.max_brightness = LED_HALF;
	pdata->led.default_trigger = "none";
	pdata->led.brightness_set = aura_sync_set;
	pdata->led.brightness_get = aura_sync_get;

	return led_classdev_register(dev, &pdata->led);
}

static void aura_sync_unregister(struct ene_8k41_platform_data *pdata)
{
	led_classdev_unregister(&pdata->led);
}

static int ene_8k41_parse_dt(struct device *dev, struct ene_8k41_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	//int retval;
	//u32 voltage_supply[2];

	printk("[AURA_POGO] ene_8k41_parse_dt\n");

	pdata->dongle = of_property_read_bool(np, "ene,station-dongle");
	printk("[AURA_POGO] dongle : %d\n", pdata->dongle);

	if (pdata->dongle) {
		pdata->int_gpio = -2;
		pdata->power_gpio = -2;
		pdata->logo_5v_en = -2;
		pdata->regulator_vdd_vmin = 0;
		pdata->regulator_vdd_vmax = 0;
		pdata->regulator_current = 0;
	}

	return 0;
}

bool station_aura_pogo=true;
EXPORT_SYMBOL(station_aura_pogo);

static int ene_8k41_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	//unsigned char buf[128] = {0};
	//int readlen = 0;
	unsigned char *fw_buf;
	int fw_size;
	struct ene_8k41_platform_data *platform_data;
	//unsigned char data[4] = {0};
	
	printk("[AURA_POGO] ene_8k41_probe.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
	goto exit_check_functionality_failed;
	} else
		printk("[AURA_POGO] I2C function test pass\n");

	printk("[AURA_POGO] client->addr : 0x%x\n", client->addr);
	//printk("[AURA_POGO] check data[0]: 0x%x\n", data[0]);

	platform_data = devm_kzalloc(&client->dev, sizeof(struct ene_8k41_platform_data), GFP_KERNEL);
	if (!platform_data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, platform_data);
// Parse platform data from dtsi
	err = ene_8k41_parse_dt(&client->dev, platform_data);
	if (err) {
		printk("[AURA_POGO] ene_8k41_parse_dt get fail !!!\n");
		return -ENOMEM;
	}

// Wait 0.5s for IC power on.\n");
	hid_to_gpio_set(0x3F, 1);
	msleep(500);
	printk("[AURA_POGO] ==========\n");

// Check I2C status for AURA_STATION
/*
	err = ene_8k41_read_bytes(client, 0xF204, data);
	if (err != 2){
		printk("[AURA_POGO] check IC :err %d\n", err);
		printk("[AURA_POGO] Skip AURA_POGO probe\n");
		goto exit_check_functionality_failed;
	}
	printk("[AURA_POGO] data[0] : 0x%x\n", data[0]);
*/
// Check FW
	platform_data->fw_version = ene_CheckFirmwareVer(client);
	printk("[AURA_POGO] FW VER : 0x%x\n", platform_data->fw_version);

	//if ( platform_data->fw_version < 0x5) { //disable auto update in kernel space
	if (0){
		printk("[AURA_POGO] Start auto FW update.");

		// get fs_size
		fw_size = ene_GetFirmwareSize(FW_PATH);
		if (fw_size <= 0) {
			printk("[AURA_POGO] get fwsize error %d.\n", fw_size);
			goto skip_auto_update;
		} else
			printk("[AURA_POGO] fw_size %d\n", fw_size);

		// set fw_buf
		fw_buf = kmalloc(fw_size+1 ,GFP_ATOMIC);

		// read FW content
		if (ene_ReadFirmware(FW_PATH, fw_buf)) {
			printk("[AURA_POGO] ERROR: request_firmware failed\n");
			kfree(fw_buf);
		}
	}
	
skip_auto_update:
/*
// Setting PWM inverse
	printk("[AURA_POGO] set PWM inverse, REG[0xF207] = 0x8\n");
	err = ene_8k41_write_bytes(client, 0xF207, 0x8);
	if (err !=1)
		printk("[AURA_POGO] ene_8k41_write_bytes:err %d\n", err);

	err = ene_8k41_read_bytes(client, 0xF207, data);
	if (err != 2)
		printk("[AURA_POGO] ene_8k41_read_bytes:err %d\n", err);

	printk("[AURA_POGO] REG[0xF207]: 0x%x\n", data[0]);
	printk("[AURA_POGO] ==========\n");
*/

// Register sys class  
	err = aura_sync_register(&client->dev, platform_data);
	if (err) {
		printk("[AURA_POGO] Failed to register LED device: %d\n", err);
		goto unled;
	}
	err = sysfs_create_group(&platform_data->led.dev->kobj, &pwm_attr_group);
	if (err)
			goto unled;

	mutex_init(&platform_data->ene_mutex);
	platform_data->current_mode = 0x0;
	platform_data->suspend_state = false;
	platform_data->FW_update_done = false;

	platform_data->hid_suspend_id = hid_vote_register("AURA_POGO");
	platform_data->notifier = ene_8k41_noti_block;
//	msm_drm_register_client(&platform_data->notifier);

// Default Calibration Data
	platform_data->RED_MAX = 255;
	platform_data->GREEN_MAX = 255;
	platform_data->BLUE_MAX = 255;

// Set global variable
	g_pdata = platform_data;

	station_aura_pogo = true;
	printk("[AURA_POGO] ene_8k41_probe done.\n");
	return 0;

unled:
	aura_sync_unregister(platform_data);
	printk("[AURA_POGO] ENE 8K41 power off.\n");

exit_check_functionality_failed:
	printk("[AURA_POGO] ene_8k41_probe fail !!!\n");
	station_aura_pogo = false;
	return err;
}

static int ene_8k41_remove(struct i2c_client *client)
{
	int err = 0;
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);

	if (!station_aura_pogo){
		printk("[AURA_POGO] ene_8k41_remove : err %d\n", err);
		return 0;		
	}

	hid_to_gpio_set(0x3F, 0);

// unregister
	printk("[AURA_POGO] sysfs_remove_group\n");
	sysfs_remove_group(&platform_data->led.dev->kobj, &pwm_attr_group);

	printk("[AURA_POGO] aura_sync_unregister\n");
	aura_sync_unregister(platform_data);

// free pointer
//	printk("[AURA_POGO] Free platform_data\n");
//	kfree(platform_data);

	mutex_destroy(&platform_data->ene_mutex);
	hid_vote_unregister(platform_data->hid_suspend_id, "AURA_POGO");
//	msm_drm_unregister_client(&platform_data->notifier);

	printk("[AURA_POGO] ene_8k41_remove : err %d\n", err);
	return 0;
}

int ene_8k41_suspend(struct device *dev)
{
	int err = 0;

//	printk("[AURA_POGO] ene_8k41_suspend : current_mode : 0x%x\n", g_pdata->current_mode);

/*
	if(!g_pdata->current_mode)
		hid_to_gpio_set(0x3F, 0);

	g_pdata->suspend_state = true;

	err = hid_suspend_vote(g_pdata->hid_suspend_id);
*/
	return err;
}

int ene_8k41_resume(struct device *dev)
{
	int err = 0;

//	printk("[AURA_POGO] ene_8k41_resume : current_mode : 0x%x\n", g_pdata->current_mode);

/*
	if(!g_pdata->current_mode){
		err = asus_wait4hid();
		if (err)
			printk("[AURA_POGO] Fail to wait HID\n");

		hid_to_gpio_set(0x3F, 1);
	}

	g_pdata->suspend_state = false;
*/
	return err;
}

static const struct i2c_device_id ene_8k41_id[] = {
	{ "ene_8k41_i2c", 0},
	{},
};
//MODULE_DEVICE_TABLE(i2c, ene_8k41_id);

static const struct dev_pm_ops ene_8k41_pm_ops = {
	.suspend	= ene_8k41_suspend,
	.resume	= ene_8k41_resume,
};

#ifdef CONFIG_OF
static const struct of_device_id ene_match_table[] = {
	{ .compatible = "ene8k41_pogo",},
	{ },
};
#else
#define ene_match_table NULL
#endif

static struct i2c_driver ene_8k41_driver = {
	.driver		= {
		.name		= "ene8k41_pogo",
		.owner = THIS_MODULE,
		.pm	= &ene_8k41_pm_ops,
		.of_match_table	= ene_match_table,
	},
	.probe		= ene_8k41_probe,
	.remove		= ene_8k41_remove,
	.id_table 	= ene_8k41_id,
};

static int __init ene_8k41_bus_init(void)
{
	int ret;

	ret = i2c_add_driver(&ene_8k41_driver);
	if (ret)
		printk("[AURA_POGO] ENE 8k41 driver int failed.\n");
	else
		printk("[AURA_POGO] ENE 8k41 driver int success.\n");
	
	return ret;
}
module_init(ene_8k41_bus_init);

static void __exit ene_8k41_bus_exit(void)
{
	i2c_del_driver(&ene_8k41_driver);
}
module_exit(ene_8k41_bus_exit);

MODULE_AUTHOR("ASUS Deeo Ho");
MODULE_DESCRIPTION("Aura sync LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("asus:ene-8k41");
