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
#include "ene_8k41_inbox.h"

#define FW_PATH "/asusfw/aura_sync/ENE-8K41-aura-V7.bin"

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

//	printk("[AURA_INBOX] ene_8k41_read_bytes : addr: 0x%x, buf[1] : 0x%x, buf[2] : 0x%x\n", (addr & 0xFFFF), buf[1], buf[2]);
	err = i2c_write_bytes(client, buf, 3);	//set register address
	if (err !=1)
		printk("[AURA_INBOX] i2c_write_bytes:err %d\n", err);

	buf[0] = 0x81;
	err = i2c_read_bytes(client, buf, 1, data, 1);	//send read command
	if (err != 2)
		printk("[AURA_INBOX] i2c_read_bytes:err %d\n", err);

	return err;
}

static int ene_8k41_write_bytes(struct i2c_client *client, short addr, char value)
{
	int err = 0;
	unsigned char buf[16] = {0};

	buf[0] = 0x00;
	buf[1] = (addr >> 8) & 0xFF;
	buf[2] = addr & 0xFF;

//	printk("[AURA_INBOX] ene_8k41_write_bytes : addr: 0x%x, buf[1] : 0x%x, buf[2] : 0x%x, value : 0x%x\n", (addr & 0xFFFF), buf[1], buf[2], value);
	err = i2c_write_bytes(client, buf, 3);	//set register address
	if (err !=1)
		printk("[AURA_INBOX] i2c_write_bytes:err %d\n", err);

	buf[0] = 0x01;
	buf[1] = value;

	err = i2c_write_bytes(client, buf, 2);	//set register address
	if (err !=1)
		printk("[AURA_INBOX] i2c_write_bytes:err %d\n", err);

	return err;
}

static int ene_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];

	printk("[AURA_INBOX] ene_GetFirmwareSize.\n");
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
		printk("[AURA_INBOX] fw_check:err %d\n", err);
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
		printk("[AURA_INBOX] Check REG[0xFF8F]:err %d\n", err);

	printk("[AURA_INBOX] REG[0xFF8F] : 0x%x\n", data[0]);

	if( (u8)(data[0]) <= 0x40){
		platform_data->fw_version = ene_CheckFirmwareVer(client);
		printk("[AURA_INBOX] FW VER : 0x%x\n", platform_data->fw_version);
		//if ( platform_data->fw_version == 0x0) { // rember change condition.
		if (0) {
			printk("[AURA_INBOX] Don't need FW update.\n");
			return 0;
		}
	} else
		printk("[ASUS_SYNC] FW is abnormal !!!");

	printk("[AURA_INBOX] Start ene_UpdateFirmware\n");

	err = ene_8k41_write_bytes(client, 0xF018, 0xCE);
	if (err !=1)
		printk("[AURA_INBOX] REG[F010]: write err %d\n", err);

	err = ene_8k41_read_bytes(client, 0xF018, data);
	if (err != 2)
		printk("[AURA_INBOX] REG[0xF018]: read err %d\n", err);

	printk("[AURA_INBOX] REG[0xF018]: 0x%x\n", data[0]);

// (1) Reset 8051
	printk("[AURA_INBOX] Reset 8051 =====\n");
	err = ene_8k41_write_bytes(client, 0xF010, 0x5);
	if (err !=1)
		printk("[AURA_INBOX] REG[0xF010]: write err %d\n", err);

	msleep(500);

// (2) Read & Fill OSC32M freq.
	printk("[AURA_INBOX] Read & Fill OSC32M freq =====\n");
	err = ene_8k41_write_bytes(client, 0xF808, 0xF0);
	if (err !=1)
		printk("[AURA_INBOX] REG[0xF808]: write err %d\n", err);
	err = ene_8k41_write_bytes(client, 0xF809, 0x01);
	if (err !=1)
		printk("[AURA_INBOX] REG[0xF809]: write err %d\n", err);
	err = ene_8k41_write_bytes(client, 0xF807, 0x90);
	if (err !=1)
		printk("[AURA_INBOX] REG[0xF807]: write err %d\n", err);

	err = ene_8k41_read_bytes(client, 0xF80B, data);
	if (err != 2)
		printk("[AURA_INBOX] REG[0xF80B]: read err %d\n", err);

	printk("[AURA_INBOX] REG[0xF80B]: 0x%x\n", data[0]);
	err = ene_8k41_write_bytes(client, 0xF806, data[0]);
	if (err != 1)
		printk("[AURA_INBOX] REG[0xF806]: write err %d\n", err);

// (3) Fill program & Erase timing
	printk("[AURA_INBOX] Fill program & Erase timing =====\n");
	err = ene_8k41_write_bytes(client, 0xF815, 0x10);
	if (err !=1)
		printk("[AURA_INBOX] REG[0xF815]: write err %d\n", err);
	err = ene_8k41_write_bytes(client, 0xF816, 0x11);
	if (err !=1)
		printk("[AURA_INBOX] REG[0xF816]: write err %d\n", err);
	err = ene_8k41_write_bytes(client, 0xF817, 0x06);
	if (err !=1)
		printk("[AURA_INBOX] REG[0xF817]: write err %d\n", err);
	err = ene_8k41_write_bytes(client, 0xF818, 0x07);
	if (err !=1)
		printk("[AURA_INBOX] REG[0xF818]: write err %d\n", err);

// (4) Erase
	printk("[AURA_INBOX] Erase page from 0x0000 to 0x3F80 =====\n");
	for (addr = 0x0000 ; addr <= 0x3F80 ; addr+= 0x80) {

		printk("[AURA_INBOX] set addr : 0x%04x\n", addr);
		err = ene_8k41_write_bytes(client, 0xF808, (addr & 0xFF));				//	Addr_L
		if (err !=1)
			printk("[AURA_INBOX] REG[0xF808]: write err %d\n", err);
		err = ene_8k41_write_bytes(client, 0xF809, (addr >> 8) & 0xFF);		//	Addr_H
		if (err !=1)
			printk("[AURA_INBOX] REG[0xF809]: write err %d\n", err);

		err = ene_8k41_write_bytes(client, 0xF807, 0x20);	// erase page cmd
		if (err !=1)
			printk("[AURA_INBOX] REG[0xF807]: write err %d\n", err);

		while(1){	 // check cmd finished. 0xF800[7] = 1
			msleep(50);
			data[0] = 0x0;
			err = ene_8k41_read_bytes(client, 0xF800, data);
			if (err != 2) {
				printk("[AURA_INBOX] REG[0xF800]: read err %d\n", err);
			}
			if ( ((data[0] >> 7) & 0x1) == 1 ){
				//printk("[AURA_INBOX] check REG[0xF800] pass, 0x%02x\n", (data[0]));
				retry=0;
				break;
			}
			printk("[AURA_INBOX] check REG[0xF800], 0x%02x\n", data[0]);
			retry++;
			if (retry > 10) {
				printk("[AURA_INBOX] retry too many times: %d, Force exit!!!\n", retry);
				return -1;
			}
		}
	}
	printk("[AURA_INBOX] erase page finished. =====\n");

// (5) Program
	printk("[AURA_INBOX] Program FW from 0x0000 to 0x3F80 =====\n");
	for (addr = 0x0000 ; addr <= 0x3F80 ; addr = addr + 0x80) {

		printk("[AURA_INBOX] set addr : 0x%04x\n", addr);
		err = ene_8k41_write_bytes(client, 0xF808, (addr & 0xFF));				//	Addr_L
		if (err !=1)
			printk("[AURA_INBOX] REG[0xF808]: write err %d\n", err);
		err = ene_8k41_write_bytes(client, 0xF809, (addr >> 8) & 0xFF);		//	Addr_H
		if (err !=1)
			printk("[AURA_INBOX] REG[0xF809]: write err %d\n", err);

		err = ene_8k41_write_bytes(client, 0xF807, 0x80);	// 3. Clear HVPL data
		if (err !=1)
			printk("[AURA_INBOX] REG[0xF807]: write err %d\n", err);

		while(1){	 // 4. check cmd finished 0xF800[7] = 1
			msleep(50);
			data[0] = 0x0;
			err = ene_8k41_read_bytes(client, 0xF800, data);
			if (err != 2) {
				printk("[AURA_INBOX] REG[0xF800]: read err %d\n", err);
			}
			if ( ((data[0] >> 7) & 0x1) == 1 ){
				//printk("[AURA_INBOX] check REG[0xF800] pass, 0x%02x\n", (data[0]));
				retry = 0;
				break;
			}
			printk("[AURA_INBOX] check REG[0xF800], 0x%02x\n", (data[0]));
			retry++;
			if (retry > 10) {
				printk("[AURA_INBOX] retry too many times: %d, Force exit!!!\n", retry);
				return -1;
			}
		}

		// 5. Set address to 0xF80A
		buf[0] = 0x00;
		buf[1] = 0xF8;	//Addr_H
		buf[2] = 0x0A;	//Addr_L
		err = i2c_write_bytes(client, buf, 3);	//set register address
		if (err !=1)
			printk("[AURA_INBOX] i2c_write_bytes:err %d\n", err);

		// 6. Send data to buffer
		buf[0] = 0x05;
		memcpy( &(buf[1]), fw_buf + addr, 128); // copy fw_buf to buf

		err = i2c_write_bytes(client, buf, 129);	 // 128 byte + 1 cmd = 129
		if (err !=1)
			printk("[AURA_INBOX] i2c_write_bytes:err %d\n", err);

		// 7. Set address to 0xF807
		buf[0] = 0x00;
		buf[1] = 0xF8;	//Addr_H
		buf[2] = 0x07;	//Addr_L
		err = i2c_write_bytes(client, buf, 3);	//set register address
		if (err !=1)
			printk("[AURA_INBOX] i2c_write_bytes:err %d\n", err);

		// 8. Program page
		buf[0] = 0x05;
		buf[1] = 0x70;
		err = i2c_write_bytes(client, buf, 2);
		if (err !=1)
			printk("[AURA_INBOX] i2c_write_bytes:err %d\n", err);

		while(1){	 // 9. check cmd finished 0xF800[7] = 1
			msleep(50);
			data[0] = 0x0;
			err = ene_8k41_read_bytes(client, 0xF800, data);
			if (err != 2) {
				printk("[AURA_INBOX] REG[0xF800]: read err %d\n", err);
			}
			if ( ((data[0] >> 7) & 0x1) == 1 ){
				//printk("[AURA_INBOX] check REG[0xF800] pass, 0x%02x\n", (data[0]));
				retry = 0;
				break;
			}
			printk("[AURA_INBOX] check REG[0xF800], 0x%02x\n", (data[0]));
			retry++;
			if (retry > 10) {
				printk("[AURA_INBOX] retry too many times: %d, Force exit!!!\n", retry);
				return -1;
			}
		}
	}

////////////////////////////
/*
	printk("[AURA_INBOX] read FW in 8K41 rom +++\n");
	for (addr = 0x0000 ; addr <= 0x0080 ; addr = addr + 0x80) {
		printk("[AURA_INBOX] set addr : 0x%04x\n", addr);
		err = ene_8k41_write_bytes(client, 0xF808, (addr & 0xFF));				//	Addr_L
		if (err !=1)
			printk("[AURA_INBOX] REG[0xF808]: write err %d\n", err);
		err = ene_8k41_write_bytes(client, 0xF809, (addr >> 8) & 0xFF);		//	Addr_H
		if (err !=1)
			printk("[AURA_INBOX] REG[0xF809]: write err %d\n", err);

		// 5. Set address to 0xF80B
		buf[0] = 0x00;
		buf[1] = 0xF8;	//Addr_H
		buf[2] = 0x0B;	//Addr_L
		err = i2c_write_bytes(client, buf, 3);	//set register address
		if (err !=1)
			printk("[AURA_INBOX] i2c_write_bytes:err %d\n", err);

		buf[0] = 0x06;
		err = i2c_read_bytes(client, buf, 1, tmp, 128);	//set register address
		if (err !=2)
			printk("[AURA_INBOX] i2c_read_bytes:err %d\n", err);

		for (i=0; i<128 ; i++)
			printk("[AURA_INBOX] tmp[%d]:0x%x\n", i, tmp[i]);
	}
	printk("[AURA_INBOX] read FW in 8K41 rom ---\n");
*/
////////////////////////////

// (9) Restart 8051
	printk("[AURA_INBOX] Restart 8051.\n");
	err = ene_8k41_write_bytes(client, 0xF010, 0x04);
	if (err !=1)
		printk("[AURA_INBOX] REG[0xF807]: write err %d\n", err);

	msleep(500);

	err = ene_8k41_write_bytes(client, 0xF018, 0x00);
	if (err !=1)
		printk("[AURA_INBOX] REG[F010]: write err %d\n", err);

	platform_data->FW_update_done = true;
	printk("[AURA_INBOX] ene_UpdateFirmware finished. %d\n", platform_data->FW_update_done);

	return 0;
}

static ssize_t red_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;

	printk("[AURA_INBOX] red_pwm_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return ret;

	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->RED_MAX), 255);

	mutex_lock(&g_pdata->ene_mutex);
	printk("[AURA_INBOX] client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);
	printk("[AURA_INBOX] red tmp %d\n", tmp);
	err = ene_8k41_write_bytes(client, 0x8010, tmp);
	if (err !=1)
		printk("[AURA_INBOX] ene_8k41_write_bytes:err %d\n", err);

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
		printk("[AURA_INBOX] red_pwm_show:err %d\n", err);

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

	printk("[AURA_INBOX] green_pwm_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return ret;

	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->GREEN_MAX), 255);

	mutex_lock(&g_pdata->ene_mutex);
	printk("[AURA_INBOX] client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);
	printk("[AURA_INBOX] green tmp %d\n", tmp);
	err = ene_8k41_write_bytes(client, 0x8011, tmp);
	if (err !=1)
		printk("[AURA_INBOX] ene_8k41_write_bytes:err %d\n", err);

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
		printk("[AURA_INBOX] green_pwm_show:err %d\n", err);

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

	printk("[AURA_INBOX] blue_pwm_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return ret;

	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->BLUE_MAX), 255);

	mutex_lock(&g_pdata->ene_mutex);
	printk("[AURA_INBOX] client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);
	printk("[AURA_INBOX] blue tmp %d\n", tmp);
	err = ene_8k41_write_bytes(client, 0x8012, tmp);
	if (err !=1)
		printk("[AURA_INBOX] ene_8k41_write_bytes:err %d\n", err);

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
		printk("[AURA_INBOX] blue_pwm_show:err %d\n", err);

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
		printk("[AURA_INBOX] Send apply cmd.\n");
		err = ene_8k41_write_bytes(client, 0x802F, 0x1);
		if (err !=1)
			printk("[AURA_INBOX] ene_8k41_write_bytes:err %d\n", err);
	} else
		printk("[AURA_INBOX] No send apply cmd.\n");

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
		printk("[AURA_INBOX] apply_show:err %d\n", err);

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

	printk("[AURA_INBOX] mode_store.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	mutex_lock(&g_pdata->ene_mutex);
	printk("[AURA_INBOX] client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
	err = ene_8k41_write_bytes(client, 0x8021, val);
	if (err !=1){
		printk("[AURA_INBOX] ene_8k41_write_bytes:err %d\n", err);
		mutex_unlock(&g_pdata->ene_mutex);
		return count;
	}

	platform_data->current_mode = (u8)val;
//	if(gDongleType == 1)
//		ASUSEvtlog("[AURA_INBOX] current_mode : %d\n", platform_data->current_mode);
//	else
//		ASUSEvtlog("[AURA_DT] current_mode : %d\n", platform_data->current_mode);

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
		printk("[AURA_INBOX] mode_show:err %d\n", err);
		mutex_unlock(&g_pdata->ene_mutex);
		return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
	}

	platform_data->current_mode = data[0];

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t vdd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	u32 val;
	int err;
	ssize_t ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if(val>0) {
		printk("[AURA_INBOX] VDD set HIGH\n");

		if ( gpio_is_valid(platform_data->power_gpio) ) {
			gpio_set_value(platform_data->power_gpio, 1);
		}else {
			err = regulator_enable(platform_data->regulator_vdd);
			if (err) {
				dev_err(dev,
					"%s: Failed to enable regulator vdd\n",
					__func__);
			}
		}
	}else {
		printk("[AURA_INBOX] VDD set LOW\n");

		if ( gpio_is_valid(platform_data->power_gpio) ) {
			gpio_set_value(platform_data->power_gpio, 0);
		}else {
			err = regulator_disable(platform_data->regulator_vdd);
			if (err) {
				dev_err(dev,
					"%s: Failed to enable regulator vdd\n",
					__func__);
			}
		}
	}
	return count;
}

static ssize_t vdd_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	int val = 0;

	if ( gpio_is_valid(platform_data->power_gpio) ) {
		val = gpio_get_value(platform_data->power_gpio);
		printk("[AURA_INBOX] power_gpio[%d] :0x%x\n", platform_data->power_gpio, val);
		return snprintf(buf, PAGE_SIZE,"%d\n", val);
	}else {
		val = regulator_is_enabled(platform_data->regulator_vdd);
		printk("[AURA_INBOX] regulator : %d\n", val);
		return snprintf(buf, PAGE_SIZE,"%d\n", val);
	}
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
		printk("[AURA_INBOX] LED_EN set HIGH\n");

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
		printk("[AURA_INBOX] LED_EN set LOW\n");

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

	printk("[AURA_INBOX] fwname : %s\n", fw_name);

	// get fs_size
	fw_size = ene_GetFirmwareSize(fw_name);
	printk("[AURA_INBOX] fwsize %d\n", fw_size);

	// set fw_buf
	fw_buf = kmalloc(fw_size+1 ,GFP_ATOMIC);

	// read FW content
	if (ene_ReadFirmware(fw_name, fw_buf)) {
		printk("[AURA_INBOX] ERROR: request_firmware failed\n");
		kfree(fw_buf);
		return -EIO;
	}

	// Start update FW
	mutex_lock(&g_pdata->ene_mutex);
	err = ene_UpdateFirmware(client, fw_buf);
	if(err)
		printk("[AURA_INBOX] ene_UpdateFirmware, err %d\n", err);

	// Update FW VER
	platform_data->fw_version = ene_CheckFirmwareVer(client);
	printk("[AURA_INBOX] FW VER : 0x%x\n", platform_data->fw_version);

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
		printk("[AURA_INBOX] fw_ver_show:err %d\n", err);
		mutex_unlock(&g_pdata->ene_mutex);
		return snprintf(buf, PAGE_SIZE,"i2c_error\n");
	}

	platform_data->fw_version = ene_CheckFirmwareVer(client);

	printk("[AURA_INBOX] fw_ver_show : 0x%x\n", platform_data->fw_version);
	mutex_unlock(&g_pdata->ene_mutex);

	//if (data[0] != 0x0 && !platform_data->FW_update_done){
	if (data[0] != 0x0){
		printk("[AURA_INBOX] FW Error, REG[0xF018] : 0x%x\n", data[0]);
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

	printk("[AURA_INBOX] set_frame.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if (val > 255){
		printk("[AURA_INBOX] Frame should not over 255.\n");
		return ret;
	}

	mutex_lock(&g_pdata->ene_mutex);
	printk("[AURA_INBOX] client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
	err = ene_8k41_write_bytes(client, 0x80F2, val);
	if (err !=1)
		printk("[AURA_INBOX] ene_8k41_write_bytes:err %d\n", err);

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
		printk("[AURA_INBOX] get_frame:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t set_speed(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	u32 val;
	int err = 0;
	ssize_t ret;

	printk("[AURA_INBOX] set_speed.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if (val != 254 && val != 255 && val != 0 && val != 1 && val != 2){
		printk("[AURA_INBOX] speed should be 254,255,0,1,2 .\n");
		return ret;
	}

	mutex_lock(&g_pdata->ene_mutex);
	printk("[AURA_INBOX] client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
	err = ene_8k41_write_bytes(client, 0x8022, val);
	if (err !=1)
		printk("[AURA_INBOX] ene_8k41_write_bytes:err %d\n", err);

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
		printk("[AURA_INBOX] get_speed:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t set_cali_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	int red_val = 0, green_val = 0, blue_val = 0;

	sscanf(buf, "%d %d %d", &red_val, &green_val, &blue_val);
	printk("[AURA_INBOX] set_cali_data, %d, %d, %d\n", red_val, green_val, blue_val);

	platform_data->RED_MAX = red_val;
	platform_data->GREEN_MAX = green_val;
	platform_data->BLUE_MAX = blue_val;

	return count;
}

static ssize_t get_cali_data(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);

	printk("[AURA_INBOX] R:%d, G:%d, B:%d\n", platform_data->RED_MAX, platform_data->GREEN_MAX, platform_data->BLUE_MAX);
	return snprintf(buf, PAGE_SIZE,"R:%d, G:%d, B:%d\n", platform_data->RED_MAX, platform_data->GREEN_MAX, platform_data->BLUE_MAX);
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
	NULL
};

static const struct attribute_group pwm_attr_group = {
	.attrs = pwm_attrs,
};


static void aura_sync_set(struct led_classdev *led,
			      enum led_brightness brightness)
{
	printk("[AURA_INBOX] aura_sync_set : %d.\n", brightness);
}

static enum led_brightness aura_sync_get(struct led_classdev *led_cdev)
{
	struct ene_8k41_platform_data *pdata;

	printk("[AURA_INBOX] aura_sync_get.\n");
	pdata = container_of(led_cdev, struct ene_8k41_platform_data, led);

	//printk("[AURA_INBOX] pdata->int_gpio : %d\n", pdata->int_gpio);
	//printk("[AURA_INBOX] pdata->power_gpio : %d\n", pdata->power_gpio);

	return pdata->led.brightness;
}

static int aura_sync_register(struct device *dev, struct ene_8k41_platform_data *pdata)
{
	if (!pdata->dongle)
		pdata->led.name = "aura_sync";
	else
		pdata->led.name = "aura_sync_inbox";

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

	printk("[AURA_INBOX] ene_8k41_parse_dt\n");

	//pdata->dongle = of_property_read_bool(np, "ene,inbox-dongle");
	//printk("[AURA_INBOX] dongle : %d\n", pdata->dongle);

	//pdata->int_gpio = of_get_named_gpio_flags(np, "ene8k41,int-gpio", 0, &pdata->int_flags);
	//printk("[AURA_INBOX] int_gpio : %d\n", pdata->int_gpio);

	pdata->power_gpio = of_get_named_gpio_flags(np, "ene8k41,power-gpio", 0, &pdata->power_flags);
	printk("[AURA_INBOX] power_gpio : %d\n", pdata->power_gpio);

	/*if (pdata->dongle) {
		pdata->logo_5v_en = -2;
		pdata->regulator_vdd_vmin = 0;
		pdata->regulator_vdd_vmax = 0;
		pdata->regulator_current = 0;
		goto pinctrl;
	}*/

	//AURA VDD L22
	/*pdata->regulator_vdd = regulator_get(dev, "vdd");
	if (IS_ERR(pdata->regulator_vdd)) {
		dev_err(dev,
				"%s: Failed to get regulator vdd\n",
				__func__);
		retval = PTR_ERR(pdata->regulator_vdd);
	}

	retval = of_property_read_u32(np, "ene,vdd-current", &(pdata->regulator_current));
	if (retval < 0) {
		dev_err(dev,
				"%s: Failed to get regulator vdd current\n",
				__func__);
	}
	retval = of_property_read_u32_array(np, "ene,vdd-voltage", voltage_supply, 2);
	if (retval < 0) {
		dev_err(dev,
				"%s: Failed to get regulator vdd voltage\n",
				__func__);
	}

	pdata->regulator_vdd_vmin = voltage_supply[0];
	pdata->regulator_vdd_vmax = voltage_supply[1];

	printk("[AURA_INBOX] regulator_current : %d\n", pdata->regulator_current);

	pdata->logo_5v_en = of_get_named_gpio_flags(np, "ene,logo_5p0_en", 0, NULL);

pinctrl:
	printk("[AURA_INBOX] Get the pinctrl node \n");*/
	// Get the pinctrl node
	/*pdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(pdata->pinctrl)) {
	     dev_err(dev, "%s: Failed to get pinctrl\n", __func__);
	}

	printk("[AURA_INBOX] Get the active setting \n");*/
	// Get the active setting
	/*if (pdata->dongle) {
		if (gDongleType == 1){
			printk("[AURA_INBOX] Get inbox_aura_en \n");
			pdata->pins_active = pinctrl_lookup_state(pdata->pinctrl, "inbox_aura_en");
			if (IS_ERR_OR_NULL(pdata->pins_active)) {
				dev_err(dev, "%s: Failed to get pinctrl state active\n", __func__);
			}
		} else {
			printk("[AURA_DT] Get dt_aura_check \n");
			pdata->pins_active = pinctrl_lookup_state(pdata->pinctrl, "dt_aura_check");
			if (IS_ERR_OR_NULL(pdata->pins_active)) {
				dev_err(dev, "%s: Failed to get pinctrl state active\n", __func__);
			}
		}
	}else {
		printk("[AURA_INBOX] Get logo_5p0_en \n");
		pdata->pins_active = pinctrl_lookup_state(pdata->pinctrl, "logo_5p0_en");
		if (IS_ERR_OR_NULL(pdata->pins_active)) {
			dev_err(dev, "%s: Failed to get pinctrl state active\n", __func__);
		}
	}

	// Set the active setting
	printk("[AURA_INBOX] set the active state\n");
	retval = pinctrl_select_state(pdata->pinctrl, pdata->pins_active);
	if (retval)
		dev_err(dev, "%s: pinctrl_select_state retval:%d\n", __func__, retval);*/

	return 0;
}

static int ene_8k41_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;//, retry = 0;
	//unsigned char buf[128] = {0};
	//int readlen = 0;
	//unsigned char data[4] = {0};
	unsigned char *fw_buf;
	int fw_size;//, val;
	struct ene_8k41_platform_data *platform_data;

	printk("[AURA_INBOX] ene_8k41_probe.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
	goto exit_check_functionality_failed;
	} else
		printk("[AURA_INBOX] I2C function test pass\n");

	printk("[AURA_INBOX] client->addr : 0x%x\n", client->addr);
	//printk("[AURA_INBOX] check data[0]: 0x%x\n", data[0]);

	platform_data = devm_kzalloc(&client->dev, sizeof(struct ene_8k41_platform_data), GFP_KERNEL);
	if (!platform_data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, platform_data);
// Parse platform data from dtsi
	err = ene_8k41_parse_dt(&client->dev, platform_data);
	if (err) {
		printk("[AURA_INBOX] ene_8k41_parse_dt get fail !!!\n");
		return -ENOMEM;
	}

	printk("[AURA_INBOX] ENE 8k41 power on.\n");
	err = gpio_direction_output(platform_data->power_gpio, 1);
	if (err)
		printk("[AURA_INBOX] power_gpio output high, err %d\n", err);

// Set ENE 8K41 power
	/*if(gpio_is_valid(platform_data->power_gpio) && platform_data->dongle) {
		err = gpio_request(platform_data->power_gpio, "8k41_power_gpio");
		if (err)
			printk("[AURA_INBOX] 8k41_power_gpio gpio_request, err %d\n", err);

		if (gDongleType == 1){
			printk("[AURA_INBOX] ENE 8k41 power on.\n");
			err = gpio_direction_output(platform_data->power_gpio, 1);
			if (err)
				printk("[AURA_INBOX] power_gpio output high, err %d\n", err);
		}else {
			printk("[AURA_DT] check DT AURA VDD\n");
			err = gpio_direction_input(platform_data->power_gpio);
			if (err)
				printk("[AURA_DT] power_gpio input, err %d\n", err);

			while(retry < 5){
				val = gpio_get_value(platform_data->power_gpio);
				printk("[AURA_DT] power_gpio[%d] :0x%x\n", platform_data->power_gpio, val);
				if(val)
					break;

				retry++;
				msleep(500);
			}
		}
	}*/

// Set ENE 8K41 int only for proto type
/*
	if(gpio_is_valid(platform_data->int_gpio)) {
		err = gpio_request(platform_data->int_gpio, "8k41_int_gpio");
		if (err)
			printk("[AURA_INBOX] 8k41_int_gpio gpio_request, err %d\n", err);

		err = gpio_direction_input(platform_data->int_gpio);
		if (err)
			printk("[AURA_INBOX] gpio_direction_intput, err %d\n", err);
	}
*/
	// Set logo 5V enable pin for SR1 & after
	/*if (!platform_data->dongle) {
		err = gpio_request(platform_data->logo_5v_en, "logo_5v_en");
		if (err)
			printk("[AURA_INBOX] logo_5v_en gpio_request, err %d\n", err);

		printk("[AURA_INBOX] logo_5v_en default off.\n");
		err = gpio_direction_output(platform_data->logo_5v_en, 0);
		if (err)
			printk("[AURA_INBOX] logo_5v_en output high, err %d\n", err);*/

	// Set PM845 L22 to ENE 8K41 VDD

		/*printk("[AURA_INBOX] set VDD %d uA\n", platform_data->regulator_current);
		err = regulator_set_load(platform_data->regulator_vdd,
				platform_data->regulator_current);
		if (err < 0) {
			dev_err(&client->dev,
					"%s: Failed to set regulator current vdd\n",
					__func__);
		}

		printk("[AURA_INBOX] set VDD 3.3V\n");
		err = regulator_set_voltage(platform_data->regulator_vdd,
				platform_data->regulator_vdd_vmin,
				platform_data->regulator_vdd_vmax);
		if (err < 0) {
			dev_err(&client->dev,
					"%s: Failed to set regulator voltage vdd\n",
					__func__);
		}

		err = regulator_enable(platform_data->regulator_vdd);
		if (err) {
			dev_err(&client->dev,
				"%s: Failed to enable regulator vdd\n",
				__func__);
		}
	}*/

// Wait 0.5s for IC power on.\n");
	msleep(500);
	printk("[AURA_INBOX] ==========\n");

// Check FW
	platform_data->fw_version = ene_CheckFirmwareVer(client);
	printk("[AURA_INBOX] FW VER : 0x%x\n", platform_data->fw_version);

	//if ( platform_data->fw_version < 0x5) { //disable auto update in kernel space
	if (0){
		printk("[AURA_INBOX] Start auto FW update.");

		// get fs_size
		fw_size = ene_GetFirmwareSize(FW_PATH);
		if (fw_size <= 0) {
			printk("[AURA_INBOX] get fwsize error %d.\n", fw_size);
			goto skip_auto_update;
		} else
			printk("[AURA_INBOX] fw_size %d\n", fw_size);

		// set fw_buf
		fw_buf = kmalloc(fw_size+1 ,GFP_ATOMIC);

		// read FW content
		if (ene_ReadFirmware(FW_PATH, fw_buf)) {
			printk("[AURA_INBOX] ERROR: request_firmware failed\n");
			kfree(fw_buf);
		}
	}

skip_auto_update:
/*
// Setting PWM inverse
	printk("[AURA_INBOX] set PWM inverse, REG[0xF207] = 0x8\n");
	err = ene_8k41_write_bytes(client, 0xF207, 0x8);
	if (err !=1)
		printk("[AURA_INBOX] ene_8k41_write_bytes:err %d\n", err);

	err = ene_8k41_read_bytes(client, 0xF207, data);
	if (err != 2)
		printk("[AURA_INBOX] ene_8k41_read_bytes:err %d\n", err);

	printk("[AURA_INBOX] REG[0xF207]: 0x%x\n", data[0]);
	printk("[AURA_INBOX] ==========\n");
*/

// Register sys class
	err = aura_sync_register(&client->dev, platform_data);
	if (err) {
		printk("[AURA_INBOX] Failed to register LED device: %d\n", err);
		goto unled;
	}
	err = sysfs_create_group(&platform_data->led.dev->kobj, &pwm_attr_group);
	if (err)
			goto unled;

	mutex_init(&platform_data->ene_mutex);
	platform_data->current_mode = 0x0;
	platform_data->suspend_state = false;
	platform_data->FW_update_done = false;

// Default Calibration Data
	/*if (gDongleType == 3){
		platform_data->RED_MAX = 80;
		platform_data->GREEN_MAX = 80;
		platform_data->BLUE_MAX = 80;
	}else {*/
		platform_data->RED_MAX = 255;
		platform_data->GREEN_MAX = 255;
		platform_data->BLUE_MAX = 255;
	//}

// Set global variable
	g_pdata = platform_data;

	printk("[AURA_INBOX] ene_8k41_probe done.\n");
	return 0;

unled:
	aura_sync_unregister(platform_data);
	printk("[AURA_INBOX] ENE 8K41 power off.\n");
	err = gpio_direction_output(platform_data->power_gpio, 0);
	if (err)
		printk("[AURA_INBOX] power_gpio output high, err %d\n", err);
exit_check_functionality_failed:
	printk("[AURA_INBOX] ene_8k41_probe fail !!!\n");
	return err;
}

static int ene_8k41_remove(struct i2c_client *client)
{
	int err = 0;
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);

// unregister
	printk("[AURA_INBOX] sysfs_remove_group\n");
	sysfs_remove_group(&platform_data->led.dev->kobj, &pwm_attr_group);

	printk("[AURA_INBOX] aura_sync_unregister\n");
	aura_sync_unregister(platform_data);

	if(gpio_is_valid(platform_data->power_gpio)) {
	// power off
		printk("[AURA_INBOX] set GPIO[%d] output low\n", platform_data->power_gpio);
		err = gpio_direction_output(platform_data->power_gpio, 0);
		if (err)
			printk("[AURA_INBOX] power_gpio output low, err %d\n", err);

	// free gpio
		printk("[AURA_INBOX] Free GPIO[%d].\n", platform_data->power_gpio);
		gpio_free(platform_data->power_gpio);
	}

// free pointer
//	printk("[AURA_INBOX] Free platform_data\n");
//	kfree(platform_data);

	mutex_destroy(&platform_data->ene_mutex);
	printk("[AURA_INBOX] ene_8k41_remove : err %d\n", err);
	return 0;
}

int ene_8k41_inbox_suspend(struct device *dev)
{
	int err = 0;

	printk("[AURA_INBOX] ene_8k41_inbox_suspend : current_mode : 0x%x\n", g_pdata->current_mode);

	if(!g_pdata->current_mode){
		//printk("[AURA_INBOX] Disable VDD.\n");
		if ( gpio_is_valid(g_pdata->power_gpio) )
			gpio_set_value(g_pdata->power_gpio, 0);
	}
	printk("[AURA_INBOX] power_gpio[%d] :0x%x\n", g_pdata->power_gpio, gpio_get_value(g_pdata->power_gpio));
	g_pdata->suspend_state = true;

	return err;
}

int ene_8k41_inbox_resume(struct device *dev)
{
	int err = 0;

	printk("[AURA_INBOX] ene_8k41_inbox_resume : current_mode : 0x%x\n", g_pdata->current_mode);

	if(!g_pdata->current_mode){
		//printk("[AURA_INBOX] Enable VDD.\n");
		if ( gpio_is_valid(g_pdata->power_gpio) )
			gpio_set_value(g_pdata->power_gpio, 1);

	}
	printk("[AURA_INBOX] power_gpio[%d] :0x%x\n", g_pdata->power_gpio, gpio_get_value(g_pdata->power_gpio));
	g_pdata->suspend_state = false;

	return err;
}

static const struct i2c_device_id ene_8k41_id[] = {
	{ "ene_8k41_i2c", 0},
	{},
};
//MODULE_DEVICE_TABLE(i2c, ene_8k41_id);

static const struct dev_pm_ops ene_8k41_pm_ops = {
	.suspend	= ene_8k41_inbox_suspend,
	.resume	= ene_8k41_inbox_resume,
};

#ifdef CONFIG_OF
static const struct of_device_id ene_match_table[] = {
	{ .compatible = "ene8k41_inbox",},
	{ },
};
#else
#define ene_match_table NULL
#endif

static struct i2c_driver ene_8k41_driver = {
	.driver		= {
		.name		= "ene8k41_inbox",
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
		printk("[AURA_INBOX] ENE 8k41 driver int failed.\n");
	else
		printk("[AURA_INBOX] ENE 8k41 driver int success.\n");

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
