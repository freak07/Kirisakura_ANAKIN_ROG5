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

#define FW_PATH "/asusfw/aura_sync/ENE-8K41-aura-V7.bin"

static bool suspend_vdd_on = false;
#define FLASH_BUF_LEN 0x10

static struct ene_8k41_platform_data *g_pdata;
static bool bumper_enable = false;
static bool CSCmode = false;

// For control Bumper LED
extern enum DEVICE_HWID g_ASUS_hwID;
//extern void bumper_vdd_switch(u32 val);
//extern int lid2_status;
static int lid2_status=1;

// For Charger mode
//extern bool g_Charger_mode;
static bool g_Charger_mode=false;

//#define WAKELOCK_HOLD_TIME 200 /* in ms */
//static struct wakeup_source ene_wakelock;

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

	//printk("[AURA_SYNC] ene_8k41_read_bytes : addr: 0x%x, buf[1] : 0x%x, buf[2] : 0x%x\n", (addr & 0xFFFF), buf[1], buf[2]);
	err = i2c_write_bytes(client, buf, 3);	//set register address
	if (err !=1)
		printk("[AURA_SYNC] i2c_write_bytes:err %d\n", err);

	buf[0] = 0x81;
	err = i2c_read_bytes(client, buf, 1, data, 1);	//send read command
	if (err != 2)
		printk("[AURA_SYNC] i2c_read_bytes:err %d\n", err);

	return err;
}

static int ene_8k41_write_bytes(struct i2c_client *client, short addr, char value)
{
	int err = 0;
	unsigned char buf[16] = {0};

	buf[0] = 0x00;
	buf[1] = (addr >> 8) & 0xFF;
	buf[2] = addr & 0xFF;

	//printk("[AURA_SYNC] ene_8k41_write_bytes : addr: 0x%x, buf[1] : 0x%x, buf[2] : 0x%x, value : 0x%x\n", (addr & 0xFFFF), buf[1], buf[2], value);
	err = i2c_write_bytes(client, buf, 3);	//set register address
	if (err !=1)
		printk("[AURA_SYNC] i2c_write_bytes:err %d\n", err);

	buf[0] = 0x01;
	buf[1] = value;
	
	err = i2c_write_bytes(client, buf, 2);	//set register address
	if (err !=1)
		printk("[AURA_SYNC] i2c_write_bytes:err %d\n", err);
	
	return err;
}

static int ene_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];

	printk("[AURA_SYNC] ene_GetFirmwareSize.\n");
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
	int ret = 0;

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
	pos = 0;
	ret = kernel_read(pfile, fw_buf, fsize, &pos);
	if (ret<0)
		pr_err("Read file %s fail\n", filepath);

	filp_close(pfile, NULL);
	return 0;
}

static char ene_CheckFirmwareVer(struct i2c_client *client)
{
	int err = 0;
	unsigned char data[2] = {0};

	err = ene_8k41_read_bytes(client, 0x80C0, data);
	if (err != 2){
		printk("[AURA_SYNC] fw_check:err %d\n", err);
		data[0] = 0xFF;
	}

	return (u8) data[0];
}

static int ene_UpdateFirmware(struct i2c_client *client, char *fw_buf)
{
	int err = 0;
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	unsigned char data[2] = {0};
	unsigned char *buf;
	//unsigned char tmp[129] = {0};
	//int  i = 0;
	short addr;
	int retry = 0;

	err = ene_8k41_read_bytes(client, 0xFF8F, data);
	if (err != 2)
		printk("[AURA_SYNC] Check REG[0xFF8F]:err %d\n", err);

	printk("[AURA_SYNC] REG[0xFF8F] : 0x%x\n", data[0]);

	if( (u8)(data[0]) <= 0x40){
		platform_data->fw_version = ene_CheckFirmwareVer(client);
		printk("[AURA_SYCN] FW VER : 0x%x\n", platform_data->fw_version);
		//if ( platform_data->fw_version == 0x0) { // rember change condition.
		if (0) {
			printk("[AURA_SYNC] Don't need FW update.\n");
			return 0;
		}
	} else
		printk("[ASUS_SYNC] FW is abnormal !!!");

	printk("[AURA_SYNC] Start ene_UpdateFirmware\n");

	buf = kmalloc( FLASH_BUF_LEN+1 , GFP_DMA | GFP_KERNEL);

	err = ene_8k41_write_bytes(client, 0xF018, 0xCE);
	if (err !=1)
		printk("[AURA_SYNC] REG[F010]: write err %d\n", err);

	err = ene_8k41_read_bytes(client, 0xF018, data);
	if (err != 2)
		printk("[AURA_SYNC] REG[0xF018]: read err %d\n", err);

	printk("[AURA_SYNC] REG[0xF018]: 0x%x\n", data[0]);

// (1) Reset 8051
	printk("[AURA_SYNC] Reset 8051 =====\n");
	err = ene_8k41_write_bytes(client, 0xF010, 0x5);
	if (err !=1)
		printk("[AURA_SYNC] REG[0xF010]: write err %d\n", err);

	msleep(500);

// (2) Read & Fill OSC32M freq.
	printk("[AURA_SYNC] Read & Fill OSC32M freq =====\n");
	err = ene_8k41_write_bytes(client, 0xF808, 0xF0);
	if (err !=1)
		printk("[AURA_SYNC] REG[0xF808]: write err %d\n", err);
	err = ene_8k41_write_bytes(client, 0xF809, 0x01);
	if (err !=1)
		printk("[AURA_SYNC] REG[0xF809]: write err %d\n", err);
	err = ene_8k41_write_bytes(client, 0xF807, 0x90);
	if (err !=1)
		printk("[AURA_SYNC] REG[0xF807]: write err %d\n", err);

	err = ene_8k41_read_bytes(client, 0xF80B, data);
	if (err != 2)
		printk("[AURA_SYNC] REG[0xF80B]: read err %d\n", err);

	printk("[AURA_SYNC] REG[0xF80B]: 0x%x\n", data[0]);
	err = ene_8k41_write_bytes(client, 0xF806, data[0]);
	if (err != 1)
		printk("[AURA_SYNC] REG[0xF806]: write err %d\n", err);

// (3) Fill program & Erase timing
	printk("[AURA_SYNC] Fill program & Erase timing =====\n");
	err = ene_8k41_write_bytes(client, 0xF815, 0x10);
	if (err !=1)
		printk("[AURA_SYNC] REG[0xF815]: write err %d\n", err);
	err = ene_8k41_write_bytes(client, 0xF816, 0x11);
	if (err !=1)
		printk("[AURA_SYNC] REG[0xF816]: write err %d\n", err);
	err = ene_8k41_write_bytes(client, 0xF817, 0x06);
	if (err !=1)
		printk("[AURA_SYNC] REG[0xF817]: write err %d\n", err);
	err = ene_8k41_write_bytes(client, 0xF818, 0x07);
	if (err !=1)
		printk("[AURA_SYNC] REG[0xF818]: write err %d\n", err);

// (4) Erase
	printk("[AURA_SYNC] Erase page from 0x0000 to 0x3F80 =====\n");
	for (addr = 0x0000 ; addr <= 0x3F80 ; addr+= 0x80) {

		//printk("[AURA_SYNC] set addr : 0x%04x\n", addr);
		err = ene_8k41_write_bytes(client, 0xF808, (addr & 0xFF));				//	Addr_L
		if (err !=1)
			printk("[AURA_SYNC] REG[0xF808]: write err %d\n", err);
		err = ene_8k41_write_bytes(client, 0xF809, (addr >> 8) & 0xFF);		//	Addr_H
		if (err !=1)
			printk("[AURA_SYNC] REG[0xF809]: write err %d\n", err);

		err = ene_8k41_write_bytes(client, 0xF807, 0x20);	// erase page cmd
		if (err !=1)
			printk("[AURA_SYNC] REG[0xF807]: write err %d\n", err);

		while(1){	 // check cmd finished. 0xF800[7] = 1
			msleep(50);
			data[0] = 0x0;
			err = ene_8k41_read_bytes(client, 0xF800, data);
			if (err != 2) {
				printk("[AURA_SYNC] REG[0xF800]: read err %d\n", err);
			}
			if ( ((data[0] >> 7) & 0x1) == 1 ){
				//printk("[AURA_SYNC] check REG[0xF800] pass, 0x%02x\n", (data[0]));
				retry=0;
				break;
			}
			printk("[AURA_SYNC] check REG[0xF800], 0x%02x\n", data[0]);
			retry++;
			if (retry > 10) {
				printk("[AURA_SYNC] Fail at : 0x%04x\n", addr);
				printk("[AURA_SYNC] retry too many times: %d, Force exit!!!\n", retry);
				kfree(buf);
				return -1;
			}
		}
	}
	printk("[AURA_SYNC] erase page finished. =====\n");
	
// (5) Program
	printk("[AURA_SYNC] Program FW from 0x0000 to 0x%04x =====\n", (0x4000 - FLASH_BUF_LEN));
	for (addr = 0x0000 ; addr <= (0x4000 - FLASH_BUF_LEN) ; addr = addr + FLASH_BUF_LEN) {

		//printk("[AURA_SYNC] set addr : 0x%04x\n", addr);
		err = ene_8k41_write_bytes(client, 0xF808, (addr & 0xFF));				//	Addr_L
		if (err !=1)
			printk("[AURA_SYNC] REG[0xF808]: write err %d\n", err);
		err = ene_8k41_write_bytes(client, 0xF809, (addr >> 8) & 0xFF);		//	Addr_H
		if (err !=1)
			printk("[AURA_SYNC] REG[0xF809]: write err %d\n", err);

		err = ene_8k41_write_bytes(client, 0xF807, 0x80);	// 3. Clear HVPL data
		if (err !=1)
			printk("[AURA_SYNC] REG[0xF807]: write err %d\n", err);

		while(1){	 // 4. check cmd finished 0xF800[7] = 1
			msleep(50);
			data[0] = 0x0;
			err = ene_8k41_read_bytes(client, 0xF800, data);
			if (err != 2) {
				printk("[AURA_SYNC] REG[0xF800]: read err %d\n", err);
			}
			if ( ((data[0] >> 7) & 0x1) == 1 ){
				//printk("[AURA_SYNC] check REG[0xF800] pass, 0x%02x\n", (data[0]));
				retry = 0;
				break;
			}
			printk("[AURA_SYNC] check REG[0xF800], 0x%02x\n", (data[0]));
			retry++;
			if (retry > 10) {
				printk("[AURA_SYNC] Fail at : 0x%04x\n", addr);
				printk("[AURA_SYNC] retry too many times: %d, Force exit!!!\n", retry);
				kfree(buf);
				return -1;
			}
		}

		// 5. Set address to 0xF80A
		buf[0] = 0x00;
		buf[1] = 0xF8;	//Addr_H
		buf[2] = 0x0A;	//Addr_L
		err = i2c_write_bytes(client, buf, 3);	//set register address
		if (err !=1)
			printk("[AURA_SYNC] i2c_write_bytes:err %d\n", err);

		// 6. Send data to buffer
		buf[0] = 0x05;
		memcpy( (buf+1), fw_buf + addr, FLASH_BUF_LEN*sizeof(char)); // copy fw_buf to buf

		err = i2c_write_bytes(client, buf, (FLASH_BUF_LEN+1)*sizeof(char));	 // 128 byte + 1 cmd = 129
		if (err !=1)
			printk("[AURA_SYNC] i2c_write_bytes:err %d\n", err);

		// 7. Set address to 0xF807
		buf[0] = 0x00;
		buf[1] = 0xF8;	//Addr_H
		buf[2] = 0x07;	//Addr_L
		err = i2c_write_bytes(client, buf, 3);	//set register address
		if (err !=1)
			printk("[AURA_SYNC] i2c_write_bytes:err %d\n", err);

		// 8. Program page
		buf[0] = 0x05;
		buf[1] = 0x70;
		err = i2c_write_bytes(client, buf, 2);
		if (err !=1)
			printk("[AURA_SYNC] i2c_write_bytes:err %d\n", err);

		while(1){	 // 9. check cmd finished 0xF800[7] = 1
			msleep(50);
			data[0] = 0x0;
			err = ene_8k41_read_bytes(client, 0xF800, data);
			if (err != 2) {
				printk("[AURA_SYNC] REG[0xF800]: read err %d\n", err);
			}
			if ( ((data[0] >> 7) & 0x1) == 1 ){
				//printk("[AURA_SYNC] check REG[0xF800] pass, 0x%02x\n", (data[0]));
				retry = 0;
				break;
			}
			printk("[AURA_SYNC] check REG[0xF800], 0x%02x\n", (data[0]));
			retry++;
			if (retry > 10) {
				printk("[AURA_SYNC] Fail at : 0x%04x\n", addr);
				printk("[AURA_SYNC] retry too many times: %d, Force exit!!!\n", retry);
				kfree(buf);
				return -1;
			}
		}
	}
	
////////////////////////////
/*
	printk("[AURA_SYNC] read FW in 8K41 rom +++\n");
	for (addr = 0x0000 ; addr <= 0x0080 ; addr = addr + 0x80) {
		printk("[AURA_SYNC] set addr : 0x%04x\n", addr);
		err = ene_8k41_write_bytes(client, 0xF808, (addr & 0xFF));				//	Addr_L
		if (err !=1)
			printk("[AURA_SYNC] REG[0xF808]: write err %d\n", err);
		err = ene_8k41_write_bytes(client, 0xF809, (addr >> 8) & 0xFF);		//	Addr_H
		if (err !=1)
			printk("[AURA_SYNC] REG[0xF809]: write err %d\n", err);

		// 5. Set address to 0xF80B
		buf[0] = 0x00;
		buf[1] = 0xF8;	//Addr_H
		buf[2] = 0x0B;	//Addr_L
		err = i2c_write_bytes(client, buf, 3);	//set register address
		if (err !=1)
			printk("[AURA_SYNC] i2c_write_bytes:err %d\n", err);

		buf[0] = 0x06;
		err = i2c_read_bytes(client, buf, 1, tmp, 128);	//set register address
		if (err !=2)
			printk("[AURA_SYNC] i2c_read_bytes:err %d\n", err);

		for (i=0; i<128 ; i++)
			printk("[AURA_SYNC] tmp[%d]:0x%x\n", i, tmp[i]);
	}
	printk("[AURA_SYNC] read FW in 8K41 rom ---\n");
*/
////////////////////////////

// (9) Restart 8051
	printk("[AURA_SYNC] Restart 8051.\n");
	err = ene_8k41_write_bytes(client, 0xF010, 0x04);
	if (err !=1)
		printk("[AURA_SYNC] REG[0xF807]: write err %d\n", err);

	msleep(500);

	err = ene_8k41_write_bytes(client, 0xF018, 0x00);
	if (err !=1)
		printk("[AURA_SYNC] REG[F010]: write err %d\n", err);

	platform_data->FW_update_done = true;
	printk("[AURA_SYNC] ene_UpdateFirmware finished. %d\n", platform_data->FW_update_done);

	kfree(buf);
	return 0;
}

void bumper_switch(u32 val)
{
	unsigned char data[4] = {0};
	u8 tmp = 0;
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);

	if (CSCmode)
		printk("[AURA_SYNC] bumper_switch. %d, CSCmode %d\n", val, CSCmode);

	if(val>0) {
		if(!lid2_status && !CSCmode){
			printk("[ASUS_SYNC] lid2_status:0x%x, no need open Bumper.\n", lid2_status);
			mutex_unlock(&g_pdata->ene_mutex);
			return;
		}

		printk("[AURA_SYNC] Bumper ON\n");
		bumper_enable = true;

		err = ene_8k41_read_bytes(g_pdata->client, 0xF203, data);
		if (err != 2)
			printk("[AURA_SYNC] led2_on_store:err %d\n", err);

		//printk("[AURA_SYNC] REG[0xF203] = 0x%x, bit1 is %d\n", data[0], data[0] & 0x01 );
		if ( !(data[0] & 0x01) ){
			tmp = (data[0] & 0x21) | 0x01;
			err = ene_8k41_write_bytes(g_pdata->client, 0xF203, tmp);
			if (err !=1)
				printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);
		}

		data[0] = 0;
		tmp = 0;

		err = ene_8k41_read_bytes(g_pdata->client, 0xF208, data);
		if (err != 2)
			printk("[AURA_SYNC] led2_on_store:err %d\n", err);

		//printk("[AURA_SYNC] REG[0xF208] = 0x%x, bit1 is %d\n", data[0], data[0] & 0x01 );
		if ( !(data[0] & 0x01) ){
			tmp = (data[0] & 0x21) | 0x01;
			err = ene_8k41_write_bytes(g_pdata->client, 0xF208, tmp);
			if (err !=1)
				printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);
		}

	}else {
		printk("[AURA_SYNC] Bumper OFF\n");
		bumper_enable = false;

		err = ene_8k41_read_bytes(g_pdata->client, 0xF203, data);
		if (err != 2)
			printk("[AURA_SYNC] led2_on_store:err %d\n", err);

		//printk("[AURA_SYNC] REG[0xF203] = 0x%x, bit1 is %d\n", data[0], data[0] & 0x01 );
		if ( data[0] & 0x01 ){
			tmp = (data[0] & 0x21) & 0xFE;
			err = ene_8k41_write_bytes(g_pdata->client, 0xF203, tmp);
			if (err !=1)
				printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);
		}

		data[0] = 0;
		tmp = 0;

		err = ene_8k41_read_bytes(g_pdata->client, 0xF208, data);
		if (err != 2)
			printk("[AURA_SYNC] led2_on_store:err %d\n", err);

		//printk("[AURA_SYNC] REG[0xF208] = 0x%x, bit1 is %d\n", data[0], data[0] & 0x01 );
		if ( data[0] & 0x01 ){
			tmp = (data[0] & 0x21) & 0xFE;
			err = ene_8k41_write_bytes(g_pdata->client, 0xF208, tmp);
			if (err !=1)
				printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);
		}

	}
	mutex_unlock(&g_pdata->ene_mutex);
	return;
}
EXPORT_SYMBOL(bumper_switch);

static ssize_t red_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;

	//printk("[AURA_SYNC] red_pwm_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return ret;

	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->RED_MAX), 255);

	mutex_lock(&g_pdata->ene_mutex);
	//printk("[AURA_SYNC] %s : client->addr : 0x%x,  reg_val : 0x%x.\n", __func__, client->addr, reg_val);
	g_red = reg_val;

	if(platform_data->RED_MAX!=255)
		printk("[AURA_SYNC] red tmp %d\n", tmp);

	err = ene_8k41_write_bytes(client, 0x8010, tmp);
	if (err !=1)
		printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);

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
		printk("[AURA_SYNC] red_pwm_show:err %d\n", err);

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

	//printk("[AURA_SYNC] green_pwm_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return ret;

	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->GREEN_MAX), 255);

	mutex_lock(&g_pdata->ene_mutex);
	//printk("[AURA_SYNC] %s : client->addr : 0x%x,  reg_val : 0x%x.\n", __func__, client->addr, reg_val);
	g_green = reg_val;

	if(platform_data->GREEN_MAX != 255)
		printk("[AURA_SYNC] green tmp %d\n", tmp);

	err = ene_8k41_write_bytes(client, 0x8012, tmp);
	if (err !=1)
		printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t green_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_8k41_read_bytes(client, 0x8012, data);
	if (err != 2)
		printk("[AURA_SYNC] green_pwm_show:err %d\n", err);

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

	//printk("[AURA_SYNC] blue_pwm_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return ret;

	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->BLUE_MAX), 255);

	mutex_lock(&g_pdata->ene_mutex);
	//printk("[AURA_SYNC] %s : client->addr : 0x%x,  reg_val : 0x%x.\n", __func__, client->addr, reg_val);
	g_blue = reg_val;

	if(platform_data->BLUE_MAX != 255)
		printk("[AURA_SYNC] blue tmp %d\n", tmp);

	err = ene_8k41_write_bytes(client, 0x8011, tmp);
	if (err !=1)
		printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t blue_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_8k41_read_bytes(client, 0x8011, data);
	if (err != 2)
		printk("[AURA_SYNC] blue_pwm_show:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t apply_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	u32 val;
	int err = 0;
	ssize_t ret;

	if (g_pdata->emulate){
		printk("[AURA_SYNC] emulate processing, block normal control\n");
		return count;
	}

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	mutex_lock(&g_pdata->ene_mutex);
	if (val > 0){
		//printk("[AURA_SYNC] Send apply. RGB:%d %d %d, mode:%d, speed:%d, led_on:%d, lid2_status:%d, bumper_enable %d\n", g_red, g_green, g_blue, g_mode, g_speed, g_led_on, lid2_status, bumper_enable);
		printk("[AURA_SYNC] Send apply. RGB:%d %d %d, mode:%d, speed:%d, led_on:%d, led2_on:%d\n", g_red, g_green, g_blue, g_mode, g_speed, g_led_on, g_led2_on);
		err = ene_8k41_write_bytes(client, 0x802F, 0x1);
		if (err !=1)
			printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);
	} else
		printk("[AURA_SYNC] No send apply cmd.\n");

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
		printk("[AURA_SYNC] apply_show:err %d\n", err);

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

	//printk("[AURA_SYNC] mode_store.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	mutex_lock(&g_pdata->ene_mutex);
	//printk("[AURA_SYNC] %s : client->addr : 0x%x,  val : 0x%x.\n", __func__, client->addr, val);
	g_mode = val;

	err = ene_8k41_write_bytes(client, 0x8021, val);
	if (err !=1){
		printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);
		mutex_unlock(&g_pdata->ene_mutex);
		return count;
	}

	platform_data->current_mode = (u8)val;
	//ASUSEvtlog("[AURA_SYNC] current_mode : %d\n", platform_data->current_mode);

	mutex_unlock(&g_pdata->ene_mutex);

	printk("[AURA_SYNC] lid2_status = 0x%x.\n", lid2_status);
	if (CSCmode){
		printk("[AURA_SYNC] CSCmode = 0x%d.\n", CSCmode);
	} else {
		if (!lid2_status && bumper_enable)
			bumper_switch(0);
	}

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
		printk("[AURA_SYNC] mode_show:err %d\n", err);
		mutex_unlock(&g_pdata->ene_mutex);
		return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
	}

	platform_data->current_mode = data[0];

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

//function for aura enable gpio+++
static ssize_t vdd2_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	u32 val;
	ssize_t ret;

	if (platform_data->aura_front_en == -ENOENT){
		printk("[AURA_SYNC] platform_data->aura_front_en NOT EXIST!!\n");
		return count;
	}

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if(val>0) {
		printk("[AURA_SYNC] VDD2 set HIGH\n");

		if ( gpio_is_valid(platform_data->aura_front_en) ) {
			if ( gpio_get_value(platform_data->aura_front_en) == 1 ){
				printk("[AURA_SYNC] VDD2 already HIGH\n");
				return count;
			}
			gpio_set_value(platform_data->aura_front_en, 1);
		}else {
			printk("[AURA_SYNC] aura_front_en is not vaild\n");
		}
	}else {
		printk("[AURA_SYNC] VDD2 set LOW\n");

		if ( gpio_is_valid(platform_data->aura_front_en) ) {
			if ( gpio_get_value(platform_data->aura_front_en) == 0 ){
				printk("[AURA_SYNC] VDD2 already LOW\n");
				return count;
			}
			gpio_set_value(platform_data->aura_front_en, 0);
		}else {
			printk("[AURA_SYNC] aura_front_en is not vaild\n");
		}
	}
	return count;
}

static ssize_t vdd2_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	int val = 0;

	if (platform_data->aura_front_en == -ENOENT){
		printk("[AURA_SYNC] platform_data->aura_front_en NOT EXIST!!\n");
		return 0;
	}

	if ( gpio_is_valid(platform_data->aura_front_en) ) {
		val = gpio_get_value(platform_data->aura_front_en);
		printk("[AURA_SYNC] aura_front_en[%d] :0x%x\n", platform_data->aura_front_en, val);
		return snprintf(buf, PAGE_SIZE,"%d\n", val);
	}else {
		printk("[AURA_SYNC] aura_front_en is not valid\n");
		return snprintf(buf, PAGE_SIZE,"aura_front_en is not valid\n");
	}
}
//function for aura enable gpio---
// ctrl reg test +++
/*
static int ene8k41_regulator_enable(void)
{
    int ret = 0;

    if(IS_ERR_OR_NULL(reg)){
        ret = PTR_ERR(reg);
        printk("[AURA_SYNC] Failed to get regulator ene_vcc %d\n", ret);
        return ret;
    }
    printk("[AURA_SYNC] regulator_set_load");
    ret = regulator_set_load(reg, 600000);
    if(ret < 0){
        printk("[AURA_SYNC] Failed to set load for ene_vcc reg %d\n", ret);
        return ret;
    }
    printk("[AURA_SYNC] regulator_enable");
    ret = regulator_enable(reg);
    if(ret){
        printk("[AURA_SYNC] Failed to enable ene_vcc reg %d\n", ret);
        return -1;
    }

    return ret;
}

static int ene8k41_regulator_disable(void)
{
	int ret = 0;

    if(IS_ERR_OR_NULL(reg)){
        ret = PTR_ERR(reg);
        printk("[AURA_SYNC] Failed to get regulator ene_vcc %d\n", ret);
        return ret;
    }

    if(regulator_is_enabled(reg) == 0){
        printk("[AURA_SYNC] ene_vcc reg is already disabled\n", ret);
        return ret;
    } else if(regulator_is_enabled(reg) < 0){
        printk("[AURA_SYNC] Failed to check regulator ene_vcc %d\n", ret);
        return ret;
    }

    ret = regulator_set_load(reg, 0);
    if(ret < 0){
        printk("[AURA_SYNC] Failed to set load for ene_vcc reg %d\n", ret);
        return ret;
    }

    ret = regulator_disable(reg);
    if(ret){
        printk("[AURA_SYNC] Failed to disable ene_vcc reg %d\n", ret);
        return -1;
    }

    return ret;
}
*/
// ctrl reg test ---

static ssize_t vdd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	//struct i2c_client *client = to_i2c_client(dev->parent);
	//struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);

	u32 val;
	ssize_t ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
	{
		printk("[AURA_SYNC] ret= %d\n",ret);
		return ret;
	}

	if(val>0) {
		printk("[AURA_SYNC] ene8k41_regulator_enable start");
		/*
		if(regulator_is_enabled(reg) > 0){
			printk("[AURA_SYNC] ene8k41_regulator is enabled ");
			return count;
		}
		ret = ene8k41_regulator_enable();
		if(ret < 0){
			printk("[AURA_SYNC] fail to enable ene8k41_regulator %d ",ret);
			return count;
		}
		*/
	}else {
		printk("[AURA_SYNC] ene8k41_regulator_disable start");
		/*
		ret = ene8k41_regulator_disable();
		if(ret < 0){
			printk("[AURA_SYNC] fail to disable ene8k41_regulator %d ",ret);
			return count;
		}
		*/
	}

	return count;

	/*u32 val;
	ssize_t ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if(val>0) {
		printk("[AURA_SYNC] VDD set HIGH\n");
		
		if ( gpio_is_valid(platform_data->aura_3p3_en) ) {
			if ( gpio_get_value(platform_data->aura_3p3_en) == 1 ){
				printk("[AURA_SYNC] VDD already HIGH\n");
				return count;
			}
			gpio_set_value(platform_data->aura_3p3_en, 1);
		}else {
			printk("[AURA_SYNC] aura_3p3_en is not vaild\n");
		}
	}else {
		printk("[AURA_SYNC] VDD set LOW\n");

		if ( gpio_is_valid(platform_data->aura_3p3_en) ) {
			if ( gpio_get_value(platform_data->aura_3p3_en) == 0 ){
				printk("[AURA_SYNC] VDD already LOW\n");
				return count;
			}
			gpio_set_value(platform_data->aura_3p3_en, 0);
		}else {
			printk("[AURA_SYNC] aura_3p3_en is not vaild\n");
		}
	}
	return count;*/
}

static ssize_t vdd_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev->parent);
	//struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	//int ret = 0;
	//val = regulator_disable(platform_data->vcc);
/*
	if(regulator_is_enabled(reg) > 0){
		printk("[AURA_SYNC] ene8k41_regulator is enabled ");
		return snprintf(buf, PAGE_SIZE,"%d\n", 1);
	}
	else {
		printk("[AURA_SYNC] ene8k41_regulator is not enabled ");
		return snprintf(buf, PAGE_SIZE,"%d\n", 0);
	}
*/
	/*
	if ( gpio_is_valid(platform_data->aura_3p3_en) ) {
		val = gpio_get_value(platform_data->aura_3p3_en);
		printk("[AURA_SYNC] aura_3p3_en[%d] :0x%x\n", platform_data->aura_3p3_en, val);
		return snprintf(buf, PAGE_SIZE,"%d\n", val);
	}else {
		printk("[AURA_SYNC] aura_3p3_en is not valid\n");
		return snprintf(buf, PAGE_SIZE,"aura_3p3_en is not valid\n");
	}*/
	
	return snprintf(buf, PAGE_SIZE,"vdd_show\n");
}

static ssize_t led_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	unsigned char data[4] = {0};
	u8 tmp = 0;
	u32 val;
	ssize_t ret;
	int err = 0;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	mutex_lock(&g_pdata->ene_mutex);
	if(val>0) {
		//printk("[AURA_SYNC] LED ON\n");
		g_led_on = 1;

		if (platform_data->aura_front_en != -ENOENT){
			printk("[AURA_SYNC] LED power on.\n");
			err = gpio_direction_output(platform_data->aura_front_en, 1);
			if (err)
				printk("[AURA_SYNC] aura_front_en output high, err %d\n", err);
		}else{
			printk("[AURA_SYNC] platform_data->aura_front_en NOT EXIST!!\n");
		}

		err = ene_8k41_read_bytes(client, 0xF203, data);
		if (err != 2)
			printk("[AURA_SYNC] led_on_store:err %d\n", err);

		//printk("[AURA_SYNC] REG[0xF203] = 0x%x, bit6 is %d\n", data[0], (data[0] & 0x20) >> 5);
		if ( !((data[0] & 0x20) >> 5) ){
			tmp = (data[0] & 0x21) | 0x20;
			err = ene_8k41_write_bytes(client, 0xF203, tmp);
			if (err !=1)
				printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);
		}

		data[0] = 0;
		tmp = 0;

		err = ene_8k41_read_bytes(client, 0xF208, data);
		if (err != 2)
			printk("[AURA_SYNC] led_on_store:err %d\n", err);

		//printk("[AURA_SYNC] REG[0xF208] = 0x%x, bit6 is %d\n", data[0], (data[0] & 0x20) >> 5);
		if ( !((data[0] & 0x20) >> 5) ){
			tmp = (data[0] & 0x21) | 0x20;
			err = ene_8k41_write_bytes(client, 0xF208, tmp);
			if (err !=1)
				printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);
		}
	}else {
		//printk("[AURA_SYNC] LED OFF\n");
		g_led_on = 0;

		err = ene_8k41_read_bytes(client, 0xF203, data);
		if (err != 2)
			printk("[AURA_SYNC] led_on_store:err %d\n", err);

		//printk("[AURA_SYNC] REG[0xF203] = 0x%x, bit6 is %d\n", data[0], (data[0] & 0x20) >> 5);
		if ( ((data[0] & 0x20) >> 5) ){
			tmp = (data[0] & 0x21) & 0xDF;
			err = ene_8k41_write_bytes(client, 0xF203, tmp);
			if (err !=1)
				printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);
		}

		data[0] = 0;
		tmp = 0;

		err = ene_8k41_read_bytes(client, 0xF208, data);
		if (err != 2)
			printk("[AURA_SYNC] led_on_store:err %d\n", err);

		//printk("[AURA_SYNC] REG[0xF208] = 0x%x, bit6 is %d\n", data[0], (data[0] & 0x20) >> 5);
		if ( ((data[0] & 0x20) >> 5) ){
			tmp = (data[0] & 0x21) & 0xDF;
			err = ene_8k41_write_bytes(client, 0xF208, tmp);
			if (err !=1)
				printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);
		}

		if (g_led2_on == 0) {
			if (platform_data->aura_front_en != -ENOENT){
				printk("[AURA_SYNC] LED power off.\n");
				err = gpio_direction_output(platform_data->aura_front_en, 0);
				if (err)
					printk("[AURA_SYNC] aura_front_en output high, err %d\n", err);
			}else{
				printk("[AURA_SYNC] platform_data->aura_front_en NOT EXIST!!\n");
			}
		}


	}
	mutex_unlock(&g_pdata->ene_mutex);

	return count;
}

static ssize_t led_on_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	int err = 0;
	unsigned char data[4] = {0};

	err = ene_8k41_read_bytes(client, 0xF208, data);
	if (err != 2)
		printk("[AURA_SYNC] led_on_show:err %d\n", err);

	printk("[AURA_SYNC] REG[0xF208] = 0x%x, bit6 is %d\n", data[0], (data[0] & 0x20) >> 5);

	return snprintf(buf, PAGE_SIZE,"%x\n", (u8)((data[0] & 0x20) >> 5) );
}

static ssize_t led2_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	//struct i2c_client *client = to_i2c_client(dev->parent);
	//struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	u32 val;
	ssize_t ret;
	//int err = 0;
	//__pm_wakeup_event(&ene_wakelock, WAKELOCK_HOLD_TIME);

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if(val>0) {
		g_led2_on = 1;

/*
		//printk("[AURA_SYNC] LED ON\n");
		if (platform_data->aura_front_en != -ENOENT){
			printk("[AURA_SYNC] LED power on.\n");
			err = gpio_direction_output(platform_data->aura_front_en, 1);
			if (err)
				printk("[AURA_SYNC] aura_front_en output high, err %d\n", err);
		}else{
			printk("[AURA_SYNC] platform_data->aura_front_en NOT EXIST!!\n");
		}
*/
	}else{
		g_led2_on = 0;
/*
		if (g_led_on == 0) {
			if (platform_data->aura_front_en != -ENOENT){
				printk("[AURA_SYNC] LED power off.\n");
				err = gpio_direction_output(platform_data->aura_front_en, 0);
				if (err)
					printk("[AURA_SYNC] aura_front_en output high, err %d\n", err);
			}else{
				printk("[AURA_SYNC] platform_data->aura_front_en NOT EXIST!!\n");
			}
		}
*/
	}

	bumper_switch(val);

	return count;
}

static ssize_t led2_on_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	int err = 0;
	unsigned char data[4] = {0};

	err = ene_8k41_read_bytes(client, 0xF208, data);
	if (err != 2)
		printk("[AURA_SYNC] led2_on_show:err %d\n", err);

	printk("[AURA_SYNC] REG[0xF208] = 0x%x, bit1 is %d\n", data[0], data[0] & 0x01 );

	return snprintf(buf, PAGE_SIZE,"%x\n", (u8)(data[0] & 0x01) );
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
	
	printk("[AURA_SYNC] fwname : %s\n", fw_name);

	// get fs_size
	fw_size = ene_GetFirmwareSize(fw_name);
	printk("[AURA_SYNC] fwsize %d\n", fw_size);
	
	// set fw_buf
	fw_buf = kmalloc(fw_size+1 ,GFP_ATOMIC);

	// read FW content
	if (ene_ReadFirmware(fw_name, fw_buf)) {
		printk("[AURA_SYNC] ERROR: request_firmware failed\n");
		kfree(fw_buf);
		return -EIO;
	}
	
	printk("[AURA_SYNC] fw_buf[0~5] : 0x%x 0x%x 0x%x 0x%x 0x%x\n", fw_buf[0], fw_buf[1], fw_buf[2], fw_buf[3], fw_buf[4]);

	// Make sure VDD is on
//	if ( gpio_is_valid(platform_data->aura_3p3_en) )
//		gpio_set_value(platform_data->aura_3p3_en, 1);

	// Start update FW
	mutex_lock(&g_pdata->ene_mutex);
	err = ene_UpdateFirmware(client, fw_buf);
	if(err)
		printk("[AURA_SYNC] ene_UpdateFirmware, err %d\n", err);

	// Update FW VER
	platform_data->fw_version = ene_CheckFirmwareVer(client);
	printk("[AURA_SYNC] FW VER : 0x%x\n", platform_data->fw_version);

	mutex_unlock(&g_pdata->ene_mutex);
	
	kfree(fw_buf);
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
		printk("[AURA_SYNC] fw_ver_show:err %d\n", err);
		mutex_unlock(&g_pdata->ene_mutex);
		return snprintf(buf, PAGE_SIZE,"i2c_error\n");
	}

	platform_data->fw_version = ene_CheckFirmwareVer(client);

	printk("[AURA_SYNC] fw_ver_show : 0x%x\n", platform_data->fw_version);
	mutex_unlock(&g_pdata->ene_mutex);

	//if (data[0] != 0x0 && !platform_data->FW_update_done){
	if (data[0] != 0x0){
		printk("[AURA_SYNC] FW Error, REG[0xF018] : 0x%x\n", data[0]);
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

	//printk("[AURA_SYNC] set_frame.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if (val > 255){
		printk("[AURA_SYNC] Frame should not over 255.\n");
		return ret;
	}

	mutex_lock(&g_pdata->ene_mutex);
	printk("[AURA_SYNC] %s : client->addr : 0x%x,  val : 0x%x.\n", __func__, client->addr, val);
	err = ene_8k41_write_bytes(client, 0x80F2, val);
	if (err !=1)
		printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);

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
		printk("[AURA_SYNC] get_frame:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t set_speed(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	u32 val;
	int err = 0;
	ssize_t ret;

	//printk("[AURA_SYNC] set_speed.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if (val != 254 && val != 255 && val != 0 && val != 1 && val != 2){
		printk("[AURA_SYNC] speed should be 254,255,0,1,2 .\n");
		return ret;
	}

	mutex_lock(&g_pdata->ene_mutex);
	//printk("[AURA_SYNC] %s : client->addr : 0x%x,  val : 0x%x.\n", __func__, client->addr, val);
	g_speed = val;

	err = ene_8k41_write_bytes(client, 0x8022, val);
	if (err !=1)
		printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);

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
		printk("[AURA_SYNC] get_speed:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t set_cali_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	int red_val = 0, green_val = 0, blue_val = 0;

	sscanf(buf, "%d %d %d", &red_val, &green_val, &blue_val);
	printk("[AURA_SYNC] set_cali_data, %d, %d, %d\n", red_val, green_val, blue_val);

	platform_data->RED_MAX = red_val;
	platform_data->GREEN_MAX = green_val;
	platform_data->BLUE_MAX = blue_val;

	return count;
}

static ssize_t get_cali_data(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);

	printk("[AURA_SYNC] R:%d, G:%d, B:%d\n", platform_data->RED_MAX, platform_data->GREEN_MAX, platform_data->BLUE_MAX);
	return snprintf(buf, PAGE_SIZE,"R:%d, G:%d, B:%d\n", platform_data->RED_MAX, platform_data->GREEN_MAX, platform_data->BLUE_MAX);
}

static ssize_t ic_status_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	u8 tmp = 0xFF;

	if(platform_data->current_mode != 0)
		return snprintf(buf, PAGE_SIZE,"%d\n", 1);

	mutex_lock(&g_pdata->ene_mutex);
	tmp = ene_CheckFirmwareVer(client);
	mutex_unlock(&g_pdata->ene_mutex);

	//printk("[AURA_SYNC] Current FW version 0x%x, tmp 0x%x\n", platform_data->fw_version, tmp);

	if (tmp != platform_data->fw_version)
		return snprintf(buf, PAGE_SIZE,"%d\n", 0);
	else
		return snprintf(buf, PAGE_SIZE,"%d\n", 1);
}

static ssize_t blink_set_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	int on_ms = 0, off_ms = 0;

	sscanf(buf, "%d %d", &on_ms, &off_ms);
	printk("[AURA_SYNC] blink_set_store, %d, %d\n", on_ms, off_ms);

	platform_data->on_ms = on_ms;
	platform_data->off_ms = off_ms;

	return count;
}

static ssize_t blink_set_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);

	printk("[AURA_SYNC] ON:%d ms, OFF:%d ms\n", platform_data->on_ms, platform_data->off_ms);
	return snprintf(buf, PAGE_SIZE,"ON:%d ms, OFF:%d ms\n", platform_data->on_ms, platform_data->off_ms);
}

static ssize_t emulate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	int tmp = 0;

	sscanf(buf, "%d", &tmp);
	printk("[AURA_SYNC] emulate_store, %d\n", tmp);

	if (tmp){
		platform_data->emulate = true;
		queue_work(platform_data->workqueue, &platform_data->aura_work);
	}else {
		platform_data->emulate = false;
	}

	return count;
}

static ssize_t emulate_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);

	printk("[AURA_SYNC] Emulate %d\n", platform_data->emulate);
	return snprintf(buf, PAGE_SIZE,"Emulate %d\n", platform_data->emulate);
}

static ssize_t suspend_vdd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	u32 val;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if(val>0) {
		printk("[AURA_SYNC] suspend_vdd set true\n");
		suspend_vdd_on = true;
	}else {
		printk("[AURA_SYNC] suspend_vdd set false\n");
		suspend_vdd_on = false;
	}

	return count;
}

static ssize_t suspend_vdd_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE,"suspend_vdd = %d\n", suspend_vdd_on);
}

static ssize_t pb_output_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	u32 val;
	int err = 0;
	ssize_t ret;

	printk("[AURA_SYNC] pb_output_store.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if (val < 0 || val > 255){
		printk("[AURA_SYNC] val must between 0 ~ 255\n");
		return count;
	}

	mutex_lock(&g_pdata->ene_mutex);
	printk("[AURA_SYNC] client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
	err = ene_8k41_write_bytes(client, 0xF203, val);
	if (err !=1)
		printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t pb_output_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_8k41_read_bytes(client, 0xF203, data);
	if (err != 2)
		printk("[AURA_SYNC] pb_output_show:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t pb_hl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	u32 val;
	int err = 0;
	ssize_t ret;

	printk("[AURA_SYNC] pb_hl_store.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if (val < 0 || val > 255){
		printk("[AURA_SYNC] val must between 0 ~ 255\n");
		return count;
	}

	mutex_lock(&g_pdata->ene_mutex);
	printk("[AURA_SYNC] client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
	err = ene_8k41_write_bytes(client, 0xF208, val);
	if (err !=1)
		printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t pb_hl_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_8k41_read_bytes(client, 0xF208, data);
	if (err != 2)
		printk("[AURA_SYNC] pb_hl_show:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static unsigned int reg_addr = 0x0;
static ssize_t register_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	u8 val, flag;
	int err = 0;

	sscanf(buf, "%x %x %d", &reg_addr, &val, &flag);
	printk("[AURA_SYNC] register_store: reg_addr 0x%x, val 0x%x, flag %d\n", reg_addr, val, flag);

	if(!flag){
		printk("[AURA_SYNC] skip.\n");
		return count;
	}

	mutex_lock(&g_pdata->ene_mutex);
	printk("[AURA_SYNC] client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
	err = ene_8k41_write_bytes(client, (short)reg_addr, val);
	if (err !=1)
		printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t register_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_8k41_read_bytes(client, (short)reg_addr, data);
	if (err != 2)
		printk("[AURA_SYNC] pb_hl_show:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"REG[0x%04x]:0x%x\n", reg_addr, data[0]);
}

static ssize_t CSCmode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	u32 val;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if(val>0) {
		printk("[AURA_SYNC] CSCmode set true\n");
		CSCmode = true;
		bumper_switch(1);
	}else {
		printk("[AURA_SYNC] CSCmode set false\n");
		CSCmode = false;
		bumper_switch(0);
	}

	return count;
}

static ssize_t CSCmode_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE,"CSCmode = %d\n", CSCmode);
}

static DEVICE_ATTR(red_pwm, 0664, red_pwm_show, red_pwm_store);
static DEVICE_ATTR(green_pwm, 0664, green_pwm_show, green_pwm_store);
static DEVICE_ATTR(blue_pwm, 0664, blue_pwm_show, blue_pwm_store);
static DEVICE_ATTR(apply, 0664, apply_show, apply_store);
static DEVICE_ATTR(mode, 0664, mode_show, mode_store);
static DEVICE_ATTR(VDD, 0664, vdd_show, vdd_store);
static DEVICE_ATTR(VDD2, 0664, vdd2_show, vdd2_store);
static DEVICE_ATTR(fw_update, 0664, NULL, fw_update_store);
static DEVICE_ATTR(fw_ver, 0664, fw_ver_show, NULL);
static DEVICE_ATTR(frame, 0664, get_frame, set_frame);
static DEVICE_ATTR(speed, 0664, get_speed, set_speed);
static DEVICE_ATTR(led_on, 0664, led_on_show, led_on_store);
static DEVICE_ATTR(Calibration, 0664, get_cali_data, set_cali_data);
static DEVICE_ATTR(IC_status, 0664, ic_status_show, NULL);
static DEVICE_ATTR(blink_set, 0664, blink_set_show, blink_set_store);
static DEVICE_ATTR(emulate, 0664, emulate_show, emulate_store);
static DEVICE_ATTR(suspend_vdd, 0664, suspend_vdd_show, suspend_vdd_store);
static DEVICE_ATTR(PB_output, 0664, pb_output_show, pb_output_store);
static DEVICE_ATTR(PB_HL, 0664, pb_hl_show, pb_hl_store);
static DEVICE_ATTR(led2_on, 0664, led2_on_show, led2_on_store);
static DEVICE_ATTR(REG, 0664, register_show, register_store);
static DEVICE_ATTR(CSCmode, 0664, CSCmode_show, CSCmode_store);

static struct attribute *pwm_attrs[] = {
	&dev_attr_red_pwm.attr,
	&dev_attr_green_pwm.attr,
	&dev_attr_blue_pwm.attr,
	&dev_attr_apply.attr,
	&dev_attr_mode.attr,
	&dev_attr_VDD.attr,
	&dev_attr_VDD2.attr,
	&dev_attr_fw_update.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_frame.attr,
	&dev_attr_speed.attr,
	&dev_attr_led_on.attr,
	&dev_attr_Calibration.attr,
	&dev_attr_IC_status.attr,
	&dev_attr_blink_set.attr,
	&dev_attr_emulate.attr,
	&dev_attr_suspend_vdd.attr,
	&dev_attr_PB_output.attr,
	&dev_attr_PB_HL.attr,
	&dev_attr_led2_on.attr,
	&dev_attr_REG.attr,
	&dev_attr_CSCmode.attr,
	NULL
};

static const struct attribute_group pwm_attr_group = {
	.attrs = pwm_attrs,
};


static void aura_sync_set(struct led_classdev *led,
			      enum led_brightness brightness)
{
	printk("[AURA_SYNC] aura_sync_set : %d.\n", brightness);
}

static enum led_brightness aura_sync_get(struct led_classdev *led_cdev)
{
	struct ene_8k41_platform_data *pdata;

	printk("[AURA_SYNC] aura_sync_get.\n");
	pdata = container_of(led_cdev, struct ene_8k41_platform_data, led);

	return pdata->led.brightness;
}

// Emulated FW pattern
static void aura_blink_work(struct work_struct *work)
{
	int err = 0;
	printk("[AURA_SYNC] aura_blink_work +++\n");

	while(g_pdata->emulate){
		// set on
		err = ene_8k41_write_bytes(g_pdata->client, 0x8021, 0x1);
		if (err !=1){
			printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);
			//mutex_unlock(&g_pdata->ene_mutex);
			goto finish;
		}
		err = ene_8k41_write_bytes(g_pdata->client, 0x802F, 0x1);
		if (err !=1)
			printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);

		msleep(g_pdata->on_ms);

		// set off
		err = ene_8k41_write_bytes(g_pdata->client, 0x8021, 0x0);
		if (err !=1){
			printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);
			//mutex_unlock(&g_pdata->ene_mutex);
			goto finish;
		}
		err = ene_8k41_write_bytes(g_pdata->client, 0x802F, 0x1);
		if (err !=1)
			printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);

		msleep(g_pdata->off_ms);
	}

finish:
	printk("[AURA_SYNC] aura_blink_work finish.\n");
}

// Resume work
static void aura_resume_work(struct work_struct *work)
{
	int i=0;
	u8 tmp=0;

	//wake_lock(&g_pdata->aura_wake_lock);
	// Enable Bumper if in need
	if(bumper_enable){
		printk("[AURA_SYNC] aura_resume_work to enable bumper LED\n");
		msleep(500);
		tmp = ene_CheckFirmwareVer(g_pdata->client);
		printk("[AURA_SYNC] Check FW Ver : 0x%x\n", tmp);

		// Make sure FW is initialed
		while(tmp != g_pdata->fw_version){
			i++;
			msleep(100);
			tmp = ene_CheckFirmwareVer(g_pdata->client);
			printk("[AURA_SYNC] re-check %d FW Ver : 0x%x\n", i, tmp);

			if(i==5){
				printk("[AURA_SYNC] Skip re-check at %d times\n", i);
				break;
			}
		}

		bumper_switch(1);
		//if ((g_ASUS_hwID >= ZS660KL_EVB && g_ASUS_hwID < ZS660KL_ER2) || (g_ASUS_hwID >= ZS660KL_CN_EVB && g_ASUS_hwID < ZS660KL_CN_ER2))
			//bumper_vdd_switch(1);
	}
	//wake_unlock(&g_pdata->aura_wake_lock);
}

// Check FW work
static void aura_check_fw_work(struct work_struct *work)
{
//	unsigned char *fw_buf;
//	int fw_size;

// Wait 0.5s for IC power on.\n");
	msleep(500);

// Check FW
	g_pdata->fw_version = ene_CheckFirmwareVer(g_pdata->client);
	printk("[AURA_SYNC] FW VER : 0x%x\n", g_pdata->fw_version);

/*
	// Trigger FW update
	if ( platform_data->fw_version < 0x5) { //disable auto update in kernel space
		printk("[AURA_SYNC] Start auto FW update.");

		// get fs_size
		fw_size = ene_GetFirmwareSize(FW_PATH);
		if (fw_size <= 0) {
			printk("[AURA_SYNC] get fwsize error %d.\n", fw_size);
			goto skip_auto_update;
		} else
			printk("[AURA_SYNC] fw_size %d\n", fw_size);

		// set fw_buf
		fw_buf = kmalloc(fw_size+1 ,GFP_ATOMIC);

		// read FW content
		if (ene_ReadFirmware(FW_PATH, fw_buf)) {
			printk("[AURA_SYNC] ERROR: request_firmware failed\n");
			kfree(fw_buf);
		}
	}
*/
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

	printk("[AURA_SYNC] ene_8k41_parse_dt\n");

	pdata->aura_3p3_en = of_get_named_gpio_flags(np, "ene,aura_3p3_en", 0, &pdata->aura_3p3_en_flags);
	printk("[AURA_SYNC] aura_3p3_en : %d\n", pdata->aura_3p3_en);

	pdata->aura_front_en = of_get_named_gpio_flags(np, "ene,aura_front_en", 0, &pdata->aura_front_en_flags);
	printk("[AURA_SYNC] aura_front_en : %d:%s\n", pdata->aura_front_en,(pdata->aura_front_en == -ENOENT ? "NOT DEFINED!!" : " DEFINED."));

	pdata->logo_5p0_en = of_get_named_gpio_flags(np, "ene,logo_5p0_en", 0, &pdata->logo_5p0_en_flags);
	printk("[AURA_SYNC] logo_5p0_en : %d\n", pdata->logo_5p0_en);

/*
	printk("[AURA_SYNC] Get the pinctrl node \n");
	// Get the pinctrl node
	pdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(pdata->pinctrl)) {
	     dev_err(dev, "%s: Failed to get pinctrl\n", __func__);
	}

	printk("[AURA_SYNC] Get the active setting \n");
	// Get the active setting
	printk("[AURA_SYNC] Get default_enable \n");
	pdata->pins_active = pinctrl_lookup_state(pdata->pinctrl, "active");
	if (IS_ERR_OR_NULL(pdata->pins_active)) {
		dev_err(dev, "%s: Failed to get pinctrl state active\n", __func__);
	}

	// Set the active setting
	printk("[AURA_SYNC] set the active state\n");
	retval = pinctrl_select_state(pdata->pinctrl, pdata->pins_active);
	if (retval)
		dev_err(dev, "%s: pinctrl_select_state retval:%d\n", __func__, retval);
*/

	return 0;
}

static int ene_8k41_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct ene_8k41_platform_data *platform_data;
	
	printk("[AURA_SYNC] ene_8k41_probe.\n");

	if(g_Charger_mode) {
		printk("[AURA_SYNC] In charger mode, stop ene_8k41_probe\n");
		return 0;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
	goto exit_check_functionality_failed;
	} else
		printk("[AURA_SYNC] I2C function test pass\n");

	printk("[AURA_SYNC] client->addr : 0x%x\n", client->addr);

	platform_data = devm_kzalloc(&client->dev, sizeof(struct ene_8k41_platform_data), GFP_KERNEL);
	if (!platform_data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, platform_data);
// Parse platform data from dtsi
	err = ene_8k41_parse_dt(&client->dev, platform_data);
	if (err) {
		printk("[AURA_SYNC] ene_8k41_parse_dt get fail !!!\n");
		return -ENOMEM;
	}

// Set ENE 8K41 power
/*
	if(regulator_is_enabled(reg) > 0){
		printk("[AURA_SYNC] ene8k41_regulator is enabled ");
		return -ENOMEM;
	}
	err = ene8k41_regulator_enable();
	if(err < 0){
		printk("[AURA_SYNC] fail to enable ene8k41_regulator %d ",err);
		return -ENOMEM;
	}

if (platform_data->aura_front_en != -ENOENT )
{
	err = gpio_request(platform_data->aura_front_en, "aura_front_en");
	if (err)
		printk("[AURA_SYNC] aura_front_en gpio_request, err %d\n", err);
}
*/

	/* disable gpio for sr & after
	 *
	err = gpio_request(platform_data->aura_3p3_en, "aura_3p3_en");
	if (err)
		printk("[AURA_SYNC] aura_3p3_en gpio_request, err %d\n", err);

	printk("[AURA_SYNC] ENE 8k41 power on.\n");
	err = gpio_direction_output(platform_data->aura_3p3_en, 1);
	if (err)
		printk("[AURA_SYNC] aura_3p3_en output high, err %d\n", err);

	gpio_set_value(platform_data->aura_3p3_en, 1);
	printk("[AURA_SYNC] aura_3p3_en[%d] :0x%x\n", platform_data->aura_3p3_en, gpio_get_value(platform_data->aura_3p3_en));
	*
	*/

	// Set logo 5V enable pin for SR1 & after
/*
	if ( gpio_is_valid(platform_data->logo_5p0_en) ){
		err = gpio_request(platform_data->logo_5p0_en, "logo_5p0_en");
		if (err)
			printk("[AURA_SYNC] logo_5p0_en gpio_request, err %d\n", err);

		printk("[AURA_SYNC] logo_5p0_en default off.\n");
		err = gpio_direction_output(platform_data->logo_5p0_en, 0);
		if (err)
			printk("[AURA_SYNC] logo_5p0_en output high, err %d\n", err);
	}else
		printk("[AURA_SYNC] logo_5p0_en is not defined. %d\n", platform_data->logo_5p0_en);
*/

// Create init workqueue
	platform_data->check_fw_workqueue = alloc_ordered_workqueue("check_fw_workqueue", 0);
	if (!platform_data->check_fw_workqueue) {
		return -1;
	}
	INIT_WORK(&platform_data->check_fw_work, aura_check_fw_work);

// Set global variable
	platform_data->client = client;
	g_pdata = platform_data;

// Use workque to check FW avoid block kernel main thread
	queue_work(platform_data->check_fw_workqueue, &platform_data->check_fw_work);

/*
// Setting PWM inverse
	printk("[AURA_SYNC] set PWM inverse, REG[0xF207] = 0x8\n");
	err = ene_8k41_write_bytes(client, 0xF207, 0x8);
	if (err !=1)
		printk("[AURA_SYNC] ene_8k41_write_bytes:err %d\n", err);

	err = ene_8k41_read_bytes(client, 0xF207, data);
	if (err != 2)
		printk("[AURA_SYNC] ene_8k41_read_bytes:err %d\n", err);

	printk("[AURA_SYNC] REG[0xF207]: 0x%x\n", data[0]);
	printk("[AURA_SYNC] ==========\n");
*/

// Register sys class  
	err = aura_sync_register(&client->dev, platform_data);
	if (err) {
		printk("[AURA_SYNC] Failed to register LED device: %d\n", err);
		goto unled;
	}
	err = sysfs_create_group(&platform_data->led.dev->kobj, &pwm_attr_group);
	if (err)
			goto unled;

// ===== Emuldate FW =====
// Init emulate workqueue
	platform_data->workqueue = alloc_ordered_workqueue("aura_workqueue", 0);
	if (!platform_data->workqueue) {
		goto unled;
	}
	INIT_WORK(&platform_data->aura_work, aura_blink_work);

	// set default
	platform_data->on_ms = 100;
	platform_data->off_ms = 100;
// =======================

// Init resume workqueue
	platform_data->resume_workqueue = alloc_ordered_workqueue("resume_aura_workqueue", 0);
	if (!platform_data->resume_workqueue) {
		goto unled;
	}
	INIT_WORK(&platform_data->resume_aura_work, aura_resume_work);

// Init wake lock
	//wake_lock_init(&platform_data->aura_wake_lock, WAKE_LOCK_SUSPEND, "aura_wake_lock");
	//wakeup_source_init(&ene_wakelock, "ene_wakelock");

//#ifdef ASUS_FTM
#if 0
	printk("[AURA_SYNC] VDD default off in FTM.\n");
	if ( gpio_is_valid(platform_data->aura_3p3_en) )
		gpio_set_value(platform_data->aura_3p3_en, 0);
#endif

	mutex_init(&platform_data->ene_mutex);
	platform_data->current_mode = 0x0;
	platform_data->suspend_state = false;
	platform_data->FW_update_done = false;

// Default Calibration Data
	platform_data->RED_MAX = 255;
	platform_data->GREEN_MAX = 255;
	platform_data->BLUE_MAX = 255;

// Set global variable
	g_pdata = platform_data;

// Reset globle var
	g_red = 255;
	g_green = 255;
	g_blue = 255;
	g_mode = 0;
	g_speed = 0;
	g_led_on = 0;
	g_led2_on = 0;

	printk("[AURA_SYNC] ene_8k41_probe done.\n");
	return 0;

unled:
	aura_sync_unregister(platform_data);
	printk("[AURA_SYNC] ENE 8K41 power off.\n");

/*
	err = ene8k41_regulator_disable();
	if(err < 0){
		printk("[AURA_SYNC] fail to disable ene8k41_regulator %d ",err);
		return err;
	}
*/
	/*err = gpio_direction_output(platform_data->aura_3p3_en, 0);
	if (err)
		printk("[AURA_SYNC] aura_3p3_en output low, err %d\n", err);*/
exit_check_functionality_failed:
	printk("[AURA_SYNC] ene_8k41_probe fail !!!\n");
	return err;
}

static int ene_8k41_remove(struct i2c_client *client)
{
	int err = 0;
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);

// unregister
	printk("[AURA_SYNC] sysfs_remove_group\n");
	sysfs_remove_group(&platform_data->led.dev->kobj, &pwm_attr_group);

	printk("[AURA_SYNC] aura_sync_unregister\n");
	aura_sync_unregister(platform_data);

// power off
/*
	err = ene8k41_regulator_disable();
	if(err < 0){
		printk("[AURA_SYNC] fail to disable ene8k41_regulator %d ",err);
		return err;
	}
*/
	/*printk("[AURA_SYNC] set GPIO[%d] output low\n", platform_data->aura_3p3_en);
	err = gpio_direction_output(platform_data->aura_3p3_en, 0);
	if (err)
		printk("[AURA_SYNC] aura_3p3_en output low, err %d\n", err);*/

// free gpio
	/*printk("[AURA_SYNC] Free GPIO[%d].\n", platform_data->aura_3p3_en);
	gpio_free(platform_data->aura_3p3_en);*/

// free pointer
//	printk("[AURA_SYNC] Free platform_data\n");
//	kfree(platform_data);

	mutex_destroy(&platform_data->ene_mutex);
	printk("[AURA_SYNC] ene_8k41_remove : err %d\n", err);
	return 0;
}

int ene_8k41_suspend(struct device *dev)
{
	int err = 0;

	if(g_Charger_mode) {
		printk("[AURA_SYNC] In charger mode, stop ene_8k41_suspend\n");
		return 0;
	}

	printk("[AURA_SYNC] ene_8k41_suspend : current_mode : 0x%x , bumper_enable %d\n", g_pdata->current_mode, bumper_enable);

	if(!g_pdata->current_mode){
		//printk("[AURA_SYNC] Disable VDD.\n");
		/*if ( gpio_is_valid(g_pdata->aura_3p3_en) )
			gpio_set_value(g_pdata->aura_3p3_en, 0);*/
/*
		err = ene8k41_regulator_disable();
		if(err < 0){
			printk("[AURA_SYNC] fail to disable ene8k41_regulator %d ",err);
			return err;
		}
*/
		g_red = 255;
		g_green = 255;
		g_blue = 255;
		g_mode = 0;
		g_speed = 0;
		g_led_on = 0;
		g_led2_on = 0;
	}

	g_pdata->suspend_state = true;

	return err;
}

int ene_8k41_resume(struct device *dev)
{
	int err = 0;

	if(g_Charger_mode) {
		printk("[AURA_SYNC] In charger mode, stop ene_8k41_resume\n");
		return 0;
	}

	printk("[AURA_SYNC] ene_8k41_resume : current_mode : 0x%x, bumper_enable %d\n", g_pdata->current_mode, bumper_enable);
/*
	if(!g_pdata->current_mode){
		//printk("[AURA_SYNC] Enable VDD.\n");
		if ( gpio_is_valid(g_pdata->aura_3p3_en) )
			gpio_set_value(g_pdata->aura_3p3_en, 1);

		err = ene8k41_regulator_enable();
		if(err < 0){
			printk("[AURA_SYNC] fail to enable ene8k41_regulator %d ",err);
			return err;
		}
	}
*/
	queue_work(g_pdata->resume_workqueue, &g_pdata->resume_aura_work);

	g_pdata->suspend_state = false;

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
	{ .compatible = "ene8k41",},
	{ },
};
#else
#define ene_match_table NULL
#endif

static struct i2c_driver ene_8k41_driver = {
	.driver		= {
		.name		= "ene8k41",
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
		printk("[AURA_SYNC] ENE 8k41 driver int failed.\n");
	else
		printk("[AURA_SYNC] ENE 8k41 driver int success.\n");
	
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
