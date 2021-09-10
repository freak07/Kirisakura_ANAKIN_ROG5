/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
 * Edit by ASUS Deeo, deeo_ho@asus.com
 * V5 for USB HID control
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

#define FW_PATH "/asusfw/aura_sync/ENE-8K41-aura-V5.bin"

static struct ene_8k41_platform_data *g_pdata;
extern int hid_to_i2c_write(char *, int );
extern int hid_to_i2c_read(char *, int );
extern int hid_get_i2c_data(char *, int , int *);

#define I2C_PORT 0x02
#define SLAVE_ADDR 0x80

__u8 buf1[7]={0x01, 0x80, 0x00, 0x00, 0x02, 0x80, 0xC0};
//                      1     128     0        0       2     128    192
__u8 buf2[5]={0x01, 0x80, 0x81, 0x00, 0x01};
//					     1      128    129     0       1

static int ene_hid_read_bytes(short addr, char *data)
{
	int err = 0, len = 0, count = 0, i;
	unsigned char buf[16] = {0};

	printk("[AURA_EC] ene_hid_read_bytes : addr: 0x%x, buf[1] : 0x%x, buf[2] : 0x%x\n", (addr & 0xFFFF), buf[1], buf[2]);	

	len = 7;
	buf[0] = I2C_PORT;
	buf[1] = SLAVE_ADDR;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x02;
	buf[5] = (addr >> 8) & 0xFF;
	buf[6] = addr & 0xFF;
	
	err = hid_to_i2c_write(buf, len);
	if (err < 0)
		printk("[AURA_EC] i2c_write_bytes:err %d\n", err);

	len = 5;
	buf[2] = 0x81;
	buf[3] = 0x00;
	buf[4] = 0x01;

	err = hid_to_i2c_read(buf, len);
	if (err < 0)
		printk("[AURA_EC] i2c_read_bytes:err %d\n", err);

	count = 2;
	len = 0;

	err = hid_get_i2c_data(data, count, &len);
	if (err < 0)
		printk("[AURA_EC] hid_get_i2c_data:err %d\n", err);

	for (i=0; i<len; i++)
		printk("[AURA_EC] data[%d]: 0x%x\n", i, data[i]);
	
//	msleep(5);
	return err;
}

static int ene_hid_write_bytes(short addr, char value)
{
	int err = 0, len = 0;
	unsigned char buf[16] = {0};

	printk("[AURA_EC] ene_hid_write_bytes : addr: 0x%x, value : 0x%x\n", (addr & 0xFFFF), value);
	len = 7;
	buf[0] = I2C_PORT;
	buf[1] = SLAVE_ADDR;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x02;
	buf[5] = (addr >> 8) & 0xFF;
	buf[6] = addr & 0xFF;
	printk("[AURA_EC] ene_hid_write_bytes: buf[5]: 0x%x, buf[6]: 0x%x \n", buf[5], buf[6]);
	
	err = hid_to_i2c_write(buf, len);
	if (err < 0)
		printk("[AURA_EC] i2c_write_bytes:err %d\n", err);

	len = 6;
	buf[0] = I2C_PORT;
	buf[1] = SLAVE_ADDR;
	buf[2] = 0x01;
	buf[3] = 0x00;
	buf[4] = 0x01;
	buf[5] = value;

	err = hid_to_i2c_write(buf, len);
	if (err < 0)
		printk("[AURA_EC] i2c_write_bytes:err %d\n", err);
	
//	msleep(5);
	return err;
}

static int ene_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];

	printk("[AURA_EC] ene_GetFirmwareSize.\n");
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

static char ene_CheckFirmwareVer(void)
{
	int err = 0;
	unsigned char data[2] = {0};

	err = ene_hid_read_bytes(0x80C0, data);
	if (err < 0)
		printk("[AURA_EC] fw_check:err %d\n", err);

	return (u8) data[0];
}

static int ene_UpdateFirmware(char *fw_buf)
{
	int err = 0;
	unsigned char data[2] = {0};
	unsigned char buf[133] = {0};
	short addr;
	int retry = 0;
	char tmp[64] = {0};
	int count = 0, len = 0, i=0;

	err = ene_hid_read_bytes(0xFF8F, data);
	if (err != 2)
		printk("[AURA_EC] Check REG[0xFF8F]:err %d\n", err);

	printk("[AURA_EC] REG[0xFF8F] : 0x%x\n", data[0]);

	if( (u8)(data[0]) <= 0x40){
		g_pdata->fw_version = ene_CheckFirmwareVer();
		printk("[AURA_SYCN] FW VER : 0x%x\n", g_pdata->fw_version);
		//if ( g_pdata->fw_version == 0x0) { // rember change condition.
		if (0){
			printk("[AURA_EC] Don't need FW update.\n");
			return 0;
		}
	} else
		printk("[ASUS_SYNC] FW is abnormal !!!");

	printk("[AURA_EC] Start ene_UpdateFirmware\n");

// (1) Reset 8051
	printk("[AURA_EC] Reset 8051 =====\n");
	err = ene_hid_write_bytes(0xF010, 0x5);
	if (err <0)
		printk("[AURA_EC] REG[F010]: write err %d\n", err);

	msleep(500);

// (2) Read & Fill OSC32M freq.
	printk("[AURA_EC] Read & Fill OSC32M freq =====\n");
	err = ene_hid_write_bytes(0xF808, 0xF0);
	if (err <0)
		printk("[AURA_EC] REG[0xF808]: write err %d\n", err);
	err = ene_hid_write_bytes(0xF809, 0x01);
	if (err <0)
		printk("[AURA_EC] REG[0xF809]: write err %d\n", err);
	err = ene_hid_write_bytes(0xF807, 0x90);
	if (err <0)
		printk("[AURA_EC] REG[0xF807]: write err %d\n", err);

	err = ene_hid_read_bytes(0xF80B, data);
	if (err <0)
		printk("[AURA_EC] REG[0xF80B]: read err %d\n", err);

	printk("[AURA_EC] REG[0xF80B]: 0x%x\n", data[0]);
	err = ene_hid_write_bytes(0xF806, data[0]);
	if (err <0)
		printk("[AURA_EC] REG[0xF806]: write err %d\n", err);

// (3) Fill program & Erase timing
	printk("[AURA_EC] Fill program & Erase timing =====\n");
	err = ene_hid_write_bytes(0xF815, 0x10);
	if (err <0)
		printk("[AURA_EC] REG[0xF815]: write err %d\n", err);
	err = ene_hid_write_bytes(0xF816, 0x11);
	if (err <0)
		printk("[AURA_EC] REG[0xF816]: write err %d\n", err);
	err = ene_hid_write_bytes(0xF817, 0x06);
	if (err <0)
		printk("[AURA_EC] REG[0xF817]: write err %d\n", err);
	err = ene_hid_write_bytes(0xF818, 0x07);
	if (err <0)
		printk("[AURA_EC] REG[0xF818]: write err %d\n", err);

// (4) Erase
	printk("[AURA_EC] Erase page from 0x0000 to 0x3F80 =====\n");
	for (addr = 0x0000 ; addr <= 0x3F80 ; addr+= 0x80) {
	//for (addr = 0x0000 ; addr < 0x80 ; addr = addr + 0x80) {
		printk("[AURA_EC] set addr : 0x%04x\n", addr);
		err = ene_hid_write_bytes(0xF808, (addr & 0xFF));				//	Addr_L
		if (err <0)
			printk("[AURA_EC] REG[0xF808]: write err %d\n", err);
		err = ene_hid_write_bytes(0xF809, (addr >> 8) & 0xFF);		//	Addr_H
		if (err <0)
			printk("[AURA_EC] REG[0xF809]: write err %d\n", err);

		err = ene_hid_write_bytes(0xF807, 0x20);	// erase page cmd
		if (err <0)
			printk("[AURA_EC] REG[0xF807]: write err %d\n", err);

		while(1){	 // check cmd finished. 0xF800[7] = 1
			msleep(50);
			err = ene_hid_read_bytes(0xF800, data);
			if (err <0) {
				printk("[AURA_EC] REG[0xF800]: read err %d\n", err);
			}
			if ( ((data[0] >> 7) & 0x1) == 1 ){
				printk("[AURA_EC] check REG[0xF800] pass, 0x%02x\n", (data[0]));
				retry=0;
				break;
			}
			printk("[AURA_EC] check REG[0xF800], 0x%02x\n", data[0]);
			retry++;
			if (retry > 10) {
				printk("[AURA_EC] retry too many times: %d, Force exit!!!\n", retry);
				return -1;
			}
		}
	}
	printk("[AURA_EC] erase page finished. =====\n");
	
// (5) Program
	printk("[AURA_EC] Program FW from 0x0000 to 0x3FE0 =====\n");
	for (addr = 0x0000 ; addr < 0x4000 ; addr = addr + 0x20) {
	//for (addr = 0x0000 ; addr < 0x80 ; addr = addr + 0x20) {
		printk("[AURA_EC] set addr : 0x%04x\n", addr);
		err = ene_hid_write_bytes(0xF808, (addr & 0xFF));				//	Addr_L
		if (err <0)
			printk("[AURA_EC] REG[0xF808]: write err %d\n", err);
		err = ene_hid_write_bytes(0xF809, (addr >> 8) & 0xFF);		//	Addr_H
		if (err <0)
			printk("[AURA_EC] REG[0xF809]: write err %d\n", err);

		err = ene_hid_write_bytes(0xF807, 0x80);	// 3. Clear HVPL data
		if (err <0)
			printk("[AURA_EC] REG[0xF807]: write err %d\n", err);

		while(1){	 // 4. check cmd finished 0xF800[7] = 1
			msleep(50);
			err = ene_hid_read_bytes(0xF800, data);
			if (err <0) {
				printk("[AURA_EC] REG[0xF800]: read err %d\n", err);
			}
			if ( ((data[0] >> 7) & 0x1) == 1 ){
				printk("[AURA_EC] check REG[0xF800] pass, 0x%02x\n", (data[0]));
				retry = 0;
				break;
			}
			printk("[AURA_EC] check REG[0xF800], 0x%02x\n", (data[0]));
			retry++;
			if (retry > 10) {
				printk("[AURA_EC] retry too many times: %d, Force exit!!!\n", retry);
				return -1;
			}
		}

		// 5. Set address to 0xF80A
		/*
		buf[0] = 0x00;
		buf[1] = 0xF8;	//Addr_H
		buf[2] = 0x0A;	//Addr_L
		err = i2c_write_bytes(client, buf, 3);	//set register address
		if (err !=1)
			printk("[AURA_EC] i2c_write_bytes:err %d\n", err);
		*/

		buf[0] = I2C_PORT;
		buf[1] = SLAVE_ADDR;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x02;
		buf[5] = 0xF8;
		buf[6] = 0x0A;
		err = hid_to_i2c_write(buf, 7);
		if (err < 0)
			printk("[AURA_EC] hid_to_i2c_write:err %d\n", err);
		
		// 6. Send data to buffer
		//buf[0] = 0x05;
		buf[0] = I2C_PORT;
		buf[1] = SLAVE_ADDR;
		buf[2] = 0x05;
		buf[3] = 0x00;
		buf[4] = 0x20;	// data length 128 bytes
		memcpy( &(buf[5]), fw_buf + addr, 32 ); // copy fw_buf to buf

//		err = i2c_write_bytes(client, buf, 129);	 // 128 byte + 1 cmd = 129
//		if (err !=1)
//			printk("[AURA_EC] i2c_write_bytes:err %d\n", err);

		err = hid_to_i2c_write(buf, 37);
		if (err < 0)
			printk("[AURA_EC] hid_to_i2c_write:err %d\n", err);

		msleep(7);

		// 7. Set address to 0xF807
/*
		buf[0] = 0x00;
		buf[1] = 0xF8;	//Addr_H
		buf[2] = 0x07;	//Addr_L
		err = i2c_write_bytes(client, buf, 3);	//set register address
		if (err !=1)
			printk("[AURA_EC] i2c_write_bytes:err %d\n", err);
*/
		buf[0] = I2C_PORT;
		buf[1] = SLAVE_ADDR;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x02;
		buf[5] = 0xF8;
		buf[6] = 0x07;
		err = hid_to_i2c_write(buf, 7);
		if (err < 0)
			printk("[AURA_EC] hid_to_i2c_write:err %d\n", err);

		msleep(5);
		// 8. Program page
/*
		buf[0] = 0x05;
		buf[1] = 0x70;
		err = i2c_write_bytes(client, buf, 2);
		if (err !=1)
			printk("[AURA_EC] i2c_write_bytes:err %d\n", err);
*/
		buf[0] = I2C_PORT;
		buf[1] = SLAVE_ADDR;
		buf[2] = 0x05;
		buf[3] = 0x00;
		buf[4] = 0x01;
		buf[5] = 0x70;
		err = hid_to_i2c_write(buf, 6);
		if (err < 0)
			printk("[AURA_EC] hid_to_i2c_write:err %d\n", err);
		msleep(5);

		while(1){	 // 9. check cmd finished 0xF800[7] = 1
			msleep(50);
			err = ene_hid_read_bytes(0xF800, data);
			if (err <0) {
				printk("[AURA_EC] REG[0xF800]: read err %d\n", err);
			}
			if ( ((data[0] >> 7) & 0x1) == 1 ){
				printk("[AURA_EC] check REG[0xF800] pass, 0x%02x\n", (data[0]));
				retry = 0;
				break;
			}
			printk("[AURA_EC] check REG[0xF800], 0x%02x\n", (data[0]));
			retry++;
			if (retry > 10) {
				printk("[AURA_EC] retry too many times: %d, Force exit!!!\n", retry);
				return -1;
			}
		}
	}
	printk("[AURA_EC] Program FW finish =====\n");
////////////////////////////

	printk("[AURA_EC] read FW in 8K41 rom +++\n");
	for (addr = 0x0000 ; addr < 0x20 ; addr = addr + 0x20) {
		printk("[AURA_EC] set addr : 0x%04x\n", addr);
		err = ene_hid_write_bytes(0xF808, (addr & 0xFF));				//	Addr_L
		if (err <0)
			printk("[AURA_EC] REG[0xF808]: write err %d\n", err);
		err = ene_hid_write_bytes(0xF809, (addr >> 8) & 0xFF);		//	Addr_H
		if (err <0)
			printk("[AURA_EC] REG[0xF809]: write err %d\n", err);

		// 5. Set address to 0xF80B
		/*
		buf[0] = 0x00;
		buf[1] = 0xF8;	//Addr_H
		buf[2] = 0x0B;	//Addr_L
		err = i2c_write_bytes(client, buf, 3);	//set register address
		if (err !=1)
			printk("[AURA_EC] i2c_write_bytes:err %d\n", err);
*/

		buf[0] = I2C_PORT;
		buf[1] = SLAVE_ADDR;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x02;
		buf[5] = 0xF8;
		buf[6] = 0x0B;
		err = hid_to_i2c_write(buf, 7);
		if (err < 0)
			printk("[AURA_EC] hid_to_i2c_write:err %d\n", err);

/*
		buf[0] = 0x06;
		err = i2c_read_bytes(client, buf, 1, tmp, 128);	//set register address
		if (err !=2)
			printk("[AURA_EC] i2c_read_bytes:err %d\n", err);
*/

		buf[0] = I2C_PORT;
		buf[1] = SLAVE_ADDR;
		buf[2] = 0x06;
		buf[3] = 0x00;
		buf[4] = 0x20;
		err = hid_to_i2c_read(buf,5);
		if (err < 0)
			printk("[AURA_EC] hid_to_i2c_read:err %d\n", err);


		count = 32;
		len = 0;

		err = hid_get_i2c_data(tmp, count, &len);
		if (err < 0)
			printk("[AURA_EC] hid_get_i2c_data:err %d\n", err);

		for (i=0; i<len ; i++)
			printk("[AURA_EC] tmp[%d]:0x%x\n", i, tmp[i]);
	}
	printk("[AURA_EC] read FW in 8K41 rom ---\n");

////////////////////////////
	
// (9) Restart 8051
	printk("[AURA_EC] Restart 8051.\n");
	err = ene_hid_write_bytes(0xF010, 0x04);
	if (err <0)
		printk("[AURA_EC] REG[0xF807]: write err %d\n", err);

	msleep(500);

	printk("[AURA_EC] ene_UpdateFirmware finished.\n");

	return 0;
}

static ssize_t red_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val;
	int err = 0;
	ssize_t ret;

	printk("[AURA_EC] red_pwm_store.\n");
	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return ret;

	err = ene_hid_write_bytes(0x8010, reg_val);
	if (err < 0)
		printk("[AURA_EC] red_pwm_store:err %d\n", err);
	
	return count;
}

static ssize_t red_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	unsigned char data[4] = {0};
	int err = 0;

	err = ene_hid_read_bytes(0x8010, data);
	if (err < 0)
		printk("[AURA_EC] red_pwm_show:err %d\n", err);

	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t green_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val;
	int err = 0;
	ssize_t ret;

	printk("[AURA_EC] green_pwm_store.\n");
	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return ret;

	err = ene_hid_write_bytes(0x8011, reg_val);
	if (err < 0)
		printk("[AURA_EC] green_pwm_store:err %d\n", err);

	return count;
}

static ssize_t green_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	unsigned char data[4] = {0};
	int err = 0;

	err = ene_hid_read_bytes(0x8011, data);
	if (err < 0)
		printk("[AURA_EC] green_pwm_show:err %d\n", err);

	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t blue_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val;
	int err = 0;
	ssize_t ret;

	printk("[AURA_EC] blue_pwm_store.\n");
	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return ret;

	err = ene_hid_write_bytes(0x8012, reg_val);
	if (err < 0)
		printk("[AURA_EC] blue_pwm_store:err %d\n", err);

	return count;
}

static ssize_t blue_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	unsigned char data[4] = {0};
	int err = 0;

	err = ene_hid_read_bytes(0x8012, data);
	if (err < 0)
		printk("[AURA_EC] blue_pwm_show:err %d\n", err);

	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t apply_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if (val > 0){
		printk("[AURA_EC] Send apply cmd.\n");

		err = ene_hid_write_bytes(0x802F, 0x1);
		if (err < 0)
			printk("[AURA_EC] apply_store:err %d\n", err);

	} else
		printk("[AURA_EC] No send apply cmd.\n");

	return count;
}

static ssize_t apply_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	unsigned char data[4] = {0};
	int err = 0;

	err = ene_hid_read_bytes(0x802F, data);
	if (err < 0)
		printk("[AURA_EC] blue_pwm_show:err %d\n", err);

	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;

	printk("[AURA_EC] mode_store.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	err = ene_hid_write_bytes(0x8021, val);
	if (err < 0)
		printk("[AURA_EC] apply_store:err %d\n", err);

	g_pdata->current_mode = (u8)val;
	return count;
}

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	unsigned char data[4] = {0};
	int err = 0;

	err = ene_hid_read_bytes(0x8021, data);
	if (err < 0)
		printk("[AURA_EC] mode_show:err %d\n", err);

	g_pdata->current_mode = data[0];
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}
/*
static ssize_t vdd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err;
	ssize_t ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if(val>0) {
		printk("[AURA_EC] VDD set HIGH\n");
		
		if ( gpio_is_valid(g_pdata->power_gpio) ) {
			gpio_set_value(g_pdata->power_gpio, 1);
		}else {
			err = regulator_enable(g_pdata->regulator_vdd);
			if (err) {
				dev_err(dev,
					"%s: Failed to enable regulator vdd\n",
					__func__);
			}
		}
	}else {
		printk("[AURA_EC] VDD set LOW\n");

		if ( gpio_is_valid(g_pdata->power_gpio) ) {
			gpio_set_value(g_pdata->power_gpio, 0);
		}else {
			err = regulator_disable(g_pdata->regulator_vdd);
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
	int val = 0;

	if ( gpio_is_valid(g_pdata->power_gpio) ) {
		val = gpio_get_value(g_pdata->power_gpio);
		return snprintf(buf, PAGE_SIZE,"power_gpio[%d] :0x%x\n", g_pdata->power_gpio, val);
	}else {
		return snprintf(buf, PAGE_SIZE,"No define power_gpio\n");
	}
}
*/
static ssize_t led_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	u32 val;
	ssize_t ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if(val>0) {
		printk("[AURA_EC] LED_EN set HIGH\n");

		// set 0xF204 as 0x02 for PC1 gpio output mode enable
		err = ene_hid_write_bytes(0xF204, 0x02);
		if (err < 0)
			printk("[AURA_EC] ene_hid_write_bytes:err %d\n", err);

		// set 0xF209 as 0x02 for PC1 gpio HIGH
		err = ene_hid_write_bytes(0xF209, 0x02);
		if (err < 0)
			printk("[AURA_EC] ene_hid_write_bytes:err %d\n", err);

	}else {
		printk("[AURA_EC] LED_EN set LOW\n");

		// set 0xF202 as 0x02 for PC1 gpio output mode disable
		err = ene_hid_write_bytes(0xF204, 0x00);
		if (err < 0)
			printk("[AURA_EC] ene_hid_write_bytes:err %d\n", err);

		// set 0xF209 as 0x02 for PC1 gpio LOW
		err = ene_hid_write_bytes(0xF209, 0x00);
		if (err < 0)
			printk("[AURA_EC] ene_hid_write_bytes:err %d\n", err);

	}

	return count;
}

static ssize_t led_en_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	unsigned char data[2] = {0};
	int err = 0;

	err = ene_hid_read_bytes(0xF209, data);
	if (err < 0)
		printk("[AURA_EC] ene_hid_write_bytes:err %d\n", err);

	return snprintf(buf, PAGE_SIZE,"LED_EN : 0x%x\n", data[0]);
}

static ssize_t fw_update_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	char fw_name[128];
	unsigned char *fw_buf;
	int fw_size;

	memset(fw_name, 0, sizeof(fw_name));
	sprintf(fw_name, "%s", buf);
	fw_name[count-1] = '\0';
	
	printk("[AURA_EC] fwname : %s\n", fw_name);

	// get fs_size
	fw_size = ene_GetFirmwareSize(fw_name);
	printk("[AURA_EC] fwsize %d\n", fw_size);
	
	// set fw_buf
	fw_buf = kmalloc(fw_size+1 ,GFP_ATOMIC);

	// read FW content
	if (ene_ReadFirmware(fw_name, fw_buf)) {
		printk("[AURA_EC] ERROR: request_firmware failed\n");
		kfree(fw_buf);
		return -EIO;
	}

	// Start update FW
	err = ene_UpdateFirmware(fw_buf);
	if(err)
		printk("[AURA_EC] ene_UpdateFirmware, err %d\n", err);

	// Update FW VER
	g_pdata->fw_version = ene_CheckFirmwareVer();
	printk("[AURA_EC] FW VER : 0x%x\n", g_pdata->fw_version);

	return count;
}

static ssize_t fw_ver_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	g_pdata->fw_version = ene_CheckFirmwareVer();

	printk("[AURA_EC] fw_ver_show : 0x%x\n", g_pdata->fw_version);
	return snprintf(buf, PAGE_SIZE,"0x%x\n", g_pdata->fw_version);
}

static ssize_t set_frame(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;

	printk("[AURA_EC] set_frame.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if (val > 255){
		printk("[AURA_EC] Frame should not over 255.\n");
		return ret;
	}

	err = ene_hid_write_bytes(0x80F2, val);
	if (err < 0)
		printk("[AURA_EC] ene_8k41_write_bytes:err %d\n", err);

	return count;

}

static ssize_t get_frame(struct device *dev, struct device_attribute *attr,char *buf)
{
	unsigned char data[4] = {0};
	int err = 0;

	err = ene_hid_read_bytes(0x80F3, data);
	if (err < 0)
		printk("[AURA_EC] get_frame:err %d\n", err);

	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t set_speed(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;

	printk("[AURA_EC] set_speed.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if (val != 254 && val != 255 && val != 0 && val != 1 && val != 2){
		printk("[AURA_EC] speed should be 254,255,0,1,2 .\n");
		return ret;
	}

	err = ene_hid_write_bytes(0x8022, val);
	if (err < 0)
		printk("[AURA_EC] ene_8k41_write_bytes:err %d\n", err);

	return count;

}

static ssize_t get_speed(struct device *dev, struct device_attribute *attr,char *buf)
{
	unsigned char data[4] = {0};
	int err = 0;

	err = ene_hid_read_bytes(0x8022, data);
	if (err < 0)
		printk("[AURA_EC] get_speed:err %d\n", err);

	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t ec_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE,"1\n");
}

static DEVICE_ATTR(red_pwm, 0664, red_pwm_show, red_pwm_store);
static DEVICE_ATTR(green_pwm, 0664, green_pwm_show, green_pwm_store);
static DEVICE_ATTR(blue_pwm, 0664, blue_pwm_show, blue_pwm_store);
static DEVICE_ATTR(apply, 0664, apply_show, apply_store);
static DEVICE_ATTR(mode, 0664, mode_show, mode_store);
//static DEVICE_ATTR(VDD, 0664, vdd_show, vdd_store);
static DEVICE_ATTR(fw_update, 0664, NULL, fw_update_store);
static DEVICE_ATTR(fw_ver, 0664, fw_ver_show, NULL);
static DEVICE_ATTR(frame, 0664, get_frame, set_frame);
static DEVICE_ATTR(speed, 0664, get_speed, set_speed);
static DEVICE_ATTR(led_en, 0664, led_en_show, led_en_store);
static DEVICE_ATTR(ec, 0664, ec_show, NULL);

static struct attribute *pwm_attrs[] = {
	&dev_attr_red_pwm.attr,
	&dev_attr_green_pwm.attr,
	&dev_attr_blue_pwm.attr,
	&dev_attr_apply.attr,
	&dev_attr_mode.attr,
//	&dev_attr_VDD.attr,
	&dev_attr_fw_update.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_frame.attr,
	&dev_attr_speed.attr,
	&dev_attr_led_en.attr,
	&dev_attr_ec.attr,
	NULL
};

static const struct attribute_group pwm_attr_group = {
	.attrs = pwm_attrs,
};


static void aura_sync_set(struct led_classdev *led,
			      enum led_brightness brightness)
{
	printk("[AURA_EC] aura_sync_set : %d.\n", brightness);
}

static enum led_brightness aura_sync_get(struct led_classdev *led_cdev)
{
	struct ene_8k41_platform_data *pdata;

	printk("[AURA_EC] aura_sync_get.\n");
	pdata = container_of(led_cdev, struct ene_8k41_platform_data, led);

	return pdata->led.brightness;
}

static int aura_sync_register(struct device *dev, struct ene_8k41_platform_data *pdata)
{
	if (!pdata->dongle)
		pdata->led.name = "aura_sync";
	else
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

	printk("[AURA_EC] ene_8k41_parse_dt\n");

	pdata->dongle = of_property_read_bool(np, "ene,station-dongle");
	printk("[AURA_EC] ene,station-dongle : %d\n", pdata->dongle);

	return 0;
}

extern bool station_aura_pogo;

static int ene_hid_probe(struct platform_device *pdev)
{
	int err = 0;
	struct ene_8k41_platform_data *platform_data;
	
	printk("[AURA_EC] ene_8k41_probe.\n");

	platform_data = kzalloc(sizeof(struct ene_8k41_platform_data), GFP_KERNEL);
	if (platform_data == NULL) {
		printk("[AURA_EC] alloc platform_data data fail.\r\n");
		return -1;
	}

// Parse platform data from dtsi
	err = ene_8k41_parse_dt(&pdev->dev, platform_data);
	if (err) {
		printk("[AURA_EC] ene_8k41_parse_dt get fail !!!\n");
		return -ENOMEM;
	}

// Check AURA_POGO exist or not
	printk("[AURA_EC] station_aura_pogo : %d\n", station_aura_pogo);
	if (station_aura_pogo){
		printk("[AURA_EC] Skip AURA_EC probe\n");
		goto skip_probe;
	}

// Wait 0.5s for IC power on.\n");
	msleep(500);
	printk("[AURA_EC] ==========\n");

// Check FW
	platform_data->fw_version = ene_CheckFirmwareVer();
	printk("[AURA_EC] FW VER : 0x%x\n", platform_data->fw_version);

// Register sys class  
	err = aura_sync_register(&pdev->dev, platform_data);
	if (err) {
		printk("[AURA_EC] Failed to register LED device: %d\n", err);
		goto unled;
	}
	err = sysfs_create_group(&platform_data->led.dev->kobj, &pwm_attr_group);
	if (err)
			goto unled;

	platform_data->current_mode = 0x0;
	platform_data->suspend_state = false;

// Set global variable
	g_pdata = platform_data;

	printk("[AURA_EC] ene_8k41_probe done.\n");
	return 0;

unled:
	aura_sync_unregister(platform_data);
skip_probe:
	return err;
}

static int ene_hid_remove(struct platform_device *pdev)
{
	int err = 0;

	if (station_aura_pogo){
		printk("[AURA_EC] ene_hid_remove : err %d\n", err);
		return 0;
	}
// unregister
	printk("[AURA_EC] sysfs_remove_group\n");
	sysfs_remove_group(&g_pdata->led.dev->kobj, &pwm_attr_group);

	printk("[AURA_EC] aura_sync_unregister\n");
	aura_sync_unregister(g_pdata);

// free pointer
//	printk("[AURA_EC] Free platform_data\n");
//	kfree(platform_data);

	printk("[AURA_EC] ene_hid_remove : err %d\n", err);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id ene_match_table[] = {
	{ .compatible = "ene8k41_station",},
	{ },
};
#else
#define ene_match_table NULL
#endif

static struct platform_driver ene_hid_driver = {
	.driver		= {
		.name		= "ene8k41_station",
		.owner = THIS_MODULE,
		.of_match_table	= ene_match_table,
	},
	.probe		= ene_hid_probe,
	.remove		= ene_hid_remove,
};

static int __init ene_hid_init(void)
{
	int ret;

	ret = platform_driver_register(&ene_hid_driver);
	if (ret)
		printk("[AURA_EC] ENE 8k41 driver int failed.\n");
	else
		printk("[AURA_EC] ENE 8k41 driver int success.\n");
	
	return ret;
}
module_init(ene_hid_init);

static void __exit ene_hid_exit(void)
{
	platform_driver_unregister(&ene_hid_driver);
}
module_exit(ene_hid_exit);

MODULE_AUTHOR("ASUS Deeo Ho");
MODULE_DESCRIPTION("Aura sync LEDs HID control");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("asus:ene-8k41");
