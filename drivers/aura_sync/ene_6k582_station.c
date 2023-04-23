/* Copyright (c) 2019, The Linux Foundation. All rights reserved.
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
#include "ene_6k582.h"

//For HID wait for completion
#include <linux/completion.h>
//#include <linux/msm_drm_notify.h>

// For Charger mode
extern bool g_Charger_mode;

#define RGB_MAX 21
extern uint8_t gDongleType;
extern int ec_i2c_set_gpio(u8 gpio, u8 value);

static struct ene_6k582_platform_data *g_pdata;
u32 addr_reg;
int g_fw_size=0;
int apply_pending=0;
int mode2_state=0;
int apply_state=0;
u32 g_delay_time=0;
int g_update_state=-1;
int g_panel_state=-1; // 1-->panel on  0--> panel off

static u32 g_red;
static u32 g_green;
static u32 g_blue;
static u32 g_mode;
static u32 g_speed;
static u32 g_led_on;
static u32 g_led2_on;

int g_sleep=-1;

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

static int ene_6k582_read_bytes(struct i2c_client *client, short addr, char *data)
{
	int err = 0;
	unsigned char buf[16] = {0};

	buf[0] = 0x00;
	buf[1] = (addr >> 8) & 0xFF;
	buf[2] = addr & 0xFF;

	pr_debug("[AURA_STATION] ene_6k582_read_bytes : addr: 0x%x, buf[1] : 0x%x, buf[2] : 0x%x\n", (addr & 0xFFFF), buf[1], buf[2]);
	err = i2c_write_bytes(client, buf, 3);	//set register address
	if (err !=1)
		printk("[AURA_STATION] i2c_write_bytes:err %d\n", err);

	buf[0] = 0x81;
	err = i2c_read_bytes(client, buf, 1, data, 1);	//send read command
	if (err != 2)
		printk("[AURA_STATION] i2c_read_bytes:err %d\n", err);

	return err;
}

static int ene_6k582_write_bytes(struct i2c_client *client, short addr, char value)
{
	int err = 0;
	unsigned char buf[16] = {0};

	buf[0] = 0x00;
	buf[1] = (addr >> 8) & 0xFF;
	buf[2] = addr & 0xFF;

	pr_debug("[AURA_STATION] ene_6k582_write_bytes : addr: 0x%x, buf[1] : 0x%x, buf[2] : 0x%x, value : 0x%x\n", (addr & 0xFFFF), buf[1], buf[2], value);
	err = i2c_write_bytes(client, buf, 3);	//set register address
	if (err !=1)
		printk("[AURA_STATION] i2c_write_bytes:err %d\n", err);

	buf[0] = 0x01;
	buf[1] = value;
	
	err = i2c_write_bytes(client, buf, 2);	//set register address
	if (err !=1)
		printk("[AURA_STATION] i2c_write_bytes:err %d\n", err);
	
	return err;
}

static int ene_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];

	printk("[AURA_STATION] ene_GetFirmwareSize.\n");
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

/*static char ene_CheckFirmwareVer(struct i2c_client *client)
{
	int err = 0;
	unsigned char data[2] = {0};

	err = ene_6k582_read_bytes(client, 0x80C0, data);
	if (err != 2){
		printk("[AURA_STATION] fw_check:err %d\n", err);
		data[0] = 0xFF;
	}

	return (u8) data[0];
}*/

static int ene_UpdateFirmware(struct i2c_client *client, char *fw_buf)
{
	int err = 0;
	struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	struct ene_checksum checksum = {0};
	unsigned char data[2] = {0};
	//unsigned char buf[129] = {0};
	//unsigned char tmp[129] = {0};
	unsigned char *buf;
	short addr;
	int retry = 0;
	int i=0;

	if(g_fw_size!=0x6c00)
		return -1;
	buf = kmalloc(sizeof(unsigned char)*129, GFP_DMA); //for i2c transfer --need to free
	//check ec mode 
	err = ene_6k582_read_bytes(client, 0xFF8F, data);
	if (err != 2)
		printk("[AURA_STATION] Check REG[0xFF8F]:err %d\n", err);

	printk("[AURA_STATION] REG[0xFF8F] : 0x%x\n", data[0]);

	/*if( (u8)(data[0]) < 0x70){
		platform_data->fw_version = ene_CheckFirmwareVer(client);
		printk("[AURA_STATION] FW VER : 0x%x\n", platform_data->fw_version);
		//if ( platform_data->fw_version == 0x0) { // rember change condition.
		if (0) {
			printk("[AURA_STATION] Don't need FW update.\n");
			kfree(buf);
			return 0;
		}
	} else
		printk("[ASUS_SYNC] FW is abnormal !!!");
	*/
	printk("[AURA_STATION] Start ene_UpdateFirmware\n");
	//check bb version
	err = ene_6k582_write_bytes(client, 0xF018, 0xCE);
	if (err !=1)
		printk("[AURA_STATION] REG[F010]: write err %d\n", err);

	err = ene_6k582_read_bytes(client, 0xF018, data);
	if (err != 2)
		printk("[AURA_STATION] REG[0xF018]: read err %d\n", err);

	printk("[AURA_STATION] REG[0xF018]: 0x%x\n", data[0]);  //expect 0xCE 

// (1) Reset 8051
	printk("[AURA_STATION] Reset 8051 =====\n");
	err = ene_6k582_write_bytes(client, 0xF010, 0x5);
	if (err !=1)
		printk("[AURA_STATION] REG[0xF010]: write err %d\n", err);

	msleep(500);

// (2) Read & Fill OSC32M freq.
	printk("[AURA_STATION] Read & Fill OSC32M freq =====\n");
	err = ene_6k582_write_bytes(client, 0xF808, 0xF0);
	if (err !=1)
		printk("[AURA_STATION] REG[0xF808]: write err %d\n", err);
	err = ene_6k582_write_bytes(client, 0xF809, 0x01);
	if (err !=1)
		printk("[AURA_STATION] REG[0xF809]: write err %d\n", err);
	err = ene_6k582_write_bytes(client, 0xF807, 0x90);
	if (err !=1)
		printk("[AURA_STATION] REG[0xF807]: write err %d\n", err);

	// wait 0xf800 [7] ==1
	while(1){	 // check cmd finished. 0xF800[7] = 1
		msleep(50);
		data[0] = 0x0;
		err = ene_6k582_read_bytes(client, 0xF800, data);
		if (err != 2) {
			printk("[AURA_STATION] REG[0xF800]: read err %d\n", err);
		}
		if ( ((data[0] >> 7) & 0x1) == 1 ){
			//printk("[AURA_STATION] check REG[0xF800] pass, 0x%02x\n", (data[0]));
			retry=0;
			break;
		}
		printk("[AURA_STATION] check REG[0xF800], 0x%02x\n", data[0]);
		retry++;
		if (retry > 10) {
			printk("[AURA_STATION] retry too many times: %d, Force exit!!!\n", retry);
			kfree(buf);
			return -1;
		}
	}


	err = ene_6k582_read_bytes(client, 0xF80B, data);
	if (err != 2)
		printk("[AURA_STATION] REG[0xF80B]: read err %d\n", err);

	printk("[AURA_STATION] REG[0xF80B]: 0x%x\n", data[0]);
	err =ene_6k582_write_bytes(client, 0xF806, data[0]);
	if (err != 1)
		printk("[AURA_STATION] REG[0xF806]: write err %d\n", err);

// (3) Fill program & Erase timing
	printk("[AURA_STATION] Fill program & Erase timing =====\n");
	err = ene_6k582_write_bytes(client, 0xF815, 0x10);
	if (err !=1)
		printk("[AURA_STATION] REG[0xF815]: write err %d\n", err);
	err = ene_6k582_write_bytes(client, 0xF816, 0x11);
	if (err !=1)
		printk("[AURA_STATION] REG[0xF816]: write err %d\n", err);
	err = ene_6k582_write_bytes(client, 0xF817, 0x06);
	if (err !=1)
		printk("[AURA_STATION] REG[0xF817]: write err %d\n", err);
	err = ene_6k582_write_bytes(client, 0xF818, 0x07);
	if (err !=1)
		printk("[AURA_STATION] REG[0xF818]: write err %d\n", err);

// (4) Erase
	printk("[AURA_STATION] Erase page from 0x0000 to 0x6F80 =====\n");
	for (addr = 0x0000 ; addr <= 0x6F80 ; addr+= 0x80) {

		
		err = ene_6k582_write_bytes(client, 0xF808, (addr & 0xFF));				//	Addr_L
		if (err !=1){
			printk("[AURA_STATION] set addr L : 0x%04x\n", addr);
			printk("[AURA_STATION] REG[0xF808]: write err %d\n", err);
		}
		err = ene_6k582_write_bytes(client, 0xF809, (addr >> 8) & 0xFF);		//	Addr_H
		if (err !=1){
			printk("[AURA_STATION] set addr H: 0x%04x\n", addr);
			printk("[AURA_STATION] REG[0xF809]: write err %d\n", err);
		}

		err = ene_6k582_write_bytes(client, 0xF807, 0x20);	// erase page cmd
		if (err !=1)
			printk("[AURA_STATION] REG[0xF807]: write err %d\n", err);

		while(1){	 // check cmd finished. 0xF800[7] = 1
			msleep(50);
			data[0] = 0x0;
			err = ene_6k582_read_bytes(client, 0xF800, data);
			if (err != 2) {
				printk("[AURA_STATION] REG[0xF800]: read err %d\n", err);
			}
			if ( ((data[0] >> 7) & 0x1) == 1 ){
				//printk("[AURA_STATION] check REG[0xF800] pass, 0x%02x\n", (data[0]));
				retry=0;
				break;
			}
			printk("[AURA_STATION] check REG[0xF800], 0x%02x\n", data[0]);
			retry++;
			if (retry > 10) {
				printk("[AURA_STATION] retry too many times: %d, Force exit!!!\n", retry);
				kfree(buf);
				return -1;
			}
		}
	}
	printk("[AURA_STATION] erase page finished. =====\n");
	
// (5) Program
	printk("[AURA_STATION] Program FW from 0x0000 to 0X6B80 =====\n");
	for (addr = 0x0000 ; addr <= 0x6B80 ; addr = addr + 0x80) {

		//printk("[AURA_STATION] set addr : 0x%04x\n", addr);
		err = ene_6k582_write_bytes(client, 0xF808, (addr & 0xFF));				//	Addr_L
		if (err !=1){
			printk("[AURA_STATION] set addr L: 0x%04x\n", addr);
			printk("[AURA_STATION] REG[0xF808]: write err %d\n", err);
		}
		err = ene_6k582_write_bytes(client, 0xF809, (addr >> 8) & 0xFF);		//	Addr_H
		if (err !=1){
			printk("[AURA_STATION] set addr H : 0x%04x\n", addr);
			printk("[AURA_STATION] REG[0xF809]: write err %d\n", err);
		}

		err = ene_6k582_write_bytes(client, 0xF807, 0x80);	// 3. Clear HVPL data
		if (err !=1)
			printk("[AURA_STATION] REG[0xF807]: write err %d\n", err);

		while(1){	 // 4. check cmd finished 0xF800[7] = 1
			msleep(50);
			data[0] = 0x0;
			err = ene_6k582_read_bytes(client, 0xF800, data);
			if (err != 2) {
				printk("[AURA_STATION] REG[0xF800]: read err %d\n", err);
			}
			if ( ((data[0] >> 7) & 0x1) == 1 ){
				//printk("[AURA_STATION] check REG[0xF800] pass, 0x%02x\n", (data[0]));
				retry = 0;
				break;
			}
			printk("[AURA_STATION] check REG[0xF800], 0x%02x\n", (data[0]));
			retry++;
			if (retry > 10) {
				printk("[AURA_STATION] retry too many times: %d, Force exit!!!\n", retry);
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
			printk("[AURA_STATION] i2c_write_bytes:err %d\n", err);

		// 6. Send data to buffer
		buf[0] = 0x05;
		memcpy( &(buf[1]), fw_buf + addr, 128); // copy fw_buf to buf

		for(i=1;i<=0x80;i++){  //cal the checksum
			checksum.wordsum += buf[i];
            checksum.xoR ^= buf[i];
		}
		checksum.sum = (u8)checksum.wordsum;
		checksum.partSum = (u8)checksum.wordsum;
		checksum.mode = 0; //CHECKSUM_FULL
		checksum.len += 0x80; // accumulate len written to flash.

		err = i2c_write_bytes(client, buf, 129);	 // 128 byte + 1 cmd = 129
		if (err !=1)
			printk("[AURA_STATION] i2c_write_bytes:err %d\n", err);

		// 7. Set address to 0xF807
		buf[0] = 0x00;
		buf[1] = 0xF8;	//Addr_H
		buf[2] = 0x07;	//Addr_L
		err = i2c_write_bytes(client, buf, 3);	//set register address
		if (err !=1)
			printk("[AURA_STATION] i2c_write_bytes:err %d\n", err);

		// 8. Program page
		buf[0] = 0x05;
		buf[1] = 0x70;
		err = i2c_write_bytes(client, buf, 2);
		if (err !=1)
			printk("[AURA_STATION] i2c_write_bytes:err %d\n", err);

		while(1){	 // 9. check cmd finished 0xF800[7] = 1
			msleep(50);
			data[0] = 0x0;
			err = ene_6k582_read_bytes(client, 0xF800, data);
			if (err != 2) {
				printk("[AURA_STATION] REG[0xF800]: read err %d\n", err);
			}
			if ( ((data[0] >> 7) & 0x1) == 1 ){
				//printk("[AURA_STATION] check REG[0xF800] pass, 0x%02x\n", (data[0]));
				retry = 0;
				break;
			}
			printk("[AURA_STATION] check REG[0xF800], 0x%02x\n", (data[0]));
			retry++;
			if (retry > 10) {
				printk("[AURA_STATION] retry too many times: %d, Force exit!!!\n", retry);
			    kfree(buf);
				return -1;
			}
		}
	}

	//program checksum
	printk("[AURA_STATION] set addr : 0x6FF0\n");
	err = ene_6k582_write_bytes(client, 0xF808, 0xF0);				//	Addr_L
	if (err !=1)
		printk("[AURA_STATION] REG[0xF808]: write err %d\n", err);
	err = ene_6k582_write_bytes(client, 0xF809, 0x6F);		//	Addr_H
	if (err !=1)
		printk("[AURA_STATION] REG[0xF809]: write err %d\n", err);
	err = ene_6k582_write_bytes(client, 0xF807, 0x80);	// 3. Clear HVPL data
	if (err !=1)
		printk("[AURA_STATION] REG[0xF807]: write err %d\n", err);

	while(1){	 // 4. check cmd finished 0xF800[7] = 1
		msleep(50);
		data[0] = 0x0;
		err = ene_6k582_read_bytes(client, 0xF800, data);
		if (err != 2) {
			printk("[AURA_STATION] REG[0xF800]: read err %d\n", err);
		}
		if ( ((data[0] >> 7) & 0x1) == 1 ){
			//printk("[AURA_STATION] check REG[0xF800] pass, 0x%02x\n", (data[0]));
			retry = 0;
			break;
		}
		printk("[AURA_STATION] check REG[0xF800], 0x%02x\n", (data[0]));
		retry++;
		if (retry > 10) {
			printk("[AURA_STATION] retry too many times: %d, Force exit!!!\n", retry);
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
		printk("[AURA_STATION] i2c_write_bytes:err %d\n", err);

	checksum.partSum = 0;

	// 6. Send data to buffer
	buf[0] = 0x05;
	memcpy( &(buf[1]), (char *)(&checksum), 16); // copy fw_buf to buf
	/*for(i=1;i<=16;i++){
		printk("[AURA_STATION] checksum buf[%d] = %x\n",i,buf[i]);
	}*/

	err = i2c_write_bytes(client, buf, 17);	 // 16 byte + 1 cmd = 17
	if (err !=1)
		printk("[AURA_STATION] i2c_write_bytes:err %d\n", err);

	err = ene_6k582_write_bytes(client, 0xF808, 0xFF);
	if (err != 1) {
		printk("[AURA_STATION] REG[0xF800]: read err %d\n", err);
	}
	err = ene_6k582_write_bytes(client, 0xF809, 0x6F);
	if (err != 1) {
		printk("[AURA_STATION] REG[0xF800]: read err %d\n", err);
	}

	// 7. Set address to 0xF807
	buf[0] = 0x00;
	buf[1] = 0xF8;	//Addr_H
	buf[2] = 0x07;	//Addr_L
	err = i2c_write_bytes(client, buf, 3);	//set register address
	if (err !=1)
		printk("[AURA_STATION] i2c_write_bytes:err %d\n", err);

	// 8. Program page
	buf[0] = 0x05;
	buf[1] = 0x70;
	err = i2c_write_bytes(client, buf, 2);
	if (err !=1)
		printk("[AURA_STATION] i2c_write_bytes:err %d\n", err);

	while(1){	 // 9. check cmd finished 0xF800[7] = 1
		msleep(50);
		data[0] = 0x0;
		err = ene_6k582_read_bytes(client, 0xF800, data);
		if (err != 2) {
			printk("[AURA_STATION] REG[0xF800]: read err %d\n", err);
		}
		if ( ((data[0] >> 7) & 0x1) == 1 ){
			//printk("[AURA_STATION] check REG[0xF800] pass, 0x%02x\n", (data[0]));
			retry = 0;
			break;
		}
		printk("[AURA_STATION] check REG[0xF800], 0x%02x\n", (data[0]));
		retry++;
		if (retry > 10) {
			printk("[AURA_STATION] retry too many times: %d, Force exit!!!\n", retry);
		    kfree(buf);
			return -1;
		}
	}



	
////////////////////////////

/*
	printk("[AURA_STATION] read FW in 8K41 rom +++\n");
	for (addr = 0x0000 ; addr <= 0x0080 ; addr = addr + 0x80) {
		printk("[AURA_STATION] set addr : 0x%04x\n", addr);
		err = ene_8k41_write_bytes(client, 0xF808, (addr & 0xFF));				//	Addr_L
		if (err !=1)
			printk("[AURA_STATION] REG[0xF808]: write err %d\n", err);
		err = ene_8k41_write_bytes(client, 0xF809, (addr >> 8) & 0xFF);		//	Addr_H
		if (err !=1)
			printk("[AURA_STATION] REG[0xF809]: write err %d\n", err);

		// 5. Set address to 0xF80B
		buf[0] = 0x00;
		buf[1] = 0xF8;	//Addr_H
		buf[2] = 0x0B;	//Addr_L
		err = i2c_write_bytes(client, buf, 3);	//set register address
		if (err !=1)
			printk("[AURA_STATION] i2c_write_bytes:err %d\n", err);

		buf[0] = 0x06;
		err = i2c_read_bytes(client, buf, 1, tmp, 128);	//set register address
		if (err !=2)
			printk("[AURA_STATION] i2c_read_bytes:err %d\n", err);

		for (i=0; i<128 ; i++)
			printk("[AURA_STATION] tmp[%d]:0x%x\n", i, tmp[i]);
	}
	printk("[AURA_STATION] read FW in 8K41 rom ---\n");
*/
////////////////////////////

// (9) Restart 8051
	printk("[AURA_STATION] Restart 8051.\n");
	err = ene_6k582_write_bytes(client, 0xF010, 0x04);
	if (err !=1)
		printk("[AURA_STATION] REG[0xF807]: write err %d\n", err);

	msleep(500);

	err = ene_6k582_write_bytes(client, 0xF018, 0x00);
	if (err !=1)
		printk("[AURA_STATION] REG[F010]: write err %d\n", err);

	platform_data->FW_update_done = true;
	printk("[AURA_STATION] ene_UpdateFirmware finished. %d\n", platform_data->FW_update_done);
	kfree(buf);
	return 0;
}

static ssize_t red_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;

	//printk("[AURA_STATION] red_pwm_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;
	pr_debug("[AURA_STATION] %s reg_val=%d \n",__func__,reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->RED_MAX), 255);

	mutex_lock(&g_pdata->ene_mutex);
	//printk("[AURA_STATION] client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);
	//printk("[AURA_STATION] first led red tmp %d\n", tmp);
	g_red=tmp;
	err = ene_6k582_write_bytes(client, 0x8404, tmp);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
	//printk("[AURA_STATION] second led red tmp %d\n", tmp);
	err = ene_6k582_write_bytes(client, 0x8405, tmp);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t red_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_read_bytes(client, 0x8404, data);
	if (err != 2)
		printk("[AURA_STATION] red_pwm_show:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t red1_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;

	//printk("[AURA_STATION] red1_pwm_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->RED_MAX), 255);

	mutex_lock(&g_pdata->ene_mutex);
	//printk("[AURA_STATION] client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);
	//printk("[AURA_STATION] red1 tmp %d\n", tmp);
	err = ene_6k582_write_bytes(client, 0x8405, tmp);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t red1_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_read_bytes(client, 0x8405, data);
	if (err != 2)
		printk("[AURA_STATION] red1_pwm_show:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}



static ssize_t green_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;

	//printk("[AURA_STATION] green_pwm_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;
	pr_debug("[AURA_STATION] %s reg_val=%d \n",__func__,reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->GREEN_MAX), 255);

	mutex_lock(&g_pdata->ene_mutex);
	//printk("[AURA_STATION] client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);
	//printk("[AURA_STATION] first led green tmp %d\n", tmp);
	g_green=tmp;
	err = ene_6k582_write_bytes(client, 0x8406, tmp);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
	//printk("[AURA_STATION] second led green tmp %d\n", tmp);
	err = ene_6k582_write_bytes(client, 0x8407, tmp);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t green_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_read_bytes(client, 0x8406, data);
	if (err != 2)
		printk("[AURA_STATION] green_pwm_show:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t green1_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;

	//printk("[AURA_STATION] green1_pwm_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->GREEN_MAX), 255);

	mutex_lock(&g_pdata->ene_mutex);
	//printk("[AURA_STATION] client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);
	//printk("[AURA_STATION] green1 tmp %d\n", tmp);
	err = ene_6k582_write_bytes(client, 0x8407, tmp);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t green1_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_read_bytes(client, 0x8407, data);
	if (err != 2)
		printk("[AURA_STATION] green1_pwm_show:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}



static ssize_t blue_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;

	//printk("[AURA_STATION] blue_pwm_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;
	pr_debug("[AURA_STATION] %s reg_val=%d \n",__func__,reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->BLUE_MAX), 255);

	mutex_lock(&g_pdata->ene_mutex);
	//printk("[AURA_STATION] client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);
	//printk("[AURA_STATION] first led blue tmp %d\n", tmp);
	g_blue=tmp;
	err = ene_6k582_write_bytes(client, 0x8408, tmp);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
	//printk("[AURA_STATION] second led blue tmp %d\n", tmp);
	err = ene_6k582_write_bytes(client, 0x8409, tmp);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t blue_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_read_bytes(client, 0x8408, data);
	if (err != 2)
		printk("[AURA_STATION] blue_pwm_show:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t blue1_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;

	//printk("[AURA_STATION] blue1_pwm_store.\n");

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	tmp = DIV_ROUND_CLOSEST(reg_val*(platform_data->BLUE_MAX), 255);

	mutex_lock(&g_pdata->ene_mutex);
	//printk("[AURA_STATION] client->addr : 0x%x,  reg_val : 0x%x.\n", client->addr, reg_val);
	//printk("[AURA_STATION] blue1 tmp %d\n", tmp);
	err = ene_6k582_write_bytes(client, 0x8409, tmp);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t blue1_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_read_bytes(client, 0x8409, data);
	if (err != 2)
		printk("[AURA_STATION] blue1_pwm_show:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}


static ssize_t apply_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	u32 val;
	int err = 0;
	ssize_t ret;
	int sleep_val=0;
	unsigned char data_r[2]={0};
	apply_state=0;

	ret = kstrtou32(buf, 10, &val);
	if (ret){
		apply_state=-1;
		return count;
	}
	pr_debug("[AURA_STATION] %s val=%d \n",__func__,val);
	mutex_lock(&g_pdata->ene_mutex);
	if (val > 0){
			printk("[AURA_STATION] Send apply. RGB:%d %d %d, mode:%d, speed:%d, led_on:%d, led2_on:%d\n", g_red, g_green, g_blue, g_mode, g_speed, g_led_on, g_led2_on);
			err = ene_6k582_write_bytes(client, 0x8452, 1);
			if (err !=1){
				apply_state=-1;
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
			}
			//apply_pending=0;

	} else{
		apply_state=-1;
		printk("[AURA_STATION] No send apply cmd.\n");
	}
	mutex_unlock(&g_pdata->ene_mutex);
	if(g_panel_state==0){
		//wake for aura on when panel off
		printk("[AURA_STATION] resume for aura on when panel off\n");
		mutex_lock(&g_pdata->ene_mutex);
		err = ene_6k582_read_bytes(client, 0x8453, data_r);
		if (err != 2){
			printk("[AURA_STATION] sleep_show 1st read before write:err %d\n", err);
		}
		mutex_unlock(&g_pdata->ene_mutex);
		if(data_r[0]==0xAA){
			printk("[AURA_STATION] ene_6k582_resume  ok, after read 1st\n");
			return count;
		}
		sleep_val = 0xAA;
		mutex_lock(&g_pdata->ene_mutex);
		err = ene_6k582_write_bytes(client, 0x8453, sleep_val);
		if (err !=1){
			printk("[AURA_STATION] ene_6k582_write_bytes 0X8453-0xAA :err %d\n", err);
		}
		err = ene_6k582_read_bytes(client, 0x8453, data_r);
		if (err != 2){
			printk("[AURA_STATION] sleep_show:err %d\n", err);
		}
		mutex_unlock(&g_pdata->ene_mutex);
		if(data_r[0]==0xAA){
			printk("[AURA_STATION] ene_6k582_resume ok after first write and check\n");
			return count;
		}else{
				//retry for the second time
				msleep(100);
				mutex_lock(&g_pdata->ene_mutex);
				err = ene_6k582_read_bytes(client, 0x8453, data_r);
				if (err != 2){
					printk("[AURA_STATION] sleep_show 2nd read before write:err %d\n", err);
				}
				mutex_unlock(&g_pdata->ene_mutex);
				if(data_r[0]==0xAA){
					printk("[AURA_STATION] ene_6k582_resume  ok, after read 2nd\n");
					return count;
				}
				sleep_val = 0xAA;
				mutex_lock(&g_pdata->ene_mutex);
				err = ene_6k582_write_bytes(client, 0x8453, sleep_val);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes 0X8453-0xAA :err %d\n", err);
				}
				err = ene_6k582_read_bytes(client, 0x8453, data_r);
				if (err != 2){
					printk("[AURA_STATION] sleep_show:err %d\n", err);
				}
				mutex_unlock(&g_pdata->ene_mutex);
				if(data_r[0]==0xAA){
					printk("[AURA_STATION] ene_6k582_resume ok after 2nd write and check\n");
						return count;
				}else{
					//retry for the third time
					msleep(100);
					mutex_lock(&g_pdata->ene_mutex);
					err = ene_6k582_read_bytes(client, 0x8453, data_r);
					if (err != 2){
						printk("[AURA_STATION] sleep_show:err %d\n", err);
					}
					mutex_unlock(&g_pdata->ene_mutex);
					if(data_r[0]==0xAA){
						printk("[AURA_STATION] ene_6k582_resume  ok, after read 3rd\n");
						return count;
					}
					sleep_val = 0xAA;
					mutex_lock(&g_pdata->ene_mutex);
					err = ene_6k582_write_bytes(client, 0x8453, sleep_val);
					if (err !=1){
						printk("[AURA_STATION] ene_6k582_write_bytes 0X8453-0xAA :err %d\n", err);
					}
					err = ene_6k582_read_bytes(client, 0x8453, data_r);
					if (err != 2){
						printk("[AURA_STATION] sleep_show:err %d\n", err);
					}
					mutex_unlock(&g_pdata->ene_mutex);
					if(data_r[0]==0xAA){
						printk("[AURA_STATION] ene_6k582_resume ok after 3rd write and check\n");
						return count;
					}else{
						printk("[AURA_STATION] ene_6k582_resume fail after retry three times\n");
					}
				} //end of third retry else
			}//end of second retry else
	}//end of g_panel_state=0
	return count;
}

static ssize_t apply_show(struct device *dev, struct device_attribute *attr,char *buf)
{

	return snprintf(buf, PAGE_SIZE,"%d\n", apply_state);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	u32 val;
	int err = 0;
	ssize_t ret;

	//printk("[AURA_STATION] mode_store.\n");
	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return count;
	pr_debug("[AURA_STATION] %s val=%d \n",__func__,val);
	g_mode=val;
	 switch(val){
	 	case 0:
			mutex_lock(&g_pdata->ene_mutex);
			err = ene_6k582_write_bytes(client, 0x8451,1);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				return count;
			}
			mutex_unlock(&g_pdata->ene_mutex);//we will do this in apply
			//apply_pending = 0x1;

	 		break;
	 	case 1: //static
			mutex_lock(&g_pdata->ene_mutex);
			//printk("[AURA_STATION] mode: 0x%x  0x8401-->0x03.\n",  val);
			err = ene_6k582_write_bytes(client, 0x8401, 0x04);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				return count;
			}
			//printk("[AURA_STATION] mode: 0x%x  0x840A-->0x01.\n",  val);
			err = ene_6k582_write_bytes(client, 0x840A, 0x01);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				return count;
			}
			//printk("[AURA_STATION] mode: 0x%x  0x840B-->0x00.\n",  val);
			err = ene_6k582_write_bytes(client, 0x840B, 0x00);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				return count;
			}
			//printk("[AURA_STATION] mode: 0x%x  0x840F-->0xE4.\n",  val);
			err = ene_6k582_write_bytes(client, 0x8451, 0xE4);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				return count;
			}
			mutex_unlock(&g_pdata->ene_mutex);
			//apply_pending = 0xE4;
	 		break;
	 	case 2:  //breath
			mutex_lock(&g_pdata->ene_mutex);
			//printk("[AURA_STATION] client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
			err = ene_6k582_write_bytes(client, 0x8401, 0x00);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8451, 0xE4);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				return count;
			}
			mutex_unlock(&g_pdata->ene_mutex);
			//apply_pending = 0xE4;
	 		break;
	 	case 3:  //strobbing
			mutex_lock(&g_pdata->ene_mutex);
			//printk("[AURA_STATION] client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
			/*err = ene_6k582_write_bytes(client, 0x8401, 0x02);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8451, 0xE2);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				return count;
			}*/
			err = ene_6k582_write_bytes(client, 0x8451, 0xE5);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				return count;
			}
			mutex_unlock(&g_pdata->ene_mutex);
			//apply_pending = 0xE2;
	 		break;
	 	case 4:  //color cycle
			mutex_lock(&g_pdata->ene_mutex);
			//printk("[AURA_STATION] client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
			err = ene_6k582_write_bytes(client, 0x8401, 0x03);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8451, 0xE3);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				return count;
			}
			mutex_unlock(&g_pdata->ene_mutex);
			//apply_pending = 0xE3;
	 		break;
	 	default:
	 		break;
	 }
	platform_data->current_mode = (u8)val;
//	ASUSEvtlog("[AURA_STATION] current_mode : %d\n", platform_data->current_mode);
	return count;
}

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	unsigned char data[4] = {0};
	int err = 0;
	return snprintf(buf, PAGE_SIZE,"%s\n", "no define");
	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_read_bytes(client, 0x8021, data);
	if (err != 2){
		printk("[AURA_STATION] mode_show:err %d\n", err);
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
	//struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	u32 val;
	//int err;
	ssize_t ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;
	pr_debug("[AURA_STATION] %s val=%d \n",__func__,val);
	if(val>0) {
		printk("[AURA_STATION] VDD set HIGH\n");
		ec_i2c_set_gpio(0x34, 1);
	}else {
		printk("[AURA_STATION] VDD set LOW\n");
		ec_i2c_set_gpio(0x34, 0);
	}
	
	return count;
}


static ssize_t vdd_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	char buffer[4] = {0};
	//int count = 2;
	//int ret = 0, len = 0;

	printk("[AURA_STATION] vdd_show\n");
	//i2c_get_gpio_data(buffer,0x34);
	//ret = hid_get_gpio_data(buffer, count, &len);
	//if (ret < 0)
		//return sprintf(buf, "%s\n", "HID_not_connect");

	return snprintf(buf, PAGE_SIZE,"0x%x\n", buffer[0]);
}

static ssize_t led_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	//struct i2c_client *client = to_i2c_client(dev->parent);
	//struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	u32 val;
	ssize_t ret;
	//int err = 0;
	return count; //for now

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;
	
}

static ssize_t led_en_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev->parent);
	//struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	//int val = 0;
	//unsigned char data[2] = {0};
	//int err = 0;
	return snprintf(buf, PAGE_SIZE,"No define logo_5v_en\n");

}

static ssize_t fw_update_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	//struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	int err = 0;
	char fw_name[128];
	unsigned char *fw_buf;
	int fw_size;

	memset(fw_name, 0, sizeof(fw_name));
	sprintf(fw_name, "%s", buf);
	fw_name[count-1] = '\0';
	
	printk("[AURA_STATION] fwname : %s\n", fw_name);

	// get fs_size
	fw_size = ene_GetFirmwareSize(fw_name);
	printk("[AURA_STATION] fwsize %d\n", fw_size);
	if(fw_size<=0){
		printk("[AURA_STATION] fwsize = %d, return\n",fw_size);
		return count;
	}

	g_fw_size = fw_size;
	
	// set fw_buf
	fw_buf = kmalloc(fw_size+1 ,GFP_ATOMIC);

	// read FW content
	if (ene_ReadFirmware(fw_name, fw_buf)) {
		printk("[AURA_STATION] ERROR: request_firmware failed\n");
		kfree(fw_buf);
		return count;
	}

	// Start update FW
	mutex_lock(&g_pdata->ene_mutex);
	err = ene_UpdateFirmware(client, fw_buf);
	if(err){
		printk("[AURA_STATION] ene_UpdateFirmware failede for the first time, and we will retry a time, err %d\n", err);
		err = ene_UpdateFirmware(client, fw_buf);
		if(err){
			printk("[AURA_STATION] ene_UpdateFirmware failed at the second time, err %d\n", err);
		}

	}

	// Update FW VER
	//platform_data->fw_version = ene_CheckFirmwareVer(client);
	//printk("[AURA_STATION] FW VER : 0x%x\n", platform_data->fw_version);

	mutex_unlock(&g_pdata->ene_mutex);
	kfree(fw_buf);
	return count;
}

static ssize_t fw_ver_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	//struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	unsigned char data[4] = {0};
	int err = 0;
	int fw_ver=0;
	char mode=0;

	mutex_lock(&g_pdata->ene_mutex);

	err = ene_6k582_read_bytes(client, 0x8021, data);
	if (err != 2){
		printk("[AURA_STATION] fw_ver_show:err %d\n", err);
		mutex_unlock(&g_pdata->ene_mutex);
		return snprintf(buf, PAGE_SIZE,"i2c_error\n");
	}
	fw_ver = data[0];
	//platform_data->fw_version = ene_CheckFirmwareVer(client);

	err = ene_6k582_read_bytes(client, 0x8020, data);
	if (err != 2){
		printk("[AURA_STATION] fw_ver_show:err %d\n", err);
		mutex_unlock(&g_pdata->ene_mutex);
		return snprintf(buf, PAGE_SIZE,"i2c_error\n");
	}
	mutex_unlock(&g_pdata->ene_mutex);

	fw_ver = fw_ver *256 + data[0];

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_read_bytes(client, 0x8451,&mode);
	if (err != 2){
		printk("[AURA_STATION] fw_ver_show:err %d\n", err);
		mutex_unlock(&g_pdata->ene_mutex);
		return snprintf(buf, PAGE_SIZE,"i2c_error\n");
	}
	mutex_unlock(&g_pdata->ene_mutex);

	if(((fw_ver & 0xff00) != 0x100) &&(mode != 0xCE)){  //when the ic is in loader mode
		fw_ver=0;
	}
	return snprintf(buf, PAGE_SIZE,"0x%x\n", fw_ver);
}

static ssize_t set_frame(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	u32 val;
	int err = 0;
	ssize_t ret;

	//printk("[AURA_STATION] set_frame.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	if (val > 255){
		printk("[AURA_STATION] Frame should not over 255.\n");
		return ret;
	}

	mutex_lock(&g_pdata->ene_mutex);
	printk("[AURA_STATION] set_frame client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
	err = ene_6k582_write_bytes(client, 0x8400, val);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t get_frame(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_read_bytes(client, 0x8400, data);
	if (err != 2)
		printk("[AURA_STATION] get_frame:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	pr_info("[AURA_STATION] %s data[0]=%d \n",__func__,data[0]);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t set_speed(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	u32 val;
	int err = 0;
	ssize_t ret;

	//printk("[AURA_STATION] set_speed.\n");
	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;
	pr_debug("[AURA_STATION] %s val=%d \n",__func__,val);
	if (val != 254 && val != 255 && val != 0 && val != 1 && val != 2){
		printk("[AURA_STATION] speed should be 254,255,0,1,2 .\n");
		return count;
	}
	/*if(val == 254){
		val = 5;
	}else if(val == 255){
		val = 10;
	}else if(val == 0){
		val = 20;
	}*/
	mutex_lock(&g_pdata->ene_mutex);
	g_speed=val;
	//printk("[AURA_STATION] client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
	err = ene_6k582_write_bytes(client, 0x8402, val);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;

}

static ssize_t get_speed(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_read_bytes(client, 0x8402, data);
	if (err != 2)
		printk("[AURA_STATION] get_speed:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}

static ssize_t set_cali_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	int red_val = 0, green_val = 0, blue_val = 0;

	sscanf(buf, "%d %d %d", &red_val, &green_val, &blue_val);
	printk("[AURA_STATION] set_cali_data, %d, %d, %d\n", red_val, green_val, blue_val);

	platform_data->RED_MAX = red_val;
	platform_data->GREEN_MAX = green_val;
	platform_data->BLUE_MAX = blue_val;

	return count;
}

static ssize_t get_cali_data(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);

	printk("[AURA_STATION] R:%d, G:%d, B:%d\n", platform_data->RED_MAX, platform_data->GREEN_MAX, platform_data->BLUE_MAX);
	return snprintf(buf, PAGE_SIZE,"R:%d, G:%d, B:%d\n", platform_data->RED_MAX, platform_data->GREEN_MAX, platform_data->BLUE_MAX);
}

static ssize_t read_reg_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_read_bytes(client, addr_reg, data);
	if (err != 2)
		printk("[AURA_STATION] read_reg_show:err %d\n", err);

	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n", data[0]);
}
static ssize_t read_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	addr_reg = 0;  //global addr, for reading reg.
	printk("[AURA_STATION] read_reg_store.\n");
	sscanf(buf, "%x", &addr_reg);
	printk("[AURA_STATION] addr=0x%x.\n",addr_reg);
	return count;
}


static ssize_t reg_debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	u32 val;
	u32 addr;
	int err = 0;
	//unsigned char data[4] = {0};

	printk("[AURA_STATION] reg_debug_store.\n");

	sscanf(buf, "%x %x ", &addr, &val);


	printk("[AURA_STATION] addr=0x%x, val=%x .\n",addr,val);
	

	mutex_lock(&g_pdata->ene_mutex);
	printk("[AURA_STATION] client->addr : 0x%x,  val : 0x%x.\n", client->addr, val);
	err = ene_6k582_write_bytes(client, addr, val);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}
static ssize_t mode2_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	int err = 0;
	//ssize_t ret;
	unsigned char rgb[RGB_MAX] = {0};
	//unsigned char tokenized_data[22] = {0};
	//unsigned char rainbow_mode = 0;
	unsigned char mode2 = 0;
	int i = 0,rgb_num=0;//rgb_num_t=0;
	long rgb_tmp = 0;
	//unsigned char reg[3] = {0};
	int addr= 0;
	int ntokens = 0;
	const char *cp = buf;
	const char *buf_tmp;
	mode2_state=0;

	sscanf(buf, "%d", &mode2);

	while ((cp = strpbrk(cp + 1, ",")))
	{
		ntokens++;  //the number of ","

	}
	printk("[AURA_STATION] mode2_store mode2=%d buf=%s ntokens=%d\n",mode2,buf,ntokens);
	if(ntokens > 6) 
	{
		printk("[AURA_STATION] mode2_store,wrong input,too many ntokens\n");
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
		printk("[AURA_STATION] mode2_store,wrong input,rgb_num != ntokens*3\n");
		mode2_state=-1;
		return count;
	}

	/*for(i=0;i<rgb_num;i++)
	{
		printk("AURA_POGO] mode2_store, rgb[%d]=0x%x \n",i,rgb[i]);
	}*/

	switch(mode2){
	 	case 0:
			mutex_lock(&g_pdata->ene_mutex);
			err = ene_6k582_write_bytes(client, 0x8451,1);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			mutex_unlock(&g_pdata->ene_mutex);
			//apply_pending = 0x1;
			break;
		case 1: //6 color rainbow
			if(ntokens!=6){
				printk("[AURA_STATION] mode2_store,wrong input. ntokensis not 6\n");
				mode2_state=-1;
				return count;
			}
			//printk("[AURA_STATION] mode2: 0x%x - 6 colors rainbow \n",  mode2);
			mutex_lock(&g_pdata->ene_mutex);
			for(i=0;i<6;i++){
				addr =  0x8421 + i;
				err = ene_6k582_write_bytes(client, addr, rgb[i*3]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+0x10, rgb[i*3+1]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+0x20, rgb[i*3+2]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
			}
			err = ene_6k582_write_bytes(client, 0x8401, 0x10);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x820F, 0xE1);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8401, 0x02);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8451, 0xE1);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			mutex_unlock(&g_pdata->ene_mutex);
			//apply_pending = 0xE1;
			break;
		case 2:  //static
			if(ntokens!=2){
				printk("[AURA_STATION] mode2_store,wrong input. ntokensis not 2\n");
				mode2_state=-1;
				return count;
			}
			mutex_lock(&g_pdata->ene_mutex);
			//printk("[AURA_STATION] client->addr : 0x%x,  mode2 : 0x%x -- static.\n", client->addr, mode2);
			for(i=0;i<2;i++){
				addr = 0x8404+i;
				err = ene_6k582_write_bytes(client, addr, rgb[i*3]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+2, rgb[i*3+1]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+4, rgb[i*3+2]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
			}
			err = ene_6k582_write_bytes(client, 0x8401, 0x04);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x840A, 0x01);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x840B, 0x0);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8451, 0xE4);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			mutex_unlock(&g_pdata->ene_mutex);
			//apply_pending =0xE4;
			break;
		case 3:  //breath at the same time
			if(ntokens!=2){
				printk("[AURA_STATION] mode2_store,wrong input. ntokensis not 2\n");
				mode2_state=-1;
				return count;
			}
			mutex_lock(&g_pdata->ene_mutex);
			//printk("[AURA_STATION] client->addr : 0x%x,  mode2 : 0x%x -- breath at the same time.\n", client->addr, mode2);
			for(i=0;i<2;i++){
				addr = 0x8404+i;
				err = ene_6k582_write_bytes(client, addr, rgb[i*3]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+2, rgb[i*3+1]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+4, rgb[i*3+2]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
			}
			err = ene_6k582_write_bytes(client, 0x8401, 0x0);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x840A, 0x01);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x840B, 0x0);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8451, 0xE4);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			mutex_unlock(&g_pdata->ene_mutex);
			//apply_pending = 0xE4;
			break;
		case 4:  //breath diff
			if(ntokens!=2){
				printk("[AURA_STATION] mode2_store,wrong input. ntokensis not 2\n");
				mode2_state=-1;
				return count;
			}
			mutex_lock(&g_pdata->ene_mutex);
			//printk("[AURA_STATION] client->addr : 0x%x,  mode2 : 0x%x -- breath diff.\n", client->addr, mode2);
			for(i=0;i<2;i++){
				addr = 0x8404+i;
				err = ene_6k582_write_bytes(client, addr, rgb[i*3]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+2, rgb[i*3+1]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+4, rgb[i*3+2]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
			}
			err = ene_6k582_write_bytes(client, 0x8401, 0x01);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x840A, 0x01);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x840B, 0x0);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8451, 0xE4);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			mutex_unlock(&g_pdata->ene_mutex);
			//apply_pending = 0xE4;
			break;
		case 5: //breath one led
			if(ntokens!=2){
				printk("[AURA_STATION] mode2_store,wrong input. ntokensis not 2\n");
				mode2_state=-1;
				return count;
			}
			mutex_lock(&g_pdata->ene_mutex);
			//printk("[AURA_STATION] client->addr : 0x%x,  mode2 : 0x%x -- breath one led.\n", client->addr, mode2);
			for(i=0;i<2;i++){
				addr = 0x8404+i;
				err = ene_6k582_write_bytes(client, addr, rgb[i*3]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+2, rgb[i*3+1]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+4, rgb[i*3+2]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
			}
			err = ene_6k582_write_bytes(client, 0x8401, 0x03);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8403, 0x00);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x840A, 0x01);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x840B, 0x0);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8451, 0xE4);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			mutex_unlock(&g_pdata->ene_mutex);
			//apply_pending = 0xE4;
			break;
		case 6: //commet
			if(ntokens!=2){
				printk("[AURA_STATION] mode2_store,wrong input. ntokensis not 2\n");
				mode2_state=-1;
				return count;
			}
			mutex_lock(&g_pdata->ene_mutex);
			//printk("[AURA_STATION] client->addr : 0x%x,  mode2 : 0x%x -- commet.\n", client->addr, mode2);
			for(i=0;i<2;i++){
				addr = 0x8404+i;
				err = ene_6k582_write_bytes(client, addr, rgb[i*3]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+2, rgb[i*3+1]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+4, rgb[i*3+2]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
			}
			err = ene_6k582_write_bytes(client, 0x8401, 0x0);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8402, 0xB); //SPEED -10ms/frame
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8403, 0x0); //direction
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8451, 0xE2);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			mutex_unlock(&g_pdata->ene_mutex);
			//apply_pending = 0xE2;
			break;
		case 7: //flash and dash
			if(ntokens!=2){
				printk("[AURA_STATION] mode2_store,wrong input. ntokensis not 2\n");
				mode2_state=-1;
				return count;
			}
			mutex_lock(&g_pdata->ene_mutex);
			//printk("[AURA_STATION] client->addr : 0x%x,  mode2 : 0x%x -- flash and dash.\n", client->addr, mode2);
			for(i=0;i<2;i++){
				addr = 0x8404+i;
				err = ene_6k582_write_bytes(client, addr, rgb[i*3]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+2, rgb[i*3+1]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+4, rgb[i*3+2]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
			}
			err = ene_6k582_write_bytes(client, 0x8401, 0x01);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8402, 0xB); //SPEED -10ms/frame
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8403, 0x0); //direction
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8451, 0xE2);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			mutex_unlock(&g_pdata->ene_mutex);
			//apply_pending = 0xE2;
	 		break;
		case 8: //commet in different direction
			if(ntokens!=2){
				printk("[AURA_STATION] mode2_store,wrong input. ntokensis not 2\n");
				mode2_state=-1;
				return count;
			}
			mutex_lock(&g_pdata->ene_mutex);
			//printk("[AURA_STATION] client->addr : 0x%x,  mode2 : 0x%x -- commet.\n", client->addr, mode2);
			for(i=0;i<2;i++){
				addr = 0x8404+i;
				err = ene_6k582_write_bytes(client, addr, rgb[i*3]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+2, rgb[i*3+1]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+4, rgb[i*3+2]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
			}
			err = ene_6k582_write_bytes(client, 0x8401, 0x0);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8402, 0xB); //SPEED -10ms/frame
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8403, 0x1); //direction
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8451, 0xE2);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			mutex_unlock(&g_pdata->ene_mutex);
			//apply_pending = 0xE2;
			break;
		case 9: //flash and dash in the different direction
			if(ntokens!=2){
				printk("[AURA_STATION] mode2_store,wrong input. ntokensis not 2\n");
				mode2_state=-1;
				return count;
			}
			mutex_lock(&g_pdata->ene_mutex);
			//printk("[AURA_STATION] client->addr : 0x%x,  mode2 : 0x%x -- flash and dash.\n", client->addr, mode2);
			for(i=0;i<2;i++){
				addr = 0x8404+i;
				err = ene_6k582_write_bytes(client, addr, rgb[i*3]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+2, rgb[i*3+1]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+4, rgb[i*3+2]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
			}
			err = ene_6k582_write_bytes(client, 0x8401, 0x01);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8402, 0xB); //SPEED -10ms/frame
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8403, 0x1); //direction
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8451, 0xE2);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			mutex_unlock(&g_pdata->ene_mutex);
			//apply_pending = 0xE2;
	 		break;
	 	case 10:
	 		if(ntokens!=6){
				printk("[AURA_STATION] mode2_store,wrong input. ntokensis not 6\n");
				mode2_state=-1;
				return count;
			}
			//printk("[AURA_STATION] mode2: 0x%x - 6 colors rainbow \n",  mode2);
			mutex_lock(&g_pdata->ene_mutex);
			for(i=0;i<6;i++){
				addr =  0x8421 + i;
				err = ene_6k582_write_bytes(client, addr, rgb[i*3]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+0x10, rgb[i*3+1]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+0x20, rgb[i*3+2]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
			}
			err = ene_6k582_write_bytes(client, 0x8401, 0x10);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x820F, 0xE1);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8401, 0x03);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8451, 0xE1);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			mutex_unlock(&g_pdata->ene_mutex);
			//apply_pending = 0xE1;
			break;
		case 11: //breath one led
			if(ntokens!=2){
				printk("[AURA_STATION] mode2_store,wrong input. ntokensis not 2\n");
				mode2_state=-1;
				return count;
			}
			mutex_lock(&g_pdata->ene_mutex);
			//printk("[AURA_STATION] client->addr : 0x%x,  mode2 : 0x%x -- breath one led.\n", client->addr, mode2);
			for(i=0;i<2;i++){
				addr = 0x8404+i;
				err = ene_6k582_write_bytes(client, addr, rgb[i*3]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+2, rgb[i*3+1]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
				err = ene_6k582_write_bytes(client, addr+4, rgb[i*3+2]);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
					mutex_unlock(&g_pdata->ene_mutex);
					mode2_state=-1;
					return count;
				}
			}
			err = ene_6k582_write_bytes(client, 0x8401, 0x02);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8403, 0x01);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x840A, 0x01);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x840B, 0x0);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			err = ene_6k582_write_bytes(client, 0x8451, 0xE4);
			if (err !=1){
				printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
				mode2_state=-1;
				return count;
			}
			mutex_unlock(&g_pdata->ene_mutex);
			//apply_pending = 0xE4;
			break;
	 	default:
	 		break;
	 }
	platform_data->current_mode = (u8)mode2;
//	ASUSEvtlog("[AURA_STATION] current_mode : %d\n", platform_data->current_mode);
	return count;
}

static ssize_t mode2_show(struct device *dev, struct device_attribute *attr,char *buf)
{

	return snprintf(buf, PAGE_SIZE,"%d\n",mode2_state);
}


static ssize_t led_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	//struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	u32 val;
	ssize_t ret;
	//int err = 0;

	int err = 0;
	unsigned char data[2] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;
	pr_debug("[AURA_STATION] %s val= %d\n", __func__,val);
	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_read_bytes(client, 0x840c, data);
	if (err != 2){
		printk("[AURA_STATION] led_on_store:err %d\n", err);
	}
	mutex_unlock(&g_pdata->ene_mutex);
	if(val==1){  //ON
		g_led_on=1;
		val = data[0] | 0x03;
	}else{ //OFF
		g_led_on=0;
		val = data[0] & 0xFC;
	}
	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_write_bytes(client, 0x840c, val);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t led_on_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	int err = 0;
	unsigned char data[2] = {0};
	int val = -1;

	err = ene_6k582_read_bytes(client, 0x840c, data);
	if (err != 2){
		printk("[AURA_STATION] led_on_show:err %d\n", err);
	}
	mutex_unlock(&g_pdata->ene_mutex);
	//printk("[AURA_STATION] led_on_show,value = 0x%x\n");
	if((data[0]&0x03) == 0x03){
		val = 1;
	}else if((data[0]&0x03) == 0){
		val = 0;
	}
	return snprintf(buf, PAGE_SIZE,"%d\n",val);
}
static ssize_t led2_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	//struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	u32 val;
	ssize_t ret;
	//int err = 0;

	int err = 0;
	unsigned char data[2] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;
	pr_debug("[AURA_STATION] %s val= %d\n", __func__,val);
	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_read_bytes(client, 0x840c, data);
	if (err != 2){
		printk("[AURA_STATION] led_on_store:err %d\n", err);
	}
	mutex_unlock(&g_pdata->ene_mutex);
	if(val==1){  //ON
		g_led2_on=1;
		val = data[0] | 0x0C;
	}else{ //OFF
		g_led2_on=0;
		val = data[0] & 0xF3;
	}
	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_write_bytes(client, 0x840c, val);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t led2_on_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	int err = 0;
	unsigned char data[2] = {0};
	int val = -1;

	err = ene_6k582_read_bytes(client, 0x840c, data);
	if (err != 2){
		printk("[AURA_STATION] led_on_show:err %d\n", err);
	}
	mutex_unlock(&g_pdata->ene_mutex);
	//printk("[AURA_STATION] led2_on_show,value = 0x%x\n");
	if((data[0]&0x0C) == 0xC){
		val = 1;
	}else if((data[0]&0x0C) == 0){
		val = 0;
	}
	return snprintf(buf, PAGE_SIZE,"%d\n",val);
}

static ssize_t sleep_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	//struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	u32 val;
	//unsigned char data[2] = {0};
	int err = 0;

	err = kstrtou32(buf, 10, &val);
	if (err)
		return count;

	if(val==1){  //stop mode
		g_sleep=0;
		val = 0xBE;
		mutex_lock(&g_pdata->ene_mutex);
		err = ene_6k582_write_bytes(client, 0X8453, val);
		if (err !=1)
			printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
		mutex_unlock(&g_pdata->ene_mutex);
	}else{ //normal mode
		g_sleep=1;
	}
	return count;
}

static ssize_t sleep_time_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	//struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	int err = 0;
	unsigned char data[2]={0};
	u32 delay_time=20000; //the time is delay_time*2.5ms.
	err = kstrtou32(buf, 10, &delay_time);
	if (err)
		return count;
	g_delay_time=delay_time;
	data[1]=(delay_time & 0xff00)>>8;
	data[0]=(delay_time & 0xff);
	printk("[AURA_STATION] sleep_time_store data[1]=%x data[0]=%x delay_time=%d\n",data[1],data[0],delay_time);
	//set delay time
	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_write_bytes(client, 0X8454,data[1]); //H
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
	err = ene_6k582_write_bytes(client, 0X8456, data[0]);   //L
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
	mutex_unlock(&g_pdata->ene_mutex);

	return count;
}


static ssize_t sleep_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	int err = 0;
	unsigned char data[2] = {0};
	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_read_bytes(client, 0x8453, data);
	if (err != 2){
		printk("[AURA_STATION] sleep_show:err %d\n", err);
	}
	mutex_unlock(&g_pdata->ene_mutex);
	return snprintf(buf, PAGE_SIZE,"%d\n",data[0]);
}
static ssize_t update_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	err = kstrtou32(buf, 10, &val);
	if (err)
		return count;
	printk("[AURA_STATION]update_state_store val=%d \n",val);
	if(val==1){  //update start
		g_update_state=1;
	}else{     //update end
		g_update_state=0;
	}
	return count;
}

static ssize_t update_state_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE,"%d\n",g_update_state);
}

static DEVICE_ATTR(red_pwm, 0664, red_pwm_show, red_pwm_store);
static DEVICE_ATTR(green_pwm, 0664, green_pwm_show, green_pwm_store);
static DEVICE_ATTR(blue_pwm, 0664, blue_pwm_show, blue_pwm_store);
static DEVICE_ATTR(red1_pwm, 0664, red1_pwm_show, red1_pwm_store);
static DEVICE_ATTR(green1_pwm, 0664, green1_pwm_show, green1_pwm_store);
static DEVICE_ATTR(blue1_pwm, 0664, blue1_pwm_show, blue1_pwm_store);
static DEVICE_ATTR(apply, 0664, apply_show, apply_store);
static DEVICE_ATTR(mode, 0664, mode_show, mode_store);
static DEVICE_ATTR(VDD, 0664, vdd_show, vdd_store);
static DEVICE_ATTR(fw_update, 0664, NULL, fw_update_store);
static DEVICE_ATTR(fw_ver, 0664, fw_ver_show, NULL);
static DEVICE_ATTR(frame, 0664, get_frame, set_frame);
static DEVICE_ATTR(speed, 0664, get_speed, set_speed);
static DEVICE_ATTR(led_en, 0664, led_en_show, led_en_store);
static DEVICE_ATTR(Calibration, 0664, get_cali_data, set_cali_data);
static DEVICE_ATTR(read_reg, 0664, read_reg_show, read_reg_store);
static DEVICE_ATTR(reg_debug, 0664, NULL, reg_debug_store);
static DEVICE_ATTR(mode2, 0664, mode2_show, mode2_store);
static DEVICE_ATTR(led_on, 0664, led_on_show, led_on_store);
static DEVICE_ATTR(led2_on, 0664, led2_on_show, led2_on_store);
static DEVICE_ATTR(sleep, 0664, sleep_show, sleep_store);
static DEVICE_ATTR(sleep_time, 0664, NULL, sleep_time_store);
static DEVICE_ATTR(update_state, 0664, update_state_show, update_state_store);

static struct attribute *pwm_attrs[] = {
	&dev_attr_red_pwm.attr,
	&dev_attr_green_pwm.attr,
	&dev_attr_blue_pwm.attr,
	&dev_attr_red1_pwm.attr,
	&dev_attr_green1_pwm.attr,
	&dev_attr_blue1_pwm.attr,
	&dev_attr_apply.attr,
	&dev_attr_mode.attr,
	&dev_attr_VDD.attr,
	&dev_attr_fw_update.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_frame.attr,
	&dev_attr_speed.attr,
	&dev_attr_led_en.attr,
	&dev_attr_Calibration.attr,
	&dev_attr_read_reg.attr,
	&dev_attr_reg_debug.attr,
	&dev_attr_mode2.attr,
	&dev_attr_led_on.attr,
	&dev_attr_led2_on.attr,
	&dev_attr_sleep.attr,
	&dev_attr_sleep_time.attr,
	&dev_attr_update_state.attr,
	NULL
};

static const struct attribute_group pwm_attr_group = {
	.attrs = pwm_attrs,
};

// Register FB notifier +++
/*
static int ene_6k582_fb_callback(struct notifier_block *nb, unsigned long val, void *data)
{
	struct ene_6k582_platform_data *platform_data;
	struct msm_drm_notifier *evdata = data;
	unsigned int blank;
	struct i2c_client *client ;   //= to_i2c_client(dev->parent)
	unsigned char data_r[2]={0};
	int err;
	int sleep_val=0;

	if (val != MSM_DRM_EARLY_EVENT_BLANK)
		return 0;

	if (evdata->id != 0)	// id=0 is internal display, external is 1
		return 0;

	printk("[AURA_STATION] go to the ene_6k582_fb_callback value = %d msm_drm_display_id = %d\n", (int)val, evdata->id);
	platform_data = container_of(nb, struct ene_6k582_platform_data, notifier);
	client = to_i2c_client(platform_data->led.dev->parent);
	if (evdata && evdata->data && val == MSM_DRM_EARLY_EVENT_BLANK && platform_data) {
		blank = *(int *)(evdata->data);

	printk("[AURA_STATION] go to the blank value = %d\n", (int)blank);

	switch (blank) {
		case MSM_DRM_BLANK_POWERDOWN:
			if(g_update_state==1){
				printk("[AURA_STATION]it is updating, not sleep\n");
				return NOTIFY_OK;
			}
			if(platform_data->current_mode == 0){
				g_panel_state=0;
				printk("[AURA_STATION] ene_6k582_suspend it is mode 0,suspend. \n");
				sleep_val = 0xBE;
				mutex_lock(&g_pdata->ene_mutex);
				err = ene_6k582_write_bytes(client, 0x8453, sleep_val);
				if (err !=1)
					printk("[AURA_STATION] ene_6k582_write_bytes 0x8453-0xBE:err %d\n", err);
				mutex_unlock(&g_pdata->ene_mutex);
			}
			break;
		case MSM_DRM_BLANK_UNBLANK:
			g_panel_state=1;
			if(gDongleType!=2){
				printk("[AURA_STATION] ene_6k582_resume return for station removed\n");
				return NOTIFY_OK;
			}
			if(platform_data->current_mode == 0){
				printk("[AURA_STATION] ene_6k582_resume it is mode 0,resume\n");
				mutex_lock(&g_pdata->ene_mutex);
				err = ene_6k582_read_bytes(client, 0x8453, data_r);
				if (err != 2){
					printk("[AURA_STATION] sleep_show 1st read before write:err %d\n", err);
				}
				mutex_unlock(&g_pdata->ene_mutex);
				if(data_r[0]==0xAA){
					printk("[AURA_STATION] ene_6k582_resume  ok, after read 1st\n");
					return NOTIFY_OK;
				}
				sleep_val = 0xAA;
				mutex_lock(&g_pdata->ene_mutex);
				err = ene_6k582_write_bytes(client, 0x8453, sleep_val);
				if (err !=1){
					printk("[AURA_STATION] ene_6k582_write_bytes 0X8453-0xAA :err %d\n", err);
				}
				err = ene_6k582_read_bytes(client, 0x8453, data_r);
				if (err != 2){
					printk("[AURA_STATION] sleep_show:err %d\n", err);
				}
				mutex_unlock(&g_pdata->ene_mutex);
				if(data_r[0]==0xAA){
					printk("[AURA_STATION] ene_6k582_resume ok after first write and check\n");
					return NOTIFY_OK;
				}else{
					//retry for the second time
					msleep(100);
					mutex_lock(&g_pdata->ene_mutex);
					err = ene_6k582_read_bytes(client, 0x8453, data_r);
					if (err != 2){
						printk("[AURA_STATION] sleep_show 2nd read before write:err %d\n", err);
					}
					mutex_unlock(&g_pdata->ene_mutex);
					if(data_r[0]==0xAA){
						printk("[AURA_STATION] ene_6k582_resume  ok, after read 2nd\n");
						return NOTIFY_OK;
					}
					sleep_val = 0xAA;
					mutex_lock(&g_pdata->ene_mutex);
					err = ene_6k582_write_bytes(client, 0x8453, sleep_val);
					if (err !=1){
						printk("[AURA_STATION] ene_6k582_write_bytes 0X8453-0xAA :err %d\n", err);
					}
					err = ene_6k582_read_bytes(client, 0x8453, data_r);
					if (err != 2){
						printk("[AURA_STATION] sleep_show:err %d\n", err);
					}
					mutex_unlock(&g_pdata->ene_mutex);
					if(data_r[0]==0xAA){
						printk("[AURA_STATION] ene_6k582_resume ok after 2nd write and check\n");
						return NOTIFY_OK;
					}else{
						//retry for the third time 
						msleep(100);
						mutex_lock(&g_pdata->ene_mutex);
						err = ene_6k582_read_bytes(client, 0x8453, data_r);
						if (err != 2){
							printk("[AURA_STATION] sleep_show:err %d\n", err);
						}
						mutex_unlock(&g_pdata->ene_mutex);
						if(data_r[0]==0xAA){
							printk("[AURA_STATION] ene_6k582_resume  ok, after read 3rd\n");
							return NOTIFY_OK;
						}
						sleep_val = 0xAA;
						mutex_lock(&g_pdata->ene_mutex);
						err = ene_6k582_write_bytes(client, 0x8453, sleep_val);
						if (err !=1){
							printk("[AURA_STATION] ene_6k582_write_bytes 0X8453-0xAA :err %d\n", err);
						}
						err = ene_6k582_read_bytes(client, 0x8453, data_r);
						if (err != 2){
							printk("[AURA_STATION] sleep_show:err %d\n", err);
						}
						mutex_unlock(&g_pdata->ene_mutex);
						if(data_r[0]==0xAA){
							printk("[AURA_STATION] ene_6k582_resume ok after 3rd write and check\n");
							return NOTIFY_OK;
						}else{
							printk("[AURA_STATION] ene_6k582_resume fail after retry three times\n");
						}
					}
				}
			}//end of if(platform_data->current_mode == 0)
			//platform_data->suspend_state = false;
			break;
		default:
			break;
		}
	}

	return NOTIFY_OK;
}


static struct notifier_block ene_6k582_noti_block = {
	.notifier_call = ene_6k582_fb_callback,
};
*/
// Register FB notifier ---

static void aura_sync_set(struct led_classdev *led,
			      enum led_brightness brightness)
{
	printk("[AURA_STATION] aura_sync_set : %d.\n", brightness);
}

static enum led_brightness aura_sync_get(struct led_classdev *led_cdev)
{
	struct ene_6k582_platform_data *pdata;

	printk("[AURA_STATION] aura_sync_get.\n");
	pdata = container_of(led_cdev, struct ene_6k582_platform_data, led);

	return pdata->led.brightness;
}

static int aura_sync_register(struct device *dev, struct ene_6k582_platform_data *pdata)
{
	//if (!pdata->dongle)
	//	pdata->led.name = "aura_sync";
	//else
		pdata->led.name = "aura_station";

	pdata->led.brightness = LED_OFF;
	pdata->led.max_brightness = LED_HALF;
	pdata->led.default_trigger = "none";
	pdata->led.brightness_set = aura_sync_set;
	pdata->led.brightness_get = aura_sync_get;

	return led_classdev_register(dev, &pdata->led);
}

static void aura_sync_unregister(struct ene_6k582_platform_data *pdata)
{
	led_classdev_unregister(&pdata->led);
}

static int ene_6k582_parse_dt(struct device *dev, struct ene_6k582_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	//int retval;
	//u32 voltage_supply[2];

	printk("[AURA_STATION] ene_6k582_parse_dt\n");

	pdata->dongle = of_property_read_bool(np, "ene,station-dongle");
	printk("[AURA_STATION] dongle : %d\n", pdata->dongle);

	/*if (pdata->dongle) {
		pdata->int_gpio = -2;
		pdata->power_gpio = -2;
		pdata->logo_5v_en = -2;
		pdata->regulator_vdd_vmin = 0;
		pdata->regulator_vdd_vmax = 0;
		pdata->regulator_current = 0;
	}*/

	return 0;
}

bool station_aura_pogo=true;
EXPORT_SYMBOL(station_aura_pogo);

static int ene_6k582_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	//unsigned char buf[128] = {0};
	//int readlen = 0;
	//unsigned char *fw_buf;
	//int fw_size;
	struct ene_6k582_platform_data *platform_data;
	//unsigned char data[4] = {0};
	unsigned char data[2] = {0};
	u32 val;
	printk("[AURA_STATION] ene_6k582_probe.\n");

	if(g_Charger_mode) {
		printk("[AURA_STATION] In charger mode, stop ene_6k582_probe\n");
		return 0;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
	  //goto exit_check_functionality_failed;
	} else
		printk("[AURA_STATION] I2C function test pass\n");

	printk("[AURA_STATION] client->addr : 0x%x\n", client->addr);
	//printk("[AURA_STATION] check data[0]: 0x%x\n", data[0]);

	platform_data = devm_kzalloc(&client->dev, sizeof(struct ene_6k582_platform_data), GFP_KERNEL);
	if (!platform_data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, platform_data);
// Parse platform data from dtsi
	err = ene_6k582_parse_dt(&client->dev, platform_data);
	if (err) {
		printk("[AURA_STATION] ene_6k582_parse_dt get fail !!!\n");
		return -ENOMEM;
	}

// Wait 0.5s for IC power on.\n");
//	hid_to_gpio_set(0x3F, 1);
	err = ec_i2c_set_gpio(0x34, 1);
	if (err<0) {
		printk("[AURA_STATION] ec_i2c_set_gpio err<0 ;IC power on fail!!!\n");
		//return -ENOMEM;
	}
	msleep(500);
	printk("[AURA_STATION] ==========\n");

// Check I2C status for AURA_STATION
/*
	err = ene_6k582_read_bytes(client, 0xF204, data);
	if (err != 2){
		printk("[AURA_STATION] check IC :err %d\n", err);
		printk("[AURA_STATION] Skip AURA_POGO probe\n");
		goto exit_check_functionality_failed;
	}
	printk("[AURA_STATION] data[0] : 0x%x\n", data[0]);
*/
// Check FW
	//platform_data->fw_version = ene_CheckFirmwareVer(client);
	//printk("[AURA_STATION] FW VER : 0x%x\n", platform_data->fw_version);

	
//skip_auto_update:
/*
// Setting PWM inverse
	printk("[AURA_STATION] set PWM inverse, REG[0xF207] = 0x8\n");
	err = ene_6k582_write_bytes(client, 0xF207, 0x8);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);

	err = ene_6k582_read_bytes(client, 0xF207, data);
	if (err != 2)
		printk("[AURA_STATION] ene_6k582_read_bytes:err %d\n", err);

	printk("[AURA_STATION] REG[0xF207]: 0x%x\n", data[0]);
	printk("[AURA_STATION] ==========\n");
*/

// Register sys class  
	err = aura_sync_register(&client->dev, platform_data);
	if (err) {
		printk("[AURA_STATION] Failed to register LED device: %d\n", err);
		goto unled;
	}
	err = sysfs_create_group(&platform_data->led.dev->kobj, &pwm_attr_group);
	if (err)
			goto unled;

	mutex_init(&platform_data->ene_mutex);
	platform_data->current_mode = 0x0;
	platform_data->suspend_state = false;
	platform_data->FW_update_done = false;

	//platform_data->hid_suspend_id = hid_vote_register("AURA_POGO");
	//platform_data->notifier = ene_6k582_noti_block;
	//msm_drm_register_client(&platform_data->notifier);

// Default Calibration Data
	platform_data->RED_MAX = 255;
	platform_data->GREEN_MAX = 255;
	platform_data->BLUE_MAX = 255;

// Set global variable
	g_pdata = platform_data;
	g_red=-1;
    g_green=-1;
   	g_blue=-1;
    g_mode=-1;
    g_speed=-1;
    g_led_on=-1;
    g_led2_on=-1;
// reset all variable
// reset RGB
	printk("[AURA_STATION] reset all variables to 0\n");
	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_write_bytes(client, 0x8404, 0);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
	err = ene_6k582_write_bytes(client, 0x8405, 0);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
	err = ene_6k582_write_bytes(client, 0x8406, 0);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
	err = ene_6k582_write_bytes(client, 0x8407, 0);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
	err = ene_6k582_write_bytes(client, 0x8408, 0);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
	err = ene_6k582_write_bytes(client, 0x8409, 0);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
// reset led
	err = ene_6k582_read_bytes(client, 0x840c, data);
	if (err != 2)
		printk("[AURA_STATION] led_on_store:err %d\n", err);
	val = data[0] & 0xFC;
	err = ene_6k582_write_bytes(client, 0x840c, val);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
	err = ene_6k582_read_bytes(client, 0x840c, data);
	if (err != 2)
		printk("[AURA_STATION] led_on_store:err %d\n", err);
	val = data[0] & 0xF3;
	err = ene_6k582_write_bytes(client, 0x840c, val);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
// reset mode
	err = ene_6k582_write_bytes(client, 0x8451, 1);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
//apply			
	err = ene_6k582_write_bytes(client, 0x8452, 1);
	if (err !=1){
		apply_state=-1;
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
	}
				
	mutex_unlock(&g_pdata->ene_mutex);
	
	
	station_aura_pogo = true;
	printk("[AURA_STATION] ene_6k582_probe done.\n");
	///kfree(fw_buf);
	return 0;

unled:
	aura_sync_unregister(platform_data);
	printk("[AURA_STATION] ENE 6k852 power off.\n");
	return err;

/*exit_check_functionality_failed:
	printk("[AURA_STATION] ene_6k582_probe fail !!!\n");
	station_aura_pogo = false;
	kfree(fw_buf);
	return err;*/
}

static int ene_6k582_remove(struct i2c_client *client)
{
	int err = 0;
	struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	if(g_Charger_mode) {
		printk("[AURA_STATION] In charger mode, stop ene_6k582_remove\n");
		return 0;
	}
	if (!station_aura_pogo){
		printk("[AURA_STATION] ene_6k582_remove : err %d\n", err);
		return 0;		
	}

	ec_i2c_set_gpio(0x34, 0);

// unregister
	printk("[AURA_STATION] sysfs_remove_group\n");
	sysfs_remove_group(&platform_data->led.dev->kobj, &pwm_attr_group);

	printk("[AURA_STATION] aura_sync_unregister\n");
	aura_sync_unregister(platform_data);

// free pointer
//	printk("[AURA_STATION] Free platform_data\n");
//	kfree(platform_data);

	mutex_destroy(&platform_data->ene_mutex);
	//hid_vote_unregister(platform_data->hid_suspend_id, "AURA_POGO");
	//msm_drm_unregister_client(&platform_data->notifier);

	printk("[AURA_STATION] ene_6k582_remove : err %d\n", err);
	return 0;
}

int ene_6k582_suspend(struct device *dev)
{
	int err = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	u32 val;
	u32 delay_time=4000; //the time is delay_time*2.5ms,default is 10s.
	if(g_Charger_mode) {
		printk("[AURA_STATION] In charger mode, stop ene_6k582_suspend\n");
		return 0;
	}
	if(g_sleep==1){
		printk("[AURA_STATION] ene_6k582_suspend not sleep,  return\n");
		return 0;
	}
	if(g_delay_time!=0){
		delay_time=g_delay_time;
	}
	printk("[AURA_STATION] ene_6k582_suspend , delay_time=%d\n",delay_time);
	if(platform_data->current_mode != 0){
		printk("[AURA_STATION] ene_6k582_suspend it is not mode 0\n");
		return 0;
	}
	//set delay time
	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_write_bytes(client, 0X8454, (delay_time&0xff00)>>8); //H
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
	err = ene_6k582_write_bytes(client, 0X8456, (delay_time&0xff));   //L
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
	mutex_unlock(&g_pdata->ene_mutex);

	//stop mode
	val = 0xBE;
	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_write_bytes(client, 0X8453, val);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
	mutex_unlock(&g_pdata->ene_mutex);
	return 0;

}

int ene_6k582_resume(struct device *dev)
{
	int err = 0;
	unsigned char data[2]={0};
	struct i2c_client *client = to_i2c_client(dev);
	struct ene_6k582_platform_data *platform_data = i2c_get_clientdata(client);
	if(g_Charger_mode) {
		printk("[AURA_STATION] In charger mode, stop ene_6k582_resume\n");
		return 0;
	}
	if(g_sleep==1){
		printk("[AURA_STATION] ene_6k582_resume not sleep, return\n");
		return 0;
	}
	printk("[AURA_STATION] ene_6k582_resume\n");
	if(gDongleType!=2){
		printk("[AURA_STATION] ene_6k582_resume return for station removed\n");
		return 0;
	}
	if(platform_data->current_mode){
		printk("[AURA_STATION] ene_6k582_resume it is not mode 0\n");
		return 0;
	}
	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_write_bytes(client, 0XF010, 0x05);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
	msleep(10);
	err = ene_6k582_write_bytes(client, 0XF010, 0x04);
	if (err !=1)
		printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
	mutex_unlock(&g_pdata->ene_mutex);


	mutex_lock(&g_pdata->ene_mutex);
	err = ene_6k582_read_bytes(client, 0x8453, data);
	if (err != 2){
		printk("[AURA_STATION] sleep_show:err %d\n", err);
	}
	mutex_unlock(&g_pdata->ene_mutex);
	if(data[0]==0xAA){
		printk("[AURA_STATION] ene_6k582_resume  ok\n");
	}else{
		mutex_lock(&g_pdata->ene_mutex);
		err = ene_6k582_write_bytes(client, 0XF010, 0x05);
		if (err !=1)
			printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
		msleep(10);
		err = ene_6k582_write_bytes(client, 0XF010, 0x04);
		if (err !=1)
			printk("[AURA_STATION] ene_6k582_write_bytes:err %d\n", err);
		mutex_unlock(&g_pdata->ene_mutex);

	}
	return 0;
}

static const struct i2c_device_id ene_6k582_id[] = {
	{ "ene_6k582_i2c", 0},
	{},
};
//MODULE_DEVICE_TABLE(i2c, ene_8k41_id);

static const struct dev_pm_ops ene_6k582_pm_ops = {
	.suspend	= ene_6k582_suspend,
	.resume	= ene_6k582_resume,
};

#ifdef CONFIG_OF
static const struct of_device_id ene_match_table[] = {
	{ .compatible = "ene6k582_station",},
	{ },
};
#else
#define ene_match_table NULL
#endif

static struct i2c_driver ene_6k582_driver = {
	.driver		= {
		.name		= "ene6k582_station",
		.owner = THIS_MODULE,
		//.pm	= &ene_6k582_pm_ops,
		.of_match_table	= ene_match_table,
	},
	.probe		= ene_6k582_probe,
	.remove		= ene_6k582_remove,
	.id_table 	= ene_6k582_id,
};

static int __init ene_6k582_bus_init(void)
{
	int ret;

	ret = i2c_add_driver(&ene_6k582_driver);
	if (ret)
		printk("[AURA_STATION] ENE 6k852 driver int failed.\n");
	else
		printk("[AURA_STATION] ENE 6k852 driver int success.\n");
	
	return ret;
}
module_init(ene_6k582_bus_init);

static void __exit ene_6k582_bus_exit(void)
{
	i2c_del_driver(&ene_6k582_driver);
}
module_exit(ene_6k582_bus_exit);

MODULE_AUTHOR("ASUS Cassie Huang");
MODULE_DESCRIPTION("Aura sync LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("asus:ene_6k582");
