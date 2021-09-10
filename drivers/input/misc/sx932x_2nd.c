/*! \file sx932x.c
 * \brief  SX932x Driver
 *
 * Driver for the SX932x 
 * Copyright (c) 2011 Semtech Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
//#define DEBUG
#define DRIVER_NAME "sx932x_2nd"

#define MAX_WRITE_ARRAY_SIZE 32

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/syscalls.h>
//#include <linux/wakelock.h>
#include <linux/uaccess.h>
#include <linux/sort.h> 
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>

#include "sx932x.h" 	/* main struct, interrupt,init,pointers */



#define IDLE			0
#define ACTIVE			1

#define SX932x_NIRQ		34

#define MAIN_SENSOR		1 //CS1

/* Failer Index */
#define SX932x_ID_ERROR 	1
#define SX932x_NIRQ_ERROR	2
#define SX932x_CONN_ERROR	3
#define SX932x_I2C_ERROR	4

#define PROXOFFSET_LOW			1500

#define SX932x_ANALOG_GAIN		1
#define SX932x_DIGITAL_GAIN		1
#define SX932x_ANALOG_RANGE		2.65

#define	TOUCH_CHECK_REF_AMB      0 // 44523
#define	TOUCH_CHECK_SLOPE        0 // 50
#define	TOUCH_CHECK_MAIN_AMB     0 // 151282

#if 1
extern int asus_extcon_set_state_sync(struct extcon_dev *edev, int cable_state); //ASUS_BSP

extern bool g_is_country_code_US;
struct pinctrl *sx932x_pinctrl;
struct pinctrl_state *sx932x_pinctrl_state_active;
struct pinctrl_state *sx932x_pinctrl_state_suspend;
//extern int g_ASUS_capID;
#endif

static int g_CAP_STATUS_UEVENT = CAP_STATUS_UEVENT_NONE; //ASUS_BSP register cap_satus uevent
static int g_CAP_STATUS_UEVENT_last = CAP_STATUS_UEVENT_NONE; //ASUS_BSP register cap_satus uevent

/*! \struct sx932x
 * Specialized struct containing input event data, platform data, and
 * last cap state read if needed.
 */
typedef struct sx932x
{
	pbuttonInformation_t pbuttonInformation;
	psx932x_platform_data_t hw;		/* specific platform data settings */
} sx932x_t, *psx932x_t;

static int irq_gpio_num;

/*! \fn static int write_register(psx93XX_t this, u8 address, u8 value)
 * \brief Sends a write register to the device
 * \param this Pointer to main parent struct 
 * \param address 8-bit register address
 * \param value   8-bit register value to write to address
 * \return Value from i2c_master_send
 */
static int write_register(psx93XX_t this, u8 address, u8 value)
{
	struct i2c_client *i2c = 0;
	char buffer[2];
	int returnValue = 0;

	buffer[0] = address;
	buffer[1] = value;
	returnValue = -ENOMEM;
#if IS_ENABLED(CONFIG_CAP_DEBUG)	
        cap2_info("%s",__func__);
#endif	
	if (this && this->bus) {
		i2c = this->bus;
		returnValue = i2c_master_send(i2c,buffer,2);
		//cap2_info("write_register Address: 0x%x Value: 0x%x Return: %d", address,value,returnValue);
	}
	return returnValue;
}

/*! \fn static int read_register(psx93XX_t this, u8 address, u8 *value) 
* \brief Reads a register's value from the device
* \param this Pointer to main parent struct 
* \param address 8-Bit address to read from
* \param value Pointer to 8-bit value to save register value to 
* \return Value from i2c_smbus_read_byte_data if < 0. else 0
*/
static int read_register(psx93XX_t this, u8 address, u8 *value)
{
	struct i2c_client *i2c = 0;
	s32 returnValue = 0;
#if IS_ENABLED(CONFIG_CAP_DEBUG)	
        cap2_info("%s",__func__);
#endif	
	if (this && value && this->bus) {
		i2c = this->bus;
		returnValue = i2c_smbus_read_byte_data(i2c,address);
#if IS_ENABLED(CONFIG_CAP_DEBUG)		
		cap2_info("read_register Address: 0x%x Return: 0x%x", address,returnValue);
#endif	
		if (returnValue >= 0) {
			*value = returnValue;
			return 0;
		} 
		else {
			return returnValue;
		}
	}
	return -ENOMEM;
}

//static int sx932x_set_mode(psx93XX_t this, unsigned char mode);

/*! \fn static int read_regStat(psx93XX_t this)
 * \brief Shortcut to read what caused interrupt.
 * \details This is to keep the drivers a unified
 * function that will read whatever register(s) 
 * provide information on why the interrupt was caused.
 * \param this Pointer to main parent struct 
 * \return If successful, Value of bit(s) that cause interrupt, else 0
 */
static int read_regStat(psx93XX_t this)
{
	u8 data = 0;
#if IS_ENABLED(CONFIG_CAP_DEBUG)	
	cap2_info("%s",__func__);
#endif
	if (this) {
		if (read_register(this,SX932x_IRQSTAT_REG,&data) == 0)
		return (data & 0x00FF);
	}
	return 0;
}

/*********************************************************************/
/*! \brief Perform a manual offset calibration
* \param this Pointer to main parent struct 
* \return Value return value from the write register
 */
static int manual_offset_calibration(psx93XX_t this)
{
	s32 returnValue = 0;
#if IS_ENABLED(CONFIG_CAP_DEBUG)	
	cap2_info("%s",__func__);
#endif	
	returnValue = write_register(this,SX932x_STAT2_REG,0x0F);
	return returnValue;
}
/*! \brief sysfs show function for manual calibration which currently just
 * returns register value.
 */
static ssize_t manual_offset_calibration_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	u8 reg_value = 0;
	psx93XX_t this = dev_get_drvdata(dev);
        cap2_info("%s Reading IRQSTAT_REG",__func__);
	read_register(this,SX932x_IRQSTAT_REG,&reg_value);
	return sprintf(buf, "%d\n", reg_value);
}

/*! \brief sysfs store function for manual calibration
 */
static ssize_t manual_offset_calibration_store(struct device *dev,
			struct device_attribute *attr,const char *buf, size_t count)
{
	psx93XX_t this = dev_get_drvdata(dev);
	unsigned long val;
	cap2_info("%s",__func__);
	if (kstrtoul(buf, 0, &val))
	  return -EINVAL;
	
	if (val) {
		cap2_info("Performing manual_offset_calibration()");
		manual_offset_calibration(this);
	}
	return count;
}

static int sx932x_Hardware_Check(psx93XX_t this)
{
	int ret;
	u8 failcode;
	u8 loop = 0;
	this->failStatusCode = 0;
	cap2_info("%s",__func__);
	//Check th IRQ Status
	while(this->get_nirq_low && this->get_nirq_low()){
		read_regStat(this);
		msleep(100);
		if(++loop >10){
			this->failStatusCode = SX932x_NIRQ_ERROR;
			cap2_info("%s SX932x_NIRQ_ERROR",__func__);
			break;
		}
	}
	
	//Check I2C Connection
	ret = read_register(this, SX932x_WHOAMI_REG, &failcode);
	if(ret < 0){
		this->failStatusCode = SX932x_I2C_ERROR;
		cap2_info("%s SX932x_I2C_ERROR",__func__);
	}
	cap2_info("%s failcode  0x%x",__func__,failcode);	
	if(failcode!= SX932x_WHOAMI_VALUE){
		this->failStatusCode = SX932x_ID_ERROR;
	}
	
	cap2_info("sx932x failcode = 0x%x",this->failStatusCode);
	return (int)this->failStatusCode;
}


/*********************************************************************/
static int sx932x_global_variable_init(psx93XX_t this)
{
	this->irq_disabled = 0;
	this->failStatusCode = 0;
	this->reg_in_dts = true;
	return 0;
}

static ssize_t sx932x_register_write_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	int reg_address = 0, val = 0;
	psx93XX_t this = dev_get_drvdata(dev);
	cap2_info("%s",__func__);
	if (sscanf(buf, "%x,%x", &reg_address, &val) != 2) {
		cap2_err("%s - The number of data are wrong",__func__);
		return -EINVAL;
	}

	write_register(this, (unsigned char)reg_address, (unsigned char)val);
	cap2_info("%s - Register(0x%x) data(0x%x)",__func__, reg_address, val);

	return count;
}
//read registers not include the advanced one
static ssize_t sx932x_register_read_store(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t count)
{
	u8 val=0;
	int regist = 0;
	psx93XX_t this = dev_get_drvdata(dev);
	cap2_info("%s Reading register",__func__);

	if (sscanf(buf, "%x", &regist) != 1) {
		cap2_err("%s - The number of data are wrong",__func__);
		return -EINVAL;
	}

	read_register(this, regist, &val);
	cap2_info("%s - Register(0x%2x) data(0x%2x)",__func__, regist, val);

	return count;
}

static void read_rawData(psx93XX_t this)
{
	u8 msb=0, lsb=0;
	u8 csx;
	s32 useful;
	s32 average;
	s32 diff;
	u16 offset;
	cap2_info("%s",__func__);
	if(this){
		for(csx =0; csx<4; csx++){
			write_register(this,SX932x_CPSRD,csx);//here to check the CS1, also can read other channel		
			read_register(this,SX932x_USEMSB,&msb);
			read_register(this,SX932x_USELSB,&lsb);
			useful = (s32)((msb << 8) | lsb);
			
			read_register(this,SX932x_AVGMSB,&msb);
			read_register(this,SX932x_AVGLSB,&lsb);
			average = (s32)((msb << 8) | lsb);
			
			read_register(this,SX932x_DIFFMSB,&msb);
			read_register(this,SX932x_DIFFLSB,&lsb);
			diff = (s32)((msb << 8) | lsb);
			
			read_register(this,SX932x_OFFSETMSB,&msb);
			read_register(this,SX932x_OFFSETLSB,&lsb);
			offset = (u16)((msb << 8) | lsb);
			if (useful > 32767)
				useful -= 65536;
			if (average > 32767)
				average -= 65536;
			if (diff > 32767)
				diff -= 65536;
			cap2_info("rawData_2nd: [CS: %d] Useful = %d Average = %d, DIFF = %d Offset = %d",csx,useful,average,diff,offset);
		}
	}
}

static ssize_t sx932x_raw_data_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	psx93XX_t this = dev_get_drvdata(dev);
	cap2_info("%s",__func__);
	read_rawData(this);
	return 0;
}

#if 0
static int read_cap_status(psx93XX_t this)
{
    u8 msb = 0, lsb = 0;
    s32 useful_ph0, useful_ph2;

	if(this->failStatusCode != 0 ){
		cap2_info("failcode = 0x%x", this->failStatusCode);
		return -1;
	}

	//get PH0(CS0) useful
	write_register(this, SX932x_CPSRD,0);
	read_register(this, SX932x_USEMSB, &msb);
	read_register(this, SX932x_USELSB, &lsb);
	useful_ph0 = (s32)((msb << 8) | lsb);
	if (useful_ph0 > 32767)
		useful_ph0 -= 65536;

	//get PH2(CS1) useful
	write_register(this, SX932x_CPSRD,2);
	read_register(this, SX932x_USEMSB, &msb);
	read_register(this, SX932x_USELSB, &lsb);
	useful_ph2 = (s32)((msb << 8) | lsb);
	if (useful_ph2 > 32767)
		useful_ph2 -= 65536;


    if (useful_ph0 >= 1800 || useful_ph2 >= 1800) {
        pr_info("[SX932x_2nd] Near\n");
		asus_extcon_set_state_sync(this->cap_satus_extcon, 1);
		return 1;
    } else {
        pr_info("[SX932x_2nd] Far\n");
		asus_extcon_set_state_sync(this->cap_satus_extcon, 0);
        return 0;
    }
}
#endif

static int read_cap_status(psx93XX_t this)
{
	u8 reg_val = 0;
	int ret = read_register(this, SX932x_USELSB, &reg_val);

	if (ret < 0) {
        cap2_err("read cap register fail");
        return -2;
    }
	else if(g_CAP_STATUS_UEVENT == CAP_STATUS_UEVENT_FAR){
		cap2_info("Far");
		asus_extcon_set_state_sync(this->cap_satus_extcon, 0);
		ret = 0;
	}else{
		cap2_info("Near");
		asus_extcon_set_state_sync(this->cap_satus_extcon, 1);
		ret = 1;
	}
	g_CAP_STATUS_UEVENT_last = g_CAP_STATUS_UEVENT;

	return ret;
}

static ssize_t sx932x_cap_status_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
    psx93XX_t this = dev_get_drvdata(dev);

    int ret = read_cap_status(this);
	if(ret == -2 ) {
		return sprintf(buf, "-1\n");
	} else {
		return sprintf(buf, "%d\n", ret);
	}
}

static int read_cap2_status(psx93XX_t this)
{
    u8 reg_val = 0;
    int ret = read_register(this, SX932x_USELSB, &reg_val);
	//cap2_err("st ret = %d ", ret);
    if (ret < 0) {
        cap2_err("read cap2 register fail");
        return -2;
    } else {
        if (reg_val == 0xfe) {
            cap2_info("CS2: near");
			asus_extcon_set_state_sync(this->cap_satus_extcon, 1);
            return 0;
        } else {
            cap2_info("CS2: far");
			asus_extcon_set_state_sync(this->cap_satus_extcon, 0);
            return -1;
        }
    }
    return 0;
}

static ssize_t sx932x_cap2_status_show(struct device *dev,
                                                struct device_attribute *attr, char *buf)
{
    psx93XX_t this = dev_get_drvdata(dev);
    int ret = read_cap2_status(this);
	//cap2_err("ret = %d ", ret);
    if (ret == 0) {
        return sprintf(buf, "1\n");
    }	else if (ret == -2){
		return sprintf(buf, "-1\n");
	} 
	else {
        return sprintf(buf, "0\n");
    }
}

static ssize_t sx932x_register_dump_show(struct device *dev,
                                                struct device_attribute *attr, char *buf)
{
    u8 reg_val = 0;
    int ret = 0, reg = 0x0;

    psx93XX_t this = dev_get_drvdata(dev);

	for(reg=0x0; reg<=0x68; reg++){
		ret = read_register(this, reg, &reg_val);
	    if (ret < 0) {
	        cap2_err("read [0x%02x] fail", reg_val);
	    } else {
	        cap2_info("[0x%02x]=0x%02x",reg, reg_val);
	        sprintf(buf, "[0x%02x]=0x%02x\n",reg, reg_val);
	    }
	}
	return 0;
}

static DEVICE_ATTR(manual_calibrate, 0664, manual_offset_calibration_show,manual_offset_calibration_store);
static DEVICE_ATTR(register_write,  0664, NULL,sx932x_register_write_store);
static DEVICE_ATTR(register_read,0664, NULL,sx932x_register_read_store);
static DEVICE_ATTR(raw_data,0664,sx932x_raw_data_show,NULL);
static DEVICE_ATTR(cap_status,0664,sx932x_cap_status_show,NULL);
static DEVICE_ATTR(cap2_status,0664,sx932x_cap2_status_show,NULL);
static DEVICE_ATTR(register_dump,0664,sx932x_register_dump_show,NULL);
static struct attribute *sx932x_attributes[] = {
	&dev_attr_manual_calibrate.attr,
	&dev_attr_register_write.attr,
	&dev_attr_register_read.attr,
	&dev_attr_raw_data.attr,
	&dev_attr_cap_status.attr,
	&dev_attr_cap2_status.attr,
	&dev_attr_register_dump.attr,
	NULL,
};
static struct attribute_group sx932x_attr_group = {
	.attrs = sx932x_attributes,
};

/****************************************************/
/*! \brief  Initialize I2C config from platform data
 * \param this Pointer to main parent struct 
 */
static void sx932x_reg_init(psx93XX_t this)
{
	psx932x_t pDevice = 0;
	psx932x_platform_data_t pdata = 0;
	int i = 0;
	/* configure device */ 
	
	cap2_info("Going to Setup I2C Registers");
	if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw))
	{
		/*******************************************************************************/
		// try to initialize from device tree!
		/*******************************************************************************/
		if (this->reg_in_dts == true) {
			while ( i < pdata->i2c_reg_num) {
				/* Write all registers/values contained in i2c_reg */
				//dev_info(this->pdev, "Going to Write Reg from dts: 0x%x Value: 0x%x\n",
				//pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
				write_register(this, pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
				i++;
			}
		} else { // use static ones!!
			while ( i < ARRAY_SIZE(sx932x_i2c_reg_setup)) {
				/* Write all registers/values contained in i2c_reg */
				//dev_info(this->pdev, "Going to Write Reg: 0x%x Value: 0x%x\n",
				//sx932x_i2c_reg_setup[i].reg,sx932x_i2c_reg_setup[i].val);
				write_register(this, sx932x_i2c_reg_setup[i].reg,sx932x_i2c_reg_setup[i].val);
				i++;
			}
		}
	/*******************************************************************************/
	} else {
		cap2_err("ERROR! platform data 0x%p",pDevice->hw);
	}
	cap2_info("End of Setup I2C Registers");
}


/*! \fn static int initialize(psx93XX_t this)
 * \brief Performs all initialization needed to configure the device
 * \param this Pointer to main parent struct 
 * \return Last used command's return value (negative if error)
 */
static int initialize(psx93XX_t this)
{
	int ret;
	int err = 0;
	if (this) {
		cap2_info("SX932x income initialize");
		/* prepare reset by disabling any irq handling */
		this->irq_disabled = 1;
		disable_irq(this->irq);
		/* perform a reset */
		write_register(this,SX932x_SOFTRESET_REG,SX932x_SOFTRESET);
		/* wait until the reset has finished by monitoring NIRQ */
		cap2_info("Sent Software Reset. Waiting until device is back from reset to continue.");
		/* just sleep for awhile instead of using a loop with reading irq status */
		msleep(100);
		
		ret = sx932x_global_variable_init(this);
		
		sx932x_reg_init(this);
		msleep(100); /* make sure everything is running */
		manual_offset_calibration(this);

		/* re-enable interrupt handling */
		enable_irq(this->irq);
		err = pinctrl_select_state(sx932x_pinctrl, sx932x_pinctrl_state_active);
		if(err < 0)
		cap2_err("%s: failed to set sx932x pinctrl active state", __func__);
		/* make sure no interrupts are pending since enabling irq will only
		* work on next falling edge */
		read_regStat(this);
		return 0;
	}
	return -ENOMEM;
}

/*! 
 * \brief Handle what to do when a touch occurs
 * \param this Pointer to main parent struct 
 */
#if 0
static void touchProcess(psx93XX_t this)
{
	int counter = 0;
	u8 i = 0;
	int numberOfButtons = 0;
	psx932x_t pDevice = NULL;
	struct _buttonInfo *buttons = NULL;
	struct input_dev *input = NULL;

	struct _buttonInfo *pCurrentButton  = NULL;

	if (this && (pDevice = this->pDevice))
	{
		//dev_info(this->pdev, "Inside touchProcess()\n");
		read_register(this, SX932x_STAT0_REG, &i);

		buttons = pDevice->pbuttonInformation->buttons;
		input = pDevice->pbuttonInformation->input;
		numberOfButtons = pDevice->pbuttonInformation->buttonSize;

		if (unlikely( (buttons==NULL) || (input==NULL) )) {
			dev_err(this->pdev, "ERROR!! buttons or input NULL!!!\n");
			return;
		}

		for (counter = 0; counter < numberOfButtons; counter++) {
			pCurrentButton = &buttons[counter];
			if (pCurrentButton==NULL) {
				dev_err(this->pdev,"ERROR!! current button at index: %d NULL!!!\n", counter);
				return; // ERRORR!!!!
			}
			switch (pCurrentButton->state) {
				case IDLE: /* Button is not being touched! */
					if (((i & pCurrentButton->mask) == pCurrentButton->mask)) {
						/* User pressed button */
						dev_info(this->pdev, "cap button %d touched\n", counter);
						input_report_key(input, pCurrentButton->keycode, 1);
						pCurrentButton->state = ACTIVE;
					} else {
						dev_info(this->pdev, "Button %d already released.\n",counter);
					}
					break;
				case ACTIVE: /* Button is being touched! */ 
					if (((i & pCurrentButton->mask) != pCurrentButton->mask)) {
						/* User released button */
						dev_info(this->pdev, "cap button %d released\n",counter);
						input_report_key(input, pCurrentButton->keycode, 0);
						pCurrentButton->state = IDLE;
					} else {
						dev_info(this->pdev, "Button %d still touched.\n",counter);
					}
					break;
				default: /* Shouldn't be here, device only allowed ACTIVE or IDLE */
					break;
			};
		}
		input_sync(input);

		read_cap_status(this);//ASUS_BSP update cap_status uevent for RIL

   //dev_info(this->pdev, "Leaving touchProcess()\n");
  }
}
#endif

//ASUS_BSP +++ register cap_satus uevent
static void cap_status_uevent_far(psx93XX_t this)
{
	g_CAP_STATUS_UEVENT = CAP_STATUS_UEVENT_FAR;
	cap2_info("%s start",__func__);	
	if(g_CAP_STATUS_UEVENT_last != g_CAP_STATUS_UEVENT)
		read_cap_status(this);
}

static void cap_status_uevent_near(psx93XX_t this)
{
	g_CAP_STATUS_UEVENT = CAP_STATUS_UEVENT_NEAR;
	cap2_info("%s start",__func__);
	if(g_CAP_STATUS_UEVENT_last != g_CAP_STATUS_UEVENT)
		read_cap_status(this);
}
//ASUS_BSP --- register cap_satus uevent

static int sx932x_parse_dt(struct sx932x_platform_data *pdata, struct device *dev)
{
	struct device_node *dNode = dev->of_node;
	enum of_gpio_flags flags;
	int ret=0;
	if (dNode == NULL)
		return -ENODEV;
	cap2_info("%s",__func__);
	pdata->irq_gpio= of_get_named_gpio_flags(dNode,"Semtech,nirq-gpio", 0, &flags);
	irq_gpio_num = pdata->irq_gpio;
	if (pdata->irq_gpio < 0) {
		cap2_err("%s - get irq_gpio error", __func__);
		return -ENODEV;
	}
	/***********************************************************************/
	// load in registers from device tree
	of_property_read_u32(dNode,"Semtech,reg-num",&pdata->i2c_reg_num);
	// layout is register, value, register, value....
	// if an extra item is after just ignore it. reading the array in will cause it to fail anyway
	cap2_info("%s -  size of elements %d ", __func__,pdata->i2c_reg_num);
	if (pdata->i2c_reg_num > 0) {
		 // initialize platform reg data array
		 pdata->pi2c_reg = devm_kzalloc(dev,sizeof(struct smtc_reg_data)*pdata->i2c_reg_num, GFP_KERNEL);
		 if (unlikely(pdata->pi2c_reg == NULL)) {
			return -ENOMEM;
		}

	 // initialize the array
		if (of_property_read_u8_array(dNode,"Semtech,reg-init",(u8*)&(pdata->pi2c_reg[0]),sizeof(struct smtc_reg_data)*pdata->i2c_reg_num))
		return -ENOMEM;
	}
	/***********************************************************************/
	cap2_info("%s -[%d] parse_dt complete", __func__,pdata->irq_gpio);
	return ret;
}

/* get the NIRQ state (1->NIRQ-low, 0->NIRQ-high) */
static int sx932x_init_platform_hw(struct i2c_client *client)
{
	psx93XX_t this = i2c_get_clientdata(client);
	struct sx932x *pDevice = NULL;
	struct sx932x_platform_data *pdata = NULL;

	int rc = 0;
        cap2_info("%s start ! ",__func__);

	if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw)) {
	        cap2_info("%s request irq",__func__);
		if (gpio_is_valid(pdata->irq_gpio)) {
			rc = devm_gpio_request(this->pdev, pdata->irq_gpio, "sx932x_irq_gpio");
			if (rc < 0) {
				cap2_err("%s SX932x Request gpio. Fail![%d]",__func__, rc);
				return rc;
			}
			rc = gpio_direction_input(pdata->irq_gpio);
			if (rc < 0) {
				cap2_err("%s SX932x Set gpio direction. Fail![%d]",__func__, rc);
				return rc;
			}
			this->irq = client->irq = gpio_to_irq(pdata->irq_gpio);
			cap2_info("%s : IRQ number = %d", __func__, this->irq);
		}
		else {
			cap2_err("%s SX932x Invalid irq gpio num.(init)",__func__);
		}
	}
	else {
		cap2_err("%s - Do not init platform HW", __func__);
	}
	
	cap2_info("%s - sx932x_irq_debug",__func__);
	return rc;
}

static void sx932x_exit_platform_hw(struct i2c_client *client)
{
	psx93XX_t this = i2c_get_clientdata(client);
	struct sx932x *pDevice = NULL;
	struct sx932x_platform_data *pdata = NULL;
        cap2_info("%s",__func__);
	if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw)) {
		if (gpio_is_valid(pdata->irq_gpio)) {
			gpio_free(pdata->irq_gpio);
		}
		else {
			cap2_err("Invalid irq gpio num.(exit)");
		}
	}
	return;
}

static int sx932x_get_nirq_state(void)
{
	int value;
	value = gpio_get_value(irq_gpio_num);
#if IS_ENABLED(CONFIG_CAP_DEBUG)	
	cap2_info("%s (gpio%d)= %d", __func__, irq_gpio_num,value);
#endif	
	return  !value;
}

//ASUS_BSP LiJen +++ add initial work for resume
static void sx93XX_init_worker_func(struct work_struct *work)
{
	psx93XX_t this = 0;
	int err = 0;
    cap2_info("%s",__func__);
	if (work) {
		this = container_of(work,sx93XX_t,initworker.work);

		if (!this) {
			cap2_err("sx93XX_init_worker_func, NULL sx93XX_t");
			return;
		}
		if (this->init){
			err = pinctrl_select_state(sx932x_pinctrl, sx932x_pinctrl_state_active);
			if(err < 0)
			cap2_err("%s: failed to set sx932x pinctrl active state", __func__);
			this->init(this);
		}
	} else {
		cap2_err("sx93XX_init_worker_func, NULL work_struct");
	}
}
//ASUS_BSP LiJen --- add initial work for resume

//ASUS_BSP LiJen +++ add hwcheck work for probe
static void sx93XX_hwcheck_worker_func(struct work_struct *work)
{
	psx93XX_t this = 0;

	if (work) {
		this = container_of(work,sx93XX_t,hwcheckworker.work);

		if (!this) {
			cap2_err("sx93XX_hwcheck_worker_func, NULL sx93XX_t");
			return;
		}
		sx932x_Hardware_Check(this);
		
	} else {
		cap2_err("sx93XX_hwcheck_worker_func, NULL work_struct");
	}
}
//ASUS_BSP LiJen --- add hwcheck work for probe

/*! \fn static int sx932x_probe(struct i2c_client *client, const struct i2c_device_id *id)
 * \brief Probe function
 * \param client pointer to i2c_client
 * \param id pointer to i2c_device_id
 * \return Whether probe was successful
 */
static int sx932x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{    
	int i = 0;
	int err = 0;

	psx93XX_t this = 0;
	psx932x_t pDevice = 0;
	psx932x_platform_data_t pplatData = 0;
	struct totalButtonInformation *pButtonInformationData = NULL;
	struct input_dev *input = NULL;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	cap2_info("%s start",__func__);

	// disable cap2 driver
	cap2_info("Disable sx932x_2nd driver");
	return 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_WORD_DATA)) {
		cap2_info("Check i2c functionality.Fail!");
		err = -EIO;
		return err;
	}

	this = devm_kzalloc(&client->dev,sizeof(sx93XX_t), GFP_KERNEL); /* create memory for main struct */
	cap2_info("Initialized Main Memory: 0x%p",this);

	pButtonInformationData = devm_kzalloc(&client->dev , sizeof(struct totalButtonInformation), GFP_KERNEL);
	if (!pButtonInformationData) {
		cap2_err("Failed to allocate memory(totalButtonInformation)");
		err = -ENOMEM;
		return err;
	}

	pButtonInformationData->buttonSize = ARRAY_SIZE(psmtcButtons);
	pButtonInformationData->buttons =  psmtcButtons;
	pplatData = devm_kzalloc(&client->dev,sizeof(struct sx932x_platform_data), GFP_KERNEL);
	if (!pplatData) {
		cap2_info("platform data is required!");
		return -EINVAL;
	}
	pplatData->get_is_nirq_low = sx932x_get_nirq_state;
	pplatData->pbuttonInformation = pButtonInformationData;
 
	client->dev.platform_data = pplatData; 
	err = sx932x_parse_dt(pplatData, &client->dev);
	if (err) {
		cap2_err("could not setup pin");
		return ENODEV;
	}

	pplatData->init_platform_hw = sx932x_init_platform_hw;
	cap2_info("SX932x init_platform_hw done!");

	if (this){
		cap2_info("SX932x initialize start!!");
		/* In case we need to reinitialize data 
		* (e.q. if suspend reset device) */
		this->init = initialize;
		/* shortcut to read status of interrupt */
		this->refreshStatus = read_regStat;
		/* pointer to function from platform data to get pendown 
		* (1->NIRQ=0, 0->NIRQ=1) */
		this->get_nirq_low = pplatData->get_is_nirq_low;
		/* save irq in case we need to reference it */
		this->irq = client->irq;
		/* do we need to create an irq timer after interrupt ? */
		this->useIrqTimer = 0;

		/* Setup function to call on corresponding reg irq source bit */
		if (MAX_NUM_STATUS_BITS>= 8)
		{
			this->statusFunc[0] = 0; /* TXEN_STAT */
			this->statusFunc[1] = 0; /* UNUSED */
			this->statusFunc[2] = 0; /* UNUSED */
			this->statusFunc[3] = 0;//read_rawData; /* CONV_STAT */
			this->statusFunc[4] = 0; /* COMP_STAT */
			this->statusFunc[5] = cap_status_uevent_far;//touchProcess; /* RELEASE_STAT */
			this->statusFunc[6] = cap_status_uevent_near;//touchProcess; /* TOUCH_STAT  */
			this->statusFunc[7] = 0; /* RESET_STAT */
		}

		/* setup i2c communication */
		this->bus = client;
		i2c_set_clientdata(client, this);

		/* record device struct */
		this->pdev = &client->dev;

		/* create memory for device specific struct */
		this->pDevice = pDevice = devm_kzalloc(&client->dev,sizeof(sx932x_t), GFP_KERNEL);
		cap2_info("Initialized Device Specific Memory: 0x%p",pDevice);

		if (pDevice){
			/* for accessing items in user data (e.g. calibrate) */
			err = sysfs_create_group(&client->dev.kobj, &sx932x_attr_group);
			//sysfs_create_group(client, &sx932x_attr_group);

			/* Add Pointer to main platform data struct */
			pDevice->hw = pplatData;

			/* Check if we hava a platform initialization function to call*/
			if (pplatData->init_platform_hw)
			pplatData->init_platform_hw(client);

			/* Initialize the button information initialized with keycodes */
			pDevice->pbuttonInformation = pplatData->pbuttonInformation;
			/* Create the input device */
			input = input_allocate_device();
			if (!input) {
				return -ENOMEM;
			}
			/* Set all the keycodes */
			__set_bit(EV_KEY, input->evbit);
			#if 1
			for (i = 0; i < pButtonInformationData->buttonSize; i++) {
				__set_bit(pButtonInformationData->buttons[i].keycode,input->keybit);
				pButtonInformationData->buttons[i].state = IDLE;
			}
			#endif
			/* save the input pointer and finish initialization */
			pButtonInformationData->input = input;
			input->name = "SX932x Cap Touch 2";
			input->id.bustype = BUS_I2C;
			if(input_register_device(input)){
				return -ENOMEM;
			}
		}

		sx93XX_2nd_IRQ_init(this);
		/* call init function pointer (this should initialize all registers */
		INIT_DELAYED_WORK(&this->initworker, sx93XX_init_worker_func); //ASUS_BSP LiJen add initial work for resume
		if (this->init){
			schedule_delayed_work(&this->initworker,0);
		}else{
			cap2_err("No init function!!!!");
			return -ENOMEM;
		}
	}else{
		return -1;
	}

	INIT_DELAYED_WORK(&this->hwcheckworker, sx93XX_hwcheck_worker_func); //ASUS_BSP LiJen add hwcheck work for probe
	schedule_delayed_work(&this->hwcheckworker,1);
	pplatData->exit_platform_hw = sx932x_exit_platform_hw;

	//ASUS_BSP +++ register cap_satus uevent
#if 1
	this->cap_satus_extcon = extcon_dev_allocate(asus_cap_extcon_cable);
	if (IS_ERR(this->cap_satus_extcon)) {
		err = PTR_ERR(this->cap_satus_extcon);
		cap2_err("failed to allocate ASUS cap_satus_extcon device rc=%d", err);
	}
	#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
	this->cap_satus_extcon->fnode_name = "cap2_status";
	#endif
	err = extcon_dev_register(this->cap_satus_extcon);
	if (err < 0) {
		cap2_err("failed to register ASUS cap_satus_extcon device rc=%d", err);
	}
#endif	
	//ASUS_BSP --- register cap_satus uevent
	
	cap2_info("sx932x_probe() Done");

	return 0;
}

/*! \fn static int sx932x_remove(struct i2c_client *client)
 * \brief Called when device is to be removed
 * \param client Pointer to i2c_client struct
 * \return Value from sx93XX_remove()
 */
//static int __devexit sx932x_remove(struct i2c_client *client)
static int sx932x_remove(struct i2c_client *client)
{
	psx932x_platform_data_t pplatData =0;
	psx932x_t pDevice = 0;
	psx93XX_t this = i2c_get_clientdata(client);
	cap2_info("%s start",__func__);
	if (this && (pDevice = this->pDevice))
	{
		input_unregister_device(pDevice->pbuttonInformation->input);

		sysfs_remove_group(&client->dev.kobj, &sx932x_attr_group);
		pplatData = client->dev.platform_data;
		if (pplatData && pplatData->exit_platform_hw)
			pplatData->exit_platform_hw(client);
		kfree(this->pDevice);
	}
	cap2_info("%s done",__func__);
	return sx93XX_2nd_remove(this);
}
#if 0//def CONFIG_PM
/*====================================================*/
/***** Kernel Suspend *****/
static int sx932x_suspend(struct device *dev)
{
	psx93XX_t this = dev_get_drvdata(dev);
	cap2_info("%s",__func__);
	sx93XX_2nd_suspend(this);
	return 0;
}
/***** Kernel Resume *****/
static int sx932x_resume(struct device *dev)
{
	psx93XX_t this = dev_get_drvdata(dev);
	cap2_info("%s",__func__);
	sx93XX_2nd_resume(this);
	return 0;
}
/*====================================================*/
#else
#define sx932x_suspend		NULL
#define sx932x_resume		NULL
#endif /* CONFIG_PM */

static struct i2c_device_id sx932x_idtable[] = {
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sx932x_idtable);
#ifdef CONFIG_OF
static struct of_device_id sx932x_match_table[] = {
	{ .compatible = "Semtech,sx932x_2nd",},
	{ },
};
#else
#define sx932x_match_table NULL
#endif
static const struct dev_pm_ops sx932x_pm_ops = {
	.suspend = sx932x_suspend,
	.resume = sx932x_resume,
};
static struct i2c_driver sx932x_driver = {
	.driver = {
		.owner			= THIS_MODULE,
		.name			= DRIVER_NAME,
		.of_match_table	= sx932x_match_table,
		.pm			= &sx932x_pm_ops,
	},
	.id_table		= sx932x_idtable,
	.probe			= sx932x_probe,
	.remove			= sx932x_remove,
};
static int __init sx932x_I2C_init(void)
{
  	cap2_info("%s",__func__);
	return i2c_add_driver(&sx932x_driver);
}
static void __exit sx932x_I2C_exit(void)
{
  	cap2_info("%s",__func__);
	i2c_del_driver(&sx932x_driver);
}

module_init(sx932x_I2C_init);
module_exit(sx932x_I2C_exit);

MODULE_AUTHOR("Semtech Corp. (http://www.semtech.com/)");
MODULE_DESCRIPTION("SX932x Capacitive Touch Controller Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

static void sx93XX_schedule_work(psx93XX_t this, unsigned long delay)
{
	unsigned long flags;
#if IS_ENABLED(CONFIG_CAP_DEBUG)	
	cap2_info("%s",__func__);
#endif
	if (this) {
		//dev_info(this->pdev, "sx93XX_schedule_work()\n");
		spin_lock_irqsave(&this->lock,flags);
		/* Stop any pending penup queues */
		cancel_delayed_work(&this->dworker);
		//after waiting for a delay, this put the job in the kernel-global workqueue. so no need to create new thread in work queue.
		schedule_delayed_work(&this->dworker,delay);
		spin_unlock_irqrestore(&this->lock,flags);
	}
	else
		cap2_err("sx93XX_schedule_work, NULL psx93XX_t");
} 

static irqreturn_t sx93XX_irq(int irq, void *pvoid)
{
	psx93XX_t this = 0;
#if IS_ENABLED(CONFIG_CAP_DEBUG)	
	cap2_info("%s",__func__);
#endif
	if (pvoid) {
		this = (psx93XX_t)pvoid;
		if ((!this->get_nirq_low) || this->get_nirq_low()) {
		sx93XX_schedule_work(this,0);
		}
		else{
			cap2_err("sx93XX_irq - nirq read high");
		}
	}
	else{
		cap2_err("sx93XX_irq, NULL pvoid");
	}
	return IRQ_HANDLED;
}

static void sx93XX_worker_func(struct work_struct *work)
{
	psx93XX_t this = 0;
	int status = 0;
	int counter = 0;
	u8 nirqLow = 0;
#if IS_ENABLED(CONFIG_CAP_DEBUG)	
	cap2_info("%s",__func__);
#endif
	if (work) {
		this = container_of(work,sx93XX_t,dworker.work);

		if (!this) {
			cap2_err("sx93XX_worker_func, NULL sx93XX_t");
			return;
		}
		if (unlikely(this->useIrqTimer)) {
			if ((!this->get_nirq_low) || this->get_nirq_low()) {
				nirqLow = 1;
			}
		}
		/* since we are not in an interrupt don't need to disable irq. */
		status = this->refreshStatus(this);
		counter = -1;
#if IS_ENABLED(CONFIG_CAP_DEBUG)		
		cap2_info("Worker - Refresh Status %d",status);
#endif		
		while((++counter) < MAX_NUM_STATUS_BITS) { /* counter start from MSB */
			if (((status>>counter) & 0x01) && (this->statusFunc[counter])) {
				//dev_info(this->pdev, "SX932x Function Pointer Found. Calling\n");
				this->statusFunc[counter](this);
			}
		}
		if (unlikely(this->useIrqTimer && nirqLow))
		{	/* Early models and if RATE=0 for newer models require a penup timer */
			/* Queue up the function again for checking on penup */
			sx93XX_schedule_work(this,msecs_to_jiffies(this->irqTimeout));
		}
	} else {
		cap2_err("sx93XX_worker_func, NULL work_struct");
	}
}

int sx93XX_2nd_remove(psx93XX_t this)
{
  	cap2_info("%s",__func__);

	if (this) {
		cancel_delayed_work_sync(&this->dworker); /* Cancel the Worker Func */
		/*destroy_workqueue(this->workq); */
		free_irq(this->irq, this);
		kfree(this);
		return 0;
	}
	return -ENOMEM;
}
void sx93XX_2nd_suspend(psx93XX_t this)
{
	int err = 0;
    cap2_info("%s",__func__);

    if (this){
		disable_irq(this->irq);
		err = pinctrl_select_state(sx932x_pinctrl, sx932x_pinctrl_state_suspend);
		if(err < 0)
			cap2_err("%s: failed to set sx932x pinctrl state", __func__);
	}

	//write_register(this,SX932x_CTRL1_REG,0x20);//make sx932x in Sleep mode
}
void sx93XX_2nd_resume(psx93XX_t this)
{
  	cap2_info("%s",__func__);

	if (this) {
	    sx93XX_schedule_work(this,500);
	    schedule_delayed_work(&this->initworker,500); //ASUS_BSP LiJen add initial work for resume
	    enable_irq(this->irq);
	}
}

int sx93XX_2nd_IRQ_init(psx93XX_t this)
{
	int err = 0;

	cap2_info("%s",__func__);

	if (this && this->pDevice)
	{
		/* initialize spin lock */
		spin_lock_init(&this->lock);
		/* initialize worker function */
		INIT_DELAYED_WORK(&this->dworker, sx93XX_worker_func);
		/* initailize interrupt reporting */
		this->irq_disabled = 0;
		err = request_irq(this->irq, sx93XX_irq, IRQF_TRIGGER_FALLING,
							this->pdev->driver->name, this);
		if (err) {
			cap2_err("irq %d busy?", this->irq);
			return err;
		}
		cap2_info("registered with irq (%d)", this->irq);

		sx932x_pinctrl = devm_pinctrl_get(this->pdev);
		if (IS_ERR(sx932x_pinctrl)){
			cap2_err("%s: failed at sx932x devm_pinctrl_get", __func__);
			return PTR_ERR(sx932x_pinctrl);
		}
		sx932x_pinctrl_state_active = pinctrl_lookup_state(sx932x_pinctrl, "sx932x_2nd_active");
		if (IS_ERR(sx932x_pinctrl_state_active)){
			cap2_err("%s: failed at sx932x pinctrl_lookup_state", __func__);
			return PTR_ERR(sx932x_pinctrl_state_active);
		}
		sx932x_pinctrl_state_suspend = pinctrl_lookup_state(sx932x_pinctrl, "sx932x_2nd_suspend");
		if (IS_ERR(sx932x_pinctrl_state_suspend)){
			cap2_err("%s: failed at sx932x pinctrl_lookup_state", __func__);
			return PTR_ERR(sx932x_pinctrl_state_suspend);
		}
		err = pinctrl_select_state(sx932x_pinctrl, sx932x_pinctrl_state_active);
		if(err < 0)
			cap2_err("%s: failed to set sx932x pinctrl state", __func__);
	
		enable_irq_wake(this->irq);

	}
	return -ENOMEM;
}
