/*
** =============================================================================
**
** File: ImmVibeSPI.c
**
** Description:
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
**
** Copyright (c) 2008-2017 Immersion Corporation. All Rights Reserved.
**
** This file contains Original Code and/or Modifications of Original Code
** as defined in and that are subject to the GNU Public License v2 -
** (the 'License'). You may not use this file except in compliance with the
** License. You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES,
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see
** the License for the specific language governing rights and limitations
** under the License.
**
** =============================================================================
*/


#include <linux/gpio.h>
#include <dt-bindings/pinctrl/qcom,pmic-gpio.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/pm_qos.h>

#include "aw8697_drv.h"

#include "aw_savefile.c"

#ifdef IMMVIBESPIAPI
#undef IMMVIBESPIAPI
#endif
#define IMMVIBESPIAPI static

/*
** This SPI supports 2 actuators.
** Don't forget to update the code (switch statements) in this file if you change NUM_ACTUATORS.
*/
//#error Please Set NUM_ACTUATORS to the number of actuators supported by this SPI.
#define NUM_ACTUATORS       1

//#define RTP_MONITOR_ENABLE 1

/*
** Include optional ImmVibeSPI_Device_GetStatus in driver.
*/
#define IMMVIBESPI_DEVICE_GETSTATUS_SUPPORT

/*
** Name displayed in TouchSense API and tools
*/
#define DEVICE_NAME "ROG5"

//I2C DMA buffer
static u8 *dma_buf_resampling=NULL;
static u8 *dma_buf_ivt=NULL;

/*resample control
 * 1: don't resample
 * 2: 12KHz to 24KHz
 * 4: 12KHz to 48KHz
 * 6: 8KHz to 48KHz
 */
static int gResample=2;// ivt 12Khz to 24Khz
//end

//pwm control
//enum aw8697_haptic_pwm_mode {
//	AW8697_PWM_48K = 0,
//	AW8697_PWM_24K = 1,
//	AW8697_PWM_12K = 2,
//};
//static int gPWM = AW8697_PWM_48K;
static int gAW_RTP_PWM = AW8697_PWM_24K;
static int gROG2_RTP_PWM = AW8697_PWM_48K;
static int gRTP_PWM = 0xf ;
//Vibrator debug interface
/*gVibDebugLog: 
 * 0x0000: none
 * 0x0001: awinic log
 * 0x0002: immersion log
 * 0x0004: immersion data
 * 0x0008: 
 * 0x0010: 
 * 0x0020: 
 * 0x0100: 
 */
int gVibDebugLog=0x0000;
// /sys/module/tspdrv/parameters/
module_param(gVibDebugLog, int , 0644);
MODULE_PARM_DESC(gVibDebugLog, "vib kerenl debug");

IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer);




/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW8697_I2C_NAME "aw8697_haptic"
#define AW8697_HAPTIC_NAME "awinic_haptic"

#define AW8697_VERSION "v1.4.3"

#define AWINIC_RAM_UPDATE_DELAY

#define AW_I2C_RETRIES 2
#define AW_I2C_RETRY_DELAY 2
#define AW_READ_CHIPID_RETRIES 2
#define AW_READ_CHIPID_RETRY_DELAY 2
#define AW8697_MAX_DSP_START_TRY_COUNT    10
#define AWINIC_READ_BIN_FLEXBALLY

#define AW8697_MAX_FIRMWARE_LOAD_CNT 20
#define OSC_CALIBRATION_T_LENGTH 5100000
#define PM_QOS_VALUE_VB 400
struct pm_qos_request pm_qos_req_vb;
/******************************************************
 *
 * variable
 *
 ******************************************************/
#define AW8697_RTP_NAME_MAX        64
static char *aw8697_ram_name = "aw8697_haptic.bin";
static char aw8697_rtp_name[][AW8697_RTP_NAME_MAX] = {
	{"aw8697_osc_rtp_24K_5s.bin"},
	{"aw8697_rtp.bin"},
	//{"aw8697_rtp_lighthouse.bin"},
	//{"aw8697_rtp_silk.bin"},
};
static char fwName[16]={0};
static char rtp_type[16]={0};
static int rtp_repeat=0;

struct aw8697_container *aw8697_rtp;
struct aw8697 *g_aw8697 = NULL;

/******************************************************
 *
 * functions
 *
 ******************************************************/
static void aw8697_interrupt_clear(struct aw8697 *aw8697);
static int aw8697_haptic_trig_enable_config(struct aw8697 *aw8697);

 /******************************************************
 *
 * aw8697 i2c write/read
 *
 ******************************************************/
static int aw8697_i2c_write(struct aw8697 *aw8697,
			    unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret =
		    i2c_smbus_write_byte_data(aw8697->i2c, reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			break;
		}
		cnt++;
		usleep_range(AW_I2C_RETRY_DELAY * 1000,
			     AW_I2C_RETRY_DELAY * 1000 + 500);
	}

	return ret;
}

static int aw8697_i2c_read(struct aw8697 *aw8697,
			   unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw8697->i2c, reg_addr);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		usleep_range(AW_I2C_RETRY_DELAY * 1000,
			     AW_I2C_RETRY_DELAY * 1000 + 500);
	}

	return ret;
}

static int aw8697_i2c_write_bits(struct aw8697 *aw8697,
				 unsigned char reg_addr, unsigned int mask,
				 unsigned char reg_data)
{
	unsigned char reg_val = 0;

	aw8697_i2c_read(aw8697, reg_addr, &reg_val);
	reg_val &= mask;
	reg_val |= reg_data;
	aw8697_i2c_write(aw8697, reg_addr, reg_val);

	return 0;
}

static int aw8697_i2c_writes(struct aw8697 *aw8697,
			     unsigned char reg_addr, unsigned char *buf,
			     unsigned int len)
{
	int ret = -1;
	unsigned char *data;

	data = kmalloc(len + 1, GFP_KERNEL);
	if (data == NULL) {
		pr_err("%s: can not allocate memory\n", __func__);
		return -ENOMEM;
	}

	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	ret = i2c_master_send(aw8697->i2c, data, len + 1);
	if (ret < 0)
		pr_err("%s: i2c master send error\n", __func__);

	kfree(data);

	return ret;
}

/*****************************************************
 *
 * ram update
 *
 *****************************************************/
static void aw8697_rtp_loaded(const struct firmware *cont, void *context)
{
	struct aw8697 *aw8697 = context;

	pr_info("%s enter\n", __func__);

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__,
		       aw8697_rtp_name[aw8697->rtp_file_num]);
		release_firmware(cont);
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__,
		aw8697_rtp_name[aw8697->rtp_file_num], cont ? cont->size : 0);

	/* aw8697 rtp update */
	mutex_lock(&aw8697->rtp_lock);
	aw8697_rtp = vmalloc(cont->size + sizeof(int));
	if (!aw8697_rtp) {
		release_firmware(cont);
		mutex_unlock(&aw8697->rtp_lock);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw8697_rtp->len = cont->size;
	pr_info("%s: rtp size = %d\n", __func__, aw8697_rtp->len);
	memcpy(aw8697_rtp->data, cont->data, cont->size);
	release_firmware(cont);
	mutex_unlock(&aw8697->rtp_lock);

	aw8697->rtp_init = 1;
	pr_info("%s: rtp update complete\n", __func__);
}

static int aw8697_rtp_update(struct aw8697 *aw8697)
{
	pr_info("%s enter\n", __func__);

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				       aw8697_rtp_name[aw8697->rtp_file_num],
				       aw8697->dev, GFP_KERNEL, aw8697,
				       aw8697_rtp_loaded);
}

static void aw8697_container_update(struct aw8697 *aw8697,
				    struct aw8697_container *aw8697_cont)
{
	int i = 0;
	unsigned int shift = 0;

	pr_info("%s enter\n", __func__);

	mutex_lock(&aw8697->lock);

	aw8697->ram.baseaddr_shift = 2;
	aw8697->ram.ram_shift = 4;

	/* RAMINIT Enable */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8697_BIT_SYSCTRL_RAMINIT_EN);

	/* base addr */
	shift = aw8697->ram.baseaddr_shift;
	aw8697->ram.base_addr =
	    (unsigned int)((aw8697_cont->data[0 + shift] << 8) |
			   (aw8697_cont->data[1 + shift]));
	pr_info("%s: base_addr=0x%4x\n", __func__, aw8697->ram.base_addr);

	aw8697_i2c_write(aw8697, AW8697_REG_BASE_ADDRH,
			 aw8697_cont->data[0 + shift]);
	aw8697_i2c_write(aw8697, AW8697_REG_BASE_ADDRL,
			 aw8697_cont->data[1 + shift]);
	/*1/2 FIFO */
	aw8697_i2c_write(aw8697, AW8697_REG_FIFO_AEH,
			(unsigned char)((aw8697->ram.base_addr >> 1) >> 8));
	/*1/2 FIFO */
	aw8697_i2c_write(aw8697, AW8697_REG_FIFO_AEL,
			(unsigned char)((aw8697->ram.base_addr >> 1) & 0x00FF));

	/*3/4 FIFO */
	aw8697_i2c_write(aw8697, AW8697_REG_FIFO_AFH,
			 (unsigned
			  char)((aw8697->ram.base_addr -
				 (aw8697->ram.base_addr >> 2)) >> 8));
	aw8697_i2c_write(aw8697, AW8697_REG_FIFO_AFL,
			 (unsigned
			  char)((aw8697->ram.base_addr -
				 (aw8697->ram.base_addr >> 2)) & 0x00FF));

	/* ram */
	shift = aw8697->ram.baseaddr_shift;
	aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRH,
			 aw8697_cont->data[0 + shift]);
	aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRL,
			 aw8697_cont->data[1 + shift]);
	shift = aw8697->ram.ram_shift;
	for (i = shift; i < aw8697_cont->len; i++) {
		aw8697->ramupdate_flag =
		    aw8697_i2c_write(aw8697, AW8697_REG_RAMDATA,
				     aw8697_cont->data[i]);
	}

#if 0
	/* ram check */
	shift = aw8697->ram.baseaddr_shift;
	aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRH,
			 aw8697_cont->data[0 + shift]);
	aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRL,
			 aw8697_cont->data[1 + shift]);
	shift = aw8697->ram.ram_shift;
	for (i = shift; i < aw8697_cont->len; i++) {
		aw8697_i2c_read(aw8697, AW8697_REG_RAMDATA, &reg_val);
		if (reg_val != aw8697_cont->data[i]) {
			pr_err
			("%s: ram check error addr=0x%04x, file_data=0x%02x, ram_data=0x%02x\n",
			__func__, i, aw8697_cont->data[i], reg_val);
			return;
		}
	}
#endif

	/* RAMINIT Disable */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8697_BIT_SYSCTRL_RAMINIT_OFF);

	mutex_unlock(&aw8697->lock);

	pr_info("%s exit\n", __func__);
}

static void aw8697_ram_loaded(const struct firmware *cont, void *context)
{
	struct aw8697 *aw8697 = context;
	struct aw8697_container *aw8697_fw;
	int i = 0;
	unsigned short check_sum = 0;

#ifdef AWINIC_READ_BIN_FLEXBALLY
	static unsigned char load_cont;
	int ram_timer_val = 1000;

	load_cont++;
#endif
	pr_info("%s enter\n", __func__);

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw8697_ram_name);
		release_firmware(cont);
#ifdef AWINIC_READ_BIN_FLEXBALLY
		if (load_cont <= 20) {
			schedule_delayed_work(&aw8697->ram_work,
					      msecs_to_jiffies(ram_timer_val));
			pr_info("%s:start hrtimer:load_cont%d\n", __func__,
				load_cont);
		}
#endif
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw8697_ram_name,
		cont ? cont->size : 0);
/*
	for(i=0; i<cont->size; i++) {
		pr_info("%s: addr:0x%04x, data:0x%02x\n", __func__, i, *(cont->data+i));
	}
*/

	/* check sum */
	for (i = 2; i < cont->size; i++)
		check_sum += cont->data[i];
	if (check_sum !=
	    (unsigned short)((cont->data[0] << 8) | (cont->data[1]))) {
		pr_err("%s: check sum err: check_sum=0x%04x\n", __func__,
		       check_sum);
		return;
	} else {
		pr_info("%s: check sum pass : 0x%04x\n", __func__, check_sum);
		aw8697->ram.check_sum = check_sum;
	}

	/* aw8697 ram update */
	aw8697_fw = kzalloc(cont->size + sizeof(int), GFP_KERNEL);
	if (!aw8697_fw) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw8697_fw->len = cont->size;
	memcpy(aw8697_fw->data, cont->data, cont->size);
	release_firmware(cont);

	aw8697_container_update(aw8697, aw8697_fw);

	aw8697->ram.len = aw8697_fw->len;

	kfree(aw8697_fw);

	aw8697->ram_init = 1;
	pr_info("%s: fw update complete\n", __func__);

	aw8697_haptic_trig_enable_config(aw8697);

	aw8697_rtp_update(aw8697);
}

static int aw8697_ram_update(struct aw8697 *aw8697)
{
	aw8697->ram_init = 0;
	aw8697->rtp_init = 0;
	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				       aw8697_ram_name, aw8697->dev, GFP_KERNEL,
				       aw8697, aw8697_ram_loaded);
}

#ifdef AWINIC_RAM_UPDATE_DELAY
static void aw8697_ram_work_routine(struct work_struct *work)
{
	struct aw8697 *aw8697 =
	    container_of(work, struct aw8697, ram_work.work);

	pr_info("%s enter\n", __func__);

	aw8697_ram_update(aw8697);

}
#endif

static int aw8697_ram_init(struct aw8697 *aw8697)
{
#ifdef AWINIC_RAM_UPDATE_DELAY
	int ram_timer_val = 2500;

	INIT_DELAYED_WORK(&aw8697->ram_work, aw8697_ram_work_routine);
	schedule_delayed_work(&aw8697->ram_work,
			      msecs_to_jiffies(ram_timer_val));
#else
	aw8697_ram_update(aw8697);
#endif
	return 0;
}

static int aw8697_haptic_set_gain(struct aw8697 *aw8697, unsigned char gain);
static void aw8697_gain_routine(struct work_struct *work)
{
	struct aw8697 *aw8697 =
	    container_of(work, struct aw8697, gain_work.work);

	pr_info("%s enter: gain=%d gain_trig=%d\n", __func__, aw8697->gain, aw8697->gain_trig);
	aw8697->gain = aw8697->gain_trig;
	aw8697_haptic_set_gain(aw8697, aw8697->gain_trig);
}

/*****************************************************
 *
 * haptic control
 *
 *****************************************************/
static int aw8697_haptic_softreset(struct aw8697 *aw8697)
{
	pr_info("%s enter\n", __func__);

	aw8697_i2c_write(aw8697, AW8697_REG_ID, 0xAA);
	usleep_range(3000, 3500);
	return 0;
}

static int aw8697_haptic_active(struct aw8697 *aw8697)
{
	if((gVibDebugLog & 0x0001)== 0x0001)
		pr_info("%s enter\n", __func__);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_WORK_MODE_MASK,
			      AW8697_BIT_SYSCTRL_ACTIVE);
	aw8697_interrupt_clear(aw8697);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
			      AW8697_BIT_SYSINTM_UVLO_MASK,
			      AW8697_BIT_SYSINTM_UVLO_EN);
	return 0;
}

static int aw8697_haptic_play_mode(struct aw8697 *aw8697,
				   unsigned char play_mode)
{
	//pr_info("%s enter\n", __func__);

	switch (play_mode) {
	case AW8697_HAPTIC_STANDBY_MODE:
		aw8697->play_mode = AW8697_HAPTIC_STANDBY_MODE;
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
				      AW8697_BIT_SYSINTM_UVLO_MASK,
				      AW8697_BIT_SYSINTM_UVLO_OFF);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_WORK_MODE_MASK,
				      AW8697_BIT_SYSCTRL_STANDBY);
		break;
	case AW8697_HAPTIC_RAM_MODE:
		aw8697->play_mode = AW8697_HAPTIC_RAM_MODE;
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_PLAY_MODE_MASK,
				      AW8697_BIT_SYSCTRL_PLAY_MODE_RAM);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_BST_MODE_MASK,
				      AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);
		if (aw8697->auto_boost) {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK,
					AW8697_BIT_BST_AUTO_BST_RAM_DISABLE);
		}
		aw8697_haptic_active(aw8697);
		if (aw8697->auto_boost) {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK,
					AW8697_BIT_BST_AUTO_BST_RAM_ENABLE);
			aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
					      AW8697_BIT_SYSCTRL_BST_MODE_MASK &
					      AW8697_BIT_SYSCTRL_WORK_MODE_MASK,
					      AW8697_BIT_SYSCTRL_BST_MODE_BOOST
					      | AW8697_BIT_SYSCTRL_STANDBY);
			aw8697_haptic_active(aw8697);
		} else {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
					AW8697_BIT_SYSCTRL_BST_MODE_MASK,
					AW8697_BIT_SYSCTRL_BST_MODE_BOOST);
		}
		usleep_range(2000, 2500);
		break;
	case AW8697_HAPTIC_RAM_LOOP_MODE:
		aw8697->play_mode = AW8697_HAPTIC_RAM_LOOP_MODE;
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_PLAY_MODE_MASK,
				      AW8697_BIT_SYSCTRL_PLAY_MODE_RAM);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_BST_MODE_MASK,
				      AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);
		if (aw8697->auto_boost) {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK,
					AW8697_BIT_BST_AUTO_BST_RAM_DISABLE);
		}
		aw8697_haptic_active(aw8697);
		break;
	case AW8697_HAPTIC_RTP_MODE:
		aw8697->play_mode = AW8697_HAPTIC_RTP_MODE;
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_PLAY_MODE_MASK,
				      AW8697_BIT_SYSCTRL_PLAY_MODE_RTP);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_BST_MODE_MASK,
				      AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);
		if (aw8697->auto_boost) {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK,
					AW8697_BIT_BST_AUTO_BST_RTP_DISABLE);
		}
		aw8697_haptic_active(aw8697);
		if (aw8697->auto_boost) {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK,
					AW8697_BIT_BST_AUTO_BST_RTP_ENABLE);
			aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
					      AW8697_BIT_SYSCTRL_BST_MODE_MASK &
					      AW8697_BIT_SYSCTRL_WORK_MODE_MASK,
					      AW8697_BIT_SYSCTRL_BST_MODE_BOOST
					      | AW8697_BIT_SYSCTRL_STANDBY);
			aw8697_haptic_active(aw8697);
		} else {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
					AW8697_BIT_SYSCTRL_BST_MODE_MASK,
					AW8697_BIT_SYSCTRL_BST_MODE_BOOST);
		}
		usleep_range(2000, 2500);
		break;
	case AW8697_HAPTIC_TRIG_MODE:
		aw8697->play_mode = AW8697_HAPTIC_TRIG_MODE;
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_PLAY_MODE_MASK,
				      AW8697_BIT_SYSCTRL_PLAY_MODE_RAM);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_BST_MODE_MASK,
				      AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);
		if (aw8697->auto_boost) {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK,
					AW8697_BIT_BST_AUTO_BST_RAM_DISABLE);
		}
		aw8697_haptic_active(aw8697);
		if (aw8697->auto_boost) {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK,
					AW8697_BIT_BST_AUTO_BST_RAM_ENABLE);
			aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
					      AW8697_BIT_SYSCTRL_BST_MODE_MASK &
					      AW8697_BIT_SYSCTRL_WORK_MODE_MASK,
					      AW8697_BIT_SYSCTRL_BST_MODE_BOOST
					      | AW8697_BIT_SYSCTRL_STANDBY);
			aw8697_haptic_active(aw8697);
		} else {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
					AW8697_BIT_SYSCTRL_BST_MODE_MASK,
					AW8697_BIT_SYSCTRL_BST_MODE_BOOST);
		}
		usleep_range(2000, 2500);
		break;
	case AW8697_HAPTIC_CONT_MODE:
		aw8697->play_mode = AW8697_HAPTIC_CONT_MODE;
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_PLAY_MODE_MASK,
				      AW8697_BIT_SYSCTRL_PLAY_MODE_CONT);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_BST_MODE_MASK,
				      AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);
		if (aw8697->auto_boost) {
			aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK,
					AW8697_BIT_BST_AUTO_BST_RAM_DISABLE);
		}
		aw8697_haptic_active(aw8697);
		break;
	default:
		dev_err(aw8697->dev, "%s: play mode %d err",
			__func__, play_mode);
		break;
	}
	return 0;
}

#ifdef RTP_MONITOR_ENABLE
static int aw8697_haptic_juge_RTP_is_going_on(struct aw8697 *aw8697)
{
	unsigned char reg_val = 0;
	unsigned char rtp_state = 0;
	static unsigned char pre_reg_val;

	aw8697_i2c_read(aw8697, AW8697_REG_SYSCTRL, &reg_val);
	if ((reg_val & AW8697_BIT_SYSCTRL_PLAY_MODE_RTP) &&
		(!(reg_val & AW8697_BIT_SYSCTRL_STANDBY)))
		rtp_state = 1;	/*is going on */
	if (pre_reg_val != reg_val)
		pr_info("%s AW8697_REG_SYSCTRL 0x04==%02x rtp_state=%d\n",
			__func__, reg_val, rtp_state);
	pre_reg_val = reg_val;
	if (aw8697->rtp_routine_on) {
		pr_info("%s:rtp_routine_on\n", __func__);
		rtp_state = 1;	/*is going on */
	}
	return rtp_state;
}
#endif
static int aw8697_haptic_play_go(struct aw8697 *aw8697, bool flag)
{
	//pr_info("%s enter, flag = %d\n", __func__, flag);

	if (flag == true) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_GO,
				      AW8697_BIT_GO_MASK, AW8697_BIT_GO_ENABLE);
		//ktime_get_real_ts64(&aw8697->pre_enter_time);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_GO,
				      AW8697_BIT_GO_MASK,
				      AW8697_BIT_GO_DISABLE);
		/*
		ktime_get_real_ts64(&aw8697->current_time);
		aw8697->interval_us = (aw8697->current_time.tv_sec - aw8697->pre_enter_time.tv_sec) * 1000000
		+ (aw8697->current_time.tv_nsec-aw8697->pre_enter_time.tv_nsec)/1000;
		if (aw8697->interval_us < 2000) {
			pr_info("aw8697->interval_us t=%d\n", aw8697->interval_us);
			mdelay(2);
		}
		*/
	}
	return 0;
}

static int aw8697_haptic_stop_delay(struct aw8697 *aw8697)
{
	unsigned char reg_val = 0;
	unsigned int cnt = 100;

	while (cnt--) {
		aw8697_i2c_read(aw8697, AW8697_REG_GLB_STATE, &reg_val);
		if ((reg_val & 0x0f) == 0x00)
			return 0;
		usleep_range(2000, 2500);
		pr_info("%s wait for standby, reg glb_state=0x%02x\n",
			 __func__, reg_val);
	}
	pr_err("%s do not enter standby automatically\n", __func__);

	return 0;
}

static int aw8697_haptic_stop(struct aw8697 *aw8697)
{
	if((gVibDebugLog & 0x0001)== 0x0001)
		pr_info("%s enter\n", __func__);

	aw8697_haptic_play_go(aw8697, false);
	aw8697_haptic_stop_delay(aw8697);
	aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_STANDBY_MODE);
	//Daniel 20210223 modify start
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
		AW8697_BIT_SYSCTRL_BST_MODE_MASK,
		AW8697_BIT_SYSCTRL_BST_MODE_BOOST);
	//Daniel 20210223 modify end

	return 0;
}

static int aw8697_haptic_start(struct aw8697 *aw8697)
{
	//pr_info("%s enter\n", __func__);

	aw8697_haptic_play_go(aw8697, true);

	return 0;
}

static int aw8697_haptic_set_wav_seq(struct aw8697 *aw8697,
				     unsigned char wav, unsigned char seq)
{
	aw8697_i2c_write(aw8697, AW8697_REG_WAVSEQ1 + wav, seq);
	return 0;
}

static int aw8697_haptic_set_wav_loop(struct aw8697 *aw8697,
				      unsigned char wav, unsigned char loop)
{
	unsigned char tmp = 0;

	if (wav % 2) {
		tmp = loop << 0;
		aw8697_i2c_write_bits(aw8697, AW8697_REG_WAVLOOP1 + (wav / 2),
				      AW8697_BIT_WAVLOOP_SEQNP1_MASK, tmp);
	} else {
		tmp = loop << 4;
		aw8697_i2c_write_bits(aw8697, AW8697_REG_WAVLOOP1 + (wav / 2),
				      AW8697_BIT_WAVLOOP_SEQN_MASK, tmp);
	}

	return 0;
}

/*
static int aw8697_haptic_set_main_loop(struct aw8697 *aw8697,
				       unsigned char loop)
{
	aw8697_i2c_write(aw8697, AW8697_REG_MAIN_LOOP, loop);
	return 0;
}
*/

static int aw8697_haptic_set_repeat_wav_seq(struct aw8697 *aw8697,
					    unsigned char seq)
{
	aw8697_haptic_set_wav_seq(aw8697, 0x00, seq);
	aw8697_haptic_set_wav_loop(aw8697, 0x00,
				   AW8697_BIT_WAVLOOP_INIFINITELY);

	return 0;
}

static int aw8697_haptic_set_bst_vol(struct aw8697 *aw8697,
				     unsigned char bst_vol)
{
	if (bst_vol & 0xe0)
		bst_vol = 0x1f;
	aw8697_i2c_write_bits(aw8697, AW8697_REG_BSTDBG4,
			      AW8697_BIT_BSTDBG4_BSTVOL_MASK, (bst_vol << 1));
	return 0;
}

static int aw8697_haptic_set_bst_peak_cur(struct aw8697 *aw8697,
					  unsigned char peak_cur)
{
	peak_cur &= AW8697_BSTCFG_PEAKCUR_LIMIT;
	aw8697_i2c_write_bits(aw8697, AW8697_REG_BSTCFG,
			      AW8697_BIT_BSTCFG_PEAKCUR_MASK, peak_cur);
	return 0;
}

static int aw8697_haptic_set_gain(struct aw8697 *aw8697, unsigned char gain)
{
	aw8697_i2c_write(aw8697, AW8697_REG_DATDBG, gain);
	return 0;
}

static int aw8697_haptic_set_pwm(struct aw8697 *aw8697, unsigned char mode)
{
	if (gRTP_PWM != mode){
	unsigned char new = 0;
	unsigned char old = 0;
	switch (mode) {
	case AW8697_PWM_48K:
		new = 48;
		break;
	case AW8697_PWM_24K:
		new = 24;
		break;
	case AW8697_PWM_12K:
		new = 12;
		break;
	}
	switch (gRTP_PWM) {
	case AW8697_PWM_48K:
		old = 48;
		break;
	case AW8697_PWM_24K:
		old = 24;
		break;
	case AW8697_PWM_12K:
		old = 12;
		break;
	}
		pr_info("%s change out pwn %d -> %d kHz\n", __func__, old, new);
		gRTP_PWM = (int)mode;
		//pr_info("%s gRTP_PWM = %d \n", __func__, gRTP_PWM);
	}else
		return 0;

	switch (mode) {
	case AW8697_PWM_48K:
		aw8697_i2c_write_bits(aw8697, AW8697_REG_PWMDBG,
				      AW8697_BIT_PWMDBG_PWM_MODE_MASK,
				      AW8697_BIT_PWMDBG_PWM_48K);
		break;
	case AW8697_PWM_24K:
		aw8697_i2c_write_bits(aw8697, AW8697_REG_PWMDBG,
				      AW8697_BIT_PWMDBG_PWM_MODE_MASK,
				      AW8697_BIT_PWMDBG_PWM_24K);
		break;
	case AW8697_PWM_12K:
		aw8697_i2c_write_bits(aw8697, AW8697_REG_PWMDBG,
				      AW8697_BIT_PWMDBG_PWM_MODE_MASK,
				      AW8697_BIT_PWMDBG_PWM_12K);
		break;
	default:
		break;
	}
	return 0;
}

static int aw8697_haptic_play_wav_seq(struct aw8697 *aw8697, unsigned char flag)
{
	pr_info("%s enter\n", __func__);

	if (flag) {
		aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_RAM_MODE);
		aw8697_haptic_start(aw8697);
	}
	return 0;
}

static int aw8697_haptic_play_repeat_seq(struct aw8697 *aw8697,
					 unsigned char flag)
{
	pr_info("%s enter\n", __func__);

	if (flag) {
		aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_RAM_LOOP_MODE);
		aw8697_haptic_start(aw8697);
	}

	return 0;
}

/*****************************************************
 *
 * motor protect
 *
 *****************************************************/
static int aw8697_haptic_swicth_motorprotect_config(struct aw8697 *aw8697,
						    unsigned char addr,
						    unsigned char val)
{
	pr_info("%s enter\n", __func__);
	if (addr == 1) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
				      AW8697_BIT_DETCTRL_PROTECT_MASK,
				      AW8697_BIT_DETCTRL_PROTECT_SHUTDOWN);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_PWMPRC,
				      AW8697_BIT_PWMPRC_PRC_MASK,
				      AW8697_BIT_PWMPRC_PRC_ENABLE);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_PRLVL,
				      AW8697_BIT_PRLVL_PR_MASK,
				      AW8697_BIT_PRLVL_PR_ENABLE);
	} else if (addr == 0) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
				      AW8697_BIT_DETCTRL_PROTECT_MASK,
				      AW8697_BIT_DETCTRL_PROTECT_NO_ACTION);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_PWMPRC,
				      AW8697_BIT_PWMPRC_PRC_MASK,
				      AW8697_BIT_PWMPRC_PRC_DISABLE);
		aw8697_i2c_write_bits(aw8697, AW8697_REG_PRLVL,
				      AW8697_BIT_PRLVL_PR_MASK,
				      AW8697_BIT_PRLVL_PR_DISABLE);
	} else if (addr == 0x2d) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_PWMPRC,
				      AW8697_BIT_PWMPRC_PRCTIME_MASK, val);
	} else if (addr == 0x3e) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_PRLVL,
				      AW8697_BIT_PRLVL_PRLVL_MASK, val);
	} else if (addr == 0x3f) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_PRTIME,
				      AW8697_BIT_PRTIME_PRTIME_MASK, val);
	} else {
		/*nothing to do; */
	}
	return 0;
}

/*****************************************************
 *
 * offset calibration
 *
 *****************************************************/
static int aw8697_haptic_offset_calibration(struct aw8697 *aw8697)
{
	unsigned int cont = 2000;
	unsigned char reg_val = 0;

	pr_info("%s enter\n", __func__);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8697_BIT_SYSCTRL_RAMINIT_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
			      AW8697_BIT_DETCTRL_DIAG_GO_MASK,
			      AW8697_BIT_DETCTRL_DIAG_GO_ENABLE);
	while (1) {
		aw8697_i2c_read(aw8697, AW8697_REG_DETCTRL, &reg_val);
		if ((reg_val & 0x01) == 0 || cont == 0)
			break;
		cont--;
	}
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8697_BIT_SYSCTRL_RAMINIT_OFF);

	return 0;
}

/*****************************************************
 *
 * trig config
 *
 *****************************************************/
static int aw8697_haptic_trig_param_init(struct aw8697 *aw8697)
{
	pr_info("%s enter\n", __func__);
#if 1//ASUS
	aw8697->trig[0].enable = AW8697_TRG1_DISABLE;
	aw8697->trig[0].default_level = AW8697_TRG1_DEFAULT_LEVEL;
	aw8697->trig[0].dual_edge = AW8697_TRG1_DUAL_EDGE;
	aw8697->trig[0].frist_seq = 3;//RAM waveform no.2
	aw8697->trig[0].second_seq = 0;//dont use for singel edge

	aw8697->trig[1].enable = AW8697_TRG2_DISABLE;
	aw8697->trig[1].default_level = AW8697_TRG2_DEFAULT_LEVEL;
	aw8697->trig[1].dual_edge = AW8697_TRG2_DUAL_EDGE;
	aw8697->trig[1].frist_seq = 4;
	aw8697->trig[1].second_seq = 0;

	aw8697->trig[2].enable = AW8697_TRG3_DISABLE;
	aw8697->trig[2].default_level = AW8697_TRG3_DEFAULT_LEVEL;
	aw8697->trig[2].dual_edge = AW8697_TRG3_DUAL_EDGE;
	aw8697->trig[2].frist_seq = 2;
	aw8697->trig[2].second_seq = 0;
#else
	aw8697->trig[0].enable = AW8697_TRG1_ENABLE;
	aw8697->trig[0].default_level = AW8697_TRG1_DEFAULT_LEVEL;
	aw8697->trig[0].dual_edge = AW8697_TRG1_DUAL_EDGE;
	aw8697->trig[0].frist_seq = AW8697_TRG1_FIRST_EDGE_SEQ;
	aw8697->trig[0].second_seq = AW8697_TRG1_SECOND_EDGE_SEQ;

	aw8697->trig[1].enable = AW8697_TRG2_ENABLE;
	aw8697->trig[1].default_level = AW8697_TRG2_DEFAULT_LEVEL;
	aw8697->trig[1].dual_edge = AW8697_TRG2_DUAL_EDGE;
	aw8697->trig[1].frist_seq = AW8697_TRG2_FIRST_EDGE_SEQ;
	aw8697->trig[1].second_seq = AW8697_TRG2_SECOND_EDGE_SEQ;

	aw8697->trig[2].enable = AW8697_TRG3_ENABLE;
	aw8697->trig[2].default_level = AW8697_TRG3_DEFAULT_LEVEL;
	aw8697->trig[2].dual_edge = AW8697_TRG3_DUAL_EDGE;
	aw8697->trig[2].frist_seq = AW8697_TRG3_FIRST_EDGE_SEQ;
	aw8697->trig[2].second_seq = AW8697_TRG3_SECOND_EDGE_SEQ;
#endif
	return 0;
}

static int aw8697_haptic_trig_param_config(struct aw8697 *aw8697)
{
	pr_info("%s enter\n", __func__);

	if (aw8697->trig[0].default_level) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG1_POLAR_MASK,
				      AW8697_BIT_TRGCFG1_TRG1_POLAR_NEG);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG1_POLAR_MASK,
				      AW8697_BIT_TRGCFG1_TRG1_POLAR_POS);
	}
	if (aw8697->trig[1].default_level) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG2_POLAR_MASK,
				      AW8697_BIT_TRGCFG1_TRG2_POLAR_NEG);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG2_POLAR_MASK,
				      AW8697_BIT_TRGCFG1_TRG2_POLAR_POS);
	}
	if (aw8697->trig[2].default_level) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG3_POLAR_MASK,
				      AW8697_BIT_TRGCFG1_TRG3_POLAR_NEG);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG3_POLAR_MASK,
				      AW8697_BIT_TRGCFG1_TRG3_POLAR_POS);
	}

	if (aw8697->trig[0].dual_edge) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG1_EDGE_MASK,
				      AW8697_BIT_TRGCFG1_TRG1_EDGE_POS_NEG);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG1_EDGE_MASK,
				      AW8697_BIT_TRGCFG1_TRG1_EDGE_POS);
	}
	if (aw8697->trig[1].dual_edge) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG2_EDGE_MASK,
				      AW8697_BIT_TRGCFG1_TRG2_EDGE_POS_NEG);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG2_EDGE_MASK,
				      AW8697_BIT_TRGCFG1_TRG2_EDGE_POS);
	}
	if (aw8697->trig[2].dual_edge) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG3_EDGE_MASK,
				      AW8697_BIT_TRGCFG1_TRG3_EDGE_POS_NEG);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG3_EDGE_MASK,
				      AW8697_BIT_TRGCFG1_TRG3_EDGE_POS);
	}

	if (aw8697->trig[0].frist_seq) {
		aw8697_i2c_write(aw8697, AW8697_REG_TRG1_WAV_P,
				 aw8697->trig[0].frist_seq);
	}
	if (aw8697->trig[0].second_seq && aw8697->trig[0].dual_edge) {
		aw8697_i2c_write(aw8697, AW8697_REG_TRG1_WAV_N,
				 aw8697->trig[0].second_seq);
	}
	if (aw8697->trig[1].frist_seq) {
		aw8697_i2c_write(aw8697, AW8697_REG_TRG2_WAV_P,
				 aw8697->trig[1].frist_seq);
	}
	if (aw8697->trig[1].second_seq && aw8697->trig[1].dual_edge) {
		aw8697_i2c_write(aw8697, AW8697_REG_TRG2_WAV_N,
				 aw8697->trig[1].second_seq);
	}
	if (aw8697->trig[2].frist_seq) {
		aw8697_i2c_write(aw8697, AW8697_REG_TRG3_WAV_P,
				 aw8697->trig[1].frist_seq);
	}
	if (aw8697->trig[2].second_seq && aw8697->trig[2].dual_edge) {
		aw8697_i2c_write(aw8697, AW8697_REG_TRG3_WAV_N,
				 aw8697->trig[1].second_seq);
	}

	return 0;
}

static int aw8697_haptic_trig_enable_config(struct aw8697 *aw8697)
{
	pr_info("%s enter\n", __func__);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG2,
			      AW8697_BIT_TRGCFG2_TRG1_ENABLE_MASK,
			      aw8697->trig[0].enable << 0);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG2,
			      AW8697_BIT_TRGCFG2_TRG2_ENABLE_MASK,
			      aw8697->trig[1].enable << 1);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_TRG_CFG2,
			      AW8697_BIT_TRGCFG2_TRG3_ENABLE_MASK,
			      aw8697->trig[2].enable << 2);

	return 0;
}

static int aw8697_haptic_auto_boost_config(struct aw8697 *aw8697,
					   unsigned char flag)
{
	aw8697->auto_boost = flag;
	if (flag) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
				      AW8697_BIT_BST_AUTO_BST_AUTOSW_MASK,
				      AW8697_BIT_BST_AUTO_BST_AUTOMATIC_BOOST);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_BST_AUTO,
				      AW8697_BIT_BST_AUTO_BST_AUTOSW_MASK,
				      AW8697_BIT_BST_AUTO_BST_MANUAL_BOOST);
	}
	return 0;
}

/*****************************************************
 *
 * vbat mode
 *
 *****************************************************/
static int aw8697_haptic_cont_vbat_mode(struct aw8697 *aw8697,
					unsigned char flag)
{
	if (flag == AW8697_HAPTIC_CONT_VBAT_HW_COMP_MODE) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_ADCTEST,
				      AW8697_BIT_ADCTEST_VBAT_MODE_MASK,
				      AW8697_BIT_ADCTEST_VBAT_HW_COMP);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_ADCTEST,
				      AW8697_BIT_ADCTEST_VBAT_MODE_MASK,
				      AW8697_BIT_ADCTEST_VBAT_SW_COMP);
	}
	return 0;
}

static int aw8697_haptic_get_vbat(struct aw8697 *aw8697)
{
	unsigned char reg_val = 0;
	unsigned int cont = 2000;

	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8697_BIT_SYSCTRL_RAMINIT_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
			      AW8697_BIT_DETCTRL_VBAT_GO_MASK,
			      AW8697_BIT_DETCTRL_VABT_GO_ENABLE);

	while (1) {
		aw8697_i2c_read(aw8697, AW8697_REG_DETCTRL, &reg_val);
		if ((reg_val & 0x02) == 0 || cont == 0)
			break;
		cont--;
	}

	aw8697_i2c_read(aw8697, AW8697_REG_VBATDET, &reg_val);
	aw8697->vbat = 6100 * reg_val / 256;
	if (aw8697->vbat > AW8697_VBAT_MAX) {
		aw8697->vbat = AW8697_VBAT_MAX;
		pr_info("%s vbat max limit = %dmV\n", __func__, aw8697->vbat);
	}
	if (aw8697->vbat < AW8697_VBAT_MIN) {
		aw8697->vbat = AW8697_VBAT_MIN;
		pr_info("%s vbat min limit = %dmV\n", __func__, aw8697->vbat);
	}

	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8697_BIT_SYSCTRL_RAMINIT_OFF);

	return 0;
}

static int aw8697_haptic_ram_vbat_comp(struct aw8697 *aw8697, bool flag)
{
	int temp_gain = 0;

	if (flag) {
		if
		(aw8697->ram_vbat_comp == AW8697_HAPTIC_RAM_VBAT_COMP_ENABLE) {
			aw8697_haptic_get_vbat(aw8697);
			temp_gain =
			    aw8697->gain * AW8697_VBAT_REFER / aw8697->vbat;
			if (temp_gain >
			    (128 * AW8697_VBAT_REFER / AW8697_VBAT_MIN)) {
				temp_gain =
				    128 * AW8697_VBAT_REFER / AW8697_VBAT_MIN;
				pr_info("%s gain limit=%d\n", __func__,
					 temp_gain);
			}
			aw8697_haptic_set_gain(aw8697, temp_gain);
		} else {
			aw8697_haptic_set_gain(aw8697, aw8697->gain);
		}
	} else {
		aw8697_haptic_set_gain(aw8697, aw8697->gain);
	}

	return 0;
}

/*****************************************************
 *
 * f0
 *
 *****************************************************/
static int aw8697_haptic_set_f0_preset(struct aw8697 *aw8697)
{
	unsigned int f0_reg = 0;

	pr_info("%s enter\n", __func__);

	f0_reg = 1000000000 / (aw8697->info.f0_pre * aw8697->info.f0_coeff);
	aw8697_i2c_write(aw8697, AW8697_REG_F_PRE_H,
			 (unsigned char)((f0_reg >> 8) & 0xff));
	aw8697_i2c_write(aw8697, AW8697_REG_F_PRE_L,
			 (unsigned char)((f0_reg >> 0) & 0xff));

	return 0;
}

static int aw8697_haptic_read_f0(struct aw8697 *aw8697)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_reg = 0;
	unsigned long f0_tmp = 0;

	pr_info("%s enter\n", __func__);

	ret = aw8697_i2c_read(aw8697, AW8697_REG_F_LRA_F0_H, &reg_val);
	f0_reg = (reg_val << 8);
	ret = aw8697_i2c_read(aw8697, AW8697_REG_F_LRA_F0_L, &reg_val);
	f0_reg |= (reg_val << 0);
	if (!f0_reg) {
		pr_info("%s not get f0 because f0_reg value is 0!\n", __func__);
		return 0;
	}
	f0_tmp = 1000000000 / (f0_reg * aw8697->info.f0_coeff);
	aw8697->f0 = (unsigned int)f0_tmp;
	pr_info("%s f0=%d\n", __func__, aw8697->f0);

	return 0;
}

static int aw8697_haptic_read_cont_f0(struct aw8697 *aw8697)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_reg = 0;
	unsigned long f0_tmp = 0;

	pr_info("%s enter\n", __func__);

	ret = aw8697_i2c_read(aw8697, AW8697_REG_F_LRA_CONT_H, &reg_val);
	f0_reg = (reg_val << 8);
	ret = aw8697_i2c_read(aw8697, AW8697_REG_F_LRA_CONT_L, &reg_val);
	f0_reg |= (reg_val << 0);
	f0_tmp = 1000000000 / (f0_reg * aw8697->info.f0_coeff);
	aw8697->cont_f0 = (unsigned int)f0_tmp;
	pr_info("%s f0=%d\n", __func__, aw8697->cont_f0);

	return 0;
}

static int aw8697_haptic_read_beme(struct aw8697 *aw8697)
{
	int ret = 0;
	unsigned char reg_val = 0;

	ret = aw8697_i2c_read(aw8697, AW8697_REG_WAIT_VOL_MP, &reg_val);
	aw8697->max_pos_beme = (reg_val << 0);
	ret = aw8697_i2c_read(aw8697, AW8697_REG_WAIT_VOL_MN, &reg_val);
	aw8697->max_neg_beme = (reg_val << 0);

	pr_info("%s max_pos_beme=%d\n", __func__, aw8697->max_pos_beme);
	pr_info("%s max_neg_beme=%d\n", __func__, aw8697->max_neg_beme);

	return 0;
}

/*****************************************************
 *
 * rtp
 *
 *****************************************************/
static void aw8697_haptic_set_rtp_aei(struct aw8697 *aw8697, bool flag)
{
	if (flag) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
				      AW8697_BIT_SYSINTM_FF_AE_MASK,
				      AW8697_BIT_SYSINTM_FF_AE_EN);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
				      AW8697_BIT_SYSINTM_FF_AE_MASK,
				      AW8697_BIT_SYSINTM_FF_AE_OFF);
	}
}

/*
static void aw8697_haptic_set_rtp_afi(struct aw8697 *aw8697, bool flag)
{
	if(flag) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
		AW8697_BIT_SYSINTM_FF_AF_MASK, AW8697_BIT_SYSINTM_FF_AF_EN);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
		AW8697_BIT_SYSINTM_FF_AF_MASK, AW8697_BIT_SYSINTM_FF_AF_OFF);
	}
}
*/

/*
static unsigned char aw8697_haptic_rtp_get_fifo_aei(struct aw8697 *aw8697)
{
	unsigned char ret;
	unsigned char reg_val;

	aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);
	reg_val &= AW8697_BIT_SYSINT_FF_AEI;
	ret = reg_val>>4;

	return ret;
}
*/

/*
static unsigned char aw8697_haptic_rtp_get_fifo_aes(struct aw8697 *aw8697)
{
	unsigned char ret;
	unsigned char reg_val;

	aw8697_i2c_read(aw8697, AW8697_REG_SYSST, &reg_val);
	reg_val &= AW8697_BIT_SYSST_FF_AES;
	ret = reg_val>>4;

	return ret;
}
*/

static unsigned char aw8697_haptic_rtp_get_fifo_afi(struct aw8697 *aw8697)
{
	unsigned char ret = 0;
	unsigned char reg_val = 0;

	if (aw8697->osc_cali_flag == 1) {
		aw8697_i2c_read(aw8697, AW8697_REG_SYSST, &reg_val);
		reg_val &= AW8697_BIT_SYSST_FF_AFS;
		ret = reg_val >> 3;
	} else {
		aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);
		reg_val &= AW8697_BIT_SYSINT_FF_AFI;
		ret = reg_val >> 3;
	}
	//pr_info("%s ret = %d\n", __func__, ret);
	return ret;
}


static unsigned char aw8697_haptic_rtp_get_fifo_afs(struct aw8697 *aw8697)
{
	unsigned char ret = 0;
	unsigned char reg_val = 0;

	aw8697_i2c_read(aw8697, AW8697_REG_SYSST, &reg_val);
	reg_val &= AW8697_BIT_SYSST_FF_AFS;
	ret = reg_val>>3;

	return ret;
}


/*****************************************************
 *
 * rtp
 *
 *****************************************************/
static int aw8697_haptic_rtp_init(struct aw8697 *aw8697)
{
	unsigned int buf_len = 0;
	unsigned char glb_state_val = 0;

	//pr_info("%s enter\n", __func__);
	pm_qos_add_request(&pm_qos_req_vb, PM_QOS_CPU_DMA_LATENCY, PM_QOS_VALUE_VB);
	aw8697->rtp_cnt = 0;
	mutex_lock(&aw8697->rtp_lock);
	while ((!aw8697_haptic_rtp_get_fifo_afs(aw8697)) &&
	       (aw8697->play_mode == AW8697_HAPTIC_RTP_MODE)) {
		//pr_info("%s rtp cnt = %d\n", __func__, aw8697->rtp_cnt);
		if (!aw8697_rtp) {
			pr_info("%s:aw8697_rtp is null break\n", __func__);
			mutex_unlock(&aw8697->rtp_lock);
			break;
		}

		if ((aw8697->rtp_cnt < aw8697->ram.base_addr)) {
			if((aw8697_rtp->len-aw8697->rtp_cnt) < (aw8697->ram.base_addr)) {
				buf_len = aw8697_rtp->len-aw8697->rtp_cnt;
			} else {
				buf_len = (aw8697->ram.base_addr);
			}
		} else if ((aw8697_rtp->len - aw8697->rtp_cnt) < (aw8697->ram.base_addr >> 2)) {
			buf_len = aw8697_rtp->len - aw8697->rtp_cnt;
		} else {
			buf_len = (aw8697->ram.base_addr >> 2);
		}
		//pr_info("%s buf_len = %d\n", __func__, buf_len);
		aw8697_i2c_writes(aw8697, AW8697_REG_RTP_DATA,
				  &aw8697_rtp->data[aw8697->rtp_cnt], buf_len);
		aw8697->rtp_cnt += buf_len;
		pr_info("%s rtp cnt = %d\n", __func__, aw8697->rtp_cnt);
		aw8697_i2c_read(aw8697, AW8697_REG_GLB_STATE, &glb_state_val);
		if ((aw8697->rtp_cnt == aw8697_rtp->len) || ((glb_state_val & 0x0f) == 0x00)) {
			pr_info("%s: rtp update complete\n", __func__);
			aw8697->rtp_cnt = 0;
			mutex_unlock(&aw8697->rtp_lock);
			pm_qos_remove_request(&pm_qos_req_vb);
			return 0;
		}
	}
	mutex_unlock(&aw8697->rtp_lock);

	if (aw8697->play_mode == AW8697_HAPTIC_RTP_MODE)
		aw8697_haptic_set_rtp_aei(aw8697, true);

	pr_info("%s exit\n", __func__);
	pm_qos_remove_request(&pm_qos_req_vb);
	return 0;
}

static void aw8697_haptic_upload_lra(struct aw8697 *aw8697, unsigned int flag)
{
	switch (flag) {
	case 1:
		if((gVibDebugLog & 0x0001)== 0x0001)
			printk("%s f0_cali_lra=%d\n", __func__, aw8697->f0_calib_data);
		aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA,
				 (char)aw8697->f0_calib_data);
		break;
	case 2:
		if((gVibDebugLog & 0x0001)== 0x0001)
			printk("%s rtp_cali_lra=%d\n", __func__, aw8697->lra_calib_data);
		aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA,
				 (char)aw8697->lra_calib_data);
		break;
	default:
		break;
	}
}

static int aw8697_clock_OSC_trim_calibration(unsigned long int theory_time, unsigned long int real_time)
{
	unsigned int real_code = 0;
	unsigned int LRA_TRIM_CODE = 0;
	unsigned int DFT_LRA_TRIM_CODE = 0;
	unsigned int Not_need_cali_threshold = 10;/*0.1 percent not need calibrate*/

	if (theory_time == real_time) {
		pr_info("aw8697_osctheory_time == real_time:%ld  theory_time = %ld not need to cali\n", real_time, theory_time);
		return 0;
	} else if (theory_time < real_time) {
		if ((real_time - theory_time) > (theory_time / 25)) {
			pr_info("aw8697_osc(real_time - theory_time) > (theory_time/50) not to cali\n");
			return DFT_LRA_TRIM_CODE;
		}

		if ((real_time - theory_time) < (Not_need_cali_threshold*theory_time/10000)) {
			pr_info("aw8697_oscmicrosecond:%ld  theory_time = %ld not need to cali\n", real_time, theory_time);
			return DFT_LRA_TRIM_CODE;
		}

		real_code = ((real_time - theory_time) * 4000) / theory_time;
		real_code = ((real_code%10 < 5) ? 0 : 1) + real_code/10;
		real_code = 32 + real_code;
	} else if (theory_time > real_time) {
		if ((theory_time - real_time) > (theory_time / 25)) {
			pr_info("aw8697_osc((theory_time - real_time) > (theory_time / 50)) not to cali\n");
			return DFT_LRA_TRIM_CODE;
		}
		if ((theory_time - real_time) < (Not_need_cali_threshold * theory_time/10000)) {
			pr_info("aw8697_oscmicrosecond:%ld  theory_time = %ld not need to cali\n", real_time, theory_time);
			return DFT_LRA_TRIM_CODE;
		}
		real_code = ((theory_time - real_time) * 4000) / theory_time;
		real_code = ((real_code%10 < 5) ? 0 : 1) + real_code/10;
		real_code = 32 - real_code;
	}
	if (real_code > 31)
		LRA_TRIM_CODE = real_code - 32;
	else
		LRA_TRIM_CODE = real_code + 32;
	pr_info("aw8697_oscmicrosecond:%ld  theory_time = %ld real_code =0X%02X LRA_TRIM_CODE 0X%02X\n", real_time, theory_time, real_code, LRA_TRIM_CODE);

	return LRA_TRIM_CODE;
}

static int aw8697_rtp_trim_lra_calibration(struct aw8697 *aw8697)
{
	unsigned char reg_val = 0;
	unsigned int fre_val = 0;
	unsigned int theory_time = 0;
	unsigned int lra_rtim_code = 0;

	aw8697_i2c_read(aw8697, AW8697_REG_PWMDBG, &reg_val);
	fre_val = (reg_val & 0x006f) >> 5;

	if (fre_val == 3)
		theory_time = (aw8697->rtp_len / 12000) * 1000000; /*12K */
	if (fre_val == 2)
		theory_time = (aw8697->rtp_len / 24000) * 1000000; /*24K */
	if (fre_val == 1 || fre_val == 0)
		theory_time = (aw8697->rtp_len / 48000) * 1000000; /*48K */

	printk("aw8697 microsecond:%ld  theory_time = %d\n", aw8697->microsecond, theory_time);

	lra_rtim_code = aw8697_clock_OSC_trim_calibration(theory_time, aw8697->microsecond);
	if (lra_rtim_code >= 0) {
		aw8697->lra_calib_data = lra_rtim_code;

		//ASUS_BSP: save rtp cali value
		aw8697_write_cali_to_file(aw8697->lra_calib_data, 0, 1);

		aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA, (char)lra_rtim_code);
	}
	return 0;
}
static unsigned char aw8697_haptic_osc_read_int(struct aw8697 *aw8697)
{
	unsigned char reg_val = 0;

	aw8697_i2c_read(aw8697, AW8697_REG_DBGSTAT, &reg_val);
	return reg_val;
}

static int aw8697_rtp_osc_calibration(struct aw8697 *aw8697)
{
	const struct firmware *rtp_file;
	int ret = -1;
	unsigned int buf_len = 0;
	unsigned char osc_int_state = 0;
	aw8697->rtp_cnt = 0;
	aw8697->timeval_flags = 1;
	aw8697->osc_cali_flag = 1;

	pr_info("%s enter\n", __func__);
	/* fw loaded */
	ret = request_firmware(&rtp_file,
			       aw8697_rtp_name[0],/*aw8697->rtp_file_num */
			       aw8697->dev);
	if (ret < 0) {
		pr_err("%s: failed to read %s\n", __func__,
		       aw8697_rtp_name[0]);/*aw8697->rtp_file_num */
		return ret;
	}
	/*awinic add stop,for irq interrupt during calibrate*/
	aw8697_haptic_stop(aw8697);
	aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA, 0x00);
	aw8697->rtp_init = 0;
	mutex_lock(&aw8697->rtp_lock);
	vfree(aw8697_rtp);
	aw8697_rtp = vmalloc(rtp_file->size + sizeof(int));
	if (!aw8697_rtp) {
		release_firmware(rtp_file);
		mutex_unlock(&aw8697->rtp_lock);
		pr_err("%s: error allocating memory\n", __func__);
		return -ENOMEM;
	}
	aw8697_rtp->len = rtp_file->size;
	aw8697->rtp_len = rtp_file->size;
	pr_info("%s: rtp file [%s] size = %d\n", __func__,
		aw8697_rtp_name[0], aw8697_rtp->len);/*aw8697->rtp_file_num */
	memcpy(aw8697_rtp->data, rtp_file->data, rtp_file->size);
	release_firmware(rtp_file);
	mutex_unlock(&aw8697->rtp_lock);

	/* gain */
	aw8697_haptic_ram_vbat_comp(aw8697, false);

	/* rtp mode config */
	aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_RTP_MODE);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_DBGCTRL,
			      AW8697_BIT_DBGCTRL_INT_MODE_MASK,
			      AW8697_BIT_DBGCTRL_INT_MODE_EDGE);
	disable_irq_nosync(gpio_to_irq(aw8697->irq_gpio));
	/* haptic start */
	aw8697_haptic_start(aw8697);
	pm_qos_add_request(&pm_qos_req_vb, PM_QOS_CPU_DMA_LATENCY, PM_QOS_VALUE_VB);
	while (1) {
		if (!aw8697_haptic_rtp_get_fifo_afi(aw8697)) {
			pr_info("%s !aw8697_haptic_rtp_get_fifo_afi done aw8697->rtp_cnt= %d \n", __func__, aw8697->rtp_cnt);
			mutex_lock(&aw8697->rtp_lock);
			if ((aw8697_rtp->len - aw8697->rtp_cnt) <
			    (aw8697->ram.base_addr >> 2))
				buf_len = aw8697_rtp->len - aw8697->rtp_cnt;
			else
				buf_len = (aw8697->ram.base_addr >> 2);

			if (aw8697->rtp_cnt != aw8697_rtp->len) {
				if (aw8697->timeval_flags == 1) {
					ktime_get_real_ts64(&aw8697->start);
					aw8697->timeval_flags = 0;
				}
				aw8697->rtpupdate_flag =
				    aw8697_i2c_writes(aw8697,
						AW8697_REG_RTP_DATA,
						&aw8697_rtp->data[aw8697->
						rtp_cnt], buf_len);
				aw8697->rtp_cnt += buf_len;
			}
			mutex_unlock(&aw8697->rtp_lock);
		}

		osc_int_state = aw8697_haptic_osc_read_int(aw8697);
		if (osc_int_state&AW8697_BIT_SYSINT_DONEI) {
			ktime_get_real_ts64(&aw8697->end);
			pr_info("%s vincent playback done aw8697->rtp_cnt= %d \n", __func__, aw8697->rtp_cnt);
			break;
		}

		ktime_get_real_ts64(&aw8697->end);
		aw8697->microsecond = (aw8697->end.tv_sec - aw8697->start.tv_sec)*1000000 +
					(aw8697->end.tv_nsec - aw8697->start.tv_nsec)/1000;
		if (aw8697->microsecond > OSC_CALIBRATION_T_LENGTH) {
			pr_info("%s vincent time out aw8697->rtp_cnt %d osc_int_state %02x\n", __func__, aw8697->rtp_cnt, osc_int_state);
			break;
		}
	}
	pm_qos_remove_request(&pm_qos_req_vb);
	enable_irq(gpio_to_irq(aw8697->irq_gpio));

	aw8697->osc_cali_flag = 0;
	aw8697->microsecond = (aw8697->end.tv_sec - aw8697->start.tv_sec)*1000000 +
				(aw8697->end.tv_nsec - aw8697->start.tv_nsec)/1000;
	/*calibration osc*/
	pr_info("%s awinic_microsecond:%ld \n", __func__, aw8697->microsecond);
	pr_info("%s exit\n", __func__);
	return 0;
}

static void aw8697_op_clean_status(struct aw8697 *aw8697)
{
	aw8697->audio_ready = false;
	aw8697->haptic_ready = false;
	aw8697->pre_haptic_number = false;
	aw8697->rtp_routine_on = 0;
	pr_info("%s enter\n", __func__);
}
//ASUS_BSP: load rtp bin for request_firmware() cannot read user space in workqueue
int aw8697_load_rtp_file(void)
{
	const struct firmware *rtp_file;
	int ret = -1;
	int Tc = 0;
	//struct aw8697 *aw8697 = container_of(work, struct aw8697, rtp_work);
    struct aw8697 *aw8697 = g_aw8697;

	pr_info("%s enter\n", __func__);
	mutex_lock(&aw8697->rtp_lock);
	//aw8697->rtp_routine_on = 1;
	/* fw loaded */
	#if 1 //ASUS_BSP
		//sprintf(fwName, "awinic_%d.bin", aw8697->rtp_file_num);
//60000~60100 (0.bin~100.bin) 60101~64999 (101.bin~4999.bin)	65000~69999 (5000.bin~9999.bin)
//70000~70100 (0.bin~100.bin) 70101~74999 (10101.bin~14999.bin)	75000~79999 (15000.bin~19999.bin)
//80000~80100 (0.bin~100.bin) 80101~849990 (20101.bin~24999.bin)	85000~89999 (25000.bin~29999.bin)
		if ((aw8697->rtp_file_num > 4999) && (aw8697->rtp_file_num < 10000))
			Tc = 1 ;
		else if ((aw8697->rtp_file_num > 14999) && (aw8697->rtp_file_num < 20000))
			Tc = 1 ;
		else if ((aw8697->rtp_file_num > 24999) && (aw8697->rtp_file_num < 30000))
			Tc = 1 ;

		if ( Tc )
			sprintf(fwName, "%d.bin", aw8697->rtp_file_num);
		else
			sprintf(fwName, "%s_%d.bin", rtp_type, aw8697->rtp_file_num);
		ret = request_firmware(&rtp_file,
		fwName,
		aw8697->dev);
	if (ret < 0) {
		pr_err("%s: failed to read %s\n", __func__, fwName);
		mutex_unlock(&aw8697->rtp_lock);
		return -1;
	}
	#else //ori
	ret = request_firmware(&rtp_file,
		aw8697_rtp_name[aw8697->rtp_file_num],
		aw8697->dev);
	if (ret < 0) {
		pr_err("%s: failed to read %s\n", __func__,
			aw8697_rtp_name[aw8697->rtp_file_num]);
		mutex_unlock(&aw8697->rtp_lock);
		return;
	}
	#endif
	aw8697->rtp_init = 0;
	vfree(aw8697_rtp);
	aw8697_rtp = vmalloc(rtp_file->size+sizeof(int));
	if (!aw8697_rtp) {
		release_firmware(rtp_file);
		pr_err("%s: error allocating memory\n", __func__);
		mutex_unlock(&aw8697->rtp_lock);
		return -1;
	}
	
	if(strcmp(rtp_type, "rog2") == 0) {
		u32 test_size;
		test_size = (u32)rtp_file->data[1]<<24|(u32)rtp_file->data[2]<<16|(u32)rtp_file->data[3]<<8|(u32)rtp_file->data[4]<<0;
		pr_info("%s rog2_size=%d\n", __func__, test_size);

		aw8697_rtp->len =(int) test_size;
	} else
		aw8697_rtp->len = rtp_file->size;
	#if 1 //ASUS_BSP
	pr_info("%s: rtp file [%s] size = %d\n", __func__, fwName, aw8697_rtp->len);
	#else //ori
	pr_info("%s: rtp file [%s] size = %d\n", __func__,
		aw8697_rtp_name[aw8697->rtp_file_num], aw8697_rtp->len);
	#endif
	if(strcmp(rtp_type, "rog2") == 0)
		memcpy(aw8697_rtp->data, rtp_file->data+5, rtp_file->size-5);
	else
		memcpy(aw8697_rtp->data, rtp_file->data, rtp_file->size);

	mutex_unlock(&aw8697->rtp_lock);
	release_firmware(rtp_file);
	return 0;
}
//

static void aw8697_rtp_work_routine(struct work_struct *work)
{
	struct aw8697 *aw8697 = g_aw8697;
	aw8697->rtp_routine_on = 1;

	mutex_lock(&aw8697->lock);

	aw8697->rtp_init = 1;

	aw8697_haptic_upload_lra(aw8697, AW8697_HAPTIC_RTP_CALI_LRA);

	/* gain */
	aw8697_haptic_ram_vbat_comp(aw8697, false);

	/* rtp mode config */
	aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_RTP_MODE);

	/* haptic start */
	aw8697_haptic_start(aw8697);

	aw8697_haptic_rtp_init(aw8697);

	mutex_unlock(&aw8697->lock);
}


/*****************************************************
 *
 * haptic - audio
 *
 *****************************************************/
static int aw8697_haptic_audio_ctr_list_insert(struct haptic_audio
				*haptic_audio, struct haptic_ctr *haptic_ctr)
{
	struct haptic_ctr *p_new = NULL;

	p_new =
	    (struct haptic_ctr *)kzalloc(sizeof(struct haptic_ctr), GFP_KERNEL);
	if (p_new == NULL) {
		pr_err("%s: kzalloc memory fail\n", __func__);
		return -1;
	}
	/* update new list info */
	p_new->cnt = haptic_ctr->cnt;
	p_new->cmd = haptic_ctr->cmd;
	p_new->play = haptic_ctr->play;
	p_new->wavseq = haptic_ctr->wavseq;
	p_new->loop = haptic_ctr->loop;
	p_new->gain = haptic_ctr->gain;

	INIT_LIST_HEAD(&(p_new->list));
	list_add(&(p_new->list), &(haptic_audio->ctr_list));

	return 0;
}

static int aw8697_haptic_audio_ctr_list_clear(struct haptic_audio *haptic_audio)
{
	struct haptic_ctr *p_ctr = NULL;
	struct haptic_ctr *p_ctr_bak = NULL;

	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
					 &(haptic_audio->ctr_list), list) {
		list_del(&p_ctr->list);
		kfree(p_ctr);
	}

	return 0;
}

static int aw8697_haptic_audio_init(struct aw8697 *aw8697)
{

	/*   pr_info("%s enter\n", __func__); */

	aw8697_haptic_set_wav_seq(aw8697, 0x01, 0x00);
	/* aw8697->haptic_audio.ori_gain = reg_val & 0xFF; */

	return 0;
}

static int aw8697_haptic_audio_off(struct aw8697 *aw8697)
{
	pr_info("%s enter\n", __func__);

	mutex_lock(&aw8697->lock);
	aw8697_haptic_set_gain(aw8697, 0x80);
	aw8697_haptic_stop(aw8697);
	aw8697_haptic_audio_ctr_list_clear(&aw8697->haptic_audio);
	mutex_unlock(&aw8697->lock);

	return 0;
}

static enum hrtimer_restart aw8697_haptic_audio_timer_func(struct hrtimer
							   *timer)
{
	struct aw8697 *aw8697 =
	    container_of(timer, struct aw8697, haptic_audio.timer);

	//pr_info("%s enter\n", __func__);
	schedule_work(&aw8697->haptic_audio.work);

	hrtimer_start(&aw8697->haptic_audio.timer,
		      ktime_set(aw8697->haptic_audio.timer_val / 1000000,
				(aw8697->haptic_audio.timer_val % 1000000) *
				1000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static void aw8697_haptic_audio_work_routine(struct work_struct *work)
{
	struct aw8697 *aw8697 =
	    container_of(work, struct aw8697, haptic_audio.work);
	struct haptic_audio *haptic_audio = NULL;
	struct haptic_ctr *p_ctr = NULL;
	struct haptic_ctr *p_ctr_bak = NULL;
	unsigned int ctr_list_flag = 0;
	unsigned int ctr_list_input_cnt = 0;
	unsigned int ctr_list_output_cnt = 0;
	unsigned int ctr_list_diff_cnt = 0;
	unsigned int ctr_list_del_cnt = 0;

#ifdef RTP_MONITOR_ENABLE
	int rtp_is_going_on = 0;
#endif

	//pr_info("%s enter\n", __func__);

	haptic_audio = &(aw8697->haptic_audio);
	mutex_lock(&aw8697->haptic_audio.lock);
	memset(&aw8697->haptic_audio.ctr, 0, sizeof(struct haptic_ctr));
	ctr_list_flag = 0;
	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
					 &(haptic_audio->ctr_list), list) {
		ctr_list_flag = 1;
		break;
	}
	//if (ctr_list_flag == 0)
		//pr_info("%s: ctr list empty\n", __func__);
	if (ctr_list_flag == 1) {
		list_for_each_entry_safe(p_ctr, p_ctr_bak,
					 &(haptic_audio->ctr_list), list) {
			ctr_list_input_cnt = p_ctr->cnt;
			break;
		}
		list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
						 &(haptic_audio->ctr_list),
						 list) {
			ctr_list_output_cnt = p_ctr->cnt;
			break;
		}
		if (ctr_list_input_cnt > ctr_list_output_cnt) {
			ctr_list_diff_cnt =
			    ctr_list_input_cnt - ctr_list_output_cnt;
		}
		if (ctr_list_input_cnt < ctr_list_output_cnt) {
			ctr_list_diff_cnt =
			    32 + ctr_list_input_cnt - ctr_list_output_cnt;
		}
		if (ctr_list_diff_cnt > 2) {
			list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
							 &(haptic_audio->
							   ctr_list), list) {
				if ((p_ctr->play == 0)
				    && (AW8697_HAPTIC_CMD_ENABLE ==
					(AW8697_HAPTIC_CMD_HAPTIC & p_ctr->
					 cmd))) {
					list_del(&p_ctr->list);
					kfree(p_ctr);
					ctr_list_del_cnt++;
				}
				if (ctr_list_del_cnt == ctr_list_diff_cnt)
					break;
			}
		}

	}

	/* get the last data from list */
	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
					 &(haptic_audio->ctr_list), list) {
		aw8697->haptic_audio.ctr.cnt = p_ctr->cnt;
		aw8697->haptic_audio.ctr.cmd = p_ctr->cmd;
		aw8697->haptic_audio.ctr.play = p_ctr->play;
		aw8697->haptic_audio.ctr.wavseq = p_ctr->wavseq;
		aw8697->haptic_audio.ctr.loop = p_ctr->loop;
		aw8697->haptic_audio.ctr.gain = p_ctr->gain;
		list_del(&p_ctr->list);
		kfree(p_ctr);
		break;
	}
	if (aw8697->haptic_audio.ctr.play) {
		pr_info
		("%s: cnt=%d, cmd=%d, play=%d, wavseq=%d, loop=%d, gain=%d\n",
		 __func__, aw8697->haptic_audio.ctr.cnt,
		 aw8697->haptic_audio.ctr.cmd,
		 aw8697->haptic_audio.ctr.play,
		 aw8697->haptic_audio.ctr.wavseq,
		 aw8697->haptic_audio.ctr.loop,
		 aw8697->haptic_audio.ctr.gain);
	}

	/* rtp mode jump */
#ifdef RTP_MONITOR_ENABLE
//hupeng close it 1206 for ASUS
	rtp_is_going_on = aw8697_haptic_juge_RTP_is_going_on(aw8697);
	if (rtp_is_going_on) {
		mutex_unlock(&aw8697->haptic_audio.lock);
		return;
	}
#endif
	mutex_unlock(&aw8697->haptic_audio.lock);
	if (aw8697->haptic_audio.ctr.cmd == AW8697_HAPTIC_CMD_ENABLE) {
		if (aw8697->haptic_audio.ctr.play == AW8697_HAPTIC_PLAY_ENABLE) {
			pr_info("%s: haptic_audio_play_start\n", __func__);
			mutex_lock(&aw8697->lock);
			aw8697_haptic_stop(aw8697);
			aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_RAM_MODE);

			aw8697_haptic_set_wav_seq(aw8697, 0x00,
						  aw8697->haptic_audio.ctr.
						  wavseq);
			aw8697_haptic_set_wav_seq(aw8697, 0x01, 0x00);

			aw8697_haptic_set_wav_loop(aw8697, 0x00,
						   aw8697->haptic_audio.ctr.
						   loop);

			aw8697_haptic_set_gain(aw8697,
					       aw8697->haptic_audio.ctr.gain);

			aw8697_haptic_start(aw8697);
			mutex_unlock(&aw8697->lock);
		} else if (AW8697_HAPTIC_PLAY_STOP ==
			   aw8697->haptic_audio.ctr.play) {
			mutex_lock(&aw8697->lock);
			aw8697_haptic_stop(aw8697);
			mutex_unlock(&aw8697->lock);
		} else if (AW8697_HAPTIC_PLAY_GAIN ==
			   aw8697->haptic_audio.ctr.play) {
			mutex_lock(&aw8697->lock);
			aw8697_haptic_set_gain(aw8697,
					       aw8697->haptic_audio.ctr.gain);
			mutex_unlock(&aw8697->lock);
		}
	}
}

/*****************************************************
 *
 * haptic cont
 *
 *****************************************************/
static int aw8697_haptic_cont(struct aw8697 *aw8697)
{
	pr_info("%s enter\n", __func__);

	/* work mode */
	aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_CONT_MODE);

	/* preset f0 */
	aw8697_haptic_set_f0_preset(aw8697);

	/* lpf */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_DATCTRL,
			      AW8697_BIT_DATCTRL_FC_MASK,
			      AW8697_BIT_DATCTRL_FC_1000HZ);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_DATCTRL,
			      AW8697_BIT_DATCTRL_LPF_ENABLE_MASK,
			      AW8697_BIT_DATCTRL_LPF_ENABLE);

	/* cont config */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_ZC_DETEC_MASK,
			      AW8697_BIT_CONT_CTRL_ZC_DETEC_ENABLE);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_WAIT_PERIOD_MASK,
			      AW8697_BIT_CONT_CTRL_WAIT_1PERIOD);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_MODE_MASK,
			      AW8697_BIT_CONT_CTRL_BY_GO_SIGNAL);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_EN_CLOSE_MASK,
			      AW8697_CONT_PLAYBACK_MODE);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_F0_DETECT_MASK,
			      AW8697_BIT_CONT_CTRL_F0_DETECT_DISABLE);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_O2C_MASK,
			      AW8697_BIT_CONT_CTRL_O2C_DISABLE);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_AUTO_BRK_MASK,
			      AW8697_BIT_CONT_CTRL_AUTO_BRK_ENABLE);

	/* TD time */
	aw8697_i2c_write(aw8697, AW8697_REG_TD_H,
			 (unsigned char)(aw8697->info.cont_td >> 8));
	aw8697_i2c_write(aw8697, AW8697_REG_TD_L,
			 (unsigned char)(aw8697->info.cont_td >> 0));
	aw8697_i2c_write(aw8697, AW8697_REG_TSET, aw8697->info.tset);

	/* zero cross */
	aw8697_i2c_write(aw8697, AW8697_REG_ZC_THRSH_H,
			 (unsigned char)(aw8697->info.cont_zc_thr >> 8));
	aw8697_i2c_write(aw8697, AW8697_REG_ZC_THRSH_L,
			 (unsigned char)(aw8697->info.cont_zc_thr >> 0));

	aw8697_i2c_write_bits(aw8697, AW8697_REG_BEMF_NUM,
			      AW8697_BIT_BEMF_NUM_BRK_MASK,
			      aw8697->info.cont_num_brk);
	/*  35*171us=5.985ms */
	aw8697_i2c_write(aw8697, AW8697_REG_TIME_NZC, 0x23);
	/* f0 driver level */
	aw8697_i2c_write(aw8697, AW8697_REG_DRV_LVL, aw8697->info.cont_drv_lvl);
	aw8697_i2c_write(aw8697, AW8697_REG_DRV_LVL_OV,
			 aw8697->info.cont_drv_lvl_ov);

	/* cont play go */
	aw8697_haptic_play_go(aw8697, true);

	return 0;
}

/*****************************************************
 *
 * haptic f0 cali
 *
 *****************************************************/
static int aw8697_haptic_get_f0(struct aw8697 *aw8697)
{
	int ret = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	unsigned char f0_pre_num = 0;
	unsigned char f0_wait_num = 0;
	unsigned char f0_repeat_num = 0;
	unsigned char f0_trace_num = 0;
	unsigned int t_f0_ms = 0;
	unsigned int t_f0_trace_ms = 0;
	unsigned int f0_cali_cnt = 50;

	pr_info("%s enter\n", __func__);

	aw8697->f0 = aw8697->info.f0_pre;

	/* f0 calibrate work mode */
	aw8697_haptic_stop(aw8697);
	aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA, 0x00);
	aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_CONT_MODE);

	//ASUS_BSP: if different with init sample rate, cannot wait AW8697_REG_SYSINT 0x01 done
	aw8697_haptic_set_pwm(aw8697, gAW_RTP_PWM);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_EN_CLOSE_MASK,
			      AW8697_BIT_CONT_CTRL_OPEN_PLAYBACK);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_F0_DETECT_MASK,
			      AW8697_BIT_CONT_CTRL_F0_DETECT_ENABLE);

	/* LPF */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_DATCTRL,
			      AW8697_BIT_DATCTRL_FC_MASK,
			      AW8697_BIT_DATCTRL_FC_1000HZ);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_DATCTRL,
			      AW8697_BIT_DATCTRL_LPF_ENABLE_MASK,
			      AW8697_BIT_DATCTRL_LPF_ENABLE);

	/* LRA OSC Source */
	if (aw8697->f0_cali_flag == AW8697_HAPTIC_CALI_F0) {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_ANACTRL,
				      AW8697_BIT_ANACTRL_LRA_SRC_MASK,
				      AW8697_BIT_ANACTRL_LRA_SRC_REG);
	} else {
		aw8697_i2c_write_bits(aw8697, AW8697_REG_ANACTRL,
				      AW8697_BIT_ANACTRL_LRA_SRC_MASK,
				      AW8697_BIT_ANACTRL_LRA_SRC_EFUSE);
	}

	/* preset f0 */
	aw8697_haptic_set_f0_preset(aw8697);

	/* f0 driver level */
	aw8697_i2c_write(aw8697, AW8697_REG_DRV_LVL, aw8697->info.cont_drv_lvl);

	/* f0 trace parameter */
	f0_pre_num = aw8697->info.f0_trace_parameter[0];
	f0_wait_num = aw8697->info.f0_trace_parameter[1];
	f0_repeat_num = aw8697->info.f0_trace_parameter[2];
	f0_trace_num = aw8697->info.f0_trace_parameter[3];
	aw8697_i2c_write(aw8697, AW8697_REG_NUM_F0_1,
			 (f0_pre_num << 4) | (f0_wait_num << 0));
	aw8697_i2c_write(aw8697, AW8697_REG_NUM_F0_2, (f0_repeat_num << 0));
	aw8697_i2c_write(aw8697, AW8697_REG_NUM_F0_3, (f0_trace_num << 0));

	/* clear aw8697 interrupt */
	ret = aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);

	/* play go and start f0 calibration */
	aw8697_haptic_play_go(aw8697, true);

	/* f0 trace time */
	t_f0_ms = 1000 * 10 / aw8697->info.f0_pre;
	t_f0_trace_ms =
	    t_f0_ms * (f0_pre_num + f0_wait_num +
		       (f0_trace_num + f0_wait_num) * (f0_repeat_num - 1));
	usleep_range(t_f0_trace_ms * 1000, t_f0_trace_ms * 1000 + 500);

	for (i = 0; i < f0_cali_cnt; i++) {
		usleep_range(2000, 2500);
		ret = aw8697_i2c_read(aw8697, AW8697_REG_GLB_STATE, &reg_val);
		/* f0 calibrate done */
		if ((reg_val & 0x0F) == 0x00) {
			aw8697_haptic_read_f0(aw8697);
			aw8697_haptic_read_beme(aw8697);
			break;
		}
		usleep_range(10000, 10500);
		pr_info("%s f0 cali sleep 10ms\n", __func__);
	}

	if (i == f0_cali_cnt)
		ret = -1;
	else
		ret = 0;

	/* restore default config */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_EN_CLOSE_MASK,
			      AW8697_CONT_PLAYBACK_MODE);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_F0_DETECT_MASK,
			      AW8697_BIT_CONT_CTRL_F0_DETECT_DISABLE);

	return ret;
}

static int aw8697_haptic_f0_calibration(struct aw8697 *aw8697)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_limit = 0;
	char f0_cali_lra = 0;
	int f0_cali_step = 0;

	pr_info("%s enter\n", __func__);

	aw8697->f0_cali_flag = AW8697_HAPTIC_CALI_F0;

	if (aw8697_haptic_get_f0(aw8697)) {
		pr_err("%s get f0 error, use defafult f0\n", __func__);
	} else {
		/* max and min limit */
		f0_limit = aw8697->f0;
		if (aw8697->f0 * 100 <
		    aw8697->info.f0_pre * (100 - aw8697->info.f0_cali_percen)) {
			f0_limit = aw8697->info.f0_pre;
		}
		if (aw8697->f0 * 100 >
		    aw8697->info.f0_pre * (100 + aw8697->info.f0_cali_percen)) {
			f0_limit = aw8697->info.f0_pre;
		}

		/* calculate cali step */
		f0_cali_step =
		    100000 * ((int)f0_limit -
			      (int)aw8697->info.f0_pre) / ((int)f0_limit * 25);
		pr_info("%s  line=%d f0_cali_step=%d\n", __func__, __LINE__,
		       f0_cali_step);
		pr_info("%s line=%d  f0_limit=%d\n", __func__, __LINE__,
		       (int)f0_limit);
		pr_info("%s line=%d  aw8697->info.f0_pre=%d\n", __func__,
		       __LINE__, (int)aw8697->info.f0_pre);

		if (f0_cali_step >= 0) {	/*f0_cali_step >= 0 */
			if (f0_cali_step % 10 >= 5)
				f0_cali_step = f0_cali_step / 10 + 1 + 32;
			else
				f0_cali_step = f0_cali_step / 10 + 32;
		} else {	/*f0_cali_step < 0 */
			if (f0_cali_step % 10 <= -5)
				f0_cali_step = 32 + (f0_cali_step / 10 - 1);
			else
				f0_cali_step = 32 + f0_cali_step / 10;
		}

		if (f0_cali_step > 31)
			f0_cali_lra = (char)f0_cali_step - 32;
		else
			f0_cali_lra = (char)f0_cali_step + 32;
		pr_info("%s f0_cali_lra=%d\n", __func__, (int)f0_cali_lra);

		aw8697->f0_calib_data = (int)f0_cali_lra;
		printk("%s f0_cali_lra=%d\n", __func__, (int)f0_cali_lra);

		//ASUS_BSP: save cali value
		aw8697_write_cali_to_file(aw8697->f0_calib_data, 0, 0);

		/* update cali step */
		aw8697_i2c_write(aw8697, AW8697_REG_TRIM_LRA,
				 (char)f0_cali_lra);
		aw8697_i2c_read(aw8697, AW8697_REG_TRIM_LRA, &reg_val);
		pr_info("%s final trim_lra=0x%02x\n", __func__, reg_val);
	}

	/* restore default work mode */
	aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_STANDBY_MODE);
	aw8697->play_mode = AW8697_HAPTIC_RAM_MODE;
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_PLAY_MODE_MASK,
			      AW8697_BIT_SYSCTRL_PLAY_MODE_RAM);
	aw8697_haptic_stop(aw8697);

	return ret;
}

/*****************************************************
 *
 * haptic fops
 *
 *****************************************************/
static int aw8697_file_open(struct inode *inode, struct file *file)
{
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	file->private_data = (void *)g_aw8697;

	return 0;
}

static int aw8697_file_release(struct inode *inode, struct file *file)
{
	file->private_data = (void *)NULL;

	module_put(THIS_MODULE);

	return 0;
}

static long aw8697_file_unlocked_ioctl(struct file *file, unsigned int cmd,
				       unsigned long arg)
{
	struct aw8697 *aw8697 = (struct aw8697 *)file->private_data;

	int ret = 0;

	dev_info(aw8697->dev, "%s: cmd=0x%x, arg=0x%lx\n", __func__, cmd, arg);

	mutex_lock(&aw8697->lock);

	if (_IOC_TYPE(cmd) != AW8697_HAPTIC_IOCTL_MAGIC) {
		dev_err(aw8697->dev, "%s: cmd magic err\n", __func__);
		mutex_unlock(&aw8697->lock);
		return -EINVAL;
	}

	switch (cmd) {
	default:
		dev_err(aw8697->dev, "%s, unknown cmd\n", __func__);
		break;
	}

	mutex_unlock(&aw8697->lock);

	return ret;
}

static ssize_t aw8697_file_read(struct file *filp, char *buff, size_t len,
				loff_t *offset)
{
	struct aw8697 *aw8697 = (struct aw8697 *)filp->private_data;
	int ret = 0;
	int i = 0;
	unsigned char reg_val = 0;
	unsigned char *pbuff = NULL;

	mutex_lock(&aw8697->lock);

	dev_info(aw8697->dev, "%s: len=%zu\n", __func__, len);

	switch (aw8697->fileops.cmd) {
	case AW8697_HAPTIC_CMD_READ_REG:
		pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
		if (pbuff != NULL) {
			for (i = 0; i < len; i++) {
				aw8697_i2c_read(aw8697, aw8697->fileops.reg + i,
						&reg_val);
				pbuff[i] = reg_val;
			}
			for (i = 0; i < len; i++) {
				dev_info(aw8697->dev, "%s: pbuff[%d]=0x%02x\n",
					 __func__, i, pbuff[i]);
			}
			ret = copy_to_user(buff, pbuff, len);
			if (ret) {
				dev_err(aw8697->dev, "%s: copy to user fail\n",
					__func__);
			}
			kfree(pbuff);
		} else {
			dev_err(aw8697->dev, "%s: alloc memory fail\n",
				__func__);
		}
		break;
	default:
		dev_err(aw8697->dev, "%s, unknown cmd %d\n", __func__,
			aw8697->fileops.cmd);
		break;
	}

	mutex_unlock(&aw8697->lock);

	return len;
}

static ssize_t aw8697_file_write(struct file *filp, const char *buff,
				 size_t len, loff_t *off)
{
	struct aw8697 *aw8697 = (struct aw8697 *)filp->private_data;
	int i = 0;
	int ret = 0;
	unsigned char *pbuff = NULL;

	pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
	if (pbuff == NULL) {
		dev_err(aw8697->dev, "%s: alloc memory fail\n", __func__);
		return len;
	}
	ret = copy_from_user(pbuff, buff, len);
	if (ret) {
		kfree(pbuff);
		dev_err(aw8697->dev, "%s: copy from user fail\n", __func__);
		return len;
	}

	for (i = 0; i < len; i++) {
		dev_info(aw8697->dev, "%s: pbuff[%d]=0x%02x\n",
			 __func__, i, pbuff[i]);
	}

	mutex_lock(&aw8697->lock);

	aw8697->fileops.cmd = pbuff[0];

	switch (aw8697->fileops.cmd) {
	case AW8697_HAPTIC_CMD_READ_REG:
		if (len == 2) {
			aw8697->fileops.reg = pbuff[1];
		} else {
			dev_err(aw8697->dev, "%s: read cmd len %zu err\n",
				__func__, len);
		}
		break;
	case AW8697_HAPTIC_CMD_WRITE_REG:
		if (len > 2) {
			for (i = 0; i < len - 2; i++) {
				dev_info(aw8697->dev,
					 "%s: write reg0x%02x=0x%02x\n",
					 __func__, pbuff[1] + i, pbuff[i + 2]);
				aw8697_i2c_write(aw8697, pbuff[1] + i,
						 pbuff[2 + i]);
			}
		} else {
			dev_err(aw8697->dev, "%s: write cmd len %zu err\n",
				__func__, len);
		}
		break;
	default:
		dev_err(aw8697->dev, "%s, unknown cmd %d\n", __func__,
			aw8697->fileops.cmd);
		break;
	}

	mutex_unlock(&aw8697->lock);

	if (pbuff != NULL)
		kfree(pbuff);
	return len;
}

static const struct file_operations aw8697_fops = {
	.owner = THIS_MODULE,
	.read = aw8697_file_read,
	.write = aw8697_file_write,
	.unlocked_ioctl = aw8697_file_unlocked_ioctl,
	.open = aw8697_file_open,
	.release = aw8697_file_release,
};

static struct miscdevice aw8697_haptic_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AW8697_HAPTIC_NAME,
	.fops = &aw8697_fops,
};

static int aw8697_haptic_init(struct aw8697 *aw8697)
{
	int ret = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	unsigned char bemf_config = 0;

	pr_info("%s enter\n", __func__);

	ret = misc_register(&aw8697_haptic_misc);
	if (ret) {
		dev_err(aw8697->dev, "%s: misc fail: %d\n", __func__, ret);
		return ret;
	}

	/* haptic audio */
	aw8697->haptic_audio.delay_val = 1;
	aw8697->haptic_audio.timer_val = 21318;
	INIT_LIST_HEAD(&(aw8697->haptic_audio.ctr_list));

	hrtimer_init(&aw8697->haptic_audio.timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	aw8697->haptic_audio.timer.function = aw8697_haptic_audio_timer_func;
	INIT_WORK(&aw8697->haptic_audio.work, aw8697_haptic_audio_work_routine);

	mutex_init(&aw8697->haptic_audio.lock);

	INIT_LIST_HEAD(&(aw8697->haptic_audio.list));
	aw8697_op_clean_status(aw8697);
	/* haptic init */
	mutex_lock(&aw8697->lock);

	aw8697->activate_mode = aw8697->info.mode;

	ret = aw8697_i2c_read(aw8697, AW8697_REG_WAVSEQ1, &reg_val);
	aw8697->index = reg_val & 0x7F;
	ret = aw8697_i2c_read(aw8697, AW8697_REG_DATDBG, &reg_val);
	aw8697->gain = reg_val & 0xFF;

	aw8697->gain_trig = 0x7F;//ASUS_BSP: init gain for trig mode

	ret = aw8697_i2c_read(aw8697, AW8697_REG_BSTDBG4, &reg_val);
	aw8697->vmax = (reg_val >> 1) & 0x1F;
	for (i = 0; i < AW8697_SEQUENCER_SIZE; i++) {
		ret = aw8697_i2c_read(aw8697, AW8697_REG_WAVSEQ1 + i, &reg_val);
		aw8697->seq[i] = reg_val;
	}

	aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_STANDBY_MODE);

	//aw8697_haptic_set_pwm(aw8697, AW8697_PWM_48K);
	aw8697_haptic_set_pwm(aw8697, gAW_RTP_PWM);
	//aw8697_haptic_set_pwm(aw8697, AW8697_PWM_12K);



	aw8697_i2c_write(aw8697, AW8697_REG_BSTDBG1, aw8697->info.bstdbg[0]);
	aw8697_i2c_write(aw8697, AW8697_REG_BSTDBG2, aw8697->info.bstdbg[1]);
	aw8697_i2c_write(aw8697, AW8697_REG_BSTDBG3, aw8697->info.bstdbg[2]);
	aw8697_i2c_write(aw8697, AW8697_REG_TSET, aw8697->info.tset);
	aw8697_i2c_write(aw8697, AW8697_REG_R_SPARE, aw8697->info.r_spare);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_ANADBG,
			      AW8697_BIT_ANADBG_IOC_MASK,
			      AW8697_BIT_ANADBG_IOC_4P65A);

	aw8697_haptic_set_bst_peak_cur(aw8697, AW8697_DEFAULT_PEAKCUR);

	aw8697_haptic_swicth_motorprotect_config(aw8697, 0x00, 0x00);

	aw8697_haptic_auto_boost_config(aw8697, false);

	aw8697_haptic_trig_param_init(aw8697);
	aw8697_haptic_trig_param_config(aw8697);

	aw8697_haptic_offset_calibration(aw8697);

	/* vbat compensation */
	aw8697_haptic_cont_vbat_mode(aw8697,
				     AW8697_HAPTIC_CONT_VBAT_HW_COMP_MODE);
	aw8697->ram_vbat_comp = AW8697_HAPTIC_RAM_VBAT_COMP_ENABLE;

	mutex_unlock(&aw8697->lock);

	/* f0 calibration */
	mutex_lock(&aw8697->lock);
	{
	//aw8697_haptic_f0_calibration(aw8697);
	unsigned int cali=0;
	if(aw8697_get_cali_from_file(&cali, 0, 0) >= 0){
		if (cali == 0x7777)
			pr_err("%s get cali value fail\n", __func__);
		else {
			aw8697->f0_calib_data=cali;
			aw8697_haptic_upload_lra(aw8697, AW8697_HAPTIC_F0_CALI_LRA);
		}
	}
	cali=0;
	if(aw8697_get_cali_from_file(&cali, 0, 1) >= 0){
		if (cali == 0x7777)
			pr_err("%s get rtp cali value fail\n", __func__);
		else {
			aw8697->lra_calib_data=cali;
			aw8697_haptic_upload_lra(aw8697, AW8697_HAPTIC_RTP_CALI_LRA);
		}
	}
	}
	mutex_unlock(&aw8697->lock);

	/* beme config */
	bemf_config = aw8697->info.bemf_config[0];
	aw8697_i2c_write(aw8697, AW8697_REG_BEMF_VTHH_H, bemf_config);
	bemf_config = aw8697->info.bemf_config[1];
	aw8697_i2c_write(aw8697, AW8697_REG_BEMF_VTHH_L, bemf_config);
	bemf_config = aw8697->info.bemf_config[2];
	aw8697_i2c_write(aw8697, AW8697_REG_BEMF_VTHL_H, bemf_config);
	bemf_config = aw8697->info.bemf_config[3];
	aw8697_i2c_write(aw8697, AW8697_REG_BEMF_VTHL_L, bemf_config);
	return ret;
}

/*****************************************************
 *
 * vibrator
 *
 *****************************************************/
#ifdef TIMED_OUTPUT
static int aw8697_vibrator_get_time(struct timed_output_dev *dev)
{
	struct aw8697 *aw8697 = container_of(dev, struct aw8697, to_dev);

	if (hrtimer_active(&aw8697->timer)) {
		ktime_t r = hrtimer_get_remaining(&aw8697->timer);

		return ktime_to_ms(r);
	}

	return 0;
}

static void aw8697_vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct aw8697 *aw8697 = container_of(dev, struct aw8697, to_dev);

	mutex_lock(&aw8697->lock);

	pr_info("%s enter\n", __func__);

	aw8697_haptic_stop(aw8697);

	if (value > 0) {
		aw8697_haptic_ram_vbat_comp(aw8697, false);
		aw8697_haptic_play_wav_seq(aw8697, value);
	}

	mutex_unlock(&aw8697->lock);

	pr_info("%s exit\n", __func__);
}

#else
static enum led_brightness aw8697_haptic_brightness_get(struct led_classdev
							*cdev)
{
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);

	return aw8697->amplitude;
}

static void aw8697_haptic_brightness_set(struct led_classdev *cdev,
					 enum led_brightness level)
{
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);

	if (!aw8697->ram_init)
		return;
	if (aw8697->ramupdate_flag < 0)
		return;
	aw8697->amplitude = level;

	mutex_lock(&aw8697->lock);

	aw8697_haptic_stop(aw8697);
	if (aw8697->amplitude > 0) {
		aw8697_haptic_ram_vbat_comp(aw8697, false);
		aw8697_haptic_play_wav_seq(aw8697, aw8697->amplitude);
	}

	mutex_unlock(&aw8697->lock);

}
#endif

static ssize_t aw8697_state_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

	return snprintf(buf, PAGE_SIZE, "%d\n", aw8697->state);
}

static ssize_t aw8697_state_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	return count;
}

static ssize_t aw8697_duration_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ktime_t time_rem;
	s64 time_ms = 0;

	if (hrtimer_active(&aw8697->timer)) {
		time_rem = hrtimer_get_remaining(&aw8697->timer);
		time_ms = ktime_to_ms(time_rem);
	}

	return snprintf(buf, PAGE_SIZE, "%lld\n", time_ms);
}

static ssize_t aw8697_duration_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	/* setting 0 on duration is NOP for now */
	if (val <= 0)
		return count;

	aw8697->duration = val;

	return count;
}

static ssize_t aw8697_activate_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

	/* For now nothing to show */
	return snprintf(buf, PAGE_SIZE, "%d\n", aw8697->state);
}

static ssize_t aw8697_activate_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_info("%s: value=%d\n", __func__, val);

	mutex_lock(&aw8697->lock);
	hrtimer_cancel(&aw8697->timer);

	aw8697->state = val;

	mutex_unlock(&aw8697->lock);
	schedule_work(&aw8697->vibrator_work);

	return count;
}

static ssize_t aw8697_activate_mode_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

	return snprintf(buf, PAGE_SIZE, "activate_mode=%d\n",
			aw8697->activate_mode);
}

static ssize_t aw8697_activate_mode_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw8697->lock);
	aw8697->activate_mode = val;
	mutex_unlock(&aw8697->lock);
	return count;
}

static ssize_t aw8697_index_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned char reg_val = 0;

	aw8697_i2c_read(aw8697, AW8697_REG_WAVSEQ1, &reg_val);
	aw8697->index = reg_val;

	return snprintf(buf, PAGE_SIZE, "%d\n", aw8697->index);
}

static ssize_t aw8697_index_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_info("%s: value=%d\n", __func__, val);

	mutex_lock(&aw8697->lock);
	aw8697->index = val;
	aw8697_haptic_set_repeat_wav_seq(aw8697, aw8697->index);
	mutex_unlock(&aw8697->lock);
	return count;
}

static ssize_t aw8697_vmax_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw8697->vmax);
}

static ssize_t aw8697_vmax_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_info("%s: value=%d\n", __func__, val);

	mutex_lock(&aw8697->lock);
	aw8697->vmax = val;
	aw8697_haptic_set_bst_vol(aw8697, aw8697->vmax);
	mutex_unlock(&aw8697->lock);
	return count;
}

static ssize_t aw8697_gain_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw8697->gain);
}

static ssize_t aw8697_gain_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_info("%s: value=%d\n", __func__, val);

	mutex_lock(&aw8697->lock);

	cancel_delayed_work(&aw8697->gain_work);

	aw8697->gain = val;
	aw8697_haptic_set_gain(aw8697, aw8697->gain);
	mutex_unlock(&aw8697->lock);
	return count;
}

static ssize_t aw8697_gain_trig_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw8697->gain_trig);
}

static ssize_t aw8697_gain_trig_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_info("%s: value=%d\n", __func__, val);

	mutex_lock(&aw8697->lock);

	cancel_delayed_work(&aw8697->gain_work);

	aw8697->gain_trig = val;
	aw8697->gain = aw8697->gain_trig;
	aw8697_haptic_set_gain(aw8697, aw8697->gain_trig);
	mutex_unlock(&aw8697->lock);
	return count;
}


static ssize_t aw8697_seq_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	size_t count = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW8697_SEQUENCER_SIZE; i++) {
		aw8697_i2c_read(aw8697, AW8697_REG_WAVSEQ1 + i, &reg_val);
		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d: 0x%02x\n", i + 1, reg_val);
		aw8697->seq[i] |= reg_val;
	}
	return count;
}

static ssize_t aw8697_seq_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		pr_info("%s: seq%d=0x%x\n", __func__, databuf[0],
			 databuf[1]);
		mutex_lock(&aw8697->lock);
		aw8697->seq[databuf[0]] = (unsigned char)databuf[1];
		aw8697_haptic_set_wav_seq(aw8697, (unsigned char)databuf[0],
					  aw8697->seq[databuf[0]]);
		mutex_unlock(&aw8697->lock);
	}
	return count;
}

static ssize_t aw8697_loop_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	size_t count = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW8697_SEQUENCER_LOOP_SIZE; i++) {
		aw8697_i2c_read(aw8697, AW8697_REG_WAVLOOP1 + i, &reg_val);
		aw8697->loop[i * 2 + 0] = (reg_val >> 4) & 0x0F;
		aw8697->loop[i * 2 + 1] = (reg_val >> 0) & 0x0F;

		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d loop: 0x%02x\n", i * 2 + 1,
				  aw8697->loop[i * 2 + 0]);
		count +=
		    snprintf(buf + count, PAGE_SIZE - count,
			     "seq%d loop: 0x%02x\n", i * 2 + 2,
			     aw8697->loop[i * 2 + 1]);
	}
	return count;
}

static ssize_t aw8697_loop_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		pr_info("%s: seq%d loop=0x%x\n", __func__, databuf[0],
			 databuf[1]);
		mutex_lock(&aw8697->lock);
		aw8697->loop[databuf[0]] = (unsigned char)databuf[1];
		aw8697_haptic_set_wav_loop(aw8697, (unsigned char)databuf[0],
					   aw8697->loop[databuf[0]]);
		mutex_unlock(&aw8697->lock);
	}

	return count;
}

static ssize_t aw8697_reg_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW8697_REG_MAX; i++) {
		if (!(aw8697_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw8697_i2c_read(aw8697, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%02x\n", i, reg_val);
	}
	return len;
}

static ssize_t aw8697_reg_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw8697_i2c_write(aw8697, (unsigned char)databuf[0],
				 (unsigned char)databuf[1]);
	}

	return count;
}

static ssize_t aw8697_rtp_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "rtp play: %d\n",
			aw8697->rtp_cnt);

	return len;
}

//20200106 add--- mem_play_show
static ssize_t aw8697_mem_play_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	size_t count = 0;
	unsigned char reg_val = 0;

	aw8697_i2c_read(aw8697, AW8697_REG_WAVSEQ1, &reg_val);
	count += snprintf(buf, PAGE_SIZE,	"seq%d: 0x%02x\n", 1, reg_val);
	pr_info("%s: waveform id == %d\n", __func__, reg_val);

	return count;
}
//20200106 add--- end mem_play_show

//20200106 add---  aw8697_rtp_play_show
static ssize_t aw8697_rtp_play_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	//struct led_classdev *cdev = dev_get_drvdata(dev);
	//struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	#if 1 //ASUS_BSP
	len += snprintf(buf + len, PAGE_SIZE - len, "fwName:[%s] %s\n",
			rtp_type, fwName);
	#else //ori
	len += snprintf(buf + len, PAGE_SIZE - len, "aw8697->rtp_file_num: %d\n",
			aw8697->rtp_file_num);
	#endif
	return len;
}
//20200106 add--- end aw8697_rtp_play_show

static ssize_t aw8697_rtp_play_repeat_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	//struct led_classdev *cdev = dev_get_drvdata(dev);
	//struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	#if 1 //ASUS_BSP
	len += snprintf(buf + len, PAGE_SIZE - len, "fwName:[%s] %s  repeat: %d\n",
			rtp_type, fwName, rtp_repeat);
	#else //ori
	len += snprintf(buf + len, PAGE_SIZE - len, "aw8697->rtp_file_num: %d\n",
			aw8697->rtp_file_num);
	#endif
	return len;
}

//20200106 add---  aw8697_rtp_play_break_show
static ssize_t aw8697_rtp_play_break_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "value:[%s] %d\n",
			rtp_type, aw8697->rtp_file_num);

	return len;
}
//20200106 add--- end aw8697_rtp_play_break_show
#define AUDIO_READY_STATUS  1024
#define FACTORY_MODE_NORMAL_RTP_NUMBER  73
#define FACTORY_MODE_HIGH_TEMP_RTP_NUMBER  72

//20200106 add--- mem_play_store
static ssize_t aw8697_mem_play_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0) {
		pr_info("%s: kstrtouint fail\n", __func__);
		return rc;
	}
	pr_info("%s: waveform id == %d\n", __func__, val);

	mutex_lock(&aw8697->lock);
	if(val == 0) {
		pr_err("%s: waveform id sould not set to 0 in current function!\n", __func__);
		goto mem_play_store_brk1;
	}	

	cancel_delayed_work(&aw8697->rtp_work);
	cancel_delayed_work(&aw8697->gain_work);

	aw8697_haptic_set_wav_seq(aw8697, 0, (unsigned char)val);
	aw8697_haptic_set_wav_seq(aw8697, 1, 0);
	aw8697_haptic_set_wav_loop(aw8697, 0, 0);
/*
    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
                    AW8697_BIT_SYSCTRL_PLAY_MODE_MASK, AW8697_BIT_SYSCTRL_PLAY_MODE_RAM);	
    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
            AW8697_BIT_SYSCTRL_WORK_MODE_MASK, AW8697_BIT_SYSCTRL_ACTIVE);
    aw8697_interrupt_clear(aw8697);
    aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
            AW8697_BIT_SYSINTM_UVLO_MASK, AW8697_BIT_SYSINTM_UVLO_EN);	
	aw8697_haptic_play_go(aw8697, true);
*/	
	aw8697_haptic_play_wav_seq(aw8697, 1);

	//ASUS_BSP: rtp data transfer complete, set gain for trig mode
	schedule_delayed_work(&aw8697->gain_work, msecs_to_jiffies(100));

mem_play_store_brk1:
	mutex_unlock(&aw8697->lock);
	return count;
}
//20200106 add--- end mem_play_store

static ssize_t aw8697_rtp_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0) {
		pr_info("%s: kstrtouint fail\n", __func__);
		return rc;
	}
	pr_info("%s: rtp[%d]\n", __func__, val);

	mutex_lock(&aw8697->lock);
	aw8697_haptic_stop(aw8697);

	aw8697_haptic_set_rtp_aei(aw8697, false);
	aw8697_interrupt_clear(aw8697);
	if (val < (sizeof(aw8697_rtp_name)/AW8697_RTP_NAME_MAX)) {
		aw8697->rtp_file_num = val;

		if (val)
			schedule_delayed_work(&aw8697->rtp_work, msecs_to_jiffies(0));
		else
			pr_err("%s: rtp_file_num 0x%02x over max value\n", __func__,
		       aw8697->rtp_file_num);
	}
	mutex_unlock(&aw8697->lock);
	return count;
}

static int aw8697_rtp_play_store_base(struct device *dev,
				 unsigned int num, const char *type, unsigned int repeat, unsigned int delay_ms)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

	int rc = 0;
	int out_pwm = gAW_RTP_PWM;
	pr_info("%s: rtp: [%s][%d], repeat=%d, delay=%d)\n", __func__, type, num, repeat, delay_ms);

	if((num <= 0) || (repeat <= 0)){
		pr_err("%s: invalid file number or repeat !\n", __func__);
		return rc;
	}

	mutex_lock(&aw8697->lock);

	cancel_delayed_work(&aw8697->rtp_work);
	cancel_delayed_work(&aw8697->gain_work);
	aw8697_haptic_stop(aw8697);
	aw8697_haptic_set_rtp_aei(aw8697, false);
	aw8697_interrupt_clear(aw8697);

	rtp_repeat = repeat;
	sprintf(rtp_type, "%s", type);
	//pr_info("%s rtp_type=%s\n",__func__, rtp_type);
	aw8697->rtp_file_num = num;

	if(strcmp(rtp_type, "rog2") == 0)
		out_pwm = gROG2_RTP_PWM;

	aw8697_haptic_set_pwm(aw8697, out_pwm);

	rc = aw8697_load_rtp_file();
	if (rc == 0)
		schedule_delayed_work(&aw8697->rtp_work, msecs_to_jiffies(delay_ms));

	mutex_unlock(&aw8697->lock);
	return rc;
}

static ssize_t aw8697_rtp_play_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	unsigned int val = 0;
	int rc = 0;

	if(!buf) return -1;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0) {
		pr_info("%s: kstrtouint fail\n", __func__);
		return rc;
	}

	aw8697_rtp_play_store_base( dev, val, "awinic", 1, 0);
	return count;
}

static ssize_t aw8697_rtp_play_repeat_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	unsigned int val = 0;
	int rc = 0;
	int repeat=0;

	if(!buf) return -1;

	rc = sscanf(buf, "%d %d", &val, &repeat);
	if ( rc != 2 ) {
		pr_info("%s: invalid input! rc=%d \n", __func__, rc);
		return count;
	}

	aw8697_rtp_play_store_base( dev, val, "awinic", repeat, 0);
	return count;
}

static ssize_t aw8697_rtp_play_delay_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	unsigned int val = 0;
	int rc = 0;
	unsigned int delay=0;

	if(!buf) return -1;

	rc = sscanf(buf, "%d %d", &val, &delay);
	if ( rc != 2 ) {
		pr_info("%s: invalid input! rc=%d \n", __func__, rc);
		return count;
	}

	aw8697_rtp_play_store_base( dev, val, "awinic", 1, delay);
	return count;
}

static ssize_t aw8697_rog2_demo_rtp_play_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	unsigned int val = 0;
	int rc = 0;

	if(!buf) return -1;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0) {
		pr_info("%s: kstrtouint fail\n", __func__);
		return rc;
	}

	aw8697_rtp_play_store_base( dev, val, "rog2", 1, 0);
	return count;
}



//20200106 add--- aw8697_rtp_play_break_store
static ssize_t aw8697_rtp_play_break_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;
	
	rc = kstrtouint(buf, 0, &val);
	if (rc < 0) {
		pr_info("%s: kstrtouint fail\n", __func__);
		return rc;
	}
	pr_info("%s: val == %d\n", __func__, val);

	mutex_lock(&aw8697->lock);
	if(val) {
		pr_err("%s: val sould set to 0 in current function!\n", __func__);
		goto rtp_play_break_store_brk1;
	}
	cancel_delayed_work(&aw8697->rtp_work);
	aw8697_haptic_stop(aw8697);
	aw8697_haptic_set_rtp_aei(aw8697, false);
	aw8697_interrupt_clear(aw8697);
	aw8697->rtp_file_num = val;
	sprintf(rtp_type, "");
	rtp_repeat = 0;
	schedule_delayed_work(&aw8697->gain_work, msecs_to_jiffies(1));
	pr_info("%s: rtp play is broken\n", __func__);
rtp_play_break_store_brk1:
	mutex_unlock(&aw8697->lock);
	return count;
}
//20200106 add--- end aw8697_rtp_play_break_store

static ssize_t aw8697_ram_update_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	/* struct timed_output_dev *to_dev = dev_get_drvdata(dev); */
	/*struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);*/
#else
	/* struct led_classdev *cdev = dev_get_drvdata(dev); */
	/* struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev); */
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "sram update mode\n");
	return len;
}

static ssize_t aw8697_ram_update_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val)
		aw8697_ram_update(aw8697);
	return count;
}

static ssize_t aw8697_f0_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	mutex_lock(&aw8697->lock);
	aw8697->f0_cali_flag = AW8697_HAPTIC_LRA_F0;
	aw8697_haptic_get_f0(aw8697);
	mutex_unlock(&aw8697->lock);
	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "aw8697 lra f0 = %d\n",
		     aw8697->f0);
	return len;
}

static ssize_t aw8697_f0_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
#ifdef TIMED_OUTPUT
	/* struct timed_output_dev *to_dev = dev_get_drvdata(dev); */
	/*struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);*/
#else
	/* struct led_classdev *cdev = dev_get_drvdata(dev); */
	/* struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev); */
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t aw8697_cali_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	mutex_lock(&aw8697->lock);
	aw8697->f0_cali_flag = AW8697_HAPTIC_CALI_F0;
	aw8697_haptic_get_f0(aw8697);
	mutex_unlock(&aw8697->lock);
	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "aw8697 cali f0 = %d\n",
		     aw8697->f0);
	return len;
}

static ssize_t aw8697_cali_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val) {
		mutex_lock(&aw8697->lock);
		aw8697_haptic_f0_calibration(aw8697);
		mutex_unlock(&aw8697->lock);
	}
	return count;
}

static ssize_t aw8697_load_cali_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int cali=0;
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if(val!=1)
		return count;

	if(aw8697_get_cali_from_file(&cali, 0, 0) >= 0 ) {
		if(cali == 0x7777) {
			pr_err("%s get cali value fail\n", __func__);
		}else {
			aw8697->f0_calib_data=cali;
			aw8697_haptic_upload_lra(aw8697, AW8697_HAPTIC_F0_CALI_LRA);
		}
	}

	cali=0;
	if(aw8697_get_cali_from_file(&cali, 0, 1) >= 0){
		if (cali == 0x7777)
			pr_err("%s get rtp cali value fail\n", __func__);
		else {
			aw8697->lra_calib_data=cali;
			aw8697_haptic_upload_lra(aw8697, AW8697_HAPTIC_RTP_CALI_LRA);
		}
	}

	return count;
}



static ssize_t aw8697_load_cali_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	//struct led_classdev *cdev = dev_get_drvdata(dev);
	//struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	unsigned int cali=0;
	unsigned int rtp_cali=0;

	if(aw8697_get_cali_from_file(&cali, 0, 0) >= 0 ) {
		if(cali == 0x7777) {
			pr_err("%s get cali value fail", __func__);
			len += snprintf(buf + len, PAGE_SIZE - len, 
						"get cali value fail");
		}else {
			len += snprintf(buf + len, PAGE_SIZE - len, 
						"aw8697 cali = %d", cali);
		}
	}else {
		len += snprintf(buf + len, PAGE_SIZE - len, 
						"get cali file fail");
	}
	len += snprintf(buf + len, PAGE_SIZE - len, 
						"      ");
	if(aw8697_get_cali_from_file(&rtp_cali, 0, 1) >= 0 ) {
		if(rtp_cali == 0x7777) {
			pr_err("%s get rtp cali value fail\n", __func__);
			len += snprintf(buf + len, PAGE_SIZE - len, 
						"get rtp cali value fail\n");
		}else {
			len += snprintf(buf + len, PAGE_SIZE - len, 
						"aw8697 rtp cali = %d\n", rtp_cali);
		}
	}else{
		len += snprintf(buf + len, PAGE_SIZE - len, 
						"get rtp cali file fail\n");
	}

	return len;
}


static ssize_t aw8697_cont_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	aw8697_haptic_read_cont_f0(aw8697);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"aw8697 cont f0 = %d\n",
			aw8697->cont_f0);
	return len;
}

static ssize_t aw8697_cont_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	aw8697_haptic_stop(aw8697);
	if (val)
		aw8697_haptic_cont(aw8697);
	return count;
}

static ssize_t aw8697_cont_td_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"aw8697 cont delay time = 0x%04x\n",
			aw8697->info.cont_td);
	return len;
}

static ssize_t aw8697_cont_td_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[1] = { 0 };

	if (sscanf(buf, "%x", &databuf[0]) == 1) {
		aw8697->info.cont_td = databuf[0];
		aw8697_i2c_write(aw8697, AW8697_REG_TD_H,
				 (unsigned char)(databuf[0] >> 8));
		aw8697_i2c_write(aw8697, AW8697_REG_TD_L,
				 (unsigned char)(databuf[0] >> 0));
	}
	return count;
}

static ssize_t aw8697_cont_drv_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"aw8697 cont drv level = %d\n",
			aw8697->info.cont_drv_lvl);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"aw8697 cont drv level overdrive= %d\n",
			aw8697->info.cont_drv_lvl_ov);
	return len;
}

static ssize_t aw8697_cont_drv_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%d %d", &databuf[0], &databuf[1]) == 2) {
		aw8697->info.cont_drv_lvl = databuf[0];
		aw8697_i2c_write(aw8697, AW8697_REG_DRV_LVL,
				 aw8697->info.cont_drv_lvl);
		aw8697->info.cont_drv_lvl_ov = databuf[1];
		aw8697_i2c_write(aw8697, AW8697_REG_DRV_LVL_OV,
				 aw8697->info.cont_drv_lvl_ov);
	}
	return count;
}

static ssize_t aw8697_cont_num_brk_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"aw8697 cont break num = %d\n",
			aw8697->info.cont_num_brk);
	return len;
}

static ssize_t aw8697_cont_num_brk_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[1] = { 0 };

	if (sscanf(buf, "%d", &databuf[0]) == 1) {
		aw8697->info.cont_num_brk = databuf[0];
		if (aw8697->info.cont_num_brk > 7)
			aw8697->info.cont_num_brk = 7;
		aw8697_i2c_write_bits(aw8697, AW8697_REG_BEMF_NUM,
				      AW8697_BIT_BEMF_NUM_BRK_MASK,
				      aw8697->info.cont_num_brk);
	}
	return count;
}

static ssize_t aw8697_cont_zc_thr_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"aw8697 cont zero cross thr = 0x%04x\n",
			aw8697->info.cont_zc_thr);
	return len;
}

static ssize_t aw8697_cont_zc_thr_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[1] = { 0 };

	if (sscanf(buf, "%x", &databuf[0]) == 1) {
		aw8697->info.cont_zc_thr = databuf[0];
		aw8697_i2c_write(aw8697, AW8697_REG_ZC_THRSH_H,
				 (unsigned char)(databuf[0] >> 8));
		aw8697_i2c_write(aw8697, AW8697_REG_ZC_THRSH_L,
				 (unsigned char)(databuf[0] >> 0));
	}
	return count;
}

static ssize_t aw8697_vbat_monitor_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	mutex_lock(&aw8697->lock);
	aw8697_haptic_stop(aw8697);
	aw8697_haptic_get_vbat(aw8697);
	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "vbat=%dmV\n", aw8697->vbat);
	mutex_unlock(&aw8697->lock);

	return len;
}

static ssize_t aw8697_vbat_monitor_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	return count;
}

static ssize_t aw8697_lra_resistance_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	unsigned char reg_val = 0;

	mutex_lock(&aw8697->lock);
	aw8697_haptic_stop(aw8697);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8697_BIT_SYSCTRL_RAMINIT_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_BST_MODE_MASK,
			      AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_ANACTRL,
			      AW8697_BIT_ANACTRL_HD_PD_MASK,
			      AW8697_BIT_ANACTRL_HD_HZ_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_D2SCFG,
			      AW8697_BIT_D2SCFG_CLK_ADC_MASK,
			      AW8697_BIT_D2SCFG_CLK_ASC_1P5MHZ);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
			      AW8697_BIT_DETCTRL_RL_OS_MASK,
			      AW8697_BIT_DETCTRL_RL_DETECT);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_DETCTRL,
			      AW8697_BIT_DETCTRL_DIAG_GO_MASK,
			      AW8697_BIT_DETCTRL_DIAG_GO_ENABLE);
	usleep_range(3000, 3500);
	aw8697_i2c_read(aw8697, AW8697_REG_RLDET, &reg_val);
	aw8697->lra = 298 * reg_val;
	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "r_lra=%dmohm\n", aw8697->lra);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_ANACTRL,
			      AW8697_BIT_ANACTRL_HD_PD_MASK,
			      AW8697_BIT_ANACTRL_HD_PD_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_D2SCFG,
			      AW8697_BIT_D2SCFG_CLK_ADC_MASK,
			      AW8697_BIT_D2SCFG_CLK_ASC_6MHZ);

	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8697_BIT_SYSCTRL_RAMINIT_OFF);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_BST_MODE_MASK,
			      AW8697_BIT_SYSCTRL_BST_MODE_BOOST);
	mutex_unlock(&aw8697->lock);

	return len;
}

static ssize_t aw8697_lra_resistance_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	return count;
}

static ssize_t aw8697_auto_boost_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "auto_boost=%d\n",
		     aw8697->auto_boost);

	return len;
}

static ssize_t aw8697_auto_boost_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw8697->lock);
	aw8697_haptic_stop(aw8697);
	aw8697_haptic_auto_boost_config(aw8697, val);
	mutex_unlock(&aw8697->lock);

	return count;
}

static ssize_t aw8697_prctmode_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	unsigned char reg_val = 0;

	aw8697_i2c_read(aw8697, AW8697_REG_RLDET, &reg_val);

	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "prctmode=%d\n",
		     reg_val & 0x20);
	return len;
}

static ssize_t aw8697_prctmode_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[2] = { 0, 0 };
	unsigned int addr = 0;
	unsigned int val = 0;

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		addr = databuf[0];
		val = databuf[1];
		mutex_lock(&aw8697->lock);
		aw8697_haptic_swicth_motorprotect_config(aw8697, addr, val);
		mutex_unlock(&aw8697->lock);
	}
	return count;
}
//ASUS_BSP: export trig control to air trigger
int aw8697_trig_control(int num, bool enable)
{
	struct aw8697 *aw8697 = g_aw8697;
	int ret=0;

	if(aw8697 == NULL){
		pr_info("%s aw8697 is null\n", __func__);
		ret=-1;
	} else if (aw8697->ram_init == 0){
		pr_info("%s aw8697 ram init is not complete\n", __func__);
		ret=-1;
	} else
		pr_info("%s num=%d enable=%d\n", __func__, num, enable);

	if (ret)
		return ret;

	switch(num){
	case 1:
		if(enable)
			aw8697->trig[0].enable = AW8697_TRG1_ENABLE;
		else
			aw8697->trig[0].enable = AW8697_TRG1_DISABLE;
		break;
	case 2:
		if(enable)
			aw8697->trig[1].enable = AW8697_TRG2_ENABLE;
		else
			aw8697->trig[1].enable = AW8697_TRG2_DISABLE;
		break;
	case 3:
		if(enable)
			aw8697->trig[2].enable = AW8697_TRG3_ENABLE;
		else
			aw8697->trig[2].enable = AW8697_TRG3_DISABLE;
		break;
	default:
		pr_info("%s num=%d is not support\n", __func__);
		ret=-1;
		break;
	}

	aw8697_haptic_trig_enable_config(aw8697);

	return ret;
}
EXPORT_SYMBOL(aw8697_trig_control);

//ASUS_BSP: add for air trigger vibrator intensity
static ssize_t aw8697_trigger_level_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int channel=0, trigger=0, level=0;
	struct aw8697 *aw8697 = g_aw8697;

	sscanf(buf, "%d %d %d", &channel, &trigger, &level);
	pr_info("%s enter. channel=%d trig=%d level=%d\n", __func__, channel, trigger, level);

	switch(trigger){
		case 1:
		case 2:
			switch(level){
				case 0:
				case 1:
				case 2:
				case 3:
					pr_info("%s set trigger %d to level %d\n", __func__, trigger, level);
					pr_info("%s trig_level[%d][%d]=%d\n", __func__, (trigger-1), level, trig_level[trigger-1][level]);
					aw8697->trig[trigger-1].frist_seq = trig_level[trigger-1][level];
					aw8697_haptic_trig_param_config(aw8697);
					//aw8697_haptic_trig_enable_config(aw8697);
					break;
				default:
					pr_info("%s  trigger %d is not support level %d\n", __func__, trigger, level);
					break;
				}
			break;
		default:
			pr_info("%s Not support trigger %d\n", __func__, trigger);
			break;
	}
	return count;
}

static ssize_t aw8697_trigger_level_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
struct aw8697 *aw8697 = g_aw8697;

	ssize_t len = 0;

len += snprintf(buf + len, PAGE_SIZE - len,
  "triger 1: waveform %d \n",
  aw8697->trig[0].frist_seq);
len += snprintf(buf + len, PAGE_SIZE - len,
  "triger 2: waveform %d \n",
  aw8697->trig[1].frist_seq);

	return len;
}

//

static ssize_t aw8697_trig_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	unsigned char i = 0;

	for (i = 0; i < AW8697_TRIG_NUM; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"trig%d: enable=%d, default_level=%d, dual_edge=%d, frist_seq=%d, second_seq=%d\n",
				i + 1, aw8697->trig[i].enable,
				aw8697->trig[i].default_level,
				aw8697->trig[i].dual_edge,
				aw8697->trig[i].frist_seq,
				aw8697->trig[i].second_seq);
	}

	return len;
}

static ssize_t aw8697_trig_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif

	unsigned int databuf[6] = { 0 };

	if (sscanf(buf, "%d %d %d %d %d %d",
		   &databuf[0], &databuf[1], &databuf[2], &databuf[3],
		   &databuf[4], &databuf[5])) {
		pr_info("%s: %d, %d, %d, %d, %d, %d\n", __func__, databuf[0],
			 databuf[1], databuf[2], databuf[3], databuf[4],
			 databuf[5]);
		if (databuf[0] > 3)
			databuf[0] = 3;
		if (databuf[0] > 0)
			databuf[0] -= 1;
		aw8697->trig[databuf[0]].enable = databuf[1];
		aw8697->trig[databuf[0]].default_level = databuf[2];
		aw8697->trig[databuf[0]].dual_edge = databuf[3];
		aw8697->trig[databuf[0]].frist_seq = databuf[4];
		aw8697->trig[databuf[0]].second_seq = databuf[5];
		mutex_lock(&aw8697->lock);
		aw8697_haptic_trig_param_config(aw8697);
		aw8697_haptic_trig_enable_config(aw8697);
		mutex_unlock(&aw8697->lock);
	}
	return count;
}

static ssize_t aw8697_ram_vbat_comp_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "ram_vbat_comp=%d\n",
		     aw8697->ram_vbat_comp);

	return len;
}

static ssize_t aw8697_ram_vbat_comp_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw8697->lock);
	if (val)
		aw8697->ram_vbat_comp = AW8697_HAPTIC_RAM_VBAT_COMP_ENABLE;
	else
		aw8697->ram_vbat_comp = AW8697_HAPTIC_RAM_VBAT_COMP_DISABLE;
	mutex_unlock(&aw8697->lock);

	return count;
}

static ssize_t aw8697_osc_cali_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct aw8697 *aw8697 = dev_get_drvdata(dev);
	ssize_t len = 0;

	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "lra_calib_data=%d\n",
		     aw8697->lra_calib_data);

	return len;
}
static ssize_t aw8697_osc_cali_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;

	int rc = 0;
	rc = kstrtouint(buf, 0, &val);
	if (rc < 0) {
	return rc;
	}
	mutex_lock(&aw8697->lock);
	/* osc calibration flag start,Other behaviors are forbidden */
	aw8697->osc_cali_run = 1;
	if (val == 1) {
		aw8697_rtp_osc_calibration(aw8697);
		aw8697_rtp_trim_lra_calibration(aw8697);
	}
	aw8697->osc_cali_run = 0;
	/* osc calibration flag end,Other behaviors are permitted */
	mutex_unlock(&aw8697->lock);

	return count;
}

static ssize_t aw8697_haptic_audio_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n",
			aw8697->haptic_audio.ctr.cnt);
	return len;
}

static ssize_t aw8697_haptic_audio_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[6] = { 0 };
	struct haptic_ctr *hap_ctr = NULL;

	if (!aw8697->ram_init)
		return count;
	if (6 ==
	    sscanf(buf, "%d %d %d %d %d %d", &databuf[0], &databuf[1],
		   &databuf[2], &databuf[3], &databuf[4], &databuf[5])) {
		if (databuf[2]) {
			pr_info
			("%s: cnt=%d, cmd=%d, play=%d, wavseq=%d, loop=%d, gain=%d\n",
			 __func__, databuf[0], databuf[1], databuf[2],
			 databuf[3], databuf[4], databuf[5]);
		}

		hap_ctr =
		    (struct haptic_ctr *)kzalloc(sizeof(struct haptic_ctr),
						 GFP_KERNEL);
		if (hap_ctr == NULL) {
			pr_err("%s: kzalloc memory fail\n", __func__);
			return count;
		}
		mutex_lock(&aw8697->haptic_audio.lock);
		hap_ctr->cnt = (unsigned char)databuf[0];
		hap_ctr->cmd = (unsigned char)databuf[1];
		hap_ctr->play = (unsigned char)databuf[2];
		hap_ctr->wavseq = (unsigned char)databuf[3];
		hap_ctr->loop = (unsigned char)databuf[4];
		hap_ctr->gain = (unsigned char)databuf[5];
		aw8697_haptic_audio_ctr_list_insert(&aw8697->haptic_audio,
						    hap_ctr);

		if (hap_ctr->cmd == 0xff) {
			pr_info("%s: haptic_audio stop\n", __func__);
			if (hrtimer_active(&aw8697->haptic_audio.timer)) {
				pr_info("%s: cancel haptic_audio_timer\n",
					__func__);
				hrtimer_cancel(&aw8697->haptic_audio.timer);
				aw8697->haptic_audio.ctr.cnt = 0;
				aw8697_haptic_audio_off(aw8697);
			}
		} else {
			if (hrtimer_active(&aw8697->haptic_audio.timer)) {
			} else {
				pr_info("%s: start haptic_audio_timer\n",
					__func__);
				aw8697_haptic_audio_init(aw8697);
				hrtimer_start(&aw8697->haptic_audio.timer,
					      ktime_set(aw8697->haptic_audio.
							delay_val / 1000000,
							(aw8697->haptic_audio.
							 delay_val % 1000000) *
							1000),
					      HRTIMER_MODE_REL);
			}
		}
		mutex_unlock(&aw8697->haptic_audio.lock);
		kfree(hap_ctr);
	}
	return count;
}

static ssize_t aw8697_haptic_audio_time_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"haptic_audio.delay_val=%dus\n",
			aw8697->haptic_audio.delay_val);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"haptic_audio.timer_val=%dus\n",
			aw8697->haptic_audio.timer_val);
	return len;
}

static ssize_t aw8697_haptic_audio_time_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697 = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[2] = { 0 };

	if (sscanf(buf, "%d %d", &databuf[0], &databuf[1]) == 2) {
		aw8697->haptic_audio.delay_val = databuf[0];
		aw8697->haptic_audio.timer_val = databuf[1];
	}
	return count;
}

static DEVICE_ATTR(state, S_IWUSR | S_IRUGO, aw8697_state_show,
		   aw8697_state_store);
static DEVICE_ATTR(duration, S_IWUSR | S_IRUGO, aw8697_duration_show,
		   aw8697_duration_store);
static DEVICE_ATTR(activate, S_IWUSR | S_IRUGO, aw8697_activate_show,
		   aw8697_activate_store);
static DEVICE_ATTR(activate_mode, S_IWUSR | S_IRUGO, aw8697_activate_mode_show,
		   aw8697_activate_mode_store);
static DEVICE_ATTR(index, S_IWUSR | S_IRUGO, aw8697_index_show,
		   aw8697_index_store);
static DEVICE_ATTR(vmax, S_IWUSR | S_IRUGO, aw8697_vmax_show,
		   aw8697_vmax_store);
static DEVICE_ATTR(gain, S_IWUSR | S_IRUGO, aw8697_gain_show,
		   aw8697_gain_store);

static DEVICE_ATTR(gain_trig, S_IWUSR | S_IRUGO, aw8697_gain_trig_show,
		   aw8697_gain_trig_store);

static DEVICE_ATTR(seq, S_IWUSR | S_IRUGO, aw8697_seq_show, aw8697_seq_store);
static DEVICE_ATTR(loop, S_IWUSR | S_IRUGO, aw8697_loop_show,
		   aw8697_loop_store);
static DEVICE_ATTR(register, S_IWUSR | S_IRUGO, aw8697_reg_show,
		   aw8697_reg_store);

static DEVICE_ATTR(mem_play, S_IWUSR | S_IRUGO, aw8697_mem_play_show, aw8697_mem_play_store);
static DEVICE_ATTR(rtp, S_IWUSR | S_IRUGO, aw8697_rtp_show, aw8697_rtp_store);

static DEVICE_ATTR(rtp_play, S_IWUSR | S_IRUGO, aw8697_rtp_play_show, aw8697_rtp_play_store);
static DEVICE_ATTR(rtp_play_repeat, S_IWUSR | S_IRUGO, aw8697_rtp_play_repeat_show, aw8697_rtp_play_repeat_store);
static DEVICE_ATTR(rtp_play_delay, S_IWUSR | S_IRUGO, NULL, aw8697_rtp_play_delay_store);
static DEVICE_ATTR(rtp_play_break, S_IWUSR | S_IRUGO, aw8697_rtp_play_break_show, aw8697_rtp_play_break_store);

static DEVICE_ATTR(rog2_demo_rtp_play, S_IWUSR | S_IRUGO, NULL, aw8697_rog2_demo_rtp_play_store);

static DEVICE_ATTR(ram_update, S_IWUSR | S_IRUGO, aw8697_ram_update_show,
		   aw8697_ram_update_store);
static DEVICE_ATTR(f0, S_IWUSR | S_IRUGO, aw8697_f0_show, aw8697_f0_store);
static DEVICE_ATTR(cali, S_IWUSR | S_IRUGO, aw8697_cali_show,
		   aw8697_cali_store);
static DEVICE_ATTR(load_cali, S_IWUSR | S_IRUGO, aw8697_load_cali_show,
		   aw8697_load_cali_store);
static DEVICE_ATTR(cont, S_IWUSR | S_IRUGO, aw8697_cont_show,
		   aw8697_cont_store);
static DEVICE_ATTR(cont_td, S_IWUSR | S_IRUGO, aw8697_cont_td_show,
		   aw8697_cont_td_store);
static DEVICE_ATTR(cont_drv, S_IWUSR | S_IRUGO, aw8697_cont_drv_show,
		   aw8697_cont_drv_store);
static DEVICE_ATTR(cont_num_brk, S_IWUSR | S_IRUGO, aw8697_cont_num_brk_show,
		   aw8697_cont_num_brk_store);
static DEVICE_ATTR(cont_zc_thr, S_IWUSR | S_IRUGO, aw8697_cont_zc_thr_show,
		   aw8697_cont_zc_thr_store);
static DEVICE_ATTR(vbat_monitor, S_IWUSR | S_IRUGO, aw8697_vbat_monitor_show,
		   aw8697_vbat_monitor_store);
static DEVICE_ATTR(lra_resistance, S_IWUSR | S_IRUGO,
		   aw8697_lra_resistance_show, aw8697_lra_resistance_store);
static DEVICE_ATTR(auto_boost, S_IWUSR | S_IRUGO, aw8697_auto_boost_show,
		   aw8697_auto_boost_store);
static DEVICE_ATTR(prctmode, S_IWUSR | S_IRUGO, aw8697_prctmode_show,
		   aw8697_prctmode_store);
static DEVICE_ATTR(trig, S_IWUSR | S_IRUGO, aw8697_trig_show,
		   aw8697_trig_store);

static DEVICE_ATTR(trigger_level, S_IWUSR | S_IRUGO, aw8697_trigger_level_show,
		   aw8697_trigger_level_store);

static DEVICE_ATTR(ram_vbat_comp, S_IWUSR | S_IRUGO, aw8697_ram_vbat_comp_show,
		   aw8697_ram_vbat_comp_store);
static DEVICE_ATTR(osc_cali, S_IWUSR | S_IRUGO, aw8697_osc_cali_show, aw8697_osc_cali_store);
static DEVICE_ATTR(haptic_audio, S_IWUSR | S_IRUGO, aw8697_haptic_audio_show,
		   aw8697_haptic_audio_store);
static DEVICE_ATTR(haptic_audio_time, S_IWUSR | S_IRUGO,
		   aw8697_haptic_audio_time_show,
		   aw8697_haptic_audio_time_store);

static struct attribute *aw8697_vibrator_attributes[] = {
	&dev_attr_state.attr,
	&dev_attr_duration.attr,
	&dev_attr_activate.attr,
	&dev_attr_activate_mode.attr,
	&dev_attr_index.attr,
	&dev_attr_vmax.attr,
	&dev_attr_gain.attr,
	&dev_attr_gain_trig.attr,
	&dev_attr_seq.attr,
	&dev_attr_loop.attr,
	&dev_attr_register.attr,
	&dev_attr_rtp.attr,
	&dev_attr_mem_play.attr,
	&dev_attr_rtp_play.attr,
	&dev_attr_rtp_play_repeat.attr,
	&dev_attr_rtp_play_delay.attr,
	&dev_attr_rtp_play_break.attr,
	&dev_attr_rog2_demo_rtp_play.attr,
	&dev_attr_ram_update.attr,
	&dev_attr_f0.attr,
	&dev_attr_cali.attr,
	&dev_attr_load_cali.attr,
	&dev_attr_cont.attr,
	&dev_attr_cont_td.attr,
	&dev_attr_cont_drv.attr,
	&dev_attr_cont_num_brk.attr,
	&dev_attr_cont_zc_thr.attr,
	&dev_attr_vbat_monitor.attr,
	&dev_attr_lra_resistance.attr,
	&dev_attr_auto_boost.attr,
	&dev_attr_prctmode.attr,
	&dev_attr_trig.attr,
	&dev_attr_trigger_level.attr,
	&dev_attr_ram_vbat_comp.attr,
	&dev_attr_osc_cali.attr,
	&dev_attr_haptic_audio.attr,
	&dev_attr_haptic_audio_time.attr,
	NULL
};

static struct attribute_group aw8697_vibrator_attribute_group = {
	.attrs = aw8697_vibrator_attributes
};

static enum hrtimer_restart aw8697_vibrator_timer_func(struct hrtimer *timer)
{
	struct aw8697 *aw8697 = container_of(timer, struct aw8697, timer);

	pr_info("%s enter\n", __func__);
	aw8697->state = 0;
	schedule_work(&aw8697->vibrator_work);

	return HRTIMER_NORESTART;
}

static void aw8697_vibrator_work_routine(struct work_struct *work)
{
	struct aw8697 *aw8697 =
	    container_of(work, struct aw8697, vibrator_work);

	pr_info("%s enter,play_mode:%d\n", __func__,aw8697->play_mode);

	mutex_lock(&aw8697->lock);

	aw8697_haptic_stop(aw8697);
	aw8697_haptic_upload_lra(aw8697, AW8697_HAPTIC_F0_CALI_LRA);
	if (aw8697->state) {
		if (aw8697->activate_mode == AW8697_HAPTIC_ACTIVATE_RAM_MODE) {
			aw8697_haptic_ram_vbat_comp(aw8697, true);
			aw8697_haptic_play_repeat_seq(aw8697, true);
		} else if (aw8697->activate_mode ==
			   AW8697_HAPTIC_ACTIVATE_CONT_MODE) {
			aw8697_haptic_cont(aw8697);
		} else {
		}
		/* run ms timer */
		hrtimer_start(&aw8697->timer,
			      ktime_set(aw8697->duration / 1000,
					(aw8697->duration % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
	if (aw8697->play_mode == 0)
	{
		pr_info("%s gain set aw8697->play_mode%d\n", __func__, aw8697->play_mode);
		schedule_delayed_work(&aw8697->gain_work, msecs_to_jiffies(100));
	}
	mutex_unlock(&aw8697->lock);
}

static int aw8697_vibrator_init(struct aw8697 *aw8697)
{
	int ret = 0;

	pr_info("%s enter\n", __func__);

#ifdef TIMED_OUTPUT
	aw8697->to_dev.name = "vibrator_aw8697";
	aw8697->to_dev.get_time = aw8697_vibrator_get_time;
	aw8697->to_dev.enable = aw8697_vibrator_enable;

	ret = timed_output_dev_register(&(aw8697->to_dev));
	if (ret < 0) {
		dev_err(aw8697->dev, "%s: fail to create timed output dev\n",
			__func__);
		return ret;
	}
	ret =
	    sysfs_create_group(&aw8697->to_dev.dev->kobj,
			       &aw8697_vibrator_attribute_group);
	if (ret < 0) {
		dev_err(aw8697->dev, "%s error creating sysfs attr files\n",
			__func__);
		return ret;
	}
#else
	aw8697->cdev.name = "vibrator";
	aw8697->cdev.brightness_get = aw8697_haptic_brightness_get;
	aw8697->cdev.brightness_set = aw8697_haptic_brightness_set;

	ret = devm_led_classdev_register(&aw8697->i2c->dev, &aw8697->cdev);
	if (ret < 0) {
		dev_err(aw8697->dev, "%s: fail to create led dev\n", __func__);
		return ret;
	}
	ret =
	    sysfs_create_group(&aw8697->cdev.dev->kobj,
			       &aw8697_vibrator_attribute_group);
	if (ret < 0) {
		dev_err(aw8697->dev, "%s error creating sysfs attr files\n",
			__func__);
		return ret;
	}
#endif
	hrtimer_init(&aw8697->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw8697->timer.function = aw8697_vibrator_timer_func;
	INIT_WORK(&aw8697->vibrator_work, aw8697_vibrator_work_routine);

	INIT_DELAYED_WORK(&aw8697->rtp_work, aw8697_rtp_work_routine);

	INIT_DELAYED_WORK(&aw8697->gain_work, aw8697_gain_routine);

	mutex_init(&aw8697->lock);
	mutex_init(&aw8697->rtp_lock);

	return 0;
}

/******************************************************
 *
 * irq
 *
 ******************************************************/
static void aw8697_interrupt_clear(struct aw8697 *aw8697)
{
	unsigned char reg_val = 0;

	//pr_info("%s enter\n", __func__);
	aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);
if((gVibDebugLog & 0x0001)== 0x0001)
	pr_info("%s: reg SYSINT=0x%x\n", __func__, reg_val);
}

static void aw8697_interrupt_setup(struct aw8697 *aw8697)
{
	unsigned char reg_val = 0;

	pr_info("%s enter\n", __func__);

	aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);
	pr_info("%s: reg SYSINT=0x%x\n", __func__, reg_val);

	/* edge int mode */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_DBGCTRL,
			      AW8697_BIT_DBGCTRL_INT_MODE_MASK,
			      AW8697_BIT_DBGCTRL_INT_MODE_EDGE);

	/* int enable */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
			      AW8697_BIT_SYSINTM_BSTERR_MASK,
			      AW8697_BIT_SYSINTM_BSTERR_OFF);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
			      AW8697_BIT_SYSINTM_OV_MASK,
			      AW8697_BIT_SYSINTM_OV_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
			      AW8697_BIT_SYSINTM_UVLO_MASK,
			      AW8697_BIT_SYSINTM_UVLO_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
			      AW8697_BIT_SYSINTM_OCD_MASK,
			      AW8697_BIT_SYSINTM_OCD_EN);
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSINTM,
			      AW8697_BIT_SYSINTM_OT_MASK,
			      AW8697_BIT_SYSINTM_OT_EN);
}

static irqreturn_t aw8697_irq(int irq, void *data)
{
	struct aw8697 *aw8697 = data;
	unsigned char reg_val = 0;
	unsigned char dbg_val = 0;
	unsigned int buf_len = 0;
	unsigned char glb_state_val = 0;
	//Daniel 20210223 modify start
	unsigned char sysst_val = 0;

	//pr_info("%s enter\n", __func__);

	aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);
	//pr_info("%s: reg SYSINT=0x%x\n", __func__, reg_val);
	aw8697_i2c_read(aw8697, AW8697_REG_DBGSTAT, &dbg_val);
	//pr_info("%s: reg DBGSTAT=0x%x\n", __func__, dbg_val);
	//pr_info("%s: reg SYSINT=0x%x DBGSTAT=0x%x\n", __func__, reg_val, dbg_val);
	aw8697_i2c_read(aw8697, AW8697_REG_SYSST, &sysst_val);

	if (reg_val & AW8697_BIT_SYSINT_OVI) {
		aw8697_op_clean_status(aw8697);
		pr_err("%s chip ov int error\n", __func__);
	}
	if ((reg_val & AW8697_BIT_SYSINT_UVLI) &&
		(sysst_val & AW8697_BIT_SYSST_UVLS)) {
		aw8697_op_clean_status(aw8697);
		pr_err("%s chip uvlo int error\n", __func__);
	}
	//Daniel 20210223 modify end
	if (reg_val & AW8697_BIT_SYSINT_OCDI) {
		aw8697_op_clean_status(aw8697);
		pr_err("%s chip over current int error\n", __func__);
	}
	if (reg_val & AW8697_BIT_SYSINT_OTI) {
		aw8697_op_clean_status(aw8697);
		pr_err("%s chip over temperature int error\n", __func__);
	}
	if (reg_val & AW8697_BIT_SYSINT_DONEI) {
		aw8697_op_clean_status(aw8697);
		pr_info("%s chip playback done\n", __func__);
	}

	if (reg_val & AW8697_BIT_SYSINT_FF_AEI) {
		if((gVibDebugLog & 0x0001)== 0x0001)
			pr_info("%s: aw8697 rtp fifo almost empty int\n", __func__);
		if (aw8697->rtp_init) {
			while ((!aw8697_haptic_rtp_get_fifo_afs(aw8697)) &&
			       (aw8697->play_mode == AW8697_HAPTIC_RTP_MODE)) {
				mutex_lock(&aw8697->rtp_lock);
				if((gVibDebugLog & 0x0001)== 0x0001)
					pr_info("%s: aw8697 rtp mode fifo update, cnt=%d\n",__func__, aw8697->rtp_cnt);
				if (!aw8697_rtp) {
					pr_info("%s:aw8697_rtp is null break\n",__func__);
					mutex_unlock(&aw8697->rtp_lock);
					break;
				}
				if ((aw8697_rtp->len - aw8697->rtp_cnt) <
				    (aw8697->ram.base_addr >> 2)) {
					buf_len =
					    aw8697_rtp->len - aw8697->rtp_cnt;
				} else {
					buf_len = (aw8697->ram.base_addr >> 2);
				}
				aw8697->rtpupdate_flag = aw8697_i2c_writes(
					aw8697, AW8697_REG_RTP_DATA,
					&aw8697_rtp->data[aw8697->rtp_cnt],
					buf_len);
				aw8697->rtp_cnt += buf_len;
				aw8697_i2c_read(aw8697, AW8697_REG_GLB_STATE, &glb_state_val);
				if ((glb_state_val & 0x0f) == 0x00) {
					aw8697_op_clean_status(aw8697);
					pr_info("%s: rtp update stop. Becase fifo is cleaned and enter standby mode.\n",__func__);
					aw8697_haptic_set_rtp_aei(aw8697,false);
					aw8697->rtp_cnt = 0;
					aw8697->rtp_init = 0;

					//ASUS_BSP: rtp data transfer stop, set gain for trig mode
					schedule_delayed_work(&aw8697->gain_work, msecs_to_jiffies(150));

					mutex_unlock(&aw8697->rtp_lock);
					break;
				}
				if (aw8697->rtp_cnt == aw8697_rtp->len) {
					//ASUS_BSP +++: add for rtp_play_repeat
					if (rtp_repeat > 0)
						rtp_repeat--;
					if (rtp_repeat){
						aw8697->rtp_cnt = 0;
						mutex_unlock(&aw8697->rtp_lock);
						continue;
					}
					//ASUS_BSP ---
					aw8697_op_clean_status(aw8697);
					pr_info("%s: rtp update complete\n",__func__);
					aw8697_haptic_set_rtp_aei(aw8697,false);
					aw8697->rtp_cnt = 0;
					aw8697->rtp_init = 0;

					//ASUS_BSP: rtp data transfer complete, set gain for trig mode
					schedule_delayed_work(&aw8697->gain_work, msecs_to_jiffies(150));

					mutex_unlock(&aw8697->rtp_lock);
					break;
				}
				mutex_unlock(&aw8697->rtp_lock);
			}
		} else {
			pr_info("%s: aw8697 rtp init = %d, init error\n",
			       __func__, aw8697->rtp_init);
		}
	}

	if (reg_val & AW8697_BIT_SYSINT_FF_AFI){
		if((gVibDebugLog & 0x0001)== 0x0001)
			pr_info("%s: aw8697 rtp mode fifo full empty\n", __func__);
	}
	if (aw8697->play_mode != AW8697_HAPTIC_RTP_MODE)
		aw8697_haptic_set_rtp_aei(aw8697, false);

	/*aw8697_i2c_read(aw8697, AW8697_REG_SYSINT, &reg_val);*/
	//pr_info("%s: reg SYSINT=0x%x\n", __func__, reg_val);
	/*aw8697_i2c_read(aw8697, AW8697_REG_SYSST, &reg_val); */
	//pr_info("%s: reg SYSST=0x%x\n", __func__, reg_val);

	//pr_info("%s exit\n", __func__);

	return IRQ_HANDLED;
}


/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw8697_parse_dt(struct device *dev, struct aw8697 *aw8697,
			   struct device_node *np)
{
	unsigned int val = 0;
	unsigned int bstdbg[6];
	unsigned int f0_trace_parameter[4];
	unsigned int bemf_config[4];
	if (HW_REV_ER <= g_ASUS_hwID) {
		aw8697->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
		if (aw8697->reset_gpio < 0) {
			dev_err(dev,
				"%s: no reset gpio provided, will not HW reset device\n",
				__func__);
			return -1;
		} else {
			dev_info(dev, "%s: reset gpio provided ok\n", __func__);
		}
	}
	aw8697->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (aw8697->irq_gpio < 0)
		dev_err(dev, "%s: no irq gpio provided.\n", __func__);
	else
		dev_info(dev, "%s: irq gpio provided ok.\n", __func__);

	val = of_property_read_u32(np, "vib_mode", &aw8697->info.mode);
	if (val != 0)
		pr_info("%s vib_mode not found\n", __func__);
	val = of_property_read_u32(np, "vib_f0_pre", &aw8697->info.f0_pre);
	if (val != 0)
		pr_info("%s vib_f0_pre not found\n", __func__);
	val =
	    of_property_read_u32(np, "vib_f0_cali_percen",
				 &aw8697->info.f0_cali_percen);
	if (val != 0)
		pr_info("%s vib_f0_cali_percen not found\n", __func__);
	val =
	    of_property_read_u32(np, "vib_cont_drv_lev",
				 &aw8697->info.cont_drv_lvl);
	if (val != 0)
		pr_info("%s vib_cont_drv_lev not found\n", __func__);
	val =
	    of_property_read_u32(np, "vib_cont_drv_lvl_ov",
				 &aw8697->info.cont_drv_lvl_ov);
	if (val != 0)
		pr_info("%s vib_cont_drv_lvl_ov not found\n", __func__);
	val = of_property_read_u32(np, "vib_cont_td", &aw8697->info.cont_td);
	if (val != 0)
		pr_info("%s vib_cont_td not found\n", __func__);
	val =
	    of_property_read_u32(np, "vib_cont_zc_thr",
				 &aw8697->info.cont_zc_thr);
	if (val != 0)
		pr_info("%s vib_cont_zc_thr not found\n", __func__);
	val =
	    of_property_read_u32(np, "vib_cont_num_brk",
				 &aw8697->info.cont_num_brk);
	if (val != 0)
		pr_info("%s vib_cont_num_brk not found\n", __func__);
	val = of_property_read_u32(np, "vib_f0_coeff", &aw8697->info.f0_coeff);
	if (val != 0)
		pr_info("%s vib_f0_coeff not found\n", __func__);

	val = of_property_read_u32(np, "vib_tset", &aw8697->info.tset);
	if (val != 0)
		pr_info("%s vib_tset not found\n", __func__);
	val = of_property_read_u32(np, "vib_r_spare", &aw8697->info.r_spare);
	if (val != 0)
		pr_info("%s vib_r_spare not found\n", __func__);
	val = of_property_read_u32_array(np, "vib_bstdbg",
					 bstdbg, ARRAY_SIZE(bstdbg));
	if (val != 0)
		pr_info("%s vib_bstdbg not found\n", __func__);
	memcpy(aw8697->info.bstdbg, bstdbg, sizeof(bstdbg));
	val = of_property_read_u32_array(np, "vib_f0_trace_parameter",
					 f0_trace_parameter,
					 ARRAY_SIZE(f0_trace_parameter));
	if (val != 0)
		pr_info("%s vib_f0_trace_parameter not found\n", __func__);
	memcpy(aw8697->info.f0_trace_parameter, f0_trace_parameter,
	       sizeof(f0_trace_parameter));
	val =
	    of_property_read_u32_array(np, "vib_bemf_config", bemf_config,
				       ARRAY_SIZE(bemf_config));
	if (val != 0)
		pr_info("%s vib_bemf_config not found\n", __func__);
	memcpy(aw8697->info.bemf_config, bemf_config, sizeof(bemf_config));

	return 0;
}

static int aw8697_hw_reset(struct aw8697 *aw8697)
{
	pr_info("%s enter\n", __func__);
	if (HW_REV_ER <= g_ASUS_hwID) {
		if (aw8697 && gpio_is_valid(aw8697->reset_gpio)) {
			gpio_set_value_cansleep(aw8697->reset_gpio, 0);
			usleep_range(1000, 2000);
			gpio_set_value_cansleep(aw8697->reset_gpio, 1);
			usleep_range(3500, 4000);
		} else {
			dev_err(aw8697->dev, "%s:  failed\n", __func__);
		}
	}
	return 0;
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int aw8697_read_chipid(struct aw8697 *aw8697)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char reg = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		/* hardware reset */
		aw8697_hw_reset(aw8697);

		ret = aw8697_i2c_read(aw8697, AW8697_REG_ID, &reg);
		if (ret < 0) {
			dev_err(aw8697->dev,
				"%s: failed to read register AW8697_REG_ID: %d\n",
				__func__, ret);
		}
		switch (reg) {
		case AW8697_CHIPID:
			pr_info("%s aw8697 detected\n", __func__);
			aw8697->chipid = AW8697_CHIPID;
			//aw8697->flags |= AW8697_FLAG_SKIP_INTERRUPTS;
			aw8697_haptic_softreset(aw8697);
			return 0;
		default:
			pr_info("%s unsupported device revision (0x%x)\n",
				__func__, reg);
			break;
		}
		cnt++;

		usleep_range(AW_READ_CHIPID_RETRY_DELAY * 1000,
			     AW_READ_CHIPID_RETRY_DELAY * 1000 + 500);
	}

	return -EINVAL;
}

/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw8697_i2c_reg_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct aw8697 *aw8697 = dev_get_drvdata(dev);

	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw8697_i2c_write(aw8697, (unsigned char)databuf[0],
				 (unsigned char)databuf[1]);
	}

	return count;
}

static ssize_t aw8697_i2c_reg_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct aw8697 *aw8697 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW8697_REG_MAX; i++) {
		if (!(aw8697_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw8697_i2c_read(aw8697, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%02x\n", i, reg_val);
	}
	return len;
}

static ssize_t aw8697_i2c_ram_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct aw8697 *aw8697 = dev_get_drvdata(dev);

	unsigned int databuf[1] = { 0 };

	if (sscanf(buf, "%x", &databuf[0]) == 1) {
		if (databuf[0] == 1)
			aw8697_ram_update(aw8697);
	}

	return count;
}

static ssize_t aw8697_i2c_ram_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct aw8697 *aw8697 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;

	aw8697_haptic_stop(aw8697);
	/* RAMINIT Enable */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8697_BIT_SYSCTRL_RAMINIT_EN);

	aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRH,
			 (unsigned char)(aw8697->ram.base_addr >> 8));
	aw8697_i2c_write(aw8697, AW8697_REG_RAMADDRL,
			 (unsigned char)(aw8697->ram.base_addr & 0x00ff));
	len += snprintf(buf + len, PAGE_SIZE - len, "aw8697_haptic_ram:\n");
	for (i = 0; i < aw8697->ram.len; i++) {
		aw8697_i2c_read(aw8697, AW8697_REG_RAMDATA, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "0x%02x,", reg_val);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	/* RAMINIT Disable */
	aw8697_i2c_write_bits(aw8697, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8697_BIT_SYSCTRL_RAMINIT_OFF);

	return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw8697_i2c_reg_show,
		   aw8697_i2c_reg_store);
static DEVICE_ATTR(ram, S_IWUSR | S_IRUGO, aw8697_i2c_ram_show,
		   aw8697_i2c_ram_store);

static struct attribute *aw8697_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_ram.attr,
	NULL
};

static struct attribute_group aw8697_attribute_group = {
	.attrs = aw8697_attributes
};

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
 static int aw8697_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct aw8697 *aw8697;
	struct device_node *np = i2c->dev.of_node;
	int irq_flags = 0;
	int ret = -1;

	pr_info("%s enter\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	aw8697 = devm_kzalloc(&i2c->dev, sizeof(struct aw8697), GFP_KERNEL);
	if (aw8697 == NULL)
		return -ENOMEM;

	aw8697->dev = &i2c->dev;
	aw8697->i2c = i2c;

	i2c_set_clientdata(i2c, aw8697);

	/* aw8697 rst & int */
	if (np) {
		ret = aw8697_parse_dt(&i2c->dev, aw8697, np);
		if (ret) {
			dev_err(&i2c->dev,
				"%s: failed to parse device tree node\n",
				__func__);
			goto err_parse_dt;
		}
	} else {
		pr_info("%s reset_gpio & irq_gpio failed\n", __func__);
		aw8697->reset_gpio = -1;
		aw8697->irq_gpio = -1;
	}
	if (HW_REV_ER <= g_ASUS_hwID) {
		if (gpio_is_valid(aw8697->reset_gpio)) {
			ret = devm_gpio_request_one(&i2c->dev, aw8697->reset_gpio,
				GPIOF_OUT_INIT_LOW, "aw8697_rst");
			if (ret) {
				dev_err(&i2c->dev, "%s: rst request failed\n",
					__func__);
				goto err_reset_gpio_request;
			} else {
				// pr_info("%s reset gpio request ok\n", __func__);
			}
		}
	}
	if (gpio_is_valid(aw8697->irq_gpio)) {
		pr_info("%s gpio_is_valid()\n", __func__);
		ret = devm_gpio_request_one(&i2c->dev, aw8697->irq_gpio,
					    GPIOF_DIR_IN, "aw8697_int");
		if (ret) {
			dev_err(&i2c->dev, "%s: int request failed\n",
				__func__);
			goto err_irq_gpio_request;
		} else {
			// pr_info("%s irq gpio request ok\n", __func__);
		}
	}

	/* aw8697 chip id */
	ret = aw8697_read_chipid(aw8697);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw8697_read_chipid failed ret=%d\n",
			__func__, ret);
		goto err_id;
	}

	/* aw8697 irq */
	if (gpio_is_valid(aw8697->irq_gpio) &&
	    !(aw8697->flags & AW8697_FLAG_SKIP_INTERRUPTS)) {
		/* register irq handler */
		aw8697_interrupt_setup(aw8697);
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
						gpio_to_irq(aw8697->irq_gpio),
						NULL, aw8697_irq, irq_flags,
						"aw8697", aw8697);
		if (ret != 0) {
			dev_err(&i2c->dev, "%s: failed to request IRQ %d: %d\n",
				__func__, gpio_to_irq(aw8697->irq_gpio), ret);
			goto err_irq;
		}
	} else {
		dev_info(&i2c->dev, "%s skipping IRQ registration\n", __func__);
		/* disable feature support if gpio was invalid */
		aw8697->flags |= AW8697_FLAG_SKIP_INTERRUPTS;
	}

	dev_set_drvdata(&i2c->dev, aw8697);

	ret = sysfs_create_group(&i2c->dev.kobj, &aw8697_attribute_group);
	if (ret < 0) {
		dev_info(&i2c->dev, "%s error creating sysfs attr files\n",
			 __func__);
		goto err_sysfs;
	}

	g_aw8697 = aw8697;

	aw8697_vibrator_init(aw8697);

	aw8697_haptic_init(aw8697);

	aw8697_ram_init(aw8697);

	pr_info("%s probe completed successfully!\n", __func__);

	return 0;

 err_sysfs:
	devm_free_irq(&i2c->dev, gpio_to_irq(aw8697->irq_gpio), aw8697);
 err_irq:
 err_id:
	if (gpio_is_valid(aw8697->irq_gpio))
		devm_gpio_free(&i2c->dev, aw8697->irq_gpio);
 err_irq_gpio_request:
	if (gpio_is_valid(aw8697->reset_gpio))
		devm_gpio_free(&i2c->dev, aw8697->reset_gpio);
 err_reset_gpio_request:
 err_parse_dt:
	devm_kfree(&i2c->dev, aw8697);
	aw8697 = NULL;
	return ret;
}

static int aw8697_i2c_remove(struct i2c_client *i2c)
{
	struct aw8697 *aw8697 = i2c_get_clientdata(i2c);

	pr_info("%s enter\n", __func__);

	sysfs_remove_group(&i2c->dev.kobj, &aw8697_attribute_group);

	devm_free_irq(&i2c->dev, gpio_to_irq(aw8697->irq_gpio), aw8697);

	if (gpio_is_valid(aw8697->irq_gpio))
		devm_gpio_free(&i2c->dev, aw8697->irq_gpio);
//	if (gpio_is_valid(aw8697->reset_gpio))
//		devm_gpio_free(&i2c->dev, aw8697->reset_gpio);

	devm_kfree(&i2c->dev, aw8697);
	aw8697 = NULL;

	return 0;
}

static int __maybe_unused aw8697_suspend(struct device *dev)
{
	int ret = 0;
	struct aw8697 *aw8697 = dev_get_drvdata(dev);

	mutex_lock(&aw8697->lock);
	hrtimer_cancel(&aw8697->timer);
	aw8697->state = 0;             
	aw8697_haptic_stop(aw8697);
	mutex_unlock(&aw8697->lock);

	return ret;
}

static int __maybe_unused aw8697_resume(struct device *dev)
{
	int ret = 0;
	return ret;
}

static SIMPLE_DEV_PM_OPS(aw8697_pm_ops, aw8697_suspend, aw8697_resume);


static void aw8697_haptic_rtp_mode_enter()
{
	struct aw8697 *aw8697 = g_aw8697;
	if(aw8697 == NULL) {
		pr_info("%s exit\n", __func__);	
	} else {
		//pr_info("%s enter\n", __func__);
		DbgOut((DBL_INFO, "%s enter\n", __func__));
		//mutex_lock(&aw8697->lock);

		aw8697_haptic_stop(aw8697);
		aw8697_haptic_set_rtp_aei(aw8697, false);
		aw8697_interrupt_clear(aw8697);
		aw8697->rtp_init = 1;
		aw8697->rtp_routine_on = 1;
		aw8697_haptic_upload_lra(aw8697, AW8697_HAPTIC_RTP_CALI_LRA);

		/* gain */
		aw8697_haptic_ram_vbat_comp(aw8697, false);

		/* rtp mode config */
		aw8697_haptic_play_mode(aw8697, AW8697_HAPTIC_RTP_MODE);

		//mutex_unlock(&aw8697->lock);
	}
}

//Daniel 20201202 modify start
static VibeStatus aw8697_rtp_update_data(struct aw8697 *aw8697,VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
	int i = 0, j = 0;
	char ret = 0;
	int resample = gResample;
	uint8_t fifo_retry_cnt = 10;

	if(aw8697 == NULL){
		DbgOut((DBL_ERROR, "%s aw8697 is NULL \n", __func__));
		return VIBE_E_FAIL;
	}
	if((gVibDebugLog & 0x0004)== 0x0004)//enable nForceData
	{
		DbgOut((DBL_WARNING, "%s:nBufferSizeInBytes=%d +++\n", __func__, nBufferSizeInBytes));
	}
    

    	if (nBufferSizeInBytes > VIBE_OUTPUT_SAMPLE_SIZE)
        return VIBE_E_FAIL;
	
	mutex_lock(&aw8697->lock);
	//aw8697_haptic_set_pwm(g_aw8697, AW8697_PWM_12K);
	//aw8697_haptic_set_pwm(g_aw8697, gAW_RTP_PWM);
	pm_qos_add_request(&pm_qos_req_vb, PM_QOS_CPU_DMA_LATENCY, PM_QOS_VALUE_VB);
	memset(dma_buf_resampling, 0, 1024*4);
	memset(dma_buf_ivt, 0, 1024*4);
	memcpy(dma_buf_ivt , pForceOutputBuffer , nBufferSizeInBytes);
	
	aw8697_i2c_read(aw8697, AW8697_REG_GLB_STATE, &ret);
	if((ret & 0x0f) == 0x00) {

		aw8697_haptic_rtp_mode_enter();
		aw8697_haptic_start(aw8697);
	}
	mutex_unlock(&aw8697->lock);

	if(!aw8697_haptic_rtp_get_fifo_afi(aw8697)){//FIFO is not full,512 bytes per frame is recommended
	mutex_lock(&aw8697->rtp_lock);
	if(resample > 1) {
		for(i=0, j=0; i< nBufferSizeInBytes * resample; i++) {
			if(i != 0 && i % resample == 0)
				++j;

			dma_buf_resampling[i] = dma_buf_ivt[j];
		}
		aw8697_i2c_writes(aw8697, AW8697_REG_RTP_DATA,dma_buf_resampling, nBufferSizeInBytes * resample);
	}
	else{
		aw8697_i2c_writes(aw8697, AW8697_REG_RTP_DATA,dma_buf_ivt, nBufferSizeInBytes);
	}
	mutex_unlock(&aw8697->rtp_lock);
		if((gVibDebugLog & 0x0004) == 0x0004)//enable nForceData
		{
			DbgOut((DBL_WARNING, "%s:aw8697 rtp write %d done\n", __func__, nBufferSizeInBytes * resample));
			for(i=0; i<=nBufferSizeInBytes * resample; i=i+4) {
				if ( (i+4) <= (nBufferSizeInBytes * resample) ){
					if (resample == 1){
						DbgOut((DBL_WARNING, "aw8697 hex: ivt[%04d~%04d] = 0x%02x 0x%02x 0x%02x 0x%02x\n", 
						i, i+3, dma_buf_ivt[i], dma_buf_ivt[i+1], dma_buf_ivt[i+2], dma_buf_ivt[i+3] ));
					} else {
						DbgOut((DBL_WARNING, "aw8697 hex: ivt[%04d~%04d] = 0x%02x 0x%02x 0x%02x 0x%02x\n", 
						i, i+3, dma_buf_resampling[i], dma_buf_resampling[i+1], dma_buf_resampling[i+2], dma_buf_resampling[i+3] ));
					}
				} else {
					switch (i%4)
					{
					case 0:
						break;
					case 1:
						if (resample == 1){
							DbgOut((DBL_WARNING, "aw8697 hex: ivt[%04d] = 0x%02x\n", 
							i, dma_buf_ivt[i]));
						} else {
							DbgOut((DBL_WARNING, "aw8697 hex: ivt[%04d] = 0x%02x\n", 
							i, dma_buf_resampling[i]));
						}
						break;
					case 2:
						if (resample == 1){
							DbgOut((DBL_WARNING, "aw8697 hex: ivt[%04d~%04d] = 0x%02x 0x%02x\n", 
							i, i+1, dma_buf_ivt[i], dma_buf_ivt[i+1]));
						} else {
							DbgOut((DBL_WARNING, "aw8697 hex: ivt[%04d~%04d] = 0x%02x 0x%02x\n", 
							i, i+1, dma_buf_resampling[i], dma_buf_resampling[i+1]));
						}
						break;
					case 3:
						if (resample == 1){
							DbgOut((DBL_WARNING, "aw8697 hex: ivt[%04d~%04d] = 0x%02x 0x%02x 0x%02x\n", 
							i, i+2, dma_buf_ivt[i], dma_buf_ivt[i+1], dma_buf_ivt[i+2]));
						} else {
							DbgOut((DBL_WARNING, "aw8697 hex: ivt[%04d~%04d] = 0x%02x 0x%02x 0x%02x\n", 
							i, i+2, dma_buf_resampling[i], dma_buf_resampling[i+1], dma_buf_resampling[i+2]));
						}
						break;
					}
				}
				//DbgOut((DBL_WARNING, "dec: ivt[%04d]=0x%03d\n", i, dma_buf_ivt[i]));
			}
			DbgOut((DBL_WARNING, "\n"));
		}
	} else {//FIFO is full
		while(fifo_retry_cnt--){
			usleep_range(300, 500);
			DbgOut((DBL_WARNING, "%s fifo is full delay 0.3ms %d\n",__func__, fifo_retry_cnt));
			if(!aw8697_haptic_rtp_get_fifo_afi(aw8697)){
				break;
			}
		}
		mutex_lock(&aw8697->rtp_lock);
		if(resample > 1) {
		for(i=0, j=0; i< nBufferSizeInBytes * resample; i++) {
			if(i != 0 && i % resample == 0)
				++j;

			dma_buf_resampling[i] = dma_buf_ivt[j];
		}
		aw8697_i2c_writes(aw8697, AW8697_REG_RTP_DATA,dma_buf_resampling, nBufferSizeInBytes * resample);
		}
		else{
			aw8697_i2c_writes(aw8697, AW8697_REG_RTP_DATA,dma_buf_ivt, nBufferSizeInBytes);
		}
		mutex_unlock(&aw8697->rtp_lock);
		pm_qos_remove_request(&pm_qos_req_vb);
		return VIBE_S_SUCCESS;//Retransmission required
	}

	//DbgOut((DBL_WARNING, "%s:aw8697 rtp write %d done\n", __func__, nBufferSizeInBytes * resample));
	usleep_range(500, 1000);
	pm_qos_remove_request(&pm_qos_req_vb);
	return VIBE_S_SUCCESS;
}//Daniel 20201202 modify end




static const struct i2c_device_id aw8697_i2c_id[] = {
	{AW8697_I2C_NAME, 0},
	{}
};

static const struct of_device_id aw8697_dt_match[] = {
	{.compatible = "awinic,aw8697_haptic"},
	{},
};

static struct i2c_driver aw8697_i2c_driver = {
	.driver = {
		   .name = AW8697_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(aw8697_dt_match),
#ifdef CONFIG_PM_SLEEP
		    .pm = &aw8697_pm_ops,
#endif
		   },
	.probe = aw8697_i2c_probe,
	.remove = aw8697_i2c_remove,
	.id_table = aw8697_i2c_id,
};


/*
** Called to disable amp (disable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{
    int result;
    struct aw8697 *aw8697 = g_aw8697;

    if(aw8697 == NULL) {
		//DbgOut((DBL_INFO, "%s:aw8697 exit \n", __func__));
		return VIBE_E_FAIL;
    }
    result = aw8697_haptic_stop(aw8697);
    schedule_delayed_work(&aw8697->gain_work, msecs_to_jiffies(1));
    //DbgOut((DBL_INFO, "%s:aw8697 result=%d \n", __func__, result));
    if(result == 0) {
        return VIBE_S_SUCCESS;
    } else {
        return VIBE_E_FAIL;
    }
    
}


IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex)
{
    int result = 0;
	struct aw8697 *aw8697 = g_aw8697;

	if (aw8697 == NULL) {
		DbgOut((DBL_ERROR, "%s: aw8697 is not exist.\n", __func__));
		return VIBE_E_FAIL;
	}else{
		 DbgOut((DBL_INFO, "%s:aw8697 enter\n", __func__));
	}
    //need to check, 
    //if immersion call AmpEnable every time before send data,
    //should call aw8697_haptic_rtp_mode_enter here
    //else call aw8697_haptic_rtp_mode_enter in aw8697_rtp_update_data
    //aw8697_haptic_rtp_mode_enter();
    //result = aw8697_haptic_set_pwm(g_aw8697, gPWM);
    //result = aw8697_haptic_set_pwm(g_aw8697, AW8697_PWM_12K);
    
    //DbgOut((DBL_INFO, "%s:aw8697 result=%d \n", __func__, result));

    cancel_delayed_work(&aw8697->gain_work);

//ASUS_BSP: stop and clear fifo 
		aw8697_haptic_stop(aw8697);
		aw8697_haptic_set_rtp_aei(aw8697, false);
		aw8697_interrupt_clear(aw8697);

		aw8697_haptic_set_pwm(g_aw8697, gAW_RTP_PWM);
		//aw8697_haptic_upload_lra(aw8697, AW8697_HAPTIC_RTP_CALI_LRA);

		/* gain */
		//aw8697_haptic_ram_vbat_comp(aw8697, false);

    if(result == 0) {
        return VIBE_S_SUCCESS;
    } else {
        return VIBE_E_FAIL;
    }
}

/*
** Called at initialization time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize(void)
{
    int retVal = 0;

    DbgOut((DBL_ERROR, "%s:aw8697 +++\n", __func__));
    
    //allocate DMA buffer
    if(!dma_buf_resampling){
		dma_buf_resampling=kmalloc(1024*4, GFP_DMA | GFP_KERNEL); //1028*4, 12k upto 24k
		if(dma_buf_resampling) {
			memset(dma_buf_resampling, 0, 1024*4);
		}
		else{
			DbgOut((DBL_ERROR, "The buffer dma_buf_resampling kmalloc failed.\n"));
		}
	}
	if(!dma_buf_ivt){
		dma_buf_ivt=kmalloc(1024*4, GFP_DMA | GFP_KERNEL);
		if(dma_buf_ivt){
			memset(dma_buf_ivt, 0, 1024*4);
		}
		else{
			DbgOut((DBL_ERROR, "The buffer dma_buf_ivt kmalloc failed.\n"));
		}
	}
	//end
	
    retVal = i2c_add_driver(&aw8697_i2c_driver);
    if (retVal) {
        DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_Initialize: Cannot add driver.\n"));
    }
    
    DbgOut((DBL_ERROR, "%s:---\n", __func__));
    return VIBE_S_SUCCESS;
}

/*
** Called at termination time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate(void)
{
//#error Please review the code between the #if and #endif

	if (g_aw8697 == NULL) {
		DbgOut((DBL_INFO, "%s: aw8697 is not exist.\n", __func__));
		return VIBE_E_FAIL;
	} else {
		DbgOut((DBL_INFO, "ImmVibeSPI_ForceOut_Terminate. del i2c aw8697 driver\n"));
	}
    i2c_del_driver(&aw8697_i2c_driver);

    return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set force output, and enable amp if required
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
	if (g_aw8697 == NULL) {
		//DbgOut((DBL_ERROR, "%s: aw8697 is not exist.\n", __func__));
		return VIBE_E_FAIL;
	} else {
		//int mode, active;
		if((gVibDebugLog & 0x0002) == 0x0002)
			DbgOut((DBL_INFO, "%s:aw8697 enter\n", __func__));
		//DbgOut((DBL_INFO, "%s:aw8697 enter. nActuatorIndex=%d nOutputSignalBitDepth=%d nBufferSizeInBytes=%d \n", __func__ ,nActuatorIndex,nOutputSignalBitDepth,nBufferSizeInBytes));
	}

    if (VIBE_S_SUCCESS != aw8697_rtp_update_data(g_aw8697,nBufferSizeInBytes,pForceOutputBuffer)) {
		DbgOut((DBL_ERROR, "%s: nActuatorIndex=%d failed ---\n", __func__, nActuatorIndex));
        return VIBE_E_FAIL;
    }

    if((gVibDebugLog & 0x0004) == 0x0004)//enable immersion message
		DbgOut((DBL_ERROR, "%s: nActuatorIndex=%d ---\n", __func__, nActuatorIndex));
    return VIBE_S_SUCCESS;

}

/*
** Called to set force output frequency parameters
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency(VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID, VibeUInt32 nFrequencyParameterValue)
{
//#error Please review the code between the #if and #endif
//DbgOut((DBL_INFO, "%s\n", __func__));
#if 0 
    /* 
    ** The following code is provided as sample. If enabled, it will allow device 
    ** frequency parameters tuning via the ImmVibeSetDeviceKernelParameter API.
    ** Please modify as required. 
    */
    switch (nActuatorIndex)
    {
        case 0:
            switch (nFrequencyParameterID)
            {
                case VIBE_KP_CFG_FREQUENCY_PARAM1:
                    /* Update frequency parameter 1 for actuator 0 */
                    break;

                case VIBE_KP_CFG_FREQUENCY_PARAM2:
                    /* Update frequency parameter 2 for actuator 0 */
                    break;

                case VIBE_KP_CFG_FREQUENCY_PARAM3:
                    /* Update frequency parameter 3 for actuator 0 */
                    break;

                case VIBE_KP_CFG_FREQUENCY_PARAM4:
                    /* Update frequency parameter 4 for actuator 0 */
                    break;

                case VIBE_KP_CFG_FREQUENCY_PARAM5:
                    /* Update frequency parameter 5 for actuator 0 */
                    break;

                case VIBE_KP_CFG_FREQUENCY_PARAM6:
                    /* Update frequency parameter 6 for actuator 0 */
                    break;
            }
            break;

        case 1:
            switch (nFrequencyParameterID)
            {
                case VIBE_KP_CFG_FREQUENCY_PARAM1:
                    /* Update frequency parameter 1 for actuator 1 */
                    break;

                case VIBE_KP_CFG_FREQUENCY_PARAM2:
                    /* Update frequency parameter 2 for actuator 1 */
                    break;

                case VIBE_KP_CFG_FREQUENCY_PARAM3:
                    /* Update frequency parameter 3 for actuator 1 */
                    break;

                case VIBE_KP_CFG_FREQUENCY_PARAM4:
                    /* Update frequency parameter 4 for actuator 1 */
                    break;

                case VIBE_KP_CFG_FREQUENCY_PARAM5:
                    /* Update frequency parameter 5 for actuator 1 */
                    break;

                case VIBE_KP_CFG_FREQUENCY_PARAM6:
                    /* Update frequency parameter 6 for actuator 1 */
                    break;
            }
            break;

        /* Etc. if more than two actuators. */
    }
#endif

    return VIBE_S_SUCCESS;
}
#if 0
/*
** Called to save an IVT data file (pIVT) to a file (szPathName)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_IVTFile_Save(const VibeUInt8 *pIVT, VibeUInt32 nIVTSize, const char *szPathname)
{
//#error Please review the code between the #if and #endif

#if 0   /* The following code is provided as sample. Please modify as required. */
    /* To be implemented */
#endif

    return VIBE_S_SUCCESS;
}

/*
** Called to delete an IVT file
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_IVTFile_Delete(const char *szPathname)
{
//#error Please review the code between the #if and #endif

#if 0   /* The following code is provided as sample. Please modify as required. */
    /* To be implemented */
#endif

    return VIBE_S_SUCCESS;
}
#endif
/*
** Called to get the device name (device name must be returned as ANSI string)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName(VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
    
    char szRevision[24];
	DbgOut((DBL_INFO, "%s\n", __func__));
    if ((!szDevName) || (nSize < 1)) {
        return VIBE_E_FAIL;
    }
	if(g_aw8697==NULL){
		sprintf(szRevision, DEVICE_NAME "-AW8697-%02x", 0x97);
	}
	else{
	
		sprintf(szRevision, DEVICE_NAME "-AW8697-%02x", g_aw8697->chipid);
	}
    if (strlen(szRevision) + strlen(szDevName) < nSize - 1) {
        strcat(szDevName, szRevision);
    }

    /* Make sure the string is NULL terminated */
    szDevName[nSize - 1] = '\0';
    
    return VIBE_S_SUCCESS;
}
/*
** Called by the TouchSense Player Service/Daemon when an application calls
** ImmVibeGetDeviceCapabilityInt32 with VIBE_DEVCAPTYPE_DRIVER_STATUS.
** The TouchSense Player does not interpret the returned status.
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetStatus(VibeUInt8 nActuatorIndex)
{
	//DbgOut((DBL_INFO, "%s\n", __func__));
    return VIBE_S_SUCCESS;
}
#if 0
/*
** Called at initialization time to get the number of actuators
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetNum(void)
{
    return NUM_ACTUATORS;
}
#endif

#ifdef IMMVIBESPI_USE_BUFFERFULL
/*
** Check if the amplifier sample buffer is full (not ready for more data).
*/
IMMVIBESPIAPI int ImmVibeSPI_ForceOut_BufferFull(void)
{
    DbgOut((DBL_INFO, "%s: enter\n", __func__));
//need to check hupeng
    return VIBE_S_SUCCESS;
}
#endif

