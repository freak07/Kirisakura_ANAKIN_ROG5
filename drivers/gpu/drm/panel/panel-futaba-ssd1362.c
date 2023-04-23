/* Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Edit by ASUS Mickey, mickey_liao@asus.com
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/mutex.h>
#include <linux/vmalloc.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/mman.h> 
#include "boot_logo.h"

#define IMAGE_SIZE 8310
#define IMAGE_HEADER_SIZE 118
#define PANEL_RESOLUTION 64*256
#define PANEL_BPP 4
#define COLOR_PANEL_BPP 16
#define FRAME_SIZE PANEL_RESOLUTION*PANEL_BPP/8
#define COLOR_FRAME_SIZE PANEL_RESOLUTION*COLOR_PANEL_BPP/8
#define BUFFER_COUNT 3

int frame_num = 0;
int delayms = 0;
int repeat = 0;
static const char *mmfile = "pmoled";
static struct page *start_page;

struct ssd1362_platform_data {
	struct spi_device *spi;
	struct mutex mutex;
	int panel_id;    //0: mono, 1: color
	int reset_gpio;
	u32 reset_gpio_flag;
	int dc_gpio;
	u32 dc_gpio_flag;
	int pwren_gpio;
	u32 pwren_gpio_flag;
	int vci_gpio;    //vci for mono and vdd for color
	u32 vci_gpio_flag;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_suspend;
	bool power_disabled;
	u8 backlight_level;
	u32 buffer_offset;
	struct page *fb_page;
	u8 *frame_buffer;
	u8 *fb_clean;
	int atd_pattern;
	int frame_size;
};

//Mono pmoled init code
u8 mono_init_code[30] = {
    0xAB, 0x01,            // set Vdd Mode //ELW2106AA VCI = 3.0V
    0xAD, 0x9E,            // set IREF selection
    0x15, 0x00, 0x7F,      // set column address
    0x75, 0x00, 0x3F,      // set row address
    0x81, 0x87,            // set contrast control
    0xA0, 0x43,            // set Re-map //0x43
    0xA1, 0x00,            // set Display start line
    0xA2, 0x00,            // set Display Offset //30 10
    0xA4,                  // set Display Mode
    0xA8, 0x3F,            // set Multiplex Ratio
    0xB1, 0x11,            // set Phase1,2 Length
    0xB3, 0xF0,            // set display clock Divide Ratio
    0xB9,                  // Gray Scale Table
    0xBC, 0x04,            // set Pre-charge Voltage
    0xBE, 0x05,            // set VCOMH deselect Level // 0.82*Vcc
	};

u8 mono_display_on[1] = {0xAF}; //mono display On

#define COLOR_INIT_SIZE 39

u8 color_init_code[COLOR_INIT_SIZE][15]= //reserve 5 bytes for further use
{{1, 1, 0x01},
 {1, 1, 0x03},
 {0, 1, 0x00},
 {1, 1, 0x04},
 {0, 1, 0x00},
 {1, 1, 0x05},
 {0, 1, 0x03},
 {1, 1, 0x06},
 {0, 1, 0x00},
 {1, 1, 0x07},
 {0, 8, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x03, 0x0F},
 {1, 1, 0x08},
 {0, 1, 0x01},
 {1, 1, 0x09},
 {0, 1, 0x07},
 {1, 1, 0x0A},
 {0, 8, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x03, 0x0F},
 {1, 1, 0x0B},
 {0, 4, 0x00, 0x00, 0x00, 0x00},
 {1, 1, 0x0E},
 {0, 6, 0x02, 0x0D, 0x02, 0x0D, 0x02, 0x0D},
 {1, 1, 0x0F},
 {0, 3, 0x01, 0x01, 0x01},
 {1, 1, 0x10},
 {0, 1, 0x01},
 {1, 1, 0x1C},
 {0, 1, 0x0C},
 {1, 1, 0x1D},
 {0, 3, 0x01, 0x01, 0x01},
 {1, 1, 0x1E},
 {0, 1, 0x00},
 {1, 1, 0x1F},
 {0, 1, 0x00},
 {1, 1, 0x20},
 {0, 1, 0x10},
 {1, 1, 0x25},
 {0, 1, 0x03},
 {1, 1, 0x2E},
 {0, 1, 0x01}};

u8 color_address_set[2][10] =
{{1, 1, 0x0A},
 {0, 8, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x03, 0x0F}};

u8 color_display_on[4][3] =
{{1, 1, 0x03},
 {0, 1, 0x00},
 {1, 1, 0x02},
 {0, 1, 0x01}};

u8 color_display_off[4][3] =
{{1, 1, 0x02},
 {0, 1, 0x00},
 {1, 1, 0x03},
 {0, 1, 0x01}};

u8 color_backlight[8][6] =
{{0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
 {0x00, 0x06, 0x00, 0x06, 0x00, 0x06},
 {0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C},
 {0x01, 0x03, 0x01, 0x03, 0x01, 0x03},
 {0x01, 0x09, 0x01, 0x09, 0x01, 0x09},
 {0x02, 0x00, 0x02, 0x00, 0x02, 0x00},
 {0x02, 0x07, 0x02, 0x07, 0x02, 0x07},
 {0x02, 0x0D, 0x02, 0x0D, 0x02, 0x0D}};

int ssd1362_power_control(struct ssd1362_platform_data *pdata, bool pwr_on)
{
	int err = 0;
	mutex_lock(&pdata->mutex);
	if (pwr_on) {
		err = pinctrl_select_state(pdata->pinctrl, pdata->pins_active);
		if (err) {
			dev_err(&pdata->spi->dev, "%s: pinctrl_select_state err:%d\n", __func__, err);
			goto failed;
		}

		if (pdata->power_disabled) {
			//set control pin output before power on sequence
			gpio_direction_output(pdata->reset_gpio, 0);
			gpio_direction_output(pdata->dc_gpio, 0);
			if (pdata->panel_id) {
				//printk("[PMOLED] enable VDD\n");
				if (g_ASUS_hwID >= HW_REV_ER) {
					//dev_info(&pdata->spi->dev, "[PMOLED] VCI H\n");
					gpio_direction_output(pdata->vci_gpio, 1);
					mdelay(2);
				}
				//dev_info(&pdata->spi->dev, "[PMOLED] PWR en H\n");
				gpio_direction_output(pdata->pwren_gpio, 1);
				mdelay(1);
				//dev_info(&pdata->spi->dev, "[PMOLED] reset H\n");
				gpio_direction_output(pdata->reset_gpio, 1);
			} else {
				//printk("[PMOLED] enable VDD\n");
				if (g_ASUS_hwID >= HW_REV_ER) {
					//dev_info(&pdata->spi->dev, "[PMOLED] VCI H\n");
					gpio_direction_output(pdata->vci_gpio, 1);
					mdelay(1);
				}
				//dev_info(&pdata->spi->dev, "[PMOLED] reset H\n");
				gpio_direction_output(pdata->reset_gpio, 1);
				msleep(50);
				//dev_info(&pdata->spi->dev, "[PMOLED] reset L\n");
				gpio_direction_output(pdata->reset_gpio, 0);
				msleep(1);
				//dev_info(&pdata->spi->dev, "[PMOLED] reset H\n");
				gpio_direction_output(pdata->reset_gpio, 1);
				msleep(1);
				//dev_info(&pdata->spi->dev, "[PMOLED] PWR en H\n");
				gpio_direction_output(pdata->pwren_gpio, 1);
			}
			pdata->power_disabled = false;
        }
	} else {
		if (!pdata->power_disabled) {
			if (pdata->panel_id) {
				//need to display off first.
				//dev_info(&pdata->spi->dev, "[PMOLED] reset L\n");
				gpio_direction_output(pdata->reset_gpio, 0);
				mdelay(1);
				//dev_info(&pdata->spi->dev, "[PMOLED] PWR en L\n");
				gpio_direction_output(pdata->pwren_gpio, 0);
				msleep(150);// by EE's request
				if (g_ASUS_hwID >= HW_REV_ER) {
					//dev_info(&pdata->spi->dev, "[PMOLED] VCI L\n");
					gpio_direction_output(pdata->vci_gpio, 0);
				}
			} else {
				//dev_info(&pdata->spi->dev, "[PMOLED] PWR en L\n");
				gpio_direction_output(pdata->pwren_gpio, 0);
				msleep(100);
				if (g_ASUS_hwID >= HW_REV_ER) {
					//dev_info(&pdata->spi->dev, "[PMOLED] VCI L\n");
					gpio_direction_output(pdata->vci_gpio, 0);
				}
				//dev_info(&pdata->spi->dev, "[PMOLED] reset L\n");
				gpio_direction_output(pdata->reset_gpio, 0);
			}
			gpio_direction_output(pdata->dc_gpio , 0);
			//for saving power
			msleep(10);
			gpio_direction_input(pdata->dc_gpio);
			gpio_direction_input(pdata->reset_gpio);
			pdata->power_disabled = true;
		}

		err = pinctrl_select_state(pdata->pinctrl, pdata->pins_suspend);
		if (err) {
			dev_err(&pdata->spi->dev, "%s: pinctrl_select_state err:%d\n", __func__, err);
			goto failed;
		}
	}

failed:
	mutex_unlock(&pdata->mutex);
	return err;
}

void ssd1362_dc_control(struct ssd1362_platform_data *pdata, bool cmd)
{
	if (cmd) {
		//dev_info(&pdata->spi->dev, "[PMOLED] DC L\n");
		gpio_direction_output(pdata->dc_gpio, 0);
	} else {
		//dev_info(&pdata->spi->dev, "[PMOLED] DC H\n");
		gpio_direction_output(pdata->dc_gpio, 1);
	}
}

int ssd1362_spi_write(struct ssd1362_platform_data *pdata, u8 *buff, int len, bool cmd)
{
	struct spi_message msg;
	struct spi_transfer xfer = {
		.bits_per_word		= 8,
		.tx_nbits			= SPI_NBITS_SINGLE,
	};
	int err = 0;

	mutex_lock(&pdata->mutex);
	if (pdata->power_disabled) {
		dev_err(&pdata->spi->dev, "%s: [PMOLED] power disabled, skip spi command\n", __func__);
		mutex_unlock(&pdata->mutex);
		return -1;
	}
	
	ssd1362_dc_control(pdata, cmd);

	spi_message_init(&msg);

	xfer.tx_buf = buff;
	xfer.len = len;
	spi_message_add_tail(&xfer, &msg);

	err = spi_sync(pdata->spi, &msg);
	mutex_unlock(&pdata->mutex);
	return err;
}
	
static int ssd1362_parse_dt(struct device *dev, struct ssd1362_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	int err;
	u32 id_gpio_flag = 0;
	int id_gpio = 0;

	printk("[PMOLED] ssd1362_parse_dt\n");

	id_gpio = of_get_named_gpio_flags(np, "futaba,id-gpio", 0, &id_gpio_flag);

	pdata->reset_gpio = of_get_named_gpio_flags(np, "futaba,reset-gpio", 0, &pdata->reset_gpio_flag);
	
	pdata->dc_gpio = of_get_named_gpio_flags(np, "futaba,dc-gpio", 0, &pdata->dc_gpio_flag);

	pdata->pwren_gpio = of_get_named_gpio_flags(np, "futaba,pwren-gpio", 0, &pdata->pwren_gpio_flag);

	if (g_ASUS_hwID >= HW_REV_ER)
		pdata->vci_gpio = of_get_named_gpio_flags(np, "futaba,vci-gpio", 0, &pdata->vci_gpio_flag);

	// Get the pinctrl node
	pdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(pdata->pinctrl)) {
	     dev_err(dev, "%s: Failed to get pinctrl\n", __func__);
	}

	// Get the active setting
	pdata->pins_active = pinctrl_lookup_state(pdata->pinctrl, "futaba_active");
	if (IS_ERR_OR_NULL(pdata->pins_active)) {
		dev_err(dev, "%s: Failed to get pinctrl state active\n", __func__);
	}
	
	// Get the suspend setting
	pdata->pins_suspend = pinctrl_lookup_state(pdata->pinctrl, "futaba_suspend");
	if (IS_ERR_OR_NULL(pdata->pins_suspend)) {
		dev_err(dev, "%s: Failed to get pinctrl state suspend\n", __func__);
	}

	if (id_gpio) {
		err = pinctrl_select_state(pdata->pinctrl, pdata->pins_active);
		if (err)
			dev_err(dev, "%s: pinctrl_select_state err:%d\n", __func__, err);
		gpio_direction_output(id_gpio, 1);
		msleep(1);
		gpio_direction_input(id_gpio);
		pdata->panel_id = gpio_get_value(id_gpio);
		pr_err("[PMOLED] panel id = %d\n", pdata->panel_id);
	}

	// Set the suspend setting
	printk("[PMOLED] set the suspend state\n");
	err = pinctrl_select_state(pdata->pinctrl, pdata->pins_suspend);
	if (err)
		dev_err(dev, "%s: pinctrl_select_state err:%d\n", __func__, err);

	return 0;
}

void ssd1362_wfbc(struct ssd1362_platform_data *pdata) //wfbc ==> write framebuffer command which is needed by color panel
{
	if (pdata->panel_id) {
		int err;
		u8 wfb= 0x0C;

		err = ssd1362_spi_write(pdata, &wfb, 1, true);
		if (err)
			dev_err(&pdata->spi->dev, "[PMOLED] sending wfb failed, err=%d\n", err);
	}
}

void ssd1362_set_backlight(struct ssd1362_platform_data *pdata, int level)
{
	int err = 0;
	pdata->backlight_level = level & 0xFF;
	if (pdata->panel_id) {
		int err = 0;
		int i = 0;
		u8 cmd= 0x0E;
		if (pdata->backlight_level == 0)
			i=0;
		else if (pdata->backlight_level>0 && pdata->backlight_level<=40)
			i=1;
		else if (pdata->backlight_level>40 && pdata->backlight_level<=80)
			i=2;
		else if (pdata->backlight_level>80 && pdata->backlight_level<=120)
			i=3;
		else if (pdata->backlight_level>120 && pdata->backlight_level<=160)
			i=4;
		else if (pdata->backlight_level>160 && pdata->backlight_level<=200)
			i=5;
		else if (pdata->backlight_level>200 && pdata->backlight_level<=240)
			i=6;
		else if (pdata->backlight_level>240)
			i=7;
		err = ssd1362_spi_write(pdata, &cmd, 1, true);
		err |= ssd1362_spi_write(pdata, color_backlight[i], 6, false);
		if (err)
			dev_err(&pdata->spi->dev, "[PMOLED] sending backlight failed, err=%d\n", err);
	} else {
		u8 cmd[2] = {
			0x81,    // Set contrast control
			0x00
			};
		cmd[1] = pdata->backlight_level;
		err = ssd1362_spi_write(pdata, cmd, sizeof(cmd), true);
		if (err)
			dev_err(&pdata->spi->dev, "[PMOLED] sending backlight cmd failed, err=%d\n", err);
	}
	dev_warn(&pdata->spi->dev, "[PMOLED] setting backlight = %d\n", pdata->backlight_level);
}

void ssd1362_panel_on(struct ssd1362_platform_data *pdata, bool pwr_on, bool color)
{
	int err;

	if (pwr_on && color) {
		int i;
		dev_warn(&pdata->spi->dev, "[PMOLED] turn on panel\n");
		ssd1362_power_control(pdata, pwr_on);
		msleep(200);
		err = 0;
		for (i=0; i<COLOR_INIT_SIZE; i++)
			err |= ssd1362_spi_write(pdata, (color_init_code[i]+2), color_init_code[i][1], color_init_code[i][0]);

		if (err)
			dev_err(&pdata->spi->dev, "[PMOLED] sending spi initial code failed, err=%d\n", err);

		ssd1362_set_backlight(pdata, pdata->backlight_level);

		err = ssd1362_spi_write(pdata, (color_address_set[0]+2), color_address_set[0][1], color_address_set[0][0]);
		err |= ssd1362_spi_write(pdata, (color_address_set[1]+2), color_address_set[1][1], color_address_set[1][0]);
		if (err)
			dev_err(&pdata->spi->dev, "[PMOLED] sending spi initial code failed, err=%d\n", err);

		ssd1362_wfbc(pdata);
		err = ssd1362_spi_write(pdata, pdata->fb_clean, pdata->frame_size, false);
		if (err)
			dev_err(&pdata->spi->dev, "[PMOLED] sending clean framebuffer failed, err=%d\n", err);
		mdelay(1);

		err = 0;
		for (i=0; i<4; i++)
			err |= ssd1362_spi_write(pdata, (color_display_on[i]+2), color_display_on[i][1], color_display_on[i][0]);
		if (err)
			dev_err(&pdata->spi->dev, "[PMOLED] sending display on cmd failed, err=%d\n", err);
	} else if (pwr_on && !color) {
		u8 cmd[6] = {
		      0x15,    // Set Segment Remap
		      0x00,    // Set Segment Remap
		      0x7F,    // 4fSet Segment Remap
		      0x75,    // Set Segment Remap
		      0x00,    // Set Segment Remap
		      0x3f     // 1fSet Segment Remap
		      };
		mono_init_code[11] = pdata->backlight_level;
		dev_warn(&pdata->spi->dev, "[PMOLED] turn on panel\n");
		ssd1362_power_control(pdata, pwr_on);
		msleep(200);
		err = ssd1362_spi_write(pdata, mono_init_code, sizeof(mono_init_code), 1);
		if (err)
			dev_err(&pdata->spi->dev, "[PMOLED] sending spi initial code failed, err=%d\n", err);
		err = ssd1362_spi_write(pdata, cmd, sizeof(cmd), true);
		if (err)
			dev_err(&pdata->spi->dev, "[PMOLED] sending ram setting code failed, err=%d\n", err);
		err = ssd1362_spi_write(pdata, pdata->fb_clean, pdata->frame_size, false);
		if (err)
			dev_err(&pdata->spi->dev, "[PMOLED] sending clean framebuffer failed, err=%d\n", err);
		err = ssd1362_spi_write(pdata, mono_display_on, sizeof(mono_display_on), true);
		if (err)
			dev_err(&pdata->spi->dev, "[PMOLED] sending display on cmd failed, err=%d\n", err);
	} else if (!pwr_on && !color) {
		dev_warn(&pdata->spi->dev, "[PMOLED] turn off panel\n");
		ssd1362_power_control(pdata, pwr_on);
	} else if (!pwr_on && color) {
		int i;
		err = 0;
		for (i=0; i<4; i++) {
			err |= ssd1362_spi_write(pdata, (color_display_off[i]+2), color_display_off[i][1], color_display_off[i][0]);
			if (i==1)
				mdelay(1);
		}
		if (err)
			dev_err(&pdata->spi->dev, "[PMOLED] sending display off cmd failed, err=%d\n", err);

		ssd1362_power_control(pdata, pwr_on);
	}
}

void show_logo(struct ssd1362_platform_data *pdata)
{
	if (pdata->panel_id) {
		ssd1362_panel_on(pdata, true, pdata->panel_id);
		ssd1362_set_backlight(pdata, 100);
		memcpy(pdata->frame_buffer, boot_logo, sizeof(boot_logo));
		ssd1362_wfbc(pdata);
		ssd1362_spi_write(pdata, pdata->frame_buffer, pdata->frame_size, false);
	}
}

static ssize_t Power_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	struct ssd1362_platform_data *pdata = dev_get_drvdata(dev);
	ssize_t err;
	u32 val;

	err = kstrtou32(buf, 10, &val);
	if (err)
		return err;

	if (val>=0 && val <=1)
		ssd1362_panel_on(pdata, val, pdata->panel_id);
	else
		dev_err(&pdata->spi->dev, "[PMOLED] please input a number between 0~1\n");
	return count;
}

static ssize_t Power_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct ssd1362_platform_data *pdata = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d", !pdata->power_disabled);
}

static ssize_t Test_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	struct ssd1362_platform_data *pdata = dev_get_drvdata(dev);
	ssize_t err;
	u32 val;

	err = kstrtou32(buf, 10, &val);
	if (err)
		return err;

	if (pdata->power_disabled) {
		dev_err(&pdata->spi->dev, "[PMOLED] panel power disabled, turn on panel power before panel test\n");
		return -1;
	}
#if 0
	if (val>255 || val<0) {
		dev_err(&pdata->spi->dev, "[PMOLED] please input a number between 0~255\n");
	} else {
		u8 content = (u8) val;
		content = content & 0xFF;
		memset(pdata->frame_buffer, content, pdata->frame_size);
	}
#else
	if (val>100 || val<0) {
		dev_err(&pdata->spi->dev, "[PMOLED] please input a number between 0~255\n");
	} else {
			memset(pdata->frame_buffer, 0x00, pdata->frame_size);
		if (val > 0)
			memset(pdata->frame_buffer, 0xFF, (pdata->frame_size*val/100));
	}
#endif
	ssd1362_wfbc(pdata);
	err = ssd1362_spi_write(pdata, pdata->frame_buffer, pdata->frame_size, false);
	if (err)
		dev_err(&pdata->spi->dev, "[PMOLED] sending test buffer failed, err=%d\n", err);
	return count;
}

static ssize_t Test_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "[PMOLED] test, input 1 for white screen, 0 for black screen\n");
}

static ssize_t Backlight_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	struct ssd1362_platform_data *pdata = dev_get_drvdata(dev);
	ssize_t err;
	u32 val;

	err = kstrtou32(buf, 10, &val);
	if (err)
		return err;

	if (pdata->power_disabled) {
		dev_err(&pdata->spi->dev, "[PMOLED] panel power disabled, turn on panel power before setting backlight\n");
		return -1;
	}

	ssd1362_set_backlight(pdata, val);

	return count;
}

static ssize_t Backlight_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct ssd1362_platform_data *pdata = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d", pdata->backlight_level);
}

static ssize_t Commit_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	struct ssd1362_platform_data *pdata = dev_get_drvdata(dev);
	ssize_t err;
	u32 val;
	u8 *ptr;

	err = kstrtou32(buf, 10, &val);
	if (err)
		return err;

	if (pdata->power_disabled) {
		dev_err(&pdata->spi->dev, "[PMOLED] panel power disabled, turn on panel power before commit buffer\n");
		return -1;
	}

	if (val>2 || val<0) {
		dev_err(&pdata->spi->dev, "[PMOLED] commit buffer index is must between 0~2\n");
	}

	ptr = pdata->frame_buffer + (val * pdata->buffer_offset);
	ssd1362_wfbc(pdata);
	err = ssd1362_spi_write(pdata, ptr, pdata->frame_size, false);
	if (err)
		dev_err(&pdata->spi->dev, "[PMOLED] sending framebuffer failed, index=%d, err=%d\n", val, err);
	
	return count;
}

static ssize_t Commit_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "[PMOLED] giving framebuffer index from 0 to 2 to commit to panel\n");
}

static ssize_t FrameNum_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	ssize_t err;
	u32 val;
	
	err = kstrtou32(buf, 10, &val);
	if (err)
		return err;

	if(val <= 0) {
		dev_err(dev, "[PMOLED] frame number must be greater than 0\n");
	} else {
		frame_num = val;
	}
	
	return count;
}

static ssize_t FrameNum_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "[PMOLED] frame number = %d\n", frame_num);
}

static ssize_t Delayms_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	ssize_t err;
	u32 val;
	
	err = kstrtou32(buf, 10, &val);
	if (err)
		return err;

	if(val <= 0) {
		dev_err(dev, "[PMOLED] delay ms must be greater than 0\n");
	} else {
		delayms = val;
	}

	return count;
}

static ssize_t Delayms_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "[PMOLED] delay ms = %d\n", delayms);
}

static ssize_t Repeat_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	ssize_t err;
	u32 val;
	
	err = kstrtou32(buf, 10, &val);
	if (err)
		return err;

	if(val < 0) {
		dev_err(dev, "[PMOLED] repeat count must be greater than or equal to 0\n");
	} else {
		repeat = val;
	}

	return count;
}

static ssize_t Repeat_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "[PMOLED] repeat count = %d\n", repeat);
}

static mm_segment_t oldfs;
static ssize_t ShowPattern_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	ssize_t err;
	u32 val;
	struct ssd1362_platform_data *pdata = dev_get_drvdata(dev);
	char path[256];
	u8 *imageBuff, *ptr;
	int i = 0, r = 0;
	err = kstrtou32(buf, 10, &val);
	if (err)
		return err;

	if (frame_num <= 0)
		return count;
	
	imageBuff = vmalloc(frame_num * IMAGE_SIZE);

	if (imageBuff == NULL) {
		dev_err(&pdata->spi->dev, "[PMOLED] cannot allocate memory for image\n");
		return -1;
	}
	
	ptr = imageBuff;
	oldfs = get_fs();
    set_fs(KERNEL_DS);
	for (i=0; i<frame_num; i++) {
		int fd;
		memset(path, 0x00, sizeof(path));
		snprintf(path, PAGE_SIZE, "/data/pmoled/pic%03d.bmp", i);
		//printk("[PMOLED] file=%s\n",path);
		fd = ksys_open(path, O_RDONLY, 0);
        if( fd < 0 ){
			dev_err(&pdata->spi->dev, "[PMOLED] open file failed %d\n");
            err = -1;
            set_fs(oldfs);
			goto failed;
        }
        ksys_read(fd,ptr,IMAGE_SIZE);
        ksys_close(fd);
        ptr += IMAGE_SIZE;
	}
	set_fs(oldfs);

	for (r=0; r<(repeat+1); r++) {
		ptr = imageBuff;
		for (i=0; i<frame_num; i++) {
			ptr += IMAGE_HEADER_SIZE;
			memcpy(pdata->frame_buffer, ptr, pdata->frame_size);
			ssd1362_wfbc(pdata);
			ssd1362_spi_write(pdata, pdata->frame_buffer, pdata->frame_size, false);
			mdelay(delayms);
			ptr += pdata->frame_size;
		}
	}

failed:
	if (imageBuff != NULL)
		vfree(imageBuff);
	
	return count;
}

static ssize_t ShowPattern_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "[PMOLED] put your patterns in /data/pmoled with pic prefix file name and 3 digital number from 000\n");
}

static ssize_t PanelID_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct ssd1362_platform_data *pdata = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d", pdata->panel_id);
}

static ssize_t Pattern_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	struct ssd1362_platform_data *pdata = dev_get_drvdata(dev);
	ssize_t err;
	u32 val;

	err = kstrtou32(buf, 10, &val);
	if (err)
		return err;
	pdata->atd_pattern = -1;
	if (pdata->power_disabled) {
		dev_err(&pdata->spi->dev, "[PMOLED] panel power disabled, turn on panel power before panel pattern test\n");
		return -1;
	}

	if (pdata->panel_id && (val>9 || val<0)) {
		dev_err(&pdata->spi->dev, "[PMOLED] color panel, please input a number between 0~9\n");
	} else if (!pdata->panel_id && (val>7 || val<0)){
		dev_err(&pdata->spi->dev, "[PMOLED] mono panel, please input a number between 0~7\n");
	} else {
		memset(pdata->frame_buffer, 0x00, pdata->frame_size);
		if (pdata->panel_id) { //Color Panel
			uint8_t *ptr = pdata->frame_buffer;
			uint8_t *tmp;
			uint16_t pattern = 0;
			uint8_t content[2];
			uint8_t r,g,b;
			int i, j;
			switch(val) {
				case 0:
					memset(pdata->frame_buffer, 0xFF, pdata->frame_size);
					break;
				case 1:
					break;
				case 2:
				case 3:
				case 4:
					if (val == 2)
						pattern = 0xF800;
					else if (val == 3)
						pattern = 0x07E0;
					else if (val == 4)
						pattern = 0x001F;
					tmp = ptr;
					for (i=0; i<256; i++) {
						if (val == 2) {
							memset(tmp, 0xF8, 1);
						} else if (val == 3) {
							memset(tmp, 0x07, 1);
							memset((tmp+1), 0xE0, 1);
						} else if (val == 4) {
							memset((tmp+1), 0x1F, 1);
						}
						tmp+=2;
					}
					tmp = ptr + 512;
					for (i=0; i <63; i++) {
						memcpy(tmp, ptr, 512);
						tmp += 512;
					}
				#if 0
					memset(ptr, 0xFF, 512);
					ptr += 512;
					if (val == 2)
						pattern = 0xF800;
					else if (val == 3)
						pattern = 0x07E0;
					else if (val == 4)
						pattern = 0x001F;
					tmp = ptr;
					for (i=0; i<256; i++) {
						if (val == 2) {
							memset(tmp, 0xF8, 1);
						} else if (val == 3) {
							memset(tmp, 0x07, 1);
							memset((tmp+1), 0xE0, 1);
						} else if (val == 4) {
							memset((tmp+1), 0x1F, 1);
						}
						tmp+=2;
					}
					#if 0
					tmp = ptr;
					if (val == 2) {
						tmp[0] = 0xFF;
						tmp[1] = 0xFF;
						tmp[510] = 0xF;
						tmp[511] = 0xFF;
					} else if (val == 3) {
						tmp[0] = 0xFF;
						tmp[1] = 0xFF;
						tmp[510] = 0xFF;
						tmp[511] = 0xFF;
					} else if (val == 4) {
						tmp[0] = 0xFF;
						tmp[1] = 0xFF;
						tmp[510] = 0xFF;
						tmp[511] = 0xFF;
					}
					#endif
					tmp = ptr + 512;
					for (i=0; i <61; i++) {
						memcpy(tmp, ptr, 512);
						tmp += 512;
					}
					memset(tmp, 0xFF, 512);
				#endif
					break;
				case 5:
					i=0;
					j=0;
					while (i<pdata->frame_size) {
						content[0] = 0xFF;
						content[1] = 0xFF;
						tmp = ptr+(i*2);
						memcpy(tmp, content, 2);
						i+=64;
						j++;
						if ((j%4) == 0)
							i += 1;
					}
					break;
				case 6:
					for (i=0; i<32; i++) {
						memset(ptr, 0xFF, 128);
						ptr += 512;
					}
					r = 1;
					g = 2;
					b = 1;
					content[0] = (r & 0x1F)<<3 | (g & 0x3F)>>3;
					content[1] = (g & 0x3F)<<5 | (b & 0x1F);
					for (i=0; i<32; i++) {
						tmp = ptr;
						for (j=0; j<64; j++) {
							memcpy(tmp, content, 2);
							tmp += 2;
						}
						ptr += 512;
					}
					break;
				case 7:
					r = 0x00;
					g = 0x00;
					b = 0x00;
					for (i=0; i<16; i++) {
						content[0] = (r & 0x1F)<<3 | (g & 0x3F)>>3;
						content[1] = (g & 0x3F)<<5 | (b & 0x1F);
						for (j=0; j<(512*4/2); j++) {
							memcpy(ptr, content, 2);
							ptr+=2;
						}
						#if 0
						if (i==0) {
							r += 1;
							g += 2;
							b += 1;
						} else {
							r += 2;
							g += 4;
							b += 2;
						}
						#else
						r += 1;
						g += 2;
						b += 1;
						#endif
					}
					break;
				case 8:
					for (i=0; i<63; i++) {
						memset(ptr, 0xFF, 52);
						memset((ptr+52+180), 0xFF, 52);
						memset((ptr+52+180+52+180), 0xFF, 52);
						ptr+=512;
					}
					break;
				case 9:
					for (i=0; i <63; i++) {
						memset((ptr+180), 0xFF, 156);
						ptr += 512;
					}
					break;
			}
		} else { //Mono Panel
			int i,j;
			uint8_t *ptr = pdata->frame_buffer;
			switch(val) {
				case 0:
					memset(pdata->frame_buffer, 0xFF, pdata->frame_size);
					break;
				case 1:
					break;
				case 2:
					memset(ptr, 0xFF, 128);
					ptr += 128;
					for (i=0; i<14; i++) {
						*ptr = 0x0F;
						ptr += 127;
						*ptr = 0xF0;
						ptr++;
					}
					for (i=0; i<42; i++) {
						int index = 0;
						index = i%4;
						if (index == 0) {
							*ptr = 0x0F;
							ptr++;
							for (j=0; j<63; j++) {
								*ptr = 0x00;
								ptr++;
								*ptr = 0x0F;
								ptr++;
							}
							*ptr = 0xF0;
							ptr++;
						} else if (index == 1) {
							*ptr = 0xFF;
							ptr++;
							for (j=0; j<63; j++) {
								*ptr = 0x00;
								ptr++;
								*ptr = 0xF0;
								ptr++;
							}
							*ptr = 0xF0;
							ptr++;
						} else if (index == 2) {
							*ptr = 0x0F;
							ptr++;
							for (j=0; j<63; j++) {
								*ptr = 0x0F;
								ptr++;
								*ptr = 0x00;
								ptr++;
							}
							*ptr = 0xFF;
							ptr++;
						} else if (index == 3) {
							*ptr = 0x0F;
							ptr++;
							for (j=0; j<63; j++) {
								*ptr = 0xF0;
								ptr++;
								*ptr = 0x00;
								ptr++;
							}
							*ptr = 0xF0;
							ptr++;
						}
					}
					for (i=0; i<5; i++) {
						*ptr = 0x0F;
						ptr += 127;
						*ptr = 0xF0;
						ptr++;
					}
					memset(ptr, 0xFF, 128);
					break;
				case 3:
					i=0;
					j=0;
					while (i<pdata->frame_size) {
						uint8_t tmp;
						if ((j/4)%2 == 0)
							tmp = 0x0F;
						else
							tmp = 0xF0;
						ptr[i] = tmp;
						i+=32;
						j++;
						if ((j%8) == 0)
							i++;
					}
					break;
				case 4:
					for (i=0; i<32; i++) {
						memset(ptr, 0xFF, 32);
						ptr+=128;
					}
					for (i=0; i<32; i++) {
						memset(ptr, 0x11, 32);
						ptr+=128;
					}
					break;
				case 5:
					for (i=0; i<16; i++) {
						uint8_t tmp = 0x00;
						tmp = (i & 0xFF) << 4 | (i & 0xFF);
						memset(ptr, tmp, 128*4);
						ptr += 128*4;
					}
					break;
				case 6:
					i=0;
					while (i < pdata->frame_size) {
						memset(ptr, 0xFF, 13);
						ptr += 13+45;
						i += 13+45;
						memset(ptr, 0xFF, 13);
						ptr += 13+44;
						i += 13+44;
						memset(ptr, 0xFF, 13);
						ptr += 13;
						i += 13;
					}
					break;
				case 7:
					for (i=0; i <63; i++) {
						uint8_t *tmp;
						tmp = ptr + 45;
						memset(tmp, 0xFF, 39);
						ptr += 128;
					}
					break;
			}
		}
		ssd1362_wfbc(pdata);
		err = ssd1362_spi_write(pdata, pdata->frame_buffer, pdata->frame_size, false);
		if (err)
			dev_err(&pdata->spi->dev, "[PMOLED] sending test buffer failed, err=%d\n", err);
		else
			pdata->atd_pattern = val;
	}

	return count;
}

static ssize_t DumpBuf_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	ssize_t err;
	u32 val;
	struct ssd1362_platform_data *pdata = dev_get_drvdata(dev);
	char path[256];
	u8 *ptr;

	err = kstrtou32(buf, 10, &val);
	if (err)
		return err;

	if (pdata->power_disabled) {
		dev_err(&pdata->spi->dev, "[PMOLED] panel power disabled, turn on panel power before panel pattern test\n");
		return -1;
	}

	if (val<0 || val>2) {
		dev_err(&pdata->spi->dev, "[PMOLED] tripple buffer, please input a number between 0~2\n");
	} else {
		int fd;
		ptr = pdata->frame_buffer+(val*pdata->frame_size);
		oldfs = get_fs();
		set_fs(KERNEL_DS);
		memset(path, 0x00, sizeof(path));
		snprintf(path, PAGE_SIZE, "/data/pmoled_dump%d.raw", val);
		fd = ksys_open(path, (O_CREAT | O_RDWR |O_TRUNC), 0);
		if( fd < 0 ) {
			dev_err(&pdata->spi->dev, "[PMOLED] open file failed %d\n");
            err = -1;
            set_fs(oldfs);
		}
		ksys_write(fd,ptr,pdata->frame_size);
        ksys_close(fd);
	}
	set_fs(oldfs);

	return count;
}

static ssize_t DumpBuf_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	return snprintf(buf, PAGE_SIZE, "[PMOLED] check the dump data in /data/pmoled\n");
}

static ssize_t Pattern_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct ssd1362_platform_data *pdata = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d", pdata->atd_pattern);
}

static DEVICE_ATTR(Power, 0664, Power_show, Power_store);
static DEVICE_ATTR(Test, 0664, Test_show, Test_store);
static DEVICE_ATTR(Backlight, 0664, Backlight_show, Backlight_store);
static DEVICE_ATTR(Commit, 0664, Commit_show, Commit_store);
static DEVICE_ATTR(FrameNum, 0664, FrameNum_show, FrameNum_store);
static DEVICE_ATTR(Delayms, 0664, Delayms_show, Delayms_store);
static DEVICE_ATTR(Repeat, 0664, Repeat_show, Repeat_store);
static DEVICE_ATTR(ShowPattern, 0664, ShowPattern_show, ShowPattern_store);
static DEVICE_ATTR_RO(PanelID);
static DEVICE_ATTR(Pattern, 0664, Pattern_show, Pattern_store);
static DEVICE_ATTR(DumpBuf, 0664, DumpBuf_show, DumpBuf_store);

static struct attribute *ssd1362_attrs[] = {
	&dev_attr_Power.attr,
	&dev_attr_Test.attr,
	&dev_attr_Backlight.attr,
	&dev_attr_Commit.attr,
	&dev_attr_FrameNum.attr,
	&dev_attr_Delayms.attr,
	&dev_attr_Repeat.attr,
	&dev_attr_ShowPattern.attr,
	&dev_attr_PanelID.attr,
	&dev_attr_Pattern.attr,
	&dev_attr_DumpBuf.attr,
	NULL
};

static const struct attribute_group ssd1362_attr_group = {
	.attrs = ssd1362_attrs,
};

static int remap_pfn_mmap(struct file *file, struct vm_area_struct *vma)
{
    //unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
    unsigned long pfn_start = page_to_pfn(start_page) + vma->vm_pgoff;
    unsigned long virt_start = (unsigned long)page_address(start_page);
    unsigned long size = vma->vm_end - vma->vm_start;
    int err = 0;

    err = remap_pfn_range(vma, vma->vm_start, pfn_start, size, vma->vm_page_prot);
    if (err)
        pr_err("%s: remap_pfn_range failed at [0x%lx  0x%lx]\n",
            __func__, vma->vm_start, vma->vm_end);
    else
        pr_err("weiwei %s: map 0x%lx to 0x%lx, size: 0x%lx\n", __func__, virt_start,
            vma->vm_start, size);

    return err;
}

static int open(struct inode *inode, struct file *filp)
{
    pr_info("[PMOLED] file open\n");
    return 0;
}

static ssize_t read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
    pr_info("[PMOLED] file read\n");
    return 0;
}

static ssize_t write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{

    pr_info("[PMOLED] file write\n");
	return 0;
}

static int release(struct inode *inode, struct file *filp)
{
    pr_info("[PMOLED] file release\n");
    return 0;
}

static const struct file_operations fops = {
    .mmap = remap_pfn_mmap,
    .open = open,
    .release = release,
    .read = read,
    .write = write,
};

extern bool g_Charger_mode;
static int ssd1362_probe(struct spi_device *spi)
{
	int err = 0;
	struct ssd1362_platform_data *pdata;

	pdata = devm_kzalloc(&spi->dev, sizeof(struct ssd1362_platform_data), GFP_KERNEL);
	if (!pdata) {
		dev_err(&spi->dev, "[PMOLED] Failed to allocate memory\n");
		return -ENOMEM;
	}

	//setup GPIO properly first, even we don't have pmoled on the device
	pdata->panel_id = -1;
	pdata->power_disabled = false;
	mutex_init(&pdata->mutex);

	err = ssd1362_parse_dt(&spi->dev, pdata);
	if (err)
		dev_err(&spi->dev, "[PMOLED] parse dt failed, err=%d\n", err);

	ssd1362_power_control(pdata, false);

	if ((g_ASUS_hwID >= HW_REV_ER) && (g_ASUS_bcID != BC_ID_PMOLED)) {
		dev_err(&spi->dev, "[PMOLED] not pmoled back cover, HWID=%d, BCID=%d\n", g_ASUS_hwID, g_ASUS_bcID);
		devm_kfree(&spi->dev, pdata);
		gpio_direction_input(76); //for power saving
		gpio_direction_input(77); //for power saving
		gpio_direction_input(78); //for power saving
		gpio_direction_input(79); //for power saving
		return -ENODEV;
	}

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0 | SPI_TX_QUAD;
	if (pdata->panel_id)
		spi->max_speed_hz = 15000000;
	else
		spi->max_speed_hz = 7500000;

	err = spi_setup(spi);	
	if (err)
		dev_err(&spi->dev, "[PMOLED] spi setup failed, err=%d\n", err);

	spi_set_drvdata(spi, pdata);
	pdata->spi = spi;

	pdata->backlight_level = 0;
	pdata->buffer_offset = ALIGN(pdata->frame_size,PAGE_SIZE);
	printk("[PMOLED] buffer_offset = %d\n", pdata->buffer_offset);
	
	err = sysfs_create_group(&spi->dev.kobj, &ssd1362_attr_group);

	if (pdata->panel_id)
		pdata->frame_size = COLOR_FRAME_SIZE;
	else
		pdata->frame_size = FRAME_SIZE;

	pdata->fb_page = alloc_pages(GFP_DMA, get_order(pdata->frame_size*BUFFER_COUNT));
	if (pdata->fb_page == NULL) {
		err = -1;
		dev_err(&spi->dev, "[PMOLED] allocate page failed, buf size=%d\n", (pdata->frame_size*BUFFER_COUNT));
	} else {
		start_page = pdata->fb_page;
		pdata->frame_buffer = (u8 *)page_address(pdata->fb_page);
		memset(pdata->frame_buffer, 0x00, (pdata->frame_size*BUFFER_COUNT));
	}

	proc_create(mmfile, S_IRUGO | S_IWUGO , NULL, &fops);

	if (pdata->frame_buffer == NULL) {
		err = -1;
		dev_err(&spi->dev, "[PMOLED] allocate framebuffer failed, buf size=%d\n", (pdata->frame_size*BUFFER_COUNT));
	}

	pdata->fb_clean = kmalloc(pdata->frame_size, GFP_DMA);

	if (pdata->fb_clean == NULL) {
		err = -1;
		dev_err(&spi->dev, "[PMOLED] allocate clean framebuffer failed, buf size=%d\n", pdata->frame_size);
	} else {
		memset(pdata->fb_clean, 0x00, pdata->frame_size);
	}

	if (!g_Charger_mode)
		show_logo(pdata);

	return err;
}

static int ssd1362_remove(struct spi_device *spi)
{
	struct ssd1362_platform_data *pdata = spi_get_drvdata(spi);

	pinctrl_select_state(pdata->pinctrl, pdata->pins_suspend);
	
	sysfs_remove_group(&spi->dev.kobj, &ssd1362_attr_group);

	if (pdata->fb_page != NULL)
		__free_pages(pdata->fb_page,get_order(pdata->frame_size*BUFFER_COUNT));
	if (pdata->fb_clean != NULL)
		kfree(pdata->fb_clean);

	remove_proc_entry(mmfile, NULL);

	return 0;
}

static const struct of_device_id ssd1362_of_match[] = {
	{ .compatible = "futaba,ssd1362", },
	{ }
};
MODULE_DEVICE_TABLE(of, ssd1362_of_match);

static const struct spi_device_id ssd1362_ids[] = {
	{ "ssd1362", 0 },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(spi, ssd1362_ids);

static struct spi_driver ssd1362_driver = {
	.driver = {
		.name		= "ssd1362",
		.of_match_table = ssd1362_of_match,
	},
	.probe		= ssd1362_probe,
	.remove		= ssd1362_remove,
	.id_table	= ssd1362_ids,
};

module_spi_driver(ssd1362_driver);

MODULE_DESCRIPTION("PMOLED SSD1362 SPI Drover");
MODULE_AUTHOR("Mickey Liao");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:ssd1362");
