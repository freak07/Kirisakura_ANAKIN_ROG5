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

#include <linux/fs.h>
#include <linux/syscalls.h>

#define RGB_MAX 21   //for rainbow color setting

struct inbox_drvdata {
	struct led_classdev led;
};

struct hidraw *rog5_inbox_hidraw;
EXPORT_SYMBOL_GPL(rog5_inbox_hidraw);

struct mutex ms51_mutex;
struct mutex update_lock;

u8 g_mode = 0;
int g_led_on = 0;
int apply_state = 0;
int mode2_state = 0;
u8 key_state = 0;

static u32 g_red_max;
static u32 g_green_max;
static u32 g_blue_max;
static u32 g_red;
static u32 g_green;
static u32 g_blue;
static u32 g_speed;

extern bool g_Charger_mode;

struct delayed_work	disable_autosuspend_work;

extern int usbhid_set_raw_report(struct hid_device *hid, unsigned int reportnum,
				 __u8 *buf, size_t count, unsigned char rtype);
extern int usbhid_get_raw_report(struct hid_device *hid, unsigned char report_number, __u8 *buf, size_t count,
		unsigned char report_type);


static int asus_usb_hid_write_aprom(u8 *data, int len)
{
	struct hid_device *hdev;
	int ret = 0;
	char *buffer;

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}
	
	buffer = kzalloc(0x06, GFP_KERNEL);
	memset(buffer,0,6);
	hdev = rog5_inbox_hidraw->hid;

	buffer[0] = 0x0B;
	buffer[1] = 0x2;
	buffer[2] = len;
	memcpy(&(buffer[3]), data, len);

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("usbhid_set_raw_report fail: ret = %d\n", ret);
	hid_hw_power(hdev, PM_HINT_NORMAL);
	kfree(buffer);

	return ret;
}

static int asus_usb_hid_write_fw(u8 *data, int len)
{
	struct hid_device *hdev;
	int ret = 0;
	u8 *buffer;

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}
	
	buffer = kzalloc(52, GFP_KERNEL);
	memset(buffer,0,52);
	hdev = rog5_inbox_hidraw->hid;

	buffer[0] = 0x0C;
	buffer[1] = 0x2;
	buffer[2] = len;
	buffer[3] = 0;
	memcpy(&(buffer[4]), data, len);

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, 52, HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("usbhid_set_raw_report fail: ret = %d\n", ret);
	hid_hw_power(hdev, PM_HINT_NORMAL);
	kfree(buffer);

	return ret;
}

static ssize_t gpio8_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val;
	int ret;
	struct hid_device *hdev;
	u8 *buffer;

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	printk("[ROG5_INBOX] %s, reg_val %d\n", __func__, reg_val);

	buffer = kzalloc(2, GFP_KERNEL);
	memset(buffer,0,2);

	buffer[0] = 0xA;
	if (reg_val > 0)
		buffer[1] = 0x01;
	else
		buffer[1] = 0x00;
		
	hdev = rog5_inbox_hidraw->hid;

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	hid_hw_power(hdev, PM_HINT_NORMAL);

	if (ret < 0)
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", ret);

	kfree(buffer);
	
	return count;
}

static ssize_t gpio9_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer;
	u8 key_state_tmp1, key_state_tmp2;
	
	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}
	
	key_state_tmp1 = key_state;
	
	buffer = kzalloc(2, GFP_KERNEL);
	memset(buffer, 0, 2);
	
	hdev = rog5_inbox_hidraw->hid;

	buffer[0] = 0x0A;
	buffer[1] = 0x09;

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("[ROG5_INBOX] usbhid_set_raw_report fail\n");
	hid_hw_power(hdev, PM_HINT_NORMAL);

	kfree(buffer);

	key_state_tmp2 = key_state;
	key_state = key_state_tmp1;
	printk("[ROG5_INBOX] gpio9_show : %x\n", key_state_tmp2);
	return snprintf(buf, PAGE_SIZE,"%x\n", key_state_tmp2);
}

static ssize_t gpio9_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val;
	int ret;
	struct hid_device *hdev;
	u8 *buffer;

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	printk("[ROG5_INBOX] %s, reg_val %d\n", __func__, reg_val);

	buffer = kzalloc(2, GFP_KERNEL);
	memset(buffer,0,2);

	buffer[0] = 0xA;
	if (reg_val > 0)
		buffer[1] = 0x91;
	else
		buffer[1] = 0x90;
		
	hdev = rog5_inbox_hidraw->hid;

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	hid_hw_power(hdev, PM_HINT_NORMAL);

	if (ret < 0)
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", ret);

	kfree(buffer);
	
	return count;
}

static ssize_t gpio10_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer;
	u8 key_state_tmp1, key_state_tmp2;

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}
	
	key_state_tmp1 = key_state;
	
	buffer = kzalloc(2, GFP_KERNEL);
	memset(buffer, 0, 2);
	
	hdev = rog5_inbox_hidraw->hid;

	buffer[0] = 0x0A;
	buffer[1] = 0x0A;

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("[ROG5_INBOX] usbhid_set_raw_report fail\n");
	hid_hw_power(hdev, PM_HINT_NORMAL);

	kfree(buffer);

	key_state_tmp2 = key_state;
	key_state = key_state_tmp1;
	printk("[ROG5_INBOX] gpio10_show : %x\n", key_state_tmp2);
	return snprintf(buf, PAGE_SIZE,"%x\n", key_state_tmp2);

}

static ssize_t gpio10_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val;
	int ret;
	struct hid_device *hdev;
	u8 *buffer;

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	printk("[ROG5_INBOX] %s, reg_val %d\n", __func__, reg_val);

	buffer = kzalloc(2, GFP_KERNEL);
	memset(buffer,0,2);

	buffer[0] = 0xA;
	if (reg_val > 0)
		buffer[1] = 0xA1;
	else
		buffer[1] = 0xA0;
		
	hdev = rog5_inbox_hidraw->hid;

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	hid_hw_power(hdev, PM_HINT_NORMAL);

	if (ret < 0)
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", ret);

	kfree(buffer);
	
	return count;
}

static ssize_t gpio11_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer;
	u8 key_state_tmp1, key_state_tmp2;

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}
	
	key_state_tmp1 = key_state;
	
	buffer = kzalloc(2, GFP_KERNEL);
	memset(buffer, 0, 2);
	
	hdev = rog5_inbox_hidraw->hid;

	buffer[0] = 0x0A;
	buffer[1] = 0x0B;

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("[ROG5_INBOX] usbhid_set_raw_report fail\n");
	hid_hw_power(hdev, PM_HINT_NORMAL);

	kfree(buffer);

	key_state_tmp2 = key_state;
	key_state = key_state_tmp1;
	printk("[ROG5_INBOX] gpio11_show : %x\n", key_state_tmp2);
	return snprintf(buf, PAGE_SIZE,"%x\n", key_state_tmp2);

}

static ssize_t gpio11_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val;
	int ret;
	struct hid_device *hdev;
	u8 *buffer;

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	printk("[ROG5_INBOX] %s, reg_val %d\n", __func__, reg_val);

	buffer = kzalloc(2, GFP_KERNEL);
	memset(buffer,0,2);

	buffer[0] = 0xA;
	if (reg_val > 0)
		buffer[1] = 0xB1;
	else
		buffer[1] = 0xB0;
		
	hdev = rog5_inbox_hidraw->hid;

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	hid_hw_power(hdev, PM_HINT_NORMAL);

	if (ret < 0)
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", ret);

	kfree(buffer);
	
	return count;
}

static ssize_t led_test_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned char data[3] = {0};
	u32 reg_val;
	int ret = 0;

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	printk("[ROG5_INBOX] %s, reg_val %d\n", __func__, reg_val);
	
	data[0] = 0x80;
	data[1] = 0x21;
	data[2] = 0x04;

	g_mode = 0x04;
	
	ret = asus_usb_hid_write_aprom(data, 3);
	if (ret < 0)
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", ret);

	msleep(10);
	
	data[0] = 0x80;
	data[1] = 0x2F;
	data[2] = 0x01;
		
	ret = asus_usb_hid_write_aprom(data, 3);
	if (ret < 0)
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", ret);

	return count;
}

static ssize_t red_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	//printk("[ROG5_INBOX] %s reg_val = %d.\n", __func__, reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*g_red_max, 255);
	//printk("[ROG5_INBOX] %s tmp = %d.\n", __func__, tmp);

	data[0] = 0x80;
	data[1] = 0x10;
	data[2] = tmp;

	g_red = tmp;

	err = asus_usb_hid_write_aprom(data, 3);
	if (err < 0)
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);

	msleep(10);
	
	data[1] = 0x13;
	err = asus_usb_hid_write_aprom(data, 3);
	if (err < 0)
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);

	return count;
}

static ssize_t red_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer, *buffer_ret;
	u8 buffer_ret_tmp[3] = {0};

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}
	
	buffer = kzalloc(0x06, GFP_KERNEL);
	memset(buffer, 0, 6);
	buffer_ret = kzalloc(3, GFP_KERNEL);
	memset(buffer_ret, 0, 3);
	
	hdev = rog5_inbox_hidraw->hid;

	buffer[0] = 0x0B;
	buffer[1] = 0x1;
	buffer[2] = 0x2;
	buffer[3] = 0x80;
	buffer[4] = 0x10;
	buffer[5] = 0x00;

	hid_hw_power(hdev, PM_HINT_FULLON);
	
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("[ROG5_INBOX] usbhid_set_raw_report fail\n");

	msleep(10);

	ret = usbhid_get_raw_report(hdev, buffer[0], buffer_ret, sizeof(buffer_ret), HID_INPUT_REPORT);
	if (ret < 0)
		printk("[ROG5_INBOX] usbhid_get_raw_report fail\n");

	//printk("[ROG5_INBOX] red_pwm_show : %x, %x, %x\n", buffer_ret[0], buffer_ret[1], buffer_ret[2]);

	hid_hw_power(hdev, PM_HINT_NORMAL);

	memcpy(buffer_ret_tmp, buffer_ret, 3);
	kfree(buffer);
	kfree(buffer_ret);

	return snprintf(buf, PAGE_SIZE,"%d\n", buffer_ret_tmp[1]);
}

static ssize_t green_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	//printk("[ROG5_INBOX] %s reg_val = %d.\n", __func__, reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*g_red_max, 255);
	//printk("[ROG5_INBOX] %s tmp = %d.\n", __func__, tmp);

	data[0] = 0x80;
	data[1] = 0x11;
	data[2] = tmp;

	g_green = tmp;

	err = asus_usb_hid_write_aprom(data, 3);
	if (err < 0)
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);

	msleep(10);
	
	data[1] = 0x14;
	err = asus_usb_hid_write_aprom(data, 3);
	if (err < 0)
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);

	return count;
}

static ssize_t green_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer, *buffer_ret;
	u8 buffer_ret_tmp[3] = {0};

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}
	
	buffer = kzalloc(0x06, GFP_KERNEL);
	memset(buffer, 0, 6);
	buffer_ret = kzalloc(3, GFP_KERNEL);
	memset(buffer_ret, 0, 3);
	
	hdev = rog5_inbox_hidraw->hid;

	buffer[0] = 0x0B;
	buffer[1] = 0x1;
	buffer[2] = 0x2;
	buffer[3] = 0x80;
	buffer[4] = 0x11;
	buffer[5] = 0x00;

	hid_hw_power(hdev, PM_HINT_FULLON);
	
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("[ROG5_INBOX] usbhid_set_raw_report fail\n");

	msleep(10);

	ret = usbhid_get_raw_report(hdev, buffer[0], buffer_ret, sizeof(buffer_ret), HID_INPUT_REPORT);
	if (ret < 0)
		printk("[ROG5_INBOX] usbhid_get_raw_report fail\n");

	//printk("[ROG5_INBOX] green_pwm_show : %x, %x, %x\n", buffer_ret[0], buffer_ret[1], buffer_ret[2]);

	hid_hw_power(hdev, PM_HINT_NORMAL);

	memcpy(buffer_ret_tmp, buffer_ret, 3);
	kfree(buffer);
	kfree(buffer_ret);

	return snprintf(buf, PAGE_SIZE,"%d\n", buffer_ret_tmp[1]);
}

static ssize_t blue_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	//printk("[ROG5_INBOX] %s reg_val = %d.\n", __func__, reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*g_red_max, 255);
	//printk("[ROG5_INBOX] %s tmp = %d.\n", __func__, tmp);

	data[0] = 0x80;
	data[1] = 0x12;
	data[2] = tmp;

	g_blue = tmp;

	err = asus_usb_hid_write_aprom(data, 3);
	if (err < 0)
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);

	msleep(10);
	
	data[1] = 0x15;
	err = asus_usb_hid_write_aprom(data, 3);
	if (err < 0)
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);

	return count;
}

static ssize_t blue_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer, *buffer_ret;
	u8 buffer_ret_tmp[3] = {0};

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}
	
	buffer = kzalloc(0x06, GFP_KERNEL);
	memset(buffer, 0, 6);
	buffer_ret = kzalloc(3, GFP_KERNEL);
	memset(buffer_ret, 0, 3);
	
	hdev = rog5_inbox_hidraw->hid;

	buffer[0] = 0x0B;
	buffer[1] = 0x1;
	buffer[2] = 0x2;
	buffer[3] = 0x80;
	buffer[4] = 0x12;
	buffer[5] = 0x00;

	hid_hw_power(hdev, PM_HINT_FULLON);
	
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("[ROG5_INBOX] usbhid_set_raw_report fail\n");

	msleep(10);

	ret = usbhid_get_raw_report(hdev, buffer[0], buffer_ret, sizeof(buffer_ret), HID_INPUT_REPORT);
	if (ret < 0)
		printk("[ROG5_INBOX] usbhid_get_raw_report fail\n");

	//printk("[ROG5_INBOX] blue_pwm_show : %x, %x, %x\n", buffer_ret[0], buffer_ret[1], buffer_ret[2]);

	hid_hw_power(hdev, PM_HINT_NORMAL);
	
	memcpy(buffer_ret_tmp, buffer_ret, 3);
	kfree(buffer);
	kfree(buffer_ret);

	return snprintf(buf, PAGE_SIZE,"%d\n", buffer_ret_tmp[1]);
}

static ssize_t red1_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	//printk("[ROG5_INBOX] %s reg_val = %d.\n", __func__, reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*g_red_max, 255);
	//printk("[ROG5_INBOX] %s tmp = %d.\n", __func__, tmp);

	data[0] = 0x80;
	data[1] = 0x13;
	data[2] = tmp;

	err = asus_usb_hid_write_aprom(data, 3);
	if (err < 0)
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);

	return count;
}

static ssize_t red1_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer, *buffer_ret;
	u8 buffer_ret_tmp[3] = {0};

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}
	
	buffer = kzalloc(0x06, GFP_KERNEL);
	memset(buffer, 0, 6);
	buffer_ret = kzalloc(3, GFP_KERNEL);
	memset(buffer_ret, 0, 3);
	
	hdev = rog5_inbox_hidraw->hid;

	buffer[0] = 0x0B;
	buffer[1] = 0x1;
	buffer[2] = 0x2;
	buffer[3] = 0x80;
	buffer[4] = 0x13;
	buffer[5] = 0x00;

	hid_hw_power(hdev, PM_HINT_FULLON);
	
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("[ROG5_INBOX] usbhid_set_raw_report fail\n");

	msleep(10);

	ret = usbhid_get_raw_report(hdev, buffer[0], buffer_ret, sizeof(buffer_ret), HID_INPUT_REPORT);
	if (ret < 0)
		printk("[ROG5_INBOX] usbhid_get_raw_report fail\n");

	//printk("[ROG5_INBOX] red1_pwm_show : %x, %x, %x\n", buffer_ret[0], buffer_ret[1], buffer_ret[2]);

	hid_hw_power(hdev, PM_HINT_NORMAL);

	memcpy(buffer_ret_tmp, buffer_ret, 3);
	kfree(buffer);
	kfree(buffer_ret);

	return snprintf(buf, PAGE_SIZE,"%d\n", buffer_ret_tmp[1]);
}

static ssize_t green1_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	//printk("[ROG5_INBOX] %s reg_val = %d.\n", __func__, reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*g_red_max, 255);
	//printk("[ROG5_INBOX] %s tmp = %d.\n", __func__, tmp);

	data[0] = 0x80;
	data[1] = 0x14;
	data[2] = tmp;

	err = asus_usb_hid_write_aprom(data, 3);
	if (err < 0)
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);

	return count;
}

static ssize_t green1_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer, *buffer_ret;
	u8 buffer_ret_tmp[3] = {0};

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}
	
	buffer = kzalloc(0x06, GFP_KERNEL);
	memset(buffer, 0, 6);
	buffer_ret = kzalloc(3, GFP_KERNEL);
	memset(buffer_ret, 0, 3);
	
	hdev = rog5_inbox_hidraw->hid;

	buffer[0] = 0x0B;
	buffer[1] = 0x1;
	buffer[2] = 0x2;
	buffer[3] = 0x80;
	buffer[4] = 0x14;
	buffer[5] = 0x00;

	hid_hw_power(hdev, PM_HINT_FULLON);
	
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("[ROG5_INBOX] usbhid_set_raw_report fail\n");

	msleep(10);

	ret = usbhid_get_raw_report(hdev, buffer[0], buffer_ret, sizeof(buffer_ret), HID_INPUT_REPORT);
	if (ret < 0)
		printk("[ROG5_INBOX] usbhid_get_raw_report fail\n");

	//printk("[ROG5_INBOX] green1_pwm_show : %x, %x, %x\n", buffer_ret[0], buffer_ret[1], buffer_ret[2]);

	hid_hw_power(hdev, PM_HINT_NORMAL);

	memcpy(buffer_ret_tmp, buffer_ret, 3);
	kfree(buffer);
	kfree(buffer_ret);

	return snprintf(buf, PAGE_SIZE,"%d\n", buffer_ret_tmp[1]);
}

static ssize_t blue1_pwm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 reg_val, tmp;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &reg_val);
	if (ret)
		return count;

	//printk("[ROG5_INBOX] %s reg_val = %d.\n", __func__, reg_val);
	tmp = DIV_ROUND_CLOSEST(reg_val*g_red_max, 255);
	//printk("[ROG5_INBOX] %s tmp = %d.\n", __func__, tmp);

	data[0] = 0x80;
	data[1] = 0x15;
	data[2] = tmp;

	err = asus_usb_hid_write_aprom(data, 3);
	if (err < 0)
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);

	return count;
}

static ssize_t blue1_pwm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer, *buffer_ret;
	u8 buffer_ret_tmp[3] = {0};

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}
	
	buffer = kzalloc(0x06, GFP_KERNEL);
	memset(buffer, 0, 6);
	buffer_ret = kzalloc(3, GFP_KERNEL);
	memset(buffer_ret, 0, 3);

	hdev = rog5_inbox_hidraw->hid;

	buffer[0] = 0x0B;
	buffer[1] = 0x1;
	buffer[2] = 0x2;
	buffer[3] = 0x80;
	buffer[4] = 0x15;
	buffer[5] = 0x00;

	hid_hw_power(hdev, PM_HINT_FULLON);
	
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("[ROG5_INBOX] usbhid_set_raw_report fail\n");

	msleep(10);

	ret = usbhid_get_raw_report(hdev, buffer[0], buffer_ret, sizeof(buffer_ret), HID_INPUT_REPORT);
	if (ret < 0)
		printk("[ROG5_INBOX] usbhid_get_raw_report fail\n");

	//printk("[ROG5_INBOX] blue1_pwm_show : %x, %x, %x\n", buffer_ret[0], buffer_ret[1], buffer_ret[2]);

	hid_hw_power(hdev, PM_HINT_NORMAL);

	memcpy(buffer_ret_tmp, buffer_ret, 3);
	kfree(buffer);
	kfree(buffer_ret);

	return snprintf(buf, PAGE_SIZE,"%d\n", buffer_ret_tmp[1]);
}

static ssize_t apply_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE,"%d\n", apply_state);
}

static ssize_t apply_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret) {
		apply_state = -1;
		return count;
	}

	//printk("[ROG5_INBOX] apply_store: %d\n", val);
	apply_state = 0;
	
	if (val) {
		data[0] = 0x80;
		data[1] = 0x2F;
		data[2] = 0x01;

		err = asus_usb_hid_write_aprom(data, 3);
		if (err < 0) {
			printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);
			apply_state = -1;
			return count;
		}

		printk("[ROG5_INBOX] Send apply. RGB:%d %d %d, mode:%d, speed:%d, led_on:%d\n", g_red, g_green, g_blue, g_mode, g_speed, g_led_on);
	}
	else {
		printk("[ROG5_INBOX] don't send apply command\n");
	}

	return count;
}

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer, *buffer_ret;
	u8 buffer_ret_tmp[3] = {0};

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}
	
	buffer = kzalloc(0x06, GFP_KERNEL);
	memset(buffer, 0, 6);
	buffer_ret = kzalloc(3, GFP_KERNEL);
	memset(buffer_ret, 0, 3);
	
	hdev = rog5_inbox_hidraw->hid;

	buffer[0] = 0x0B;
	buffer[1] = 0x1;
	buffer[2] = 0x2;
	buffer[3] = 0x80;
	buffer[4] = 0x21;
	buffer[5] = 0x00;

	hid_hw_power(hdev, PM_HINT_FULLON);
	
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("usbhid_set_raw_report fail\n");

	msleep(10);

	ret = usbhid_get_raw_report(hdev, buffer[0], buffer_ret, sizeof(buffer_ret), HID_INPUT_REPORT);
	if (ret < 0)
		printk("usbhid_get_raw_report fail\n");

	//printk("[ROG5_INBOX] mode : %x, %x, %x\n", buffer_ret[0], buffer_ret[1], buffer_ret[2]);

	hid_hw_power(hdev, PM_HINT_NORMAL);

	memcpy(buffer_ret_tmp, buffer_ret, 3);
	kfree(buffer);
	kfree(buffer_ret);

	return snprintf(buf, PAGE_SIZE,"0x%02x\n", buffer_ret_tmp[1]);
}

static ssize_t set_frame(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	if (val > 255){
		printk("[ROG5_INBOX] Frame should not over 255.\n");
		return count;
	}
	
	data[0] = 0x80;
	data[1] = 0xF2;
	data[2] = val;

	err = asus_usb_hid_write_aprom(data, 3);
	if (err < 0) {
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);
		return count;
	}
		
	//printk("[ROG5_INBOX] set_frame: %d\n", val);
	
	return count;
}

static ssize_t get_frame(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer, *buffer_ret;
	u8 buffer_ret_tmp[3] = {0};

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}
	
	buffer = kzalloc(0x06, GFP_KERNEL);
	memset(buffer, 0, 6);
	buffer_ret = kzalloc(3, GFP_KERNEL);
	memset(buffer_ret, 0, 3);
	
	hdev = rog5_inbox_hidraw->hid;

	buffer[0] = 0x0B;
	buffer[1] = 0x1;
	buffer[2] = 0x2;
	buffer[3] = 0x80;
	buffer[4] = 0xF3;
	buffer[5] = 0x00;

	hid_hw_power(hdev, PM_HINT_FULLON);
	
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("[ROG5_INBOX] usbhid_set_raw_report fail\n");

	msleep(10);

	ret = usbhid_get_raw_report(hdev, buffer[0], buffer_ret, sizeof(buffer_ret), HID_INPUT_REPORT);
	if (ret < 0)
		printk("[ROG5_INBOX] usbhid_get_raw_report fail\n");

	//printk("[ROG5_INBOX] get_frame : %x, %x, %x\n", buffer_ret[0], buffer_ret[1], buffer_ret[2]);

	hid_hw_power(hdev, PM_HINT_NORMAL);

	memcpy(buffer_ret_tmp, buffer_ret, 3);
	kfree(buffer);
	kfree(buffer_ret);

	return snprintf(buf, PAGE_SIZE,"%d\n", buffer_ret_tmp[1]);
}

static ssize_t set_speed(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	if (val != 254 && val != 255 && val != 0){
		printk("[ROG5_INBOX] speed should be 0, 255, 254\n");
		return count;
	}
	
	data[0] = 0x80;
	data[1] = 0x22;
	data[2] = val;

	err = asus_usb_hid_write_aprom(data, 3);
	if (err < 0) {
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);
		return count;
	}

	g_speed = val;
	//printk("[ROG5_INBOX] set_speed: %d\n", val);
	
	return count;
}

static ssize_t get_speed(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer, *buffer_ret;
	u8 buffer_ret_tmp[3] = {0};

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}
	
	buffer = kzalloc(0x06, GFP_KERNEL);
	memset(buffer, 0, 6);
	buffer_ret = kzalloc(3, GFP_KERNEL);
	memset(buffer_ret, 0, 3);
	
	hdev = rog5_inbox_hidraw->hid;

	buffer[0] = 0x0B;
	buffer[1] = 0x1;
	buffer[2] = 0x2;
	buffer[3] = 0x80;
	buffer[4] = 0x22;
	buffer[5] = 0x00;

	hid_hw_power(hdev, PM_HINT_FULLON);
	
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("[ROG5_INBOX] usbhid_set_raw_report fail\n");

	msleep(10);

	ret = usbhid_get_raw_report(hdev, buffer[0], buffer_ret, sizeof(buffer_ret), HID_INPUT_REPORT);
	if (ret < 0)
		printk("[ROG5_INBOX] usbhid_get_raw_report fail\n");

	//printk("[ROG5_INBOX] get_speed : %x, %x, %x\n", buffer_ret[0], buffer_ret[1], buffer_ret[2]);

	hid_hw_power(hdev, PM_HINT_NORMAL);

	memcpy(buffer_ret_tmp, buffer_ret, 3);
	kfree(buffer);
	kfree(buffer_ret);

	return snprintf(buf, PAGE_SIZE,"%d\n", buffer_ret_tmp[1]);
}

static ssize_t set_cali_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int red_val = 0, green_val = 0, blue_val = 0;

	sscanf(buf, "%d %d %d", &red_val, &green_val, &blue_val);
	printk("[ROG5_INBOX] set_cali_data, %d, %d, %d\n", red_val, green_val, blue_val);

	g_red_max = red_val;
	g_green_max = green_val;
	g_blue_max = blue_val;

	return count;
}

static ssize_t get_cali_data(struct device *dev, struct device_attribute *attr,char *buf)
{
	printk("[ROG5_INBOX] R:%d, G:%d, B:%d\n", g_red_max, g_green_max, g_blue_max);
	return snprintf(buf, PAGE_SIZE,"R:%d, G:%d, B:%d\n", g_red_max, g_green_max, g_blue_max);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	//printk("[ROG5_INBOX] mode_store: %d\n", val);

	data[0] = 0x80;
	data[1] = 0x21;
	data[2] = val;

	g_mode = val;

	err = asus_usb_hid_write_aprom(data, 3);
	if (err < 0)
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);

	return count;
}

static ssize_t fw_mode_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer, *buffer_ret;
	u8 buffer_ret_tmp[3] = {0};

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}
	
	buffer = kzalloc(0x06, GFP_KERNEL);
	memset(buffer, 0, 6);
	buffer_ret = kzalloc(3, GFP_KERNEL);
	memset(buffer_ret, 0, 3);

	hdev = rog5_inbox_hidraw->hid;

	buffer[0] = 0x0B;
	buffer[1] = 0x1;
	buffer[2] = 0x1;
	buffer[3] = 0xCA;
	buffer[4] = 0x00;
	buffer[5] = 0x00;

	hid_hw_power(hdev, PM_HINT_FULLON);
	
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("usbhid_set_raw_report fail\n");

	msleep(10);

	ret = usbhid_get_raw_report(hdev, buffer[0], buffer_ret, sizeof(buffer_ret), HID_INPUT_REPORT);
	if (ret < 0)
		printk("usbhid_get_raw_report fail\n");

	printk("[ROG5_INBOX] fw version : %x, %x, %x\n", buffer_ret[0], buffer_ret[1], buffer_ret[2]);

	hid_hw_power(hdev, PM_HINT_NORMAL);

	memcpy(buffer_ret_tmp, buffer_ret, 3);
	kfree(buffer);
	kfree(buffer_ret);

	return snprintf(buf, PAGE_SIZE,"%d\n", buffer_ret_tmp[1]);
}

static ssize_t fw_ver_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer, *buffer_ret;
	u8 buffer_ret_tmp[3] = {0};

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}
	
	buffer = kzalloc(0x06, GFP_KERNEL);
	memset(buffer, 0, 6);
	buffer_ret = kzalloc(3, GFP_KERNEL);
	memset(buffer_ret, 0, 3);
	
	hdev = rog5_inbox_hidraw->hid;

	buffer[0] = 0x0B;
	buffer[1] = 0x1;
	buffer[2] = 0x2;
	buffer[3] = 0xCB;
	buffer[4] = 0x01;
	buffer[5] = 0x00;

	hid_hw_power(hdev, PM_HINT_FULLON);
	
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("usbhid_set_raw_report fail\n");

	msleep(10);

	ret = usbhid_get_raw_report(hdev, buffer[0], buffer_ret, sizeof(buffer_ret), HID_INPUT_REPORT);
	if (ret < 0)
		printk("usbhid_get_raw_report fail\n");

	printk("[ROG5_INBOX] fw version : %x, %x, %x\n", buffer_ret[0], buffer_ret[1], buffer_ret[2]);

	hid_hw_power(hdev, PM_HINT_NORMAL);

	memcpy(buffer_ret_tmp, buffer_ret, 3);
	kfree(buffer);
	kfree(buffer_ret);

	return snprintf(buf, PAGE_SIZE,"0x%02x%02x\n", buffer_ret_tmp[1], buffer_ret_tmp[2]);
}

/*
static int ms51_fw_erase(struct i2c_client *client)
{
	int err = 0;
	unsigned char buf[2] = {0};
	buf[0] = 0xA3;
	buf[1] = 0x1;

	printk("[AURA_MS51_INBOX] ms51_fw_erase \n");
	err = i2c_write_bytes(client, buf, 2);
	if (err !=1)
		printk("[AURA_MS51_INBOX] i2c_write_bytes:err %d\n", err);
	printk("[AURA_MS51_INBOX] after erase\n");
	return err;

}
*/

static int ms51_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT		
	struct inode *inode;
	unsigned long magic;
#endif	
	off_t fsize = 0;
	char filepath[128];

	printk("[ROG5_INBOX] ms51_GetFirmwareSize.\n");
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", firmware_name);
	if (NULL == pfile) {
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}
#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT	
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
#endif	
	filp_close(pfile, NULL);
	return fsize;
}

static int ms51_ReadFirmware(char *fw_name, unsigned char *fw_buf)
{
#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT	
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
#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT	
	inode = pfile->f_dentry->d_inode;
#endif
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, fw_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);
#endif	
	return 0;
}

static int ms51_UpdateFirmware(char *fw_buf, int fwsize)
{
	int err = 0;
	unsigned char *buf;
	short addr;
	int count = 0;

	buf = kmalloc(sizeof(unsigned char)*49, GFP_DMA);
	if (!buf) {
		printk("unable to allocate key input memory\n");
		return -ENOMEM;
	}

	//erase--remove this because we will send all 13kb data (add 0 to the end)
	//err = ms51_fw_erase(client);
	//if (err !=1)
		//printk("[AURA_MS51_INBOX] ms51_fw_erase :err %d\n", err);
	//msleep(500);
	//printk("[AURA_MS51_INBOX] after erase :\n");


	//flash

	//first write
	memset(buf,0,sizeof(unsigned char)*48);
	buf[0] = 0xA0;
	buf[13] = 0x34;
	memcpy(&(buf[16]),fw_buf+0,32);

	printk("[ROG5_INBOX] ms51_update_write : num=0\n");
	err = asus_usb_hid_write_fw(buf, 48);
	if (err < 0)
		printk("[ROG5_INBOX] ms51_update_write :err %d\n", err);
	msleep(1000);

	//the follwing write
	for(addr = 32; addr < 13*1024; addr = addr+32){
		memset(buf,0,sizeof(unsigned char)*48);
		buf[0] = 0xA0;
		if(addr <= fwsize-32){
			printk("if: addr = %d\n", addr);
			memcpy(&(buf[16]),fw_buf+addr,32);
			count = 48;
		}else{
			printk("else: addr = %d\n", addr);
			if(addr >= fwsize){
				memset(&(buf[16]),0,sizeof(unsigned char)*32);
				count = 16;
			}else{
				memcpy(&(buf[16]),fw_buf+addr,fwsize-addr);
				//memset(&(buf[16+fwsize-addr]),0,sizeof(unsigned char)*(32-fwsize+addr));
				count = 16 + (fwsize-addr);
			}
		}

		printk("[ROG5_INBOX] ms51_update_write : num=%d, count=%d\n", addr/32, count);
		err = asus_usb_hid_write_fw(buf, count);
		msleep(10);
		if (err < 0)
			printk("[ROG5_INBOX] ms51_update_write :err %d\n", err);
	}//end for

	printk("[ROG5_INBOX] ms51_UpdateFirmware finished.\n");
	kfree(buf);
	return 0;
}

static ssize_t fw_update_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *buf, size_t count)
{
//	unsigned char data[3] = {0};
	int err = 0;
	int fw_size;
	unsigned char *fw_buf;

	char fw_name[128];

	memset(fw_name, 0, sizeof(fw_name));
	sprintf(fw_name, "%s", buf);
	fw_name[count-1] = '\0';

	printk("fw_update_store: %s\n", fw_name);

	// get fs_size
	fw_size = ms51_GetFirmwareSize(fw_name);
	if(fw_size<=0)
	{
		printk("[ROG5_INBOX] fwsize %d\n", fw_size);
		return count;
	}
	printk("[ROG5_INBOX] fwsize %d\n", fw_size);

	// set fw_buf
	fw_buf = kmalloc(fw_size+1, GFP_ATOMIC);

	// read FW content
	if (ms51_ReadFirmware(fw_name, fw_buf)) {
		printk("[ROG5_INBOX] ERROR: request_firmware failed\n");
		kfree(fw_buf);
		return count;
	}

	// reset to ldrom
//	data[0] = 0xCB;
//	data[1] = 0x02;
//	data[2] = 0x00;
//		
//	err = asus_usb_hid_write_aprom(data, 2);
//	if (err < 0)
//		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);

	msleep(1000);

	mutex_lock(&ms51_mutex);
	err = ms51_UpdateFirmware(fw_buf, fw_size);
	if(err)
		printk("[ROG5_INBOX] ms51_UpdateFirmware, err %d\n", err);

	mutex_unlock(&ms51_mutex);

	kfree(fw_buf);

	return count;
}

static ssize_t ap2ld_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	printk("[ROG5_INBOX] AP to LD.\n");

	data[0] = 0xCB;
	data[1] = 0x02;
	data[2] = 0x00;
		
	err = asus_usb_hid_write_aprom(data, 2);
	if (err < 0)
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);

	return count;
}


static ssize_t led_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	//printk("[ROG5_INBOX] led_on: %d\n", val);

	if (val) {
		data[0] = 0x80;
		data[1] = 0x21;
		data[2] = g_mode;

		g_led_on = 1;
	}
	else {
		data[0] = 0x80;
		data[1] = 0x21;
		data[2] = 0x00;

		g_led_on = 0;
	}

	err = asus_usb_hid_write_aprom(data, 3);
	if (err < 0)
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);

	msleep(10);

	//apply all setting
	data[0] = 0x80;
	data[1] = 0x2F;
	data[2] = 0x01;
		
	ret = asus_usb_hid_write_aprom(data, 3);
	if (ret < 0)
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", ret);

	return count;
}

static ssize_t led_on_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", g_led_on);
}

static int fan_enable(unsigned char enable)
{
	int ret = 0;
	unsigned char data[3] = {0};
	
	printk("[ROG5_INBOX] fan_enable: %d\n", enable);

	data[0] = 0x60;
	data[1] = 0x07;
	data[2] = enable;
		
	ret = asus_usb_hid_write_aprom(data, 3);
	if (ret < 0)
		printk("[ROG5_INBOX] fan_enable: asus_usb_hid_write err: %d\n", ret);
	
	return ret;
}

static int fan_rpm(unsigned char rpm)
{
	int ret = 0;
	unsigned char data[3] = {0};

	printk("[ROG5_INBOX] fan_rpm: %d\n", rpm);

	if (rpm < 0 || rpm > 255) {
		printk("[ROG5_INBOX] fan_rpm: input value error: %d (0 ~ 255)\n", rpm);
		return -1;
	}

	data[0] = 0x60;
	data[1] = 0x01;
	data[2] = rpm;
		
	ret = asus_usb_hid_write_aprom(data, 3);
	if (ret < 0)
		printk("[ROG5_INBOX] fan_rpm: asus_usb_hid_write err: %d\n", ret);
	
	return ret;
}

static ssize_t fan_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	ssize_t ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	printk("[ROG5_INBOX] fan_enable_store: %d\n", val);

	if (val > 0)
		fan_enable(1);
	else
		fan_enable(0);

	return count;
}

//+++inbox user fan
static ssize_t inbox_user_fan(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int num = 99;
	u32  tmp;
	int err = 0;

	mutex_lock(&update_lock);
	
	sscanf(buf, "%d", &num);
	printk("[INBOX_FAN] %s: %d", __func__, num);
	
	switch (num) {
		case 0:
			tmp = 0;
			err = fan_rpm(tmp);
			msleep(30);
			err = fan_enable(0);
			break;
		case 1:
			tmp = 127;
			err = fan_rpm(tmp);
			msleep(30);
			err = fan_enable(1);
			break;
		case 2:
			tmp = 135;
			err = fan_rpm(tmp);
			msleep(30);
			err = fan_enable(1);
			break;
		case 3:
			tmp = 163;
			err = fan_rpm(tmp);
			msleep(30);
			err = fan_enable(1);
			break;
		case 4:
			tmp = 171;
			err = fan_rpm(tmp);
			msleep(30);
			err = fan_enable(1);
			break;
		default:
			printk("[INBOX_FAN] %s :mode isn't 0-4, unsupport\n", __func__);

	}
	msleep(500); //Wait 0.5s
	mutex_unlock(&update_lock);
	printk("%s ---", __func__);
	return size;
}

//+++inbox thermal fan
static ssize_t inbox_thermal_fan(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int num = 99;
	u32 tmp;
	int err = 0;

	mutex_lock(&update_lock);
	sscanf(buf, "%d", &num);
	printk("[INBOX_FAN] %s: %d", __func__, num);
	
	switch (num) {
		case 0:
			tmp = 0;
			err = fan_rpm(tmp);
			err = fan_enable(1);
			break;
		case 1:
			tmp = 127;
			err = fan_rpm(tmp);
			err = fan_enable(1);
			break;
		case 2:
			tmp = 135;
			err = fan_rpm(tmp);
			msleep(30);
			err = fan_enable(1);
			break;
		case 3:
			tmp = 163;
			err = fan_rpm(tmp);
			msleep(30);
			err = fan_enable(1);
			break;
		case 4:
			tmp = 171;
			err = fan_rpm(tmp);
			msleep(30);
			err = fan_enable(1);
			break;
		default:
			printk("[INBOX_FAN] %s :mode isn't 0-4, unsupport\n",__func__);
	}
	msleep(500); //Wait 0.5s
	mutex_unlock(&update_lock);
	printk("%s ---", __func__);
	return size;
}
//---inbox thermal fan

static ssize_t fan_rpm_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer, *buffer_ret;
	u8 buffer_ret_tmp[3] = {0};
	int raw_data = 0;
	long rpm = 0;

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}
	
	buffer = kzalloc(0x06, GFP_KERNEL);
	memset(buffer, 0, 6);
	buffer_ret = kzalloc(3, GFP_KERNEL);
	memset(buffer_ret, 0, 3);
	
	hdev = rog5_inbox_hidraw->hid;

	buffer[0] = 0x0B;
	buffer[1] = 0x2;
	buffer[2] = 0x2;
	buffer[3] = 0x60;
	buffer[4] = 0x02;
	buffer[5] = 0x00;

	hid_hw_power(hdev, PM_HINT_FULLON);
	
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("usbhid_set_raw_report fail\n");

	msleep(10);

	buffer[0] = 0x0B;
	buffer[1] = 0x1;
	buffer[2] = 0x1;
	buffer[3] = 0xCC;
	buffer[4] = 0x00;
	buffer[5] = 0x00;

	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("usbhid_set_raw_report fail\n");

	msleep(10);

	ret = usbhid_get_raw_report(hdev, buffer[0], buffer_ret, sizeof(buffer_ret), HID_INPUT_REPORT);
	if (ret < 0)
		printk("usbhid_get_raw_report fail\n");

	printk("[ROG5_INBOX] fan_rpm : %x, %x, %x\n", buffer_ret[0], buffer_ret[1], buffer_ret[2]);

	hid_hw_power(hdev, PM_HINT_NORMAL);

	memcpy(buffer_ret_tmp, buffer_ret, 3);
	kfree(buffer);
	kfree(buffer_ret);

	raw_data = buffer_ret_tmp[1]*256 + buffer_ret_tmp[2];
	if (raw_data != 0) {
		rpm = (24 * 1000000) >> 9;
		do_div(rpm,raw_data);
		return snprintf(buf, PAGE_SIZE,"%ld\n", rpm*30);
	} else {
		return snprintf(buf, PAGE_SIZE,"%d\n", raw_data);
	}
}

static ssize_t fan_rpm_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	int err = 0;
	ssize_t ret;
	unsigned char data[3] = {0};

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	printk("[ROG5_INBOX] fan_rpm_store: %d\n", val);

	if (val < 0 || val > 255) {
		printk("[ROG5_INBOX] fan_rpm_store: input value error: %d (0 ~ 255)\n", val);
		return count;
	}

	data[0] = 0x60;
	data[1] = 0x01;
	data[2] = val;
		
	err = asus_usb_hid_write_aprom(data, 3);
	if (err < 0)
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);

	return count;
}

static ssize_t unique_id_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer, *buffer_ret;
	u8 buffer_ret_tmp[14] = {0};

	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}
	
	buffer = kzalloc(52, GFP_KERNEL);
	memset(buffer,0,52);
	buffer_ret = kzalloc(14, GFP_KERNEL);
	memset(buffer_ret,0,14);

	hdev = rog5_inbox_hidraw->hid;

	buffer[0] = 0x0C;
	buffer[1] = 0x2;
	buffer[2] = 0x2;
	buffer[3] = 0;
	buffer[4] = 0xCB;
	buffer[5] = 0x3;

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, 52, HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("usbhid_set_raw_report fail\n");
	msleep(10);
	
	buffer[0] = 0x0C;
	buffer[1] = 0x1;
	buffer[2] = 0x1;
	buffer[3] = 0xC;
	buffer[4] = 0xCC;
	buffer[5] = 0x0;
	
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, 52, HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("usbhid_set_raw_report fail\n");
	
	msleep(10);

	ret = usbhid_get_raw_report(hdev, buffer[0], buffer_ret, 14, HID_INPUT_REPORT);
	if (ret < 0)
		printk("usbhid_get_raw_report fail\n");

	printk("[ROG5_INBOX] unique id : %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x\n", 
		buffer_ret[0], buffer_ret[1], buffer_ret[2], buffer_ret[3], buffer_ret[4], buffer_ret[5], buffer_ret[6],
		buffer_ret[7], buffer_ret[8], buffer_ret[9], buffer_ret[10], buffer_ret[11], buffer_ret[12], buffer_ret[13]);

	hid_hw_power(hdev, PM_HINT_NORMAL);

	memcpy(buffer_ret_tmp, buffer_ret, 14);
	kfree(buffer);
	kfree(buffer_ret);

	return snprintf(buf, PAGE_SIZE,"0x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
		buffer_ret_tmp[1], buffer_ret_tmp[2], buffer_ret_tmp[3], buffer_ret_tmp[4], buffer_ret_tmp[5], buffer_ret_tmp[6],
		buffer_ret_tmp[7], buffer_ret_tmp[8], buffer_ret_tmp[9], buffer_ret_tmp[10], buffer_ret_tmp[11], buffer_ret_tmp[12]);
}

static ssize_t mode2_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE,"%d\n", mode2_state);
}

static ssize_t mode2_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned char rgb[RGB_MAX] = {0};
	//unsigned char tokenized_data[22] = {0};
	unsigned char rainbow_mode = 0;
	unsigned char mode2 = 0;
	int err = 0;
	int i = 0, n = 0, rgb_num = 0;//rgb_num_t=0;
	long rgb_tmp = 0;
	int ntokens = 0;
	const char *cp = buf;
	const char *buf_tmp;
	unsigned char data[3] = {0};

	mode2_state = 0;

	sscanf(buf, "%d", &mode2);

	while ((cp = strpbrk(cp + 1, ",")))
	{
		ntokens++;  //the number of ","
		//printk("[ROG5_INBOX] mode2_store %s.\n", cp);
	}
	printk("[ROG5_INBOX] mode2_store mode2=%d buf=%s ntokens=%d\n", mode2, buf, ntokens);
	if(ntokens > 6) 
	{
		printk("[ROG5_INBOX] mode2_store: wrong input, too many ntokens\n");
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
		printk("[ROG5_INBOX] mode2_store: wrong input, rgb_num != ntokens*3\n");
		mode2_state=-1;
		return count;
	}

	/*for(i=0;i<rgb_num;i++)
	{
		printk("[AURA_MS51_INBOX] mode2_store, rgb[%d]=0x%x \n",i,rgb[i]);
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
		case 7: //flash and dash
			rainbow_mode = 0x14;
			break;
		case 8: //commet in different direction direction
			rainbow_mode = 0x13;
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
	
	g_mode = (u8)rainbow_mode;

	switch(rainbow_mode){
		case 0:  //mode 0
			break;
		case 0x1: //static
		case 0x2: //breath at the same time
		case 0xF: //breath one led
		case 0x10: //breath one led
		case 0x11: //breath at the different time
			if(ntokens != 2){
				printk("[ROG5_INBOX] mode2_store: wrong input.\n");
				mode2_state = -1;
				return count;
			}
			//sscanf(buf, "%x, %x %x %x,%x %x %x", 
				//&rainbow_mode,&rgb[0],&rgb[1],&rgb[2],&rgb[3],&rgb[4],&rgb[5]);
			//printk("[ROG5_INBOX] mode2_store,static two leds. mode=0x%x,client->addr:0x%x.\n", rainbow_mode,client->addr);
			
			for(i=0; i<6; i++){
				data[0] = 0x80;
				data[1] = (0x10 + i);
				data[2] = rgb[i];

				err = asus_usb_hid_write_aprom(data, 3);
				if (err < 0) {
					mode2_state = -1;
					printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);
				}
			}
			break;
		case 0x7:
		case 0x8://6 colors rainbow
			if(ntokens != 6){
				printk("[ROG5_INBOX] mode2_store,wrong input, ntokens wrong.\n");
				mode2_state=-1;
				return count;
			}
			
			for(i=0; i<=5; i++){
				data[0] = 0xD0 + i;
				for(n=0; n<=2; n++){
					data[1] = n;
					data[2] = rgb[3*i+n];

					err = asus_usb_hid_write_aprom(data, 3);
					if (err < 0){
						mode2_state=-1;
						printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);
					}

				}
			}
			break;
		case 0x12://comet
		case 0x13://comet in different direction
		case 0x14://flash and dash
		case 0x15://flash and dash in different direction
			if(ntokens != 2){
				printk("[ROG5_INBOX] mode2_store,wrong input.\n");
				return count;
			}
			//sscanf(buf, "%x, %x %x %x", &rainbow_mode,&rgb[0],&rgb[1],&rgb[2]);
			//printk("[AURA_MS51_INBOX] mode2_store,comet or flash and dash. mode=0x%x,client->addr:0x%x.\n", rainbow_mode,client->addr);
			
			for(i=0; i<=1; i++){
				data[0] = 0xDB + i;
				for(n=0; n<=2; n++){
					data[1] = n;
					data[2] = rgb[3*i+n];
			
					err = asus_usb_hid_write_aprom(data, 3);
					if (err < 0) {
						mode2_state = -1;
						printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);
					}
				}
			}
			break;
		default:
			break;
	}

	// send apply all command
	data[0] = 0x80;
	data[1] = 0x21;
	data[2] = rainbow_mode;
			
	err = asus_usb_hid_write_aprom(data, 3);
	if (err < 0) {
		mode2_state = -1;
		printk("[ROG5_INBOX] asus_usb_hid_write:err %d\n", err);
	}

	return count;
}

static ssize_t key_state_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	int ret = 0;
	struct hid_device *hdev;
	u8 *buffer;
	u8 key_state_tmp = 0;

	key_state_tmp = key_state;
	
	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX] rog5_inbox_hidraw is NULL !\n");
		return -1;
	}
	
	buffer = kzalloc(2, GFP_KERNEL);
	memset(buffer, 0, 2);
	
	hdev = rog5_inbox_hidraw->hid;

	buffer[0] = 0x0A;
	buffer[1] = 0x0B;

	hid_hw_power(hdev, PM_HINT_FULLON);
	ret = usbhid_set_raw_report(hdev, buffer[0], buffer, sizeof(buffer), HID_OUTPUT_REPORT);
	if (ret < 0)
		printk("[ROG5_INBOX] usbhid_set_raw_report fail\n");
	hid_hw_power(hdev, PM_HINT_NORMAL);

	kfree(buffer);

	if (key_state_tmp > 0) {
		if (key_state == 0xb0) {
			key_state = key_state_tmp;
			return snprintf(buf, PAGE_SIZE,"0x%x\n", key_state_tmp);
		}
		else {
			key_state = key_state_tmp;
			return snprintf(buf, PAGE_SIZE,"fail\n");
		}
	}
	else if (key_state_tmp == 0) {
		if (key_state == 0xb1) {
			key_state = key_state_tmp;
			return snprintf(buf, PAGE_SIZE,"0x%x\n", key_state_tmp);
		}
		else {
			key_state = key_state_tmp;
			return snprintf(buf, PAGE_SIZE,"fail\n");
		}
	}

	key_state = key_state_tmp;
	return snprintf(buf, PAGE_SIZE,"0x%x\n", key_state);
}

static ssize_t aprom_command_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	unsigned char data[3] = {0};
//	int ret;

	data[0] = 0xCB;
	data[1] = 0x01;
	data[2] = 0x00;

//	ret = asus_usb_hid_read_aprom(data);
//	if (ret < 0){
//		printk("[ROG5_INBOX] rog5_inbox_led_test_show:err %d\n", ret);
//	}

	//printk("[ROG5_INBOX] data: 0x%x\n", data[0]);
	return snprintf(buf, PAGE_SIZE,"0x%x 0x%x 0x%x\n", data[0], data[1], data[2]);
}

static ssize_t aprom_command_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 *data;
	int ret = 0;
	int i = 0;

	printk("%s: count = %d\n",__func__, count);

	if (count != 15)
		printk("command length is error\n");

	data = kzalloc(12, GFP_KERNEL);
	memset(data, 0, 12);
	
	ret = kstrtou32(buf, 10, data);
	
	for (i=0; i < count; i++)
		printk("data[i] = %d\n", data[i]);
	
	
	printk("[ROG5_INBOX] %s, data %d\n", __func__, data);
	
	return count;
}

static DEVICE_ATTR(gpio8, 0664, NULL, gpio8_store);
static DEVICE_ATTR(gpio9, 0664, gpio9_show, gpio9_store);
static DEVICE_ATTR(gpio10, 0664, gpio10_show, gpio10_store);
static DEVICE_ATTR(gpio11, 0664, gpio11_show, gpio11_store);
static DEVICE_ATTR(led_test, 0664, NULL, led_test_store);
static DEVICE_ATTR(red_pwm, 0664, red_pwm_show, red_pwm_store);
static DEVICE_ATTR(green_pwm, 0664, green_pwm_show, green_pwm_store);
static DEVICE_ATTR(blue_pwm, 0664, blue_pwm_show, blue_pwm_store);
static DEVICE_ATTR(red1_pwm, 0664, red1_pwm_show, red1_pwm_store);
static DEVICE_ATTR(green1_pwm, 0664, green1_pwm_show, green1_pwm_store);
static DEVICE_ATTR(blue1_pwm, 0664, blue1_pwm_show, blue1_pwm_store);
static DEVICE_ATTR(apply, 0664, apply_show, apply_store);
static DEVICE_ATTR(mode, 0664, mode_show, mode_store);
static DEVICE_ATTR(frame, 0664, get_frame, set_frame);
static DEVICE_ATTR(speed, 0664, get_speed, set_speed);
static DEVICE_ATTR(Calibration, 0664, get_cali_data, set_cali_data);
static DEVICE_ATTR(fw_mode, 0664, fw_mode_show, NULL);
static DEVICE_ATTR(fw_ver, 0664, fw_ver_show, NULL);
static DEVICE_ATTR(fw_update, 0664, NULL, fw_update_store);
static DEVICE_ATTR(ap2ld, 0664, NULL, ap2ld_store);
static DEVICE_ATTR(led_on, 0664, led_on_show, led_on_store);
static DEVICE_ATTR(fan_enable, 0664, NULL, fan_enable_store);
static DEVICE_ATTR(fan_rpm, 0664, fan_rpm_show, fan_rpm_store);
static DEVICE_ATTR(inbox_user_type, 0664, NULL, inbox_user_fan);
static DEVICE_ATTR(inbox_thermal_type, 0664, NULL, inbox_thermal_fan);
static DEVICE_ATTR(unique_id, 0664, unique_id_show, NULL);
static DEVICE_ATTR(mode2, 0664, mode2_show, mode2_store);
static DEVICE_ATTR(key_state, 0664, key_state_show, NULL);
static DEVICE_ATTR(aprom_command, 0664, aprom_command_show, aprom_command_store);

static struct attribute *pwm_attrs[] = {
	&dev_attr_gpio8.attr,
	&dev_attr_gpio9.attr,
	&dev_attr_gpio10.attr,
	&dev_attr_gpio11.attr,
	&dev_attr_led_test.attr,
	&dev_attr_red_pwm.attr,
	&dev_attr_green_pwm.attr,
	&dev_attr_blue_pwm.attr,
	&dev_attr_red1_pwm.attr,
	&dev_attr_green1_pwm.attr,
	&dev_attr_blue1_pwm.attr,
	&dev_attr_apply.attr,
	&dev_attr_mode.attr,
	&dev_attr_frame.attr,
	&dev_attr_speed.attr,
	&dev_attr_Calibration.attr,
	&dev_attr_fw_mode.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_fw_update.attr,
	&dev_attr_ap2ld.attr,
	&dev_attr_led_on.attr,
	&dev_attr_fan_enable.attr,
	&dev_attr_fan_rpm.attr,
	&dev_attr_inbox_user_type.attr,
	&dev_attr_inbox_thermal_type.attr,
	&dev_attr_unique_id.attr,
	&dev_attr_mode2.attr,
	&dev_attr_key_state.attr,
	&dev_attr_aprom_command.attr,
	NULL
};

static const struct attribute_group pwm_attr_group = {
	.attrs = pwm_attrs,
};


static void aura_sync_set(struct led_classdev *led,
			      enum led_brightness brightness)
{
	//printk("[GAMEPAD_III] aura_sync_set : %d.\n", brightness);
}

static enum led_brightness aura_sync_get(struct led_classdev *led_cdev)
{
	struct inbox_drvdata *data;

	//printk("[GAMEPAD_III] aura_sync_get.\n");
	data = container_of(led_cdev, struct inbox_drvdata, led);

	return data->led.brightness;
}

static int aura_sync_register(struct device *dev, struct inbox_drvdata *data)
{
	data->led.name = "aura_inbox";

	data->led.brightness = LED_OFF;
	data->led.max_brightness = LED_HALF;
	data->led.default_trigger = "none";
	data->led.brightness_set = aura_sync_set;
	data->led.brightness_get = aura_sync_get;

	return led_classdev_register(dev, &data->led);
}
static void aura_sync_unregister(struct inbox_drvdata *data)
{
	led_classdev_unregister(&data->led);
}


#ifdef CONFIG_PM
static int rog5_inbox_usb_resume(struct hid_device *hdev)
{
	if(g_Charger_mode) {
		printk("[ROG5_INBOX] In charger mode, stop rog5_inbox_usb_resume\n");
		return 0;
	}

	printk("%s\n", __func__);
	return 0;
}

static int rog5_inbox_usb_suspend(struct hid_device *hdev, pm_message_t message)
{
	if(g_Charger_mode) {
		printk("[ROG5_INBOX] In charger mode, stop rog5_inbox_usb_suspend\n");
		return 0;
	}
	
	printk("%s\n", __func__);
	return 0;
}
#endif /* CONFIG_PM */

static int rog5_inbox_usb_raw_event(struct hid_device *hdev,
		struct hid_report *report, u8 *data, int size)
{
	return 0;
}

void disable_autosuspend_worker(struct work_struct *work)
{
	if (rog5_inbox_hidraw == NULL) {
		printk("[ROG5_INBOX]disable_autosuspend_worker: rog5_inbox_hidraw is NULL\n");
		return;
	}
		
	usb_disable_autosuspend(interface_to_usbdev(to_usb_interface(rog5_inbox_hidraw->hid->dev.parent)));
}

#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
extern void inbox_connect(void);
#endif
static int rog5_inbox_usb_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret = 0;
	unsigned int cmask = HID_CONNECT_DEFAULT;
	struct inbox_drvdata *drvdata;

	if(g_Charger_mode) {
		printk("[ROG5_INBOX] In charger mode, stop rog5_inbox_usb_probe\n");
		return 0;
	}

	printk("[ROG5_INBOX] hid->name : %s\n", hdev->name);
	printk("[ROG5_INBOX] hid->vendor  : 0x%4x\n", hdev->vendor);
	printk("[ROG5_INBOX] hid->product : 0x%x\n", hdev->product);
#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT	
	ASUSEvtlog("[ROG5_INBOX] rog5 inbox connect\n");
#endif

	drvdata = devm_kzalloc(&hdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (drvdata == NULL) {
		hid_err(hdev, "[ROG5_INBOX] Can't alloc drvdata\n");
		return -ENOMEM;
	}
	hid_set_drvdata(hdev, drvdata);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "[ROG5_INBOX] parse failed\n");
		goto err_free;
	}

	ret = hid_hw_start(hdev, cmask);
	if (ret) {
		hid_err(hdev, "[ROG5_INBOX] hw start failed\n");
		goto err_free;
	}

	rog5_inbox_hidraw = hdev->hidraw;

	mutex_init(&ms51_mutex);
		
	// Register sys class  
	ret = aura_sync_register(&hdev->dev, drvdata);
	if (ret) {
		hid_err(hdev, "[ROG5_INBOX] aura_sync_register failed\n");
		goto err_free;
	}
	ret = sysfs_create_group(&drvdata->led.dev->kobj, &pwm_attr_group);
	if (ret)
		goto unregister;

	mutex_init(&update_lock);
	
// Set global variable
	g_red_max = 255;
	g_green_max = 255;
	g_blue_max = 255;
	g_red=-1;
	g_green=-1;
	g_blue=-1;
	g_mode=-1;
	g_speed=-1;
	g_led_on=-1;

	INIT_DELAYED_WORK(&disable_autosuspend_work, disable_autosuspend_worker);
	schedule_delayed_work(&disable_autosuspend_work, msecs_to_jiffies(1000));

	device_init_wakeup(&interface_to_usbdev(to_usb_interface(hdev->dev.parent))->dev, true);

#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
	inbox_connect();
#endif

	return 0;

unregister:
	aura_sync_unregister(drvdata);
err_free:
	printk("[ROG5_INBOX] rog5_inbox_usb_probe fail.\n");
	hid_hw_stop(hdev);
	return ret;
}


static void rog5_inbox_usb_remove(struct hid_device *hdev)
{
	struct inbox_drvdata *drvdata = dev_get_drvdata(&hdev->dev);;
	
	if(g_Charger_mode) {
		printk("[ROG5_INBOX] In charger mode, stop rog5_inbox_usb_remove\n");
		return;
	}

#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT	
	ASUSEvtlog("[ROG5_INBOX] rog5 inbox disconnect\n");
#endif

	sysfs_remove_group(&drvdata->led.dev->kobj, &pwm_attr_group);
	aura_sync_unregister(drvdata);
	rog5_inbox_hidraw = NULL;
	hid_hw_stop(hdev);
}

static struct hid_device_id rog5_inbox_idtable[] = {
	{ HID_USB_DEVICE(0x0BDA, 0x4BF0),
		.driver_data = 0 },
	{ HID_USB_DEVICE(0x0BDA, 0x4A80),
		.driver_data = 0 },
	{ }
};
MODULE_DEVICE_TABLE(hid, rog5_inbox_idtable);

static struct hid_driver rog5_inbox_hid_driver = {
	.name		= "rog5_inbox",
	.id_table		= rog5_inbox_idtable,
	.probe			= rog5_inbox_usb_probe,
	.remove			= rog5_inbox_usb_remove,
	.raw_event		= rog5_inbox_usb_raw_event,
#ifdef CONFIG_PM
	.suspend        = rog5_inbox_usb_suspend,
	.resume			= rog5_inbox_usb_resume,
#endif
};

static int __init rog5_inbox_usb_init(void)
{
	printk("[ROG5_INBOX] rog5_inbox_usb_init\n");
	return hid_register_driver(&rog5_inbox_hid_driver);
}

static void __exit rog5_inbox_usb_exit(void)
{
	printk("[ROG5_INBOX] rog5_inbox_usb_exit\n");
	hid_unregister_driver(&rog5_inbox_hid_driver);
}

module_init(rog5_inbox_usb_init);
module_exit(rog5_inbox_usb_exit);


MODULE_AUTHOR("ASUS Lenter");
MODULE_DESCRIPTION("ROG5 INBOX HID Interface");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("asus:rog5 inbox HID");
