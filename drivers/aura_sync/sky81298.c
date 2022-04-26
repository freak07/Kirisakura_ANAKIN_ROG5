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
#include <linux/leds.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/mutex.h>

static struct i2c_client *g_client;

struct sky81298_platform_data {
	u8 reg;

	//struct device dev;;
	struct led_classdev led;	/* LED control */
	char led_name[32];
};

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

static int sky81298_read_bytes(struct i2c_client *client, u8 addr, char *data)
{
	int err = 0;
	unsigned char buf[16] = {0};

	buf[0] = addr;

//	printk("[Bumper] sky81298_read_bytes : addr: 0x%x, buf[0] : 0x%x\n", addr, buf[0]);
	err = i2c_read_bytes(client, buf, 1, data, 1);	//send read command
	if (err != 2)
		printk("[Bumper] i2c_read_bytes:err %d\n", err);

	return err;
}

static int sky81298_write_bytes(struct i2c_client *client, u8 addr, char value)
{
	int err = 0;
	unsigned char buf[16] = {0};

	buf[0] = addr;
	buf[1] = value;

//	printk("[Bumper] sky81298_write_bytes : addr: 0x%x, buf[0] : 0x%x, buf[1] : 0x%x, value : 0x%x\n", addr, buf[0], buf[1], value);
	err = i2c_write_bytes(client, buf, 2);	//set register address
	if (err !=1)
		printk("[Bumper] i2c_write_bytes:err %d\n", err);

	return err;
}

static ssize_t register_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct sky81298_platform_data *platform_data = i2c_get_clientdata(client);
	u8 reg_val;
	ssize_t ret;

	ret = kstrtou8(buf, 10, &reg_val);
	if (ret)
		return ret;

	platform_data->reg = reg_val;
	printk("[Bumper] register_store : REG[0x%x].\n", platform_data->reg);

	return count;
}

static ssize_t register_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct sky81298_platform_data *platform_data = i2c_get_clientdata(client);

	printk("[Bumper] register_show : REG[0x%x].\n", platform_data->reg);

	return snprintf(buf, PAGE_SIZE,"REG[0x%x]\n", platform_data->reg);
}

static ssize_t set_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct sky81298_platform_data *platform_data = i2c_get_clientdata(client);
	u8 reg_val;
	int err = 0;
	ssize_t ret;

	ret = kstrtou8(buf, 10, &reg_val);
	if (ret)
		return ret;

	printk("[Bumper] set_value : REG[0x%x] : val : 0x%x\n", platform_data->reg, reg_val);
	err = sky81298_write_bytes(client, platform_data->reg, reg_val);
	if (err !=1)
		printk("[Bumper] sky81298_write_bytes:err %d\n", err);

	return count;
}

static ssize_t get_value(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct sky81298_platform_data *platform_data = i2c_get_clientdata(client);
	unsigned char data[4] = {0};
	int err = 0;

	printk("[Bumper] get_value : REG[0x%x].\n", platform_data->reg);

	err = sky81298_read_bytes(client, platform_data->reg, data);
	if (err != 2)
		printk("[Bumper] sky81298_read_bytes:err %d\n", err);

	return snprintf(buf, PAGE_SIZE,"REG[0x%x]:0x%x\n", platform_data->reg, data[0]);
}

void bumper_vdd_switch(u32 val)
{
	int err = 0;

	if(val){
	// Set Defautl register value
		printk("[Bumper] bumper_vdd_switch on\n");
		err = sky81298_write_bytes(g_client, 0x5, 0x2);				// set IMM1
		if (err !=1)
			printk("[Bumper] sky81298_write_bytes:err %d\n", err);

		err = sky81298_write_bytes(g_client, 0x6, 0x2);				// set IMM2
		if (err !=1)
			printk("[Bumper] sky81298_write_bytes:err %d\n", err);

		err = sky81298_write_bytes(g_client, 0x7, 0x2);				// set IMM3
		if (err !=1)
			printk("[Bumper] sky81298_write_bytes:err %d\n", err);

		err = sky81298_write_bytes(g_client, 0x8, 0x15);			// set MM_EN1 & MM_EN2 & MM_EN3
		if (err !=1)
			printk("[Bumper] sky81298_write_bytes:err %d\n", err);
	}else {
	// Set Defautl register value
		printk("[Bumper] bumper_vdd_switch off\n");
		err = sky81298_write_bytes(g_client, 0x5, 0x2);
		if (err !=1)
			printk("[Bumper] sky81298_write_bytes:err %d\n", err);

		err = sky81298_write_bytes(g_client, 0x6, 0x2);
		if (err !=1)
			printk("[Bumper] sky81298_write_bytes:err %d\n", err);

		err = sky81298_write_bytes(g_client, 0x7, 0x2);
		if (err !=1)
			printk("[Bumper] sky81298_write_bytes:err %d\n", err);

		err = sky81298_write_bytes(g_client, 0x8, 0x0);
		if (err !=1)
			printk("[Bumper] sky81298_write_bytes:err %d\n", err);
	}
}
EXPORT_SYMBOL(bumper_vdd_switch);

static ssize_t vdd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 reg_val = 0;
	ssize_t ret;;

	ret = kstrtou8(buf, 10, &reg_val);
	if (ret)
		return ret;

	bumper_vdd_switch(reg_val);

	return count;
}

static DEVICE_ATTR(register, 0664, register_show, register_store);
static DEVICE_ATTR(value, 0664, get_value, set_value);
static DEVICE_ATTR(VDD, 0664, NULL, vdd_store);

static struct attribute *pwm_attrs[] = {
	&dev_attr_register.attr,
	&dev_attr_value.attr,
	&dev_attr_VDD.attr,
	NULL
};

static const struct attribute_group pwm_attr_group = {
	.attrs = pwm_attrs,
};


static void bumper_led_set(struct led_classdev *led,
			      enum led_brightness brightness)
{
	printk("[Bumper] bumper_led_set : %d.\n", brightness);
}

static enum led_brightness bumper_led_get(struct led_classdev *led_cdev)
{
	struct sky81298_platform_data *pdata;

	printk("[Bumper] bumper_led_get.\n");
	pdata = container_of(led_cdev, struct sky81298_platform_data, led);

	return pdata->led.brightness;
}

static int bumper_led_register(struct device *dev, struct sky81298_platform_data *pdata)
{
	pdata->led.name = "bumper";
	pdata->led.brightness = LED_OFF;
	pdata->led.max_brightness = LED_HALF;
	pdata->led.default_trigger = "none";
	pdata->led.brightness_set = bumper_led_set;
	pdata->led.brightness_get = bumper_led_get;

	return led_classdev_register(dev, &pdata->led);
}

static void bumper_led_unregister(struct sky81298_platform_data *pdata)
{
	led_classdev_unregister(&pdata->led);
}

static int sky81298_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct sky81298_platform_data *platform_data;
	
	printk("[Bumper] sky81298_probe.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
	goto exit_check_functionality_failed;
	} else
		printk("[Bumper] I2C function test pass\n");

	printk("[Bumper] client->addr : 0x%x\n", client->addr);

	platform_data = devm_kzalloc(&client->dev, sizeof(struct sky81298_platform_data), GFP_KERNEL);
	if (!platform_data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, platform_data);

// Register sys class  
	err = bumper_led_register(&client->dev, platform_data);
	if (err) {
		printk("[Bumper] Failed to register LED device: %d\n", err);
		goto unled;
	}
	err = sysfs_create_group(&platform_data->led.dev->kobj, &pwm_attr_group);
	if (err)
			goto unled;

	g_client = client;

	printk("[Bumper] sky81298_probe done.\n");
	return 0;

unled:
	bumper_led_unregister(platform_data);
exit_check_functionality_failed:
	printk("[Bumper] sky81298_probe fail !!!\n");
	return err;
}

static int sky81298_remove(struct i2c_client *client)
{
	int err = 0;

	printk("[Bumper] sky81298_remove : err %d\n", err);
	return 0;
}

int sky81298_suspend(struct device *dev)
{
	int err = 0;
	return err;
}

int sky81298_resume(struct device *dev)
{
	int err = 0;
	return err;
}

static const struct i2c_device_id sky81298_id[] = {
	{ "sky81298_i2c", 0},
	{},
};
//MODULE_DEVICE_TABLE(i2c, sky81298_id);

static const struct dev_pm_ops sky81298_pm_ops = {
	.suspend	= sky81298_suspend,
	.resume	= sky81298_resume,
};

#ifdef CONFIG_OF
static const struct of_device_id ene_match_table[] = {
	{ .compatible = "sky81298",},
	{ },
};
#else
#define ene_match_table NULL
#endif

static struct i2c_driver sky81298_driver = {
	.driver		= {
		.name		= "sky81298",
		.owner = THIS_MODULE,
		.pm	= &sky81298_pm_ops,
		.of_match_table	= ene_match_table,
	},
	.probe		= sky81298_probe,
	.remove		= sky81298_remove,
	.id_table 	= sky81298_id,
};

static int __init sky81298_bus_init(void)
{
	int ret;

	ret = i2c_add_driver(&sky81298_driver);
	if (ret)
		printk("[Bumper] sky81298 driver int failed.\n");
	else
		printk("[Bumper] sky81298 driver int success.\n");
	
	return ret;
}
module_init(sky81298_bus_init);

static void __exit sky81298_bus_exit(void)
{
	i2c_del_driver(&sky81298_driver);
}
module_exit(sky81298_bus_exit);

MODULE_AUTHOR("ASUS Deeo Ho");
MODULE_DESCRIPTION("Bumper LED driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("asus:sky81298");
