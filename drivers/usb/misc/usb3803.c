/*
 * drivers/usb/misc/usb3803.c - usb3803 usb hub driver
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>

/* DEFINE RESISTERs */
#define CFG1        0x06
#define BST21       0xf8
#define BSTUP3      0xf6
#define VSNS21      0xf5
#define VSNSUP3     0xf4
#define SP_ILOCK_REG    0xE7

#define USB3803_I2C_NAME "usb3803"
#define usb3803_addr 0x29

enum usb3803_mode{
	USB_3803_MODE_HUB = 0,
	USB_3803_MODE_BYPASS = 1,
	USB_3803_MODE_STANDBY = 2,
};

struct gpio_control {
	u32 HUB_RESET;
	u32 HUB_BYPASS;
	u32 HUB_INT;
	u32 HUB_CONNECT;
};


struct usb3803 {
	enum usb3803_mode	mode;
	struct device		*dev;

	/* debugfs entries */
	struct dentry		*root;
	u8			reg_CFG1;
	u8			reg_BST21;
	u8			reg_BSTUP3;
	u8			reg_VSNS21;
	u8			reg_VSNSUP3;
};

int current_hub_mode;
int current_clk;
struct i2c_client *hub_client;
struct gpio_control *hub_gpio_ctrl;

struct clk		*ref_clk;
static int hub_port = 0x0;
struct usb3803 *hub_usb3803;

static int usb3803_register_write(char reg, u8 data);
static int usb3803_register_read(char reg, char *data);
static void usb3803_gpio_init(struct usb3803 *hub);
int usb3803_set_mode(int mode);
int usb3803_connect_n(int val);
int usb3803_reset_n(int val);
int usb3803_clock_en(int val);
int usb3803_bypass_n(int val);

static ssize_t mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (current_hub_mode == USB_3803_MODE_HUB)
		return sprintf(buf, "%s", "hub\n");
	else if (current_hub_mode == USB_3803_MODE_BYPASS)
		return sprintf(buf, "%s", "bypass\n");
	else if (current_hub_mode == USB_3803_MODE_STANDBY)
		return sprintf(buf, "%s", "standby\n");

	return 0;
}
static ssize_t mode_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	if (!strncmp(buf, "hub", 3)) {
		usb3803_set_mode(USB_3803_MODE_HUB);
		pr_info("usb3803 mode set to hub\n");
	} else if (!strncmp(buf, "bypass", 6)) {
		usb3803_set_mode(USB_3803_MODE_BYPASS);
		pr_info("usb3803 mode set to bypass\n");
	} else if (!strncmp(buf, "standby", 7)) {
		usb3803_set_mode(USB_3803_MODE_STANDBY);
		pr_info("usb3803 mode set to standby\n");
	}
	return size;
}
static DEVICE_ATTR_RW(mode);

static ssize_t port_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "hub_port=0x%x", hub_port);
}
static ssize_t port_store(
	struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	if (!strncmp(buf, "disable_cp", 10)) {
		hub_port = (0x1 << 2) | hub_port;
		pr_debug("usb3803 port disable cp\n");
	} else if (!strncmp(buf, "disable_wimax", 13)) {
		hub_port = (0x1 << 1) | hub_port;
		pr_debug("usb3803 port disable wimaxb\n");
	} else if (!strncmp(buf, "disable_ap", 10)) {
		hub_port = (0x1 << 3) | hub_port;
		pr_debug("usb3803 port disable ap\n");
	} else if (!strncmp(buf, "enable_cp", 9)) {
		hub_port = ~(0x1 << 2) & hub_port;
		pr_debug("usb3803 port enable cp\n");
	} else if (!strncmp(buf, "enable_wimax", 12)) {
		hub_port = ~(0x1 << 1) & hub_port;
		pr_debug("usb3803 port enable wimaxb\n");
	} else if (!strncmp(buf, "enable_ap", 9)) {
		hub_port = ~(0x1 << 3) & hub_port;
		pr_debug("usb3803 port enable ap\n");
	}

	if (current_hub_mode == USB_3803_MODE_HUB)
		usb3803_set_mode(USB_3803_MODE_HUB);
	pr_debug("usb3803 mode setting (%s)\n", buf);

	return size;
}
static DEVICE_ATTR_RW(port);

static int usb3803_register_write(char reg, u8 data)
{
	int ret;
	char buf[2];
	struct i2c_msg msg[] = {
		{
			.addr = hub_client->addr,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
	};

	buf[0] = reg;
	buf[1] = data;

	ret = i2c_transfer(hub_client->adapter, msg, 1);

	if (ret < 0)
			printk("%s: failed to write i2c addr=%x reg=%x\n",	__func__, hub_client->addr, reg);

	return ret;
}

static int usb3803_register_read(char reg, char *data)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr = hub_client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = hub_client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = data,
		},
	};

	ret = i2c_transfer(hub_client->adapter, msgs, 2);

	if (ret < 0)
			printk("%s: failed to read i2c addr=%x reg=%x\n",	__func__, hub_client->addr, reg);

	return ret;
}

static void usb3803_create_debugfs(struct usb3803 *hub)
{
	hub->root = debugfs_create_dir(dev_name(hub->dev), NULL);
	debugfs_create_x8("reg_CFG1", 0644, hub->root, &hub->reg_CFG1);
	debugfs_create_x8("reg_BST21", 0644, hub->root, &hub->reg_BST21);
	debugfs_create_x8("reg_BSTUP3", 0644, hub->root, &hub->reg_BSTUP3);
	debugfs_create_x8("reg_VSNS21", 0644, hub->root, &hub->reg_VSNS21);
	debugfs_create_x8("reg_VSNSUP3", 0644, hub->root, &hub->reg_VSNSUP3);
}

int usb3803_set_mode(int mode)
{
	int ret = 0;
	char data;

	switch (mode) {
	case USB_3803_MODE_HUB:
		pr_info("%s USB_3803_MODE_HUB\n", __func__);
		usb3803_reset_n(0);
		msleep(20);
		/* enable clock */
		usb3803_clock_en(1);
		/* hub mode */
		usb3803_bypass_n(1);
		/* reset assert */
		usb3803_reset_n(1);

		usleep_range(5000, 5000);

		usb3803_register_write(SP_ILOCK_REG, 0x03);
		usb3803_register_read(SP_ILOCK_REG, &data);
		pr_info("[%s] SP_ILOCK_REG: 0x%x \n", __func__, data);

		//set self_power
		usb3803_register_read(CFG1, &data);
		pr_info("[%s] default CFG1: 0x%x \n", __func__, data);
		data |= 0x80;
		usb3803_register_write(CFG1, data);
		usb3803_register_read(CFG1, &data);
		pr_info("[%s] set self_power CFG1: 0x%x \n", __func__, data);

		//set tuning eye diagram param
		usb3803_register_write(BST21, 0x70);
		usb3803_register_write(BSTUP3, 0x32);

		//read default eye diagram param
		usb3803_register_read(BST21, &data);
		pr_info("[%s] default BST21: 0x%x \n", __func__, data);
		usb3803_register_read(BSTUP3, &data);
		pr_info("[%s] default BSTUP3: 0x%x \n", __func__, data);
		usb3803_register_read(VSNS21, &data);
		pr_info("[%s] default VSNS21: 0x%x \n", __func__, data);
		usb3803_register_read(VSNSUP3, &data);
		pr_info("[%s] default VSNSUP3: 0x%x \n", __func__, data);

		//overide eye diagram param
		if (hub_usb3803->reg_BST21) {
			usb3803_register_write(BST21, hub_usb3803->reg_BST21);
			usb3803_register_read(BST21, &data);
			pr_info("[%s] overide BST21: 0x%x\n", __func__, data);
		}
		if (hub_usb3803->reg_BSTUP3) {
			usb3803_register_write(BSTUP3, hub_usb3803->reg_BSTUP3);
			usb3803_register_read(BSTUP3, &data);
			pr_info("[%s] overide BSTUP3: 0x%x\n", __func__, data);
		}
		if (hub_usb3803->reg_VSNS21) {
			usb3803_register_write(VSNS21, hub_usb3803->reg_VSNS21);
			usb3803_register_read(VSNS21, &data);
			pr_info("[%s] overide VSNS21: 0x%x\n", __func__, data);
		}
		if (hub_usb3803->reg_VSNSUP3) {
			usb3803_register_write(VSNSUP3, hub_usb3803->reg_VSNSUP3);
			usb3803_register_read(VSNSUP3, &data);
			pr_info("[%s] overide VSNSUP3: 0x%x\n", __func__, data);
		}

		usb3803_register_write(SP_ILOCK_REG, 0x02);
		usb3803_register_read(SP_ILOCK_REG, &data);
		pr_info("[%s] SP_ILOCK_REG: 0x%x \n", __func__, data);

		break;

	case USB_3803_MODE_BYPASS:
		pr_info("%s USB_3803_MODE_BYPASS\n", __func__);
		usb3803_reset_n(0);
		msleep(10);
		/* disable clock */
		usb3803_clock_en(0);
		/* bypass mode  */
		usb3803_bypass_n(0);
		/* reset assert */
		usb3803_reset_n(1);

		break;

	case USB_3803_MODE_STANDBY:
		pr_info("%s USB_3803_MODE_STANDBY\n", __func__);
		usb3803_reset_n(0);
		/* disable clock */
		usb3803_clock_en(0);
		usb3803_bypass_n(0);
		break;

	default:
		pr_err("[%s] Invalid mode %d\n", __func__, mode);
		break;
	}

	current_hub_mode = mode;
	return ret;
}
EXPORT_SYMBOL(usb3803_set_mode);

static void usb3803_gpio_init(struct usb3803 *hub)
{
	int ret = 0;

	pr_info("%s\n", __func__);
	hub_gpio_ctrl->HUB_BYPASS = of_get_named_gpio(hub->dev->of_node, "HUB_BYPASS", 0);
	ret = gpio_request(hub_gpio_ctrl->HUB_BYPASS, "HUB_BYPASS");
	if (ret)
		pr_info("%s, failed to request HUB_BYPASS\n", __func__);

	hub_gpio_ctrl->HUB_RESET = of_get_named_gpio(hub->dev->of_node, "HUB_RESET", 0);
	ret = gpio_request(hub_gpio_ctrl->HUB_RESET, "HUB_RESET");
	if (ret)
		pr_info("%s, failed to request HUB_RESET\n", __func__);

	hub_gpio_ctrl->HUB_CONNECT = of_get_named_gpio(hub->dev->of_node, "HUB_CONNECT", 0);
	ret = gpio_request(hub_gpio_ctrl->HUB_CONNECT, "HUB_CONNECT");
	if (ret)
		pr_info("%s, failed to request HUB_CONNECT\n", __func__);

}

int usb3803_connect_n(int val)
{
	int ret = 0;

	ret = gpio_direction_output(hub_gpio_ctrl->HUB_CONNECT, val);
	if (ret) {
		pr_info("USB3803 failed to control HUB_CONNECT\n");
	}
	return 0;
}

int usb3803_reset_n(int val)
{
	int ret = 0;

	ret = gpio_direction_output(hub_gpio_ctrl->HUB_RESET, val);
	if (ret) {
		pr_info("USB3803 failed to control HUB_RESET\n");
	}
	return 0;
}

int usb3803_bypass_n(int val)
{
	int ret = 0;

	ret = gpio_direction_output(hub_gpio_ctrl->HUB_BYPASS, val);
	if (ret) {
		pr_info("USB3803 failed to control HUB_BYPASS\n");
	}
	return 0;
}

int usb3803_clock_en(int val)
{
	int err;

	if (val){
		pr_info("%s enable bb_clk3\n", __func__);
		err = clk_prepare_enable(ref_clk);
		if (err) {
			pr_err("%s fail to enable bb_clk3\n", __func__);
			return err;
		}
		current_clk = val;
	} else {
		if (current_clk) {
			pr_info("%s disable bb_clk3\n", __func__);
			clk_disable_unprepare(ref_clk);
			current_clk = val;
		} else {
			pr_info("%s bb_clk3 already disable\n", __func__);
		}
	}

	return 0;
}

int usb3803_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct usb3803 *hub;
	int err = 0;

	pr_info("%s\n", __func__);

	//i2c init
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		pr_err("%s i2c bus does not support usb3803\n", __func__);
	}

	hub = devm_kzalloc(&client->dev, sizeof(struct usb3803), GFP_KERNEL);
	if (!hub)
		return -ENOMEM;

	hub_client = client;
	i2c_set_clientdata(client, hub);
	hub_client->addr = usb3803_addr;

	hub->dev = &client->dev;

	//gpio init
	hub_gpio_ctrl = devm_kzalloc(&client->dev, sizeof(*hub_gpio_ctrl), GFP_KERNEL);
	if (!hub_gpio_ctrl)
		return -ENOMEM;

	usb3803_gpio_init(hub);

	//ref_clk init
	ref_clk = devm_clk_get(hub->dev, "bb_clk3");
	if (IS_ERR(ref_clk)){
		pr_err("%s clk_get bb_clk3 fail!\n", __func__);
		ref_clk = NULL;
	}
	current_clk = 0;

	pr_info("%s hub default setting\n", __func__);
	usb3803_connect_n(1);
	usb3803_set_mode(USB_3803_MODE_BYPASS);

	err = device_create_file(&client->dev, &dev_attr_mode);
	err = device_create_file(&client->dev, &dev_attr_port);

	usb3803_create_debugfs(hub);

	hub_usb3803 = hub;

	return err;

}

static int usb3803_remove(struct i2c_client *client)
{
	return 0;
}

static const struct of_device_id usb3803_match_table[] = {
	{ .compatible = "hub,usb3803",},
	{ },
};

static const struct i2c_device_id usb3803_id[] = {
	{ USB3803_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver usb3803_driver = {
	.probe		= usb3803_probe,
	.remove		= usb3803_remove,
	.id_table = usb3803_id,
	.driver		= {
		.name		= USB3803_I2C_NAME,
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(usb3803_match_table),
	},
};

static int __init usb3803_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&usb3803_driver);
	if (ret != 0)
		pr_err("USB3803 I2C add driver error ret %d\n", ret);
	return ret;
}
module_init(usb3803_init);

static void __exit usb3803_exit(void)
{
	i2c_del_driver(&usb3803_driver);
}
module_exit(usb3803_exit);

MODULE_DESCRIPTION("USB3803 USB HUB driver");
MODULE_LICENSE("GPL");
