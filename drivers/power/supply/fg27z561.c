/*
fg27z561 ADC Driver
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
//#include <linux/hwmon.h>
//#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
//#include <linux/mutex.h>
#include <linux/of.h>

#include <linux/kernel.h>
#include <linux/types.h>
//#include <linux/spmi.h>
//#include <linux/fs.h>
#include <linux/cdev.h>
//#include <linux/semaphore.h>
#include <linux/device.h>
//#include <linux/syscalls.h>
//#include <asm/uaccess.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
/*#include <linux/sched.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/clk.h>
#include <linux/wakelock.h>*/

//ASUS charger BSP : global fg27z561_READY +++
bool fg27z561_ready = false;
EXPORT_SYMBOL(fg27z561_ready);

//Define register addresses of fg27z561 0x55
#define fg27z561_raddr	0x55

struct fg27z561_data
{
	u32 fg27z561;
};

struct i2c_client *fg27z561_client;

/*
fg27z561_write_reg():	write 8 bits reg function
slave_addr:	SMBus address (7 bits)
cmd_reg   :	cmd register for programming
write_val  :	the value will be written
*/
#if 0
static int i2c_write_bytes(struct i2c_client *client, char *write_buf, int writelen)
{
	struct i2c_msg msg;
	int ret=-1;
	
	msg.flags = !I2C_M_RD;		//write
	msg.addr = fg27z561_client->addr;
	msg.len = writelen;
	msg.buf = write_buf; 

	ret = i2c_transfer(client->adapter,&msg, 1);
	return ret;
} 
#endif
int fg27z561_write_reg(uint8_t cmd_reg, uint8_t write_val)
{
	int ret = 0;

	printk("[BAT][CHG] fg27z561_write_reg start\n");
	//fg27z561_client->addr = fg27z561_raddr; //real SMBus address (8 bits)
	ret = i2c_smbus_write_byte_data(fg27z561_client, cmd_reg, write_val);
	if (ret < 0) {
		printk("%s: failed to write i2c addr=%x\n",
			__func__, fg27z561_raddr);
	}

	return ret;
}
EXPORT_SYMBOL(fg27z561_write_reg);

/*
fg27z561_mask_write_reg():	write 8 bits reg function
slave_addr:	SMBus address (7 bits)
cmd_reg   :	cmd register for programming
write_val  :	the value will be written
*/

/*
fg27z561_read_reg():	read 8 bits reg function
slave_addr:	SMBus address (7 bits)
cmd_reg   :	cmd register for programming
store_read_val  :	value be read will store here

*/
int fg27z561_read_reg(uint8_t cmd_reg, int16_t *store_read_val)
{
	
	int ret = 0;

	//fg27z561_client->addr = fg27z561_raddr;
	ret = i2c_smbus_read_word_data(fg27z561_client, cmd_reg);
	if (ret < 0) {
		printk("%s: failed to read i2c addr=%x\n",	__func__, fg27z561_raddr);
	}

	*store_read_val = (uint16_t) ret;

	return ret;
	/*
	int err = 0;
	unsigned char buf[16] = {0};
	struct i2c_msg msgs;
	
	buf[0] = addr & 0xFF;
	printk("[BAT][CHG] i2c_write_bytes start\n");
	err = i2c_write_bytes(fg27z561_client, buf, 1);
	if (err !=1)
		printk("[BAT][CHG] i2c_write_bytes addr: 0x%x err:%d\n", addr,err);
	printk("[BAT][CHG] i2c_write_bytes end\n");
	msleep(1);//wait for ic
	//read data
	msgs.flags = I2C_M_RD;		//read
	msgs.addr = fg27z561_client->addr;
	msgs.len = 2;
	msgs.buf = data;
	err = i2c_transfer(fg27z561_client->adapter,&msgs, 1);

	return err;
	*/ 
}
EXPORT_SYMBOL(fg27z561_read_reg);

static ssize_t adapter_value_show(struct device *dev, struct device_attribute *da,
	char *buf)
{
	uint16_t val = 0;
	int err=0;
		printk("[BAT][CHG]adc ack show\n");
		err = fg27z561_read_reg(0x08, &val);
		printk("[BAT][CHG]%d", val);
	return sprintf(buf, "%d\n",val);
}
int read_temperature_now(void)
{
	uint16_t val = 0;
	int err=0;
		err = fg27z561_read_reg(0x06, &val);
		printk("[BAT][CHG]temp = %d", val);
	return val;
	
}EXPORT_SYMBOL(read_temperature_now);
int read_soc_now(void)
{
	uint16_t val = 0;
	int err=0;
		err = fg27z561_read_reg(0x2C, &val);
		printk("[BAT][CHG]soc = %d", val);
	return val;
	
}EXPORT_SYMBOL(read_soc_now);
int read_current_now(void)
{
	int16_t val = 0;
	int err=0;
		err = fg27z561_read_reg(0x0C, &val);
		printk("[BAT][CHG]current = %d", val);
	return val;
	
}EXPORT_SYMBOL(read_current_now);
int read_voltage_now(void)
{
	uint16_t val = 0;
	int err=0;
		err = fg27z561_read_reg(0x08, &val);
		printk("[BAT][CHG]voltage = %d", val);
	return val;
	
}EXPORT_SYMBOL(read_voltage_now);


static ssize_t read_voltage_show(struct device *dev, struct device_attribute *da,
	char *buf)
{
	uint16_t val = 0;
	int err=0;
		printk("[BAT][CHG]adc ack show\n");
		err = fg27z561_read_reg(0x08, &val);
		printk("[BAT][CHG]%d", val);
	return sprintf(buf, "%d\n",val);
}

static DEVICE_ATTR(adapter_value, 0664, adapter_value_show, NULL);
static DEVICE_ATTR(read_voltage, 0664, read_voltage_show, NULL);

static struct attribute *dump_reg_attrs[] = {
	&dev_attr_adapter_value.attr,
	&dev_attr_read_voltage.attr,
	NULL
};

static const struct attribute_group dump_reg_attr_group = {
	.attrs = dump_reg_attrs,
};

static int fg27z561_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct fg27z561_data *data;
	int rc;

	printk("[BAT][CHG] %s start\n", __FUNCTION__);

	fg27z561_ready = 0;
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[BAT][CHG] %s: i2c bus does not support the fg27z561\n", __FUNCTION__);
	}

	data = devm_kzalloc(&client->dev, sizeof(struct fg27z561_data), GFP_KERNEL);

	if (!data)
		return -ENOMEM;
		
	fg27z561_client = client;
	i2c_set_clientdata(client, data);
	fg27z561_client->addr = fg27z561_raddr;

	rc = sysfs_create_group(&client->dev.kobj, &dump_reg_attr_group);
	if (rc){
		pr_err("[BAT][CHG] Failed to register fg27z561 device: %d\n", rc);
		goto exit_remove;
	}
	fg27z561_ready = 1;
	
	printk("[BAT][CHG] %s end\n", __FUNCTION__);

	return 0;

exit_remove:
		
		sysfs_remove_group(&client->dev.kobj, &dump_reg_attr_group);
	return rc;

}

static struct of_device_id fg27z561_match_table[] = {
	{ .compatible = "fg27z561",},
	{ },
};

static const struct i2c_device_id fg27z561_id[] = {
	{ "fg27z561", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fg27z561_id);

static struct i2c_driver fg27z561_driver = {
	.driver = {
		.name = "fg27z561",
		.owner		= THIS_MODULE,
		.of_match_table	= fg27z561_match_table,
	},
	.probe = fg27z561_probe,
	.id_table = fg27z561_id,
};

module_i2c_driver(fg27z561_driver);

MODULE_DESCRIPTION("fg27z561 driver");
MODULE_LICENSE("GPL");

