#ifndef LASER_ASUS_MODULE_H
#define LASER_ASUS_MODULE_H

#define LOG_TAG "stmvl53l5: "

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/regulator/consumer.h>

#define MODULE_NAME                     "stmvl53l5"

#define log(fmt, args...) printk(KERN_INFO "%s: "fmt,MODULE_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "%s: "fmt,MODULE_NAME,##args)

struct regulator_info {
	struct regulator *reg;  /* voltage regulator handle */
	const char *name;       /* regulator name */
	uint32_t max_volt;      /* max voltage level */
	uint32_t min_volt;      /* min voltage level */
	uint32_t load_curr;     /* current */
	bool is_enabled;        /* is this regulator enabled? */
};

int32_t asus_laser_init(struct device *dev);
int32_t asus_laser_enable(void);
int32_t asus_laser_disable(void);
int32_t asus_laser_remove(void);

#endif
