#include "sonacomm.h"

#include <linux/proc_fs.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include "locking.h"
#include "file_control.h"

#define asus_grip_queue "snt8100fsr-asus_queue"

extern void snt_set_pinctrl(struct device *dev, char *str);
extern void set_1V2_2V8_pin_func(struct work_struct *work_orig);
extern void asus_init_probe(void);
extern void check_i2c_error(void);

