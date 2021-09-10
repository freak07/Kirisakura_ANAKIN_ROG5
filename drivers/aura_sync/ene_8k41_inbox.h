#include <linux/leds.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/syscalls.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mutex.h>
#include <linux/kernel.h>

struct ene_8k41_platform_data {
	int int_gpio;
	u32 int_flags;
	int power_gpio;
	u32 power_flags;

	u8 fw_version;
	bool dongle;

	struct regulator *regulator_vdd;
	int regulator_vdd_vmin;
	int regulator_vdd_vmax;
	int regulator_current;
	int logo_5v_en;

	// Calibration
	u32 RED_MAX;
	u32 GREEN_MAX;
	u32 BLUE_MAX;

	u8 current_mode;
	bool suspend_state;

	bool FW_update_done;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_active;

	struct mutex ene_mutex;
	int hid_suspend_id;

	// Register framebuffer notify
	struct notifier_block notifier;

	//struct device dev;;
	struct led_classdev led;	/* LED control */
	char led_name[32];

// For emulated FW pattern
	//struct delayed_work
	struct workqueue_struct		*workqueue;
	struct work_struct			aura_work;

	struct i2c_client *client;

	int on_ms;
	int off_ms;
	bool emulate;
};
