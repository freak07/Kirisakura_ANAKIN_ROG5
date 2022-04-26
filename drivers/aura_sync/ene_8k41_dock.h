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
	int check_3v_gpio;
	u32 check_3v_flags;

	u8 fw_version;
	bool dongle;

	// Calibration
	u32 RED_MAX;
	u32 GREEN_MAX;
	u32 BLUE_MAX;

	u8 current_mode;
	bool suspend_state;

	bool FW_update_done;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;

	struct mutex ene_mutex;

	// Register framebuffer notify
	struct notifier_block notifier;

	//struct device dev;;
	struct led_classdev led;	/* LED control */
	char led_name[32];
};
