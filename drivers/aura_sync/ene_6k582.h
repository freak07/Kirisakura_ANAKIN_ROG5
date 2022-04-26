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

struct ene_6k582_platform_data {

	u8 fw_version;
	bool dongle;

//	struct regulator *regulator_vdd;
//	int regulator_vdd_vmin;
//	int regulator_vdd_vmax;
//	int regulator_current;

	int logo_5p0_en;
	u32 logo_5p0_en_flags;

	int aura_3p3_en;
	u32 aura_3p3_en_flags;

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
};

struct ene_checksum
{
    u16 len; // current chips with maskROM support flash size <= 64KB.
    u8 rev0;
    u8 rev1;
    u8 sum;
    u8 xoR;
    u8 mode;    // CHKSUM_MODE_XXX.
    u8 partSum; // partial mode check sum.
    u8 partXor; // partial mode check xor.
    u8 rp : 1;
    u8 wp : 1;
    u8 rev2 : 6;
    u16 wordsum;
    u8 rev3[4];
};
