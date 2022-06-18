#include <linux/fcntl.h> 
//#include <stdio.h>
//#include <stdlib.h>
#include <linux/unistd.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <linux/string.h>
#include <linux/syscalls.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mutex.h>
#include <linux/hid.h>
#include <linux/semaphore.h>

//Include extcon register
#include <../extcon/extcon.h>

//For HID wait for completion
#include <linux/completion.h>

#define	CLASS_NAME		    "ec_hid"
#define	TEST_STRING		    "ROG_DONGLE"
//#define HID_PATCH			"/dev/hidraw0"

// For FANDG 6
bool FANDG_USBID_detect = false;
EXPORT_SYMBOL(FANDG_USBID_detect);
bool FANDG_PDID_detect = false;
EXPORT_SYMBOL(FANDG_PDID_detect);

enum asus_dongle_type
{
	Dongle_NO_INSERT = 0,
	Dongle_INBOX5,
	Dongle_Station2,
	Dongle_DT1,
	Dongle_PCIE,
	Dongle_ERROR,
	Dongle_Others,
	Dongle_BackCover,
	Dongle_FANDG6,
	Dongle_default_status = 255,
};

enum asus_dock_event
{
	EXTRA_DOCK_STATE_UNDOCKED = 0,
	EXTRA_DOCK_STATE_ASUS_INBOX = 6,
	EXTRA_DOCK_STATE_ASUS_STATION =7,
	EXTRA_DOCK_STATE_ASUS_DT = 8,
	EXTRA_DOCK_STATE_ASUS_OTHER = 9,
/*
	EXTRA_DOCK_STATE_KEYBOARD = 10,
	EXTRA_DOCK_STATE_ASUS_PRODONGLE = 13,
	EXTRA_DOCK_STATE_ASUS_GAMEVICE = 14,
	EXTRA_DOCK_STATE_PD_GV = 15,
	EXTRA_DOCK_STATE_STATION_LOCK = 16,
	EXTRA_DOCK_STATE_STATION_LOW_BATTERY = 17,
	EXTRA_DOCK_STATE_STATION_UNLOCK = 18,
	EXTRA_DOCK_STATE_INBOX_INSERT_ERROR = 19,
	EXTRA_DOCK_STATE_STATION_INSERT_ERROR = 20,
	EXTRA_DOCK_STATE_GAMEVICE_INSERT_ERROR = 21,
	EXTRA_DOCK_STATE_STATION_INSERT_ERROR = 20,
	EXTRA_DOCK_STATE_GAMEVICE_INSERT_ERROR = 21,
	EXTRA_DOCK_STATE_PD_GV_INSERT_ERROR = 22,
*/
	EXTRA_DOCK_STATE_BACKGROUND_PAD = 23,
	EXTRA_DOCK_STATE_ASUS_2nd_INBOX = 26,
};

/*
 * 	gDongleEvent : only for Station ( gDongleType == 2 )
 *
 * 	0 	: Normal mode
 * 	1 	: Upgrade mode
 * 	2 	: Low Battery mode
 * 	3 	: ShutDown & Virtual remove mode
 */
enum asus_DongleEvent_type
{
	DongleEvent_Normal_mode = 0,
	DongleEvent_Upgrade_mode,
	DongleEvent_LowBattery_mode,
	DongleEvent_shutdown_remove_mode,
};

extern struct hidraw *g_hidraw;
static struct class *ec_hid_class;

struct ec_hid_data *g_hid_data;
EXPORT_SYMBOL(g_hid_data);

bool station_shutdown = false;
EXPORT_SYMBOL(station_shutdown);

// Block HID input report
bool block_hid_input = false;
EXPORT_SYMBOL(block_hid_input);

bool is_ec_adc_tm7_run = false;

// Record HID used status
bool hid_used = false;
EXPORT_SYMBOL(hid_used);

// Register Hall sensor
bool suspend_by_hall = false;
EXPORT_SYMBOL(suspend_by_hall);

// Sync update status with Station touch
int station_touch_recovery = 0;
EXPORT_SYMBOL(station_touch_recovery);

struct ec_hid_data {
	dev_t devt;
	struct device *dev;
	
	uint8_t previous_event;

	u8 fw_version;

	int pogo_det;
	int usb2_mux2_en;
	int pogo_fan_on;
	int pogo_aura_en;
	int pogo_temp_intr;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_default;

	bool lock;
	struct mutex report_mutex;
	struct mutex pogo_mutex;
	struct semaphore pogo_sema;
	struct mutex pogo_id_mutex;
};

