#include <linux/leds.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/syscalls.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mutex.h>
#include <linux/kernel.h>

#define RGB_MAX 21   //for rainbow color setting
#define FEATURE_WRITE_COMMAND_SIZE 65
#define FEATURE_READ_COMMAND_SIZE 65
#define FEATURE_WRITE_48B_COMMAND_SIZE 52
#define FEATURE_READ_48B_COMMAND_SIZE 52

#define NUC1261_REPORT_ID 0x0B
#define NUC1261_CMD_LEN 5
#define NUC1261_CMD_DELAY 5	//5ms

struct inbox_drvdata {
	struct led_classdev led;
};

//extern bool g_Charger_mode;
static bool g_Charger_mode=false;

//struct mutex ms51_mutex;
struct mutex fandg6_update_lock;

struct hidraw *rog6_inbox_hidraw;
//EXPORT_SYMBOL_GPL(rog6_inbox_hidraw);

extern int usbhid_set_raw_report(struct hid_device *hid, unsigned int reportnum,
                                __u8 *buf, size_t count, unsigned char rtype);
extern int usbhid_get_raw_report(struct hid_device *hid, unsigned char report_number, __u8 *buf, size_t count,
               unsigned char report_type);

extern void FANDG_connect(int val);
extern bool FANDG_USBID_detect;

// Choose I2C addr
static u8 IC_switch;

// Debug flag
static bool DEBUG;

// Key GPIO Select
static int select_key;

enum ic_list
{
	nuc1261 = 1,
//	addr_0x16 = 1,
	addr_0x18 = 2,
	addr_0x75 = 3,
	addr_0x40 = 4,
	addr_0x44 = 5,
	addr_ohter = 255,
};
enum thermometer_chip {
	THERMOMETER_NONE,
	THERMOMETER_TI_HDC2010,
	THERMOMETER_SHT4x,
};
static u8 g_thermometer = 0;
static u8 g_ms51_addr = 0;

static u8 g_led_on = 0;
static u8 g_door_on = 0;
static u8 g_logo_on = 0;
static u8 g_cooling_en = 0;
static u8 g_cooling_stage = 0;
static u8 g_TH_meaure = 0;

//  For 2Leds MS51
static u8 g_2led_mode = 0;
static u8 g_2led_mode2 = 0;
static u8 g_2led_apply = 0;
static u8 key_state = 0;

static u32 g_2led_red_max;
static u32 g_2led_green_max;
static u32 g_2led_blue_max;
static u32 g_2led_red;
static u32 g_2led_green;
static u32 g_2led_blue;
static u32 g_2led_speed;

//  For 3Leds MS51
static u8 g_3led_mode = 0;
static u8 g_3led_mode2 = 0;
static u8 g_3led_apply = 0;

static u32 g_3led_red_max;
static u32 g_3led_green_max;
static u32 g_3led_blue_max;
static u32 g_3led_red;
static u32 g_3led_green;
static u32 g_3led_blue;
static u32 g_3led_speed;
