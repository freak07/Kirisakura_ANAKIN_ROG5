/*
 * Copyright (c) 2019-2020, The ASUS Company. All rights reserved.
 */

#define pr_fmt(fmt)	"BATTERY_CHG: %s: " fmt, __func__

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/soc/qcom/battery_charger_asus.h>

#include <linux/of_gpio.h>
//ASUS BSP +++
#include "fg-core.h"
#include <dt-bindings/iio/qcom,spmi-vadc.h>
#include <dt-bindings/iio/qcom,spmi-adc7-pm8350.h>
#include <dt-bindings/iio/qcom,spmi-adc7-pm8350b.h>
#include <linux/iio/consumer.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/delay.h>
#include <asus_chg_wakelock.h>
//ASUS BSP ---

//[+++] Add the structure for PMIC-GLINK response

#include <drm/drm_panel.h>

struct ADSP_ChargerPD_Info {
	int PlatformID;
	int BATT_ID;
	int    VBUS_SRC;//The VBUS is from side or bottom
	bool   chg_limit_en;
	u32    chg_limit_cap;
	bool   usbin_suspend_en;
    bool   charging_suspend_en;
	char   ChgPD_FW[64];
	char firmware_version[128];
	int batt_temp;
	int fg_real_soc;
	int cell1_voltage;
	int cell2_voltage;
};

struct evtlog_context_resp_msg3 {
	struct pmic_glink_hdr		hdr;
	u8				buf[128];
	u32				reserved;
};

struct battery_charger_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			battery_id;
	u32			property_id;
	u32			value;
};

struct battery_charger_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			property_id;
	u32			value;
	u32			ret_code;
};

struct oem_get_platformID_req {
	struct pmic_glink_hdr	hdr;
};

struct oem_get_platformID_resp {
	struct pmic_glink_hdr	hdr;
	u32    PlatID_version;
};

struct oem_get_BattID_req {
	struct pmic_glink_hdr	hdr;
};

struct oem_get_BattID_resp {
	struct pmic_glink_hdr	hdr;
	u32    Batt_ID;
};

struct oem_get_VBUS_SRC_req {
	struct pmic_glink_hdr	hdr;
};

struct oem_get_VBUS_SRC_resp {
	struct pmic_glink_hdr	hdr;
	u32    vbus_src;
};

struct oem_set_USB2_Client_resp {
	struct pmic_glink_hdr	hdr;
	u32    on;
};

struct oem_set_BTM_OTG_req {
	struct pmic_glink_hdr	hdr;
	u32    on;
};

struct oem_set_BTM_OTG_resp {
	struct pmic_glink_hdr	hdr;
};

struct oem_chg_limit_en_req {
	struct pmic_glink_hdr	hdr;
	u32    enable;
};

struct oem_chg_limit_en_resp {
	struct pmic_glink_hdr	hdr;
	u32    value;
};

struct oem_chg_limit_cap_req {
	struct pmic_glink_hdr	hdr;
	u32    cap;
};

struct oem_chg_limit_cap_resp {
	struct pmic_glink_hdr	hdr;
	u32    value;
};

struct oem_usbin_suspend_en_req {
	struct pmic_glink_hdr	hdr;
	u32    enable;
};

struct oem_usbin_suspend_en_resp {
	struct pmic_glink_hdr	hdr;
	u32    value;
};

struct oem_charging_suspend_en_req {
	struct pmic_glink_hdr	hdr;
	u32    enable;
};

struct oem_charging_suspend_en_resp {
	struct pmic_glink_hdr	hdr;
	u32    value;
};

struct oem_get_ChgPD_FW_Ver_resp {
	struct pmic_glink_hdr	hdr;
	char   ver[64] ;
};
struct oem_get_FW_version_req {
	struct pmic_glink_hdr	hdr;
};

struct oem_get_FW_version_resp {
	struct pmic_glink_hdr	hdr;
	char    fw_version[128];
};
struct oem_get_batt_temp_req {
	struct pmic_glink_hdr	hdr;
};

struct oem_get_fg_soc_req {
	struct pmic_glink_hdr	hdr;
};

struct oem_get_cell_voltage_req {
	struct pmic_glink_hdr	hdr;
};

struct oem_get_batt_temp_resp {
	struct pmic_glink_hdr	hdr;
	int    batt_temp;
};

struct oem_get_fg_soc_resp {
	struct pmic_glink_hdr	hdr;
	int    fg_soc;
};

struct oem_get_cell_voltage_resp {
	struct pmic_glink_hdr	hdr;
	u16    cell1_voltage;
	u16    cell2_voltage;
};

struct oem_set_ASUS_media_req {
	struct pmic_glink_hdr	hdr;
	u8    asus_media;
};

struct oem_set_Shutdown_mode_req {
	struct pmic_glink_hdr	hdr;
	u8    shutdown_mode;
};

struct oem_set_hwid_to_ADSP_req {
	struct pmic_glink_hdr	hdr;
	u32    hwid_val;
};

struct oem_set_hwid_to_ADSP_resp {
	struct pmic_glink_hdr	hdr;
};

struct oem_set_SideOTG_WA_resp {
	struct pmic_glink_hdr	hdr;
	u32    enable;
};

struct oem_set_Charger_Type_resp {
	struct pmic_glink_hdr	hdr;
	u32    charger_type;
};

struct battery_charger_ship_mode_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			ship_mode_type;
};

struct oem_chg_limit_mode_req {
	struct pmic_glink_hdr	hdr;
	u32    mode;
};

struct oem_chg_limit_mode_resp {
	struct pmic_glink_hdr	hdr;
	u32    value;
};

struct oem_chg_limit_mode2_req {
	struct pmic_glink_hdr	hdr;
	u32    mode;
	u32    value;
};

struct oem_evt_adsp_to_hlos_req {
	struct pmic_glink_hdr	hdr;
	u32    EvtID;
	u32    value;
};

struct oem_set_Adapter_ID_resp {
	struct pmic_glink_hdr	hdr;
	u32    adapter_id;
};

//Send cc reset req to rt1715
struct oem_set_bot_cc_reset_msg {
	struct pmic_glink_hdr hdr;
	u32 reset;
};

struct oem_get_cc_status_msg {
	struct pmic_glink_hdr hdr;
	u32 side_cc_status;
};

struct extcon_dev	*bat_id_extcon;
struct extcon_dev	*bat_extcon;
struct extcon_dev	*quickchg_extcon;
struct extcon_dev	*thermal_extcon;
struct extcon_dev	*audio_dongle_extcon;
extern int asus_extcon_set_state_sync(struct extcon_dev *edev, int cable_state);
//Define the OWNER ID
#define PMIC_GLINK_MSG_OWNER_OEM	32782
#define MSG_OWNER_BC				32778

//Define the opcode
#define OEM_ASUS_EVTLOG_IND			0x1002
#define OEM_PD_EVTLOG_IND			0x1003
#define OEM_ASUS_EVTLOG_PRINT_IND		0x1004
#define OEM_ASUS_BOT_CC_RESET_IND		0x1005
#define OME_GET_BATT_ID				0x2001
#define OEM_GET_ADSP_PLATFORM_ID	0x2002
#define OEM_GET_VBUS_SOURCE			0x2003
#define OEM_GET_CHG_LIMIT			0x2004 //OEM_SET_CHG_LIMIT = 0x2103
#define OEM_GET_CHG_LIMIT_CAP		0x2005 //OEM_SET_CHG_LIMIT_CAP = 0x2104
#define OEM_GET_USBIN_SUSPNED		0x2006 //OEM_SET_USBIN_SUSPNED = 0x2105
#define OEM_GET_ChgPD_FW_VER		0x2007
#define OEM_GET_CHARGING_SUSPNED	0x2008 //OEM_SET_CHARGING_SUSPNED = 0x2108
#define OEM_GET_CHG_LIMIT_MODE		0x2009 //OEM_SET_CHG_LIMIT_MODE = 0x2110
#define OEM_PANELONOFF_CHG_LIMIT_REQ	0x2010
#define OEM_GET_DEBUG_MASK_REQ		0x2011

#define OEM_SET_USB2_CLIENT			0x2101
#define OEM_SET_BTM_OTG				0x2102
#define OEM_SET_CHG_LIMIT			0x2103
#define OEM_SET_CHG_LIMIT_CAP		0x2104
#define OEM_SET_USBIN_SUSPNED		0x2105
#define OEM_SET_HWID_TO_ADSP		0x2106
#define OEM_SET_SideOTG_WA			0x2107
#define OEM_SET_CHARGING_SUSPNED	0x2108
#define OEM_SET_CHARGER_TYPE_CHANGE	0x2109
#define OEM_SET_CHG_LIMIT_MODE		0x2110
#define OEM_SET_DEBUG_MASK_REQ		0x2111
#define OEM_EVT_ADSP_TO_HLOS_REQ	0x2112

#define OEM_OVERWRITE_I_LIMIT_REQ	0x2113
#define OEM_OVERWRITE_V_LIMIT_REQ	0x2114
#define OEM_OVERWRITE_I_STEP_REQ	0x2115
#define OEM_VIRTUAL_THERMAL_CHG_LIMIT_REQ	0x2116
#define OEM_SET_CHG_LIMIT_MODE2		0x2117

#define OEM_SET_ADAPTER_ID_CHANGE	0x2119
#define OEM_GET_SIDEPORT_CC_STATUS_REQ	0x2120

#define OEM_GET_FW_version			0x3001
#define OEM_GET_Batt_temp			0x3002
#define OEM_GET_FG_SoC_REQ			0x3003
#define OEM_SET_ASUS_media			0x3104
#define OEM_GET_Cell_Voltage_REQ	0x3005
#define OEM_SET_Shutdown_mode       0x3106
//Define Message Type
#define MSG_TYPE_REQ_RESP	1
#define MSG_TYPE_NOTIFICATION	2

//define OEM_SET_CHG_LIMIT_MODE
#define SET_SLOW_CHG_MODE		0x1
#define SET_SIDE_THM_ALT_MODE	0x2
#define SET_BTM_THM_ALT_MODE	0x4
#define SET_BYPASS_CHG_MODE		0x8

#define SET_18W_WA_MODE			0x20

#define AUDIO_REQ_SET_LCM_MODE	0x100

//define the event ID from ADSP to HLOS
#define ADSP_HLOS_EVT_ADUIO_INVALID 0x01
#define ADSP_HLOS_EVT_VBUS_ATTACHED 0x02
//[---] Add the structure for PMIC-GLINK response

//[+++] Add debug log
#define CHARGER_TAG "[BAT][CHG]"
#define ERROR_TAG "[ERR]"
#define CHG_DBG(...)  printk(KERN_INFO CHARGER_TAG __VA_ARGS__)
#define CHG_DBG_E(...)  printk(KERN_ERR CHARGER_TAG ERROR_TAG __VA_ARGS__)
//[---] Add debug log

//[+++] Add the global variables
#define ASUS_CHARGER_TYPE_LEVEL0 0 // For disconnection, reset to default
#define ASUS_CHARGER_TYPE_LEVEL1 1 // This is for normal 18W QC3 or PD
#define ASUS_CHARGER_TYPE_LEVEL2 2 // This is for ASUS 30W adapter
#define ASUS_CHARGER_TYPE_LEVEL3 3 // This is for ASUS 65W adapter

#define SWITCH_LEVEL4_NOT_QUICK_CHARGING    8 //Now, this is used for 65W
#define SWITCH_LEVEL4_QUICK_CHARGING        7 //Now, this is used for 65W
#define SWITCH_LEVEL3_NOT_QUICK_CHARGING    6 //EQual to SWITCH_NXP_NOT_QUICK_CHARGING(ASUS 30W)
#define SWITCH_LEVEL3_QUICK_CHARGING        5 //EQual to SWITCH_NXP_QUICK_CHARGING(ASUS 30W)
#define SWITCH_LEVEL1_NOT_QUICK_CHARGING    4 //EQual to SWITCH_QC_NOT_QUICK_CHARGING(DCP 10W)
#define SWITCH_LEVEL1_QUICK_CHARGING        3 //EQual to SWITCH_QC_QUICK_CHARGING (DCP 10W)
#define SWITCH_LEVEL2_NOT_QUICK_CHARGING    2 //EQual to SWITCH_QC_NOT_QUICK_CHARGING_PLUS (QC 18W)
#define SWITCH_LEVEL2_QUICK_CHARGING        1 //EQual to SWITCH_QC_QUICK_CHARGING_PLUS (QC 18W)
#define SWITCH_LEVEL0_DEFAULT               0 //EQual to SWITCH_QC_OTHER
int g_SWITCH_LEVEL = SWITCH_LEVEL0_DEFAULT;
struct delayed_work	asus_set_qc_state_work;
#define Side_Port_Not_Asus_VID_or_No_charger 102
#define Side_Port_Asus_VID 103
#define Bottom_Port_Not_Asus_VID_or_No_charger 100
#define Bottom_Port_Asus_VID 101
bool g_ADAPTER_ID = 0;
extern bool g_Charger_mode;
bool g_IsBootComplete = 0;

extern struct battery_chg_dev *g_bcdev;
struct power_supply *qti_phy_usb;
struct power_supply *qti_phy_bat;
int PMI_MUX_GPIO;
int POGO_OTG_GPIO;
struct ADSP_ChargerPD_Info ChgPD_Info;
char st_battery_name[64] = "C21P2001-T-03-0001-0.17";
extern int usb2_host_mode;
u32 chg_limit_mode;
u32 debug_mask;
bool g_vbus_plug = 0;
volatile int g_ultra_cos_spec_time = 2880;
int g_charger_mode_full_time = 0;
int demo_recharge_delta = 2;
int factory_recharger_delta = 2;
//[+++] The possible module to set usbin_suspend
bool smartchg_stop_flag = 0;
bool bypass_stop_flag = 0;
bool node_usbin_suspend_flag = 0;
bool ultra_bat_life_flag = 0;
bool g_cos_over_full_flag = 0;
bool demo_app_property_flag = 0;
bool cn_demo_app_flag = 0;
//[---] The possible module to set usbin_suspend
bool fac_chg_limit_en = 0;
int fac_chg_limit_cap = 70;
bool g_qxdm_en = false;//add to printk the WIFI hotspot & QXDM UTS event
bool g_wifi_hs_en = false;//add to printk the WIFI hotspot & QXDM UTS event
bool feature_stop_chg_flag = false;
//bool bFactoryChgLimit = false;
struct wake_lock cable_resume_wake_lock;
struct wake_lock cos_wa_wake_lock;
//[---] Add the global variables

//[+++] Add the PDO of source for RT1715
const u32 default_rt1715_src_caps[] = { 0x00019032 };	/* 5V, 500 mA */
int default_rt1715_src_caps_size = ARRAY_SIZE(default_rt1715_src_caps);
//[---] Add the PDO of source for RT1715

bool side_port_cc_status = false;

void monitor_charging_enable(void);
int asus_set_vbus_attached_status(int value);
int asus_thermal_btm(void);
int asus_thermal_side(void);
void asus_update_thermal_result(void);

//[+++] Add the external function
extern int battery_chg_write(struct battery_chg_dev *bcdev, void *data, int len);
extern int rt_chg_get_during_swap(void);
extern bool rt_chg_check_asus_vid(void);
extern void qti_charge_notify_device_charge(void);
extern void qti_charge_notify_device_not_charge(void);

#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
typedef void(*dwc3_role_switch_fn)(bool);
dwc3_role_switch_fn dwc3_role_switch;
extern void msm_dwc3_register_switch(dwc3_role_switch_fn *funcPtr);
#endif
//[---] Add the external function
//[CR] To reset cc pin from rt1715
extern int typec_disable_function(bool disable);

//ASUS_BSP +++ LiJen add to printk the WIFI hotspot & QXDM UTS event
#include <linux/proc_fs.h>
#define uts_status_PROC_FILE	"driver/UTSstatus"
static struct proc_dir_entry *uts_status_proc_file;
static int uts_status_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "WIFIHS:%d, QXDM:%d\n", g_wifi_hs_en, g_qxdm_en);
	return 0;
}

static ssize_t uts_status_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val;
	char messages[8]="";

	len =(len > 8 ?8:len);
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);

	switch (val) {
	case 0:
		printk("%s: WIFI Hotspot disable\n");
		g_wifi_hs_en = false;
		break;
	case 1:
		printk("%s: WIFI Hotspot enable\n");
		g_wifi_hs_en = true;
		break;
	case 2:
		printk("%s: QXDM disable\n");
		g_qxdm_en = false;
		break; 
	case 3:
		printk("%s: QXDM enable\n");
		g_qxdm_en = true;
		break;       
	default:
		printk("%s: Invalid mode\n");
		break;
	}
    
	return len;
}

static int uts_status_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, uts_status_proc_read, NULL);
}

static const struct file_operations uts_status_fops = {
	.owner = THIS_MODULE,
    .open = uts_status_proc_open,
    .read = seq_read,
	.write = uts_status_proc_write,
    .release = single_release,
};

void static create_uts_status_proc_file(void)
{
	uts_status_proc_file = proc_create(uts_status_PROC_FILE, 0666, NULL, &uts_status_fops);

    if (uts_status_proc_file) {
		printk("create_uts_status_proc_file sucessed!\n");
    } else {
	    printk("create_uts_status_proc_file failed!\n");
    }
}
//ASUS_BSP --- LiJen add to printk the WIFI hotspot & QXDM UTS event

void usbin_suspend_to_ADSP(int enable);
void write_CHGLimit_value(int input);
void get_oem_batt_temp_from_ADSP(void);
void get_oem_cell_volt_from_ADSP(void);

//[+++] Add thermal alert adc function
bool usb_alert_side_flag = 0;
bool usb_alert_btm_flag = 0;
bool pre_usb_alert_side_flag = 0;
bool pre_usb_alert_btm_flag = 0;
bool g_once_usb_thermal_btm = 0;
bool g_once_usb_thermal_side = 0;
int g_temp_state = 0;
int g_temp_side_state = 0;
int g_temp_btm_state = 0;
static int usb1_active=0;
static int usb2_active=0;
static struct alarm bat_alarm;
struct timespec64  last_check_time_1m;
struct notifier_block charge_notify;
struct notifier_block host_notify;
struct iio_channel *side_usb_temp_vadc_chan;
struct iio_channel *btm_usb_temp_vadc_chan;
struct delayed_work	asus_min_check_work;
struct delayed_work	asus_18W_workaround_work;
//[+++] Add thermal alert adc function

//[+++]Add log to show charging status in ASUSEvtlog.txt
static char *charging_stats[] = {
	"UNKNOWN",
	"CHARGING",
	"DISCHARGING",
	"NOT_CHARGING",
	"FULL",
};
char *health_type[] = {
	"Unknown",
	"Good",
	"Overheat",
	"Dead",
	"Over voltage",
	"Unspecified failure",
	"Cold",
	"Watchdog timer expire",
	"Safety timer expire",
	"Over current",
	"Calibration required",
	"Warm",
	"Cool",
	"Hot",
};
char *qc_extcon_type[] = {
	"Other",
	"18W",
	"18W(N)",
	"10W",
	"10W(N)",
	"30W",
	"30W(N)",
	"65W",
	"65(W)",
};
struct delayed_work	asus_update_batt_status_work;
struct timespec64  last_check_time_3m;
//[---]Add log to show charging status in ASUSEvtlog.txt

#if 0
static const int battery_prop_map[BATT_PROP_MAX] = {
	[BATT_STATUS]		= POWER_SUPPLY_PROP_STATUS,
	[BATT_HEALTH]		= POWER_SUPPLY_PROP_HEALTH,
	[BATT_PRESENT]		= POWER_SUPPLY_PROP_PRESENT,
	[BATT_CHG_TYPE]		= POWER_SUPPLY_PROP_CHARGE_TYPE,
	[BATT_CAPACITY]		= POWER_SUPPLY_PROP_CAPACITY,
	[BATT_VOLT_OCV]		= POWER_SUPPLY_PROP_VOLTAGE_OCV,
	[BATT_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[BATT_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[BATT_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[BATT_CHG_CTRL_LIM]	= POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	[BATT_CHG_CTRL_LIM_MAX]	= POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	[BATT_TEMP]		= POWER_SUPPLY_PROP_TEMP,
	[BATT_TECHNOLOGY]	= POWER_SUPPLY_PROP_TECHNOLOGY,
	[BATT_CHG_COUNTER]	= POWER_SUPPLY_PROP_CHARGE_COUNTER,
	[BATT_CYCLE_COUNT]	= POWER_SUPPLY_PROP_CYCLE_COUNT,
	[BATT_CHG_FULL_DESIGN]	= POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	[BATT_CHG_FULL]		= POWER_SUPPLY_PROP_CHARGE_FULL,
	[BATT_MODEL_NAME]	= POWER_SUPPLY_PROP_MODEL_NAME,
	[BATT_TTF_AVG]		= POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	[BATT_TTE_AVG]		= POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	[BATT_POWER_NOW]	= POWER_SUPPLY_PROP_POWER_NOW,
	[BATT_POWER_AVG]	= POWER_SUPPLY_PROP_POWER_AVG,
};

static const int usb_prop_map[USB_PROP_MAX] = {
	[USB_ONLINE]		= POWER_SUPPLY_PROP_ONLINE,
	[USB_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[USB_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[USB_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[USB_CURR_MAX]		= POWER_SUPPLY_PROP_CURRENT_MAX,
	[USB_INPUT_CURR_LIMIT]	= POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	[USB_ADAP_TYPE]		= POWER_SUPPLY_PROP_USB_TYPE,
};
#endif
static const char * const power_supply_usb_type_text[] = {
	"Unknown", "SDP", "DCP", "CDP", "ACA", "C",
	"PD", "PD_DRP", "PD_PPS", "BrickID"
};

struct delayed_work	asus_set_ASUS_media_work;
void asus_set_ASUS_media_worker(struct work_struct *work)
{
	extern bool g_ASUS_Media;

	struct oem_set_ASUS_media_req req_msg = { { 0 } };
	int rc;

	pr_err("asuslib g_ASUS_Media= %d\n", g_ASUS_Media);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_SET_ASUS_media;
	req_msg.asus_media = g_ASUS_Media;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("[BAT][CHG] Failed to set asus_set_delta_soc rc=%d\n", rc);
	}
}

static struct notifier_block fb_notif;
static struct drm_panel *active_panel;
struct delayed_work	asus_set_panelonoff_current_work;
int g_drm_blank = 0;

int asus_set_panelonoff_charging_current_limit(u32 panelOn);
int asus_set_invalid_audio_dongle(int src, int set);

static int drm_check_dt(struct device_node *np)
{
    int i = 0;
    int count = 0;
    struct device_node *node = NULL;
    struct drm_panel *panel = NULL;

    count = of_count_phandle_with_args(np, "panel", NULL);
    if (count <= 0) {
        pr_err("find drm_panel count(%d) fail", count);
        return -ENODEV;
    }

    for (i = 0; i < count; i++) {
        node = of_parse_phandle(np, "panel", i);
        panel = of_drm_find_panel(node);
        of_node_put(node);
        if (!IS_ERR(panel)) {
            pr_err("find drm_panel successfully");
            active_panel = panel;
            return 0;
        }
    }

    pr_err("no find drm_panel");

    return -ENODEV;
}

void asus_set_panelonoff_current_worker(struct work_struct *work)
{
	if (g_drm_blank == DRM_PANEL_BLANK_UNBLANK) {
		asus_set_panelonoff_charging_current_limit(true);
	} else if (g_drm_blank == DRM_PANEL_BLANK_POWERDOWN) {
		asus_set_panelonoff_charging_current_limit(false);
	} else if (g_drm_blank == DRM_PANEL_BLANK_LP) {
		asus_set_panelonoff_charging_current_limit(false);
	}
}

static int drm_notifier_callback(struct notifier_block *self,
			unsigned long event, void *data)
{
	struct drm_panel_notifier *evdata = data;
	int *blank = NULL;

	if (!evdata) {
		printk("[BAT][CHG]drm_notifier_callback: evdata is null");
		return 0;
	}

	if (!((event == DRM_PANEL_EARLY_EVENT_BLANK)
		|| (event == DRM_PANEL_EVENT_BLANK))) {
		pr_err("event(%lu) do not need process", event);
		return 0;
	}

	blank = evdata->data;
	g_drm_blank = *blank;

	switch (*blank) {
	case DRM_PANEL_BLANK_UNBLANK:
		if (DRM_PANEL_EVENT_BLANK == event) {
			printk("[BAT][CHG] DRM_PANEL_BLANK_UNBLANK,Display on");
			printk("[BAT][CHG] asus_set_panelonoff_charging_current_limit = true");
			schedule_delayed_work(&asus_set_panelonoff_current_work, 0);
		}
		break;
	case DRM_PANEL_BLANK_POWERDOWN:
		if (DRM_PANEL_EVENT_BLANK == event) {
			printk("[BAT][CHG] DRM_PANEL_BLANK_POWERDOWN,Display off");
			printk("[BAT][CHG] asus_set_panelonoff_charging_current_limit = false");
			schedule_delayed_work(&asus_set_panelonoff_current_work, 0);
		}
		break;
	case DRM_PANEL_BLANK_LP:
		if (DRM_PANEL_EVENT_BLANK == event) {
			printk("[BAT][CHG] DRM_PANEL_BLANK_LP,Display resume into LP1/LP2");
			printk("[BAT][CHG] asus_set_panelonoff_charging_current_limit = false");
			schedule_delayed_work(&asus_set_panelonoff_current_work, 0);
		}
		break;
	case DRM_PANEL_BLANK_FPS_CHANGE:
		break;
	default:
		break;
	}

	return 0;
}

void RegisterDRMCallback()
{
	int ret = 0;

	pr_err("[BAT][CHG] RegisterDRMCallback");
	ret = drm_check_dt(g_bcdev->dev->of_node);
	if (ret) {
		pr_err("[BAT][CHG] parse drm-panel fail");
	}

	fb_notif.notifier_call = drm_notifier_callback;

	if (active_panel) {
		pr_err("[BAT][CHG] RegisterDRMCallback: registering fb notification");
		ret = drm_panel_notifier_register(active_panel, &fb_notif);
		if (ret)
			pr_err("[BAT][CHG] drm_panel_notifier_register fail: %d", ret);
	}

	return;
}
//drm_panel_notifier_register(active_panel, &

static int read_property_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id)
{
	struct battery_charger_req_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.battery_id = 0;
	req_msg.value = 0;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_get;

	pr_debug("psy: %s prop_id: %u\n", pst->psy->desc->name,
		req_msg.property_id);

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

int asus_set_charger_limit_mode(u32 mode, u32 value)
{
	struct oem_chg_limit_mode2_req req_msg = { { 0 } };
	int rc;

	CHG_DBG("%s. mode : 0x%x, value : %d\n", __func__, mode, value);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_SET_CHG_LIMIT_MODE2;
	req_msg.mode = mode;
	req_msg.value= value;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to set charger_limit_mode rc=%d\n", rc);
		return rc;
	}
	return 0;
}

int asus_set_panelonoff_charging_current_limit(u32 panelOn)
{
	struct oem_chg_limit_mode_req req_msg = { { 0 } };
	int rc;

	pr_err("panelOn= 0x%x\n", panelOn);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_PANELONOFF_CHG_LIMIT_REQ;
	req_msg.mode = panelOn;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("[BAT][CHG] Failed to set asus_set_panelonoff_charging_current_limit rc=%d\n", rc);
		return rc;
	}
	return 0;
}
//[PM_debug+++]
#if defined ASUS_ZS673KS_PROJECT
bool bBTM_OTG_EN; //used by qcom-hv-haptic.c
#endif
//[PM_debug---]
//[+++]Add the PMIC-GLINK interface of external functions for USB or RT1715 drivers
int BTM_OTG_EN(bool enable)
{
	struct oem_set_BTM_OTG_req req_msg = { { 0 } };
	int rc;

    //[PM_debug+++]
#if defined ASUS_ZS673KS_PROJECT
    bBTM_OTG_EN = enable;
#endif
    //[PM_debug---]
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_SET_BTM_OTG;
	req_msg.on = enable;
	CHG_DBG("%s. enable : %d", __func__, enable);
	if (g_bcdev == NULL) {
		pr_err("g_bcdev is null\n");
		return -1;
	}
	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to set BTM OTG rc=%d\n", rc);
		return rc;
	}
	return 0;
}
EXPORT_SYMBOL(BTM_OTG_EN);

int PASS_HWID_TO_ADSP() {
    struct oem_set_hwid_to_ADSP_req req_msg = { { 0 } };
	int rc;

	if (g_ASUS_hwID < 0) {
		CHG_DBG("%s. Incorrect HWID : %d", __func__, g_ASUS_hwID);
		return -1;
	}
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_SET_HWID_TO_ADSP;
	req_msg.hwid_val = g_ASUS_hwID;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to set HWID to ADSP rc=%d\n", rc);
		return rc;
	}
	return 0;
}
//[---]Add the PMIC-GLINK interface of external functions for USB or RT1715 drivers

static ssize_t BTM_OTG_EN1_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	int rc, tmp;
	bool btm_otg_en;
	tmp = simple_strtol(buf, NULL, 10);

	CHG_DBG("%s. Set BTM_OTG_EN : %d\n", __func__, tmp);
	btm_otg_en = tmp;
	rc = BTM_OTG_EN(btm_otg_en);
	if (rc)
		pr_err("%s. Failed to control BTM_OTG_EN\n", __func__);
	return count;
}

static ssize_t BTM_OTG_EN1_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "BTM_OTG read OK\n");
}
static CLASS_ATTR_RW(BTM_OTG_EN1);

static ssize_t pmi_mux_en_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	int rc, tmp;
	bool pmi_mux_en;
	tmp = simple_strtol(buf, NULL, 10);

	CHG_DBG("%s. Set PMI_MUX_EN : %d\n", __func__, tmp);
	pmi_mux_en = tmp;
	rc = gpio_direction_output(PMI_MUX_GPIO, pmi_mux_en);
	if (rc)
		pr_err("%s. Failed to control PMI_MUX_EN\n", __func__);
	return count;
}

static ssize_t pmi_mux_en_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	bool PMI_MUX_status;
	PMI_MUX_status = gpio_get_value_cansleep(PMI_MUX_GPIO);

	return scnprintf(buf, PAGE_SIZE, "PMI_MUX_EN : %d\n", PMI_MUX_status);
}
static CLASS_ATTR_RW(pmi_mux_en);

//[+++] Addd the interface for accessing the BATTERY power supply
static ssize_t asus_get_FG_SoC_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct oem_get_fg_soc_req req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_FG_SoC_REQ;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get battery SO rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.fg_real_soc);
}
static CLASS_ATTR_RO(asus_get_FG_SoC);

static ssize_t asus_get_cell_voltage_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct oem_get_cell_voltage_req req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_Cell_Voltage_REQ;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get battery cell voltage rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "cell1 voltage=%d, cell2 voltage=%d\n", ChgPD_Info.cell1_voltage, ChgPD_Info.cell2_voltage);
}
static CLASS_ATTR_RO(asus_get_cell_voltage);
//[---] Addd the interface for accessing the BATTERY power supply

//[+++] Add the interface for accesing the inforamtion of ChargerPD on ADSP
static ssize_t asus_get_PlatformID_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct oem_get_platformID_req req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_ADSP_PLATFORM_ID;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get PlatformID rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.PlatformID);
}
static CLASS_ATTR_RO(asus_get_PlatformID);

static ssize_t asus_get_BattID_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct oem_get_BattID_req req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OME_GET_BATT_ID;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get BattID rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.BATT_ID);
}
static CLASS_ATTR_RO(asus_get_BattID);

static ssize_t POGO_OTG_EN_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	int rc, tmp;
	bool otg_en;
	tmp = simple_strtol(buf, NULL, 10);

	CHG_DBG("%s. Set POGO_OTG_EN : %d\n", __func__, tmp);
	otg_en = tmp;
	rc = gpio_direction_output(POGO_OTG_GPIO, otg_en);
	if (rc)
		pr_err("%s. Failed to control POGO_OTG_EN\n", __func__);
	return count;
}

static ssize_t POGO_OTG_EN_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	bool POGO_OTG_status;
	POGO_OTG_status = gpio_get_value_cansleep(POGO_OTG_GPIO);

	return scnprintf(buf, PAGE_SIZE, "%d\n", POGO_OTG_status);
}
static CLASS_ATTR_RW(POGO_OTG_EN);

static ssize_t get_usb_type_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	//struct power_supply *psy;
	struct psy_state *pst;
	int rc = 0 , val = 0;

	if (g_bcdev == NULL)
		return -1;
	CHG_DBG("%s\n", __func__);
	pst = &g_bcdev->psy_list[PSY_TYPE_USB];
	rc = read_property_id(g_bcdev, pst, 7);//7:USB_ADAP_TYPE
	if (!rc) {
		val = pst->prop[7];//7:USB_ADAP_TYPE
		CHG_DBG("%s. val : %d\n", __func__, val);
	}

	return scnprintf(buf, PAGE_SIZE, "%s\n", power_supply_usb_type_text[val]);
}
static CLASS_ATTR_RO(get_usb_type);

static ssize_t vbus_side_btm_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct oem_get_VBUS_SRC_req req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_VBUS_SOURCE;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get VBUS_SOURCE rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.VBUS_SRC);
}
static CLASS_ATTR_RO(vbus_side_btm);

static ssize_t charger_limit_en_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct oem_chg_limit_en_req req_msg = { { 0 } };
	int rc, tmp;

	tmp = simple_strtol(buf, NULL, 10);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_SET_CHG_LIMIT;
	req_msg.enable = (bool) tmp;
	fac_chg_limit_en = (bool) tmp;//Tranfer this feature back to kernel

	CHG_DBG("%s. enable : %d", __func__, req_msg.enable);
	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to set CHG_LIMIT rc=%d\n", rc);
		return rc;
	}

	return count;
}

static ssize_t charger_limit_en_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct oem_chg_limit_en_resp req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_CHG_LIMIT;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get CHG_LIMIT rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.chg_limit_en);
}
static CLASS_ATTR_RW(charger_limit_en);

static ssize_t charger_limit_cap_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct oem_chg_limit_cap_req req_msg = { { 0 } };
	int rc, tmp;

	tmp = simple_strtol(buf, NULL, 10);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_SET_CHG_LIMIT_CAP;
	req_msg.cap = tmp;
	fac_chg_limit_cap = tmp;//Tranfer this feature back to kernel

	CHG_DBG("%s. cap : %d", __func__, req_msg.cap);
	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to set CHG_LIMIT_CAP rc=%d\n", rc);
		return rc;
	}

	return count;
}

static ssize_t charger_limit_cap_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct oem_chg_limit_cap_resp req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_CHG_LIMIT_CAP;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get CHG_LIMIT_CAP rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.chg_limit_cap);
}
static CLASS_ATTR_RW(charger_limit_cap);

static ssize_t usbin_suspend_en_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	int tmp = 0;

	tmp = simple_strtol(buf, NULL, 10);

	if (tmp == 1)
		node_usbin_suspend_flag = 1;
	else
		node_usbin_suspend_flag = 0;

	CHG_DBG("%s. node_usbin_suspend_flag : %d", __func__, tmp);
	monitor_charging_enable();

	return count;
}

static ssize_t usbin_suspend_en_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct oem_usbin_suspend_en_resp req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_USBIN_SUSPNED;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get USBIN_SUSPEND_EN rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.usbin_suspend_en);
}
static CLASS_ATTR_RW(usbin_suspend_en);

static ssize_t charging_suspend_en_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct oem_charging_suspend_en_req req_msg = { { 0 } };
	int rc, tmp;

	tmp = simple_strtol(buf, NULL, 10);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_SET_CHARGING_SUSPNED;
	req_msg.enable = tmp;

	CHG_DBG("%s. enable : %d", __func__, req_msg.enable);
	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to set CHARGING_SUSPEND_EN rc=%d\n", rc);
		return rc;
	}

	return count;
}

static ssize_t charging_suspend_en_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct oem_charging_suspend_en_resp req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_CHARGING_SUSPNED;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get CHARGING_SUSPEND_EN rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.charging_suspend_en);
}
static CLASS_ATTR_RW(charging_suspend_en);

static ssize_t get_ChgPD_FW_Ver_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct oem_get_ChgPD_FW_Ver_resp req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_ChgPD_FW_VER;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("%s. Failed to get ChgPD_FW_Ver rc=%d\n", __func__, rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%s\n", ChgPD_Info.ChgPD_FW);
}
static CLASS_ATTR_RO(get_ChgPD_FW_Ver);
int asus_get_Batt_ID(void)
{
	struct oem_get_BattID_req req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OME_GET_BATT_ID;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get BattID rc=%d\n", rc);
		return rc;
	}
	return 0;
}
static ssize_t asus_get_fw_version_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct oem_get_FW_version_req req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_FW_version;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get FW_version rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%s\n", ChgPD_Info.firmware_version);
}
static CLASS_ATTR_RO(asus_get_fw_version);
static ssize_t asus_get_batt_temp_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct oem_get_batt_temp_req req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_Batt_temp;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get FW_version rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.batt_temp);
}
static CLASS_ATTR_RO(asus_get_batt_temp);
//[---] Add the interface for accesing the inforamtion of ChargerPD on ADSP

//[---] Add the interface for usb thermal alert temp
static ssize_t once_usb_thermal_side_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	if (g_once_usb_thermal_side)
		return sprintf(buf, "FAIL\n");
	else
		return sprintf(buf, "PASS\n");
}
static CLASS_ATTR_RO(once_usb_thermal_side);

static ssize_t once_usb_thermal_btm_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	if (g_once_usb_thermal_btm)
		return sprintf(buf, "FAIL\n");
	else
		return sprintf(buf, "PASS\n");

}
static CLASS_ATTR_RO(once_usb_thermal_btm);
//[---] Add the interface for usb thermal alert temp

static ssize_t enter_ship_mode_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_charger_ship_mode_req_msg msg = { { 0 } };

	int rc, tmp;
	bool ship_en;
	tmp = simple_strtol(buf, NULL, 10);
	ship_en = tmp;
	if (ship_en == 0) {
		CHG_DBG("%s. NO action for SHIP mode\n", __func__);
		return count;
	}
	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = 0x36;// = BC_SHIP_MODE_REQ_SET
	msg.ship_mode_type = 0;// = SHIP_MODE_PMIC

	rc = battery_chg_write(g_bcdev, &msg, sizeof(msg));
	if (rc < 0)
		pr_err("%s. Failed to write SHIP mode: %d\n", rc);
	CHG_DBG("%s. Set SHIP Mode OK\n", __func__);

	return count;
}

static ssize_t enter_ship_mode_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "No function\n");
}
static CLASS_ATTR_RW(enter_ship_mode);

static ssize_t set_shutdown_mode_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct oem_set_Shutdown_mode_req msg = { { 0 } };

	int rc, tmp;
	bool shutdown_en;
	tmp = simple_strtol(buf, NULL, 10);
	shutdown_en = tmp;
	if (shutdown_en == 0) {
		CHG_DBG("%s. NO action for shutdown mode\n", __func__);
		return count;
	}
	msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = OEM_SET_Shutdown_mode;
	msg.shutdown_mode = 1;

	rc = battery_chg_write(g_bcdev, &msg, sizeof(msg));
	if (rc < 0)
		pr_err("%s. Failed to write shutdown mode: %d\n", rc);
	CHG_DBG("%s. Set shutdown Mode OK\n", __func__);

	return count;
}

static ssize_t set_shutdown_mode_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "No function\n");
}
static CLASS_ATTR_RW(set_shutdown_mode);


static ssize_t set_debugmask_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct oem_chg_limit_mode_req req_msg = { { 0 } };
	int rc;
	u32 mask;

	mask = (u32) simple_strtol(buf, NULL, 16);

	pr_err(" mask= 0x%x\n", mask);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_SET_DEBUG_MASK_REQ;
	req_msg.mode = mask;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("[BAT][CHG] Failed to set set_debugmask_store rc=%d\n", rc);
		return rc;
	}
	return count;
}

static ssize_t set_debugmask_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
 	struct oem_chg_limit_mode_req req_msg = { { 0 } };
	int rc;
	u32 mask;

	mask = (u32) simple_strtol(buf, NULL, 16);

	pr_err(" mask= 0x%x\n", mask);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_DEBUG_MASK_REQ;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("[BAT][CHG] Failed to set set_debugmask_store rc=%d\n", rc);
		return rc;
	}
	return scnprintf(buf, PAGE_SIZE, "0x%x\n", debug_mask);
}
static CLASS_ATTR_RW(set_debugmask);

static ssize_t set_virtualthermal_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct oem_chg_limit_mode_req req_msg = { { 0 } };
	int rc;
	u32 mask;

	mask = (u32) simple_strtol(buf, NULL, 16);

	pr_err(" mask= 0x%x\n", mask);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_VIRTUAL_THERMAL_CHG_LIMIT_REQ;
	req_msg.mode = mask;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("[BAT][CHG] Failed to set set_virtualthermal_store rc=%d\n", rc);
		return rc;
	}
	return count;
}

static ssize_t set_virtualthermal_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "No function\n");
}
static CLASS_ATTR_RW(set_virtualthermal);

static ssize_t set_i_limit_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct oem_chg_limit_mode_req req_msg = { { 0 } };
	int rc;
	u32 mask;

	mask = (u32) simple_strtol(buf, NULL, 10);

	pr_err(" set_i_limit= %d\n", mask);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_OVERWRITE_I_LIMIT_REQ;
	req_msg.mode = mask;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("[BAT][CHG] Failed to set set_i_limit_store rc=%d\n", rc);
		return rc;
	}
	return count;
}

static ssize_t set_i_limit_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "No function\n");
}
static CLASS_ATTR_RW(set_i_limit);
static ssize_t set_v_limit_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct oem_chg_limit_mode_req req_msg = { { 0 } };
	int rc;
	u32 mask;

	mask = (u32) simple_strtol(buf, NULL, 10);

	pr_err(" set_v_limit= %d\n", mask);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_OVERWRITE_V_LIMIT_REQ;
	req_msg.mode = mask;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("[BAT][CHG] Failed to set set_v_limit_store rc=%d\n", rc);
		return rc;
	}
	return count;
}

static ssize_t set_v_limit_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "No function\n");
}
static CLASS_ATTR_RW(set_v_limit);

static ssize_t set_i_step_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct oem_chg_limit_mode_req req_msg = { { 0 } };
	int rc;
	u32 mask;

	mask = (u32) simple_strtol(buf, NULL, 10);

	pr_err(" set_i_step= %d\n", mask);
	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_OVERWRITE_I_STEP_REQ;
	req_msg.mode = mask;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("[BAT][CHG] Failed to set set_i_step_store rc=%d\n", rc);
		return rc;
	}
	return count;
}

static ssize_t set_i_step_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "No function\n");
}
static CLASS_ATTR_RW(set_i_step);
static ssize_t charger_limit_mode_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	u32 mode = 0, value = 0;
	sscanf(buf, "%d,%d", &mode, &value);
	asus_set_charger_limit_mode(mode, value);

	return count;
}

static ssize_t charger_limit_mode_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "No function\n");
	//Do this later.
	#if 0
	struct oem_chg_limit_mode_resp req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_CHG_LIMIT_MODE;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get CHG_LIMIT_MODE rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", chg_limit_mode);
	#endif
}
static CLASS_ATTR_RW(charger_limit_mode);
static ssize_t ultra_bat_life_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	int tmp = 0;

	tmp = simple_strtol(buf, NULL, 10);

	if (tmp == 0) {
		ultra_bat_life_flag = 0;
	} else if (tmp == 1) {
		ultra_bat_life_flag = 1;
		write_CHGLimit_value(0);
	}
	CHG_DBG("%s: ultra_bat_life_flag = %d\n", __func__, ultra_bat_life_flag);
	monitor_charging_enable();
	return count;
}

static ssize_t ultra_bat_life_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ultra_bat_life_flag);
}
static CLASS_ATTR_RW(ultra_bat_life);

static ssize_t get_adapter_vid_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	bool btm_vid = 0;
	bool vid = 0;

	btm_vid = rt_chg_check_asus_vid();
	vid = (g_ADAPTER_ID | btm_vid);

	return scnprintf(buf, PAGE_SIZE, "%d\n", vid);
}
static CLASS_ATTR_RO(get_adapter_vid);

static ssize_t smartchg_stop_charging_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	int tmp = 0;

	tmp = simple_strtol(buf, NULL, 10);

	if (tmp == 0) {
		smartchg_stop_flag = 0;
	} else if (tmp == 1) {
		smartchg_stop_flag = 1;
	}
	CHG_DBG("%s: smartchg_stop_flag : %d\n", __func__, smartchg_stop_flag);
	monitor_charging_enable();
	return count;
}

static ssize_t smartchg_stop_charging_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", smartchg_stop_flag);
}
static CLASS_ATTR_RW(smartchg_stop_charging);

static ssize_t bypass_stop_charging_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	int tmp = 0;

	tmp = simple_strtol(buf, NULL, 10);

	if (tmp == 0) {
		bypass_stop_flag = 0;
	} else if (tmp == 1) {
		bypass_stop_flag = 1;
	}
	CHG_DBG("%s: bypass_stop_flag : %d\n", __func__, bypass_stop_flag);
	//asus_set_charger_limit_mode(SET_BYPASS_CHG_MODE, bypass_stop_flag);
	monitor_charging_enable();
	return count;
}

static ssize_t bypass_stop_charging_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", bypass_stop_flag);
}
static CLASS_ATTR_RW(bypass_stop_charging);

static ssize_t demo_app_property_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	int tmp = 0;

	tmp = simple_strtol(buf, NULL, 10);

	if (tmp == 0) {
		demo_app_property_flag = 0;
	} else if (tmp == 1) {
		demo_app_property_flag = 1;
	}
	CHG_DBG("%s: demo_app_property_flag : %d\n", __func__, demo_app_property_flag);
	monitor_charging_enable();
	return count;
}
static ssize_t demo_app_property_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", demo_app_property_flag);
}
static CLASS_ATTR_RW(demo_app_property);

static ssize_t cn_demo_app_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	int tmp = 0;

	tmp = simple_strtol(buf, NULL, 10);

	if (tmp == 0) {
		cn_demo_app_flag = 0;
	} else if (tmp == 1) {
		cn_demo_app_flag = 1;
	}
	CHG_DBG("%s: cn_demo_app_flag : %d\n", __func__, cn_demo_app_flag);
	monitor_charging_enable();
	return count;
}
static ssize_t cn_demo_app_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", cn_demo_app_flag);
}
static CLASS_ATTR_RW(cn_demo_app);

static ssize_t boot_completed_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	int tmp = 0;
	bool btm_vid = 0;

	tmp = simple_strtol(buf, NULL, 10);
	btm_vid = rt_chg_check_asus_vid();

	g_IsBootComplete = tmp;
	CHG_DBG("%s: g_IsBootComplete : %d, g_ADAPTER_ID : %d btm_vid : %d\n", __func__, g_IsBootComplete, g_ADAPTER_ID, btm_vid);
	msleep(10);
	if (g_ADAPTER_ID == true)
		asus_extcon_set_state_sync(quickchg_extcon, Side_Port_Asus_VID);
	else if (btm_vid == true)
		asus_extcon_set_state_sync(quickchg_extcon, Bottom_Port_Asus_VID);

	return count;
}
static ssize_t boot_completed_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_IsBootComplete);
}
static CLASS_ATTR_RW(boot_completed);
EXPORT_SYMBOL(g_IsBootComplete);

static struct attribute *asuslib_class_attrs[] = {
	&class_attr_BTM_OTG_EN1.attr,
	&class_attr_pmi_mux_en.attr,
	&class_attr_asus_get_FG_SoC.attr,
	&class_attr_asus_get_cell_voltage.attr,
	&class_attr_asus_get_PlatformID.attr,
	&class_attr_asus_get_BattID.attr,
	&class_attr_POGO_OTG_EN.attr,
	&class_attr_get_usb_type.attr,
	&class_attr_vbus_side_btm.attr,
	&class_attr_charger_limit_en.attr,
	&class_attr_charger_limit_cap.attr,
	&class_attr_usbin_suspend_en.attr,
	&class_attr_charging_suspend_en.attr,
	&class_attr_get_ChgPD_FW_Ver.attr,
	&class_attr_asus_get_fw_version.attr,
	&class_attr_asus_get_batt_temp.attr,
	&class_attr_once_usb_thermal_side.attr,
	&class_attr_once_usb_thermal_btm.attr,
	&class_attr_enter_ship_mode.attr,
	&class_attr_set_shutdown_mode.attr,
	&class_attr_charger_limit_mode.attr,
	&class_attr_set_debugmask.attr,
	&class_attr_set_i_limit.attr,
	&class_attr_set_v_limit.attr,
	&class_attr_set_i_step.attr,
	&class_attr_set_virtualthermal.attr,
	&class_attr_ultra_bat_life.attr,
	&class_attr_get_adapter_vid.attr,
	&class_attr_smartchg_stop_charging.attr,
	&class_attr_demo_app_property.attr,
	&class_attr_cn_demo_app.attr,
	&class_attr_bypass_stop_charging.attr,
	&class_attr_boot_completed.attr,
	NULL,
};
ATTRIBUTE_GROUPS(asuslib_class);

struct class asuslib_class = {
	.name = "asuslib",
	.class_groups = asuslib_class_groups,
};

int asus_init_power_supply_prop(void) {

	// Initialize the power supply for usb properties
	if (!qti_phy_usb)
		qti_phy_usb = power_supply_get_by_name("usb");

	if (!qti_phy_usb) {
		pr_err("Failed to get usb power supply, rc=%d\n");
		return -ENODEV;
	}

	// Initialize the power supply for battery properties
	if (!qti_phy_bat)
		qti_phy_bat = power_supply_get_by_name("battery");

	if (!qti_phy_bat) {
		pr_err("Failed to get battery power supply, rc=%d\n");
		return -ENODEV;
	}
	return 0;
};

void handle_bot_cc_reset(u32 reset)
{
	pr_err("[CHG] handle bot cc reset=%d", reset);
	
	typec_disable_function(1);
	msleep(100);
	typec_disable_function(0);
	
	return;
}

static void handle_notification(struct battery_chg_dev *bcdev, void *data,
				size_t len)
{
	struct evtlog_context_resp_msg3	*evtlog_msg;
	struct oem_set_USB2_Client_resp *usb2_client_msg;
	struct oem_set_SideOTG_WA_resp *SideOTG_WA_msg;
	struct oem_set_Charger_Type_resp *Update_charger_type_msg;
	struct oem_evt_adsp_to_hlos_req *evt_adsp_to_hlos_msg;
	struct oem_set_Adapter_ID_resp *Update_adapter_id_msg;
	struct oem_set_bot_cc_reset_msg *bot_cc_reset_msg;
	struct pmic_glink_hdr *hdr = data;
	int rc;
	static int pre_chg_type = 0;
	static u32 pre_adapter_id = 0;

    switch(hdr->opcode) {
    case OEM_ASUS_EVTLOG_PRINT_IND:
        if (len == sizeof(*evtlog_msg)) {
            evtlog_msg = data;
            pr_err("[adsp] evtlog= %s\n", evtlog_msg->buf);
            ASUSEvtlog("[BAT][ADSP]%s", evtlog_msg->buf);
        }
        break;
    case OEM_ASUS_EVTLOG_IND:
        if (len == sizeof(*evtlog_msg)) {
            evtlog_msg = data;
            pr_err("[adsp] evtlog= %s\n", evtlog_msg->buf);
        }
        break;
    case OEM_PD_EVTLOG_IND:
        if (len == sizeof(*evtlog_msg)) {
            evtlog_msg = data;
            pr_err("[PD] %s\n", evtlog_msg->buf);
        }
        break;
    case OEM_ASUS_BOT_CC_RESET_IND:
	pr_err("[adsp] OEM_BOT_CC_RESET_Ind\n");

	if(len == sizeof(*bot_cc_reset_msg)) {
	    bot_cc_reset_msg = data;
	    handle_bot_cc_reset(bot_cc_reset_msg->reset);
	}
	break;
    case OEM_SET_USB2_CLIENT:
        if (len == sizeof(*usb2_client_msg)) {
            usb2_client_msg = data;
            CHG_DBG("%s OEM_SET_USB2_CLIENT. enable : %d, usb2_host_mode : %d\n", __func__, usb2_client_msg->on, usb2_host_mode);
            if (dwc3_role_switch != NULL) {
                if (usb2_client_msg->on == 0 && rt_chg_get_during_swap())
                    CHG_DBG("Skip to send dwc3_role_switch because swap\n");
				else if (usb2_host_mode == 1)
					CHG_DBG("Skip to send dwc3_role_switch because USB2_HOST active\n");
                else
                    dwc3_role_switch(usb2_client_msg->on);
            } else {
                CHG_DBG_E("%s OEM_SET_USB2_CLIENT. dwc3_role_switch = NULL\n", __func__);
            }
        } else {
            pr_err("Incorrect response length %zu for ome_set_USB2_client\n",
                len);
        }
        break;
    case OEM_SET_SideOTG_WA:
        if (len == sizeof(*SideOTG_WA_msg)) {
            SideOTG_WA_msg = data;
            CHG_DBG("%s OEM_SET_SideOTG_WA. enable : %d, HWID : %d\n", __func__, SideOTG_WA_msg->enable, g_ASUS_hwID);
            if (g_ASUS_hwID == HW_REV_SR || (g_ASUS_hwID >= HW_REV_ER2)) {
                CHG_DBG("%s. Skip to control SideOTG for SR or ER2 later\n", __func__);
                break;
            }
            if (gpio_is_valid(POGO_OTG_GPIO)) {
                rc = gpio_direction_output(POGO_OTG_GPIO, SideOTG_WA_msg->enable);
                if (rc)
                    pr_err("%s. Failed to control POGO_OTG_EN\n", __func__);
            } else {
                CHG_DBG_E("%s. POGO_OTG_GPIO is invalid\n", __func__);
            }
        } else {
            pr_err("Incorrect response length %zu for OEM_SET_SideOTG_WA\n",
                len);
        }
        break;
    case OEM_SET_CHARGER_TYPE_CHANGE:
        if (len == sizeof(*Update_charger_type_msg)) {
            Update_charger_type_msg = data;
            CHG_DBG("%s OEM_SET_CHARGER_TYPE_CHANGE. new type : %d, old type : %d\n", __func__, Update_charger_type_msg->charger_type, pre_chg_type);
            if (Update_charger_type_msg->charger_type != pre_chg_type) {
                switch (Update_charger_type_msg->charger_type) {
                case ASUS_CHARGER_TYPE_LEVEL0:
                    g_SWITCH_LEVEL = SWITCH_LEVEL0_DEFAULT;
                break;
                case ASUS_CHARGER_TYPE_LEVEL1:
                    g_SWITCH_LEVEL = SWITCH_LEVEL2_QUICK_CHARGING;
                    set_qc_stat(POWER_SUPPLY_STATUS_CHARGING);
                break;
                case ASUS_CHARGER_TYPE_LEVEL2:
                    g_SWITCH_LEVEL = SWITCH_LEVEL3_QUICK_CHARGING;
                    set_qc_stat(POWER_SUPPLY_STATUS_CHARGING);
                break;
                case ASUS_CHARGER_TYPE_LEVEL3:
                    g_SWITCH_LEVEL = SWITCH_LEVEL4_QUICK_CHARGING;
                    set_qc_stat(POWER_SUPPLY_STATUS_CHARGING);
                break;
				default:
					g_SWITCH_LEVEL = SWITCH_LEVEL0_DEFAULT;
				break;
                }
                if (IS_ENABLED(CONFIG_QTI_PMIC_GLINK_CLIENT_DEBUG) && qti_phy_bat)
                    power_supply_changed(qti_phy_bat);
            }
            pre_chg_type = Update_charger_type_msg->charger_type;
        } else {
            pr_err("Incorrect response length %zu for OEM_SET_CHARGER_TYPE_CHANGE\n",
                len);
        }
        break;
        case OEM_EVT_ADSP_TO_HLOS_REQ:
        if (len == sizeof(*evt_adsp_to_hlos_msg)) {
            evt_adsp_to_hlos_msg = data;
            CHG_DBG("%s[OEM_EVT_ADSP_TO_HLOS_REQ] EVTID : 0x%x, value : %d\n", __func__
                    ,evt_adsp_to_hlos_msg->EvtID
                    ,evt_adsp_to_hlos_msg->value);
            if (evt_adsp_to_hlos_msg->EvtID == ADSP_HLOS_EVT_ADUIO_INVALID) {
                asus_set_invalid_audio_dongle(1, evt_adsp_to_hlos_msg->value);
            } else if (evt_adsp_to_hlos_msg->EvtID == ADSP_HLOS_EVT_VBUS_ATTACHED) {
                asus_set_vbus_attached_status(evt_adsp_to_hlos_msg->value);
            }
        }
        break;
    case OEM_SET_ADAPTER_ID_CHANGE:
        if (len == sizeof(*Update_adapter_id_msg)) {
            Update_adapter_id_msg = data;
            CHG_DBG("%s OEM_SET_ADAPTER_ID_CHANGE. new type : 0x%x, old type : 0x%x\n", __func__, Update_adapter_id_msg->adapter_id, pre_adapter_id);
            if (Update_adapter_id_msg->adapter_id != pre_adapter_id) {
                if (Update_adapter_id_msg->adapter_id == 0x0b05) {
                    CHG_DBG("%s. Set VID quickchg_extcon state: 103\n", __func__);
                    if (g_IsBootComplete == 1)
                        asus_extcon_set_state_sync(quickchg_extcon, Side_Port_Asus_VID);
                    g_ADAPTER_ID = true;
                } else {
                    CHG_DBG("%s. Set VID quickchg_extcon state: 102\n", __func__);
                    if (g_IsBootComplete == 1)
                        asus_extcon_set_state_sync(quickchg_extcon, Side_Port_Not_Asus_VID_or_No_charger);
                    g_ADAPTER_ID = false;
                }

                if (IS_ENABLED(CONFIG_QTI_PMIC_GLINK_CLIENT_DEBUG) && qti_phy_bat)
                    power_supply_changed(qti_phy_bat);
            }
            pre_adapter_id = Update_adapter_id_msg->adapter_id;
        } else {
			pr_err("Incorrect response length %zu for OEM_SET_ADAPTER_ID_CHANGE\n",
                len);
        }
        break;
    default:
        pr_err("Unknown opcode: %u\n", hdr->opcode);
        break;
    }

}

static void handle_message(struct battery_chg_dev *bcdev, void *data,
				size_t len)
{
	struct oem_get_platformID_resp *platform_id_msg;
	struct oem_get_BattID_resp *batt_id_msg;
	struct oem_get_VBUS_SRC_resp *vbus_src_msg;
	struct oem_set_BTM_OTG_resp *btm_otg_resp_msg;
	struct oem_chg_limit_en_req *chg_limit_en_req_msg;
	struct oem_chg_limit_en_resp *chg_limit_en_resp_msg;
	struct oem_chg_limit_cap_req *chg_limit_cap_req_msg;
	struct oem_chg_limit_cap_resp *chg_limit_cap_resp_msg;
	struct oem_usbin_suspend_en_req *usbin_suspend_en_req_msg;
	struct oem_usbin_suspend_en_resp *usbin_suspend_en_resp_msg;
    struct oem_charging_suspend_en_req *charging_suspend_en_req_msg;
    struct oem_charging_suspend_en_resp *charging_suspend_en_resp_msg;
	struct oem_get_ChgPD_FW_Ver_resp *ChgPD_FW_Ver_msg;
	struct oem_set_hwid_to_ADSP_resp *set_HWID_To_ADSP_msg;
	struct oem_chg_limit_mode_req *chg_limit_mode_req_msg;
	struct oem_chg_limit_mode2_req *chg_limit_mode2_req_msg;
	struct oem_chg_limit_mode_resp *chg_limit_mode_resp_msg;
	struct pmic_glink_hdr *hdr = data;
	bool ack_set = false;
	struct oem_get_FW_version_resp *fw_version_msg;
	struct oem_get_batt_temp_resp *batt_temp_msg;
	struct oem_get_fg_soc_resp *fg_soc_msg;
	struct oem_get_cell_voltage_resp *cell_voltage_msg;
	struct oem_set_ASUS_media_req *set_asus_media_msg;
	struct oem_get_cc_status_msg *get_cc_status_msg;
	struct oem_set_Shutdown_mode_req *set_shutdown_mode_msg;

	switch (hdr->opcode) {
	case OEM_GET_ADSP_PLATFORM_ID:
		if (len == sizeof(*platform_id_msg)) {
			platform_id_msg = data;
			ChgPD_Info.PlatformID = platform_id_msg->PlatID_version;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for asus_get_PlatformID\n",
				len);
		}
		break;
	case OME_GET_BATT_ID:
		if (len == sizeof(*batt_id_msg)) {
			batt_id_msg = data;
			ChgPD_Info.BATT_ID = batt_id_msg->Batt_ID;
			ack_set = true;
			if(ChgPD_Info.BATT_ID < 51000*1.15 && ChgPD_Info.BATT_ID > 51000*0.85)
				asus_extcon_set_state_sync(bat_id_extcon, 1);
			else if(ChgPD_Info.BATT_ID < 100000*1.15 && ChgPD_Info.BATT_ID > 100000*0.85)
				asus_extcon_set_state_sync(bat_id_extcon, 1);
			else if(ChgPD_Info.BATT_ID < 10000*1.15 && ChgPD_Info.BATT_ID > 10000*0.85)
				asus_extcon_set_state_sync(bat_id_extcon, 1);
			else
				asus_extcon_set_state_sync(bat_id_extcon, 0);
		} else {
			pr_err("Incorrect response length %zu for asus_get_BattID\n",
				len);
		}
		break;
	case OEM_GET_FW_version:
		if (len == sizeof(*fw_version_msg)) {
			fw_version_msg = data;
			strcpy(ChgPD_Info.firmware_version, fw_version_msg->fw_version);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for asus_FW_version\n",
				len);
		}
		break;
	case OEM_GET_Batt_temp:
		if (len == sizeof(*batt_temp_msg)) {
			batt_temp_msg = data;
			ChgPD_Info.batt_temp = batt_temp_msg->batt_temp;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for asus_batt_temp\n",
				len);
		}
		break;
	case OEM_GET_FG_SoC_REQ:
		if (len == sizeof(*fg_soc_msg)) {
			fg_soc_msg = data;
			ChgPD_Info.fg_real_soc = fg_soc_msg->fg_soc;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_GET_FG_SoC_REQ\n",
				len);
		}
		break;
	case OEM_GET_Cell_Voltage_REQ:
		if (len == sizeof(*cell_voltage_msg)) {
			cell_voltage_msg = data;
			ChgPD_Info.cell1_voltage = cell_voltage_msg->cell1_voltage;
			ChgPD_Info.cell2_voltage = cell_voltage_msg->cell2_voltage;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for asus_get_cell_voltage\n",
				len);
		}
		break;
	case OEM_SET_ASUS_media:
		if (len == sizeof(*set_asus_media_msg)) {
			CHG_DBG("%s OEM_SET_ASUS_media successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_SET_ASUS_media\n",
				len);
		}
		break;
	case OEM_SET_Shutdown_mode:
		if (len == sizeof(*set_shutdown_mode_msg)) {
			CHG_DBG("%s OEM_SET_Shutdown_mode successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_SET_Shutdown_mode\n",
				len);
		}
		break;
	case OEM_GET_VBUS_SOURCE:
		if (len == sizeof(*vbus_src_msg)) {
			vbus_src_msg = data;
			ChgPD_Info.VBUS_SRC 	= vbus_src_msg->vbus_src;
			ack_set = true;
			g_vbus_plug = vbus_src_msg->vbus_src;
			CHG_DBG("%s g_vbus_plug = %d\n", __func__, g_vbus_plug);
			if (g_vbus_plug == 1)
				qti_charge_notify_device_charge();
			else
				qti_charge_notify_device_not_charge();
		} else {
			pr_err("Incorrect response length %zu for get VBUS_SRC\n",
				len);
		}
		break;
	case OEM_SET_BTM_OTG:
		if (len == sizeof(*btm_otg_resp_msg)) {
			CHG_DBG("%s OEM_SET_BTM_OTG successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_SET_BTM_OTG\n",
				len);
		}
		break;
	case OEM_SET_CHG_LIMIT:
		if (len == sizeof(*chg_limit_en_req_msg)) {
			CHG_DBG("%s OEM_SET_CHG_LIMIT successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_SET_CHG_LIMIT\n",
				len);
		}
		break;
	case OEM_GET_CHG_LIMIT:
		if (len == sizeof(*chg_limit_en_resp_msg)) {
			chg_limit_en_resp_msg = data;
			CHG_DBG("%s OEM_GET_CHG_LIMIT successfully\n", __func__);
			ChgPD_Info.chg_limit_en = chg_limit_en_resp_msg->value;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_GET_CHG_LIMIT\n",
				len);
		}
		break;
	case OEM_SET_CHG_LIMIT_CAP:
		CHG_DBG("%s. OEM_SET_CHG_LIMIT_CAP. len :%d, sizeof : %d \n", __func__, len, sizeof(*chg_limit_cap_req_msg));
		if (len == sizeof(*chg_limit_cap_req_msg)) {
			CHG_DBG("%s OEM_SET_CHG_LIMIT_CAP successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_SET_CHG_LIMIT_CAP\n",
				len);
		}
		break;
	case OEM_GET_CHG_LIMIT_CAP:
		CHG_DBG("%s. OEM_GET_CHG_LIMIT_CAP. len :%d, sizeof : %d \n", __func__, len, sizeof(*chg_limit_cap_resp_msg));
		if (len == sizeof(*chg_limit_cap_resp_msg)) {
			chg_limit_cap_resp_msg = data;
			CHG_DBG("%s OEM_GET_CHG_LIMIT_CAP successfully\n", __func__);
			ChgPD_Info.chg_limit_cap = chg_limit_cap_resp_msg->value;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_GET_CHG_LIMIT_CAP\n",
				len);
		}
		break;
	case OEM_SET_USBIN_SUSPNED:
		if (len == sizeof(*usbin_suspend_en_req_msg)) {
			CHG_DBG("%s OEM_SET_USBIN_SUSPNED successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_SET_USBIN_SUSPNED\n",
				len);
		}
		break;
	case OEM_GET_USBIN_SUSPNED:
		if (len == sizeof(*usbin_suspend_en_resp_msg)) {
			usbin_suspend_en_resp_msg = data;
			CHG_DBG("%s OEM_GET_USBIN_SUSPNED successfully\n", __func__);
			ChgPD_Info.usbin_suspend_en = usbin_suspend_en_resp_msg->value;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_GET_USBIN_SUSPNED\n",
				len);
		}
		break;
    case OEM_SET_CHARGING_SUSPNED:
		if (len == sizeof(*charging_suspend_en_req_msg)) {
			CHG_DBG("%s OEM_SET_CHARGING_SUSPNED successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_SET_CHARGING_SUSPNED\n",
				len);
		}
		break;
	case OEM_GET_CHARGING_SUSPNED:
		if (len == sizeof(*charging_suspend_en_resp_msg)) {
			charging_suspend_en_resp_msg = data;
			CHG_DBG("%s OEM_GET_CHARGING_SUSPNED successfully\n", __func__);
			ChgPD_Info.charging_suspend_en = charging_suspend_en_resp_msg->value;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_GET_CHARGING_SUSPNED\n",
				len);
		}
		break;
	case OEM_GET_ChgPD_FW_VER:
		if (len == sizeof(*ChgPD_FW_Ver_msg)) {
			ChgPD_FW_Ver_msg = data;
			CHG_DBG("%s. ChgPD_FW : %s\n", __func__, ChgPD_FW_Ver_msg->ver);
			strcpy(ChgPD_Info.ChgPD_FW, ChgPD_FW_Ver_msg->ver);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_GET_ChgPD_FW_VER\n",
				len);
		}
		break;
	case OEM_SET_HWID_TO_ADSP:
		if (len == sizeof(*set_HWID_To_ADSP_msg)) {
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_SET_HWID_TO_ADSP\n",
				len);
		}
		break;
	case OEM_SET_CHG_LIMIT_MODE:
		if (len == sizeof(*chg_limit_mode_req_msg)) {
			CHG_DBG("%s OEM_SET_CHG_LIMIT_MODE successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_SET_CHG_LIMIT_MODE\n",
				len);
		}
		break;
	case OEM_SET_CHG_LIMIT_MODE2:
		if (len == sizeof(*chg_limit_mode2_req_msg)) {
			CHG_DBG("%s OEM_SET_CHG_LIMIT_MODE2 successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_SET_CHG_LIMIT_MODE2\n",
				len);
		}
		break;
	case OEM_GET_CHG_LIMIT_MODE:
		if (len == sizeof(*chg_limit_mode_resp_msg)) {
			chg_limit_mode_resp_msg = data;
			CHG_DBG("%s OEM_GET_CHG_LIMIT_MODE successfully\n", __func__);
			chg_limit_mode = chg_limit_mode_resp_msg->value;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_GET_CHG_LIMIT_MODE\n",
				len);
		}
		break;
	case OEM_PANELONOFF_CHG_LIMIT_REQ:
		if (len == sizeof(*chg_limit_mode_req_msg)) {
			CHG_DBG("%s OEM_PANELONOFF_CHG_LIMIT_REQ successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_PANELONOFF_CHG_LIMIT_REQ\n",
				len);
		}
		break;
	case OEM_SET_DEBUG_MASK_REQ:
		if (len == sizeof(*chg_limit_mode_req_msg)) {
			CHG_DBG("%s OEM_SET_DEBUG_MASK_REQ successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_SET_DEBUG_MASK_REQ\n",
				len);
		}
		break;
	case OEM_GET_DEBUG_MASK_REQ:
		if (len == sizeof(*chg_limit_mode_resp_msg)) {
			chg_limit_mode_resp_msg = data;
			CHG_DBG("%s OEM_GET_DEBUG_MASK_REQ successfully\n", __func__);
			debug_mask = chg_limit_mode_resp_msg->value;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_GET_DEBUG_MASK_REQ\n",
				len);
		}
		break;
	case OEM_VIRTUAL_THERMAL_CHG_LIMIT_REQ:
		if (len == sizeof(*chg_limit_mode_req_msg)) {
			CHG_DBG("%s OEM_VIRTUAL_THERMAL_CHG_LIMIT_REQ successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_VIRTUAL_THERMAL_CHG_LIMIT_REQ\n",
				len);
		}
		break;
	case OEM_OVERWRITE_I_LIMIT_REQ:
		if (len == sizeof(*chg_limit_mode_req_msg)) {
			CHG_DBG("%s OEM_OVERWRITE_I_LIMIT_REQ successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_OVERWRITE_I_LIMIT_REQ\n",
				len);
		}
		break;
	case OEM_OVERWRITE_V_LIMIT_REQ:
		if (len == sizeof(*chg_limit_mode_req_msg)) {
			CHG_DBG("%s OEM_OVERWRITE_V_LIMIT_REQ successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_OVERWRITE_V_LIMIT_REQ\n",
				len);
		}
		break;
	case OEM_OVERWRITE_I_STEP_REQ:
		if (len == sizeof(*chg_limit_mode_req_msg)) {
			CHG_DBG("%s OEM_OVERWRITE_I_STEP_REQ successfully\n", __func__);
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_OVERWRITE_I_STEP_REQ\n",
				len);
		}
		break;
	case OEM_GET_SIDEPORT_CC_STATUS_REQ:
		if (len == sizeof(*get_cc_status_msg)) {
			get_cc_status_msg = data;
			side_port_cc_status = get_cc_status_msg->side_cc_status;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for OEM_GET_SIDEPORT_CC_STATUS_REQ\n",
				len);
		}
		break;
	default:
		pr_err("Unknown opcode: %u\n", hdr->opcode);
		ack_set = true;
		break;
	}

	if (ack_set)
		complete(&bcdev->ack);
}

static int asusBC_msg_cb(void *priv, void *data, size_t len)
{
	struct pmic_glink_hdr *hdr = data;

	//pr_err("owner: %u type: %u opcode: %u len: %zu\n", hdr->owner, hdr->type, hdr->opcode, len);

	if (hdr->owner == PMIC_GLINK_MSG_OWNER_OEM) {
		if (hdr->type == MSG_TYPE_NOTIFICATION)
			handle_notification(g_bcdev, data, len);
		else
			handle_message(g_bcdev, data, len);
	}
	return 0;
}

static void asusBC_state_cb(void *priv, enum pmic_glink_state state)
{
	pr_err("Enter asusBC_state_cb\n");
}


int asus_set_invalid_audio_dongle(int src, int set) {
	static int side_value = 0, btm_value = 0;
	static int final_value = 0, pre_value = 0;

	if (src == 1)
		side_value = set;
	else if (src == 2)
		btm_value = set;

	final_value = side_value | btm_value;
	CHG_DBG("invalid_audio_dongle. src = %d, set = %d, final_value = %d\n", src, set, final_value);
	if (pre_value != final_value)
		asus_extcon_set_state_sync(audio_dongle_extcon, final_value);

	pre_value = final_value;

	return 0;
}
EXPORT_SYMBOL(asus_set_invalid_audio_dongle);

int asus_set_vbus_attached_status(int value) {
	//int ret = 0;
	if (value) {
		g_vbus_plug = 1;
		ASUSEvtlog("[BAT][Ser]Cable Plug-in");
		qti_charge_notify_device_charge();
		schedule_delayed_work(&asus_min_check_work, 0);
		schedule_delayed_work(&asus_18W_workaround_work, msecs_to_jiffies(10000));
	} else {
		g_vbus_plug = 0;
		ASUSEvtlog("[BAT][Ser]Cable Plug-out");
		qti_charge_notify_device_not_charge();
		//[+++]Reset the parameter for limiting the charging
		feature_stop_chg_flag = false;
		//bFactoryChgLimit = false;
		//[---]Reset the parameter for limiting the charging

		g_once_usb_thermal_side = 0;
		g_once_usb_thermal_btm = 0;

		//[+++]Update the status of thermal alert
		asus_update_thermal_result();
		//[---]Update the status of thermal alert

		alarm_cancel(&bat_alarm);
		cancel_delayed_work(&asus_min_check_work);
		cancel_delayed_work(&asus_18W_workaround_work);
		wake_unlock(&cable_resume_wake_lock);
	}
	CHG_DBG("%s: g_vbus_plug : %d \n", __func__, g_vbus_plug);
	return 0;
}

static DEFINE_SPINLOCK(bat_alarm_slock);
static enum alarmtimer_restart batAlarm_handler(struct alarm *alarm, ktime_t now)
{
	CHG_DBG("%s: batAlarm triggered\n", __func__);
	return ALARMTIMER_NORESTART;
}

int g_temp_THR_SIDE = 85000;
int g_recovery_temp_THR_SIDE = 75000;
int g_temp_THR_BTM = 70000;
int g_recovery_temp_THR_BTM = 60000;

int asus_thermal_side(void)
{
	int rc;
	int adc_temp;

	if (IS_ERR_OR_NULL(side_usb_temp_vadc_chan)) {
		CHG_DBG_E("%s: iio_channel not ready\n", __func__);
		return -ENXIO;
	}

	rc = iio_read_channel_processed(side_usb_temp_vadc_chan, &adc_temp);
	if (rc < 0)
		CHG_DBG_E("%s: iio_read_channel_processed fail\n", __func__);
	else
		CHG_DBG("%s: side_adc_temp = %d\n", __func__, adc_temp);

	if (adc_temp > g_temp_THR_SIDE && !usb_alert_side_flag) {
		usb_alert_side_flag = 1;
		g_once_usb_thermal_side = 1;
		//asus_set_charger_limit_mode(SET_SIDE_THM_ALT_MODE, 1);
	} else if (adc_temp < g_recovery_temp_THR_SIDE && usb_alert_side_flag) {
		usb_alert_side_flag = 0;
		//asus_set_charger_limit_mode(SET_SIDE_THM_ALT_MODE, 0);
	}
	return 0;
}

int asus_thermal_btm(void)
{
	int rc;
	int adc_temp;

	if (IS_ERR_OR_NULL(btm_usb_temp_vadc_chan)) {
		CHG_DBG_E("%s: iio_channel not ready\n", __func__);
		return -ENXIO;
	}

	rc = iio_read_channel_processed(btm_usb_temp_vadc_chan, &adc_temp);
	if (rc < 0)
		CHG_DBG_E("%s: iio_read_channel_processed fail\n", __func__);
	else
		CHG_DBG("%s: btm_adc_temp = %d\n", __func__, adc_temp);

	if (adc_temp > g_temp_THR_BTM && !usb_alert_btm_flag) {
		usb_alert_btm_flag = 1;
		g_once_usb_thermal_btm = 1;
		//asus_set_charger_limit_mode(SET_BTM_THM_ALT_MODE, 1);
	} else if (adc_temp < g_recovery_temp_THR_BTM && usb_alert_btm_flag) {
		usb_alert_btm_flag = 0;
		//asus_set_charger_limit_mode(SET_BTM_THM_ALT_MODE, 0);
	}

	return 0;
}

void asus_update_thermal_result(void)
{
	int ret = 0;
	//Update the status of thermal alert flag
	ret = asus_thermal_side();
	if (ret < 0)
		CHG_DBG_E("%s: asus_thermal_side fail\n", __func__);

	ret = asus_thermal_btm();
	if (ret < 0)
		CHG_DBG_E("%s: asus_thermal_btm fail\n", __func__);

	if (thermal_extcon == NULL) {
		CHG_DBG_E("%s: Skip to set thermal_extcon(NULL). \n", __func__);
		return;
	}

	//Update the status of thermal alert extcon
	CHG_DBG("%s: usb_alert_side_flag : %d, usb_alert_btm_flag : %d, g_vbus_plug : %d\n",
	__func__, usb_alert_side_flag, usb_alert_btm_flag, g_vbus_plug);
	if ((usb_alert_side_flag | usb_alert_btm_flag) && g_vbus_plug) {
		CHG_DBG("%s: Set thermal_extcon 2\n", __func__);
		asus_extcon_set_state_sync(thermal_extcon, 2);
	} else if (usb_alert_side_flag && usb1_active) {
		CHG_DBG("%s: Set thermal_extcon 2\n", __func__);
		asus_extcon_set_state_sync(thermal_extcon, 2);
	} else if (usb_alert_btm_flag && usb2_active) {
		CHG_DBG("%s: Set thermal_extcon 2\n", __func__);
		asus_extcon_set_state_sync(thermal_extcon, 2);
	} else if (usb_alert_side_flag || usb_alert_btm_flag) {
		CHG_DBG("%s: Set thermal_extcon 1\n", __func__);
		asus_extcon_set_state_sync(thermal_extcon, 1);
	} else {
		CHG_DBG("%s: Set thermal_extcon 0\n", __func__);
		asus_extcon_set_state_sync(thermal_extcon, 0);
	}

	//Update the status of USBIN_SUSPEND
	if (usb_alert_side_flag != pre_usb_alert_side_flag)
		asus_set_charger_limit_mode(SET_SIDE_THM_ALT_MODE, usb_alert_side_flag);

	if (usb_alert_btm_flag != pre_usb_alert_btm_flag)
		asus_set_charger_limit_mode(SET_BTM_THM_ALT_MODE, usb_alert_btm_flag);
	
	pre_usb_alert_side_flag = usb_alert_side_flag;
	pre_usb_alert_btm_flag = usb_alert_btm_flag;

	return;
}

/*+++ Add demo app read ADF function +++*/
#define ADF_PATH "/ADF/ADF"
static bool ADF_check_status(void)
{
    char buf[32];
	struct file *fd;
	struct inode *inode;
	off_t fsize;
	loff_t pos;
	mm_segment_t old_fs;

	if (g_Charger_mode)
		return false;

	fd = filp_open(ADF_PATH, O_RDONLY, 0);
	if (IS_ERR_OR_NULL(fd)) {
        CHG_DBG("%s: OPEN (%s) failed\n", __func__, ADF_PATH);
		return -ENODATA;
    }

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	inode = fd->f_path.dentry->d_inode;
	fsize = inode->i_size;
	pos = 0;

	vfs_read(fd, buf, fsize, &pos);

	filp_close(fd, NULL);
	set_fs(old_fs);

	if (buf[3] == 1 || buf[3] == 2)
		return true;
	else
		return false;
}
/*--- Add demo app read ADF function--- */

void monitor_charging_enable(void)
{
	union power_supply_propval prop = {};
	int bat_capacity;
	int rc;
	bool demo_app_state_flag = 0;
	bool charging_pause = 0;
	bool feature_usbin_suspend = 0;
	static bool pre_charging_pause = 0;

	//Case 1 : For factory test, Has the highest priority to suspend USBIN
	if (node_usbin_suspend_flag) {
		CHG_DBG("%s: USBIN suspend because of node_usbin_suspend_flag = 1\n", __func__);
		usbin_suspend_to_ADSP(1);
		return;
	}

	//Get the capacity of the battery
	rc = power_supply_get_property(qti_phy_bat, POWER_SUPPLY_PROP_CAPACITY, &prop);
	if (rc < 0)
		pr_err("Failed to get battery SOC, rc=%d\n", rc);
	bat_capacity = prop.intval;

	//Case 2 : For Factory test, limit the max battery SoC
	// Charger_limit_function for factory +++
	#if 0
	if (fac_chg_limit_en) {
		if (bat_capacity > fac_chg_limit_cap) {
			bFactoryChgLimit = true;
		} else if (bat_capacity <= (fac_chg_limit_cap - factory_recharger_delta)) {
			bFactoryChgLimit = false;
		}
	} else {
		bFactoryChgLimit = false;
	}
	#endif
	if (fac_chg_limit_en && bat_capacity >= fac_chg_limit_cap) {
		charging_pause = 1;
	}

	// Case 3 : Add Maximun Battery Lifespan +++
	//Check if the device reaches 48 hours in COS
	if (g_Charger_mode) {
		if (bat_capacity == 100 && !g_cos_over_full_flag) {
			g_charger_mode_full_time ++;
			if (g_charger_mode_full_time >= g_ultra_cos_spec_time) {
				write_CHGLimit_value(1);
				g_cos_over_full_flag = 1;
			}
		}
	}
	// Add Maximun Battery Lifespan ---

	//[+++]Case 4 : For ww/cn demo app, limit the max battery SoC
	if (demo_app_property_flag)
		demo_app_state_flag = ADF_check_status();
	//[---]Case 4 : For ww/cn demo app, limit the max battery SoC

	if (cn_demo_app_flag || demo_app_state_flag || ultra_bat_life_flag || g_cos_over_full_flag) {
		if (bat_capacity > 60) {
			feature_usbin_suspend = 1;
			feature_stop_chg_flag = true;
		} else if (bat_capacity >= (60 - demo_recharge_delta)) {
			feature_usbin_suspend = 0;
			feature_stop_chg_flag = true;
		} else {
			feature_usbin_suspend = 0;
			feature_stop_chg_flag = false;
		}
	} else {
		feature_stop_chg_flag = false;
		feature_usbin_suspend = 0;
	}

	CHG_DBG("%s: smartchg_stop_flag : %d, bypass_stop_flag : %d\n"
	, __func__, smartchg_stop_flag, bypass_stop_flag);
	CHG_DBG("%s: bat_capacity : %d. ultra_bat_life_flag : %d, g_cos_over_full_flag : %d\n"
	, __func__, bat_capacity, ultra_bat_life_flag, g_cos_over_full_flag);
	CHG_DBG("%s: fac_chg_limit_en : %d, fac_chg_limit_cap : %d\n"
	, __func__, fac_chg_limit_en, fac_chg_limit_cap);
	CHG_DBG("%s: demo_app_property_flag : %d, demo_app_state_flag : %d, cn_demo_app_flag : %d\n"
	, __func__, demo_app_property_flag, demo_app_state_flag, cn_demo_app_flag);

	if (smartchg_stop_flag || bypass_stop_flag || feature_stop_chg_flag) {
		charging_pause = 1;
	}

	CHG_DBG("%s: charging_pause : %d, pre_charging_pause : %d\n", __func__, charging_pause, pre_charging_pause);
	if (charging_pause != pre_charging_pause) {
		asus_set_charger_limit_mode(SET_BYPASS_CHG_MODE, charging_pause);
		pre_charging_pause = charging_pause;
	}

	CHG_DBG("%s: g_once_usb_thermal_btm : %d, g_once_usb_thermal_side : %d, feature_usbin_suspend : %d\n",
			__func__, g_once_usb_thermal_btm, g_once_usb_thermal_side, feature_usbin_suspend);
	//Check if to set usbin_suspend to ADSP
	if (g_once_usb_thermal_btm || g_once_usb_thermal_side || feature_usbin_suspend) {
		usbin_suspend_to_ADSP(1);
	} else {
		usbin_suspend_to_ADSP(0);
	}
}

void asus_min_check_worker(struct work_struct *work)
{
	unsigned long batflags;
	struct timespec new_batAlarm_time;
	int RTCSetInterval = 60;
	//int rc;
	//bool thermal_extcon_state;

	if (!g_vbus_plug)
		return;

	if (!g_bcdev) {
		CHG_DBG_E("%s: driver not ready yet!\n", __func__);
		pm_relax(g_bcdev->dev);
		return;
	}

	//[+++]Update the thermal alert status
	#if 0
	rc = asus_thermal_side();
	if (rc < 0)
		CHG_DBG_E("%s: asus_thermal_side fail\n", __func__);

	rc = asus_thermal_btm();
	if (rc < 0)
		CHG_DBG_E("%s: asus_thermal_side fail\n", __func__);

	thermal_extcon_state = usb_alert_side_flag | usb_alert_btm_flag;
	if (g_vbus_plug && thermal_extcon_state) {
		CHG_DBG("%s: Set thermal_extcon 2\n", __func__);
		asus_extcon_set_state_sync(thermal_extcon, 2);
	} else if (thermal_extcon_state) {
		CHG_DBG("%s: Set thermal_extcon 1\n", __func__);
		asus_extcon_set_state_sync(thermal_extcon, 1);
	} else {
		CHG_DBG("%s: Set thermal_extcon 0\n", __func__);
		asus_extcon_set_state_sync(thermal_extcon, 0);
	}
	#endif
	asus_update_thermal_result();
	//[---]Update the thermal alert status

	ktime_get_coarse_real_ts64(&last_check_time_1m);
	new_batAlarm_time.tv_sec = last_check_time_1m.tv_sec + RTCSetInterval;

	spin_lock_irqsave(&bat_alarm_slock, batflags);
	alarm_start(&bat_alarm, timespec_to_ktime(new_batAlarm_time));
	spin_unlock_irqrestore(&bat_alarm_slock, batflags);

	//monitor_charging_enable(jeita_charging_enable);
	monitor_charging_enable();//Always TRUE, JEITA is decided on ADSP

	schedule_delayed_work(&asus_min_check_work, msecs_to_jiffies(60000));

	pm_relax(g_bcdev->dev);
}

void asus_18W_workaround_worker(struct work_struct *work)
{
	int rc = 0;

	CHG_DBG("%s.\n", __func__);
	rc = asus_set_charger_limit_mode(SET_18W_WA_MODE, 0);
	if (rc < 0)
		CHG_DBG_E("%s: Failed to set asus_set_charger_limit_mode, rc=%d\n", __func__, rc);
}

#if 0
static int charge_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
	static unsigned long pre_event = 0;

	CHG_DBG("%s: start, event:%lu\n", __func__, event);

	if (pre_event == event) {
		CHG_DBG("%s: same event %lu, return\n", __func__, event);
		return 0;
	}
	pre_event = event;

    switch (event) {
    case QTI_POWER_SUPPLY_CHARGED:
		CHG_DBG("%s: vbus plugin\n", __func__);
		g_vbus_plug = 1;
		schedule_delayed_work(&asus_min_check_work, 0);
        break;
    case QTI_POWER_SUPPLY_UNCHARGED:
		CHG_DBG("%s: vbus plugout\n", __func__);
		g_vbus_plug = 0;
		alarm_cancel(&bat_alarm);
		cancel_delayed_work(&asus_min_check_work);
		break;
    }
	return 0;
}
#endif

#define USB_BUS_ADD			0x0003
#define USB_BUS_REMOVE		0x0004
static int host_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
	struct usb_bus *ubus = data;
	char usb1[]="a600000.dwc3";
	char usb2[]="a800000.dwc3";
	char devname[15];
	//int ret = 0;

	if(event >= 3){
		if(ubus->controller->parent) {
			strcpy(devname, dev_name(ubus->controller->parent));
			if(!strcmp(devname, usb1)){
				if (event == USB_BUS_ADD){
					usb1_active = 1;
				}
				else if (event == USB_BUS_REMOVE){
					usb1_active = 0;
				}
			}

			if(!strcmp(devname, usb2)){
				if (event == USB_BUS_ADD){
					usb2_active = 1;
				}
				else if (event == USB_BUS_REMOVE){
					usb2_active = 0;
				}
			}
		}
	}
	CHG_DBG("%s.  usb1_active = %d, usb2_active = %d, event = %lu\n", __func__, usb1_active, usb2_active, event);

	//[+++]Update the status of thermal alert
	asus_update_thermal_result();
	//[---]Update the status of thermal alert
	return NOTIFY_DONE;
}

#define CHECK_MINIMUM_INTERVAL (30)
#define REPORT_CAPACITY_POLLING_TIME (180)
int asus_chg_resume(struct device *dev)
{
	struct timespec64  mtNow;
	int nextCheckInterval;

	pm_stay_awake(g_bcdev->dev);

	ktime_get_coarse_real_ts64(&mtNow);

	if (mtNow.tv_sec - last_check_time_3m.tv_sec >= REPORT_CAPACITY_POLLING_TIME) {
		cancel_delayed_work(&asus_update_batt_status_work);
		schedule_delayed_work(&asus_update_batt_status_work, 0);
	}

	if (g_vbus_plug) {
		/* If next check time less than 30s, do check (next check time = last check time + 60s) */
		nextCheckInterval = 60 - (mtNow.tv_sec - last_check_time_1m.tv_sec);
		CHG_DBG("%s: nextCheckInterval = %d\n", __func__, nextCheckInterval);
		if (nextCheckInterval <= CHECK_MINIMUM_INTERVAL) {
			cancel_delayed_work(&asus_min_check_work);
			schedule_delayed_work(&asus_min_check_work, 0);
		} else {
			cancel_delayed_work(&asus_min_check_work);
			schedule_delayed_work(&asus_min_check_work, msecs_to_jiffies(nextCheckInterval));
			pm_relax(g_bcdev->dev);
		}
		wake_lock_timeout(&cable_resume_wake_lock, msecs_to_jiffies(1500));
	} else {
		pm_relax(g_bcdev->dev);
	}
	return 0;
}

//ASUS BSP : Show "+" on charging icon +++
void asus_set_qc_state_worker(struct work_struct *work)
{
	asus_extcon_set_state_sync(quickchg_extcon, g_SWITCH_LEVEL);
}

void set_qc_stat(int status)
{
	if (quickchg_extcon == NULL) {
		CHG_DBG("%s: quickchg_extcon not Ready\n", __func__);
		return;
	}

	if (!g_vbus_plug) {
		cancel_delayed_work(&asus_set_qc_state_work);
		asus_extcon_set_state_sync(quickchg_extcon, SWITCH_LEVEL0_DEFAULT);
		CHG_DBG("%s: status: %d, switch: %d\n", __func__, status, SWITCH_LEVEL0_DEFAULT);
		return;
	}

	switch (status) {
	//"qc" stat happends in charger mode only, refer to smblib_get_prop_batt_status
	case POWER_SUPPLY_STATUS_CHARGING:
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
	case POWER_SUPPLY_STATUS_FULL:
		CHG_DBG("%s: status: %d, switch: %d\n", __func__, status, g_SWITCH_LEVEL);
		cancel_delayed_work(&asus_set_qc_state_work);
		//if (g_Charger_mode)
			schedule_delayed_work(&asus_set_qc_state_work, 0);
		//else
		//schedule_delayed_work(&asus_set_qc_state_work, msecs_to_jiffies(3000));
		break;
	default:
		break;
	}
}
//ASUS BSP : Show "+" on charging icon ---

static int print_battery_status(void)
{
	union power_supply_propval prop = {};
	int rc = 0;
	char battInfo[256];
	int bat_cap = 0, bat_fcc = 0, bat_vol = 0, bat_cur = 0, bat_temp = 0, chg_sts = 0, bat_health = 0, charge_counter = 0;
	char UTSInfo[256]; //ASUS_BSP add to printk the WIFI hotspot & QXDM UTS event

	get_oem_batt_temp_from_ADSP();
	get_oem_cell_volt_from_ADSP();

	rc = power_supply_get_property(qti_phy_bat, POWER_SUPPLY_PROP_CAPACITY, &prop);
	if (rc < 0)
		CHG_DBG_E("%s: Failed to get POWER_SUPPLY_PROP_CAPACITY, rc=%d\n", __func__, rc);
	else
		bat_cap = prop.intval;

	rc = power_supply_get_property(qti_phy_bat, POWER_SUPPLY_PROP_CHARGE_FULL, &prop);
	if (rc < 0)
		CHG_DBG_E("%s: Failed to get POWER_SUPPLY_PROP_CHARGE_FULL, rc=%d\n", __func__, rc);
	else
		bat_fcc = prop.intval;

	rc = power_supply_get_property(qti_phy_bat, POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
	if (rc < 0)
		CHG_DBG_E("%s: Failed to get POWER_SUPPLY_PROP_VOLTAGE_NOW, rc=%d\n", __func__, rc);
	else
		bat_vol = prop.intval;

	rc = power_supply_get_property(qti_phy_bat, POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
	if (rc < 0)
		CHG_DBG_E("%s: Failed to get POWER_SUPPLY_PROP_CURRENT_NOW, rc=%d\n", __func__, rc);
	else
		bat_cur = prop.intval;

	rc = power_supply_get_property(qti_phy_bat, POWER_SUPPLY_PROP_TEMP, &prop);
	if (rc < 0)
		CHG_DBG_E("%s: Failed to get POWER_SUPPLY_PROP_TEMP, rc=%d\n", __func__, rc);
	else
		bat_temp = prop.intval;

	rc = power_supply_get_property(qti_phy_bat, POWER_SUPPLY_PROP_STATUS, &prop);
	if (rc < 0)
		CHG_DBG_E("%s: Failed to get POWER_SUPPLY_PROP_STATUS, rc=%d\n", __func__, rc);
	else
		chg_sts = prop.intval;

	rc = power_supply_get_property(qti_phy_bat, POWER_SUPPLY_PROP_HEALTH, &prop);
	if (rc < 0)
		CHG_DBG_E("%s: Failed to get POWER_SUPPLY_PROP_HEALTH, rc=%d\n", __func__, rc);
	else
		bat_health = prop.intval;

	rc = power_supply_get_property(qti_phy_bat, POWER_SUPPLY_PROP_CHARGE_COUNTER, &prop);
	if (rc < 0)
		CHG_DBG_E("%s: Failed to get POWER_SUPPLY_PROP_CHARGE_COUNTER, rc=%d\n", __func__, rc);
	else
		charge_counter = prop.intval;

	snprintf(battInfo, sizeof(battInfo), "report Capacity ==>%d, FCC:%dmAh, RM:%dmAh, BMS:%d, V:%dmV, Vcell1:%dmV, Vcell2_V:%dmV, Cur:%dmA, ",
		bat_cap,
		bat_fcc/1000,
		charge_counter/1000,
		bat_cap,
		bat_vol/1000,
		ChgPD_Info.cell1_voltage,
		ChgPD_Info.cell2_voltage,
		bat_cur/1000);
	snprintf(battInfo, sizeof(battInfo), "%sgaugeTemp:%d.%dC, pmicTemp:%dC, BATID:%d, CHG_Status:%d(%s), QC_Extcon:%d(%s), BAT_HEALTH:%s\n",
		battInfo,
		bat_temp/10,
		bat_temp%10,
		ChgPD_Info.batt_temp,
		ChgPD_Info.BATT_ID,
		chg_sts,
		charging_stats[chg_sts],
		g_SWITCH_LEVEL,
		qc_extcon_type[g_SWITCH_LEVEL],
		health_type[bat_health]);

	ASUSEvtlog("[BAT][Ser]%s", battInfo);
	//ASUS_BSP +++ add to printk the WIFI hotspot & QXDM UTS event
	snprintf(UTSInfo, sizeof(UTSInfo), "WIFI_HS=%d, QXDM=%d", g_wifi_hs_en, g_qxdm_en);
	ASUSEvtlog("[UTS][Status]%s", UTSInfo);
	//ASUS_BSP --- add to printk the WIFI hotspot & QXDM UTS event

	ktime_get_coarse_real_ts64(&last_check_time_3m);

	return 0;
}

void  asus_update_batt_status_worker(struct work_struct *work)
{
	int ret = 0;

	ret = print_battery_status();
	if (ret < 0)
		CHG_DBG_E("%s: print_battery_status fail\n", __func__);

	if (!g_vbus_plug) {
		#if 0
		ret = asus_thermal_side();
		if (ret < 0)
			CHG_DBG_E("%s: asus_thermal_side fail\n", __func__);

		ret = asus_thermal_btm();
		if (ret < 0)
			CHG_DBG_E("%s: asus_thermal_side fail\n", __func__);

		if (usb_alert_side_flag | usb_alert_btm_flag) {
			CHG_DBG("%s: Set thermal_extcon 1\n", __func__);
			asus_extcon_set_state_sync(thermal_extcon, 1);
		} else {
			CHG_DBG("%s: Set thermal_extcon 0\n", __func__);
			asus_extcon_set_state_sync(thermal_extcon, 0);
		}
		#endif
		asus_update_thermal_result();
	}

	schedule_delayed_work(&asus_update_batt_status_work, msecs_to_jiffies(180000));
}
//[---]Add for ASUSEvtlog print battery status regularly

void get_vbus_status_from_ADSP(void)
{
	struct oem_get_VBUS_SRC_req req_msg = {};
	int rc = 0;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_VBUS_SOURCE;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get VBUS_SOURCE rc=%d\n", rc);
		return ;
	}
}

void usbin_suspend_to_ADSP(int enable)
{
	struct oem_usbin_suspend_en_req req_msg = { { 0 } };
	int rc = 0;
	static int pre_enable = 0;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_SET_USBIN_SUSPNED;
	req_msg.enable = enable;

	CHG_DBG("%s. enable : %d, pre_enable : %d", __func__, req_msg.enable, pre_enable);
	if (req_msg.enable != pre_enable) {
		rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
		if (rc < 0) {
			pr_err("Failed to set USBIN_SUSPEND_EN rc=%d\n", rc);
			return;
		}
		pre_enable = req_msg.enable;
	}
}

void get_oem_batt_temp_from_ADSP(void)
{
	struct oem_get_batt_temp_req req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_Batt_temp;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get FW_version rc=%d\n", rc);
		return;
	}
}

void get_oem_cell_volt_from_ADSP(void)
{
	struct oem_get_cell_voltage_req req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_Cell_Voltage_REQ;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get battery cell voltage rc=%d\n", rc);
		return;
	}
}

/* +++ Add Maximun Battery Lifespan +++ */
#define CHG_LIMIT_PATH	"/asdf/CHGLimit"
void write_CHGLimit_value(int input)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8] = "";

	sprintf(buf, "%d", input);

	fp = filp_open(CHG_LIMIT_PATH, O_RDWR | O_CREAT | O_SYNC, 0666);
	if (IS_ERR_OR_NULL(fp)) {
		CHG_DBG_E("%s: open (%s) fail\n", __func__, CHG_LIMIT_PATH);
		return;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp, buf, 8, &pos_lsts);

	set_fs(old_fs);
	filp_close(fp, NULL);

	CHG_DBG("%s : %s\n", __func__, buf);
}
/* --- Add Maximun Battery Lifespan --- */

//[+++] Add the interface for audio driver request to set PMIC LCM status
int audio_req_set_lcm_mode(bool enable) {
	int rc = 0;

	CHG_DBG("%s. enable : %d\n", __func__, enable);
	rc = asus_set_charger_limit_mode(AUDIO_REQ_SET_LCM_MODE, enable);
	return rc;
}
EXPORT_SYMBOL(audio_req_set_lcm_mode);
//[---] Add the interface for audio driver request to set PMIC LCM status

void get_cc_status_from_ADSP(void)
{
	struct oem_get_batt_temp_req req_msg = {};
	int rc;

	req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = OEM_GET_SIDEPORT_CC_STATUS_REQ;

	rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get CC_status rc=%d\n", rc);
		return;
	}
}

int asuslib_init(void) {
	int rc = 0;
	struct pmic_glink_client_data client_data = { };
	struct pmic_glink_client	*client;

	printk(KERN_ERR "%s +++\n", __func__);
	// Initialize the necessary power supply
	rc = asus_init_power_supply_prop();
	if (rc < 0) {
		pr_err("Failed to init power_supply chains\n");
		return rc;
	}

	// Register the class node
	rc = class_register(&asuslib_class);
	if (rc) {
		pr_err("%s: Failed to register asuslib class\n", __func__);
		return -1;
	}

	//Register GPIO for PMI_MUX
	PMI_MUX_GPIO = of_get_named_gpio(g_bcdev->dev->of_node, "PMI_MUX_EN", 0);
	rc = gpio_request(PMI_MUX_GPIO, "PMI_MUX_EN");
	if (rc) {
		pr_err("%s: Failed to initalize the PMI_MUX_EN\n", __func__);
		return -1;
	}
	POGO_OTG_GPIO = of_get_named_gpio(g_bcdev->dev->of_node, "POGO_OTG_EN", 0);
	rc = gpio_request(POGO_OTG_GPIO, "POGO_OTG_EN");
	if (rc) {
		pr_err("%s: Failed to initalize the POGO_OTG_EN\n", __func__);
		return -1;
	}

	alarm_init(&bat_alarm, ALARM_REALTIME, batAlarm_handler);
	INIT_DELAYED_WORK(&asus_min_check_work, asus_min_check_worker);
	INIT_DELAYED_WORK(&asus_18W_workaround_work, asus_18W_workaround_worker);

#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
	msm_dwc3_register_switch(&dwc3_role_switch);
#endif
	//[+++] Init the PMIC-GLINK
	client_data.id = PMIC_GLINK_MSG_OWNER_OEM;
	client_data.name = "asus_BC";
	client_data.msg_cb = asusBC_msg_cb;
	client_data.priv = g_bcdev;
	client_data.state_cb = asusBC_state_cb;
	client = pmic_glink_register_client(g_bcdev->dev, &client_data);
	if (IS_ERR(client)) {
		rc = PTR_ERR(client);
		if (rc != -EPROBE_DEFER)
			dev_err(g_bcdev->dev, "Error in registering with pmic_glink %d\n",
				rc);
		return rc;
	}
	//[---] Init the PMIC-GLINK

	//[+++] Init the info structure of ChargerPD from ADSP
	ChgPD_Info.PlatformID = 0;
	ChgPD_Info.BATT_ID = 0;
	ChgPD_Info.VBUS_SRC = 0;
	ChgPD_Info.chg_limit_en = 0;
	ChgPD_Info.chg_limit_cap = 0;
	ChgPD_Info.usbin_suspend_en = 0;
    ChgPD_Info.charging_suspend_en = 0;
	strcpy(ChgPD_Info.firmware_version , st_battery_name);
	//[---] Init the info structure of ChargerPD from ADSP

	get_vbus_status_from_ADSP();

	bat_extcon = extcon_dev_allocate(asus_fg_extcon_cable);
	if (IS_ERR(bat_id_extcon)) {
		rc = PTR_ERR(bat_extcon);
	}
	bat_extcon->fnode_name = "battery";
	bat_extcon->name = ChgPD_Info.firmware_version;

	rc = extcon_dev_register(bat_extcon);
	if (rc < 0)
		printk(KERN_ERR "[BAT][CHG] failed to register bat_extcon device rc=%d\n", rc);

	bat_id_extcon = extcon_dev_allocate(asus_fg_extcon_cable);
	if (IS_ERR(bat_id_extcon)) {
		rc = PTR_ERR(bat_id_extcon);
	}
	bat_id_extcon->fnode_name = "battery_id";

	rc = extcon_dev_register(bat_id_extcon);
	if (rc < 0)
		printk(KERN_ERR "[BAT][CHG] failed to register bat_id_extcon device rc=%d\n", rc);

	//[+++]Register the extcon for quick_charger
	quickchg_extcon = extcon_dev_allocate(asus_fg_extcon_cable);
	if (IS_ERR(quickchg_extcon)) {
		rc = PTR_ERR(quickchg_extcon);
		printk(KERN_ERR "[BAT][CHG] failed to allocate ASUS quickchg extcon device rc=%d\n", rc);
	}
	#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
	quickchg_extcon->fnode_name = "quick_charging";
	#endif
	rc = extcon_dev_register(quickchg_extcon);
	if (rc < 0)
		printk(KERN_ERR "[BAT][CHG] failed to register ASUS quickchg extcon device rc=%d\n", rc);
		
	INIT_DELAYED_WORK(&asus_set_qc_state_work, asus_set_qc_state_worker);
	//[---]Register the extcon for quick_charger

	//[+++]Register the extcon for thermal alert
	thermal_extcon = extcon_dev_allocate(asus_fg_extcon_cable);
	if (IS_ERR(thermal_extcon)) {
		rc = PTR_ERR(thermal_extcon);
		printk(KERN_ERR "[BAT][CHG] failed to allocate ASUS thermal alert extcon device rc=%d\n", rc);
	}
	#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
	thermal_extcon->fnode_name = "usb_connector";
	#endif	
	rc = extcon_dev_register(thermal_extcon);
	if (rc < 0)
		printk(KERN_ERR "[BAT][CHG] failed to register ASUS thermal alert extcon device rc=%d\n", rc);
	//[---]Register the extcon for thermal alert

	//[+++]Register the extcon for invalid audio donlge
	audio_dongle_extcon = extcon_dev_allocate(asus_fg_extcon_cable);
	if (IS_ERR(audio_dongle_extcon)) {
		rc = PTR_ERR(audio_dongle_extcon);
		printk(KERN_ERR "[BAT][CHG] failed to allocate audio dongle extcon device rc=%d\n", rc);
	}
	audio_dongle_extcon->fnode_name = "invalid_dongle";

	rc = extcon_dev_register(audio_dongle_extcon);
	if (rc < 0)
		printk(KERN_ERR "[BAT][CHG] failed to register audio dongle extcon device rc=%d\n", rc);
	//[---]Register the extcon for thermal alert

	asus_get_Batt_ID();

	//[+++] Init usb temp vadc channel

	side_usb_temp_vadc_chan = iio_channel_get(g_bcdev->dev, "pm8350b_amux_thm6");
	if (IS_ERR_OR_NULL(side_usb_temp_vadc_chan)) {
		CHG_DBG_E("%s: iio_channel_get fail\n", __func__);
	}
	btm_usb_temp_vadc_chan = iio_channel_get(g_bcdev->dev, "pm8350_amux_thm2");
	if (IS_ERR_OR_NULL(btm_usb_temp_vadc_chan)) {
		CHG_DBG_E("%s: iio_channel_get fail\n", __func__);
	}

	#if 0 // Use Glink to notify
	//register USB_Online notify
	charge_notify.notifier_call = charge_notifier_callback;
	qti_charge_register_notify(&charge_notify);
	#endif
	//[---] Init usb temp vadc channel

	//[+++] Register the notifier to receive the host change
	host_notify.notifier_call = host_notifier_callback;
	usb_register_notify(&host_notify);
	//[---] Register the notifier to receive the host change

    //PASS_HWID_TO_ADSP();

	wake_lock_init(&cable_resume_wake_lock, g_bcdev->dev, "cable_resume_wake_lock");
	if (g_Charger_mode) {
		//In post-cs10, something blocks the device in the beginning.
		//and it will be late to detect cable-out, so make a wakelock to avoid this duration.
		wake_lock_init(&cos_wa_wake_lock, g_bcdev->dev, "cos_wa_wake_lock");
		wake_lock_timeout(&cos_wa_wake_lock, msecs_to_jiffies(90000));
	}
	//register drm notifier
	INIT_DELAYED_WORK(&asus_set_panelonoff_current_work, asus_set_panelonoff_current_worker);
	RegisterDRMCallback();

	INIT_DELAYED_WORK(&asus_update_batt_status_work, asus_update_batt_status_worker);
	schedule_delayed_work(&asus_update_batt_status_work, msecs_to_jiffies(5000));

	INIT_DELAYED_WORK(&asus_set_ASUS_media_work, asus_set_ASUS_media_worker);
	schedule_delayed_work(&asus_set_ASUS_media_work, 0);

	create_uts_status_proc_file(); //ASUS_BSP LiJen add to printk the WIFI hotspot & QXDM UTS event

	CHG_DBG("%s:g_vbus_plug=%d\n", __func__, g_vbus_plug);
	if (g_vbus_plug) {
		ASUSEvtlog("[BAT][Ser]Cable Plug-in booting");
		schedule_delayed_work(&asus_min_check_work, 0);
	}

	if (g_Charger_mode)
		ASUSEvtlog("[BAT][CHG] Enter COS");
	else
		ASUSEvtlog("[BAT][CHG] Enter MOS");

	CHG_DBG_E("Load the asuslib_init Succesfully\n");
	return rc;
}

int asuslib_deinit(void) {
	//int rc;

	wake_lock_destroy(&cable_resume_wake_lock);
	if (g_Charger_mode)
		wake_lock_destroy(&cos_wa_wake_lock);
	class_unregister(&asuslib_class);
	return 0;
}
