/*
 * Copyright (c) 2019-2020, The ASUS Company. All rights reserved.
 */

struct extcon_dev   *bat_id_extcon;
struct extcon_dev   *bat_extcon;
struct extcon_dev   *quickchg_extcon;
extern int asus_extcon_set_state_sync(struct extcon_dev *edev, int cable_state);

//qti_battery_charger+++
#define MSG_OWNER_BC                32778

struct battery_charger_req_msg {
    struct pmic_glink_hdr   hdr;
    u32         battery_id;
    u32         property_id;
    u32         value;
};
//qti_battery_charger---

//Define the OWNER ID
#define PMIC_GLINK_MSG_OWNER_OEM    32782

//Define Message Type
#define MSG_TYPE_REQ_RESP   1
#define MSG_TYPE_NOTIFY   2

//Define the opcode
#define OEM_OPCODE_READ_BUFFER               0x10000
#define OEM_OPCODE_WRITE_BUFFER              0x10001

#define OEM_ASUS_EVTLOG_IND                  0x1002
#define OEM_PD_EVTLOG_IND                    0x1003
#define OEM_SET_OTG_WA                       0x2107
#define OEM_SET_CHARGER_TYPE_CHANGE          0x2109


#define MAX_OEM_PROPERTY_DATA_SIZE           16

//Add the structure +++
struct ADSP_ChargerPD_Info {
    int PlatformID;
    int BATT_ID;
    bool   chg_limit_en;
    u32    chg_limit_cap;
    bool   usbin_suspend_en;
    bool   charging_suspend_en;
    char   ChgPD_FW[64];
    int firmware_version;
    int batt_temp;
};

struct battman_oem_read_buffer_req_msg { 
    struct pmic_glink_hdr   hdr;
    u32 oem_property_id;
    u32 data_size;
};

struct battman_oem_read_buffer_resp_msg { 
    struct pmic_glink_hdr   hdr;
    u32 oem_property_id;
    u32 data_buffer[MAX_OEM_PROPERTY_DATA_SIZE];
    u32 data_size; //size = 0 if failed, otherwise should be data_size.
};

struct battman_oem_write_buffer_req_msg {
    struct pmic_glink_hdr   hdr;
    u32 oem_property_id;
    u32 data_buffer[MAX_OEM_PROPERTY_DATA_SIZE];
    u32 data_size;
};

struct battman_oem_write_buffer_resp_msg {
    struct pmic_glink_hdr   hdr;
    u32 oem_property_id;
    u32 return_status;
};

struct evtlog_context_resp_msg3 {
    struct pmic_glink_hdr       hdr;
    u8              buf[128];
    u32             reserved;
};

struct oem_set_OTG_WA_resp {
    struct pmic_glink_hdr   hdr;
    u32    enable;
};

struct oem_set_Charger_Type_resp {
    struct pmic_glink_hdr   hdr;
    u32    charger_type;
};
//Add the structure ---

//Add oem property
enum battman_oem_property {
    BATTMAN_OEM_ADSP_PLATFORM_ID,
    BATTMAN_OEM_BATT_ID,
    BATTMAN_OEM_CHG_LIMIT_EN,
    BATTMAN_OEM_CHG_LIMIT_CAP,
    BATTMAN_OEM_USBIN_SUSPEND,
    BATTMAN_OEM_CHARGING_SUSPNED,
    BATTMAN_OEM_CHGPD_FW_VER,
    BATTMAN_OEM_FW_VERSION,
    BATTMAN_OEM_BATT_TEMP,
    BATTMAN_OEM_PROPERTY_MAX,
};


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

#define SWITCH_LEVEL3_NOT_QUICK_CHARGING    6 //EQual to SWITCH_NXP_NOT_QUICK_CHARGING
#define SWITCH_LEVEL3_QUICK_CHARGING        5 //EQual to SWITCH_NXP_QUICK_CHARGING
#define SWITCH_LEVEL1_NOT_QUICK_CHARGING    4 //EQual to SWITCH_QC_NOT_QUICK_CHARGING
#define SWITCH_LEVEL1_QUICK_CHARGING        3 //EQual to SWITCH_QC_QUICK_CHARGING
#define SWITCH_LEVEL2_NOT_QUICK_CHARGING    2 //EQual to SWITCH_QC_NOT_QUICK_CHARGING_PLUS
#define SWITCH_LEVEL2_QUICK_CHARGING        1 //EQual to SWITCH_QC_QUICK_CHARGING_PLUS
#define SWITCH_LEVEL0_DEFAULT               0 //EQual to SWITCH_QC_OTHER

extern struct battery_chg_dev *g_bcdev;
struct power_supply *qti_phy_usb;
struct power_supply *qti_phy_bat;
int POGO_OTG_GPIO;
struct ADSP_ChargerPD_Info ChgPD_Info;
char st_battery_name[64] = "C21P2001-T-03-0001-0.17";
//[---] Add the global variables

