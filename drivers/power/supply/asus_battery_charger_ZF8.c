/*
 * Copyright (c) 2019-2020, The ASUS Company. All rights reserved.
 */

#define pr_fmt(fmt) "BATTERY_CHG: %s: " fmt, __func__

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/soc/qcom/battery_charger.h>

#include <linux/of_gpio.h>
//ASUS BSP +++
#include "fg-core.h"
#include "asus_battery_charger_ZF8.h"
//ASUS BSP ---


//[+++] Add the external function
extern int battery_chg_write(struct battery_chg_dev *bcdev, void *data, int len);
//[---] Add the external function

static const char * const power_supply_usb_type_text[] = {
    "Unknown", "SDP", "DCP", "CDP", "ACA", "C",
    "PD", "PD_DRP", "PD_PPS", "BrickID"
};

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

static ssize_t oem_prop_read(enum battman_oem_property prop, size_t count)
{
    struct battman_oem_read_buffer_req_msg req_msg = { { 0 } };
    int rc;

    req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
    req_msg.hdr.type = MSG_TYPE_REQ_RESP;
    req_msg.hdr.opcode = OEM_OPCODE_READ_BUFFER;
    req_msg.oem_property_id = prop;
    req_msg.data_size = count;

    rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
    if (rc < 0) {
        pr_err("Failed to read buffer rc=%d\n", rc);
        return rc;
    }

    return count;
}

static ssize_t oem_prop_write(enum battman_oem_property prop,
                    u32 *buf, size_t count)
{
    struct battman_oem_write_buffer_req_msg req_msg = { { 0 } };
    int rc;

    req_msg.hdr.owner = PMIC_GLINK_MSG_OWNER_OEM;
    req_msg.hdr.type = MSG_TYPE_REQ_RESP;
    req_msg.hdr.opcode = OEM_OPCODE_WRITE_BUFFER;
    req_msg.oem_property_id = prop;
    memcpy(req_msg.data_buffer, buf, sizeof(u32)*count);
    req_msg.data_size = count;

    if (g_bcdev == NULL) {
        pr_err("g_bcdev is null\n");
        return -1;
    }
    rc = battery_chg_write(g_bcdev, &req_msg, sizeof(req_msg));
    if (rc < 0) {
        pr_err("Failed to write buffer rc=%d\n", rc);
        return rc;
    }

    return count;
}

int asus_get_Batt_ID(void)
{
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_BATT_ID, 1);
    if (rc < 0) {
        pr_err("Failed to get BattID rc=%d\n", rc);
        return rc;
    }
    return 0;
}

//[+++] Addd the interface for accessing the BATTERY power supply
static ssize_t asus_get_FG_SoC_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    union power_supply_propval prop = {};
    int bat_cap, rc = 0;

    rc = power_supply_get_property(qti_phy_bat,
        POWER_SUPPLY_PROP_CAPACITY, &prop);
    if (rc < 0) {
        pr_err("Failed to get battery SOC, rc=%d\n", rc);
        return rc;
    }
    bat_cap = prop.intval;
    printk(KERN_ERR "%s. BAT_SOC : %d", __func__, bat_cap);

    return scnprintf(buf, PAGE_SIZE, "%d\n", bat_cap);
}
static CLASS_ATTR_RO(asus_get_FG_SoC);
//[---] Addd the interface for accessing the BATTERY power supply

//[+++] Add the interface for accesing the inforamtion of ChargerPD on ADSP
static ssize_t asus_get_PlatformID_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_ADSP_PLATFORM_ID, 1);
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
    int rc;

    rc = asus_get_Batt_ID();
    if (rc < 0) {
        pr_err("Failed to get BattID rc=%d\n", rc);
        return rc;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.BATT_ID);
}
static CLASS_ATTR_RO(asus_get_BattID);

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

static ssize_t charger_limit_en_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
    int rc;
    u32 tmp;

    tmp = simple_strtol(buf, NULL, 10);

    CHG_DBG("%s. enable : %d", __func__, tmp);
    rc = oem_prop_write(BATTMAN_OEM_CHG_LIMIT_EN, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set CHG_LIMIT_EN rc=%d\n", rc);
        return rc;
    }

    return count;
}

static ssize_t charger_limit_en_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_CHG_LIMIT_EN, 1);
    if (rc < 0) {
        pr_err("Failed to get CHG_LIMIT_EN rc=%d\n", rc);
        return rc;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.chg_limit_en);
}
static CLASS_ATTR_RW(charger_limit_en);

static ssize_t charger_limit_cap_store(struct class *c,
                    struct class_attribute *attr,
                    const char *buf, size_t count)
{
    int rc;
    u32 tmp;

    tmp = simple_strtol(buf, NULL, 10);

    CHG_DBG("%s. cap : %d", __func__, tmp);
    rc = oem_prop_write(BATTMAN_OEM_CHG_LIMIT_CAP, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set CHG_LIMIT_CAP rc=%d\n", rc);
        return rc;
    }

    return count;
}

static ssize_t charger_limit_cap_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_CHG_LIMIT_CAP, 1);
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
    int rc;
    u32 tmp;
    tmp = simple_strtol(buf, NULL, 10);

    rc = oem_prop_write(BATTMAN_OEM_USBIN_SUSPEND, &tmp, 1);

    pr_err("%s. enable : %d", __func__, tmp);
    if (rc < 0) {
        pr_err("Failed to set USBIN_SUSPEND_EN rc=%d\n", rc);
        return rc;
    }

    return count;
}

static ssize_t usbin_suspend_en_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_USBIN_SUSPEND, 1);
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
    int rc;
    u32 tmp;
    tmp = simple_strtol(buf, NULL, 10);

    CHG_DBG("%s. enable : %d", __func__, tmp);
    rc = oem_prop_write(BATTMAN_OEM_CHARGING_SUSPNED, &tmp, 1);
    if (rc < 0) {
        pr_err("Failed to set CHARGING_SUSPEND_EN rc=%d\n", rc);
        return rc;
    }

    return count;
}

static ssize_t charging_suspend_en_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_CHARGING_SUSPNED, 1);
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
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_CHGPD_FW_VER, 32);
    if (rc < 0) {
        pr_err("%s. Failed to get ChgPD_FW_Ver rc=%d\n", __func__, rc);
        return rc;
    }

    return scnprintf(buf, PAGE_SIZE, "%s\n", ChgPD_Info.ChgPD_FW);
}
static CLASS_ATTR_RO(get_ChgPD_FW_Ver);

static ssize_t asus_get_fw_version_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_FW_VERSION, 1);
    if (rc < 0) {
        pr_err("Failed to get FW_version rc=%d\n", rc);
        return rc;
    }

    return scnprintf(buf, PAGE_SIZE, "0x00%x\n", ChgPD_Info.firmware_version);
}
static CLASS_ATTR_RO(asus_get_fw_version);

static ssize_t asus_get_batt_temp_show(struct class *c,
                    struct class_attribute *attr, char *buf)
{
    int rc;

    rc = oem_prop_read(BATTMAN_OEM_BATT_TEMP, 1);
    if (rc < 0) {
        pr_err("Failed to get FW_version rc=%d\n", rc);
        return rc;
    }

    return scnprintf(buf, PAGE_SIZE, "%d\n", ChgPD_Info.batt_temp);
}
static CLASS_ATTR_RO(asus_get_batt_temp);

static struct attribute *asuslib_class_attrs[] = {
    &class_attr_asus_get_FG_SoC.attr,
    &class_attr_asus_get_PlatformID.attr,
    &class_attr_asus_get_BattID.attr,
    &class_attr_get_usb_type.attr,
    &class_attr_charger_limit_en.attr,
    &class_attr_charger_limit_cap.attr,
    &class_attr_usbin_suspend_en.attr,
    &class_attr_charging_suspend_en.attr,
    &class_attr_get_ChgPD_FW_Ver.attr,
    &class_attr_asus_get_fw_version.attr,
    &class_attr_asus_get_batt_temp.attr,
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

static void handle_notification(struct battery_chg_dev *bcdev, void *data,
                size_t len)
{
    struct evtlog_context_resp_msg3 *evtlog_msg;
    struct oem_set_OTG_WA_resp *OTG_WA_msg;
    struct oem_set_Charger_Type_resp *Update_charger_type_msg;
    struct pmic_glink_hdr *hdr = data;
    int rc;
    static int pre_chg_type = 0;

    switch(hdr->opcode) {
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
    case OEM_SET_OTG_WA:
        if (len == sizeof(*OTG_WA_msg)) {
            OTG_WA_msg = data;
            CHG_DBG("%s OEM_SET_OTG_WA. enable : %d, HWID : %d\n", __func__, OTG_WA_msg->enable, g_ASUS_hwID);
            if(g_ASUS_hwID <= HW_REV_EVB2) {
                if (gpio_is_valid(POGO_OTG_GPIO)) {
                    rc = gpio_direction_output(POGO_OTG_GPIO, OTG_WA_msg->enable);
                    if (rc)
                        pr_err("%s. Failed to control POGO_OTG_EN\n", __func__);
                } else {
                    CHG_DBG_E("%s. POGO_OTG_GPIO is invalid\n", __func__);
                }
            }
        } else {
            pr_err("Incorrect response length %zu for OEM_SET_OTG_WA\n",
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
                    asus_extcon_set_state_sync(quickchg_extcon, SWITCH_LEVEL0_DEFAULT);
                break;
                case ASUS_CHARGER_TYPE_LEVEL1:
                    asus_extcon_set_state_sync(quickchg_extcon, SWITCH_LEVEL1_QUICK_CHARGING);
                break;
                case ASUS_CHARGER_TYPE_LEVEL2:
                    asus_extcon_set_state_sync(quickchg_extcon, SWITCH_LEVEL2_QUICK_CHARGING);
                break;
                case ASUS_CHARGER_TYPE_LEVEL3:
                    asus_extcon_set_state_sync(quickchg_extcon, SWITCH_LEVEL3_QUICK_CHARGING);
                break;
                default:
                    asus_extcon_set_state_sync(quickchg_extcon, SWITCH_LEVEL0_DEFAULT);
                break;
                }
            }
            pre_chg_type = Update_charger_type_msg->charger_type;
        } else {
            pr_err("Incorrect response length %zu for OEM_SET_CHARGER_TYPE_CHANGE\n",
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
    struct battman_oem_read_buffer_resp_msg *oem_read_buffer_resp_msg;
    struct battman_oem_write_buffer_resp_msg *oem_write_buffer_resp_msg;

    struct pmic_glink_hdr *hdr = data;
    bool ack_set = false;
    
    switch (hdr->opcode) {
    case OEM_OPCODE_READ_BUFFER:
        if (len == sizeof(*oem_read_buffer_resp_msg)) {
            oem_read_buffer_resp_msg = data;
            switch (oem_read_buffer_resp_msg->oem_property_id) {
            case BATTMAN_OEM_ADSP_PLATFORM_ID:
                ChgPD_Info.PlatformID = oem_read_buffer_resp_msg->data_buffer[0];
                ack_set = true;
                break;
            case BATTMAN_OEM_BATT_ID:
                ChgPD_Info.BATT_ID = oem_read_buffer_resp_msg->data_buffer[0];
                ack_set = true;
                if(ChgPD_Info.BATT_ID < 51000*1.15 && ChgPD_Info.BATT_ID > 51000*0.85)
                    asus_extcon_set_state_sync(bat_id_extcon, 1);
                else if(ChgPD_Info.BATT_ID < 100000*1.15 && ChgPD_Info.BATT_ID > 100000*0.85)
                    asus_extcon_set_state_sync(bat_id_extcon, 1);
                else
                    asus_extcon_set_state_sync(bat_id_extcon, 0);
                break;
            case BATTMAN_OEM_CHG_LIMIT_EN:
                CHG_DBG("%s BATTMAN_OEM_CHG_LIMIT_EN successfully\n", __func__);
                ChgPD_Info.chg_limit_en = oem_read_buffer_resp_msg->data_buffer[0];
                ack_set = true;
                break;
            case BATTMAN_OEM_CHG_LIMIT_CAP:
                CHG_DBG("%s BATTMAN_OEM_CHG_LIMIT_CAP successfully\n", __func__);
                ChgPD_Info.chg_limit_cap = oem_read_buffer_resp_msg->data_buffer[0];
                ack_set = true;
                break;
            case BATTMAN_OEM_USBIN_SUSPEND:
                ChgPD_Info.usbin_suspend_en = oem_read_buffer_resp_msg->data_buffer[0];
                ack_set = true;
                break;
            case BATTMAN_OEM_CHARGING_SUSPNED:
                CHG_DBG("%s BATTMAN_OEM_CHARGING_SUSPNED successfully\n", __func__);
                ChgPD_Info.charging_suspend_en = oem_read_buffer_resp_msg->data_buffer[0];
                ack_set = true;
                break;
            case BATTMAN_OEM_CHGPD_FW_VER:
                CHG_DBG("%s. ChgPD_FW : %s\n", __func__, (char*)oem_read_buffer_resp_msg->data_buffer);
                strcpy(ChgPD_Info.ChgPD_FW, (char*)oem_read_buffer_resp_msg->data_buffer);
                ack_set = true;
                break;
            case BATTMAN_OEM_FW_VERSION:
                ChgPD_Info.firmware_version = oem_read_buffer_resp_msg->data_buffer[0];
                ack_set = true;
                break;
            case BATTMAN_OEM_BATT_TEMP:
                ChgPD_Info.batt_temp = oem_read_buffer_resp_msg->data_buffer[0];
                ack_set = true;
                break;
            default:
                ack_set = true;
                pr_err("Unknown property_id: %u\n", oem_read_buffer_resp_msg->oem_property_id);
            }
        } else {
            pr_err("Incorrect response length %zu for OEM_OPCODE_READ_BUFFER\n", len);
        }
        break;
    case OEM_OPCODE_WRITE_BUFFER:
        if (len == sizeof(*oem_write_buffer_resp_msg)) {
            oem_write_buffer_resp_msg = data;
            switch (oem_write_buffer_resp_msg->oem_property_id) {
            case BATTMAN_OEM_CHG_LIMIT_EN:
            case BATTMAN_OEM_CHG_LIMIT_CAP:
            case BATTMAN_OEM_USBIN_SUSPEND:
            case BATTMAN_OEM_CHARGING_SUSPNED:
                CHG_DBG("%s set property:%d successfully\n", __func__, oem_write_buffer_resp_msg->oem_property_id);
                ack_set = true;
                break;
            default:
                ack_set = true;
                pr_err("Unknown property_id: %u\n", oem_write_buffer_resp_msg->oem_property_id);
            }
        } else {
            pr_err("Incorrect response length %zu for OEM_OPCODE_READ_BUFFER\n", len);
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

    // pr_err("owner: %u type: %u opcode: %u len: %zu\n", hdr->owner, hdr->type, hdr->opcode, len);

    if (hdr->owner == PMIC_GLINK_MSG_OWNER_OEM) {
        if (hdr->type == MSG_TYPE_NOTIFY)
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

int asuslib_init(void) {
    int rc = 0;
    struct pmic_glink_client_data client_data = { };
    struct pmic_glink_client    *client;

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

    if(g_ASUS_hwID <= HW_REV_EVB2)
    {
        POGO_OTG_GPIO = of_get_named_gpio(g_bcdev->dev->of_node, "POGO_OTG_EN", 0);
        rc = gpio_request(POGO_OTG_GPIO, "POGO_OTG_EN");
        if (rc) {
            pr_err("%s: Failed to initalize the POGO_OTG_EN\n", __func__);
            return -1;
        }
    }

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
    // ChgPD_Info.VBUS_SRC = 0;
    ChgPD_Info.chg_limit_en = 0;
    ChgPD_Info.chg_limit_cap = 0;
    ChgPD_Info.usbin_suspend_en = 0;
    ChgPD_Info.charging_suspend_en = 0;
    ChgPD_Info.firmware_version = 0;
    //[---] Init the info structure of ChargerPD from ADSP
    bat_extcon = extcon_dev_allocate(asus_fg_extcon_cable);
    if (IS_ERR(bat_id_extcon)) {
        rc = PTR_ERR(bat_extcon);
    }       
    printk("[BAT]extcon_dev_register");
    rc = extcon_dev_register(bat_extcon);
    bat_extcon->name = st_battery_name;
    bat_id_extcon = extcon_dev_allocate(asus_fg_extcon_cable);
    if (IS_ERR(bat_id_extcon)) {
        rc = PTR_ERR(bat_id_extcon);
    }       
    printk("[BAT]extcon_dev_register");
    rc = extcon_dev_register(bat_id_extcon);

    //[+++]Register the extcon for quick_charger
    quickchg_extcon = extcon_dev_allocate(asus_fg_extcon_cable);
    if (IS_ERR(quickchg_extcon)) {
        rc = PTR_ERR(quickchg_extcon);
        printk(KERN_ERR "[BAT][CHG] failed to allocate ASUS quickchg extcon device rc=%d\n", rc);
    }

    rc = extcon_dev_register(quickchg_extcon);
    if (rc < 0)
        printk(KERN_ERR "[BAT][CHG] failed to register ASUS quickchg extcon device rc=%d\n", rc);
    //[---]Register the extcon for quick_charger

    asus_get_Batt_ID();

    CHG_DBG_E("Load the asuslib_init Succesfully\n");
    return rc;
}

int asuslib_deinit(void) {
    //int rc;

    class_unregister(&asuslib_class);
    return 0;
}
