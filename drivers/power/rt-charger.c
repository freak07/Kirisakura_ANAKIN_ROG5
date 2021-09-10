/*
 * Dummy Charger driver for test RT1711
 *
 * Copyright (c) 2012 Marvell International Ltd.
 * Author:	Jeff Chang <jeff_chagn@mrichtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/usb/tcpci.h>
#include <linux/workqueue.h>
#include <linux/usb/rt_tcpm.h>
#include <linux/device.h>
#include <linux/usb/role.h>
#include <linux/usb/rt1711h.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/usb.h>
#include <linux/usb/pd_policy_engine.h>
#include <linux/extcon.h>
#include <../extcon/extcon.h>

extern struct gpio_control *gpio_ctrl;
extern struct extcon_dev *quickchg_extcon;

int test_flag;
static int tcpc_swap_state;
static int tcpc_pd_state;
static int tcpc_typec_state;
static int tcpc_pre_typec_state;
static uint16_t pre_vid_ext = 0;
extern uint16_t vid_ext;
///extern int asus_request_DPDM_flag(int enable);
extern void rt1715_dwc3_msm_usb_set_role(enum usb_role role);
extern int BTM_OTG_EN(bool enable);
extern int asus_extcon_set_state_sync(struct extcon_dev *edev, int cable_state);
extern int asus_set_invalid_audio_dongle(int src, int set);
extern int get_net_status(void);
extern int get_prodock_state (void);
extern int aura_screen_on;
extern bool g_hpd;
#ifdef CONFIG_USB_EC_DRIVER
extern uint8_t gDongleType;
#else
static uint8_t gDongleType;
#endif
static int tcpc_sink_voltage;
static int tcpc_sink_current;
static int usb1_active=0;
static int usb2_active=0;
static int usb2_stopPower=0;
static int usb1_stopPower_screen=0;
static int usb2_stopPower_screen=0;
static int pro_dock_active_side = 0;
static int pro_dock_active_bottom = 0;
static int gamevice_active = 0;
static int gamepad_active = 0;
static int btm_vid=0, btm_pid=0;
static int lastcall = 0;
uint16_t vid=0;
struct notifier_block	host_nb;
struct delayed_work rdo_work;
struct workqueue_struct *rdo_wq;
struct delayed_work close5vwork;
struct workqueue_struct *close5v_wq;
static void close5v_by_suspend(struct work_struct *work);
struct delayed_work open5vwork;
struct workqueue_struct *open5v_wq;
static void open5v_by_suspend(struct work_struct *work);
struct rt_charger_info *g_info;
static void rt1715_send_rdo(struct work_struct *work);

//Owner
#define MSG_OWNER_RT			32784 //add MSG_OWNER_RT for RT1715
//Type
#define MSG_TYPE_RT_ADSP_NOTIFY		0xF0
#define MSG_TYPE_ADSP_RT_NOTIFY		0xF1
//Opcode
#define MSG_OPCODE_RT_PDO		0xF2
#define MSG_OPCODE_RT_VID		0xF3
#define MSG_OPCODE_RT_RDO		0xF4
#define MSG_OPCODE_RT_APSD		0xF5
#define MSG_OPCODE_RT_SIDE_OTG	0xF6

#define Bottom_Port_Not_Asus_VID_or_No_charger 100
#define Bottom_Port_Asus_VID 101

#define RT_WAIT_TIME_MS			1000

struct rt1715_pdo_resp_msg {
	struct pmic_glink_hdr	hdr;
	u8	tcpc_pd_state;
	u8	nr;
	u32	pdos[PDO_MAX_NR];
	u32	vid;
};

struct rt1715_vid_resp_msg {
	struct pmic_glink_hdr	hdr;
	u8	tcpc_pd_state;
	u32	vid;
};

struct rt1715_apsd_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32	value;
};

struct rt1715_side_otg_resp_msg {
	struct pmic_glink_hdr	hdr;
	u8	side_otg_en;
};

struct rt1715_resp_msg {
	struct pmic_glink_hdr	hdr;
};

struct rt1715_rdo_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32 selected_pos;
	u32 request_voltage_mv;
	u32 request_current_ma;
	u32 bPPSSelected;
};

/*
 * rt_chg_get_remote_cc return
 * -1 : query fail
 *  0 : unattach
 *  1 : attach SNK
 *  2 : attach SRC
 * */
int rt_chg_get_remote_cc(void) {
    struct tcpc_device *tcpc;
    int result, cc1, cc2;
    tcpc = tcpc_dev_get_by_name("type_c_port0");

	if (tcpci_get_cc(tcpc) < 0)
	{
		result = -1;
		pr_info("%s get CC status fail result = %d\n", __func__, result);
	}
	else {
		cc1 = tcpc->typec_remote_cc[0];
		cc2 = tcpc->typec_remote_cc[1];
		if (cc1 > 0 && cc2 <=0){
			if (cc1 <= TYPEC_CC_VOLT_RD)
				result = 2;
			else
				result = 1;
		}
		else if (cc2 > 0 && cc1 <=0){
			if (cc2 <= TYPEC_CC_VOLT_RD)
				result = 2;
			else
				result = 1;
		}
		else
			result = 0;
		pr_info("%s CC1/CC2 status: %d/%d result = %d\n", __func__, cc1, cc2, result);
	}

    return result;
}
EXPORT_SYMBOL(rt_chg_get_remote_cc);

bool rt_chg_check_asus_vid(void) {
	pr_info("%s: vid = 0x%04x, vid_ext = 0x%04x\n", __func__, vid, vid_ext);
	if(vid==2821 || vid_ext==2821)
		return true;
	else
		return false;
}
EXPORT_SYMBOL(rt_chg_check_asus_vid);

int rt_chg_get_during_swap(void) {
    struct tcpc_device *tcpc;
    tcpc = tcpc_dev_get_by_name("type_c_port0");

    pr_info("%s: during swap = %d\n", __func__, tcpc->pd_port.pe_data.during_swap);
    return tcpc->pd_port.pe_data.during_swap;
}
EXPORT_SYMBOL(rt_chg_get_during_swap);

int typec_disable_function(bool disable) {
    struct tcpc_device *tcpc;
    tcpc = tcpc_dev_get_by_name("type_c_port0");

    tcpm_typec_disable_function(tcpc, disable);
    pr_info("%s enter, disable= %d\n", __func__, disable);
    return 0;
}
EXPORT_SYMBOL(typec_disable_function);

//int tcpc_check_vsafe0v(struct tcpc_device *tcpc)
//{
	//pr_info("%s !!!!!!!!!!Please inpement check vsafe0v function !!!!!!!!!\n", __func__);
	//return 0;
//}

//static int chg_enable_vbus(struct rt_charger_info *info, int enable)
//{
	////pd_dbg_info("%s enable = %d\n", __func__, enable);
	//gpio_set_value(info->vbus_gpio, enable);
	//info->status = enable ?
		//POWER_SUPPLY_STATUS_CHARGING : POWER_SUPPLY_STATUS_DISCHARGING;
	//return 0;
//}

static int rt_chg_handle_sink_vbus(struct tcp_notify *tcp_noti)
{
	tcpc_sink_voltage = tcp_noti->vbus_state.mv;
	tcpc_sink_current = tcp_noti->vbus_state.ma;
	pr_info("%s tcpc_sink_voltage = %d, tcpc_sink_current = %d\n", __func__,tcpc_sink_voltage, tcpc_sink_current);
	return 0;
}

int asus_request_BTM_otg_en(int enable){
	int rc = 0;
 
	pr_info("%s enable = %d\n", __func__, enable);
	rc = BTM_OTG_EN(enable);
	if (rc) {
		pr_info("failed to control HAPTICS_BOOST_VREG_EN\n");
		return -1;
	}
 
	rc = gpio_direction_output(gpio_ctrl->BTM_OTG_EN, enable);
	if (rc) {
		pr_info("failed to control BTM_OTG_EN\n");
		return -1;
	}

	return 0;
}

static int rt_chg_handle_source_vbus(struct tcp_notify *tcp_noti)
{
	bool enable = (tcp_noti->vbus_state.mv > 0) ? true : false;
	int rc;
 
	rc = asus_request_BTM_otg_en(enable && !usb2_stopPower && !usb2_stopPower_screen);

	msleep(5);
	pr_info("[PD] %s enable = %d, usb2_stopPower = %d, usb2_stopPower_screen = %d, rc = %d\n",
		__func__, enable, usb2_stopPower, usb2_stopPower_screen, rc);

	return 0;
}

//static int rt_chg_handle_source_vbus(struct tcp_notify *tcp_noti, int enable)
//{
	//struct power_supply *chg;
	//struct tcpc_device *tcpc;
	//union power_supply_propval val;

	//chg = power_supply_get_by_name("rt-chg");
	//if (!chg) {
		//pr_err("%s: no rt-charger psy\n", __func__);
		//return -ENODEV;
	//}

	//val.intval = enable;
	//chg->desc->set_property(chg, POWER_SUPPLY_PROP_ONLINE, &val);

	//tcpc = tcpc_dev_get_by_name("type_c_port0");
	//if (!tcpc)
		//return -EINVAL;

//#ifdef CONFIG_USB_POWER_DELIVERY
	//tcpm_notify_vbus_stable(tcpc);
//#endif

	//return 0;
//}

static int chg_get_prop(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct rt_charger_info *info =
		power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = info->status;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = info->online;
		break;
#ifdef CONFIG_DUAL_PD_PORT
	case POWER_SUPPLY_PROP_PD_CAP:
		val->intval = 0;
		break;
#endif
	default:
		return -ENODEV;
	}
	return 0;
}

static int chg_set_prop(struct power_supply *psy,
                     enum power_supply_property psp,
                     const union power_supply_propval *val)
{
#ifdef CONFIG_DUAL_PD_PORT
           int mv, ma;
           int ret = 0;
#endif
           struct rt_charger_info *info =
                     power_supply_get_drvdata(psy);

           switch (psp) {
           case POWER_SUPPLY_PROP_STATUS:
                     break;
           case POWER_SUPPLY_PROP_ONLINE:
                     info->online = val->intval;
                     //chg_enable_vbus(info, info->online);
                     break;
#ifdef CONFIG_DUAL_PD_PORT
           case POWER_SUPPLY_PROP_PD_CAP:
                     /* call tcpm*/
                     mv = ((val->intval)>>16) & 0xFFFF;
                     ma = (val->intval) & 0xFFFF;
                     pr_info("%s mv= %d, ma=%d\n", __func__, mv, ma);
                     ret = tcpm_set_apdo_charging_policy(info->tcpc, DPM_CHARGING_POLICY_PPS,
                                                                                                                     mv, ma, NULL);
                     break;
#endif
           default:
                     return -ENODEV;
           }
#ifdef CONFIG_DUAL_PD_PORT
           return ret;
#else
           return 0;
#endif
}

static enum power_supply_property rt_chg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
#ifdef CONFIG_DUAL_PD_PORT
	POWER_SUPPLY_PROP_PD_CAP,
#endif
};

static int rtchg_init_vbus(struct rt_charger_info *info)
{
//#ifdef CONFIG_OF
	//struct device_node *np = info->dev->of_node;
	//int ret;

	//if (np == NULL) {
		//pr_err("Error: rt-changer np = NULL\n");
		//return -1;
	//}

	//info->vbus_gpio = of_get_named_gpio(np, "rt,vbus_gpio", 0);
	//ret = gpio_request(info->vbus_gpio, "DUMMY CHG VBUS CONTRL");
	//if (ret < 0) {
		//pr_err("Error: failed to request GPIO %d\n", info->vbus_gpio);
		//return ret;
	//}
	//ret = gpio_direction_output(info->vbus_gpio, 0);
	//if (ret < 0) {
		//pr_err("Error: failed to set GPIO as output pin\n");
		//return ret;
	//}
//#endif /* CONFIG_OF */

	//pr_info("%s: OK\n", __func__);
	return 0;
}

int rt1715_glink_write(struct rt_charger_info *info, void *data, int len)
{
	int rc;

	mutex_lock(&info->rw_lock);
	reinit_completion(&info->ack);
	rc = pmic_glink_write(info->client, data, len);
	if (!rc) {
		rc = wait_for_completion_timeout(&info->ack,
					msecs_to_jiffies(RT_WAIT_TIME_MS));
		if (!rc) {
			pr_err("Error, timed out sending message\n");
			mutex_unlock(&info->rw_lock);
			return -ETIMEDOUT;
		}

		rc = 0;
	}
	mutex_unlock(&info->rw_lock);

	return rc;
}

void rt1715_pdo_notify(void) {

	int rc = -1;
	int i, ret;
	struct rt1715_pdo_resp_msg rt1715_pdo_msg = {};
	struct tcpm_remote_power_cap remote_cap;
	struct tcpc_device *tcpc;

	//get pdo +++
	tcpc = tcpc_dev_get_by_name("type_c_port0");
	ret = tcpm_get_remote_power_cap(tcpc, &remote_cap);
	if(ret < 0) {
		pr_info("%s: Get remote power cap fail, error number = %d\n", __func__, ret);
		return;
	}
	rt1715_pdo_msg.nr = remote_cap.nr;
	for (i = 0; i < remote_cap.nr; i++)
		rt1715_pdo_msg.pdos[i] = tcpc->pd_port.pe_data.remote_src_cap.pdos[i];
	//get pdo ---

	rt1715_pdo_msg.hdr.owner = MSG_OWNER_RT;
	rt1715_pdo_msg.hdr.type = MSG_TYPE_RT_ADSP_NOTIFY;
	rt1715_pdo_msg.hdr.opcode = MSG_OPCODE_RT_PDO;
	rt1715_pdo_msg.tcpc_pd_state = tcpc_pd_state;
	rt1715_pdo_msg.vid = vid_ext;

	//pr_info("[USB] %s tcpc_pd_state = %d\n", __func__, tcpc_pd_state);
	rc = rt1715_glink_write(g_info, &rt1715_pdo_msg, sizeof(rt1715_pdo_msg));
	if (rc < 0) {
		pr_err("Error in sending message rc=%d\n", rc);
		return;
	}

}

void rt1715_vid_notify(void) {

	int rc = -1;
	struct rt1715_vid_resp_msg rt1715_vid_msg = {};

	rt1715_vid_msg.hdr.owner = MSG_OWNER_RT;
	rt1715_vid_msg.hdr.type = MSG_TYPE_RT_ADSP_NOTIFY;
	rt1715_vid_msg.hdr.opcode = MSG_OPCODE_RT_VID;
	rt1715_vid_msg.tcpc_pd_state = tcpc_pd_state;
	rt1715_vid_msg.vid = vid_ext;

	//pr_info("[USB] %s vid = 0x%04x\n", __func__, rt1715_vid_msg.vid);
	rc = rt1715_glink_write(g_info, &rt1715_vid_msg, sizeof(rt1715_vid_msg));
	if (rc < 0) {
		pr_err("Error in sending message rc=%d\n", rc);
		return;
	}

	if (quickchg_extcon == NULL){
		pr_info("%s: quickchg_extcon not ready\n", __func__);
		return;
	}
	else{
		if (vid_ext == 0x0b05 && pre_vid_ext == 0x0)
			asus_extcon_set_state_sync(quickchg_extcon, Bottom_Port_Asus_VID);
		else if (vid_ext == 0x0 && pre_vid_ext == 0x0b05)
			asus_extcon_set_state_sync(quickchg_extcon, Bottom_Port_Not_Asus_VID_or_No_charger);
	}

	pr_info("%s: vid_ext = 0x%04x pre_vid_ext = 0x%04x\n", __func__, vid_ext, pre_vid_ext);
	pre_vid_ext = vid_ext;

}

static void rt1715_apsd_notify(struct notifier_block *nb) {

	int rc = -1;
	struct rt1715_apsd_resp_msg rt1715_apsd_msg = {};
	struct rt_charger_info *info = container_of(nb, struct rt_charger_info, nb);

	rt1715_apsd_msg.hdr.owner = MSG_OWNER_RT;
	rt1715_apsd_msg.hdr.type = MSG_TYPE_RT_ADSP_NOTIFY;
	rt1715_apsd_msg.hdr.opcode = MSG_OPCODE_RT_APSD;
	rt1715_apsd_msg.value = 1;

	rc = rt1715_glink_write(info, &rt1715_apsd_msg, sizeof(rt1715_apsd_msg));
	if (rc < 0) {
		pr_err("Error in sending message rc=%d\n", rc);
		return;
	}

}

/* Please handle the notification in notifier call function,
 * User should control the Power here when you got SOURCE_VBUS notification
 * and SINK_VBUS notification
 */
static int chg_tcp_notifer_call(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct tcp_notify *tcp_noti = data;
	struct rt_charger_info *info = container_of(nb, struct rt_charger_info, nb);
	struct tcpc_device *tcpc;
	uint16_t dpm_flags;
	uint32_t vdos[VDO_MAX_NR];
	int ret = 0;
	
	tcpc = tcpc_dev_get_by_name("type_c_port0");

	pr_info("%s: %d\n", __func__, (int)event);

	switch (event) {
	case TCP_NOTIFY_PR_SWAP:
		pr_info("%s TCP_NOTIFY_PR_SWAP\n", __func__);
		//asus_request_DPDM_flag(1);
		break;
	case TCP_NOTIFY_VCONN_SWAP:
		/* Do what you want to do here */
		break;
	case TCP_NOTIFY_HARD_RESET_STATE:
		pr_info("%s TCP_NOTIFY_HARD_RESET_STATE\n", __func__);
		break;
	case TCP_NOTIFY_PD_STATE:
		pr_info("%s TCP_NOTIFY_PD_STATE, pd state = %d\n", __func__,tcp_noti->pd_state.connected);
		tcpc_pd_state = tcp_noti->pd_state.connected;
		switch (tcp_noti->pd_state.connected) {
		case PD_CONNECT_PE_READY_SNK:
		case PD_CONNECT_PE_READY_SNK_PD30:
			dpm_flags = tcpm_inquire_dpm_flags(tcpc);
			info->peer_usb_comm = (dpm_flags &= DPM_FLAGS_PARTNER_USB_COMM);
			pr_info("%s tcpc_pd_state = PD_CONNECT_PE_READY_SNK/PD3.0, USB_COMM = %d\n", __func__, info->peer_usb_comm);
			rt1715_pdo_notify();
			rt1715_vid_notify();
			if (info->peer_usb_comm && tcpc_pre_typec_state == TYPEC_UNATTACHED)
				rt1715_dwc3_msm_usb_set_role(USB_ROLE_DEVICE);
			break;
		case PD_CONNECT_PE_READY_SRC:
		case PD_CONNECT_PE_READY_SRC_PD30:
			ret = tcpm_inquire_pd_partner_inform(tcpc, vdos);
			vid = vdos[0] & 0xFFFF;
			dpm_flags = tcpm_inquire_dpm_flags(tcpc);
			info->peer_usb_comm = (dpm_flags &= DPM_FLAGS_PARTNER_USB_COMM);
			pr_info("%s tcpc_pd_state = PD_CONNECT_PE_READY_SRC, VID = %x, ret = %d, USB_COMM = %d\n",__func__, vid, ret, info->peer_usb_comm);
			rt1715_pdo_notify();
			if (vid == 0x0b05){
				gamepad_active = 1;
				pr_info("%s gamepad_active = %d\n", __func__, gamepad_active);
			}
			break;
		case PD_CONNECT_PE_READY_SNK_APDO:
			dpm_flags = tcpm_inquire_dpm_flags(tcpc);
			info->peer_usb_comm = (dpm_flags &= DPM_FLAGS_PARTNER_USB_COMM);
			pr_info("%s tcpc_pd_state = PD_CONNECT_PE_READY_SNK_APDO, USB_COMM = %d\n", __func__, info->peer_usb_comm);
			/* TODO: pps event */
			rt1715_pdo_notify();
			rt1715_vid_notify();
			if (info->peer_usb_comm && tcpc_pre_typec_state == TYPEC_UNATTACHED)
				rt1715_dwc3_msm_usb_set_role(USB_ROLE_DEVICE);
			break;
		case PD_CONNECT_NONE:
			pr_info("%s tcpc_pd_state = PD_CONNECT_NONE\n", __func__);
			vid_ext = 0;
			rt1715_pdo_notify();
			rt1715_vid_notify();
			break;
		case PD_CONNECT_HARD_RESET:
			pr_info("%s tcpc_pd_state = PD_CONNECT_HARD_RESET,\n", __func__);
			vid_ext = 0;
			rt1715_pdo_notify();
			rt1715_vid_notify();
			break;
		default:
			break;
		}
		break;
	case TCP_NOTIFY_DIS_VBUS_CTRL:
		/* Implement disable power path (otg & charger) behavior here */
		//rt_chg_handle_source_vbus(tcp_noti, 0);
		break;
	case TCP_NOTIFY_SOURCE_VBUS:
		pr_info("%s TCP_NOTIFY_SOURCE_VBUS\n", __func__);
		rt_chg_handle_source_vbus(tcp_noti);
		/* Implement source vbus behavior here */
		//rt_chg_handle_source_vbus(
			//tcp_noti, (tcp_noti->vbus_state.mv > 0) ? 1 : 0);
		break;
	case TCP_NOTIFY_SINK_VBUS:
		pr_info("%s TCP_NOTIFY_SINK_VBUS\n", __func__);
		/* Implement sink vubs behavior here */
		rt_chg_handle_sink_vbus(tcp_noti);
		//asus_request_DPDM_flag(0);
		break;
	case TCP_NOTIFY_SOURCE_VCONN:
		pr_info("%s TCP_NOTIFY_SOURCE_VCONN\n", __func__);
		break;
	case TCP_NOTIFY_DR_SWAP:
		pr_info("%s TCP_NOTIFY_DR_SWAP\n", __func__);
		tcpc_swap_state = tcp_noti->swap_state.new_role;
		if (tcpc_swap_state==0){
		    pr_info("%s TCP_NOTIFY_DR_SWAP: PD_ROLE_UFP\n", __func__);
		    if (info->peer_usb_comm)
				rt1715_dwc3_msm_usb_set_role(USB_ROLE_DEVICE);
		}
		if (tcpc_swap_state==1){
		    pr_info("%s TCP_NOTIFY_DR_SWAP: PD_ROLE_DFP\n", __func__);
		    if (info->peer_usb_comm) 
				rt1715_dwc3_msm_usb_set_role(USB_ROLE_HOST);
		}
		break;
	case TCP_NOTIFY_TYPEC_STATE:
		pr_info("%s TCP_NOTIFY_TYPEC_STATE\n", __func__);
		tcpc_pre_typec_state = tcpc_typec_state;
		tcpc_typec_state = tcp_noti->typec_state.new_state;
		pr_info("%s tcpc_typec_state = %d, tcpc_pre_typec_state = %d\n", __func__, tcpc_typec_state, tcpc_pre_typec_state);
		if (tcpc_typec_state == TYPEC_ATTACHED_SNK) {
			pr_info("%s tcpc_typec_state = TYPEC_ATTACHED_SNK\n", __func__);
		}
		if (tcpc_typec_state == TYPEC_ATTACHED_SRC) {
			pr_info("%s tcpc_typec_state = TYPEC_ATTACHED_SRC\n", __func__);
			pr_info("%s OTG Plug in\n", __func__);
			if (tcpc_pre_typec_state != TYPEC_ATTACHED_SNK)
				rt1715_dwc3_msm_usb_set_role(USB_ROLE_HOST);
		}
		if (tcpc_typec_state == TYPEC_UNATTACHED) {
			pr_info("%s tcpc_typec_state = TYPEC_UNATTACHED\n", __func__);
			pr_info("%s OTG Plug out\n", __func__);
			rt1715_dwc3_msm_usb_set_role(USB_ROLE_NONE);
			if (gamepad_active) {
				gamepad_active = 0;
				pr_info("%s gamepad_active = %d\n", __func__, gamepad_active);
			}
		}
		//analog audio dongle +++
		if (tcpc_typec_state == TYPEC_ATTACHED_AUDIO) {
			pr_info("%s tcpc_typec_state = TYPEC_ATTACHED_AUDIO\n", __func__);
			ret = asus_set_invalid_audio_dongle(2, 1);
		}
		else if (tcpc_pre_typec_state == TYPEC_ATTACHED_AUDIO) {
			pr_info("%s tcpc_pre_typec_state = TYPEC_ATTACHED_AUDIO\n", __func__);
			ret = asus_set_invalid_audio_dongle(2, 0);
		}
		//analog audio dongle ---
		break;
	default:
		pr_info("%s TCP_NOTIFY_DEFAULT, event = %d\n", __func__, (int)event);
		break;
	};

	return NOTIFY_OK;
}

//void test_set_flag(int en)
//{
	//test_flag = en;
//}

//void dwork_func(struct work_struct *work)
//{
	//struct rt_charger_info *info =
		//container_of(work, struct rt_charger_info, dwork.work);

	//if (test_flag)
		//pr_info("WHATWHATWHATWHATWHATWHATWWWWWWWHATHATHATHATHATHATHAT\n");

	//schedule_delayed_work(&info->dwork, msecs_to_jiffies(10));
//}

static char * rt_charger_supplied_to[] = {
	"main-battery",
};

static const struct power_supply_desc rt_power_supply_desc = {
	.name		= "rt-chg",
	.type 		= POWER_SUPPLY_TYPE_UNKNOWN,
	.properties	= rt_chg_props,
	.num_properties = ARRAY_SIZE(rt_chg_props),
	.get_property	= chg_get_prop,
	.set_property	= chg_set_prop,
};

static int rt_chg_power_supply_init(struct rt_charger_info *info)
{
	struct power_supply_config psy_cfg = { .drv_data = info, };

	psy_cfg.supplied_to = rt_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(rt_charger_supplied_to);

	info->chg = power_supply_register(info->dev, &rt_power_supply_desc,
					&psy_cfg);

	return PTR_ERR_OR_ZERO(info->chg);
}

static void handle_message(struct rt_charger_info *info, void *data,
				size_t len)
{
	struct rt1715_resp_msg *resp_msg = data;
	struct rt1715_pdo_resp_msg *adsp_pdo_msg;
	struct rt1715_vid_resp_msg *adsp_vid_msg;
	struct rt1715_rdo_resp_msg *adsp_rdo_msg;
	struct rt1715_apsd_resp_msg *adsp_apsd_msg;
	struct rt1715_side_otg_resp_msg *adsp_side_otg_msg;
	int i;
	bool ack_set = false;

	switch (resp_msg->hdr.opcode) {
	case MSG_OPCODE_RT_PDO:
		if (len == sizeof(*adsp_pdo_msg)) {
			adsp_pdo_msg = data;
			ack_set = true;
			pr_info("[PD] %s adsp_rt1715_msg tcpc_pd_state = %d\n", __func__, adsp_pdo_msg->tcpc_pd_state);
			for (i = 0; i < adsp_pdo_msg->nr; i++) {
				pr_info("[PD] %s nr = %d, pdos = 0x%x\n", __func__, i, adsp_pdo_msg->pdos[i]);
			}
		} else {
			pr_info("[PD] %s Incorrect response length %zu for MSG_OPCODE_RT_PDO\n", __func__, len);
		}
		break;
	case MSG_OPCODE_RT_VID:
		if (len == sizeof(*adsp_vid_msg)) {
			adsp_vid_msg = data;
			ack_set = true;
			pr_info("[PD] %s adsp_vid_msg vid = 0x%04x\n", __func__, adsp_vid_msg->vid);
		} else {
			pr_info("[PD] %s Incorrect response length %zu for MSG_OPCODE_RT_VID\n", __func__, len);
		}
		break;
	case MSG_OPCODE_RT_RDO:
		if (len == sizeof(*adsp_rdo_msg)) {
			adsp_rdo_msg = data;
			ack_set = true;
			pr_info("[PD] %s adsp_rdo_msg\n", __func__);
			g_info->select_pos = adsp_rdo_msg->selected_pos;
			g_info->rdo_mv = adsp_rdo_msg->request_voltage_mv;
			g_info->rdo_ma = adsp_rdo_msg->request_current_ma;
			g_info->fixed_pdo = !(adsp_rdo_msg->bPPSSelected);
			queue_delayed_work(rdo_wq, &rdo_work, 0);
		} else {
			pr_info("[PD] %s Incorrect response length %zu for MSG_OPCODE_RT_VID\n", __func__, len);
		}
		break;
	case MSG_OPCODE_RT_APSD:
		if (len == sizeof(*adsp_apsd_msg)) {
			adsp_apsd_msg = data;
			ack_set = true;
			pr_err("ADSP LOG: adsp_apsd_msg value = %d\n",adsp_apsd_msg->value);
		} else {
			pr_err("Incorrect response length %zu for MSG_OPCODE_RT_APSD\n", len);
		}
		break;
	case MSG_OPCODE_RT_SIDE_OTG:
		if (len == sizeof(*adsp_side_otg_msg)) {
			adsp_side_otg_msg = data;
			ack_set = true;
			pr_err("ADSP LOG: adsp_side_otg_msg value = %d\n",adsp_side_otg_msg->side_otg_en);
		} else {
			pr_err("Incorrect response length %zu for MSG_OPCODE_RT_SIDE_OTG\n", len);
		}
		break;
	default:
		pr_info("[PD] %s Unknown opcode: %u\n", __func__, resp_msg->hdr.opcode);
		ack_set = true;
		break;
	}

	if (ack_set)
		complete(&info->ack);
}

static int rt_chg_callback(void *priv, void *data, size_t len)
{
	struct pmic_glink_hdr *hdr = data;
	struct rt_charger_info *info = priv;

	pr_debug("owner: %u type: %u opcode: %#x len: %zu\n", hdr->owner,
		hdr->type, hdr->opcode, len);

	if (hdr->owner == MSG_OWNER_RT){
		switch (hdr->type){
			case MSG_TYPE_ADSP_RT_NOTIFY:
				handle_message(info, data, len);
				break;
			default:
				break;
		}
	}

	return 0;
}

static void rt_chg_state_cb(void *priv, enum pmic_glink_state state)
{
	//struct rt_charger_info *info = priv;
	pr_debug("rt_chg_state_cb: %d\n", state);
}

static int isCharging(void){
	struct power_supply *usb_psy;
	union power_supply_propval val = {0};
	int vbus_online = 0;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_info("[PD] rt_drm_notifier, Could not get usb psy\n");
		return -ENODEV;
	}

	power_supply_get_property(usb_psy,
			POWER_SUPPLY_PROP_ONLINE, &val);
	vbus_online = val.intval;

	pr_info("[PD] rt_drm_notifier, isCharging, vbus_online = %d\n", vbus_online);
	return vbus_online;
}

static int isKeep5v(int vid, int pid){
	pr_info("rt_host_notifier, isKeep5v VID = %04x, PID = %04x\n", vid, pid);
	/*
	 * White List        pid   vid
	 * host controller : 7531  2     (0x1d6b 0x0002)
	 * host controller : 7531  3     (0x1d6b 0x0003)
	 * Kunai bumper    : 2821  30976 (0x0b05 0x7900)
	 * Kunai holder    : 2821  30977 (0x0b05 0x7901)
	 * Kunai 3 bumper  : 2821  30980 (0x0b05 0x7904)
	 * Kunai 3 holder  : 2821  30981 (0x0b05 0x7905)
	 */
	if ((vid == 2821 && pid == 30980) || (vid == 2821 && pid == 30981) ||
	(vid == 2821 && pid == 30976) || (vid == 2821 && pid == 30977) ||
	(vid == 7531 && pid == 2) || (vid == 7531 && pid == 3))
		return 1;
	else
		return 0;
}

static void checkOTGdevice(int vid, int pid, char* ipro, int from, int event){
	char prodock[]="ASUS Pro-Dock USB2.0";
	char gamevice[]="GV187";
	//pr_info("rt_host_notifier, checkOTGdevice, VID = %04x, PID = %04x, iProduct = %s, last call = %d, event = %d\n", vid, pid, ipro, from, event);
	if (vid == 8457 && pid == 10263){
		if (from == 1){
			if (event == USB_DEVICE_ADD && !strcmp(ipro, prodock))
				pro_dock_active_side = 1;
			else if (event == USB_DEVICE_REMOVE)
				pro_dock_active_side = 0;
		}
		else if (from == 2){
			if (event == USB_DEVICE_ADD && !strcmp(ipro, prodock))
				pro_dock_active_bottom= 1;
			else if (event == USB_DEVICE_REMOVE)
				pro_dock_active_bottom = 0;
		}
		pr_info("rt_host_notifier, pro_dock_active side/bottom = %d/%d\n", pro_dock_active_side, pro_dock_active_bottom);
	} else if (vid == 10232 && pid == 3005 && (from == 2)){
		if (event == USB_DEVICE_ADD && !strcmp(ipro, gamevice))
			gamevice_active = 1;
		else if (event == USB_DEVICE_REMOVE)
			gamevice_active = 0;
		pr_info("rt_host_notifier, gamevice_active = %d\n", gamevice_active);
	}
}

static void asus_request_SIDE_otg_en(int enable) {

	int rc = -1;
	struct rt1715_side_otg_resp_msg rt1715_side_otg_msg = {};

	rt1715_side_otg_msg.hdr.owner = MSG_OWNER_RT;
	rt1715_side_otg_msg.hdr.type = MSG_TYPE_RT_ADSP_NOTIFY;
	rt1715_side_otg_msg.hdr.opcode = MSG_OPCODE_RT_SIDE_OTG;
	rt1715_side_otg_msg.side_otg_en = enable;

	if (g_info != NULL)
		rc = rt1715_glink_write(g_info, &rt1715_side_otg_msg, sizeof(rt1715_side_otg_msg));
	if (rc < 0) {
		pr_err("Error in sending message rc=%d\n", rc);
		return;
	}

}

static void open5v_by_suspend(struct work_struct *work) {
	if(usb1_stopPower_screen){
		pr_info("[PD] rt_drm_notifier, panel power on -- reopen side power\n");
		asus_request_SIDE_otg_en(1);
		usb1_stopPower_screen = 0;
	}

	if(usb2_stopPower_screen){
		pr_info("[PD] rt_drm_notifier, panel power on -- reopen btm power\n");
		if(!usb2_stopPower){
			typec_disable_function(1);
			msleep(100);
			typec_disable_function(0);
		}
		usb2_stopPower_screen = 0;
	}
	cancel_delayed_work(&close5vwork);
}

static void close5v_by_suspend(struct work_struct *work) {
	if(pro_dock_active_side && (get_net_status() == 0) && (get_prodock_state() ==1) && !g_hpd){
		pr_info("[PD] rt_drm_notifier, panel power off -- stop side power\n");
		asus_request_SIDE_otg_en(0);
		usb1_stopPower_screen = 1;
	}

	if((gamepad_active && !aura_screen_on) || gamevice_active || (pro_dock_active_bottom && (get_net_status() == 0) && (get_prodock_state() ==1))){
		pr_info("[PD] rt_drm_notifier, panel power off -- stop btm power\n");
		rt1715_dwc3_msm_usb_set_role(USB_ROLE_NONE);
		asus_request_BTM_otg_en(0);
		usb2_stopPower_screen = 1;
	}
}

void rt_send_screen_suspend(void){
	pr_info("[PD] rt_drm_notifier, rt_send_screen_suspend\n");
	if(!isCharging()){
		queue_delayed_work(close5v_wq, &close5vwork, HZ * 5);
	}
}
EXPORT_SYMBOL(rt_send_screen_suspend);

void rt_send_screen_resume(void){
	pr_info("[PD] rt_drm_notifier, rt_send_screen_resume\n");
	queue_delayed_work(open5v_wq, &open5vwork, 0);
}
EXPORT_SYMBOL(rt_send_screen_resume);

static int rt_host_notifier(struct notifier_block *nb,
	unsigned long event, void *ptr){
	struct usb_device *udev = ptr;
	struct usb_bus *ubus = ptr;
	char usb1[]="a600000.dwc3";
	char usb2[]="a800000.dwc3";
	char buf[24], devname[15];

	if(event <=2) {
		usb_string(udev, udev->descriptor.iProduct, buf, sizeof(buf));
		pr_info("rt_host_notifier, VID = %04x, PID = %04x, iProduct = %s\n", le16_to_cpu(udev->descriptor.idVendor), le16_to_cpu(udev->descriptor.idProduct), buf);
		checkOTGdevice(le16_to_cpu(udev->descriptor.idVendor), le16_to_cpu(udev->descriptor.idProduct), buf, lastcall, event);
		if (!strcmp(dev_name(udev->bus->controller->parent), usb2) && event == USB_DEVICE_ADD){
			btm_vid = le16_to_cpu(udev->descriptor.idVendor);
			btm_pid = le16_to_cpu(udev->descriptor.idProduct);
		}
	} else{
		//pr_info("rt_host_notifier, ubus->controller = %s\n", dev_name(ubus->controller));
		if(ubus->controller->parent) {
			strcpy(devname, dev_name(ubus->controller->parent));
			//pr_info("rt_host_notifier,  devname = %s\n", devname);
			if(!strcmp(devname, usb1)){
				if (event == USB_BUS_ADD){
					usb1_active = 1;
					lastcall = 1;
					usb1_stopPower_screen = 0;
				}
				else if (event == USB_BUS_REMOVE){
					usb1_active = 0;
				}
			}

			if(!strcmp(devname, usb2)){
				if (event == USB_BUS_ADD){
					usb2_active = 1;
					lastcall = 2;
					usb2_stopPower = 0;
					usb2_stopPower_screen = 0;
				}
				else if (event == USB_BUS_REMOVE){
					usb2_active = 0;
					btm_vid = 0;
					btm_pid = 0;
				}
				pr_info("rt_host_notifier,  usb1_active = %d, usb2_active = %d, event = %lu\n", usb1_active, usb2_active, event);
			}
		}
	}

	if (usb1_active && usb2_active && (event == USB_DEVICE_ADD) && (gDongleType == 0)){
		if(!isKeep5v(btm_vid, btm_pid)){
			usb2_stopPower = 1;
			rt1715_dwc3_msm_usb_set_role(USB_ROLE_NONE);
			asus_request_BTM_otg_en(0);
			pr_info("rt_host_notifier,  close bottom 5v\n");
		} else{
			//pr_info("rt_host_notifier,  bottom is Gamevice do not close bottom 5v\n");
		}
	} else if (usb2_stopPower && !usb1_active){
		usb2_stopPower = 0;
		typec_disable_function(1);
		msleep(100);
		typec_disable_function(0);
		pr_info("rt_host_notifier,  reopen bottom 5v\n");
	}
	return NOTIFY_DONE;
}

static void rt1715_send_rdo(struct work_struct *work) 
{
	int ret = 10;
	uint8_t max_power_policy = 0x21;

	pr_info("[PD] %s pos = %d, mv = %d, ma = %d, fixed_pdo = %d", 
		__func__, g_info->select_pos, g_info->rdo_mv, g_info->rdo_ma, g_info->fixed_pdo);

	if (g_info->tcpc->pd_port.pe_pd_state != PE_SNK_READY){
		pr_info("[PD] %s pd state is not ready\n", __func__);
		return;
	}

	if (g_info->fixed_pdo == 1){
		/* reqeust fixed pdo */
		tcpm_set_pd_charging_policy(g_info->tcpc, max_power_policy, NULL);
		tcpm_dpm_pd_request(g_info->tcpc, g_info->rdo_mv, g_info->rdo_ma, NULL);
	}
	else{
		/* request APDO */
		ret = tcpm_set_apdo_charging_policy(g_info->tcpc, DPM_CHARGING_POLICY_PPS, g_info->rdo_mv, g_info->rdo_ma, NULL);
	}
}

static int rt_charger_probe(struct platform_device *pdev)
{
	struct rt_charger_info *info;
	struct device *dev = &pdev->dev;
	struct pmic_glink_client_data client_data = { };
	int ret;

	pr_info("%s\n", __func__);
	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->dev = &pdev->dev;

	client_data.id = MSG_OWNER_RT;
	client_data.name = "RT_charger";
	client_data.msg_cb = rt_chg_callback;
	client_data.priv = info;
	client_data.state_cb = rt_chg_state_cb;

	info->client = pmic_glink_register_client(dev, &client_data);
	if (IS_ERR(info->client)) {
		ret = PTR_ERR(info->client);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Error in registering with pmic_glink %d\n",
				ret);
		return ret;
	}

	mutex_init(&info->rw_lock);
	init_completion(&info->ack);

	platform_set_drvdata(pdev, info);

	ret = rt_chg_power_supply_init(info);

 	if (ret < 0) {
		dev_err(&pdev->dev, "chg register power supply fail\n");
 		return -EINVAL;
 	}

	ret = rtchg_init_vbus(info);
	if (ret < 0) {
		pr_err("%s gpio init fail\n", __func__);
		return -EINVAL;
	}

	/* Get tcpc device by tcpc_device'name */
	info->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!info->tcpc) {
		dev_err(&pdev->dev, "get rt1711-tcpc fail\n");
		//power_supply_unregister(&info->chg);
		return -ENODEV;
	}

	/* register tcpc notifier */
	info->nb.notifier_call = chg_tcp_notifer_call;
	ret = register_tcp_dev_notifier(info->tcpc, &info->nb, TCP_NOTIFY_TYPE_ALL);
	if (ret < 0) {
		dev_err(&pdev->dev, "register tcpc notifer fail\n");
		return -EINVAL;
	}

	//INIT_DELAYED_WORK(&info->dwork, dwork_func);

	//schedule_delayed_work(&info->dwork, 0);

	host_nb.notifier_call = rt_host_notifier;
	usb_register_notify(&host_nb);

	close5v_wq = create_singlethread_workqueue("close5v_by_suspend");
	INIT_DELAYED_WORK(&close5vwork, close5v_by_suspend);

	open5v_wq = create_singlethread_workqueue("open5v_by_suspend");
	INIT_DELAYED_WORK(&open5vwork, open5v_by_suspend);

	rdo_wq = create_singlethread_workqueue("rt1715_send_rdo");
	INIT_DELAYED_WORK(&rdo_work, rt1715_send_rdo);

	g_info = info;

	ret = tcpm_inquire_typec_attach_state(info->tcpc);
	if (ret == TYPEC_ATTACHED_SNK)
		rt1715_apsd_notify(&info->nb);

	pr_info("%s: disable/enable typec function\n", __func__);
	typec_disable_function(1);
	msleep(100);
	typec_disable_function(0);

	pr_info("%s: OK!\n", __func__);
	return 0;
}

static int rt_charger_remove(struct platform_device *pdev)
{
	//struct rt_charger_info *info = platform_get_drvdata(pdev);

	//power_supply_unregister(info->chg);
	return 0;
}

static struct of_device_id rt_match_table[] = {
	{.compatible = "richtek,rt-charger",},
};

static struct platform_driver rt_charger_driver = {
	.driver = {
		.name = "rt-charger",
		.owner = THIS_MODULE,
		.of_match_table = rt_match_table,
	},
	.probe = rt_charger_probe,
	.remove = rt_charger_remove,
};

static int __init rt_chg_init(void)
{
	int ret;

	ret = platform_driver_register(&rt_charger_driver);
	if(ret)
		pr_info("%s: unable to register driver (%d)\n", __func__, ret);

	return ret;
}

static void __exit rt_chg_exit(void)
{
	platform_driver_unregister(&rt_charger_driver);
}
late_initcall(rt_chg_init);
module_exit(rt_chg_exit);

MODULE_DESCRIPTION("Dummy Charger driver  for kernel-4.9");
MODULE_LICENSE("GPL");
