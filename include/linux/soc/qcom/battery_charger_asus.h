/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */
 
#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
#include <linux/device.h>
#include <linux/power_supply.h>
#endif

#ifndef _BATTERY_CHARGER_H
#define _BATTERY_CHARGER_H

enum battery_charger_prop {
	BATTERY_RESISTANCE,
	BATTERY_CHARGER_PROP_MAX,
};

//ASUS_BSP Beryl +++ 
#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
#define QTI_POWER_SUPPLY_CHARGED   0x0001
#define QTI_POWER_SUPPLY_UNCHARGED 0x0002
extern void qti_charge_register_notify(struct notifier_block *nb);
extern void qti_charge_unregister_notify(struct notifier_block *nb);
#endif
//ASUS_BSP Beryl ---

#if IS_ENABLED(CONFIG_QTI_BATTERY_CHARGER)
int qti_battery_charger_get_prop(const char *name,
				enum battery_charger_prop prop_id, int *val);
#else
static inline int
qti_battery_charger_get_prop(const char *name,
				enum battery_charger_prop prop_id, int *val)
{
	return -EINVAL;
}
#endif

#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
//[+++] ASUS_BSP : Add for sub-function
struct psy_state {
	struct power_supply	*psy;
	char			*model;
	const int		*map;
	u32			*prop;
	u32			prop_count;
	u32			opcode_get;
	u32			opcode_set;
};

enum psy_type {
	PSY_TYPE_BATTERY,
	PSY_TYPE_USB,
	PSY_TYPE_WLS,
	PSY_TYPE_MAX,
};

struct battery_chg_dev {
	struct device			*dev;
	struct class			battery_class;
	struct pmic_glink_client	*client;
	struct mutex			rw_lock;
	struct completion		ack;
	struct completion		fw_buf_ack;
	struct completion		fw_update_ack;
	struct psy_state		psy_list[PSY_TYPE_MAX];
	struct dentry			*debugfs_dir;
	u32				*thermal_levels;
	const char			*wls_fw_name;
	int				curr_thermal_level;
	int				num_thermal_levels;
	atomic_t			state;
	struct work_struct		subsys_up_work;
	struct work_struct		usb_type_work;
	int				fake_soc;
	bool				block_tx;
	bool				ship_mode_en;
	bool				debug_battery_detected;
	bool				wls_fw_update_reqd;
	u32				wls_fw_version;
	u16				wls_fw_crc;
	struct notifier_block		reboot_notifier;
	u32				thermal_fcc_ua;
	u32				restrict_fcc_ua;
	u32				last_fcc_ua;
	u32				usb_icl_ua;
	bool				restrict_chg_en;
	/* To track the driver initialization status */
	bool				initialized;
};

int asuslib_init(void);
int asuslib_deinit(void);
int asus_chg_resume(struct device *dev);
void set_qc_stat(int status);
//[---] ASUS_BSP : Add for sub-function
#endif
#endif
