/*
 * Copyright (C) 2016 RichTek Inc.
 *
 * TCPC Interface for dual role
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/usb/tcpci.h>
#include <linux/usb/tcpci_typec.h>
#include <linux/usb/typec.h>

#ifdef CONFIG_DUAL_ROLE_USB_INTF
static struct tcpc_device *tcpc_dev;

static int tcpc_dual_role_set_prop_pr(struct typec_port *port,
				      enum typec_role trole)
{
	int ret;
	uint8_t val, role;

	switch (trole) {
	case TYPEC_SOURCE:
		val = DUAL_ROLE_PROP_PR_SRC;
		role = PD_ROLE_SOURCE;
		break;
	case TYPEC_SINK:
		val = DUAL_ROLE_PROP_PR_SNK;
		role = PD_ROLE_SINK;
		break;
	default:
		return 0;
	}

	if (val == tcpc_dev->dual_role_pr) {
		pr_info("%s wrong role (%d->%d)\n",
			__func__, tcpc_dev->dual_role_pr, val);
		return 0;
	}

	ret = tcpm_dpm_pd_power_swap(tcpc_dev, role, NULL);
	pr_info("%s power role swap (%d->%d): %d\n",
		__func__, tcpc_dev->dual_role_pr, val, ret);

	if (ret == TCPM_ERROR_NO_PD_CONNECTED) {
		ret = tcpm_typec_role_swap(tcpc_dev);
		pr_info("%s typec role swap (%d->%d): %d\n",
			__func__, tcpc_dev->dual_role_pr, val, ret);
	}

	return ret;
}

static int tcpc_dual_role_set_prop_dr(struct typec_port *port,
				      enum typec_data_role data)
{
	int ret;
	uint8_t val, role;

	switch (data) {
	case TYPEC_HOST:
		val = DUAL_ROLE_PROP_DR_HOST;
		role = PD_ROLE_DFP;
		break;
	case TYPEC_DEVICE:
		val = DUAL_ROLE_PROP_DR_DEVICE;
		role = PD_ROLE_UFP;
		break;
	default:
		return 0;
	}

	if (val == tcpc_dev->dual_role_dr) {
		pr_info("%s wrong role (%d->%d)\n",
			__func__, tcpc_dev->dual_role_dr, val);
		return 0;
	}

	ret = tcpm_dpm_pd_data_swap(tcpc_dev, role, NULL);
	pr_info("%s data role swap (%d->%d): %d\n",
		__func__, tcpc_dev->dual_role_dr, val, ret);

	return ret;
}

static int tcpc_dual_role_set_prop_vconn(struct typec_port *port,
					 enum typec_role trole)
{
	int ret;
	uint8_t val, role;

	switch (trole) {
	case TYPEC_SINK:
		val = DUAL_ROLE_PROP_VCONN_SUPPLY_NO;
		role = PD_ROLE_VCONN_OFF;
		break;
	case TYPEC_SOURCE:
		val = DUAL_ROLE_PROP_VCONN_SUPPLY_YES;
		role = PD_ROLE_VCONN_ON;
		break;
	default:
		return 0;
	}

	if (val == tcpc_dev->dual_role_vconn) {
		pr_info("%s wrong role (%d->%d)\n",
			__func__, tcpc_dev->dual_role_vconn, val);
		return 0;
	}

	ret = tcpm_dpm_pd_vconn_swap(tcpc_dev, role, NULL);
	pr_info("%s vconn swap (%d->%d): %d\n",
		__func__, tcpc_dev->dual_role_vconn, val, ret);

	return ret;
}

static int tcpm_port_type_set(struct typec_port *port,
				enum typec_port_type type)
{
	uint8_t role;

	switch (type) {
	case TYPEC_PORT_SNK:
		role = TYPEC_ROLE_SNK;
		break;
	case TYPEC_PORT_SRC:
		role = TYPEC_ROLE_SRC;
		break;
	case TYPEC_PORT_DRP:
		role = TYPEC_ROLE_DRP;
		break;
	default:
		return 0;
	}

	return tcpm_typec_change_role(tcpc_dev, role);
}

static int tcpm_try_role(struct typec_port *port, int role)
{
	if (role != TYPEC_ROLE_TRY_SRC && role != TYPEC_ROLE_TRY_SNK)
		return 0;

	return tcpm_typec_change_role(tcpc_dev, role);
}

static const struct typec_operations typec_ops = {
	.dr_set = tcpc_dual_role_set_prop_dr,
	.pr_set = tcpc_dual_role_set_prop_pr,
	.vconn_set = tcpc_dual_role_set_prop_vconn,
	.try_role = tcpm_try_role,
	.port_type_set = tcpm_port_type_set,
};

int tcpc_dual_role_phy_init(struct tcpc_device *tcpc)
{
	int err;

	tcpc->typec_caps.revision = 0x0120;	/* Type-C spec release 1.2 */
	tcpc->typec_caps.pd_revision = 0x0300;	/* USB-PD spec release 3.0 */
	tcpc->typec_caps.type = TYPEC_PORT_DRP;
	tcpc->typec_caps.data = TYPEC_PORT_DRD;
	tcpc->typec_caps.prefer_role = TYPEC_SINK;
	tcpc->typec_caps.ops = &typec_ops;
	tcpc_dev = tcpc;
	tcpc->typec_port = typec_register_port(&tcpc->dev, &tcpc->typec_caps);
	if (IS_ERR(tcpc->typec_port)) {
		err = PTR_ERR(tcpc->typec_port);
		return -EINVAL;
	}

	return 0;
}
#endif /* CONFIG_DUAL_ROLE_USB_INTF */
