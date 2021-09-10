// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2011, 2014-2016, 2018, 2020, The Linux Foundation. All rights reserved.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

//[PM_debug +++]
//int msm_show_resume_irq_mask;
/*Mask IRQ triggered number during gic_show_resume_irq() in kernel/drivers/irqchip/irq-gic-v3.c */
int msm_show_resume_irq_mask = 1; 
//[PM_debug ---]

module_param_named(
	debug_mask, msm_show_resume_irq_mask, int, 0664);
