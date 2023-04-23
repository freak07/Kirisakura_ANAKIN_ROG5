// SPDX-License-Identifier: GPL-2.0-only
/*
 * kernel/power/suspend.c - Suspend to RAM and standby functionality.
 *
 * Copyright (c) 2020 Asus
 */

#ifndef _LINUX_PM_DEBUG_H
#define _LINUX_PM_DEBUG_H
#include <linux/timer.h>

//[PM_debug +++]
#define PM_UNATTENDED_TIMEOUT   1000*60*10		//10min
/*#define PM_UNATTENDED_TIMEOUT   1000*20		//20s*/

extern void __pm_printk(const char *fmt, ...);
#define pm_printk(fmt, ...) \
	do {								\
		__pm_printk(fmt, __FILE__, __func__, __LINE__, ##__VA_ARGS__);\
	} while (0)
    
extern struct timer_list unattended_timer;
extern void unattended_timer_expired(struct timer_list *unused);
extern int pm_stay_unattended_period;
extern void del_pm_timer(void);
extern void mod_pm_timer(void);
//[PM_debug ---]
#endif /* _LINUX_PM_DEBUG_H */
