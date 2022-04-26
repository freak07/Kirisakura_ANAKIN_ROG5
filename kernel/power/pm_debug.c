// SPDX-License-Identifier: GPL-2.0-only
/*
 * kernel/power/suspend.c - Suspend to RAM and standby functionality.
 *
 * Copyright (c) 2020 Asus
 */

#define pr_fmt(fmt) "PM: " fmt

#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/cpu.h>
#include <linux/cpuidle.h>
#include <linux/gfp.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <linux/swait.h>
#include <linux/ftrace.h>
#include <trace/events/power.h>
#include <linux/compiler.h>
#include <linux/moduleparam.h>
#include <linux/wakeup_reason.h>
#include <linux/jiffies.h>

#include "power.h"

//[PM_debug +++]
#include <linux/pm_debug.h>

int pm_stay_unattended_period = 0;
int pmsp_flag = 0;
extern void asus_uts_print_active_locks(void);
void unattended_timer_expired(struct timer_list *unused);
//[PM_debug ---]

//[PM_debug +++]
//for split file path
static int pm_file_path(char *str, const char *delim, char dst[][80]) {
    char *s = kstrdup(str, GFP_KERNEL);  
    char *token;  
    int n = 0;
    for(token = strsep(&s, delim); token != NULL; token = strsep(&s, delim)) {  
        strcpy(dst[n++], token);
    }  
    return n;
}
// suspend.h: pm_printk()
//control length of path, if debug_path_length == 0, don't print path and line info
int debug_path_length = 4; 
//int debug_path_length = 0; 
void __pm_printk(const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;
	int num = 0, line = 0;
    char concated[100];
    char delim[] = "/";  
    char dst[20][80];
    char *file=NULL, *func=NULL;
    int i = 0;

    memset(concated, '\0', 100);
	va_start(args, fmt);
    vaf.fmt = fmt;
    file = va_arg(args, char*); //get __FILE__
    func = va_arg(args, char*); //get __func__
    line = va_arg(args, int); //get __LINE__
    //if debug_path_length == 0, don't print path and line info
    if(debug_path_length){ 
        num = pm_file_path(file, delim, dst);
        strcat(concated, dst[num-debug_path_length]);
        for(i=(debug_path_length-1);i>=1;i--){
            strcat(concated, "/");
            strcat(concated, dst[num-i]);
        }
    }
    vaf.va = &args;
    if(debug_path_length)
        printk(KERN_DEBUG "[PM]%s:%s():%d:%pV", concated, func, line, &vaf);
    else //if path_depth == 0, don't print path and line info
        printk(KERN_DEBUG "[PM]%s():%pV", func, &vaf);
	va_end(args);
}
//unattended_timer
DEFINE_TIMER(unattended_timer, unattended_timer_expired);
EXPORT_SYMBOL_GPL(unattended_timer);

void unattended_timer_expired(struct timer_list *unused)
{
	//printk("[PM]unattended_timer_expired\n");
    pm_printk("[PM]unattended_timer_expired\n");
#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT	
	ASUSEvtlog("[PM]unattended_timer_expired\n");
#endif	
	pm_stay_unattended_period += PM_UNATTENDED_TIMEOUT;
	pmsp_flag = 1;
	asus_uts_print_active_locks();
	mod_timer(&unattended_timer, jiffies + msecs_to_jiffies(PM_UNATTENDED_TIMEOUT));
}
EXPORT_SYMBOL_GPL(unattended_timer_expired);

void del_pm_timer(){
	del_timer(&unattended_timer);
	pm_stay_unattended_period = 0;
}
EXPORT_SYMBOL_GPL(del_pm_timer);
void mod_pm_timer(){
    mod_timer(&unattended_timer,
						jiffies + msecs_to_jiffies(PM_UNATTENDED_TIMEOUT));
}
EXPORT_SYMBOL_GPL(mod_pm_timer);                       
//[PM_debug ---]
