/*****************************************************************************
* File: debug.h
*
* (c) 2016 Sentons Inc. - All Rights Reserved.
*
* All infmtion contained herein is and remains the property of Sentons
* Incorporated and its suppliers if any. The intellectual and technical
* concepts contained herein are proprietary to Sentons Incorporated and its
* suppliers and may be covered by U.S. and Foreign Patents, patents in
* process, and are protected by trade secret or copyright law. Dissemination
* of this information or reproduction of this material is strictly forbidden
* unless prior written permission is obtained from Sentons Incorporated.
*
* SENTONS PROVIDES THIS SOURCE CODE STRICTLY ON AN "AS IS" BASIS,
* WITHOUT ANY WARRANTY WHATSOEVER, AND EXPRESSLY DISCLAIMS ALL
* WARRANTIES, EXPRESS, IMPLIED OR STATUTORY WITH REGARD THERETO, INCLUDING
* THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
* PURPOSE, TITLE OR NON-INFRINGEMENT OF THIRD PARTY RIGHTS. SENTONS SHALL
* NOT BE LIABLE FOR ANY DAMAGES SUFFERED BY YOU AS A RESULT OF USING,
* MODIFYING OR DISTRIBUTING THIS SOFTWARE OR ITS DERIVATIVES.
*
*
*****************************************************************************/
#include <linux/kernel.h>

#ifndef DEBUG_H
#define DEBUG_H

enum _debug_error {
    E_DONTCARE= -3,
    E_ALREADY = -2,
    E_FINISHED= -1,
    E_SUCCESS = 0,
    E_FAILURE = 1,
    E_BADOPEN = 12,
    E_BADREAD = 13,
    E_BADWRITE= 14,
    E_BADSIZE = 25,
    E_TOOFEW  = 36,
    E_TOOMANY = 37,
    E_TIMEOUT = 99   //IRQ Timeout
};

typedef enum _debug_error error_t;

// This TAG string is displayed in every printk() based macro below
#define TAG "[EDGE] snt8100fsr: "

// Predefined debug strings
#define OOM_STRING "OUT OF MEMORY"

// Prefix and Postfix for function begin and return messages
#define FB_PRE  "%s("
#define FB_POST ")\n"
#define FR_PRE  "%s => "
#define FR_POST "\n"

// Prefix and Postfix string for all normal print statements
#define P_PRE "%s: "
#define P_POST "\n"

// Function print statement debug level
#define PRINT_FUNC_LVL KERN_DEBUG

// Helper Strings
#define EMERG_STR   "!!! EMERG !!!: "
#define ALERT_STR   "!!! ALERT!!! : "
#define CRIT_STR    "!!! CRIT !!!: "
#define ERR_STR     "!!! ERROR !!!: "
#define WARN_STR    "WARN: "
#define NOTICE_STR  "NOTE: "
#define INFO_STR    "INFO: "
#define DEBUG_STR   "DBUG: "
#define FUNC_STR    "FUNC: "

#define MODULE_NAME			"ASH_ATTR"
#define SENSOR_TYPE_NAME		"Grip"
// Helper macro to select the correct print macro
#define GET_MACRO(_1,_2,_3,_4,_5,_6,_7,_8,NAME,...) NAME

// Print macros
#define PRINT_EMERG_A7(fmt, a1, a2, a3, a4, a5, a6, a7) printk(KERN_EMERG TAG EMERG_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5, a6, a7)
#define PRINT_EMERG_A6(fmt, a1, a2, a3, a4, a5, a6) printk(KERN_EMERG TAG EMERG_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5, a6)
#define PRINT_EMERG_A5(fmt, a1, a2, a3, a4, a5)     printk(KERN_EMERG TAG EMERG_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5)
#define PRINT_EMERG_A4(fmt, a1, a2, a3, a4)         printk(KERN_EMERG TAG EMERG_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4)
#define PRINT_EMERG_A3(fmt, a1, a2, a3)             printk(KERN_EMERG TAG EMERG_STR P_PRE fmt P_POST, __func__, a1, a2, a3)
#define PRINT_EMERG_A2(fmt, a1, a2)                 printk(KERN_EMERG TAG EMERG_STR P_PRE fmt P_POST, __func__, a1, a2)
#define PRINT_EMERG_A1(fmt, a1)                     printk(KERN_EMERG TAG EMERG_STR P_PRE fmt P_POST, __func__, a1)
#define PRINT_EMERG_A0(fmt)                         printk(KERN_EMERG TAG EMERG_STR P_PRE fmt P_POST, __func__)
#define PRINT_EMERG(...) GET_MACRO(__VA_ARGS__, PRINT_EMERG_A7, PRINT_EMERG_A6, PRINT_EMERG_A5, PRINT_EMERG_A4, PRINT_EMERG_A3, PRINT_EMERG_A2, PRINT_EMERG_A1, PRINT_EMERG_A0)(__VA_ARGS__)

#define PRINT_ALERT_A7(fmt, a1, a2, a3, a4, a5, a6, a7) printk(KERN_ALERT TAG ALERT_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5, a6, a7)
#define PRINT_ALERT_A6(fmt, a1, a2, a3, a4, a5, a6) printk(KERN_ALERT TAG ALERT_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5, a6)
#define PRINT_ALERT_A5(fmt, a1, a2, a3, a4, a5)     printk(KERN_ALERT TAG ALERT_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5)
#define PRINT_ALERT_A4(fmt, a1, a2, a3, a4)         printk(KERN_ALERT TAG ALERT_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4)
#define PRINT_ALERT_A3(fmt, a1, a2, a3)             printk(KERN_ALERT TAG ALERT_STR P_PRE fmt P_POST, __func__, a1, a2, a3)
#define PRINT_ALERT_A2(fmt, a1, a2)                 printk(KERN_ALERT TAG ALERT_STR P_PRE fmt P_POST, __func__, a1, a2)
#define PRINT_ALERT_A1(fmt, a1)                     printk(KERN_ALERT TAG ALERT_STR P_PRE fmt P_POST, __func__, a1)
#define PRINT_ALERT_A0(fmt)                         printk(KERN_ALERT TAG ALERT_STR P_PRE fmt P_POST, __func__)
#define PRINT_ALERT(...) GET_MACRO(__VA_ARGS__, PRINT_ALERT_A7, PRINT_ALERT_A6, PRINT_ALERT_A5, PRINT_ALERT_A4, PRINT_ALERT_A3, PRINT_ALERT_A2, PRINT_ALERT_A1, PRINT_ALERT_A0)(__VA_ARGS__)

#define PRINT_CRIT_A7(fmt, a1, a2, a3, a4, a5, a6, a7) printk(KERN_CRIT TAG CRIT_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5, a6, a7)
#define PRINT_CRIT_A6(fmt, a1, a2, a3, a4, a5, a6) printk(KERN_CRIT TAG CRIT_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5, a6)
#define PRINT_CRIT_A5(fmt, a1, a2, a3, a4, a5)     printk(KERN_CRIT TAG CRIT_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5)
#define PRINT_CRIT_A4(fmt, a1, a2, a3, a4)         printk(KERN_CRIT TAG CRIT_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4)
#define PRINT_CRIT_A3(fmt, a1, a2, a3)             printk(KERN_CRIT TAG CRIT_STR P_PRE fmt P_POST, __func__, a1, a2, a3)
#define PRINT_CRIT_A2(fmt, a1, a2)                 printk(KERN_CRIT TAG CRIT_STR P_PRE fmt P_POST, __func__, a1, a2)
#define PRINT_CRIT_A1(fmt, a1)                     printk(KERN_CRIT TAG CRIT_STR P_PRE fmt P_POST, __func__, a1)
#define PRINT_CRIT_A0(fmt)                         printk(KERN_CRIT TAG CRIT_STR P_PRE fmt P_POST, __func__)
#define PRINT_CRIT(...) GET_MACRO(__VA_ARGS__, PRINT_CRIT_A7, PRINT_CRIT_A6, PRINT_CRIT_A5, PRINT_CRIT_A4, PRINT_CRIT_A3, PRINT_CRIT_A2, PRINT_CRIT_A1, PRINT_CRIT_A0)(__VA_ARGS__)

#define PRINT_ERR_A7(fmt, a1, a2, a3, a4, a5, a6) printk(KERN_ERR TAG ERR_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5, a6, a7)
#define PRINT_ERR_A6(fmt, a1, a2, a3, a4, a5, a6) printk(KERN_ERR TAG ERR_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5, a6)
#define PRINT_ERR_A5(fmt, a1, a2, a3, a4, a5)     printk(KERN_ERR TAG ERR_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5)
#define PRINT_ERR_A4(fmt, a1, a2, a3, a4)         printk(KERN_ERR TAG ERR_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4)
#define PRINT_ERR_A3(fmt, a1, a2, a3)             printk(KERN_ERR TAG ERR_STR P_PRE fmt P_POST, __func__, a1, a2, a3)
#define PRINT_ERR_A2(fmt, a1, a2)                 printk(KERN_ERR TAG ERR_STR P_PRE fmt P_POST, __func__, a1, a2)
#define PRINT_ERR_A1(fmt, a1)                     printk(KERN_ERR TAG ERR_STR P_PRE fmt P_POST, __func__, a1)
#define PRINT_ERR_A0(fmt)                         printk(KERN_ERR TAG ERR_STR P_PRE fmt P_POST, __func__)
#define PRINT_ERR(...) GET_MACRO(__VA_ARGS__, PRINT_ERR_A7, PRINT_ERR_A6, PRINT_ERR_A5, PRINT_ERR_A4, PRINT_ERR_A3, PRINT_ERR_A2, PRINT_ERR_A1, PRINT_ERR_A0)(__VA_ARGS__)

#define PRINT_WARN_A7(fmt, a1, a2, a3, a4, a5, a6, a7) printk(KERN_WARNING TAG WARN_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5, a6, a7)
#define PRINT_WARN_A6(fmt, a1, a2, a3, a4, a5, a6) printk(KERN_WARNING TAG WARN_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5, a6)
#define PRINT_WARN_A5(fmt, a1, a2, a3, a4, a5)     printk(KERN_WARNING TAG WARN_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5)
#define PRINT_WARN_A4(fmt, a1, a2, a3, a4)         printk(KERN_WARNING TAG WARN_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4)
#define PRINT_WARN_A3(fmt, a1, a2, a3)             printk(KERN_WARNING TAG WARN_STR P_PRE fmt P_POST, __func__, a1, a2, a3)
#define PRINT_WARN_A2(fmt, a1, a2)                 printk(KERN_WARNING TAG WARN_STR P_PRE fmt P_POST, __func__, a1, a2)
#define PRINT_WARN_A1(fmt, a1)                     printk(KERN_WARNING TAG WARN_STR P_PRE fmt P_POST, __func__, a1)
#define PRINT_WARN_A0(fmt)                         printk(KERN_WARNING TAG WARN_STR P_PRE fmt P_POST, __func__)
#define PRINT_WARN(...) GET_MACRO(__VA_ARGS__, PRINT_WARN_A7, PRINT_WARN_A6, PRINT_WARN_A5, PRINT_WARN_A4, PRINT_WARN_A3, PRINT_WARN_A2, PRINT_WARN_A1, PRINT_WARN_A0)(__VA_ARGS__)

#define PRINT_NOTICE_A7(fmt, a1, a2, a3, a4, a5, a6, a7) printk(KERN_NOTICE TAG NOTICE_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5, a6, a7)
#define PRINT_NOTICE_A6(fmt, a1, a2, a3, a4, a5, a6) printk(KERN_NOTICE TAG NOTICE_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5, a6)
#define PRINT_NOTICE_A5(fmt, a1, a2, a3, a4, a5)     printk(KERN_NOTICE TAG NOTICE_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5)
#define PRINT_NOTICE_A4(fmt, a1, a2, a3, a4)         printk(KERN_NOTICE TAG NOTICE_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4)
#define PRINT_NOTICE_A3(fmt, a1, a2, a3)             printk(KERN_NOTICE TAG NOTICE_STR P_PRE fmt P_POST, __func__, a1, a2, a3)
#define PRINT_NOTICE_A2(fmt, a1, a2)                 printk(KERN_NOTICE TAG NOTICE_STR P_PRE fmt P_POST, __func__, a1, a2)
#define PRINT_NOTICE_A1(fmt, a1)                     printk(KERN_NOTICE TAG NOTICE_STR P_PRE fmt P_POST, __func__, a1)
#define PRINT_NOTICE_A0(fmt)                         printk(KERN_NOTICE TAG NOTICE_STR P_PRE fmt P_POST, __func__)
#define PRINT_NOTICE(...) GET_MACRO(__VA_ARGS__, PRINT_NOTICE_A7, PRINT_NOTICE_A6, PRINT_NOTICE_A5, PRINT_NOTICE_A4, PRINT_NOTICE_A3, PRINT_NOTICE_A2, PRINT_NOTICE_A1, PRINT_NOTICE_A0)(__VA_ARGS__)

#define PRINT_INFO_A7(fmt, a1, a2, a3, a4, a5, a6, a7) printk(KERN_INFO TAG INFO_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5, a6, a7)
#define PRINT_INFO_A6(fmt, a1, a2, a3, a4, a5, a6) printk(KERN_INFO TAG INFO_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5, a6)
#define PRINT_INFO_A5(fmt, a1, a2, a3, a4, a5)     printk(KERN_INFO TAG INFO_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5)
#define PRINT_INFO_A4(fmt, a1, a2, a3, a4)         printk(KERN_INFO TAG INFO_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4)
#define PRINT_INFO_A3(fmt, a1, a2, a3)             printk(KERN_INFO TAG INFO_STR P_PRE fmt P_POST, __func__, a1, a2, a3)
#define PRINT_INFO_A2(fmt, a1, a2)                 printk(KERN_INFO TAG INFO_STR P_PRE fmt P_POST, __func__, a1, a2)
#define PRINT_INFO_A1(fmt, a1)                     printk(KERN_INFO TAG INFO_STR P_PRE fmt P_POST, __func__, a1)
#define PRINT_INFO_A0(fmt)                         printk(KERN_INFO TAG INFO_STR P_PRE fmt P_POST, __func__)
#define PRINT_INFO(...) GET_MACRO(__VA_ARGS__, PRINT_INFO_A7, PRINT_INFO_A6, PRINT_INFO_A5, PRINT_INFO_A4, PRINT_INFO_A3, PRINT_INFO_A2, PRINT_INFO_A1, PRINT_INFO_A0)(__VA_ARGS__)

#ifndef SNT_DEBUG
//#define SNT_DEBUG
#endif

#ifdef SNT_DEBUG
#define PRINT_DEBUG_A7(fmt, a1, a2, a3, a4, a5, a6, a7) printk(KERN_DEBUG TAG DEBUG_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5, a6, a7)
#define PRINT_DEBUG_A6(fmt, a1, a2, a3, a4, a5, a6) printk(KERN_DEBUG TAG DEBUG_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5, a6)
#define PRINT_DEBUG_A5(fmt, a1, a2, a3, a4, a5)     printk(KERN_DEBUG TAG DEBUG_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4, a5)
#define PRINT_DEBUG_A4(fmt, a1, a2, a3, a4)         printk(KERN_DEBUG TAG DEBUG_STR P_PRE fmt P_POST, __func__, a1, a2, a3, a4)
#define PRINT_DEBUG_A3(fmt, a1, a2, a3)             printk(KERN_DEBUG TAG DEBUG_STR P_PRE fmt P_POST, __func__, a1, a2, a3)
#define PRINT_DEBUG_A2(fmt, a1, a2)                 printk(KERN_DEBUG TAG DEBUG_STR P_PRE fmt P_POST, __func__, a1, a2)
#define PRINT_DEBUG_A1(fmt, a1)                     printk(KERN_DEBUG TAG DEBUG_STR P_PRE fmt P_POST, __func__, a1)
#define PRINT_DEBUG_A0(fmt)                         printk(KERN_DEBUG TAG DEBUG_STR P_PRE fmt P_POST, __func__)
#define PRINT_DEBUG(...) GET_MACRO(__VA_ARGS__, PRINT_DEBUG_A7, PRINT_DEBUG_A6, PRINT_DEBUG_A5, PRINT_DEBUG_A4, PRINT_DEBUG_A3, PRINT_DEBUG_A2, PRINT_DEBUG_A1, PRINT_DEBUG_A0)(__VA_ARGS__)

#define PRINT_FUNC_A7(fmt, a1, a2, a3, a4, a5, a6, a7) printk(PRINT_FUNC_LVL TAG FUNC_STR FB_PRE fmt FB_POST, __func__, a1, a2, a3, a4, a5, a6, a7)
#define PRINT_FUNC_A6(fmt, a1, a2, a3, a4, a5, a6) printk(PRINT_FUNC_LVL TAG FUNC_STR FB_PRE fmt FB_POST, __func__, a1, a2, a3, a4, a5, a6)
#define PRINT_FUNC_A5(fmt, a1, a2, a3, a4, a5)     printk(PRINT_FUNC_LVL TAG FUNC_STR FB_PRE fmt FB_POST, __func__, a1, a2, a3, a4, a5)
#define PRINT_FUNC_A4(fmt, a1, a2, a3, a4)         printk(PRINT_FUNC_LVL TAG FUNC_STR FB_PRE fmt FB_POST, __func__, a1, a2, a3, a4)
#define PRINT_FUNC_A3(fmt, a1, a2, a3)             printk(PRINT_FUNC_LVL TAG FUNC_STR FB_PRE fmt FB_POST, __func__, a1, a2, a3)
#define PRINT_FUNC_A2(fmt, a1, a2)                 printk(PRINT_FUNC_LVL TAG FUNC_STR FB_PRE fmt FB_POST, __func__, a1, a2)
#define PRINT_FUNC_A1(fmt, a1)                     printk(PRINT_FUNC_LVL TAG FUNC_STR FB_PRE fmt FB_POST, __func__, a1)
#define PRINT_FUNC_A0(fmt)                         printk(PRINT_FUNC_LVL TAG FUNC_STR FB_PRE fmt FB_POST, __func__)
#define PRINT_FUNC(...) GET_MACRO(__VA_ARGS__, PRINT_FUNC_A7, PRINT_FUNC_A6, PRINT_FUNC_A5, PRINT_FUNC_A4, PRINT_FUNC_A3, PRINT_FUNC_A2, PRINT_FUNC_A1, PRINT_FUNC_A0)(__VA_ARGS__)
#else
#define PRINT_DEBUG(stuff...)	pr_debug("snt8100fsr: " stuff)
#define PRINT_FUNC(...) do {} while (0)
#endif
#endif // DEBUG_H

