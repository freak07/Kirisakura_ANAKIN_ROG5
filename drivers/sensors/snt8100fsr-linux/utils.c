/*****************************************************************************
* File: utils.c
*
* (c) 2016 Sentons Inc. - All Rights Reserved.
*
* All information contained herein is and remains the property of Sentons
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
*****************************************************************************/
#include <linux/module.h>

#include "config.h"
#include "serial_bus.h"
#include "debug.h"
#include "locking.h"

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/

/*==========================================================================*/
/* LOCAL PROTOTYPES                                                         */
/*==========================================================================*/

/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/

/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/
uint32_t get_time_in_ms(void) {
    struct timespec tv;
    getnstimeofday(&tv);
    return (tv.tv_sec * 1000) + (tv.tv_nsec / 1000000);
}

/*
 * Parses a string in base 10 decimal format or base 16 hexidecimal format
 * and autodetects which format to use based on if the string prefix "0x" is
 * present, then hexidecimal will be used. Returns 0 on success, -1 on error.
 */
int string_to_uint32(const char *str, uint32_t *value) {
    long     tmp;

    if(strncmp(str, "0x", 2) == 0) {
        if(kstrtol(&str[2], 16, &tmp) == 0) {
            *value = (uint32_t)tmp;
            return 0;
        }
    } else {
        if(kstrtol(str, 10, &tmp) == 0) {
            *value = (uint32_t)tmp;
            return 0;
        }
    }

    return -1;
}

extern int grip_write_fail_count;
extern int grip_write_fail_reset_trigger;
int read_register(struct snt8100fsr *snt8100fsr,
                  int reg,
                  void *value) {
    int ret=0;

    //PRINT_FUNC();
    /*ASUS BSP Clay +++: when write fail occurs, prevent the otherm i2c write before reset chip to prevent unnecesary i2c timeout dely*/
	if(grip_write_fail_count>grip_write_fail_reset_trigger){
		PRINT_CRIT("Skip i2c read since rw_fail_count = %d", grip_write_fail_count);
		return -1;
	}
	/*ASUS BSP Clay ---: when write fail occurs, prevent the otherm i2c write before reset chip to prevent unnecesary i2c timeout dely*/


    // Read the event register
    MUTEX_LOCK(&snt8100fsr->sb_lock);
    ret = sb_read_register(snt8100fsr, reg, value);
    if (ret) {
        PRINT_CRIT("sb_read_register() failed");
    } else {
        PRINT_DEBUG("0x%04x == 0x%04X", reg, *((uint16_t *)value));
    }
    mutex_unlock(&snt8100fsr->sb_lock);
    return ret;
}

int write_register(struct snt8100fsr *snt8100fsr,
                   int reg,
                   void *value) {
    int ret=0;

    PRINT_FUNC("0x%04x = 0x%04x", reg, *((uint16_t *)value));

    // Read the event register
    MUTEX_LOCK(&snt8100fsr->sb_lock);
    ret = sb_write_register(snt8100fsr, reg, value);
    if (ret) {
        PRINT_CRIT("sb_write_register() failed");
        mutex_unlock(&snt8100fsr->sb_lock);
        return ret;
    }
    mutex_unlock(&snt8100fsr->sb_lock);
    PRINT_DEBUG("done");
    return ret;
}

int cust_write_registers(void *dev, int reg, int num, void *value) {

    int ret=0;
    struct snt8100fsr *snt8100fsr = (struct snt8100fsr *) dev;

    mutex_lock(&snt8100fsr->sb_lock);
    ret = sb_write_fifo(snt8100fsr, reg, num*2, value);
    if (ret) {
        PRINT_CRIT("cust_write_registers() failed (%d)", ret);
    }
    mutex_unlock(&snt8100fsr->sb_lock);
    return ret;
}

int cust_read_registers(void *dev, int reg, int num, void *value) {

    int ret=0;
    struct snt8100fsr *snt8100fsr = (struct snt8100fsr *) dev;

    mutex_lock(&snt8100fsr->sb_lock);
    ret = sb_read_fifo(snt8100fsr, reg, num*2, value);
    if (ret) {
        PRINT_CRIT("cust_read_registers() failed (%d)", ret);
    } else {
    }
    mutex_unlock(&snt8100fsr->sb_lock);
    return ret;
}

