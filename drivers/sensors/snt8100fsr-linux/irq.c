/*****************************************************************************
* File: irq.c
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
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <asm/irq.h>
#include <asm/io.h>

#include "config.h"
#include "irq.h"
#include "debug.h"

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

struct irq_db_s snt_irq_db = {
    NULL,               /* top              */
    NULL,               /* bottom           */
    IRQ_TRIGGER,        /* flags            */
    IRQ_NAME,           /* name             */
    0,                  /* irq_num          */
    IRQ_GPIO,           /* gpio_num         */
    NULL,               /* work             */
    false               /* installed_flag   */

};

#if USE_TRIG0_IRQ
struct irq_db_s trig0_irq_db = {
    NULL,               /* top              */
    NULL,               /* bottom           */
    TRIG0_TRIGGER,      /* flags            */
    TRIG0_NAME,         /* name             */
    0,                  /* irq_num          */
    TRIG0_GPIO,         /* gpio_num         */
    NULL,               /* work             */
    false               /* installed_flag   */

};
#endif

#if USE_TRIG1_IRQ
struct irq_db_s trig1_irq_db = {
    NULL,               /* top              */
    NULL,               /* bottom           */
    TRIG1_TRIGGER,      /* flags            */
    TRIG1_NAME,         /* name             */
    0,                  /* irq_num          */
    TRIG1_GPIO,         /* gpio_num         */
    NULL,               /* work             */
    false               /* installed_flag   */

};
#endif

#if USE_TRIG2_IRQ
struct irq_db_s trig2_irq_db = {
    NULL,               /* top              */
    NULL,               /* bottom           */
    TRIG2_TRIGGER,      /* flags            */
    TRIG2_NAME,         /* name             */
    0,                  /* irq_num          */
    TRIG2_GPIO,         /* gpio_num         */
    NULL,               /* work             */
    false               /* installed_flag   */

};
#endif

#if USE_TRIG3_IRQ
struct irq_db_s trig3_irq_db = {
    NULL,               /* top              */
    NULL,               /* bottom           */
    TRIG3_TRIGGER,      /* flags            */
    TRIG3_NAME,         /* name             */
    0,                  /* irq_num          */
    TRIG3_GPIO,         /* gpio_num         */
    NULL,               /* work             */
    false               /* installed_flag   */

};
#endif

/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/
int irq_handler_register(p_irq_db_t p_db) {
    unsigned int flags;
    int ret;

    PRINT_FUNC();

    if (p_db == NULL) {
        PRINT_CRIT("NULL IRQ data block");
        return -EIO;
    }
    flags = p_db->flags;

    if (p_db->installed_flag) {
        PRINT_CRIT("IRQ handler already registered");
        return -EIO;
    }

    if (p_db->bottom) {
        flags |= IRQF_ONESHOT;
    }
	
    PRINT_DEBUG("IRQ_NUMBER: %d", p_db->irq_num);
    ret = request_threaded_irq(p_db->irq_num,
                               p_db->top,
                               NULL,
                               flags,
                               p_db->name,
                               p_db);
    if (ret) {
        PRINT_CRIT("GPIO %d for IRQ %d already claimed or allocation failed!",
                   p_db->gpio_num, p_db->irq_num);
        return -EIO;
    }

    p_db->installed_flag = true;
    PRINT_DEBUG("Done registering IRQ %d handler", p_db->irq_num);
    return 0;
}

void irq_handler_unregister(p_irq_db_t p_db) {
    PRINT_FUNC();

    if (p_db == NULL) {
        PRINT_DEBUG("NULL IRQ data block");
        return;
    }

    if (p_db->installed_flag) {
        free_irq(p_db->irq_num, p_db);
        PRINT_DEBUG("Free'd IRQ %d", p_db->irq_num);
    }

    p_db->installed_flag = false;

    PRINT_DEBUG("done");
}

bool is_irq_handler_registered( p_irq_db_t p_db ) {
    if (p_db == NULL) return false;
    return p_db->installed_flag;
}
