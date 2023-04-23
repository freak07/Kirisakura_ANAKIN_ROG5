/*****************************************************************************
* File: irq.h
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
#include <linux/interrupt.h>

#ifndef IRQ_H
#define IRQ_H

typedef struct irq_db_s 
{
    irq_handler_t       top;
    irq_handler_t       bottom;
    unsigned long       flags; 
    char *              name;
    unsigned int        irq_num;
    unsigned int        gpio_num;
    void *              work;
    bool                installed_flag;

} irq_db_t, *p_irq_db_t;

extern irq_db_t snt_irq_db;

#if USE_TRIG0_IRQ
extern irq_db_t trig0_irq_db;
#endif

#if USE_TRIG1_IRQ
extern irq_db_t trig1_irq_db;
#endif

#if USE_TRIG2_IRQ
extern irq_db_t trig2_irq_db;
#endif

#if USE_TRIG3_IRQ
extern irq_db_t trig3_irq_db;
#endif

int irq_handler_register(p_irq_db_t p_db);
void irq_handler_unregister(p_irq_db_t p_db);
bool is_irq_handler_registered(p_irq_db_t p_db);
#endif
