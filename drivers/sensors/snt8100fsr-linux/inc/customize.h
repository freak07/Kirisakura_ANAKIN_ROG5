/*****************************************************************************
* File: customize.h
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
*
*
*****************************************************************************/
#include <linux/input.h>
#include "track_report.h"

#ifndef CUSTOMIZE_H
#define CUSTOMIZE_H

void register_input_events(struct input_dev *input_dev);
void process_track_reports(uint16_t frame,
                           struct track_report *tr,
                           size_t count);
void process_trigger(int trig_id, int gpio_value);

extern uint16_t TAP_BIT0[];
extern uint16_t TAP_BIT1[];
extern uint16_t TAP_BIT2[];
extern uint16_t TAP_BIT3[];
extern uint16_t TAP_BIT4[];
extern uint16_t TAP_BIT5[];
extern uint16_t TAP_BIT6[];
extern uint16_t TAP_BIT7[];

extern uint16_t SQ_BIT0[];
extern uint16_t SQ_BIT1[];
extern uint16_t SQ_BIT2[];
extern uint16_t SQ_BIT3[];
extern uint16_t SQ_BIT4[];
extern uint16_t SQ_BIT5[];
extern uint16_t SQ_BIT6[];
extern uint16_t SQ_BIT7[];
extern uint16_t SQ_BIT8[];

extern uint16_t SLIDE_BIT0[];
extern uint16_t SLIDE_BIT1[];
extern uint16_t SLIDE_BIT2[];
extern uint16_t SLIDE_BIT3[];
extern uint16_t SLIDE_BIT4[];
extern uint16_t SLIDE_BIT5[];

extern uint16_t SWIPE_BIT0[];
extern uint16_t SWIPE_BIT1[];
extern uint16_t SWIPE_BIT2[];
extern uint16_t SWIPE_BIT3[];
extern uint16_t SWIPE_BIT4[];

extern uint16_t TAP0_BIT0;
extern uint16_t TAP0_BIT1;
extern uint16_t TAP0_BIT2;
extern uint16_t TAP0_BIT3;
extern uint16_t TAP0_BIT4;
extern uint16_t TAP0_BIT5;
extern uint16_t TAP0_BIT6;
extern uint16_t TAP0_BIT7;

extern uint16_t TAP1_BIT0;
extern uint16_t TAP1_BIT1;
extern uint16_t TAP1_BIT2;
extern uint16_t TAP1_BIT3;
extern uint16_t TAP1_BIT4;
extern uint16_t TAP1_BIT5;
extern uint16_t TAP1_BIT6;
extern uint16_t TAP1_BIT7;

extern uint16_t TAP2_BIT0;
extern uint16_t TAP2_BIT1;
extern uint16_t TAP2_BIT2;
extern uint16_t TAP2_BIT3;
extern uint16_t TAP2_BIT4;
extern uint16_t TAP2_BIT5;
extern uint16_t TAP2_BIT6;
extern uint16_t TAP2_BIT7;

extern uint16_t TAP3_BIT0;
extern uint16_t TAP3_BIT1;
extern uint16_t TAP3_BIT2;
extern uint16_t TAP3_BIT3;
extern uint16_t TAP3_BIT4;
extern uint16_t TAP3_BIT5;
extern uint16_t TAP3_BIT6;
extern uint16_t TAP3_BIT7;

extern uint16_t SQ1_BIT0;
extern uint16_t SQ1_BIT1;
extern uint16_t SQ1_BIT2;
extern uint16_t SQ1_BIT3;
extern uint16_t SQ1_BIT4;
extern uint16_t SQ1_BIT5;
extern uint16_t SQ1_BIT6;
extern uint16_t SQ1_BIT7;
extern uint16_t SQ1_BIT8;

extern uint16_t SQ2_BIT0;
extern uint16_t SQ2_BIT1;
extern uint16_t SQ2_BIT2;
extern uint16_t SQ2_BIT3;
extern uint16_t SQ2_BIT4;
extern uint16_t SQ2_BIT5;
extern uint16_t SQ2_BIT6;
extern uint16_t SQ2_BIT7;
extern uint16_t SQ2_BIT8;

extern uint16_t SLIDE0_BIT0;
extern uint16_t SLIDE0_BIT1;
extern uint16_t SLIDE0_BIT2;
extern uint16_t SLIDE0_BIT3;
extern uint16_t SLIDE0_BIT4;
extern uint16_t SLIDE0_BIT5;
extern uint16_t SLIDE0_BIT6;

extern uint16_t SLIDE1_BIT0;
extern uint16_t SLIDE1_BIT1;
extern uint16_t SLIDE1_BIT2;
extern uint16_t SLIDE1_BIT3;
extern uint16_t SLIDE1_BIT4;
extern uint16_t SLIDE1_BIT5;
extern uint16_t SLIDE1_BIT6;

extern uint16_t SWIPE1_BIT0;
extern uint16_t SWIPE1_BIT1;
extern uint16_t SWIPE1_BIT2;
extern uint16_t SWIPE1_BIT3;
extern uint16_t SWIPE1_BIT4;

extern uint16_t SWIPE2_BIT0;
extern uint16_t SWIPE2_BIT1;
extern uint16_t SWIPE2_BIT2;
extern uint16_t SWIPE2_BIT3;
extern uint16_t SWIPE2_BIT4;

#endif // CUSTOMIZE_H
void set_operational_profile(void *dev, int p);

