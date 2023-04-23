/*****************************************************************************
* File: firmware.h
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
#include "serial_bus.h"

#ifndef FIRMWARE_H
#define FIRMWARE_H

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/
#define FIRMWARE_ALREADY_LOADED_RESULT 0xF0

/*
 * Boot Configuration Record
 */
#define BCR_I2CADDR_WIDTH    8
#define BCR_I2CADDR_MASK     ((1<<BCR_I2CADDR_WIDTH)-1)
#define BCR_I2CADDR_POS      24

#define BCR_LOGLVL_WIDTH     3
#define BCR_LOGLVL_MASK      ((1<<BCR_LOGLVL_WIDTH)-1)
#define BCR_LOGLVL_POS       8

#define BCR_IRPTDUR_WIDTH    6
#define BCR_IRPTDUR_MASK     ((1<<BCR_IRPTDUR_WIDTH)-1)
#define BCR_IRPTDUR_POS      2

#define BCR_IRPTLVL_WIDTH    1
#define BCR_IRPTLVL_MASK     ((1<<BCR_IRPTLVL_WIDTH)-1)
#define BCR_IRPTLVL_POS      1

#define BCR_IRPTPOL_WIDTH    1
#define BCR_IRPTPOL_MASK     ((1<<BCR_IRPTPOL_WIDTH)-1)
#define BCR_IRPTPOL_POS      0

#define BCR_IRPTCFG_WIDTH    8
#define BCR_IRPTCFG_MASK     ((1<<BCR_IRPTCFG_WIDTH)-1)
#define BCR_IRPTCFG_POS      0

/*
 * Boot Configuration Record Logging level
 */
#define BCR_LOGLVL_DEFAULT   0
#define BCR_LOGLVL_INFO      1
#define BCR_LOGLVL_DIAG      2
#define BCR_LOGLVL_WARN      3
#define BCR_LOGLVL_ERROR     4
#define BCR_LOGLVL_FATAL     5
#define BCR_LOGLVL_OFF       6

/*
 * Boot Configuration Record Interrupt Polarity levels
 */
#define BCR_IRQ_POLARITY_ACTIVE_HIGH    0   /* default */
#define BCR_IRQ_POLARITY_ACTIVE_LOW     1 

/*
 * Boot Configuration Record Types
 */
#define BCR_IRQ_TYPE_LEVEL              0   /* default */
#define BCR_IRQ_TYPE_PULSE              1

/*==========================================================================*/
/* PROTOTYPES                                                               */
/*==========================================================================*/
int upload_firmware(struct snt8100fsr *snt8100fsr, char *filename);
int upload_firmware_fwd(struct snt8100fsr *snt8100fsr, char *filename);
int irq_handler_fwd( void);

#endif
