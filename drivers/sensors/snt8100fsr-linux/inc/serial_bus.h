/*****************************************************************************
* File: serial-bus.c
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
#include "device.h"

#ifndef SERIALBUS_H
#define SERIALBUS_H

//=========================================================================
// INCLUDES
//=========================================================================

//=========================================================================
// DEFINES
//=========================================================================
#define SPI_FIFO_SIZE        8
#define I2C_FIFO_SIZE        8

//=========================================================================
// CONSTANTS
//=========================================================================
enum {
  SB_TYPE_NONE = 0,
  SB_TYPE_I2C  = 1,
  SB_TYPE_SPI  = 2,
  SB_TYPE_MAX,
};

//=========================================================================
// FUNCTIONS
//=========================================================================
int sb_get_boot_status(struct snt8100fsr *snt8100fsr);
int sb_wake_device(struct snt8100fsr *snt8100fsr);
int sb_read_and_write(struct snt8100fsr *snt8100fsr,
                      int num_write,
                      uint8_t *data_out,
                      uint8_t *data_in);
int sb_read_register(struct snt8100fsr *snt8100fsr,
                     uint16_t reg,
                     void *in_val);
int sb_write_register(struct snt8100fsr *snt8100fsr,
                      uint16_t reg,
                      void *out_val);
int sb_read_fifo(struct snt8100fsr *snt8100fsr,
                 uint16_t reg,
                 uint16_t len,
                 void *in_val);
int sb_write_fifo(struct snt8100fsr *snt8100fsr,
                  uint16_t reg,
                  uint16_t len,
                  void *out_val);

#endif
