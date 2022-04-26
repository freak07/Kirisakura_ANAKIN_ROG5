/*****************************************************************************
* File: i2c-bus.h
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
#include <linux/kernel.h>

#ifndef I2C_H
#define I2C_H

int snt_i2c_open(void *dev);
void snt_i2c_close(struct snt8100fsr *snt8100fsr);
uint8_t snt_i2c_get_boot_status(struct snt8100fsr *snt8100fsr);
int snt_i2c_wake_device(struct snt8100fsr *snt8100fsr);

int snt_i2c_read(struct snt8100fsr *snt8100fsr,
                 int num_read,
                 uint8_t *data_in);

int snt_i2c_write(struct snt8100fsr *snt8100fsr,
                  int num_write,
                  uint8_t *data_out);

int snt_i2c_read_register(struct snt8100fsr *snt8100fsr,
                          uint16_t reg,
                          uint16_t *in_val);
int snt_i2c_write_register(struct snt8100fsr *snt8100fsr,
                           uint16_t reg,
                           uint16_t out_val);
int snt_i2c_read_fifo(struct snt8100fsr *snt8100fsr,
                      uint16_t reg,
                      uint16_t len,
                      uint8_t *in_val);
int snt_i2c_write_fifo(struct snt8100fsr *snt8100fsr,
                       uint16_t reg,
                       uint16_t len,
                       uint8_t *out_val);

#endif // I2C_H

