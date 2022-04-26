/*****************************************************************************
* File: spi-bus.h
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
#include <linux/spi/spidev.h>

#ifndef SPI_H
#define SPI_H

int snt_spi_open(struct spi_device *spi);

void snt_spi_close(struct spi_device *spi);

uint8_t snt_spi_get_boot_status(struct spi_device *spi);

int snt_spi_wake_device(struct spi_device *spi);

int snt_spi_read(struct spi_device *spi, int num_read, uint8_t *data_in);

int snt_spi_write(struct spi_device *spi, int num_write, uint8_t *data_out);

int snt_spi_read_and_write(struct spi_device *spi,
                           int num_write,
                           uint8_t *data_out,
                           uint8_t *data_in);

int snt_spi_read_fifo(struct spi_device *spi,
                      uint16_t reg,
                      uint16_t len,
                      uint8_t *in_val);

int snt_spi_write_fifo(struct spi_device *spi,
                       uint16_t reg,
                       uint16_t len,
                       uint8_t *out_val);

#endif // SPI_H

