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

/*==========================================================================*/
/* INCLUDES                                                                 */
/*==========================================================================*/
#include <asm/ioctl.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spidev.h>
#include <linux/delay.h>

#include "config.h"
#include "device.h"
#include "serial_bus.h"
#include "utils.h"
#include "spi_bus.h"
#include "i2c_bus.h"
#include "debug.h"


/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/
#undef DEBUG_TIMING_INFO    // Define to display BUS speed timing information

/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/
const uint32_t StatusReqPayload[2] = {4, 0};
uint8_t StatusDataIn[8];

/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/

/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/
/*
 * sb_get_boot_status() can only be called during firmware loading
 * prior to the last interrupt saying the system is ready.
 */
 int retry_rate = 0;
int sb_get_boot_status(struct snt8100fsr *snt8100fsr) {
    switch (snt8100fsr->bus_type) {
        case BUS_TYPE_SPI:
            return 0;//snt_spi_get_boot_status(snt8100fsr->spi);
        case BUS_TYPE_I2C:
            return snt_i2c_get_boot_status(snt8100fsr);

        default:
            PRINT_CRIT("Unknown Serial Bus Type %d", snt8100fsr->bus_type);
            break;
    }

    return -1;
}

int sb_wake_device(struct snt8100fsr *snt8100fsr) {
    switch (snt8100fsr->bus_type) {
        case BUS_TYPE_SPI:
            return snt_spi_wake_device(snt8100fsr->spi);
        case BUS_TYPE_I2C:
            return snt_i2c_wake_device(snt8100fsr);

        default:
            PRINT_CRIT("Unknown Serial Bus Type %d", snt8100fsr->bus_type);
            break;
    }
    return -1;
}

int sb_read_and_write(struct snt8100fsr *snt8100fsr,
                      int num_write,
                      uint8_t *data_out,
                      uint8_t *data_in) {
   int ret = 0, i;

#ifdef DEBUG_TIMING_INFO
    uint32_t start_time, duration;
    start_time = get_time_in_ms();
#endif

    switch (snt8100fsr->bus_type) {
        case BUS_TYPE_SPI:
            ret = snt_spi_read_and_write(snt8100fsr->spi,
                                         num_write,
                                         data_out,
                                         data_in);

            if (ret) {
                PRINT_CRIT("snt_spi_read_and_write() failed");
                return -1;
            }

            for(i = 0; i < num_write; i++) {
                if (data_in[i]) {
                    PRINT_ERR("UNEXPECTED RETURN STATUS = 0x%02X", data_in[i]);
                    return data_in[i];
                }
            }
        break;
        case BUS_TYPE_I2C:
	while(retry_rate<10){
            ret = snt_i2c_write(snt8100fsr, num_write, data_out);
            if (ret) {
                PRINT_CRIT("snt_i2c_write() failed");
		retry_rate++;
		msleep(10);
            }else{
            	retry_rate = 0;
		break;
            }
	}
	if(retry_rate == 10){
		return -1;
	}
        break;
        default:
            PRINT_CRIT("Unknown Serial Bus Type %d", snt8100fsr->bus_type);
            return -1;
    }

#ifdef DEBUG_TIMING_INFO
    duration = get_time_in_ms() - start_time;
    if (duration == 0)
        duration = 1;

    PRINT_DEBUG("%d bytes took %dms at %d kbit/s",
                num_write,
                duration,
                ((num_write * 8 / duration) * 1000) / 1024);
#endif
    return ret;
}

int sb_read_register(struct snt8100fsr *snt8100fsr,
                     uint16_t reg,
                     void *in_val) {
    switch (snt8100fsr->bus_type) {
        case BUS_TYPE_SPI:
            return snt_spi_read_fifo(snt8100fsr->spi,
                                     reg,
                                     2,
                                     (uint8_t *)in_val);
        case BUS_TYPE_I2C:
            return snt_i2c_read_fifo(snt8100fsr, reg, 2, (uint8_t *)in_val);

        default:
            PRINT_CRIT("Unknown Serial Bus Type %d", snt8100fsr->bus_type);
            break;
    }

    return -1;
}

int sb_write_register(struct snt8100fsr *snt8100fsr,
                      uint16_t reg,
                      void *out_val) {
    switch (snt8100fsr->bus_type) {
        case BUS_TYPE_SPI:
            return snt_spi_write_fifo(snt8100fsr->spi,
                                      reg,
                                      2,
                                      (uint8_t *)out_val);
        case BUS_TYPE_I2C:
            return snt_i2c_write_fifo(snt8100fsr, reg, 2, (uint8_t *)out_val);

        default:
            PRINT_CRIT("Unknown Serial Bus Type %d", snt8100fsr->bus_type);
            break;
    }

    return -1;
}

int sb_read_fifo(struct snt8100fsr *snt8100fsr,
                 uint16_t reg,
                 uint16_t len,
                 void *in_val) {
    switch (snt8100fsr->bus_type) {
        case BUS_TYPE_SPI:
            return snt_spi_read_fifo(snt8100fsr->spi,
                                     reg,
                                     len,
                                     (uint8_t *)in_val);
        case BUS_TYPE_I2C:
            return snt_i2c_read_fifo(snt8100fsr, reg, len, in_val);

        default:
            PRINT_CRIT("Unknown Serial Bus Type %d", snt8100fsr->bus_type);
            break;
    }

    return 0;
}

int sb_write_fifo(struct snt8100fsr *snt8100fsr,
                  uint16_t reg,
                  uint16_t len,
                  void *out_val) {
    switch (snt8100fsr->bus_type) {
        case BUS_TYPE_SPI:
            return snt_spi_write_fifo(snt8100fsr->spi,
                                      reg,
                                      len,
                                      (uint8_t *)out_val);
        case BUS_TYPE_I2C:
            return snt_i2c_write_fifo(snt8100fsr, reg, len, out_val);

        default:
            PRINT_CRIT("Unknown Serial Bus Type %d", snt8100fsr->bus_type);
            break;
    }

    return 0;
}
