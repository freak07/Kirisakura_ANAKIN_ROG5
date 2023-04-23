/*****************************************************************************
* File: spi-bus.c
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
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
//#include <linux/wakelock.h>
//#include <soc/qcom/scm.h>

#include "config.h"
#include "serial_bus.h"
#include "sonacomm.h"
#include "device.h"
#include "event.h"
#include "spi_bus.h"
#include "hardware.h"
#include "sysfs.h"
#include "memory.h"
#include "utils.h"
#include "main.h"
#include "debug.h"

#include "asus_init.h"
/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/
#define MAX_CMD_FIFO        1024
#define CMD_HDR_SIZE        8
#define MAX_CMD_BYTES       (MAX_CMD_FIFO + 2*CMD_HDR_SIZE)
#define SPI_BITS_PER_WORD   8

#define DATA_SIZE           (17 * 1024)

/*==========================================================================*/
/* LOCAL PROTOTYPES                                                         */
/*==========================================================================*/
static int resync_fifo(struct spi_device *spi);

/*==========================================================================*/
/* SPI DEVICE INFO                                                          */
/*==========================================================================*/
#define MOD_ALIAS "snt8100fsr"

struct spi_board_info spi_dev_info = {
    .modalias       = MOD_ALIAS,
    .max_speed_hz   = SPI_MAX_SPEED_HZ,
    .mode           = SPI_MODE_0,
    .bus_num        = SPI_BUS,
    .chip_select    = SPI_CHIPSELECT,
};

/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/
static uint32_t *request_out;
static uint8_t  *status_in;
static uint8_t  *data_in;
static uint8_t  *data_out;
static bool     resyncing_fifo = false;

/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/

int snt_spi_open(struct spi_device *spi) {
    int ret;

    // Allocate our data buffers in contiguous memory for DMA support
    status_in = memory_allocate(SPI_FIFO_SIZE, GFP_DMA);
    if (status_in == NULL) {
        PRINT_CRIT("status_in = memory_allocate(%d) failed", SPI_FIFO_SIZE);
        return -1;
    }
	
	
    spi->bits_per_word = SPI_BITS_PER_WORD;
    spi->mode = SPI_MODE_0;

    request_out = memory_allocate(SPI_FIFO_SIZE, GFP_DMA);
    if (request_out == NULL) {
        PRINT_CRIT("request_out = memory_allocate(%d) failed", SPI_FIFO_SIZE);
        return -1;
    }

    data_in = memory_allocate(DATA_SIZE, GFP_DMA);
    if (data_in == NULL) {
        PRINT_CRIT("data_in = memory_allocate(%d) failed", DATA_SIZE);
        return -1;
    }

    data_out = memory_allocate(DATA_SIZE, GFP_DMA);
    if (data_out == NULL) {
        PRINT_CRIT("data_out = memory_allocate(%d) failed", DATA_SIZE);
        return -1;
    }

    ret = spi_setup(spi);
    if (ret) {
        PRINT_CRIT("Failed to setup slave device");
        return -1;
    }

    PRINT_INFO("Registered new SPI slave device");
    return 0;
}

void snt_spi_close(struct spi_device *spi) {
    PRINT_FUNC();

    if (request_out) {
        memory_free(request_out);
        request_out = NULL;
    }

    if (status_in) {
        memory_free(status_in);
        status_in = NULL;
    }

    if (data_in) {
        memory_free(data_in);
        data_in = NULL;
    }

    if (data_out) {
        memory_free(data_out);
        data_out = NULL;
    }

    if (spi) {
        PRINT_INFO("Unregistering spi device");
        //spi_unregister_device(spi);
        spi = NULL;
    }

    PRINT_DEBUG("done");
}

/*
 * snt_spi_get_boot_status() can only be called during firmware loading
 * prior to the last interrupt saying the system is ready.
 */
uint8_t snt_spi_get_boot_status(struct spi_device *spi) {
    int     count;
    int     i;
    u16     num_write = 8;
    uint8_t ret       = 0;

    // Setup our outgoing request
    request_out[0] = 4;
    request_out[1] = 0;

    // Write the data to the bus
    count = snt_spi_read_and_write(spi,
                                   num_write,
                                   (uint8_t *)request_out,
                                   status_in);
    if (count != num_write) {
        PRINT_ERR("Only a partial number of bytes written\n");
        PRINT_ERR("  (%d) instead of full (%d)\n", count, num_write);
        goto cleanup;
    }

    for (i=0; i < count; i++)
      if (status_in[i]) {
        PRINT_ERR("UNEXPECTED PAYLOAD STATUS = 0x%02x\n", status_in[i]);
        ret = status_in[i];
        goto cleanup;
      }

  cleanup:
    return ret;
}

/*
 * Wake the device up over the SPI bus by wiggling the slave select line for
 * at least 50us.
 */
int snt_spi_wake_device(struct spi_device *spi) {
#ifdef DYNAMIC_PWR_CTL
		uint8_t in_val[16];
		PRINT_FUNC();
		/* issue a 0-byte read of register 0x00. 16 bytes total */
		snt_spi_read_fifo(spi, 0, 0, in_val);
		/* do a second one to make sure the spi fifos on SNT8100 get resync'd	*/
		snt_spi_read_fifo(spi, 0, 0, in_val);
	
#else
    struct spi_transfer t;
    struct spi_message m;
    struct snt8100fsr *snt8100fsr = dev_get_drvdata(&spi->dev);

    PRINT_FUNC();

    // We need only wiggle chip-select for at least 50us
    spi_message_init(&m);
    memset(&t, 0, sizeof(t));

    t.cs_change = 1;
    t.delay_usecs = SPI_WAKE_CHIP_SELECT_TIME;
     t.speed_hz = snt8100fsr->spi_freq_khz * 1000;
    spi_message_add_tail(&t, &m);

    spi_sync(spi, &m);
#endif

    PRINT_DEBUG("done");
    return 0;
}

int snt_spi_read(struct spi_device *spi, int num_read, uint8_t *data_in) {
    struct spi_transfer t;
    struct spi_message m;
    struct snt8100fsr *snt8100fsr = dev_get_drvdata(&spi->dev);

    PRINT_FUNC("%d bytes", num_read);

    spi_message_init(&m);
    memset(&t, 0, sizeof(t));

    t.rx_buf = data_in;
    t.len = num_read;
    t.speed_hz = snt8100fsr->spi_freq_khz * 1000;
    spi_message_add_tail(&t, &m);

    spi_sync(spi, &m);

    PRINT_DEBUG("%d bytes read", num_read);
    return 0;
}

int snt_spi_write(struct spi_device *spi, int num_write, uint8_t *data_out) {
    struct spi_transfer t;
    struct spi_message m;
    struct snt8100fsr *snt8100fsr = dev_get_drvdata(&spi->dev);

    PRINT_FUNC("%d bytes", num_write);

    spi_message_init(&m);
    memset(&t, 0, sizeof(t));

    t.tx_buf = data_out;
    t.len = num_write;
    t.speed_hz = snt8100fsr->spi_freq_khz * 1000;
    spi_message_add_tail(&t, &m);

    spi_sync(spi, &m);

    PRINT_DEBUG("%d bytes written", num_write);
    return 0;
}

int snt_spi_read_and_write(struct spi_device *spi,
                           int num_write,
                           uint8_t *data_out,
                           uint8_t *data_in) {
    struct spi_transfer t;
    struct spi_message m;
    struct snt8100fsr *snt8100fsr = dev_get_drvdata(&spi->dev);

    PRINT_FUNC("%d bytes", num_write);

    spi_message_init(&m);
    memset(&t, 0, sizeof(t));

    t.tx_buf = data_out;
    t.rx_buf = data_in;
    t.len = num_write;
    t.speed_hz = snt8100fsr->spi_freq_khz * 1000;
    spi_message_add_tail(&t, &m);

    spi_sync(spi, &m);

    PRINT_DEBUG("%d bytes read and written", num_write);
    return 0;
}

int snt_spi_read_fifo(struct spi_device *spi,
                      uint16_t reg,
                      uint16_t len,
                      uint8_t *in_val) {
    int ret, i;

    // Transaction size is cmd hdr + number of SPI FIFOs to handle read
    int xfer_len = ((len + 4 + (SPI_FIFO_SIZE-1)) / SPI_FIFO_SIZE) * SPI_FIFO_SIZE;
    int num_write = xfer_len + CMD_HDR_SIZE;

    PRINT_FUNC("reg 0x%X", reg);

    if (len % 2) {
        PRINT_CRIT("Length must be multiple of 2 (%d)", len);
        return -1;
    }

    if (reg == REGISTER_FIFO_COMMAND) {
        if (len > SC_MAX_PACKET_LENGTH) {
            PRINT_CRIT("Length must be <= than %d (%d)", SC_MAX_PACKET_LENGTH, len);
            return -1;
        }
    } else if (reg == REGISTER_FIFO_STREAM) {
        if (len > SC_MAX_FRAME_BUFFER) {
            PRINT_CRIT("Length must be <= than %d (%d)", SC_MAX_FRAME_BUFFER, len);
            return -1;
        }
    } else {
        if (len > MAX_CMD_FIFO) {
            PRINT_CRIT("Length must be <= than %d (%d)", MAX_CMD_FIFO, len);
            return -1;
        }

        if (num_write > MAX_CMD_BYTES) {
            PRINT_CRIT("Total write len must <= %d (%d)", MAX_CMD_BYTES, num_write);
            return -1;
        }
    }

    /*
     * Write and Read of a 2 byte value
     * [2 Read Command] [2 Register] [4 Len] [       8 Pad         ]
     * [           8 Bytes Pad             ] [4 Pad] [2 Val] [2 Pad]
     */
    memset(data_out, 0, DATA_SIZE);
    data_out[0] = 0x29;
    data_out[1] = 0xe6;
    data_out[2] = (reg >> 0 ) & 0xff;
    data_out[3] = (reg >> 8 ) & 0xff;
    data_out[4] = (len >> 0 ) & 0xff;
    data_out[5] = (len >> 8 ) & 0xff;
    data_out[6] = (len >> 16) & 0xff;
    data_out[7] = (len >> 24) & 0xff;

    ret = snt_spi_read_and_write(spi, num_write, data_out, data_in);
    if (ret) {
        PRINT_CRIT("snt_spi_read_and_write() failed");
        return -1;
    }

    // Check if we received an error, and resync the FIFO if we have one
    for (i = 1; i <= 11; i++) {
        if (data_in[i] != 0) {
            if (resyncing_fifo == false) {
                PRINT_CRIT("Fault detected, MISO byte #%d = 0x%0x received. "
                           "Re-synchronizing remote FIFO", i, data_in[i]);
                ret = resync_fifo(spi);
                if (ret) {
                    PRINT_CRIT("Unable to resync FIFO");
                    return -1;
                } else {
                    PRINT_NOTICE("Resync FIFO successful, retrying original request");
                    return snt_spi_read_fifo(spi, reg, len, in_val);
                }
            } else {
                PRINT_CRIT("Fault detected during resync FIFO. "
                           "MISO byte #%d = 0x%0x received.", i, data_in[i]);
                return -1;
            }
        }
    }

    // Strip off cmd hdr + 4 byte padding
    memcpy((void*)in_val, (void*)&data_in[CMD_HDR_SIZE+4], len);

    PRINT_DEBUG("done");
    return 0;
}

static int resync_fifo(struct spi_device *spi) {
    int i, ret;
    uint8_t byte_in = 0;
    uint8_t byte_out = 0;
    uint16_t value;

    PRINT_FUNC();

    resyncing_fifo = true;

    for (i = 0; i < 8; i++) {
        // Send a 1 byte value of 0 on the SPI bus
        ret = snt_spi_read_and_write(spi, 1, &byte_out, &byte_in);
        if (ret) {
            PRINT_CRIT("resync: snt_spi_read_and_write() failed");
        }

        // Read a register
        value = 0;
        ret = snt_spi_read_fifo(spi,
                                REGISTER_LOOPBACK,
                                WIDTH_OF_REGISTER_VALUE,
                                (uint8_t *)&value);
        if (ret) {
            PRINT_CRIT("resync: snt_spi_read_and_write() failed");
        } else if (value != LOOPBACK_REGISTER_DEFAULT_VALUE) {
            PRINT_WARN("resync: loopback value invalid");
        } else {
            PRINT_NOTICE("resync fifo successful");
            resyncing_fifo = false;
            return 0;
        }

        // Read a register one more time
        value = 0;
        ret = snt_spi_read_fifo(spi,
                                REGISTER_LOOPBACK,
                                WIDTH_OF_REGISTER_VALUE,
                                (uint8_t *)&value);
        if (ret) {
            PRINT_CRIT("resync: snt_spi_read_and_write() failed");
        } else if (value != LOOPBACK_REGISTER_DEFAULT_VALUE) {
            PRINT_WARN("resync: loopback value invalid");
        } else {
            PRINT_NOTICE("resync fifo successful");
            resyncing_fifo = false;
            return 0;
        }
    }

    PRINT_CRIT("Failed to resync fifo");
    resyncing_fifo = false;
    return -1;
}

int snt_spi_write_fifo(struct spi_device *spi,
                       uint16_t reg,
                       uint16_t len,
                       uint8_t *out_val) {
    int ret;

    // Transaction size is cmd hdr + number of SPI FIFOs to handle read
    int xfer_len = ((len + (SPI_FIFO_SIZE-1)) / SPI_FIFO_SIZE) * SPI_FIFO_SIZE;
    int num_write = xfer_len + CMD_HDR_SIZE;

    PRINT_FUNC("reg 0x%X", reg);

    if (len % 2) {
        PRINT_CRIT("Length must be multiple of 2 (%d)", len);
        return -1;
    }

    if (reg == REGISTER_FIFO_COMMAND) {
        if (len > SC_MAX_PACKET_LENGTH) {
            PRINT_CRIT("Length must be <= than %d (%d)", SC_MAX_PACKET_LENGTH, len);
            return -1;
        }

    } else if (reg == REGISTER_FIFO_STREAM) {
        if (len > SC_MAX_FRAME_BUFFER) {
            PRINT_CRIT("Length must be <= than %d (%d)", SC_MAX_FRAME_BUFFER, len);
            return -1;
        }
    } else {
        if (len > MAX_CMD_FIFO) {
            PRINT_CRIT("Length must be <= than %d (%d)", MAX_CMD_FIFO, len);
            return -1;
        }

        if (num_write > MAX_CMD_BYTES) {
          PRINT_CRIT("Total write len must <= %d (%d)\n", MAX_CMD_BYTES, num_write);
          return -1;
        }
    }

    /*
     *  Write of a 2 byte value
     * [2 Read Command] [2 Register] [4 Len] [2 Val] [   6 Pad     ]
     */
    memset(data_out, 0, DATA_SIZE);
    data_out[0] = 0x28;
    data_out[1] = 0xe6;
    data_out[2] = (reg >> 0 ) & 0xff;
    data_out[3] = (reg >> 8 ) & 0xff;
    data_out[4] = (len >> 0 ) & 0xff;
    data_out[5] = (len >> 8 ) & 0xff;
    data_out[6] = (len >> 16) & 0xff;
    data_out[7] = (len >> 24) & 0xff;

    memcpy((void*)&data_out[8], (void*)out_val, len);

    ret = snt_spi_write(spi, num_write, data_out);
    if (ret) {
        PRINT_CRIT("snt_spi_read_and_write() failed");
        return -1;
    }

    PRINT_DEBUG("done");
    return 0;
}

/*==========================================================================*/
/* Device Probe/Remove/Resume/Suspend                                       */
/*==========================================================================*/
#ifdef USE_SPI_BUS
static int snt_spi_probe(struct spi_device *spi) {
    int ret;
    struct device *dev = &spi->dev;
    struct device_node *np = dev->of_node;
    struct snt8100fsr *snt8100fsr;

    PRINT_FUNC();

    ret = main_init();
    if (ret) {
        return ret;
    }

    snt8100fsr = kzalloc(sizeof(*snt8100fsr),
                                 GFP_KERNEL);

    if(!snt8100fsr) {
        PRINT_CRIT("failed to allocate memory for struct snt8100fsr");
        return -ENOMEM;
    }

    memset(snt8100fsr, 0, sizeof(*snt8100fsr));

    snt8100fsr->bus_type = BUS_TYPE_SPI;
    snt8100fsr->dev = dev;
    dev_set_drvdata(dev, snt8100fsr);
    snt8100fsr->spi = spi;

    ret = snt_spi_device_init(spi, snt8100fsr);
    if(ret) {
        PRINT_ERR("snt_spi_device_init() failed");
        return ret;
    }

    ret = snt_spi_open(snt8100fsr->spi);
        if (ret) {
        PRINT_CRIT("snt_spi_open() failed");
        return ret;
    }
		
	snt8100fsr->en_demo = 0;
	snt8100fsr->en_sensor_evt = 1;

    // Save this as our main device and to be used for sysFS access
    snt8100fsr_g = snt8100fsr;
    mutex_init(&snt8100fsr_g->track_report_sysfs_lock);
    mutex_init(&snt8100fsr_g->sb_lock);
    mutex_init(&snt8100fsr_g->ap_lock);
    mutex_init(&snt8100fsr_g->tap_lock);
	
    PRINT_INFO("Test ap_lock");
    mutex_lock(&snt8100fsr->ap_lock);
    PRINT_INFO("Test ap_lock1");
    mutex_unlock(&snt8100fsr->ap_lock);
    PRINT_INFO("Test ap_lock2");
    mutex_lock(&snt8100fsr_g->ap_lock);
    PRINT_INFO("Test ap_lock3");
    mutex_unlock(&snt8100fsr_g->ap_lock);
    PRINT_INFO("Test ap_lock4");
    asus_init_probe();

#ifdef DYNAMIC_PWR_CTL
    // create wake semaphore
    sema_init(&snt8100fsr_g->wake_rsp, 0);
    sema_init(&snt8100fsr_g->wake_req, 0);
    snt8100fsr_g->enable_dpc_flag = DYNAMIC_PWR_CTL;
#endif

    // Set up semaphores to control waiting for sonacomm response
    sema_init(&snt8100fsr_g->sc_wf_rsp_req, 0);
    sema_init(&snt8100fsr_g->sc_wf_rsp, 0);

    /*
     * Upload the firmware asynchronously. When finished,
     * it will call start_event_processing() in event.c
     */
#ifdef UPLOAD_FIRMWARE
    upload_firmware(snt8100fsr, FW_PATH);
#else
    start_event_processing(snt8100fsr);
#endif

    snt_sysfs_init(snt8100fsr_g, true);

    PRINT_DEBUG("done");
    return 0;
}

static int snt_spi_remove(struct spi_device *spi) {
    struct snt8100fsr *snt8100fsr = dev_get_drvdata(&spi->dev);

    PRINT_FUNC();

    if (snt8100fsr) {
        stop_event_processing(snt8100fsr);
        if (snt8100fsr_g) {
            snt_sysfs_init(snt8100fsr_g, false);
        }
    }

    snt_spi_close(spi);

    if (snt8100fsr) {
        memory_free(snt8100fsr);
    }

    main_exit();
    return 0;
}

static int snt_spi_suspend(struct device *dev)
{
    int ret;
    PRINT_FUNC();
    ret = snt_suspend(dev);
    PRINT_DEBUG("done");
    return ret;
}

static int snt_spi_resume(struct device *dev)
{
    int ret;
    PRINT_FUNC();
    ret = snt_resume(dev);
    PRINT_DEBUG("done");
    return ret;
}

/*==========================================================================*/
/* Driver Registration                                                      */
/*==========================================================================*/
static const struct of_device_id snt8100fsr_spi_dt_ids[] = {
    { .compatible = "sentons,snt8100fsr-spi" },
    { /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, snt8100fsr_spi_dt_ids);

static const struct dev_pm_ops snt_spi_pm_ops = {
    .suspend = snt_spi_suspend,
    .resume = snt_spi_resume,
};

static struct spi_device_id snt_spi_id[] = {
    {SENTONS_DRIVER_NAME, 0},
    {},
};

static struct spi_driver snt8100fsr_spi_driver = {
    .driver		= {
        .name	= SENTONS_DRIVER_NAME,
        .owner	= THIS_MODULE,
        .of_match_table	= of_match_ptr(snt8100fsr_spi_dt_ids),
        .pm = &snt_spi_pm_ops,
    },
    .probe		= snt_spi_probe,
    .remove		= snt_spi_remove,
    .id_table   = snt_spi_id,
};

module_spi_driver(snt8100fsr_spi_driver);
#endif
