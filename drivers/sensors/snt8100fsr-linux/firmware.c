/*****************************************************************************
* File: firmware.c
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

#include "config.h"
#include "irq.h"
#include "serial_bus.h"
#include "workqueue.h"
#include "event.h"
#include "file.h"
#include "memory.h"
#include "device.h"
#include "firmware.h"
#include "utils.h"
#include "debug.h"
#include "file_control.h"
#include "locking.h"
/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/
#define MAX_PAYLOAD_NUM     1000 // 1 = Hdr only
#define MAX_PAYLOAD_BYTES   (65 * 1024)

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/
struct upload_work {
	struct delayed_work my_work;
	struct snt8100fsr *snt8100fsr;

	char *filename;
	struct file *f;
	int payload_num;
	int file_offset;

	int num_write;
	uint32_t size;
	uint32_t pay_write;

	bool final_irq;
	bool waiting_for_irq;

	uint32_t firmware_upload_time;
	uint32_t payload_upload_time;
	uint32_t total_bytes_written;

	uint8_t *data_in;
	uint8_t *data_out;
};
/* ASUS BSP Clay: load different fw version according to HWID +++ */
//extern enum DEVICE_HWID g_ASUS_hwID;
/* ASUS BSP Clay: load different fw version according to HWID --- */
	
/*==========================================================================*/
/* PROTOTYPES                                                               */
/*==========================================================================*/
void upload_wq_func(struct work_struct *work);
static void upload_firmware_internal(struct upload_work *w);
static irqreturn_t irq_handler_top(int irq, void *dev);

static void download_wq_func(struct work_struct *work);
static void download_firmware_internal(struct upload_work *w);

static int open_firmware_file(struct upload_work *w);
static error_t firmware_waiting_irq(struct upload_work *w);
static error_t firmware_transfer_payloads(struct upload_work *w);
static void firmware_cleanup(struct upload_work *w, error_t err);
/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/
static uint32_t bcr_log_level = 0;
static uint32_t bcr_addr = 0;
static uint32_t bcr_irpt = 0;

static struct upload_work *work;
static struct upload_work *rst_work;

/*==========================================================================*/
/* BOOT FIRMWARE CONFIG                                                     */
/*==========================================================================*/
void set_bcr_addr(uint32_t addr) {
	bcr_addr = (addr & BCR_I2CADDR_MASK) << BCR_I2CADDR_POS;
}

void set_bcr_log_lvl(uint32_t l) {
	bcr_log_level = (l & BCR_LOGLVL_MASK) << BCR_LOGLVL_POS;
}

void set_bcr_irpt_lvl(uint32_t l) {
	bcr_irpt |= (l & BCR_IRPTLVL_MASK) << BCR_IRPTLVL_POS;
}

void set_bcr_irpt_pol(uint32_t p) {
	bcr_irpt |= (p & BCR_IRPTPOL_MASK) << BCR_IRPTPOL_POS;
}

void set_bcr_irpt_dur(uint32_t d) {
	bcr_irpt |= (d & BCR_IRPTDUR_MASK) << BCR_IRPTDUR_POS;
}

/*
 * set_bcr_word()
 *
 * [31:24] - I2C address. valid 0,0x2c,0x2d,0x5c,0x5d (0 means "all 4")
 * [23:11] - Reserved.
 * [10:8] - Logging Level
 * [7:2] - Edge Duration in 78MHz tics. "0" means 10 tics (default).
 * [1] - Interrupt Level. 0 - Level, 1 - Edge
 * [0] - Interrupt Polarity. 0 - Active High, 1 - Active Low
 */
uint32_t set_bcr_word(void) {
	uint32_t bcr = bcr_addr | bcr_log_level | bcr_irpt;
	PRINT_DEBUG("Boot Cfg Record = 0x%08x", bcr);
	return bcr;
}

struct delayed_work own_work;
struct delayed_work rst_dl_fw_work;
void Grip_check_lock_status(void){
	if(mutex_is_locked(&snt8100fsr_g->ap_lock))
		PRINT_INFO("AP_Lock is locked");
	if(mutex_is_locked(&snt8100fsr_g->sb_lock))
		PRINT_INFO("SB_Lock is locked");
	if(mutex_is_locked(&snt8100fsr_g->tap_lock))
		PRINT_INFO("TAP_Lock is locked");
}
/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/
int upload_firmware_fwd(struct snt8100fsr *snt8100fsr, char *filename) {
	PRINT_FUNC();
	// We can't upload the firmware directly as we need to pace
	// ourselves for how fast the hardware can accept data. So we
	// must setup a background thread with a workerqueue and process
	// the upload a piece at a time.
	rst_work = (void *)workqueue_alloc_work(sizeof(struct upload_work), download_wq_func);
	if (!rst_work) {
		PRINT_CRIT(OOM_STRING);
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&rst_dl_fw_work, download_wq_func);
	snt8100fsr_g->chip_reset_flag = GRIP_RST_FW_DL;
	rst_work->snt8100fsr = snt8100fsr;

	rst_work->filename = FW_PATH;
		
	/* [Todo] use vendor/asusfw path by default */
	/*
	if(fw_version == 0){
		rst_work->filename = FW_PATH_ASUSFW;
	}else{
		rst_work->filename = FW_PATH_VENDOR;
	}
	*/
	
	//PRINT_INFO("fw: %s,prjID = %d, hwID=", rst_work->filename, g_ASUS_prjID, g_ASUS_hwID);
	// Allocate our data buffers in contiguous memory for DMA support
	rst_work->data_in = memory_allocate(SNT_FWDL_BUF_SIZE, GFP_DMA);
	if (rst_work->data_in == NULL) {
		PRINT_CRIT("data_in = memory_allocate(%d) failed", SNT_FWDL_BUF_SIZE);
		return -ENOMEM;
	}

	rst_work->data_out = memory_allocate(SNT_FWDL_BUF_SIZE, GFP_DMA);
	if (rst_work->data_out == NULL) {
		PRINT_CRIT("data_out = memory_allocate(%d) failed", SNT_FWDL_BUF_SIZE);
		return -ENOMEM;
	}

	// Setup our logging level
	set_bcr_log_lvl(BCR_LOGGING_LEVEL);
	set_bcr_addr(BCR_ADDRESS_FILTER);
	set_bcr_irpt_lvl(BCR_INTERRUPT_TYPE);
	set_bcr_irpt_pol(BCR_INTERRUPT_POLARITY);
#if BCR_INTERRUPT_TYPE==BCR_IRQ_TYPE_PULSE
	// Setup interrupt edge duration
	set_bcr_irpt_dur(0x3f); //one equals 0.0128, so the duratin is 0x3f * 0.0128 = 63*0.0128 = 0.8064 us
#endif

	PRINT_INFO("1. upload_firmware_fwd: start first download_wq");
	workqueue_queue_work(&rst_dl_fw_work, 0);
	
	return 0;
}


int upload_firmware(struct snt8100fsr *snt8100fsr, char *filename) {
	// We can't upload the firmware directly as we need to pace
	// ourselves for how fast the hardware can accept data. So we
	// must setup a background thread with a workerqueue and process
	// the upload a piece at a time.
	PRINT_INFO("START");
	work = (void *)workqueue_alloc_work(sizeof(struct upload_work), upload_wq_func);
	if (!work) {
		PRINT_CRIT(OOM_STRING);
		return -ENOMEM;
	}
	
	//PRINT_INFO("ap_lock has initialed");
	//MUTEX_LOCK(&snt8100fsr_g->ap_lock);

	//INIT_DELAYED_WORK(&own_work, upload_wq_func);
	
	/* ASUS BSP Clay: protec status change */
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	snt8100fsr_g->chip_reset_flag = GRIP_FST_FW_DL;
	mutex_unlock(&snt8100fsr_g->ap_lock);
	work->snt8100fsr = snt8100fsr;

	work->filename = FW_PATH;
	
	PRINT_INFO("FW file path:%s", work->filename);
	
	// Allocate our data buffers in contiguous memory for DMA support
	work->data_in = memory_allocate(SNT_FWDL_BUF_SIZE, GFP_DMA);
	if (work->data_in == NULL) {
		PRINT_CRIT("data_in = memory_allocate(%d) failed", SNT_FWDL_BUF_SIZE);
		return -ENOMEM;
	}

	work->data_out = memory_allocate(SNT_FWDL_BUF_SIZE, GFP_DMA);
	if (work->data_out == NULL) {
		PRINT_CRIT("data_out = memory_allocate(%d) failed", SNT_FWDL_BUF_SIZE);
		return -ENOMEM;
	}

	// Setup our logging level
	set_bcr_log_lvl(BCR_LOGGING_LEVEL);
	set_bcr_addr(BCR_ADDRESS_FILTER);
	set_bcr_irpt_lvl(BCR_INTERRUPT_TYPE);
	set_bcr_irpt_pol(BCR_INTERRUPT_POLARITY);
#if BCR_INTERRUPT_TYPE==BCR_IRQ_TYPE_PULSE
	// Setup interrupt edge duration
	set_bcr_irpt_dur(0x3f); //one equals 0.0128, so the duratin is 0x3f * 0.0128 = 63*0.0128 = 0.8064 us
#endif

	//schedule_delayed_work(&own_work, msecs_to_jiffies(60));
	PRINT_INFO("END");
	return 0;
}

int irq_handler_fwd( void) {
	int delay;
	PRINT_INFO("Enter irq_handler_fwd");
	// Add a delay in milliseconds if our boot loader is logging output.
	// During it's logging, it can't receive data, so we delay a bit.
	// We have a known amount of delay, so it's always safe.
	if (BCR_LOGGING_LEVEL != BCR_LOGLVL_OFF) {
		delay = FIRMWARE_LOG_DELAY_MS;
	} else {
		delay = 0;
	}

	work->waiting_for_irq = false;
	if (workqueue_mod_work(&rst_dl_fw_work, delay) == false) {
		workqueue_queue_work(&rst_dl_fw_work, delay);
	}else{
		PRINT_INFO("irq_handler_fwd: workqueue_mod_work FAIL");
	}
	return 0;
}
void upload_wq_func(struct work_struct *work_orig) {
	static int count = 0;
	//upload_firmware_internal((struct upload_work *)work);
	if(count >= GRIP_WAIT_FW_PROBE){
		PRINT_INFO("wait struct create timeout, return");
		return;
	}
	if(snt8100fsr_g==NULL||work==NULL){
		PRINT_INFO("delay the work");
		schedule_delayed_work(&own_work, msecs_to_jiffies(1000));
		count++;
		return;
	}
	upload_firmware_internal(work);
	PRINT_DEBUG("SNT upload_wq_func done");
	return;
}

static void download_wq_func(struct work_struct *work_orig) {
	//download_firmware_internal((struct upload_work *)work);
	download_firmware_internal(rst_work);
	PRINT_DEBUG("SNT download_wq_func done");
	return;
}

static irqreturn_t irq_handler_top(int irq, void *dev) {
	int delay;
	PRINT_INFO("Enter");
	// Add a delay in milliseconds if our boot loader is logging output.
	// During it's logging, it can't receive data, so we delay a bit.
	// We have a known amount of delay, so it's always safe.
	if (BCR_LOGGING_LEVEL != BCR_LOGLVL_OFF) {
		delay = FIRMWARE_LOG_DELAY_MS;
	} else {
		delay = 0;
	}

	work->waiting_for_irq = false;
	/*
	if (workqueue_mod_work(work, delay) == false) {
		workqueue_queue_work(work, delay);
	}*/
	if (workqueue_mod_work(&own_work, delay) == false) {
		workqueue_queue_work(&own_work, delay);
	}
	
	return IRQ_HANDLED;
}

int retry_times = 25;
//extern bool g_Charger_mode;
static bool g_Charger_mode = false;
static int open_firmware_file(struct upload_work *w) {
	int ret = 0, i = 0;
	PRINT_FUNC("0x%p", w);
	if(g_Charger_mode) {
		ASUSEvtlog("[Grip] Sensor: Charger mode, don't load fw!!!");
		return -1;
	}
	// If we haven't opened the firmware file, do so
	if (w->f == 0) {
		PRINT_INFO("Opening file: %s", w->filename);
		while(i < retry_times){
			ret = file_open(w->filename, O_RDONLY, 0, &w->f);
			if(ret) {
				if( (i%5) == 0){
					PRINT_ERR("Unable to open firmware file '%s', error %d",
					w->filename, ret);
				}
				msleep(1000);
				i++;
			}else{
				break;
			}
		}
	}
	if(i > retry_times){
		ASUSEvtlog("[Grip] Sensor: can't find firmware file!!!");
	}
	return ret;
}

static error_t firmware_waiting_irq( struct upload_work *w) {
	PRINT_FUNC();
	/* If we are here, and are waiting for an IRQ, 
	 * then we have a timeout
	 * condition as the IRQ didn't occurr.
	 */
	if(FIRMWARE_UPLOAD_WITH_IRQ) {
		if (w->waiting_for_irq) {
			PRINT_CRIT("Timeout waiting for interrupt. Please ensure hardware "
						 "is correctly wired and firmware image is valid.");
			w->waiting_for_irq = false;
			return E_TIMEOUT;
		}

		w->waiting_for_irq = true;

		/* Queue a job to check for an IRQ timeout. The timer includes the
		 * time to transfer a payload, so ensure it's long enough.
		 */
		if(snt8100fsr_g->chip_reset_flag == GRIP_FST_FW_DL){
			PRINT_INFO("own_work");
			workqueue_queue_work(&own_work, FIRMWARE_IRQ_TIMEOUT_MS);
		}else{
			PRINT_INFO("Call reset IRQ workqueue: 2s");
			workqueue_queue_work(&rst_dl_fw_work, FIRMWARE_IRQ_TIMEOUT_MS_RST);
		}
	}
	return E_SUCCESS;
}

static error_t firmware_transfer_payloads( struct upload_work *w) {
	int ret;
	int num_write;
	uint32_t payload_write;
	uint32_t payload_size;
	uint32_t size;
	uint32_t payload_duration;
	PRINT_FUNC();
	/*
	 * Read size of next payload from the file. 
	 * Actually reading full
	 * HW_FIFO_SIZE so will need to write this out
	 */
	num_write = file_read(w->f, w->file_offset, (void *)w->data_out, SPI_FIFO_SIZE);
	if (num_write <= 0) {
		PRINT_INFO("EOF Reached. Firmware data uploaded.");
		return E_FINISHED;
	}
	

	w->file_offset += SPI_FIFO_SIZE;
	w->payload_num++;

	// Size is first long word of buffer read
	payload_size = ((uint32_t*)w->data_out)[0];
	size = payload_size;
	PRINT_DEBUG("Payload %d = %d Bytes (%d inc 'size' field)", w->payload_num, size, size + 4);

	// If this is first segment, then pad word is boot cfg
	if (w->payload_num == 1) {
		((uint32_t*)w->data_out)[1] = set_bcr_word();

		// Record the start time of the first payload to measure the total
		// time the firmware upload takes to complete.
		w->firmware_upload_time = get_time_in_ms();
	}

	// Record the start of the transfer of this payload
	w->payload_upload_time = get_time_in_ms();

	// Write the size out to chip
	ret = sb_read_and_write(w->snt8100fsr, num_write, w->data_out, w->data_in);
	if (ret == FIRMWARE_ALREADY_LOADED_RESULT) {
		PRINT_NOTICE("Existing firmware already loaded...");
		return E_ALREADY;
	} else if (ret) {
		PRINT_ERR("sb_write() failed");
		return E_FAILURE;
	}else{
	}

	w->total_bytes_written += num_write;
	size -= (SPI_FIFO_SIZE - sizeof(size));

	// Fatal if read_size not /8
	if (size % SPI_FIFO_SIZE) {
		PRINT_ERR("Size not multiple of %d", SPI_FIFO_SIZE);
		return E_BADSIZE;
	}

	// Get payload and write it out in SNT_FWDL_BUF_SIZE chunks
	payload_write = 0;
	while (size != 0 && payload_write < MAX_PAYLOAD_BYTES) {
		int read_size = min((unsigned int)SNT_FWDL_BUF_SIZE, size);


		num_write = file_read(w->f, w->file_offset, (void*)w->data_out, read_size);
		if (num_write <= 0) {
			PRINT_DEBUG("EOF Reached. Stopping...");
			return E_BADREAD;
		}

		w->file_offset += read_size;

		/* Write the data to the bus */
		ret = sb_read_and_write(w->snt8100fsr, num_write, w->data_out, w->data_in);
		if (ret == FIRMWARE_ALREADY_LOADED_RESULT) {
			PRINT_NOTICE("Existing firmware already loaded...");
			return E_ALREADY;
		} else if (ret) {
			PRINT_ERR("sb_write() failed");
			return E_BADWRITE;
		}

		w->total_bytes_written += num_write;
		size -= num_write;
		payload_write += num_write;
	}

	// Calculate how long this total payload took
	payload_duration = get_time_in_ms() - w->payload_upload_time;
	if (payload_duration == 0)
		payload_duration = 1;

	PRINT_INFO("Payload %d took %dms at %d kbit/s", w->payload_num, payload_duration,
				((payload_size * 8 / payload_duration) * 1000) / 1024);

	if (w->payload_num >= MAX_PAYLOAD_NUM) {
		PRINT_DEBUG("Max Payload Reached. Stopping...");
		return E_TOOMANY;
	}

	if (FIRMWARE_UPLOAD_WITH_IRQ) {
		PRINT_INFO("Waiting for IRQ for next payload");
	} else {
		PRINT_INFO("workqueue_queue_work()");
		//workqueue_queue_work(w, FIRMWARE_UPLOAD_DELAY_MS);
		if(snt8100fsr_g->chip_reset_flag == GRIP_FST_FW_DL){
			workqueue_queue_work(&own_work, FIRMWARE_UPLOAD_DELAY_MS);
		} else {
			//PRINT_INFO("4. firmware_transfer_payloads: download wq trigger");
			workqueue_queue_work(&rst_dl_fw_work, FIRMWARE_UPLOAD_DELAY_MS);
		}
	}
	return E_SUCCESS;
}

static void firmware_cleanup(struct upload_work *w, error_t err) {
	PRINT_FUNC();
	if(err <= E_SUCCESS) {
		int duration;
		duration = get_time_in_ms() - w->firmware_upload_time;
		if (duration == 0)
			duration = 1;

		PRINT_DEBUG("Total %d bytes took %dms at %d kbit/s", w->total_bytes_written, duration,
					((w->total_bytes_written * 8 / duration) * 1000) / 1024);
	}

	if(snt8100fsr_g->chip_reset_flag == GRIP_FST_FW_DL){	
		if(work->data_in!=NULL){
			PRINT_INFO("DATA_IN");
			memory_free(work->data_in);
		}if(work->data_out!=NULL){
			PRINT_INFO("DATA_OUT");
			memory_free(work->data_out);
		}
	}else{
		if(rst_work->data_in!=NULL){
			PRINT_INFO("DATA_IN");
			memory_free(rst_work->data_in);
		}if(rst_work->data_out!=NULL){
			PRINT_INFO("DATA_OUT");
			memory_free(rst_work->data_out);
		}
	}

	if(FIRMWARE_UPLOAD_WITH_IRQ) {
		// Cancel our irq timeout work queue item
		//workqueue_cancel_work(w);
		PRINT_INFO("Clear wq work and unregister irq");
		if(snt8100fsr_g->chip_reset_flag == GRIP_FST_FW_DL){
			workqueue_cancel_work(&own_work);
		} else {
			workqueue_cancel_work(&rst_dl_fw_work);
		}
	}

	if (w->f) {
		filp_close(w->f, NULL);
		w->f = NULL;
	}

	//workqueue_free_work(w);
	PRINT_DEBUG("Finished");
	return;
}   
static void upload_firmware_internal(struct upload_work *w) {
	/*
	 * insmod driver causes firmware to be uploaded to chip 
	 */
	int ret;
	error_t err = E_SUCCESS;
	PRINT_FUNC("0x%p", w);
	PRINT_INFO("Enter upload_firmware_internal");

	if(snt8100fsr_g == NULL){
		PRINT_INFO("snt8100fsr is NULL, do nothing");
		return;
	}

	/* prevent load fw again when load status = 1 */
	if(snt8100fsr_g->grip_fw_loading_status == true)
		return;
	
	// If we haven't opened the firmware file, do so
	if (w->f == 0) {
		ret = open_firmware_file(w);
		if (ret) {
			err = E_BADOPEN;
			goto cleanup;
		}
		else {
			PRINT_INFO("Opening file: %s success", w->filename);
			// [dy] unique to upload_firmware
			// Register our interrupt handler 
			if(FIRMWARE_UPLOAD_WITH_IRQ) {
				snt_irq_db.top = irq_handler_top;
				ret = gpio_request(snt8100fsr_g->hostirq_gpio, IRQ_NAME);
				if (ret) {
					PRINT_INFO("gpio_request ERROR(%d). \n", ret);
				}
				ret = gpio_direction_input(snt8100fsr_g->hostirq_gpio);
				if (ret) {
					PRINT_INFO("gpio_direction_input ERROR(%d). \n", ret);
				}
				snt_irq_db.irq_num = gpio_to_irq(snt8100fsr_g->hostirq_gpio);
				irq_handler_register(&snt_irq_db);
			}
		}
	}
	err = firmware_waiting_irq(w);
	PRINT_INFO("Exit firmware_waiting_irq");
	if (err >= E_FAILURE) {
		PRINT_CRIT("firmware_waiting_irq err %d", (int)err);
		goto cleanup;
	}
	err = firmware_transfer_payloads(w);
	if (err >= E_FAILURE) {
		PRINT_CRIT("firmware_transfer_payloads err %d", (int)err);
	goto cleanup;
	}
	else if (err == E_FINISHED) { goto cleanup; }
	return;
cleanup:
	PRINT_INFO("firmware_cleanup");
	firmware_cleanup( w, err);
	if(FIRMWARE_UPLOAD_WITH_IRQ) {
		irq_handler_unregister(&snt_irq_db);  //[dy] unique to upload_firmware
	}

	/* ASUS BSP Clay: protec status change */
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	snt8100fsr_g->chip_reset_flag = GRIP_FW_DL_END;
	mutex_unlock(&snt8100fsr_g->ap_lock);
	Grip_check_lock_status();
	if(err <= E_SUCCESS) { //[dy] unique to upload_firmware
		snt8100fsr_g->grip_fw_loading_status = true;
		MUTEX_LOCK(&snt8100fsr_g->ap_lock);
		start_event_processing(w->snt8100fsr);
		PRINT_INFO("SUCESS to load fw");
		if(snt8100fsr_g->Recovery_Flag){
			queue_delayed_work(asus_wq, &rst_recovery_wk, msecs_to_jiffies(0));
		}
		mutex_unlock(&snt8100fsr_g->ap_lock);
	}else{
		PRINT_INFO("FAILED to load fw");
		ASUSEvtlog("[Grip] Sensor: Load fw fail!!!");
		start_event_processing(w->snt8100fsr);
		snt8100fsr_g->grip_fw_loading_status = false;
		//Power_Control(0);
		queue_delayed_work(asus_wq, &rst_gpio_wk, msecs_to_jiffies(1000));
		PRINT_INFO("Call reset work");
	}
	Grip_check_lock_status();
	return;
}

static void download_firmware_internal(struct upload_work *w) {
	/*
	 * Chip reset sets event register bit FWD so driver must
	 * download chip firmware.  
	 * IRQ already set up by start_event_processing
	 */
	error_t err = E_SUCCESS;
	int ret;
	PRINT_FUNC("0x%p", w);
	PRINT_INFO("Enter download_firmware_internal");

	// If we haven't opened the firmware file, do so
	if (w->f == 0) {
		ret = open_firmware_file(w);
		if(ret) {
			err = E_BADOPEN;
			goto cleanup;
		}
	}

/*
	err = firmware_waiting_irq(w);
	if (err >= E_FAILURE) {
		PRINT_CRIT("firmware_waiting_irq err %d", (int)err);
		goto cleanup;
	}
*/
	err = firmware_transfer_payloads(w);
	if (err >= E_FAILURE) {
		PRINT_CRIT("firmware_transfer_payloads err %d", (int)err);
		goto cleanup;
	}
	else if (err == E_FINISHED) { goto cleanup; }
	return;
cleanup:
	PRINT_INFO("firmware_cleanup start");
	firmware_cleanup( w, err);
	PRINT_INFO("firmware_cleanup done");

	/* ASUS BSP Clay: protec status change */
	MUTEX_LOCK(&snt8100fsr_g->ap_lock);
	snt8100fsr_g->chip_reset_flag = GRIP_FW_DL_END;
	mutex_unlock(&snt8100fsr_g->ap_lock);

	Grip_check_lock_status();
	// [dy] 2017-09-01 fwd done, set context_fwd to false
	// [dy] 2017-09-05 initializations after reset  
	set_context_fwd_done(w->snt8100fsr);
	if(err <= E_SUCCESS) { //[dy] unique to upload_firmware
		snt8100fsr_g->grip_fw_loading_status = true;
		ASUS_Handle_Reset(w->snt8100fsr);
	}else{
		snt8100fsr_g->grip_fw_loading_status = false;
		ASUSEvtlog("[Grip] Sensor: Retry Load fw fail!!!");
		queue_delayed_work(asus_wq, &rst_gpio_wk, msecs_to_jiffies(1000));
	}
	Grip_check_lock_status();
	return;
}
