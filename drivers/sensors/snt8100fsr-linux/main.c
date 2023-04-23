/*****************************************************************************
* File: main.c
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
#include <linux/spi/spi.h>
#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include "config.h"
#include "sonacomm.h"
#include "serial_bus.h"
#include "file.h"
#include "spi_bus.h"
#include "irq.h"
#include "memory.h"
#include "workqueue.h"
#include "firmware.h"
#include "input_device.h"
#include "device.h"
#include "event.h"
#include "main.h"
#include "debug.h"

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/

/*==========================================================================*/
/* LOCAL PROTOTYPES                                                         */
/*==========================================================================*/
static int main_cleanup(void);

/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/

/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/
int main_init(void) {
	int ret = -1;

	PRINT_FUNC();

	// Initialize our workqueue system for background processing
	ret = workqueue_init_snt();
	if (ret != 0) {
		PRINT_CRIT("workqueue_init_snt() failed");
		goto main_init_failed;
	}

	// Initialize the input device
	ret = input_device_init();
	if (ret != 0) {
		PRINT_CRIT("Input device init failed: %d", ret);
		goto main_init_failed;
	}

	PRINT_INFO("done");
	return 0;

main_init_failed:
	PRINT_CRIT("Failed to initialize driver");
	main_cleanup();
	return ret;
}

int main_exit(void) {
	PRINT_FUNC();
	return main_cleanup();
}

int main_cleanup(void) {
	PRINT_FUNC();
	if(log_track_reports_file) {
		file_close(log_track_reports_file);
		log_track_reports_file = NULL;
	}
	if(log_d1test_file) {
		file_close(log_d1test_file);
		log_d1test_file = NULL;
	}
	if(log_no_touch_frame_file) {
		file_close(log_no_touch_frame_file);
		log_no_touch_frame_file = NULL;
	}
	PRINT_INFO("freeing IRQ workq");
	irq_handler_unregister(&snt_irq_db);
		
	#if USE_TRIG0_IRQ
	irq_handler_unregister(&trig0_irq_db);
	if (trig0_irq_db.work) {
		PRINT_INFO("freeing TRIG0 workq");
		workqueue_free_work(trig0_irq_db.work);
		trig0_irq_db.work = NULL;
	}
	#endif

	#if USE_TRIG1_IRQ
	irq_handler_unregister(&trig1_irq_db);
	if (trig1_irq_db.work) {
		PRINT_INFO("freeing TRIG1 workq");
		workqueue_free_work(trig1_irq_db.work);
		trig1_irq_db.work = NULL;
	}
	#endif

	#if USE_TRIG2_IRQ
	irq_handler_unregister(&trig2_irq_db);
	if (trig2_irq_db.work) {
		PRINT_INFO("freeing TRIG2 workq");
		workqueue_free_work(trig2_irq_db.work);
		trig2_irq_db.work = NULL;
	}
	#endif

	#if USE_TRIG3_IRQ
	irq_handler_unregister(&trig3_irq_db);
	if (trig3_irq_db.work) {
		PRINT_INFO("freeing TRIG3 workq");
		workqueue_free_work(trig3_irq_db.work);
		trig3_irq_db.work = NULL;
	}
	#endif
	
	input_device_cleanup();
	workqueue_cleanup();
	memory_cleanup();
	PRINT_DEBUG("done");
	return 0;
}

/*
 * Module declaration
 */


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sentons, Inc.");
MODULE_DESCRIPTION("SPI/I2C Driver For Sentons SNT8100FSR Touch Device");
MODULE_VERSION(SNT_VERSION);
