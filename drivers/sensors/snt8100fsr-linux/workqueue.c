/*****************************************************************************
* File: workqueue.c
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
#include <linux/module.h>
#include <linux/kernel.h>

#include "workqueue.h"
#include "memory.h"
#include "config.h"
#include "debug.h"

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/
#define QUEUE_NAME "snt8100fsr-queue"

/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/

/*==========================================================================*/
/* LOCAL PROTOTYPES                                                         */
/*==========================================================================*/

/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/
static struct workqueue_struct *wq;

/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/
int workqueue_init_snt(void) {
	PRINT_FUNC();
	wq = create_workqueue(QUEUE_NAME);
	if (!wq) {
		PRINT_CRIT("Unable to create_workqueue(%s)", QUEUE_NAME);
		return -1;
	}

	return 0;
}

void workqueue_cleanup() {
	/* Wait for all the work in the queue to complete before destroying the work queue */
	flush_workqueue(wq);
	destroy_workqueue(wq);
	wq = NULL;
}

void *workqueue_alloc_work(size_t work_size, work_func_t work_func) {
	struct delayed_work *work = memory_allocate(work_size, GFP_KERNEL);
	if (!work) {
		PRINT_CRIT(OOM_STRING);
		return NULL;
	}

	memset(work, 0, work_size);

	//INIT_DELAYED_WORK((struct delayed_work *)work, work_func);
	return work;
}

void workqueue_free_work(void *work) {
	memory_free(work);
}

bool workqueue_queue_work(void *work, unsigned long delay) {
	//PRINT_FUNC("0x%p, %lums", work, delay);
	delay = msecs_to_jiffies(delay);
	if(wq==NULL){
		PRINT_INFO("workqueue_queue_work wq = NULL");
		return 0;
	} else if(work==NULL){
		PRINT_INFO("workqueue_queue_work work = NULL");
		return 0;
	}
	return queue_delayed_work(wq, work, delay);
	//return schedule_delayed_work(work, delay);
}

bool workqueue_mod_work(void *work, unsigned long delay) {
	//PRINT_FUNC("0x%p, %lums", work, delay);
	delay = msecs_to_jiffies(delay);
	return mod_delayed_work(wq, work, delay);
}

bool workqueue_cancel_work(void *work) {
	//PRINT_FUNC("0x%p", work);
	if(work==NULL){
		PRINT_INFO("workqueue_queue_work work = NULL");
		return 0;
	}
	return cancel_delayed_work(work);
}
