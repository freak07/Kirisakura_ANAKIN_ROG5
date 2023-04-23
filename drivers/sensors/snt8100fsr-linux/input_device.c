/*****************************************************************************
* File: device.c
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

#include "customize.h"
#include "config.h"
#include "debug.h"
#include "input_device.h"

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/
#define INPUT_DEVICE_NAME   "snt8100fsr"
#define TIMER_DURATION      5000

/*==========================================================================*/
/* CONSTANTS                                                                */
/*=================================currently enabled SysFS device=========================================*/

/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/
static struct input_dev *input_dev;
static bool   input_dev_registered;

/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* INITIALIZATION                                                           */
/*==========================================================================*/
int input_device_init(void) {
	int err;

	PRINT_FUNC();

	input_dev_registered = false;

	input_dev = input_allocate_device();
	if (!input_dev) {
		PRINT_ERR("Not enough memory\n");
		return -ENOMEM;
	}

	input_dev->name = INPUT_DEVICE_NAME;

	// Call the customize.c layer to register which events we'll use
	register_input_events(input_dev);

	err = input_register_device(input_dev);
	if (err) {
		PRINT_ERR("Failed to register device: %d\n", err);
		input_device_cleanup();
		return err;
	}
	input_dev_registered = true;

	PRINT_DEBUG("success");
	return 0;
}

void input_device_cleanup(void) {
	PRINT_FUNC();
	if (input_dev) {
		if (input_dev_registered) {
			input_unregister_device(input_dev);
			input_dev_registered = false;
		}

		input_free_device(input_dev);
		input_dev = NULL;
	}

	PRINT_DEBUG("done");
}

struct input_dev * get_input_device(void) {
	return input_dev;
}
