/*****************************************************************************
* File: memory.c
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/mutex.h>

#include "config.h"
#include "memory.h"
#include "debug.h"

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/
#define MEMORY_PREFIX       0xAA0000FF
#define MEMORY_POSTFIX      0xEE0000FF

/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/
struct allocation {
	void *buf;
	size_t size;
	struct list_head list;
};

/*==========================================================================*/
/* LOCAL PROTOTYPES                                                         */
/*==========================================================================*/
static void set_memory_prefix_and_postfix(void *buf, size_t size);
static bool is_memory_prefix_valid(void *buf);
static bool is_memory_postfix_valid(void *buf, size_t size);
static size_t get_memory_prefix_and_postfix_size(void);

/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/
static LIST_HEAD(alloc_list);
static DEFINE_MUTEX(lock);

/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/
void memory_cleanup(void) {
	struct allocation *a, *next;
	bool leak = false;

	// Free all tracked memory
	mutex_lock(&lock);
	list_for_each_entry_safe(a, next, &alloc_list, list) {
		/*
		* We are freeing this memory that would normally be leaked. Please
		* find the leak and free it there so that we don't do it here.
		*/
		PRINT_ERR("Possible Leak! Freeing 0x%p, %zu bytes",
				a->buf + sizeof(uint32_t),
				a->size - get_memory_prefix_and_postfix_size());

		leak = true;

		// Ensure this buffer has our memory prefix+postfix signatures,
		// don't do anything with the result, it will log on an error
		is_memory_prefix_valid(a->buf);
		is_memory_postfix_valid(a->buf, a->size);

		list_del(&a->list);
		kfree(a);
	}

	if (!leak) {
		PRINT_DEBUG("No memory leaks detected!");
	}

	mutex_unlock(&lock);
}

bool memory_validate(void) {
	bool valid = true;
	struct allocation *a;

	//PRINT_FUNC();

	// Free all tracked memory
	mutex_lock(&lock);
	list_for_each_entry(a, &alloc_list, list) {
		// Ensure this buffer has our memory prefix+postfix signatures,
		// don't do anything with the result, it will log on an error
		if (!is_memory_prefix_valid(a->buf) ||
			!is_memory_postfix_valid(a->buf, a->size)) {
				valid = false;
		}
	}
	mutex_unlock(&lock);

	//PRINT_DEBUG("%s", valid ? "true" : "false");
	return valid;
}

void *memory_allocate(size_t size, gfp_t flags) {
	struct allocation *a;
	int new_size;
	void *buf, *ret_buf;

	//PRINT_FUNC("%zu bytes, 0x%08X", size, flags);

	// Allocate 8 more bytes for a prefix and postfix signature
	new_size = size + (sizeof(uint32_t) * 2);
	buf = kzalloc(new_size, flags);
	if (buf == NULL) {
		PRINT_CRIT(OOM_STRING " %zu bytes, 0x%08X", size, flags);
		return NULL;
	}

	set_memory_prefix_and_postfix(buf, new_size);

	// Allocate our allocation structure to keep track of this allocation
	a = kmalloc(sizeof(struct allocation), GFP_KERNEL);
	if (a == NULL) {
		PRINT_CRIT(OOM_STRING " %zu bytes, 0x%08X",
				sizeof(struct allocation), GFP_KERNEL);
		kfree(buf);
		return NULL;
	}

	a->buf = buf;
	a->size = new_size;

	mutex_lock(&lock);
	list_add(&a->list, &alloc_list);
	mutex_unlock(&lock);

	ret_buf = buf + sizeof(uint32_t);

	//PRINT_DEBUG("0x%p", ret_buf);
	return ret_buf;
}

void memory_free(void *buf) {
	struct allocation *a, *next;
	bool found = false;
	void *buf_p;
	size_t size = 0;

	//PRINT_FUNC("0x%p", buf);

	// Backup our buffer by 4 bytes (sizeof uint32_t)
	buf_p = buf - sizeof(uint32_t);

	// Ensure this buffer has our memory prefix signature, don't do anything
	// with the result, it will log on an error
	if(is_memory_prefix_valid(buf_p)==false)
		return;

	// Iterate through our allocations to find this one
	mutex_lock(&lock);
	list_for_each_entry_safe(a, next, &alloc_list, list) {
		if (a->buf == buf_p) {
			found = true;
			size = a->size;

			// Ensure this buffer has our memory postfix signature,
			// don't do anything with the result, it will log on an error
			if(is_memory_postfix_valid(buf_p, a->size)==false)
				return;
			
			list_del(&a->list);
			kfree(a);
			break;
		}
	}
	mutex_unlock(&lock);

	if (!found) {
		PRINT_CRIT("Corruption, memory 0x%p was not being tracked!", buf);
		// Set our size to prefix/postfix size so we show 0 on method exit
		size = get_memory_prefix_and_postfix_size();
	}

	// Free the full buffer including the memory prefix and postfix signatures
	kfree(buf_p);

	// Display we freed the buffer the user passed in
	//PRINT_DEBUG("Free'd 0x%p, %zu bytes", buf,
	//			size - get_memory_prefix_and_postfix_size());
}

static size_t get_memory_prefix_and_postfix_size(void) {
	return (sizeof(uint32_t) * 2);
}

static void set_memory_prefix_and_postfix(void *buf, size_t size) {
	// Add the prefix to the beginning of the memory buffer
	((uint32_t *)buf)[0] = MEMORY_PREFIX;

	// Add the postfix to the end of the memory buffer
	*(uint32_t *)(&((uint8_t *)buf)[size - sizeof(uint32_t)]) =
			MEMORY_POSTFIX;
}

static bool is_memory_prefix_valid(void *buf) {
	uint32_t prefix = ((uint32_t *)buf)[0];
	if (prefix != MEMORY_PREFIX) {
		PRINT_CRIT("Memory corrupted, bad prefix: 0x%08x", prefix);
		return false;
	}

	return true;
}

static bool is_memory_postfix_valid(void *buf, size_t size) {
	uint32_t postfix = *(uint32_t *)(&((uint8_t *)buf)[size - sizeof(uint32_t)]);
	if (postfix != MEMORY_POSTFIX) {
		PRINT_CRIT("Memory corrupted, bad posfix: 0x%08x", postfix);
		return false;
	}

	return true;
}
