/*****************************************************************************
* File: hardware.c
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
#include <linux/string.h>
#include <linux/limits.h>
#include <linux/errno.h>

#include "config.h"
#include "file.h"
#include "utils.h"
#include "device.h"
#include "memory.h"
#include "debug.h"
#include "hardware.h"

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* CONSTANTS                                                                */
/*==========================================================================*/

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/
struct register_entry register_dict_g[] = {
    {"event", REGISTER_EVENT},
    {"enable", REGISTER_ENABLE},
    {"frame_rate", REGISTER_FRAME_RATE},
    {"phy_stat_lsb", REGISTER_PHY_STAT_LSB},
    {"phy_stat_msb", REGISTER_PHY_STAT_LSB},
    {"actions", REGISTER_ACTIONS},
    {"irpt_cfg", REGISTER_IRPT_CFG},
    {"i2c_cfg", REGISTER_I2C_CFG},
    {"loopback", REGISTER_LOOPBACK},
    {"chipid_lsb", REGISTER_CHIP_ID_LSB},
    {"chipid_msb", REGISTER_CHIP_ID_MSB},
    {"bar0_fsf", REGISTER_BAR0_FSF},
    {"bar1_fsf", REGISTER_BAR1_FSF},
    {"bar2_fsf", REGISTER_BAR2_FSF},
    {"bar3_fsf", REGISTER_BAR3_FSF},
    {"bypass", REGISTER_BYPASS},
    {"bar_ctrl", REGISTER_BAR_CTRL},
    {0, 0}
};

/*==========================================================================*/
/* LOCAL PROTOTYPES                                                         */
/*==========================================================================*/
int load_register_from_disk(struct snt8100fsr *snt8100fsr, int reg, char *path);

/*==========================================================================*/
/* GLOBAL VARIABLES                                                         */
/*==========================================================================*/

/*==========================================================================*/
/* METHODS                                                                  */
/*==========================================================================*/
/*
 * load_registers_from_disk()
 *
 * Loads registers 0 through MAX_REGISTER_ADDRESS from disk. If the register
 * file doesn't exist, no message is displayed. If it does exist, its value
 * will be loaded and then sent to the snt8100fsr hardware device immedietely.
 */
void load_registers_from_disk(struct snt8100fsr *snt8100fsr) {
    int         i, ret;
    char        *full_path;
    char        *reg_name;

    PRINT_FUNC("%s", REGISTER_FILES_PATH);

    PRINT_DEBUG("Searching for '%s0' through '%s%d'",
                REGISTER_PREFIX,
                REGISTER_PREFIX,
                MAX_REGISTER_ADDRESS);

    full_path = memory_allocate(PATH_MAX, 0);

    // Search for numerical register files, e.g. "register_13"
    for(i = 0; i < MAX_REGISTER_ADDRESS + 1; i++) {
        snprintf(full_path, PATH_MAX, "%s%s%d",
                 REGISTER_FILES_PATH, REGISTER_PREFIX, i);

        load_register_from_disk(snt8100fsr, i, full_path);
    }

    PRINT_DEBUG("Finshed. Searching for labeled register files.");

    // Search for labeled register files, e.g. "register_frame_rate"
    i = 0;
    reg_name = register_dict_g[i].str;
    while (reg_name) {
        strcpy(full_path, REGISTER_FILES_PATH);
        strcat(full_path, REGISTER_PREFIX);
        strcat(full_path, reg_name);

        ret = load_register_from_disk(snt8100fsr,
                                      register_dict_g[i].n,
                                      full_path);
        if (ret == -ENOENT) {
            PRINT_DEBUG("'%s%s' not found", REGISTER_PREFIX, reg_name);
        }

        reg_name = register_dict_g[++i].str;
    }

    memory_free(full_path);
    PRINT_DEBUG("done");
}

int load_register_from_disk(struct snt8100fsr *snt8100fsr, int reg, char *path) {
    int         n, ret;
    char        data[16];
    uint32_t    reg_value;
    struct file *f;
	
	PRINT_FUNC();
	
    // Open the file
    ret = file_open(path, O_RDONLY, 0, &f);
    if (ret == -ENOENT) {
        // Do nothing, file not found. Return code will be returned at the end.
    } else if (ret) {
        PRINT_ERR("'%s' unopenable, error %d", path, ret);
    } else {
        // Load the contents
        n = file_read(f, 0, (void *)data, sizeof(data) - 1);
        if (n <= 0) {
            PRINT_ERR("'%s' has no contents", path);
        } else {
            // Ensure we have a null terminator
            data[n] = 0;

            // Parse the field and write the register if successful
            if (string_to_uint32(data, &reg_value) == 0) {
                PRINT_INFO("'%s' set to 0x%X", path, reg_value);
                ret = write_register(snt8100fsr, reg, &reg_value);
                if (ret) {
                    PRINT_ERR("Unable to write register 0x%02X as value 0x%X (%d)",
                              reg, reg_value, reg_value);
                    ret = -1; // for final return
                }

                // If we loaded the frame rate, we track this variable
                if (reg == REGISTER_FRAME_RATE) {
                    if (reg_value > 5 && reg_value != 0xFFFF) {
                        snt8100fsr->frame_rate = reg_value;
                    }
                }
            } else {
                PRINT_ERR("'%s' has invalid contents", path);
            }
        }

        file_close(f);
    }

    return ret;
}

int register_number_for_key(char *key)
{
    int i = 0;
    char *name = register_dict_g[i].str;
    while (name) {
        if (strcmp(name, key) == 0)
            return register_dict_g[i].n;
        name = register_dict_g[++i].str;
    }
    return 0;
}
