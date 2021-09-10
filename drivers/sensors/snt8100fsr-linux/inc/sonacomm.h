/*****************************************************************************
* File: sonacomm.h
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
#ifndef SONACOMM_H
#define SONACOMM_H

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/
#define SC_MAX_FRAME_BUFFER         (16 * 1024) + (4) // 16k + 4 byte length
#define SC_MAX_PACKET_LENGTH        2048
#define SC_OVERHEAD                 16
#define SC_DATA_PAYLOAD_BYTES       (SC_MAX_PACKET_LENGTH - SC_OVERHEAD)

#define SC_MAGIC_NUMBER             0x12345679

/*
 * Major command codes
 */
#define mc_no_command        0
#define mc_d1_test           9
#define mc_frame_dump       13
#define mc_fw_update        19
#define mc_set_sys_param    22
#define mc_get_sys_param    23
#define mc_no_touch         32
#define mc_flash_update     40
#define mc_c2_loopback      48
#define mc_update_regs      50
#define mc_reg_script       51

/*
 * Minor command codes
 */
#define min_ok              1
#define min_data_frag       2
#define min_data_last       3
#define min_data_bad        4
#define min_abort           5
#define min_bad_part1       6
#define min_not_supported   7

#define FWUPDATE_STATUS_OKAY            0
#define FWUPDATE_STATUS_DATA_BAD        min_data_bad
#define FWUPDATE_STATUS_ABORT           min_abort
#define FWUPDATE_STATUS_NOT_SUPPORTED   min_not_supported
#define FWUPDATE_STATUS_FILE_ERR        100
#define FWUPDATE_STATUS_DRIVER_ERR      101

#define get_sys_param_event_log         63


/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/
struct __attribute__((__packed__)) sc_command {
    uint32_t   magic;
    uint32_t   address;
    uint32_t   length;
    uint16_t   major;
    uint8_t    minor;
    uint8_t    trans_id;
    uint32_t   data[SC_DATA_PAYLOAD_BYTES/sizeof(uint32_t)];
};

#define SC_COMMAND_HEADER_SIZE 16

/*==========================================================================*/
/* PROTOTYPES                                                               */
/*==========================================================================*/

#endif // SONACOMM_H
