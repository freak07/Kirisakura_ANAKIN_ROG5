/*****************************************************************************
* File: hardware.h
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
#include "device.h"

#ifndef HARDWARE_H
#define HARDWARE_H

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/
/*
 * When adding a register, update the hardware.c register table as well
 *
 * Register addresses
 */
#define REGISTER_EVENT          0x0000
#define REGISTER_ENABLE         0x0001
#define REGISTER_FRAME_RATE     0x0002
#define REGISTER_PHY_STAT_LSB   0x0003
#define REGISTER_PHY_STAT_MSB   0x0004
#define REGISTER_ACTIONS        0x0005
#define REGISTER_IRPT_CFG       0x0006
#define REGISTER_IRPT_MASK      0x0007
#define REGISTER_I2C_CFG        0x0008
#define REGISTER_LOOPBACK       0x0009
#define REGISTER_CHIP_ID_LSB    0x000A
#define REGISTER_CHIP_ID_MSB    0x000B
#define REGISTER_BAR0_FSF       0x000C
#define REGISTER_BAR1_FSF       0x000D
#define REGISTER_BAR2_FSF       0x000E
#define REGISTER_BAR3_FSF       0x000F
#define REGISTER_BYPASS         0x002A
#define REGISTER_BAR_CTRL       0x002B
#define REGISTER_BNK_CFG_OFFSET 0x002C
#define REGISTER_FRP_MAX_SIZE   0x0046
#define REGISTER_TIRGGER_LINK1         0x001A
#define REGISTER_TIRGGER_LINK2         0x001B
#define REGISTER_TIRGGER3_PULSE_DUR         0x001D
#define REGISTER_TIRGGER3_CONTROL         0x0010
#define REGISTER_TIRGGER5_PULSE_DUR         0x001E
#define REGISTER_TIRGGER5_CONTROL         0x0011
#define REGISTER_SQUEEZE_FORCECALIB         0x0034

//Gesture Reg
#define REGISTER_SQUEEZE_CTL 0x006C
#define REGISTER_SQUEEZE_DUR 0x0071
#define REGISTER_SQUEEZE_UP 0x0079
#define REGISTER_SQUEEZE_DROP 0x007A
#define REGISTER_TAP1_CTL 0x0010
#define REGISTER_TAP1_DUR 0x0012
#define REGISTER_TAP1_FUP_FORCE 0x0016
#define REGISTER_TAP2_CTL 0x0018
#define REGISTER_TAP2_DUR 0x001A
#define REGISTER_TAP2_FUP_FORCE 0x001E
#define REGISTER_TAP3_CTL 0x0020
#define REGISTER_TAP3_DUR 0x0022
#define REGISTER_TAP3_FUP_FORCE 0x0026
#define REGISTER_SWIPE1_CTL 0x0060
#define REGISTER_SWIPE2_CTL 0x0064
#define REGISTER_SWIPE3_CTL 0x0068
#define REGISTER_SWIPE1_LEN 0x0063
#define REGISTER_SWIPE2_LEN 0x0067
#define REGISTER_SWIPE3_LEN 0x006b
//raw data
#define REGISTR_RAW_DATA 0x0040
//DPC Reg
#define REGISTER_DPC_HIGH_FRAME 0x0039
#define REGISTER_DPC_LOW_FRAME 0x003A
#define REGISTER_DPC_CONDITION 0x003B
//DPC Lowpower control
#define REGISTER_DPC_LOWPOWER 0x003D

// Max register value before FIFO's begin
#define MAX_REGISTER_ADDRESS    0x00FF

// FIFO Register Addresses
#define REGISTER_FIFO_TOUCH     0x0100
#define REGISTER_FIFO_CONFIG    0x0200
#define REGISTER_FIFO_COMMAND   0x0300
#define REGISTER_FIFO_STREAM    0x0400

// Width of the register's value in bytes
#define WIDTH_OF_REGISTER_VALUE 2

// The looback register's default value
#define LOOPBACK_REGISTER_DEFAULT_VALUE 0x7022

#define TOUCH_FIFO_LENGTH_WIDTH 2 // FIFO Length is 2 bytes wide
#define TOUCH_FIFO_FRAME_WIDTH  2 // FIFO Frame field is 2 bytes wide

#define CMD_FIFO_LENGTH_WIDTH   4

#define TRACK_REPORTS_MAX_NUM   18
#define TRACK_REPORTS_MAX_LEN   ((sizeof(struct track_report) * TRACK_REPORTS_MAX_NUM) + 2)
#define PRODUCT_CONFIG_MAX_LEN  1024

#define REG_PART_MAX_LEN        8192
#define DEEP_TRACE_BUF_LEN      4096

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/
struct register_entry {
    char *str;
    int n;
};

extern struct register_entry register_dict_g[];

/*==========================================================================*/
/* REGISTER STRUCTURES                                                      */
/*==========================================================================*/
//struct register_event {
struct register_event {
    uint16_t boot : 1;
    uint16_t touch : 1;
    uint16_t command : 1;
    uint16_t stream : 1;
    uint16_t host : 1;
    uint16_t test : 1;
    uint16_t rst : 1;
    uint16_t fwd : 1;
    uint16_t padding : 7;
    uint16_t fault : 1;
};

struct register_enable {
    uint16_t enable : 1;
    uint16_t padding : 15;
};

struct register_frame_rate {
    uint16_t rate;
};

struct register_phy_stat_lsb {
    uint16_t stat;
};

struct register_phy_stat_msb {
    uint16_t stat;
};

struct register_actions {
    uint16_t test : 1;
    uint16_t ftouch : 1;
    uint16_t fcfg : 1;
    uint16_t fcmd : 1;
    uint16_t fstr : 1;
    uint16_t rst : 1;
    uint16_t cref : 1;
    uint16_t flpm : 1;
    uint16_t padding : 8;
};

struct register_irpt_cfg {
    uint16_t hpol : 1;
    uint16_t hlvl : 1;
    uint16_t host_edge_duration : 5;
    uint16_t hdis : 1;
    uint16_t padding : 8;
};

struct register_i2c_cfg {
    uint16_t slave_addr : 8;
    uint16_t padding : 7;
    uint16_t wake_addr : 1;
};

/*==========================================================================*/
/* PROTOTYPES                                                               */
/*==========================================================================*/
int register_number_for_key(char *key);
void load_registers_from_disk(struct snt8100fsr *snt8100fsr);

#endif // HARDWARE_H
