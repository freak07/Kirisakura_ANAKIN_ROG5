/*
*	ES928x ALSA Codec Driver
*	Copyright (C) 2019 Samuel Schaefer <schaefer.samuel@gmail.com>, ESS Technology Inc.
*		
*	
*	This program is free software; you can redistribute it and/or modify
*	it under the terms of the GNU General Public License as published by
*	the Free Software Foundation; either version 2 of the License, or
*	(at your option) any later version.
*	
*	This program is distributed in the hope that it will be useful,
*	but WITHOUT ANY WARRANTY; without even the implied warranty of
*	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*	GNU General Public License for more details.
*	
*	You should have received a copy of the GNU General Public License along
*	with this program; if not, write to the Free Software Foundation, Inc.,
*	51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#ifndef _ES928X_H
#define _ES928X_H

#define ES928X_CHAR_DEV_DRV

/* System Registers */
#define ES928X_REG_SYS_BASIC_SYSTEM_CTRL                   0
#define ES928X_REG_SYS_EXP_BITS                            1
#define ES928X_REG_SYS_CLK_CTRL_SIGS                       4
#define ES928X_REG_SYS_CLK_SEL_SIGS                        5

/* GPIO Registers */
#define ES928X_REG_IO_GPIO_1_2_CFG                         6
#define ES928X_REG_IO_GPIO_3_4_CFG                         7
#define ES928X_REG_IO_GPIO_5_6_CFG                         8
#define ES928X_REG_IO_GPIO_7_8_CFG                         9
#define ES928X_REG_IO_GPIO_9_10_CFG                       10
#define ES928X_REG_IO_GPIO_11_12_CFG                      11
#define ES928X_REG_IO_GPIO_13_14_CFG                      12
#define ES928X_REG_IO_GPIO_INV_1                          13
#define ES928X_REG_IO_GPIO_INV_2                          14
#define ES928X_REG_IO_INT_MASK_1                          15
#define ES928X_REG_IO_INT_MASK_2                          16
#define ES928X_REG_IO_INT_CLEAR_1                         17
#define ES928X_REG_IO_INT_CLEAR_2                         18

/* Class G Registers */
#define ES928X_REG_CG_CLASS_G_CFG                         19
#define ES928X_REG_CG_CLASS_G_LATCH_MODE_DECAY_DELAY      20
#define ES928X_REG_CG_CLASS_G_THRS                        21
#define ES928X_REG_CG_ENSM_DELAY_CFG                      22
#define ES928X_REG_CG_CLASS_G_AMP_CTRL                    23

/* Amp Control Registers */
#define ES928X_REG_AMP_SHUNT_POWER_SUPPLY_CTRL            24
#define ES928X_REG_AMP_CHARGE_PUMP_AMP_CFG                25
#define ES928X_REG_AMP_AMP_ENS                            26
#define ES928X_REG_AMP_OVER_CURR_PROT                     27

/* DAC Registers */
#define ES928X_REG_DAC_IN_SEL_REGS                        28
#define ES928X_REG_DAC_SPDIF_REGS                         29
#define ES928X_REG_DAC_DSD_DOP_OP_DAC_MIX_SEL             30
#define ES928X_REG_DAC_MASTER_MODE_CFG                    31
#define ES928X_REG_DAC_MQB_REG_1                          32
#define ES928X_REG_DAC_VOL_1                              36
#define ES928X_REG_DAC_VOL_2                              37
#define ES928X_REG_DAC_MASTER_TRIM_1                      38
#define ES928X_REG_DAC_MASTER_TRIM_2                      39
#define ES928X_REG_DAC_MASTER_TRIM_3                      40
#define ES928X_REG_DAC_VOL_CTRL_OPTIONS                   41
#define ES928X_REG_DAC_AUTOMUTE_TIME                      42
#define ES928X_REG_DAC_AUTOMUTE_LEVEL                     43
#define ES928X_REG_DAC_AUTOMUTE_DRE_CFG                   44
#define ES928X_REG_DAC_DRE_CFG                            45
#define ES928X_REG_DAC_DRE_GAIN_1_1                       46
#define ES928X_REG_DAC_DRE_GAIN_1_2                       47
#define ES928X_REG_DAC_DRE_GAIN_2_1                       48
#define ES928X_REG_DAC_DRE_GAIN_2_2                       49
#define ES928X_REG_DAC_DRE_OFF_THRS                       50
#define ES928X_REG_DAC_DRE_ON_THRS                        51
#define ES928X_REG_DAC_FILTER_SET                         52
#define ES928X_REG_DAC_PROG_COEFF_ADDR                    53
#define ES928X_REG_DAC_PROG_FIR_RAM_DATA_1                54
#define ES928X_REG_DAC_PROG_FIR_RAM_DATA_2                55
#define ES928X_REG_DAC_PROG_FIR_RAM_DATA_3                56
#define ES928X_REG_DAC_THD_IIR                            57
#define ES928X_REG_DAC_THD_COMP_C2_CH1_1                  58
#define ES928X_REG_DAC_THD_COMP_C2_CH1_2                  59
#define ES928X_REG_DAC_THD_COMP_C3_CH1_1                  60
#define ES928X_REG_DAC_THD_COMP_C3_CH1_2                  61
#define ES928X_REG_DAC_THD_COMP_C2_CH2_1                  62
#define ES928X_REG_DAC_THD_COMP_C2_CH2_2                  63
#define ES928X_REG_DAC_THD_COMP_C3_CH2_1                  64
#define ES928X_REG_DAC_THD_COMP_C3_CH2_2                  65
#define ES928X_REG_DAC_SYNC_SET                           66
#define ES928X_REG_DAC_ASRC_DPLL_BW                       67
#define ES928X_REG_DAC_DPLL_LOCK_SPEED_PHASE              68
#define ES928X_REG_DAC_FORCE_DPLL_NUM                     69
#define ES928X_REG_DAC_SOFT_START_CFG                     73
#define ES928X_REG_DAC_SOFT_START_CUST                    74
#define ES928X_REG_DAC_NSMOD_CFG_1                        75
#define ES928X_REG_DAC_NSMOD_CFG_2                        76
#define ES928X_REG_DAC_CLK_GEAR                           77
#define ES928X_REG_DAC_AUTO_CLK_GEAR_THRS_1               78
#define ES928X_REG_DAC_AUTO_CLK_GEAR_THRS_2               79
#define ES928X_REG_DAC_AUTO_CLK_GEAR_THRS_3               80
#define ES928X_REG_DAC_ANALOG_VOL_CTRL_1                  81
#define ES928X_REG_DAC_ANALOG_VOL_CTRL_2                  82
#define ES928X_REG_DAC_CHARGE_PUMP_CLK_1                  83
#define ES928X_REG_DAC_CHARGE_PUMP_CLK_2                  84

/* ADC Registers */
#define ES928X_REG_ADC_VOL_1                              85
#define ES928X_REG_ADC_VOL_2                              86
#define ES928X_REG_ADC_VOL_3                              87
#define ES928X_REG_ADC_VOL_4                              88
#define ES928X_REG_ADC_VOL_RATE_CH_INV                    90
#define ES928X_REG_ADC_DITHER_MASK_1                      91
#define ES928X_REG_ADC_DITHER_MASK_2                      92
#define ES928X_REG_ADC_CFG                                93
#define ES928X_REG_ADC_IIR_SET_I2S_MODE                   94
#define ES928X_REG_ADC_FILTER_CTRL                        95
#define ES928X_REG_ADC_CH_IN_SEL_1                        96
#define ES928X_REG_ADC_CH_IN_SEL_2                        97
#define ES928X_REG_ADC_FS_RATIOS                          98
#define ES928X_REG_ADC_CLK_EN_ADC_RUN_VOL                 99
#define ES928X_REG_ADC_ANALOG_CTRLS_A                    100
#define ES928X_REG_ADC_ANALOG_CTRLS_B                    101
#define ES928X_REG_ADC_CLK_SEL_ADC_EN                    102
#define ES928X_REG_ADC_PDM_CLK_VREF_SEL                  103
#define ES928X_REG_ADC_DEC_RATE                          104

/* AUX Mix Registers */
#define ES928X_REG_ASP_AUX_ASP_STEP_SIZE_1               105
#define ES928X_REG_ASP_AUX_ASP_STEP_SIZE_2               106
#define ES928X_REG_ASP_AUX_ASP_CUSTOM_ADDR               107

/* MUX and V2I Registers */
#define ES928X_REG_MUX_MUX_ENABLES                       108
#define ES928X_REG_MUX_ADC_L_MUX_CFG                     109
#define ES928X_REG_MUX_ADC_R_MUX_CFG                     110
#define ES928X_REG_MUX_ADC_MIC_MUX_CFG                   111

/* JDET Registers */
#define ES928X_REG_JDET_JDET_IDET_CFG_1                  112
#define ES928X_REG_JDET_CONFIG                           113
#define ES928X_REG_JDET_MICBIAS_POWER_DOWN_LOW           119
#define ES928X_REG_JDET_MICBIAS_POWER_DOWN_HIGH          120

/* PLL Registers */
#define ES928X_REG_PLL_VCO_CTRL                          122
#define ES928X_REG_PLL_CFG_DIV_FB                        123
#define ES928X_REG_PLL_CFG_DIV_IN_1                      124
#define ES928X_REG_PLL_CFG_DIV_IN_2                      125
#define ES928X_REG_PLL_CFG_DIV_OUT_1                     126
#define ES928X_REG_PLL_CFG_DIV_OUT_2                     127

/* ASP Registers */
#define ES928X_REG_ASP_EN                                128
#define ES928X_REG_ASP_PROGRAM                           129
#define ES928X_REG_ASP_PROGRAM_ADDR                      130
#define ES928X_REG_ASP_DATA                              131
#define ES928X_REG_ASP_DATA_ADDR                         139

/* PWM Registers */
#define ES928X_REG_PWM_COUNT_1                           140
#define ES928X_REG_PWM_FREQ_1                            141
#define ES928X_REG_PWM_COUNT_2                           142
#define ES928X_REG_PWM_FREQ_2                            143
#define ES928X_REG_PWM_COUNT_3                           144
#define ES928X_REG_PWM_FREQ_3                            145

/* DAC2 Registers */
#define ES928X_REG_DAC_XTALK_CFG                         146
#define ES928X_REG_DAC_XTALK_SCALE_CH1_1                 147
#define ES928X_REG_DAC_XTALK_SCALE_CH1_2                 148
#define ES928X_REG_DAC_XTALK_SCALE_CH2_1                 149
#define ES928X_REG_DAC_XTALK_SCALE_CH2_2                 150
#define ES928X_REG_DAC_DC_OFFSET_CH1_1                   151
#define ES928X_REG_DAC_DC_OFFSET_CH1_2                   152
#define ES928X_REG_DAC_DC_OFFSET_CH2_1                   153
#define ES928X_REG_DAC_DC_OFFSET_CH2_2                   154
#define ES928X_REG_DAC_DC_OFFSET_RAMP_RATE               155
#define ES928X_REG_DAC_MQA_GROUP_DELAY                   156
#define ES928X_REG_DAC_ASP_CUSTOM_RAM1_ADDR_DAC          157
#define ES928X_REG_DAC_ASP_CUSTOM_RAM2_ADDR_DAC          158
#define ES928X_REG_DAC_ASP_CUSTOM_ADDR2_ANC              159
#define ES928X_REG_DAC_ASP_STEP_SIZE_DAC_1               160
#define ES928X_REG_DAC_ASP_STEP_SIZE_DAC_2               161

/* ADC2 Registers */
#define ES928X_REG_ADC_ASP_CFG                           163
#define ES928X_REG_ADC_PEAK_DETECTOR_CFG                 164
#define ES928X_REG_ADC_PEAK_THRES_1                      165
#define ES928X_REG_ADC_PEAK_THRES_2                      166

/* Analog Control Registers */
#define ES928X_REG_MUX_PGA_IN_MUX                        167
#define ES928X_REG_MUX_EXP_BITS1                         168
#define ES928X_REG_MUX_EXP_OUT_1                         169
#define ES928X_REG_MUX_EXP_OUT_2                         170

/* Read Only Registers */
#define ES928X_REG_RB_OCP_INT_RB                         192
#define ES928X_REG_RB_PLL_VCO_CTRL                       193
#define ES928X_REG_RB_PLL_CFG_DIV_FB                     194
#define ES928X_REG_RB_PLL_CFG_DIV_IN_1                   195
#define ES928X_REG_RB_PLL_CFG_DIV_IN_2                   196
#define ES928X_REG_RB_PLL_CFG_DIV_OUT_1                  197
#define ES928X_REG_RB_PLL_CFG_DIV_OUT_2                  198
#define ES928X_REG_RB_AUX_DAC_BUS_RB                     199
#define ES928X_REG_RB_DPLL_NUM_RB_1                      200
#define ES928X_REG_RB_DPLL_NUM_RB_2                      201
#define ES928X_REG_RB_DPLL_NUM_RB_3                      202
#define ES928X_REG_RB_DPLL_NUM_RB_4                      203
#define ES928X_REG_RB_DAC_LG_PL_RB_1                     204
#define ES928X_REG_RB_DAC_LG_PL_RB_2                     205
#define ES928X_REG_RB_DAC_LG_PL_RB_3                     206
#define ES928X_REG_RB_DAC_LG_PL_RB_4                     207
#define ES928X_REG_RB_DAC_LG_PL_RB_5                     208
#define ES928X_REG_RB_DAC_LG_PL_RB_6                     209
#define ES928X_REG_RB_DAC_LG_PL_RB_7                     210
#define ES928X_REG_RB_DAC_LG_PL_RB_8                     211
#define ES928X_REG_RB_GPIO_IN_RB_1                       212
#define ES928X_REG_RB_GPIO_IN_RB_2                       213
#define ES928X_REG_RB_INT_STATE_RB_1                     214
#define ES928X_REG_RB_INT_STATE_RB_2                     215
#define ES928X_REG_RB_CHIP_ID_RB                         220
#define ES928X_REG_MCU_LOAD_MIC_STATUS                   221
#define ES928X_REG_MCU_DAC_IMP_CODE_L                    222
#define ES928X_REG_MCU_DAC_IMP_CODE_R                    223
#define ES928X_REG_MCU_MIC_IMP_CODE                      224
#define ES928X_REG_MCU_BUT_IMP_CODE                      225
#define ES928X_REG_MCU_FW_VERSION_MAJOR                  226
#define ES928X_REG_MCU_FW_VERSION_MINOR                  227
#define ES928X_REG_MCU_FW_VERSION_LSB_BUILD              228
#define ES928X_REG_MCU_FW_VERSION_MSB_BUILD              229
#define ES928X_REG_MCU_INTERRUPT_FLAGS                   230
#define ES928X_REG_MCU_BUTTON_STATUS                     231
#define ES928X_REG_MCU_MASTER_CTRL                       232
#define ES928X_REG_MCU_FW_CNTR                           238
#define ES928X_REG_MCU_FW_STATUS                         239
#define ES928X_REG_MCU_FW_DATA                           240
#define ES928X_REG_RB_ADC_PEAK_RB_1                      241
#define ES928X_REG_RB_ADC_PEAK_RB_2                      242
#define ES928X_REG_RB_ADC_PEAK_RB_3                      243
#define ES928X_REG_RB_ADC_PEAK_INT_RB                    247
#define ES928X_REG_RB_DRE_STATES_RB                      248
#define ES928X_REG_RB_MQB_AUTHORIZATION_STATE_RB         249
#define ES928X_REG_REV_ID_RB                             251
#define ES928X_REG_FW_CUST_STATUS                        252
#define ES928X_REG_FW_CUST_WRITE_ADDR_L                  253
#define ES928X_REG_FW_CUST_WRITE_ADDR_H                  254
#define ES928X_REG_FW_CUST_WRITE_DATA                    255

/* Synchronous I2C Interface Registers (0x94 or 0x96) */
#define ES928X_REG_SYNC_PLL_VCO_CTRL				     128
#define ES928X_REG_SYNC_PLL_CFG_DIV_FB                   129
#define ES928X_REG_SYNC_PLL_CFG_DIV_IN_1                 130
#define ES928X_REG_SYNC_PLL_CFG_DIV_IN_2                 131
#define ES928X_REG_SYNC_PLL_CFG_DIV_OUT_1                132
#define ES928X_REG_SYNC_PLL_CFG_DIV_OUT_2                133


// Register 0
#define ES928X_OSC_DRV_SHIFT                                4
#define ES928X_OSC_DRV_MASK                              0xF0
#define ES928X_DVDD_SHUNT_ENB_MASK                       0x08
#define ES928X_SEL1V_MASK                                0x04
#define ES928X_PD_XOSC_MASK                              0x02
// Register 1
#define ES928X_MIC_BIAS_SHUNT_MASK                       0x01
// Register 4
#define ES928X_SEL_DAC_CLKIN_SHIFT                          6
#define ES928X_SEL_DAC_CLKIN_MASK                        0xC0
#define ES928X_ENB_DAC_CLKIN_MASK                        0x20
#define ES928X_ENB_ADC_CLKIN_MASK                        0x10
// Register 5
#define ES928X_SEL_ACLK_IN2_MASK                         0x08
#define ES928X_SEL_ACLK_IN1_MASK                         0x04
#define ES928X_MCU_CONFIGURE_PLL_MASK                    0x01
// Register 6
#define ES928X_GPIO2_CFG_SHIFT                              4
#define ES928X_GPIO2_CFG_MASK                            0xF0
#define ES928X_GPIO1_CFG_SHIFT                              0
#define ES928X_GPIO1_CFG_MASK                            0x0F
// Register 7
#define ES928X_GPIO4_CFG_SHIFT                              4
#define ES928X_GPIO4_CFG_MASK                            0xF0
#define ES928X_GPIO3_CFG_SHIFT                              0
#define ES928X_GPIO3_CFG_MASK                            0x0F
// Register 8
#define ES928X_GPIO6_CFG_SHIFT                              4
#define ES928X_GPIO6_CFG_MASK                            0xF0
#define ES928X_GPIO5_CFG_SHIFT                              0
#define ES928X_GPIO5_CFG_MASK                            0x0F
// Register 9
#define ES928X_GPIO8_CFG_SHIFT                              4
#define ES928X_GPIO8_CFG_MASK                            0xF0
#define ES928X_GPIO7_CFG_SHIFT                              0
#define ES928X_GPIO7_CFG_MASK                            0x0F
// Register 10
#define ES928X_GPIO10_CFG_SHIFT                             4
#define ES928X_GPIO10_CFG_MASK                           0xF0
#define ES928X_GPIO9_CFG_SHIFT                              0
#define ES928X_GPIO9_CFG_MASK                            0x0F
// Register 11
#define ES928X_GPIO12_CFG_SHIFT                             4
#define ES928X_GPIO12_CFG_MASK                           0xF0
#define ES928X_GPIO11_CFG_SHIFT                             0
#define ES928X_GPIO11_CFG_MASK                           0x0F
// Register 12
#define ES928X_GPIO14_CFG_SHIFT                             4
#define ES928X_GPIO14_CFG_MASK                           0xF0
#define ES928X_GPIO13_CFG_SHIFT                             0
#define ES928X_GPIO13_CFG_MASK                           0x0F
// Register 13-14
#define ES928X_GPIO_INTERRUPT_ENABLE_MASK                0x80
#define ES928X_INVERT_GPIO_14_MASK                       0x20
#define ES928X_INVERT_GPIO_13_MASK                       0x10
#define ES928X_INVERT_GPIO_12_MASK                       0x08
#define ES928X_INVERT_GPIO_11_MASK                       0x04
#define ES928X_INVERT_GPIO_10_MASK                       0x02
#define ES928X_INVERT_GPIO_9_MASK                        0x01
#define ES928X_INVERT_GPIO_8_MASK                        0x80
#define ES928X_INVERT_GPIO_7_MASK                        0x40
#define ES928X_INVERT_GPIO_6_MASK                        0x20
#define ES928X_INVERT_GPIO_5_MASK                        0x10
#define ES928X_INVERT_GPIO_4_MASK                        0x08
#define ES928X_INVERT_GPIO_3_MASK                        0x04
#define ES928X_INVERT_GPIO_2_MASK                        0x02
#define ES928X_INVERT_GPIO_1_MASK                        0x01
// Register 15-16
#define ES928X_IM_DAC_CLKEN_1FS_MASK                     0x40
#define ES928X_IM_AUX_MIX_CLKEN_1FS_MASK                 0x20
#define ES928X_IM_MCU_MASK                               0x10
#define ES928X_IM_AUTOMUTE_STATUS_CH2_MASK               0x08
#define ES928X_IM_AUTOMUTE_STATUS_CH1_MASK               0x04
#define ES928X_IM_SOFT_START_DONE_MASK                   0x02
#define ES928X_IM_LOCK_STATUS_MASK                       0x01
// Register 17-18
#define ES928X_IC_DAC_CLKEN_1FS_MASK                     0x40
#define ES928X_IC_AUX_MIX_CLKEN_1FS_MASK                 0x20
#define ES928X_IC_MCU_MASK                               0x10
#define ES928X_IC_AUTOMUTE_STATUS_CH2_MASK               0x08
#define ES928X_IC_AUTOMUTE_STATUS_CH1_MASK               0x04
#define ES928X_IC_SOFT_START_DONE_MASK                   0x02
#define ES928X_IC_LOCK_STATUS_MASK                       0x01
// Register 19
#define ES928X_CG_DECAY_RATE_SHIFT                          4
#define ES928X_CG_DECAY_RATE_MASK                        0xF0
#define ES928X_CG_UNLATCH_ON_MUTE_MASK                   0x02
#define ES928X_CG_ENABLE_MASK                            0x01
// Register 20
#define ES928X_CG_LATCH_MODE_MASK                        0x80
#define ES928X_CG_DECAY_DELAY_SHIFT                         0
#define ES928X_CG_DECAY_DELAY_MASK                       0x7F
// Register 21
#define ES928X_CG_THRESHOLD_SHIFT                           0
#define ES928X_CG_THRESHOLD_MASK                         0xFF
// Register 22
#define ES928X_ENSM_POSTDELAY_SHIFT                         4
#define ES928X_ENSM_POSTDELAY_MASK                       0xF0
#define ES928X_ENSM_PREDELAY_SHIFT                          0
#define ES928X_ENSM_PREDELAY_MASK                        0x0F
// Register 23
#define ES928X_CG_AMP_MODE_SHIFT                            4
#define ES928X_CG_AMP_MODE_MASK                          0x30
#define ES928X_ABG_TUNE_HIGH_SHIFT                          2
#define ES928X_ABG_TUNE_HIGH_MASK                        0x0C
#define ES928X_ABG_TUNE_LOW_SHIFT                           0
#define ES928X_ABG_TUNE_LOW_MASK                         0x03
// Register 24
#define ES928X_AREG_PDB_MASK                             0x80
#define ES928X_SEL3V3_CPH_MASK                           0x40
#define ES928X_SEL3V3_PS_MASK                            0x20
#define ES928X_SHTINB_MASK                               0x10
#define ES928X_SHTOUTBR_MASK                             0x08
#define ES928X_SHTOUTBL_MASK                             0x04
#define ES928X_ABG_TUNE_SHIFT                               0
#define ES928X_ABG_TUNE_MASK                             0x03
// Register 25
#define ES928X_HPA_HIQ_MASK                              0x80
#define ES928X_LBHPA_MASK                                0x40
#define ES928X_ENCPH_MASK                                0x10
#define ES928X_ENCPL_MASK                                0x08
#define ES928X_ENSM_CPH_MASK                             0x04
#define ES928X_ENSM_CPL_MASK                             0x02
#define ES928X_ENSM_PS_MASK                              0x01
// Register 26
#define ES928X_HPA_FB_AMP_PDB_R_MASK                     0x80
#define ES928X_HPA_FB_AMP_PDB_L_MASK                     0x40
#define ES928X_ENCP_OE_MASK                              0x20
#define ES928X_AREF_FC_LN_MASK                           0x10
#define ES928X_ENHPA_MASK                                0x08
#define ES928X_ENHPA_OUT_MASK                            0x04
#define ES928X_ENFCB_MASK                                0x02
#define ES928X_APDB_MASK                                 0x01
// Register 27
#define ES928X_OC_SD_GAIN_SHIFT                             1
#define ES928X_OC_SD_GAIN_MASK                           0x7E
#define ES928X_OC_SD_EN_MASK                             0x01
// Register 28
#define ES928X_DAC_SERIAL_LENGTH_SHIFT                      6
#define ES928X_DAC_SERIAL_LENGTH_MASK                    0xC0
#define ES928X_DAC_SERIAL_MODE_SHIFT                        4
#define ES928X_DAC_SERIAL_MODE_MASK                      0x30
#define ES928X_DAC_AUTO_SELECT_SHIFT                        2
#define ES928X_DAC_AUTO_SELECT_MASK                      0x0C
#define ES928X_DAC_INPUT_SELECT_SHIFT                       0
#define ES928X_DAC_INPUT_SELECT_MASK                     0x03
// Register 29
#define ES928X_DAC_SPDIF_SEL_SHIFT                          4
#define ES928X_DAC_SPDIF_SEL_MASK                        0xF0
#define ES928X_DAC_SPDIF_LOAD_USER_BITS_MASK             0x04
#define ES928X_DAC_IGNORE_DATA_BIT_MASK                  0x02
#define ES928X_DAC_SPDIF_IGNORE_VALID_BIT_MASK           0x01
// Register 30
#define ES928X_DAC_USE_USB_BRIDGE_MASK                   0x80
#define ES928X_DAC_MONO_MODE_MASK                        0x40
#define ES928X_DAC_DSD_1024_MASK                         0x20
#define ES928X_DAC_DOP_ENABLE_MASK                       0x10
#define ES928X_DAC_CH2_MIX_SEL_MASK                      0x08
#define ES928X_DAC_CH1_MIX_SEL_MASK                      0x02
// Register 31
#define ES928X_DAC_PAYLOAD_SELECT_SHIFT                     6
#define ES928X_DAC_PAYLOAD_SELECT_MASK                   0xC0
#define ES928X_ADC_MIX_ENABLE_RIGHT_MASK                 0x20
#define ES928X_ADC_MIX_ENABLE_LEFT_MASK                  0x10
#define ES928X_DAC_MASTER_MODE_ENABLE_MASK               0x08
#define ES928X_DAC_MASTER_MODE_DIV_SHIFT                    0
#define ES928X_DAC_MASTER_MODE_DIV_MASK                  0x03
// Register 32
#define ES928X_DAC_MQA_ENABLE_MASK                       0x80
// Register 36
#define ES928X_DAC_VOLUME1_SHIFT                            0
#define ES928X_DAC_VOLUME1_MASK                          0xFF
// Register 37
#define ES928X_DAC_VOLUME2_SHIFT                            0
#define ES928X_DAC_VOLUME2_MASK                          0xFF
// Register 38-40  (24-bit)
#define ES928X_MASTER_TRIM_SHIFT                            0
// Register 41
#define ES928X_DAC_VOL_RATE_SHIFT                           4
#define ES928X_DAC_VOL_RATE_MASK                         0xF0
#define ES928X_DAC_USE_CH1_VOLUME_MASK                   0x08
#define ES928X_DAC_LATCH_VOLUME_MASK                     0x04
#define ES928X_DAC_MUTE_RIGHT_MASK                       0x02
#define ES928X_DAC_MUTE_LEFT_MASK                        0x01
// Register 42
#define ES928X_AUTOMUTE_TIME_SHIFT                          0
#define ES928X_AUTOMUTE_TIME_MASK                        0xFF
// Register 43
#define ES928X_AUTOMUTE_LEVEL_SHIFT                         0
#define ES928X_AUTOMUTE_LEVEL_MASK                       0x7F
// Register 44
#define ES928X_DAC_DRE_FORCE2_MASK                       0x80
#define ES928X_DAC_DRE_FORCE1_MASK                       0x40
#define ES928X_DAC_DRE_DITHER_BYPASS_MASK                0x20
#define ES928X_DAC_DRE_VOLUME_CTRL_MASK                  0x10
#define ES928X_DAC_DRE_DISABLE_FB_AMP_MASK               0x08
#define ES928X_DAC_AUTOMUTE_TIMER_FAST_MASK              0x04
#define ES928X_DAC_AUTOMUTE_CONFIG_SHIFT                    0
#define ES928X_DAC_AUTOMUTE_CONFIG_MASK                  0x03
// Register 45
#define ES928X_DAC_DRE_ENABLE_MASK                       0x80
#define ES928X_DAC_DRE_ENABLE_PEAK_FILTER_MASK           0x40
#define ES928X_DAC_DRE_DECAY_RATE_SHIFT                     0
#define ES928X_DAC_DRE_DECAY_RATE_MASK                   0x3F
// Register 46-47 (16bit)
#define ES928X_DAC_DRE_GAIN1_SHIFT                          0
// Register 48-49 (16bit)
#define ES928X_DAC_DRE_GAIN2_SHIFT                          0
// Register 50
#define ES928X_DAC_DRE_OFF_THRESHOLD_SHIFT                  0
#define ES928X_DAC_DRE_OFF_THRESHOLD_MASK                0x7F
// Register 51
#define ES928X_DAC_DRE_MIN_PEAK_MASK                     0x80
#define ES928X_DAC_DRE_ON_THRESHOLD_SHIFT                   0
#define ES928X_DAC_DRE_ON_THRESHOLD_MASK                 0x7F
// Register 52
#define ES928X_DAC_BYPASS_STAGE1_MASK                    0x80
#define ES928X_DAC_BYPASS_OSF_MASK                       0x40
#define ES928X_DAC_FORCE_SCALE_2X_MASK                   0x20
#define ES928X_DAC_PROG_COEFF_WE_MASK                    0x10
#define ES928X_DAC_PROG_COEFF_EN_MASK                    0x08
#define ES928X_DAC_FILTER_SHAPE_SHIFT                       0
#define ES928X_DAC_FILTER_SHAPE_MASK                     0x07
// Register 53
#define ES928X_DAC_PROG_COEFF_STAGE_MASK                 0x80
#define ES928X_DAC_PROG_COEFF_ADDR_SHIFT                    0
#define ES928X_DAC_PROG_COEFF_ADDR_MASK                  0x7F
// Register 54-56 (24-bit)
#define ES928X_DAC_PROG_COEFF_IN_SHIFT                      0
// Register 57
#define ES928X_DAC_ENABLE_SEPARATE_THD_COMP_MASK         0x80
#define ES928X_DAC_BYPASS_THD_MASK                       0x40
#define ES928X_DAC_BYPASS_IIR_MASK                       0x20
#define ES928X_DAC_ASP_ENABLE_MASK                       0x10
#define ES928X_DAC_ASP_ENABLE_SHIFT                         4
#define ES928X_AUTOMUTE_PERSIST_MASK                     0x08
#define ES928X_HPA_EN_MASK                               0x04
#define ES928X_LOW_POWER_MODE_MASK                       0x02
#define ES928X_RUN_VOLUME_MASK                           0x01
// Register 58-59 (16-bit)
#define ES928X_DAC_THD_COMP_C2_SHIFT                        0
// Register 60-61 (16-bit)
#define ES928X_DAC_THD_COMP_C3_SHIFT                        0
// Register 62-63 (16-bit)
#define ES928X_DAC_THD_COMP_C2_CH2_SHIFT                    0
// Register 64-65 (16-bit)
#define ES928X_DAC_THD_COMP_C3_CH2_SHIFT                    0
// Register 66
#define ES928X_DAC_CP_CLK_OVERRIDE_MASK                  0x80
#define ES928X_DAC_CP_CLK_EN_MASK                        0x40
#define ES928X_DAC_SYNC_MODE_MASK                        0x04
#define ES928X_DAC_FIN_FLIP_MASK                         0x02
#define ES928X_DAC_BYPASS_ASRC_MASK                      0x01
// Register 67
#define ES928X_DPLL_BW_I2S_SHIFT                            4
#define ES928X_DPLL_BW_I2S_MASK                          0xF0
#define ES928X_DPLL_BW_DSD_SHIFT                            0
#define ES928X_DPLL_BW_DSD_MASK                          0x0F
// Register 68
#define ES928X_DAC_STOP_DIV_SHIFT                           4
#define ES928X_DAC_STOP_DIV_MASK                         0xF0
#define ES928X_DAC_PHASE_SHIFT_SHIFT                        0
#define ES928X_DAC_PHASE_SHIFT_MASK                      0x0F
// Register 69-72 (32-bit)
#define ES928X_DAC_FORCE_DPLL_NUM_SHIFT                     0
// Register 73
#define ES928X_DAC_SOFT_START_MASK                       0x80
#define ES928X_DAC_SOFT_START_ON_LOCK_MASK               0x40
#define ES928X_DAC_SOFT_START_TYPE_MASK                  0x20
#define ES928X_DAC_SOFT_START_TIME_SHIFT                    0
#define ES928X_DAC_SOFT_START_TIME_MASK                  0x1F
// Register 74
#define ES928X_DAC_SOFT_START_ORDER_MASK                 0x40
#define ES928X_DAC_SOFT_START_SKIP_5050_MASK             0x20
#define ES928X_DAC_SOFT_START_DELAY_5050_MASK            0x10
#define ES928X_DAC_SOFT_START_DELAY_SHIFT                   0
#define ES928X_DAC_SOFT_START_DELAY_MASK                 0x0F
// Register 75
#define ES928X_DAC_AMP_PDB_ON_OC_MASK                    0x80
#define ES928X_DAC_NSMOD_GAIN_MASK                       0x40
#define ES928X_DAC_NSMOD_NO_DITHER_MASK                  0x20
#define ES928X_DAC_NSMOD_DITHER_BYPASS_MASK              0x10
#define ES928X_DAC_NSMOD_DITHER_SCALE_SHIFT                 0
#define ES928X_DAC_NSMOD_DITHER_SCALE_MASK               0x0F
// Register 76
#define ES928X_DAC_NSMOD_FIRST_ORDER_MASK                0x40
#define ES928X_DAC_CH2_ANALOG_SWAP_MASK                  0x20
#define ES928X_DAC_CH1_ANALOG_SWAP_MASK                  0x10
#define ES928X_DAC_FORCE_DACR_TO_ZERO_MASK               0x08
#define ES928X_DAC_FORCE_DACL_TO_ZERO_MASK               0x04
#define ES928X_DAC_FORCE_DACR_B_TO_ZERO_MASK             0x02
#define ES928X_DAC_FORCE_DACL_B_TO_ZERO_MASK             0x01
// Register 77
#define ES928X_DACR_TRIB_MASK                            0x80
#define ES928X_DACL_TRIB_MASK                            0x40
#define ES928X_DAC_AUTO_CLK_GEAR_EN_MASK                 0x20
#define ES928X_DAC_LOW_POWER_CLK_GEAR_MASK               0x10
#define ES928X_DAC_MAX_CLK_GEAR_SHIFT                       2
#define ES928X_DAC_MAX_CLK_GEAR_MASK                     0x0C
#define ES928X_DAC_CLK_GEAR_SHIFT                           0
#define ES928X_DAC_CLK_GEAR_MASK                         0x03
// Register 78-80 (2*12-bit)
#define ES928X_DAC_AUTO_CLK_GEAR_MIN_MSB_MASK            0xFF
#define ES928X_DAC_AUTO_CLK_GEAR_MIN_LSB_MASK            0xF0
#define ES928X_DAC_AUTO_CLK_GEAR_MIN_SHIFT                  4
#define ES928X_DAC_AUTO_CLK_GEAR_MAX_MSB_MASK            0x0F
#define ES928X_DAC_AUTO_CLK_GEAR_MAX_LSB_MASK            0xFF
#define ES928X_DAC_AUTO_CLK_GEAR_MAX_SHIFT                  0
// Register 81
#define ES928X_DAC_ANALOG_VOLUME2_USE_SECONDARY_MASK     0x80
#define ES928X_DAC_ENABLE_DETECTS_MASK                   0x40
#define ES928X_DAC_ANALOG_1DB_ENB_MASK                   0x20
#define ES928X_DAC_ANALOG_VOLUME_MAIN_SHIFT                 0
#define ES928X_DAC_ANALOG_VOLUME_MAIN_MASK               0x1F
// Register 82
#define ES928X_DAC_LOW_POWER_FSYNC_MASK                  0x80
#define ES928X_DAC_ANALOG_VOLUME_CTRL_ROLLY_MASK         0x40
#define ES928X_DAC_ANALOG_VOLUME_AUTOMUTE_MASK           0x20
#define ES928X_DAC_ANALOG_VOLUME_SECONDARY_SHIFT            0
#define ES928X_DAC_ANALOG_VOLUME_SECONDARY_MASK          0x1F
// Register 83-84 (12-bit)
#define ES928X_FSYNC_CLK_SEL_SHIFT                          6
#define ES928X_FSYNC_CLK_SEL_MASK                        0xC0
#define ES928X_FSYNC_SEL_SHIFT                              4
#define ES928X_FSYNC_SEL_MASK                            0x30
#define ES928X_FSYNC_DIV_MSB_MASK                        0xF0
#define ES928X_FSYNC_DIV_LSB_MASK                        0xFF
#define ES928X_FSYNC_DIV_SHIFT                              0
// Register 85
#define ES928X_ADC_VOLUME1_SHIFT                            0
#define ES928X_ADC_VOLUME1_MASK                          0xFF
// Register 86
#define ES928X_ADC_VOLUME2_SHIFT                            0
#define ES928X_ADC_VOLUME2_MASK                          0xFF
// Register 87
#define ES928X_ADC_VOLUME3_SHIFT                            0
#define ES928X_ADC_VOLUME3_MASK                          0xFF
// Register 88
#define ES928X_ADC_VOLUME4_SHIFT                            0
#define ES928X_ADC_VOLUME4_MASK                          0xFF
// Register 90
#define ES928X_ADC_VOL_RATE_SHIFT                           5
#define ES928X_ADC_VOL_RATE_MASK                         0xE0
#define ES928X_ADC_CH4_INVERT_MASK                       0x08
#define ES928X_ADC_CH3_INVERT_MASK                       0x04
#define ES928X_ADC_CH2_INVERT_MASK                       0x02
#define ES928X_ADC_CH1_INVERT_MASK                       0x01
// Register 91-92 (16-bit)
#define ES928X_ADC_DITHER_MASK_SHIFT                        0
// Register 93
#define ES928X_ADC_FIR_MODE_SHIFT                           6
#define ES928X_ADC_FIR_MODE_MASK                         0xC0
#define ES928X_ADC_MUTE_RIGHT_MASK                       0x20
#define ES928X_ADC_MUTE_LEFT_MASK                        0x10
#define ES928X_ADC_DITHER_SCALE_SHIFT                       0
#define ES928X_ADC_DITHER_SCALE_MASK                     0x0F
// Register 94
#define ES928X_ADC_IIR2_BW_SEL_SHIFT                        6
#define ES928X_ADC_IIR2_BW_SEL_MASK                      0xC0
#define ES928X_ADC_IIR1_BW_SEL_SHIFT                        4
#define ES928X_ADC_IIR1_BW_SEL_MASK                      0x30
#define ES928X_ADC_IIR0_BW_SEL_SHIFT                        2
#define ES928X_ADC_IIR0_BW_SEL_MASK                      0x0C
// Register 95
#define ES928X_ADC_NEG_MASK                              0x80
#define ES928X_ADC_FIR_HOLD_ZERO_MASK                    0x20
#define ES928X_ADC_IIR0_BYPASS_MASK                      0x10
#define ES928X_ADC_DC_BYPASS_MASK                        0x08
#define ES928X_ADC_ANALOG_HOLD_ZERO_MASK                 0x04
#define ES928X_ADC_INPUT_SCALE_SHIFT                        0
#define ES928X_ADC_INPUT_SCALE_MASK                      0x03
// Register 96-97
#define ES928X_ADC_MONO_STEREOB_MASK                     0x80
#define ES928X_ADC_CH4_SEL_SHIFT                            1
#define ES928X_ADC_CH4_SEL_MASK                          0x0E
#define ES928X_ADC_CH3_SEL_MSB_SHIFT                        0
#define ES928X_ADC_CH3_SEL_MSB_MASK                      0x01
#define ES928X_ADC_CH3_SEL_LSB_SHIFT                        6
#define ES928X_ADC_CH3_SEL_LSB_MASK                      0xC0
#define ES928X_ADC_CH2_SEL_SHIFT                            3
#define ES928X_ADC_CH2_SEL_MASK                          0x38
#define ES928X_ADC_CH1_SEL_SHIFT                            0
#define ES928X_ADC_CH1_SEL_MASK                          0x07
// Register 98
#define ES928X_ADC_PDM_FS_RATIO_SHIFT                       4
#define ES928X_ADC_PDM_FS_RATIO_MASK                     0xF0
#define ES928X_ADC_FS_RATIO_SHIFT                           0
#define ES928X_ADC_FS_RATIO_MASK                         0x0F
// Register 99
#define ES928X_ADC_RUN_VOLUME34_MASK                     0x40
#define ES928X_ADC_RUN_VOLUME12_MASK                     0x20
#define ES928X_ADC_CLKEN_CH_4_MASK                       0x08
#define ES928X_ADC_CLKEN_CH_3_MASK                       0x04
#define ES928X_ADC_CLKEN_CH_2_MASK                       0x02
#define ES928X_ADC_CLKEN_CH_1_MASK                       0x01
// Register 100
#define ES928X_ADC_SUM_SEL_SHIFT                            4
#define ES928X_ADC_SUM_SEL_MASK                          0x30
#define ES928X_ADC_INT_23_BIAS_SHIFT                        2
#define ES928X_ADC_INT_23_BIAS_MASK                      0x0C
#define ES928X_ADC_INT_1_BIAS_SHIFT                         0
#define ES928X_ADC_INT_1_BIAS_MASK                       0x03
// Register 101
#define ES928X_ADC_COMP_SEL_SHIFT                           4
#define ES928X_ADC_COMP_SEL_MASK                         0x70
#define ES928X_ADC_AMIC_ENA_MASK                         0x08
#define ES928X_ADC_IDAC_SEL_SHIFT                           0
#define ES928X_ADC_IDAC_SEL_MASK                         0x07
// Register 102
#define ES928X_PDM_ENA_SHIFT                                6
#define ES928X_PDM_ENA_MASK                              0xC0
#define ES928X_ADC_R_ENA_MASK                            0x20
#define ES928X_ADC_L_ENA_MASK                            0x10
#define ES928X_SEL_ADC_CLKIN_SHIFT                          2
#define ES928X_SEL_ADC_CLKIN_MASK                        0x0C
#define ES928X_SEL_ADC_CLK_DIV_SHIFT                        0
#define ES928X_SEL_ADC_CLK_DIV_MASK                      0x03
// Register 103
#define ES928X_USE_DITHER_MASK                           0x80
#define ES928X_USE_DWA_MASK                              0x40
#define ES928X_USE_STATE_MASK                            0x20
#define ES928X_PDM_CLK_DIV_SHIFT                            2
#define ES928X_PDM_CLK_DIV_MASK                          0x0C
#define ES928X_ADC_VREF_SEL_SHIFT                           0
#define ES928X_ADC_VREF_SEL_MASK                         0x03
// Register 104
#define ES928X_ADC_DEC_RATE_SHIFT                           0
#define ES928X_ADC_DEC_RATE_MASK                         0xFF
// Register 105-106 (16-bit)
#define ES928X_ASP_AUX_STEP_SIZE_SHIFT                      0
// Register 107
#define ES928X_AUX_ALTER_MIX_MASK                        0x80
#define ES928X_ASP_AUX_CUSTOM_ADDR_SHIFT                    0
#define ES928X_ASP_AUX_CUSTOM_ADDR_MASK                  0x3F
// Register 108
#define ES928X_MUX_MIC_ENA_MASK                          0x10
#define ES928X_MUX_R_ENA_MASK                            0x08
#define ES928X_MUX_L_ENA_MASK                            0x04
#define ES928X_MUX_BIAS_SEL_SHIFT                           0
#define ES928X_MUX_BIAS_SEL_MASK                         0x03
// Register 109
#define ES928X_MUX_L_INPUT_SEL_SHIFT                        6
#define ES928X_MUX_L_INPUT_SEL_MASK                      0xC0
#define ES928X_MUX_L_GAIN_SHIFT                             0
#define ES928X_MUX_L_GAIN_MASK                           0x3F
// Register 110
#define ES928X_MUX_R_INPUT_SEL_SHIFT                        6
#define ES928X_MUX_R_INPUT_SEL_MASK                      0xC0
#define ES928X_MUX_R_GAIN_SHIFT                             0
#define ES928X_MUX_R_GAIN_MASK                           0x3F
// Register 111
#define ES928X_MUX_MIC_INPUT_SEL_SHIFT                      6
#define ES928X_MUX_MIC_INPUT_SEL_MASK                    0xC0
#define ES928X_MUX_MIC_GAIN_SHIFT                           0
#define ES928X_MUX_MIC_GAIN_MASK                         0x3F
// Register 112
#define ES928X_MIC_REG_PDB_MASK                          0x10
#define ES928X_MIC_REG_GAIN_SHIFT                           0
#define ES928X_MIC_REG_GAIN_MASK                         0x0F
// Register 113
#define ES928X_JDET_HOLD_MASK                            0x80
// Register 119-120
#define ES928X_MIC_REG_PDB_HIGH_SHIFT                       0
#define ES928X_MIC_REG_PDB_LOW_SHIFT                        0
// Register 122
#define ES928X_CP_PDB_MASK                               0x80
#define ES928X_VCO_PDB_MASK                              0x40
#define ES928X_VCO_FREQ_BAND_SHIFT                          2
#define ES928X_VCO_FREQ_BAND_MASK                        0x1C
#define ES928X_VCO_BIAS_SEL_SHIFT                           0
#define ES928X_VCO_BIAS_SEL_MASK                         0x03
// Register 123
#define ES928X_CLK_FB_DIV_SHIFT                             0
// Register 124-125
#define ES928X_CP_BIAS_SEL_MSB_MASK                      0x80
#define ES928X_CLK_IN_DIV_MSB_SHIFT                         0
#define ES928X_CLK_IN_DIV_MSB_MASK                       0x1F
#define ES928X_CLK_IN_DIV_LSB_SHIFT                         4
#define ES928X_CLK_IN_DIV_LSB_MASK                       0xF0
#define ES928X_PLL_LOW_BW_MASK                           0x01
// Register 126-127
#define ES928X_PFD_DELAY_SEL_SHIFT                          6
#define ES928X_PFD_DELAY_SEL_MASK                        0xC0
#define ES928X_PLL_SEL_CLKIN_SHIFT                          4
#define ES928X_PLL_SEL_CLKIN_MASK                        0x30
#define ES928X_USB_SEL_CLKIN_MASK                        0x08
#define ES928X_CP_BIAS_SEL_SHIFT                            1
#define ES928X_CP_BIAS_SEL_MASK                          0x06
#define ES928X_CLK_OUT_DIV_MSB_SHIFT                        0
#define ES928X_CLK_OUT_DIV_MSB_MASK                      0x01
#define ES928X_CLK_OUT_DIV_LSB_SHIFT                        0
#define ES928X_CLK_OUT_DIV_LSB_MASK                      0xFF
// Register 128
#define ES928X_ASP_DAC_PROGRAM_EN_MASK                   0x80
#define ES928X_AUX_ASP_ENABLE_MASK                       0x40
#define ES928X_ASP_DAC_PROGRAM_WE_MASK                   0x20
#define ES928X_ASP_DAC_COEFF_WE_MASK                     0x10
#define ES928X_ASP_AUX_PROGRAM_WE_MASK                   0x02
#define ES928X_ASP_AUX_COEFF_WE_MASK                     0x01
// Register 129
#define ES928X_ASP_PROGRAM_SHIFT                            0
#define ES928X_ASP_PROGRAM_MASK                          0xFF
// Register 130
#define ES928X_ASP_PROGRAM_ADDR_SHIFT                       0
#define ES928X_ASP_PROGRAM_ADDR_MASK                     0xFF
// Register 140
#define ES928X_PWM1_COUNT_SHIFT                             0
#define ES928X_PWM1_COUNT_MASK                           0xFF
// Register 141
#define ES928X_PWM1_FREQ_SHIFT                              0
#define ES928X_PWM1_FREQ_MASK                            0xFF
// Register 142
#define ES928X_PWM2_COUNT_SHIFT                             0
#define ES928X_PWM2_COUNT_MASK                           0xFF
// Register 143
#define ES928X_PWM2_FREQ_SHIFT                              0
#define ES928X_PWM2_FREQ_MASK                            0xFF
// Register 144
#define ES928X_PWM3_COUNT_SHIFT                             0
#define ES928X_PWM3_COUNT_MASK                           0xFF
// Register 145
#define ES928X_PWM3_FREQ_SHIFT                              0
#define ES928X_PWM3_FREQ_MASK                            0xFF
// Register 146
#define ES928X_ENABLE_CT_MASK                            0x01
// Register 147-148 (16-bit)
#define ES928X_CROSSTALK_SCALE_CH1_SHIFT                    0
// Register 149-150 (16-bit)
#define ES928X_CROSSTALK_SCALE_CH2_SHIFT                    0
// Register 151-152 (16-bit)
#define ES928X_DC_OFFSET_CH1_SHIFT                          0
// Register 153-154 (16-bit)
#define ES928X_DC_OFFSET_CH2_SHIFT                          0
// Register 155
#define ES928X_DC_RAMP_RATE_SHIFT                           0
#define ES928X_DC_RAMP_RATE_MASK                         0xFF
// Register 156
#define ES928X_ENABLE_PROG_COEFF_GROUP_DELAY_MASK        0x80
// Register 157
#define ES928X_ASP_CUSTOM_RAM1_ADDR_DAC_SHIFT               0
#define ES928X_ASP_CUSTOM_RAM1_ADDR_DAC_MASK             0x3F
// Register 158
#define ES928X_ASP_CUSTOM_RAM2_ADDR_DAC_SHIFT               0
#define ES928X_ASP_CUSTOM_RAM2_ADDR_DAC_MASK             0x3F
// Register 159
#define ES928X_ASP_CUSTOM_ADDR2_ANC_SHIFT                   0
#define ES928X_ASP_CUSTOM_ADDR2_ANC_MASK                 0x3F
// Register 160-161 (16-bit)
#define ES928X_ASP_STEP_SIZE_DAC_SHIFT                      0
// Register 163
#define ES928X_ENABLE_ADC_ASP_MASK                       0x80
#define ES928X_ADC_ASP_ALTER_CHANNEL_MASK                0x40
#define ES928X_ASP_CUSTOM_ADDR2_ADC_SHIFT                   0
#define ES928X_ASP_CUSTOM_ADDR2_ADC_MASK                 0x3F
// Register 164
#define ES928X_ADC_PEAK_INT_MASK_MASK                    0x80
#define ES928X_ADC_PEAK_INT_CLEAR_MASK                   0x40
#define ES928X_ADC_PEAK_DECAY_RATE_SHIFT                    0
#define ES928X_ADC_PEAK_DECAY_RATE_MASK                  0x3F
// Register 165-166 (16-bit)
#define ES928X_ADC_PEAK_THRESHOLD_SHIFT                     0
// Register 167
#define ES928X_FORCE_MUX_R_INPUT_SEL_MASK                0x80
#define ES928X_MUX_R_INPUT_SEL_3BIT_SHIFT                   4
#define ES928X_MUX_R_INPUT_SEL_3BIT_MASK                 0x70
#define ES928X_FORCE_MUX_L_INPUT_SEL_MASK                0x08
#define ES928X_MUX_L_INPUT_SEL_3BIT_SHIFT                   0
#define ES928X_MUX_L_INPUT_SEL_3BIT_MASK                 0x07
// Register 168
#define ES928X_EXP_BITS1_SHIFT                              0
#define ES928X_EXP_BITS1_MASK                            0xFF
// Register 169-170 (16-bit)
#define ES928X_EXP_OUTPUT_SHIFT                             0
// Register 192
#define ES928X_OCRP_MASK                                 0x02
#define ES928X_OCRN_MASK                                 0x01
// Register 193 (122)
// Register 194-196 (123-125)
// Register 197-198 (126-128)
// Register 199
#define ES928X_RO_AUX_MIX_CLKEN_1FS_MASK                 0x80
#define ES928X_RO_DAC_CLKEN_1FS_MASK                     0x40
#define ES928X_RO_DAC_MQB_AUTH_MASK                      0x20
#define ES928X_RO_DAC_VOL_MIN_MASK                       0x10
#define ES928X_RO_DAC_SOFT_START_MUTED_MASK              0x08
#define ES928X_RO_DAC_AUTOMUTE_STATUS_SHIFT                 1
#define ES928X_RO_DAC_AUTOMUTE_STATUS_MASK               0x06
#define ES928X_RO_DAC_LOCK_STATUS_MASK                   0x01
// Register 200-203 (32-bit)
#define ES928X_RO_DPLL_NUM_SHIFT                            0
// Register 204-211 (64-bit)
#define ES928X_RO_DAC_LARGE_PAYLOAD_SHIFT                   0
// Register 212-213 (15-bit)
#define ES928X_GPIO_I_R_LSB_SHIFT                           0
#define ES928X_GPIO_I_R_LSB_MASK                         0xFF
#define ES928X_GPIO_I_R_MSB_SHIFT                           0
#define ES928X_GPIO_I_R_MSB_MASK                         0x7F
// Register 214-215
#define ES928X_RO_DAC_CLKEN_1FS_R_MASK                   0x40
#define ES928X_RO_AUX_MIX_CLKEN_1FS_R_MASK               0x20
#define ES928X_RO_MCU_R_MASK                             0x10
#define ES928X_RO_AUTOMUTE_STATUS_CH2_R_MASK             0x08
#define ES928X_AUTOMUTE_STATUS_CH1_R_MASK                0x04
#define ES928X_SOFT_START_DONE_R_MASK                    0x02
#define ES928X_LOCK_STATUS_R_MASK                        0x01
// Register 220
#define ES928X_RO_CHIP_ID_SHIFT                             0
#define ES928X_RO_CHIP_ID_MASK                           0x1F
// Register 230
#define ES928X_BOOT_FINISHED_FLAG                  0x00000080
#define ES928X_MIC_IN_STATUS                       0x00000020
#define ES928X_JACK_IN                             0x00000010
#define ES928X_HP_TYPE_STATUS                      0x00000008
#define ES928X_ZMEAS_STATUS                        0x00000004
#define ES928X_BUTTON_RELEASE                      0x00000002
#define ES928X_BUTTON_PRESS                        0x00000001
// Register 232
#define ES928X_DAC_EN_MASK                               0x01
#define ES928X_ADC_EN_MASK                               0x02
#define ES928X_PLL_EN_MASK                               0x04
#define ES928X_DAC_MUTE_MASK                             0x08
#define ES928X_JDET_RRUN_MASK                            0x80
// Register 239
#define ES928X_FW_BLOCK_VERIFICTION_FAIL                 0x02
#define ES928X_FW_BLOCK_READBACK_FAIL                    0x06
// Register 241-243 (23-bit)
#define ES928X_ADC_PEAK_LSB_SHIFT                           0
#define ES928X_ADC_PEAK_LSB_MASK                         0xFF
#define ES928X_ADC_PEAK_MSB_SHIFT                           0
#define ES928X_ADC_PEAK_MSB_MASK                         0x7F
// Register 247
#define ES928X_ADC_PEAK_INT_MASK                         0x01
// Register 248
#define ES928X_CHANNEL_GAIN_CH2_MASK                     0x20
#define ES928X_CHANNEL_GAIN_CH1_MASK                     0x10
#define ES928X_DRE_DETECT_CH2_MASK                       0x08
#define ES928X_DRE_DETECT_CH1_MASK                       0x04
#define ES928X_DRE_SELECT_CH2_MASK                       0x02
#define ES928X_DRE_SELECT_CH1_MASK                       0x01
// Register 249
#define ES928X_MQB_AUTH_STATE_SHIFT                         0
#define ES928X_MQB_AUTH_STATE_MASK                       0x03
// Register 254
#define ES928X_DAC_CLK_GEAR_R_SHIFT                         4
#define ES928X_DAC_CLK_GEAR_R_MASK                       0x30
#define ES928X_DAC_INPUT_SELECTION_SHIFT                    0
#define ES928X_DAC_INPUT_SELECTION_MASK                  0x0F

/* SYNC I2C Interface */
// Register 128 (122)
// Register 129-131 (123-125)
// Register 132-133 (126-128)

//Firmware 
#define ES928X_FW_BLOCK_SIZE                              128
#define ES928X_FW_ROM_SIZE                             0x2800
#define ES928X_FW_VERSION_1                           0x204A4

struct es928x_fw
{
	unsigned int major;
	unsigned int minor;
	unsigned int build;
};

//Clocks
#define ES928X_CLOCK_MIN_FREQ                         2000000
enum es928x_clk_src {
	XOSC = 0,
	PLL,
	ACLK1,
	ACLK2,
	CLK_END,
};

//JDET
#define ES928X_JDET_REPORT_ALL_MASK	(SND_JACK_MECHANICAL |			\
					 SND_JACK_HEADSET | SND_JACK_LINEOUT |	\
					 SND_JACK_BTN_0 | SND_JACK_BTN_1 |	\
					 SND_JACK_BTN_2 | SND_JACK_BTN_3)
void es928x_jdet_jack_det(struct snd_soc_component *component, struct snd_soc_jack *jack);
struct es928x_jdet_priv
{
	struct snd_soc_component *component;
	struct snd_soc_jack *jack;
	struct delayed_work zdet_work;
	struct work_struct jdet_work;
	struct delayed_work poweroff_work;
	int irq;
	bool jack_inserted;
	bool zmeas_done;
	int current_plug;
	bool debug_mode;
};

struct es928x_asp
{
	u8 program[128];
	int coefficients[128];
	unsigned int program_addr;
	unsigned int coeff_addr;
};

struct es928x_priv 
{
	struct device *dev;
	struct snd_soc_component *component;
	struct regmap *regmap;

	unsigned int clocks[CLK_END];
	bool rx_i2s_master;
	bool tx_i2s_master;
	u8 dac_clk_src;
	u8 adc_clk_src;
	u8 pll_config;
	
	struct mutex lock;	
	struct es928x_jdet_priv *jdet;
	int rst_gpio;
	struct es928x_fw *fw;
	struct work_struct firmware_work;

	struct es928x_asp *dac_asp;
	int gpio_1p8;
	int gpio_3p3;
	int gpio_audio_debug;
	char audio_debug[256]; // ASUS BSP Jackson +++
};

enum es928x_plug_type {
	ES928X_PLUG_TYPE_INVALID = -1,
	ES928X_PLUG_TYPE_NONE,
	ES928X_PLUG_TYPE_HEADSET,
	ES928X_PLUG_TYPE_HEADPHONE,
	ES928X_PLUG_TYPE_HIGH_HPH,
	ES928X_PLUG_TYPE_GND_MIC_SWAP,
	ES928X_PLUG_TYPE_ANC_HEADPHONE,
};

#define ES928X_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)



#endif
