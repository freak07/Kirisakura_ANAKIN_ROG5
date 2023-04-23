/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#ifndef _DT_BINDINGS_CLK_QCOM_APSS_CC_SDXLEMUR_H
#define _DT_BINDINGS_CLK_QCOM_APSS_CC_SDXLEMUR_H

#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
/* APSS_CC clocks */
#define APCS_CPU_PLL						0
#define APCS_MUX_CLK						1
#else
/* APSS_CC clocks */
#define APCS_CLK						0
#define APCS_CPU_PLL						1
#endif

#endif
