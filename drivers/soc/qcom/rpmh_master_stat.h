/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2018-2020, The Linux Foundation. All rights reserved.
 */

#ifndef __QCOM_RPM_STATS_LOG_H__
#define __QCOM_RPM_STATS_LOG_H__

#if IS_ENABLED(CONFIG_QTI_RPM_STATS_LOG)

void msm_rpmh_master_stats_update(void);

#else

static inline void msm_rpmh_master_stats_update(void) {}

#endif
//[PM_debug +++]
void msm_rpmh_master_stats_print(void);
void soc_sleep_stats_print(void);
//[PM_debug ---]
#endif /* __QCOM_RPM_STATS_LOG_H__ */
