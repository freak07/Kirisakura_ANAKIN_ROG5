/*
 * Copyright (c) 2020, ASUS. All rights reserved.
 */
#ifndef _DRM_ANAKIN_H_
#define _DRM_ANAKIN_H_

#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>
#define ASUS_NOTIFY_GHBM_ON_REQ        0
#define ASUS_NOTIFY_GHBM_ON_READY      1
#define ASUS_NOTIFY_SPOT_READY         2
#define ASUS_NOTIFY_FOD_TOUCHED        3

void anakin_drm_notify(int var, int value);
void drm_anakin_sysfs_destroy(void);
int drm_anakin_sysfs_init(void);
bool is_DSI_mode(int,int);
bool refreshrate_match(int,int);

#else
static inline void anakin_drm_notify(int var, int value) {}
static inline void drm_anakin_sysfs_destroy(void) {}
static inline int drm_anakin_sysfs_init(void) { return 0; }
static inline bool is_DSI_mode(int vdisplay, int vtotal) { return false; }
static inline bool refreshrate_match(int refresh1, int refresh2) { return true; }

#endif
#endif /* _DRM_ANAKIN_H_ */

