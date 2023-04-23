/*****************************************************************************
* File: sysfs.h
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
#ifndef SYSFS_H
#define SYSFS_H

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/

/*==========================================================================*/
/* PROTOTYPES                                                               */
/*==========================================================================*/
ssize_t sysfs_register_show(struct device *dev,
                            struct device_attribute *attr,
                            uint16_t reg,
                            char *buf);

ssize_t sysfs_register_store(struct device *dev,
                             struct device_attribute *attr,
                             uint16_t reg,
                             const char *buf,
                             size_t count);

ssize_t sysfs_set_reg_store(struct device *dev,
                             struct device_attribute *attr,
                             uint16_t reg,
                             const char *buf,
                             size_t count);

ssize_t sysfs_get_reg_show(struct device *dev,
                            struct device_attribute *attr,
                            uint16_t reg,
                            char *buf);

ssize_t sysfs_get_reg_store(struct device *dev,
                             struct device_attribute *attr,
                             uint16_t reg,
                             const char *buf,
                             size_t count);

ssize_t sysfs_log_no_touch_frame_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf);
ssize_t sysfs_log_no_touch_frame_store(struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf,
                                 size_t count);

ssize_t sysfs_track_report_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf);

ssize_t sysfs_log_track_report_show(struct device *dev,
                                    struct device_attribute *attr,
                                    char *buf);

ssize_t sysfs_event_log_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count);

ssize_t sysfs_event_log_show(struct device *dev,
                                    struct device_attribute *attr,
                                    char *buf);

ssize_t sysfs_log_track_report_bin_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count);

ssize_t sysfs_log_track_report_bin_show(struct device *dev,
                                    struct device_attribute *attr,
                                    char *buf);

ssize_t sysfs_log_track_report_bin_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count);

ssize_t sysfs_log_d1test_show(struct device *dev,
                                    struct device_attribute *attr,
                                    char *buf);

ssize_t sysfs_log_d1test_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count);

ssize_t sysfs_log_frames_show(struct device *dev,
                                    struct device_attribute *attr,
                                    char *buf);

ssize_t sysfs_log_frames_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count);

ssize_t sysfs_deep_trace_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count);

ssize_t sysfs_register_enable_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf);

ssize_t sysfs_register_enable_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t count);

ssize_t sysfs_register_frame_rate_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf);

ssize_t sysfs_register_frame_rate_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf,
                                         size_t count);

ssize_t sysfs_register_event_show(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf);

ssize_t sysfs_register_event_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count);

ssize_t sysfs_product_config_show(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf);

ssize_t sysfs_wake_device_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf,
                                size_t count);

ssize_t sysfs_load_registers_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count);

ssize_t sysfs_reg_script_file_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count);

#ifdef SUPPORT_FLASH
ssize_t sysfs_fwupdate_show(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf);

ssize_t sysfs_fwupdate_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count);

ssize_t sysfs_write_flash_reg_part_file_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count);

ssize_t sysfs_write_flash_reg_part_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count);


ssize_t sysfs_read_flash_reg_part_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count);

ssize_t sysfs_read_flash_reg_part_show(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf);

#endif

ssize_t sysfs_set_sys_param_show(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf);

ssize_t sysfs_set_sys_param_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count);
ssize_t sysfs_get_sys_param_show(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf);

ssize_t sysfs_get_sys_param_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count);

ssize_t sysfs_sc_reset_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count);

ssize_t sysfs_profile_show(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf);

ssize_t sysfs_profile_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count);

#ifdef DYNAMIC_PWR_CTL
int snt_activity_request(void);

ssize_t sysfs_enable_dynamic_pwr_ctl_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count);


ssize_t sysfs_enable_dynamic_pwr_ctl_show(struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf);
#endif

int snt_sysfs_init(struct snt8100fsr *snt8100fsr, bool enable);

#endif // SYSFS_H

