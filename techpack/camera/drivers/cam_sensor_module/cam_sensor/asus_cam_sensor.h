#ifndef ASUS_CAM_SENSOR_H
#define ASUS_CAM_SENSOR_H

#include "cam_sensor_dev.h"

void asus_cam_sensor_init(struct cam_sensor_ctrl_t *s_ctrl);

int32_t get_file_size(const char *filename, uint64_t* size);
#endif
