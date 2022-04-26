#include "focaltech_core.h"
/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define READ_FIG_NUM_REG                    0x02
#define READ_GESTURE_1_REG                  0x07
#define READ_GESTURE_2_REG                  0x0D
#define SET_GESTURE_LEN_REG                 0x94
/*****************************************************************************
* 1. Global variable or extern global variabls/functions
*****************************************************************************/
/*****************************************************************************
* 2. Static function prototypes
*******************************************************************************/
/*Gesture related*/
int asus_gesture_init(struct fts_ts_data *ts_data);
int asus_gesture_readdata(struct fts_ts_data *ts_data, u8 *data);
int asus_create_sysfs(struct fts_ts_data *ts_data);
int asus_remove_sysfs(struct fts_ts_data *ts_data);
void set_vaild_length(int level);