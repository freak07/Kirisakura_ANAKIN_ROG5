#include "focaltech_core.h"
/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_FOD_X_H_POS_REG                 0xE1
#define FTS_FOD_X_L_POS_REG                 0xE2
#define FTS_FOD_Y_H_POS_REG                 0xE2
#define FTS_FOD_Y_L_POS_REG                 0xE2

#define FTS_REG_EDGEPALM_MODE_EN            0x8C
#define FTS_REG_EDGEPALM_LEFT               0x8D
#define FTS_REG_EDGEPALM_RIGHT              0x8E

#define TOTAL_SLOT 10

/*****************************************************************************
* 1. Global variable or extern global variabls/functions
*****************************************************************************/
/*****************************************************************************
* 2. Static function prototypes
*******************************************************************************/
/*Game related*/
int asus_game_create_sysfs(struct fts_ts_data *ts_data);
int asus_game_remove_sysfs(struct fts_ts_data *ts_data);
void ATR_touch(int id,int action, int x, int y, int random);
int set_rotation_mode(int angle);
void asus_game_recovery(struct fts_ts_data *ts_data);

/*Extra mode*/
int asus_create_sysfs(struct fts_ts_data *ts_data);
int asus_remove_sysfs(struct fts_ts_data *ts_data);
void asus_tp_charge_mode (bool enable);
void fts_ex_fun_recovery(struct fts_ts_data *ts_data) ;


/*Gesture related*/
int asus_gesture_init(struct fts_ts_data *ts_data);
int is_enter_gesture_mode (struct fts_ts_data *ts_data);
int set_gesture_register (struct fts_ts_data *ts_data);
void write_fp_xy(struct fts_ts_data *ts_data);
void asus_gesture_report(struct fts_ts_data *ts_data, int gesture_id);
int asus_gesture_suspend(struct fts_ts_data *ts_data);
int asus_gesture_resume(struct fts_ts_data *ts_data);