/////////////////////////////////////////////////////////////////////////////////////////////
////                  ASUS Debugging mechanism
/////////////////////////////////////////////////////////////////////////////////////////////
#ifndef __ASUSDEBUG_H__
#define __ASUSDEBUG_H__


extern phys_addr_t PRINTK_BUFFER_PA;
extern void *PRINTK_BUFFER_VA;
extern phys_addr_t RTB_BUFFER_PA;
#define PRINTK_BUFFER_SIZE      (0x00400000)

#define PRINTK_BUFFER_MAGIC     (0xFEEDBEEF)
#define PRINTK_BUFFER_SLOT_SIZE (0x00040000)

#define PRINTK_BUFFER_SLOT1     (PRINTK_BUFFER_VA)
#define PRINTK_BUFFER_SLOT2     ((void *)((ulong)PRINTK_BUFFER_VA + (ulong)PRINTK_BUFFER_SLOT_SIZE))

#define PHONE_HANG_LOG_BUFFER   ((void *)((ulong)PRINTK_BUFFER_VA + (ulong)2*PRINTK_BUFFER_SLOT_SIZE ) - (ulong)0x3FC00)
#define PHONE_HANG_LOG_SIZE     (SZ_1M + PRINTK_BUFFER_SLOT_SIZE*2 + 0x3FC00)

/* ASUS_BSP Paul +++ */
#define LOGCAT_BUFFER_PA        ((void *)((ulong)PRINTK_BUFFER_PA + (ulong)SZ_2M + (ulong)SZ_1M ))
#define LOGCAT_BUFFER           ((void *)((ulong)PRINTK_BUFFER_VA + (ulong)SZ_2M + (ulong)SZ_1M ))
#define LOGCAT_BUFFER_SIZE      (SZ_1M)
#define LAST_KMSG_SIZE          (SZ_16K)
/* ASUS_BSP Paul --- */

/////////////////////////////////////////////////////////////////////////////////////////////
////                  Eventlog mask mechanism
/////////////////////////////////////////////////////////////////////////////////////////////
#define ASUS_ASDF_BASE_DIR "/asdf/"
#define ASUS_EVTLOG_PATH ASUS_ASDF_BASE_DIR"ASUSEvtlog"
#define ASUS_EVTLOG_STR_MAXLEN (256)
#define ASUS_EVTLOG_MAX_ITEM (20)
/*AS-K SubSys Health Record+++*/
#define SUBSYS_HEALTH_MEDICAL_TABLE_PATH "/asdf/SubSysMedicalTable"
#define SUBSYS_BUS_ROOT "/sys/bus/msm_subsys/devices"
#define SUBSYS_NUM_MAX 10
#define SUBSYS_W_MAXLEN (170) /*%04d-%02d-%02d %02d:%02d:%02d : [SSR]:name reason*/
#define SUBSYS_R_MAXLEN (512)
#define SUBSYS_C_MAXLEN (30)
/*AS-K SubSys Health Record---*/
//[+++]Record the impoartant power event
#define ASUS_ERCLOG_PATH ASUS_ASDF_BASE_DIR"ASUSErclog"
#define ASUS_ERCLOG_STR_MAXLEN (256)
#define ASUS_ERCLOG_MAX_ITEM (20)
#define ASUS_ERCLOG_FILENAME_MAXLEN (128)

#define ASUS_USB_THERMAL_ALERT "ASUS_thermal_alert"
#define ASUS_VBUS_LOW_IMPEDANCE  "ASUS_VBUS_low_impedance"
#define ASUS_AICL_SUSPEND "ASUS_AICL_suspend"
#define ASUS_JEITA_HARD_HOT "ASUS_JEITA_hard_hot"
#define ASUS_JEITA_HARD_COLD "ASUS_JEITA_hard_cold"
#define ASUS_USB_WATER_INVADE "ASUS_USB_water_invade"
#define ASUS_OUTPUT_OVP "ASUS_Output_OVP"
//[---]Record the impoartant power event

void save_all_thread_info(void);
void delta_all_thread_info(void);
void save_phone_hang_log(int delta);
void save_last_shutdown_log(char *filename);
#if defined(CONFIG_MSM_RTB)
void save_rtb_log(void);
#endif
void get_last_shutdown_log(void);
void printk_lcd(const char *fmt, ...);
void printk_lcd_xy(int xx, int yy, unsigned int color, const char *fmt, ...);
void ASUSEvtlog(const char *fmt, ...);
void ASUSErclog(const char * filename, const char *fmt, ...);//Record the important power event
void SubSysHealthRecord(const char *fmt, ...);/*AS-K SubSys Health Record+*/
//20101202_Bruno: added to get debug mask value
bool isASUS_MSK_set(const char *fmt);
#endif
