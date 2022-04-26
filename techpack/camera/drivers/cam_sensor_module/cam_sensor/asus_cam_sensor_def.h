#ifndef ASUS_CAM_SENSOR_DEF_H
#define ASUS_CAM_SENSOR_DEF_H


#define SENSOR_ID_IMX363  0x0363
#define SENSOR_ID_IMX563  0x0563
#define SENSOR_ID_IMX586  0x0586
#define SENSOR_ID_IMX686  0x0686


#define SENSOR_ID_OV8856  0x885a
#define SENSOR_ID_OV13855 0xD855
#define SENSOR_ID_OV13B10 0x0D42
#define SENSOR_ID_OV24B1Q 0x2442

#define PROC_MODULE_CAMERA0	"driver/CameraModule0"
#define PROC_MODULE_CAMERA1	"driver/CameraModule1"
#define PROC_MODULE_CAMERA2	"driver/CameraModule2"
#define PROC_MODULE_CAMERA3	"driver/CameraModule3"
#define PROC_MODULE_CAMERA4	"driver/CameraModule4"


#define PROC_RESOUTION_CAMERA0	"driver/CameraResoution0"
#define PROC_RESOUTION_CAMERA1	"driver/CameraResoution1"
#define PROC_RESOUTION_CAMERA2	"driver/CameraResoution2"
#define PROC_RESOUTION_CAMERA3	"driver/CameraResoution3"
#define PROC_RESOUTION_CAMERA4	"driver/CameraResoution4"


#define PROC_STATUS_CAMERA0	"driver/CameraStatus0"
#define PROC_STATUS_CAMERA1	"driver/CameraStatus1"
#define PROC_STATUS_CAMERA2	"driver/CameraStatus2"
#define PROC_STATUS_CAMERA3	"driver/CameraStatus3"
#define PROC_STATUS_CAMERA4	"driver/CameraStatus4"


#define OTP_DATA_LEN_WORD (32)
#define OTP_DATA_LEN_BYTE (OTP_DATA_LEN_WORD*2)
#define OTP_ID_LEN (12)

#define PROC_OTP_Camera0     "driver/otp0"
#define PROC_OTP_Camera1     "driver/otp1"
#define PROC_OTP_Camera2     "driver/otp2"
#define PROC_OTP_Camera3     "driver/otp3"
#define PROC_OTP_Camera4     "driver/otp4"

#define PROC_SENSOR_I2C_RW "driver/sensor_i2c_rw"

#define PROC_EEPROM_I2C_R  "driver/eeprom_i2c_r"

#define PROC_ARCSOFT_CALI "driver/dualcam_cali"

#define PROC_ARCSOFT_CALI_1x ""PROC_ARCSOFT_CALI"_1x"
#define PROC_ARCSOFT_CALI_3x ""PROC_ARCSOFT_CALI"_3x"

#define FACTORYDIR "/vendor/factory/"
#define GOLDENDIR "/vendor/lib64/camera/"

#define DUAL_CALI_BIN ""FACTORYDIR"dualcam_cali.bin"

#define PROC_SENSORS_RES "driver/camera_res"

#define PROC_MODULE_CHANGE_REAR "driver/rear_module_change"
#define PROC_MODULE_CHANGE_FRONT "driver/front_module_change"
#define PROC_MODULE_CHANGE_REAR2 "driver/rear2_module_change"
#define PROC_MODULE_CHANGE_REAR3 "driver/rear3_module_change"
#define PROC_MODULE_CHANGE_REAR4 "driver/rear4_module_change"
#define PROC_MODULE_CHANGE_FRONT2 "driver/front2_module_change"

#define	PROC_CSI_CHECK	"driver/cam_csi_check"  //ASUS_BSP "Add for camera csi debug"
#define	PROC_CCI_CHECK	"driver/cam_cci_check"  //ASUS_BSP "Add for camera cci debug"

#endif
