#ifndef _ASUS_GLOBAL_VAR_H
#define _ASUS_GLOBAL_VAR_H

#if 1 // defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
enum EXTERN_DEVICE_PROJID
{
	EXTERN_PROJECT_INVALID            = -1,
	EXTERN_PROJECT_ANAKIN_ENTRY       =  0,
	EXTERN_PROJECT_ANAKIN_ELITE       =  1,
	EXTERN_PROJECT_PICASSO            =  2,
	EXTERN_PROJECT_ANAKIN2_ERC_ARUA   =  9,
	EXTERN_PROJECT_ANAKIN2_ERC_POMLED = 10,
	EXTERN_PROJECT_ANAKIN2_ERA        = 11,
	EXTERN_PROJECT_ANAKIN2_ARUA       = 12,
	EXTERN_PROJECT_ANAKIN2_POMLED     = 13,
	EXTERN_PROJECT_ANAKIN2_PX_ARUA    = 14,
	EXTERN_PROJECT_ANAKIN2_PX_POMLED  = 15
};
extern enum EXTERN_DEVICE_PROJID g_EXTERN_ASUS_prjID;

enum EXTERN_DEVICE_HWID
{
	EXTERN_HW_REV_INVALID             = -1,
	EXTERN_HW_REV_EVB                 =  0,
	EXTERN_HW_REV_EVB2                =  1,
	EXTERN_HW_REV_SR                  =  2,
	EXTERN_HW_REV_ER                  =  3,
	EXTERN_HW_REV_ER2                 =  4,
	EXTERN_HW_REV_PR                  =  5,
	EXTERN_HW_REV_PR2                 =  6,
	EXTERN_HW_REV_MP                  =  7,
	EXTERN_HW_REV_MP3                 =  8,
	EXTERN_HW_REV_MP4                 =  9,
	EXTERN_HW_REV_MP5                 = 10,
	EXTERN_HW_REV_MP6                 = 11,
	EXTERN_HW_REV_MP7                 = 12,
	EXTERN_HW_REV_MP8                 = 13,
	EXTERN_HW_REV_MP9                 = 14,
	EXTERN_HW_REV_MP10                = 15,
	EXTERN_HW_REV_ANAKIN2_ER          = 20,
	EXTERN_HW_REV_ANAKIN2_PR          = 21,
	EXTERN_HW_REV_ANAKIN2_MP          = 22
};
extern enum EXTERN_DEVICE_HWID g_EXTERN_ASUS_hwID;

enum EXTERN_DEVICE_SKUID
{
	EXTERN_SKU_ID_INVALID             = -1,
	EXTERN_SKU_ID_0                   =  0,
	EXTERN_SKU_ID_1                   =  1,
	EXTERN_SKU_ID_2                   =  2,
	EXTERN_SKU_ID_3                   =  3,
	EXTERN_SKU_ID_4                   =  4,
	EXTERN_SKU_ID_5                   =  5,
	EXTERN_SKU_ID_6                   =  6,
	EXTERN_SKU_ID_7                   =  7
};
extern enum EXTERN_DEVICE_SKUID g_EXTERN_ASUS_skuID;

enum EXTERN_DEVICE_DDRID
{
	EXTERN_DDR_VENDOR_INVALID         = -1,
	EXTERN_DDR_6400                   =  0,
	EXTERN_DDR_5500                   =  1
};
extern enum EXTERN_DEVICE_DDRID g_EXTERN_ASUS_ddrID;

enum EXTERN_DEVICE_NFCID
{
	EXTERN_NFC_VENDOR_INVALID         = -1,
	EXTERN_NFC_NOT_SUPPORT            =  0,
	EXTERN_NFC_SUPPORT                =  1
};
extern enum EXTERN_DEVICE_NFCID g_EXTERN_ASUS_nfcID;

enum EXTERN_DEVICE_RFID
{
	EXTERN_RF_SKU_INVALID             = -1,
	EXTERN_CN_SKU                     =  0,
	EXTERN_WW_SKU                     =  1,
	EXTERN_CN_SKU_LOW                 =  2,
	EXTERN_WW_SKU_LOW                 =  3,
};
extern enum EXTERN_DEVICE_RFID g_EXTERN_ASUS_rfID;

enum EXTERN_DEVICE_FPID
{
	EXTERN_FP_VENDOR_INVALID          = -1,
	EXTERN_FP_VENDOR1                 =  0,
	EXTERN_FP_VENDOR2                 =  1
};
extern enum EXTERN_DEVICE_FPID g_EXTERN_ASUS_fpID;

enum EXTERN_DEVICE_BCID
{
	EXTERN_BC_ID_INVALID              = -1,
	EXTERN_BC_ID_AURA_Light           =  0,
	EXTERN_BC_ID_PMOLED               =  1
};
extern enum EXTERN_DEVICE_BCID g_EXTERN_ASUS_bcID;

enum EXTERN_DEVICE_SECDISPID
{
	EXTERN_SEC_DISP_ID_INVALID        = -1,
	EXTERN_SEC_DISP_ID_MONO           =  0,
	EXTERN_SEC_DISP_ID_COLOR          =  1
};
extern enum EXTERN_DEVICE_SECDISPID g_EXTERN_ASUS_secdispID;
#endif //#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
#endif //#ifndef _ASUS_GLOBAL_VAR_H
