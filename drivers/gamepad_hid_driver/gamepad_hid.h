#ifndef _GAMEPAD_HID_H_
#define _GAMEPAD_HID_H_

#define ASUS_GAMEPAD_GET_RED_PWM            0x01
#define ASUS_GAMEPAD_GET_GREEN_PWM          0x02
#define ASUS_GAMEPAD_GET_BLUE_PWM           0x03
#define ASUS_GAMEPAD_GET_MODE               0x04
#define ASUS_GAMEPAD_GET_FRAME              0x06
#define ASUS_GAMEPAD_GET_FW_VERSION         0x07
#define ASUS_GAMEPAD_GET_SPEED              0x08
#define ASUS_GAMEPAD_GET_LED_ON             0x09
#define ASUS_GAMEPAD_GET_FWMODE             0x0A

#define ASUS_GAMEPAD_SET_RED_PWM            0x81
#define ASUS_GAMEPAD_SET_GREEN_PWM          0x82
#define ASUS_GAMEPAD_SET_BLUE_PWM           0x83
#define ASUS_GAMEPAD_SET_MODE               0x84
#define ASUS_GAMEPAD_SET_APPLY              0x85
#define ASUS_GAMEPAD_SET_FRAME              0x86
#define ASUS_GAMEPAD_SET_SPEED              0x88
#define ASUS_GAMEPAD_SET_LED_ON             0x89

#define ASUS_GAMEPAD_REPORT_ID              0x00
#define ASUS_GAMEPAD_REPORT_TYPE            0x01

// For Read FW
#define ASUS_GAMEPAD_FW_REPORT_ID           0xA1
#define ASUS_GAMEPAD_GET_MIDDLE_FW_VERSION  0x0C
#define ASUS_GAMEPAD_GET_RIGHT_FW_VERSION   0x0D
#define ASUS_GAMEPAD_GET_BT_FW_VERSION      0x0E

#endif

