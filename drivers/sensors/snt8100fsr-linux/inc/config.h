/*****************************************************************************
* File: config.h
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
*
*
*****************************************************************************/
#include <linux/interrupt.h>
#include "firmware.h"
#include "serial_bus.h"

#ifndef CONFIG_H
#define CONFIG_H

#define SNT_VERSION     "Rel.3.6.3"
#define USE_MT_DEMO_APP
/*
 * Select which bus to use. Only one bus can be active at a time.
 */
//#define USE_SPI_BUS
#define USE_I2C_BUS

/* FIRMWARE_IRQ_TIMEOUT_MS: Amount of time to wait for an interrupt during
 * firmware upload in ms. The timer includes the time to transfer a single
 * payload, so ensure it's long enough.
 */

/*
 *  To support boot-from-flash mode:
 * //#define USE_SPI_BUS
 * #define USE_I2C_BUS
 *
 * in #ifdef USE_I2C_BUS
 * //#define UPLOAD_FIRMWARE
 * #define SUPPORT_FLASH
 * // If you want snt_i2c_write DBUG: message every 512 bytes, uncomment,
 * //#define SNT_I2C_WRITE_DEBUG_VERBOSE
 * //#define SNT_I2C_READ_FIFO_PKT_DEBUG_VERBOSE
 */


/*
 * Custom BUS Settings
 */
#ifdef USE_SPI_BUS
#define SENTONS_DRIVER_NAME "snt8100fsr-spi"
#define UPLOAD_FIRMWARE
#define SNT_FWDL_BUF_SIZE         (1024 * 16)
//#define SNT_FWDL_BUF_SIZE         64
#define FIRMWARE_IRQ_TIMEOUT_MS   2000
// uncomment to add feature.
#define DYNAMIC_PWR_CTL		1		// 1 - enable by default
										// 0 - disable by default
#endif

#ifdef USE_I2C_BUS
#define SENTONS_DRIVER_NAME "snt8100fsr-i2c"
#define SENTONS_DRIVER_WAKE_DEVICE_NAME "snt8100fsr-i2c-wake"
#define UPLOAD_FIRMWARE
// comment out to remove feature.
#define DYNAMIC_PWR_CTL      1       // 1 - enable by default
//#define SUPPORT_FLASH
#define CONFIG_I2C_MODULE                   // (used by linux/i2c.h)
#define SNT_FWDL_BUF_SIZE         512
#define FIRMWARE_IRQ_TIMEOUT_MS   15000      // supports 400KHz
#define FIRMWARE_IRQ_TIMEOUT_MS_RST   15000      // supports 400KHz
#define SNT_I2C_WRITE_DEBUG_VERBOSE
#define SNT_I2C_READ_FIFO_PKT_DEBUG_VERBOSE
#define FW_FAIL_REDOWNLOAD_LIMIT    1
#define GRIP_WAIT_FW_PROBE    10
#endif


/*
 * if this is commented out then touch reporting will not be enabled by default
 */
//#define TOUCH_ENABLED_AT_STARTUP

/*
 * Set to 1 to use the DTS nodes for configuration. Refer to device.c.
 * If set to 0, then use this config.h file for configuration.
 */
#define USE_DEVICE_TREE_CONFIG 0

// SysFS name
#define SYSFS_NAME "snt8100fsr"

// SySFS permissions for showing, storing, or both
#define SYSFS_PERM_SHOW (S_IRUSR|S_IRUGO)
#define SYSFS_PERM_STORE (S_IWUSR|S_IRUGO)
#define SYSFS_PERM_SHOW_STORE (S_IWUSR|S_IRUSR|S_IRUGO|S_IWGRP)

// The default frame rate to sample frames from the sensor hardware in hz
#define DEFAULT_FRAME_RATE_CN 100
#define DEFAULT_FRAME_RATE 50

// The location on disk of the firmware to upload to the hardware on boot
#define FW_PATH   "/vendor/etc/grip_fw/snt8100fsr.image"

/* ASUS BSP Clay: load different fw version according to HWID +++ */
#define ER_FW_PATH   "/vendor/etc/grip_fw/snt8100fsr.image_2.12.43"
/* ASUS BSP Clay: load different fw version according to HWID --- */

// The location of the register configuration files, leave a trailing /.
#define REGISTER_FILES_PATH "/sys/snt8100fsr/"

#define BOOT_INIT_REG_LOCATION "/vendor/factory/snt_reg_init"
#define BOOT_INIT_REG1_LOCATION "/vendor/etc/grip_cal/boot_profile_rscript.txt"
#define BOOT_INIT_BUFFER_SIZE  1024
/*
 * The prefix sysFS and register configuration files will have, for example,
 * if the prefix is "register_" and the register name is "frame_rate", then
 * the register would be labeled as: "register_frame_rate".
 */
#define REGISTER_PREFIX "register_"

// The location of the track report log file
#define TRACK_REPORT_LOG_LOCATION "/data/track_report.log"

// The location of the binary track report log file
#define TRACK_REPORT_BIN_LOG_LOCATION "/data/track_report.bin"

// The location of the deep trace file
#define DEEP_TRACE_LOCATION       "/data/deep_trace.bin"

// The location of the event log file
#define EVENT_LOG_LOCATION        "/data/event_log.bin"

// The location of the small sensor data log file
#define D1TEST_DATA_LOG_LOCATION "/data/d1test_data.log"

// The location of the small sensor data log file
#define FRAME_DATA_LOG_LOCATION "/data/frame_data.log"

// The location of the no touch data log file
#define NO_TOUCH_FRAME_DATA_LOG_LOCATION "/data/no_touch_frame_data.log"

// The location of the flash register partition read file
#define FRP_CAPTURE_FILE_LOCATION "/data/frp_out.txt"

// SPI Bus Settings if not loaded via DeviceTree nodes (bottom of file for chart)
#define SPI_MAX_SPEED_HZ    1000000
#define SPI_MAX_SPEED_KHZ    1000
#define SPI_BUS             1
#define SPI_CHIPSELECT      0

/*
 * The amount of time in microseconds to keep chip select active to wake up
 * the device.
 */
#define SPI_WAKE_CHIP_SELECT_TIME 100

/*
 * the maximum size of an I2c bus transfer. Adjust this to fit OS xfer
 * requirement for i2c.
 */
 #define I2C_MAX_PKT_SIZE_BYTES     512

/*
 * The frame rate to use when the operating system is suspended in hz.
 * Use 5 for Squeeze to Wake mode in which the user squeezes the sensors
 * in order to wake up the device via interrupt.
 * Use 65535 for Deep Sleep in which the sensors will be disabled and the only
 * way to wake up is via sb_wake_device() call, usually made from dev resume()
 *
 * This value is saved into snt8100fsr->suspended_frame_rate and can be
 * updated dynamically at runtime in code and via sysfs in sysfs.c.
 */
#define DEFAULT_SUSPENDED_FRAME_RATE 5
#define DEEP_SLEEP_FRAME_RATE 0xFF
// Should we use the IRQ to perform the firmware upload?
#define FIRMWARE_UPLOAD_WITH_IRQ    true

/*
 * enable these defined to turn Trigger Interrupt support
 */
#define USE_TRIG0_IRQ               0
#define USE_TRIG1_IRQ               0
#define USE_TRIG2_IRQ               0
#define USE_TRIG3_IRQ               0

#define USE_TRIG_IRQ        (USE_TRIG0_IRQ||USE_TRIG1_IRQ||USE_TRIG2_IRQ||USE_TRIG3_IRQ)

// Boot Configuration Record (BCR)
// values found in firmware.h
#define BCR_LOGGING_LEVEL   BCR_LOGLVL_OFF // BCR_LOGLVL_DEFAULT

// May set to 0x2c 0x2d 0x5c 0x5d 0x00
#define BCR_ADDRESS_FILTER    0x00

// Boot Configuration Record
// Interrupt shaping configuration. Note: DEFAULT IS STRONGLY RECOMMENDED
// AS THE FIRST RESET INTERRUPT WILL ALWAYS BE LEVEL, ACTIVE HIGH
#define BCR_INTERRUPT_POLARITY      BCR_IRQ_POLARITY_ACTIVE_HIGH
#define BCR_INTERRUPT_TYPE          BCR_IRQ_TYPE_PULSE

// Delay in IRQ'less Firmware Uploads due to Boot Rom LOG Glitch
#define FIRMWARE_LOG_DELAY_MS 2

// For firmware uploads not using the IRQ, use this delay
#define FIRMWARE_UPLOAD_DELAY_MS 100

// Our interrupt name for the GPIO19 pin interrupt
#define IRQ_NAME            "host_157_int"
#define TRIG0_NAME          "trig0_112_int"
#define TRIG1_NAME          "trig1_26_int"
#define TRIG2_NAME          "trig2_22_int"
#define TRIG3_NAME          "trig3_99_int"

#define SNT_HOST1_NAME "qcom,snt-host1-gpio"
#define SNT_RST_NAME "rst_gpios"
#define SNT_TRIG0_NAME "qcom,snt-trig0-gpio"
#define SNT_TRIG1_NAME "qcom,snt-trig1-gpio"
#define SNT_TRIG2_NAME "qcom,snt-trig2-gpio"
#define SNT_TRIG3_NAME "qcom,snt-trig3-gpio"
#define SNT_ID_NAME "snt-id-gpio"

#define GRIP_SOC_WAKE_ON		"snt_wake_up"
#define GRIP_SOC_GPIO51_OFF		"snt_wake_down"



#define GRIP_1V2_ON			"snt_1v2_active"
#define GRIP_1V2_OFF		"snt_1v2_close"
#define GRIP_RST_ON		"gpio12_rst_on"
#define GRIP_RST_OFF	"gpio12_rst_off"
#define GRIP_TRIG1_NO_PULL "FT_TRIG1_suspend"
#define GRIP_TRIG2_NO_PULL "FT_TRIG2_suspend"
#define GRIP_INT_SUSPEND "FT_INT_suspend"
#define GRIP_WAKE_SUSPEND "snt_wake"
// This is used to calculate device pin value from the GPIO pin value
#define BEAGLEBONE_GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

// Our used GPIO pins
#define BEAGLEBONE_GPIO48 BEAGLEBONE_GPIO_TO_PIN(1, 16)
#define BEAGLEBONE_GPIO49 BEAGLEBONE_GPIO_TO_PIN(1, 17)
#define BEAGLEBONE_GPIO20 BEAGLEBONE_GPIO_TO_PIN(0, 20)
#define BEAGLEBONE_GPIO60 BEAGLEBONE_GPIO_TO_PIN(1, 28)

// The GPIO we will use for interrupt handling
#define RST_GPIO    12
#define IRQ_GPIO    157

#define TRIG0_GPIO_EVB  91
#define TRIG2_GPIO_EVB  93
#define TRIG0_GPIO  112
#define TRIG1_GPIO  26
#define TRIG2_GPIO  22
#define TRIG3_GPIO  99

// The Interrupt number
#define IRQ_NUMBER     gpio_to_irq(IRQ_GPIO)
#define TRIG0_IRQ_NUM  gpio_to_irq(TRIG0_GPIO)
#define TRIG1_IRQ_NUM  gpio_to_irq(TRIG1_GPIO)
#define TRIG2_IRQ_NUM  gpio_to_irq(TRIG2_GPIO)
#define TRIG3_IRQ_NUM  gpio_to_irq(TRIG3_GPIO)

// The triggers for the interrupt handler to execute on
#define IRQ_TRIGGER     IRQF_TRIGGER_RISING | IRQF_ONESHOT
#define TRIG0_TRIGGER   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
#define TRIG1_TRIGGER   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
#define TRIG2_TRIGGER   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
#define TRIG3_TRIGGER   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
//#define TRIG0_TRIGGER   IRQF_TRIGGER_RISING
//#define TRIG1_TRIGGER   IRQF_TRIGGER_RISING
//#define TRIG2_TRIGGER   IRQF_TRIGGER_RISING

/*
 * From: https://groups.google.com/forum/#!topic/beagleboard/Lo0GEl1RdbU
 * Beagle Bone Black SPI Clock Speed Chart
 * CLKD	Divide by	SPI_CLK [Hz]
 * 0	1       48,000,000
 * 1	2       24,000,000
 * 2	4       12,000,000
 * 3	8       6,000,000
 * 4	16      3,000,000
 * 5	32      1,500,000
 * 6	64      750,000
 * 7	128     375,000
 * 8	256     187,500
 * 9	512     93,750
 * 10	1024	46,875
 * 11	2048	23,438
 * 12	4096	11,719
 * 13	8192	5,859
 * 14	16384	2,930
 * 15	32768	1,465
 */

/*
 * Set up 64 vs 32 bit arch detection. Note this only works for arm right now
 */
#ifdef __aarch64__
#define ENVIRONMENT64
#define PTR_2_UINT uint64_t
#else
#define ENVIRONMENT32
#define PTR_2_UINT  uint32_t
#endif


#endif // CONFIG_H

