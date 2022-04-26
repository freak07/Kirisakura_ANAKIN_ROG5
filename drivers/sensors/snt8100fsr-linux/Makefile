#
# Makefile for the input core drivers.
#
# Each configuration option enables a list of files
ccflags-y += -I$(srctree)/drivers/sensors/snt8100fsr-linux/inc
#ccflags-y += -Werror
#ccflags-y += -Idrivers/sensors/snt8100fsr-linux/inc

obj-y        += crc.o
obj-y        += customize.o
obj-y        += device.o
obj-y        += event.o
obj-y        += file.o
obj-y        += firmware.o
obj-y        += hardware.o
obj-y        += input_device.o
obj-y        += i2c_bus.o
obj-y        += i2c_wake_bus.o
obj-y        += irq.o
obj-y        += main.o
obj-y        += memory.o
obj-y        += serial_bus.o
obj-y        += spi_bus.o
obj-y        += sysfs.o
obj-y        += utils.o
obj-y        += workqueue.o
obj-y   	+= file_control.o
obj-y           += asus_init.o
DEBUG = n

ifeq ($(DEBUG),y)
        DEBFLAGS = -O -g -DGRIP_DEBUG
else
        DEBFLAGS = -O -g
endif

ccflags-y += $(DEBFLAGS)
