obj-y 	+= ALSPSsensor_Hardware.o
obj-n	+= cm36686/
obj-n	+= ap3425/
obj-n	+= ap3045/
obj-y   += vcnl36866/
ifeq ($(CONFIG_MACH_ASUS_VODKA),y)
obj-y   += ALSPSsensor_Hardware_2nd.o
obj-y	+= vcnl36866_2nd/
endif

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DASH_HW_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)

