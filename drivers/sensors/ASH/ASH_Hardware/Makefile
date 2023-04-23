obj-y += ASH_Hardware.o
obj-y += HALLsensor_Hardware.o
obj-y += ALSPSsensor_Hardware/
obj-n += psensor_Hardware/
obj-n += lsensor_Hardware/
obj-n += ALSPS_FRGB_Hardware/
obj-n += SARsensor_Hardware/

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DASH_I2C_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)
