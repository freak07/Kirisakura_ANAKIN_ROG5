obj-y += ASH_ATTR.o
obj-y += HALLsensor_ATTR.o
obj-y += psensor_ATTR.o
obj-y += lsensor_ATTR.o
ifeq ($(CONFIG_MACH_ASUS_VODKA),y)
obj-y += psensor_2nd_ATTR.o
obj-y += lsensor_2nd_ATTR.o
endif
obj-y += FRGBsensor_ATTR.o
obj-y += SARsensor_ATTR.o

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DASH_ATTR_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)


