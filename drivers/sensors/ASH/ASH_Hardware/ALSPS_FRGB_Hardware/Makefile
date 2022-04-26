obj-y 	+= ALSPS_FRGB_Hardware.o
obj-y	+= cm36656/
obj-n   += vcnl36863/
obj-n	+= example/

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DASH_HW_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)

