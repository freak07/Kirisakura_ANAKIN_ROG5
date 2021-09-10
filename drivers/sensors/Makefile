
# SPDX-License-Identifier: GPL-2.0-only
#obj-$(CONFIG_SENSORS_SSC)	+= sensors_ssc.o
obj-y                           += icm206xx.o
obj-y                           += ASH/
obj-y                           += laser/
obj-$(CONFIG_MACH_ASUS_ZS673KS)	+= snt8100fsr-linux/

ifeq ($(CONFIG_MACH_ASUS_ZS673KS),y)
KBUILD_CFLAGS += -DONE_PL_CHIP
KBUILD_CFLAGS += -DBACK_PSENSOR
endif

ifeq ($(CONFIG_MACH_ASUS_PICASSO),y)
KBUILD_CFLAGS += -DONE_PL_CHIP
endif

ifeq ($(CONFIG_MACH_ASUS_VODKA),y)
KBUILD_CFLAGS += -DPSENSOR_CAL24
KBUILD_CFLAGS += -DLONG_LENGTH_LOGN_IT_PERF=1
KBUILD_CFLAGS += -DRECOVERY_PSENSOR
endif

#ifeq ($(ASUS_FTM), y)
#KBUILD_CFLAGS += -DLONG_LENGTH_LOGN_IT_NON_PERF=1
#else #Shipping
#KBUILD_CFLAGS += -DLONG_LENGTH_LOGN_IT_PERF=1
#endif #####ifdef ASUS_FTM

