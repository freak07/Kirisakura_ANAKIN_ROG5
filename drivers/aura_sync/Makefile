#
# Makefile for aura sync drivers.
#
ccflags-y += -DDEFAULT_SYMBOL_NAMESPACE=ANDROID_GKI_VFS_EXPORT_ONLY
ccflags-y += -DANDROID_GKI_VFS_EXPORT_ONLY=VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver

obj-$(CONFIG_AURALIGHT_ENE8K41_PHONE)	+= ene_8k41.o
obj-$(CONFIG_AURALIGHT_MS51_SIDE)		+= ms51_side.o
obj-$(CONFIG_AURALIGHT_MS51_PHONE)		+= ms51_phone.o
obj-$(CONFIG_AURALIGHT_MS51_BACKCOVER)	+= ms51_backcover.o
obj-y	+= rog5_inbox.o

#obj-y		+= sky81298.o
#obj-m  		+= ml51fb9ae_inbox.o
#obj-m  		+= ms51_inbox.o
#obj-m    	+= ene_6k582_station.o
#obj-m		+= ene_8k41_dock.o

# ASUS_SZ_BSP Yadong : Add for factory test +++
#ifeq ($(ASUS_FTM), y)
#obj-m	+= ene_8k41_inbox.o
#endif
# ASUS_SZ_BSP Yadong : Add for factory test ---

#obj-$(CONFIG_AURA_SYNC_INBOX)	+= ene_8k41_inbox.o
#obj-$(CONFIG_AURA_SYNC_STATION)	+= ene_8k41_station.o
#obj-$(CONFIG_AURA_SYNC_POGO)	+= ene_8k41_pogo.o
