# Makefile for the focaltech touchscreen drivers.
#ccflags-y += -DDEFAULT_SYMBOL_NAMESPACE=ANDROID_GKI_VFS_EXPORT_ONLY
ccflags-y += -DANDROID_GKI_VFS_EXPORT_ONLY=VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver
obj-$(CONFIG_TOUCHSCREEN_ROG) := focaltech_fts_rog.o
focaltech_fts_rog-y :=  focaltech_core.o focaltech_ex_fun.o focaltech_ex_mode.o focaltech_gesture.o focaltech_esdcheck.o focaltech_point_report_check.o focaltech_i2c.o focaltech_flash.o focaltech_flash/focaltech_upgrade_rog5.o focaltech_test/focaltech_test.o focaltech_test/focaltech_test_ini.o focaltech_test/supported_ic/focaltech_test_rog5.o
focaltech_fts_rog-y += asus/asus_ex_fun.o asus/asus_game.o asus/asus_gesture.o
