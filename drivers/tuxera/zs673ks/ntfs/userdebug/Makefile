# Rules for making Tuxera NTFS.
#
# Copyright (c) 2009-2019 Anton Altaparmakov & Tuxera Inc. All rights reserved.
#
# You can set various options for the NTFS driver here:

# Set this to the name of the generated file system you want.  This will also
# become the kernel module name.  Normally this is just "tntfs".
CONFIG_NTFS_NAME = tntfs

ifeq ($(wildcard include/linux/compile.h include/generated/compile.h),)
EXTRA_CFLAGS += -DTUXERA_NO_COMPILE_H -DUTS_MACHINE=\"$(UTS_MACHINE)\"
ifdef CONFIG_SMP
SMP=SMP
endif
ifdef CONFIG_PREEMPT
PREEMPT=preempt
endif
EXTRA_CFLAGS += -DUTS_VERSION=\"$(SMP)\ $(PREEMPT)\ UNKNOWN\"
endif

obj-m += $(CONFIG_NTFS_NAME).o

$(CONFIG_NTFS_NAME)-objs :=	aops.o \
				attr_list.o \
				attrib.o \
				bitmap.o \
				collate.o \
				compat.o \
				compress.o \
				debug.o \
				dir.o \
				file.o \
				index.o \
				inode.o \
				iupdate.o \
				lcnalloc.o \
				logfile.o \
				mft.o \
				mst.o \
				namei.o \
				quota.o \
				runlist.o \
				secure.o \
				super.o \
				sysctl.o \
				tuts.o \
				unistr.o \
				usnjrnl.o \
				xattr.o
