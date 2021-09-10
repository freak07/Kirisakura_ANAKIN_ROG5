#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2020, The Linux Foundation. All rights reserved.

# Script to generate a defconfig variant based on the input

usage() {
	echo "Usage: $0 <platform_defconfig_variant>"
	echo "Variants: <platform>-gki_defconfig, <platform>-qgki_defconfig, <platform>-consolidate_defconfig and <platform>-qgki-debug_defconfig"
	echo "Example: $0 lahaina-gki_defconfig"
	exit 1
}

if [ -z "$1" ]; then
	echo "Error: Failed to pass input argument"
	usage
fi

SCRIPTS_ROOT=$(readlink -f $(dirname $0)/)

TEMP_DEF_NAME=`echo $1 | sed -r "s/_defconfig$//"`
DEF_VARIANT=`echo ${TEMP_DEF_NAME} | sed -r "s/.*-//"`
PLATFORM_NAME=`echo ${TEMP_DEF_NAME} | sed -r "s/-.*$//"`

PLATFORM_NAME=`echo $PLATFORM_NAME | sed "s/vendor\///g"`

REQUIRED_DEFCONFIG=`echo $1 | sed "s/vendor\///g"`

# We should be in the kernel root after the envsetup
if [[  "${REQUIRED_DEFCONFIG}" != *"gki"* ]]; then
	source ${SCRIPTS_ROOT}/envsetup.sh $PLATFORM_NAME generic_defconfig
else
	source ${SCRIPTS_ROOT}/envsetup.sh $PLATFORM_NAME
fi

KERN_MAKE_ARGS="ARCH=$ARCH \
		CROSS_COMPILE=$CROSS_COMPILE \
		REAL_CC=$REAL_CC \
		CLANG_TRIPLE=$CLANG_TRIPLE \
		HOSTCC=$HOSTCC \
		HOSTLD=$HOSTLD \
		HOSTAR=$HOSTAR \
		LD=$LD \
		"

# Allyes fragment temporarily created on GKI config fragment
QCOM_GKI_ALLYES_FRAG=${CONFIGS_DIR}/${PLATFORM_NAME}_ALLYES_GKI.config

echo "[Build] config.sh QCOM_GKI_ALLYES_FRAG=$QCOM_GKI_ALLYES_FRAG" >> generate_defconfig.txt

if [[ "${REQUIRED_DEFCONFIG}" == *"gki"* ]]; then
if [ ! -f "${QCOM_GKI_FRAG}" ]; then
	echo "Error: Invalid input"
	usage
fi
fi

FINAL_DEFCONFIG_BLEND=""

echo "[Build] config.sh REQUIRED_DEFCONFIG=$REQUIRED_DEFCONFIG" >> generate_defconfig.txt

case "$REQUIRED_DEFCONFIG" in
	${PLATFORM_NAME}-qgki-debug_defconfig )
		FINAL_DEFCONFIG_BLEND+=" $QCOM_DEBUG_FRAG"
		echo "[Build] config.sh FINAL_DEFCONFIG_BLEND (1)+=$FINAL_DEFCONFIG_BLEND" >> generate_defconfig.txt
		;&	# Intentional fallthrough
	${PLATFORM_NAME}-qgki-consolidate_defconfig )
		FINAL_DEFCONFIG_BLEND+=" $QCOM_CONSOLIDATE_FRAG"
		echo "[Build] config.sh FINAL_DEFCONFIG_BLEND (2)+=$FINAL_DEFCONFIG_BLEND" >> generate_defconfig.txt
		;&	# Intentional fallthrough
	${PLATFORM_NAME}-qgki_defconfig )
		# DEBUG_FS fragment.
		FINAL_DEFCONFIG_BLEND+=" $QCOM_DEBUG_FS_FRAG"

		FINAL_DEFCONFIG_BLEND+=" $QCOM_QGKI_FRAG"
		${SCRIPTS_ROOT}/fragment_allyesconfig.sh $QCOM_GKI_FRAG $QCOM_GKI_ALLYES_FRAG
		FINAL_DEFCONFIG_BLEND+=" $QCOM_GKI_ALLYES_FRAG "

		echo "[Build] config.sh FINAL_DEFCONFIG_BLEND (3)+=$FINAL_DEFCONFIG_BLEND" >> generate_defconfig.txt
		;;
	${PLATFORM_NAME}-gki_defconfig )
		FINAL_DEFCONFIG_BLEND+=" $QCOM_GKI_FRAG "
		echo "[Build] config.sh FINAL_DEFCONFIG_BLEND (4)+=$FINAL_DEFCONFIG_BLEND" >> generate_defconfig.txt
		;;
	${PLATFORM_NAME}-debug_defconfig )
		FINAL_DEFCONFIG_BLEND+=" $QCOM_GENERIC_DEBUG_FRAG "
		echo "[Build] config.sh FINAL_DEFCONFIG_BLEND (5)+=$FINAL_DEFCONFIG_BLEND" >> generate_defconfig.txt
		;&
	${PLATFORM_NAME}_defconfig )
		FINAL_DEFCONFIG_BLEND+=" $QCOM_GENERIC_PERF_FRAG "
		echo "[Build] config.sh FINAL_DEFCONFIG_BLEND (6)+=$FINAL_DEFCONFIG_BLEND" >> generate_defconfig.txt
		;;
esac

FINAL_DEFCONFIG_BLEND+=${BASE_DEFCONFIG}

# Reverse the order of the configs for the override to work properly
# Correct order is base_defconfig GKI.config QGKI.config consolidate.config debug.config
FINAL_DEFCONFIG_BLEND=`echo "${FINAL_DEFCONFIG_BLEND}" | awk '{ for (i=NF; i>1; i--) printf("%s ",$i); print $1; }'`

echo "[Build] config.sh FINAL_DEFCONFIG_BLEND=$FINAL_DEFCONFIG_BLEND" >> generate_defconfig.txt

echo "+++[Build]+++" >> generate_defconfig.txt
echo FINAL_DEFCONFIG_BLEND=$FINAL_DEFCONFIG_BLEND
echo "[Build] config.sh ONLY_GKI=$ONLY_GKI" >> generate_defconfig.txt
if [ "$ONLY_GKI" == "0" ]; then
echo "[Build] config.sh ASUS_BUILD_PROJECT=$ASUS_BUILD_PROJECT" >> generate_defconfig.txt
echo "[Build] config.sh TARGET_BUILD_VARIANT=$TARGET_BUILD_VARIANT" >> generate_defconfig.txt
ASUS_CONFIGS_DIR="${KERN_SRC}/arch/${ARCH}/configs/vendor/"
echo "[Build] config.sh ASUS_CONFIGS_DIR=$ASUS_CONFIGS_DIR" >> generate_defconfig.txt
if [ "$ASUS_BUILD_PROJECT" == "ZS673KS" ] || [ "$ASUS_BUILD_PROJECT" == "PICASSO" ]; then
	if [ "$TARGET_BUILD_VARIANT" == "userdebug" ]; then
		ASUS_DEFCINFIG=$ASUS_BUILD_PROJECT"_defconfig"
		echo "[Build] userdebug : ASUS_DEFCINFIG=$ASUS_DEFCINFIG" >> generate_defconfig.txt
		FINAL_DEFCONFIG_BLEND+=" $ASUS_CONFIGS_DIR$ASUS_DEFCINFIG"
	else
		echo "[Build] user : ASUS_DEFCINFIG=$ASUS_DEFCINFIG" >> generate_defconfig.txt
		ASUS_DEFCINFIG=$ASUS_BUILD_PROJECT"-perf_defconfig"
		FINAL_DEFCONFIG_BLEND+=" $ASUS_CONFIGS_DIR$ASUS_DEFCINFIG"
	fi
fi

if [ "$ASUS_BUILD_PROJECT" == "SAKE" ] || [ "$ASUS_BUILD_PROJECT" == "VODKA" ]; then
	if [ "$TARGET_BUILD_VARIANT" == "userdebug" ]; then
		ASUS_DEFCINFIG=$ASUS_BUILD_PROJECT"_defconfig"
		echo "[Build] userdebug : ASUS_DEFCINFIG=$ASUS_DEFCINFIG"
		FINAL_DEFCONFIG_BLEND+=" $ASUS_CONFIGS_DIR$ASUS_DEFCINFIG"
	else
		echo "[Build] user : ASUS_DEFCINFIG=$ASUS_DEFCINFIG"
		ASUS_DEFCINFIG=$ASUS_BUILD_PROJECT"-perf_defconfig"
		FINAL_DEFCONFIG_BLEND+=" $ASUS_CONFIGS_DIR$ASUS_DEFCINFIG"
	fi
fi
else
echo "ASUS_GKI_BUILD"
fi
echo FINAL_DEFCONFIG_BLEND_4=$FINAL_DEFCONFIG_BLEND
# --- ASUS_BSP : add ZS673KS defconfig to overwrite .config

#REQUIRED_DEFCONFIG    =lahaina-qgki-debug_defconfig
echo REQUIRED_DEFCONFIG=$REQUIRED_DEFCONFIG
echo "---[Build]---" >> generate_defconfig.txt

echo "[Build]defconfig blend for $REQUIRED_DEFCONFIG: $FINAL_DEFCONFIG_BLEND" >> generate_defconfig.txt

MAKE_ARGS=$KERN_MAKE_ARGS \
MAKE_PATH=${MAKE_PATH} \
	${KERN_SRC}/scripts/kconfig/merge_config.sh $FINAL_DEFCONFIG_BLEND
${MAKE_PATH}make $KERN_MAKE_ARGS savedefconfig
mv defconfig $CONFIGS_DIR/$REQUIRED_DEFCONFIG

# Cleanup the allyes config fragment and other generated files
rm -rf $QCOM_GKI_ALLYES_FRAG .config include/config/ include/generated/ arch/$ARCH/include/generated/
