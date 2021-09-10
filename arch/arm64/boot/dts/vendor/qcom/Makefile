
# +++ ASUS_BSP : add for asus dtbo
$(warning [Build] build dtbo CONFIG_BUILD_ARM64_DT_OVERLAY = $(CONFIG_BUILD_ARM64_DT_OVERLAY))
$(warning [Build] build dtbo ASUS_BUILD_PROJECT = $(ASUS_BUILD_PROJECT))
$(warning [Build] build dtbo CONFIG_ARCH_QTI_VM = $(CONFIG_ARCH_QTI_VM))
# --- ASUS_BSP : add for asus dtbo

ifneq ($(CONFIG_ARCH_QTI_VM), y)
ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)

# +++ ASUS_BSP : add for asus dtbo
ifneq (,$(filter ZS673KS,$(ASUS_BUILD_PROJECT)))
$(warning build dtbo for ZS673KS...)

dtbo-$(CONFIG_ARCH_LAHAINA) += \
                ZS673KS-EVB-overlay.dtbo \
                ZS673KS-EVB2-overlay.dtbo \
                ZS673KS-SR1-overlay.dtbo \
                ZS673KS-ER1-overlay.dtbo \
                ZS673KS-ER2-overlay.dtbo \
                ZS673KS-PR1-overlay.dtbo \
                ZS673KS-MP-overlay.dtbo \
                ZS673KS-MP3-overlay.dtbo \
                ZS673KS-MP4-overlay.dtbo \
                ZS673KS-MP5-overlay.dtbo \

lahaina-mtp-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
ZS673KS-EVB-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
ZS673KS-EVB2-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
ZS673KS-SR1-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
ZS673KS-ER1-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
ZS673KS-ER2-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
ZS673KS-PR1-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
ZS673KS-MP-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
ZS673KS-MP3-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
ZS673KS-MP4-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
ZS673KS-MP5-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb

else ifneq (,$(filter PICASSO,$(ASUS_BUILD_PROJECT)))

dtbo-$(CONFIG_ARCH_LAHAINA) += \
                PICASSO-EVB-overlay.dtbo \
                PICASSO-EVB2-overlay.dtbo \
                PICASSO-SR1-overlay.dtbo \
                PICASSO-ER1-overlay.dtbo \

PICASSO-EVB-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
PICASSO-EVB2-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
PICASSO-SR1-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
PICASSO-ER1-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb


else ifneq (,$(filter SAKE,$(ASUS_BUILD_PROJECT)))
$(warning build dtbo for SAKE...)

dtbo-$(CONFIG_ARCH_LAHAINA) += \
                SAKE-EVB-overlay.dtbo \
                SAKE-EVB2-overlay.dtbo \
                SAKE-SR1-overlay.dtbo \

lahaina-mtp-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
SAKE-EVB-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
SAKE-EVB2-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
SAKE-SR1-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb	
else ifneq (,$(filter VODKA,$(ASUS_BUILD_PROJECT)))
$(warning build dtbo for VODKA...)

dtbo-$(CONFIG_ARCH_LAHAINA) += \
                VODKA-EVB-overlay.dtbo \
                VODKA-EVB2-overlay.dtbo \
                VODKA-SR1-overlay.dtbo \

lahaina-mtp-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
VODKA-EVB-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
VODKA-EVB2-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
VODKA-SR1-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
# --- ASUS_BSP : add for asus dtbo
else
dtbo-$(CONFIG_ARCH_LAHAINA) += lahaina-rumi-overlay.dtbo \
	lahaina-mtp-overlay.dtbo \
	lahaina-mtp-hsp-overlay.dtbo \
	lahaina-cdp-overlay.dtbo \
	lahaina-qrd-overlay.dtbo \
	lahaina-atp-overlay.dtbo \
	lahaina-hdk-overlay.dtbo \
	lahaina-qrd-hsp-overlay.dtbo \
	lahaina-qrd-module-overlay.dtbo \
	lahainap-mtp-overlay.dtbo \
	lahainap-cdp-overlay.dtbo \
	lahainap-atp-overlay.dtbo \
	lahainap-qrd-overlay.dtbo \
	lahaina-cdp-v2.2-overlay.dtbo \
	lahaina-mtp-v2-overlay.dtbo \
	lahaina-mtp-v2.1-overlay.dtbo

lahaina-rumi-overlay.dtbo-base := lahaina.dtb
lahaina-mtp-hsp-overlay.dtbo-base := lahaina.dtb
lahaina-mtp-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
lahaina-cdp-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
lahaina-atp-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
lahaina-hdk-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
lahaina-qrd-hsp-overlay.dtbo-base := lahaina.dtb
lahaina-qrd-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
lahaina-qrd-module-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb

lahainap-mtp-overlay.dtbo-base := lahainap.dtb lahainap-v2.dtb lahainap-v2.1.dtb
lahainap-cdp-overlay.dtbo-base := lahainap.dtb lahainap-v2.dtb lahainap-v2.1.dtb
lahainap-atp-overlay.dtbo-base := lahainap.dtb lahainap-v2.dtb lahainap-v2.1.dtb
lahainap-qrd-overlay.dtbo-base := lahainap.dtb lahainap-v2.dtb lahainap-v2.1.dtb
lahaina-cdp-v2.2-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb \
					lahaina-v2.1.dtb
lahaina-mtp-v2-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb \
					lahaina-v2.1.dtb
lahaina-mtp-v2.1-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb \
					lahaina-v2.1.dtb
endif
else
dtb-$(CONFIG_ARCH_LAHAINA) += lahaina-rumi.dtb \
	lahaina-mtp-hsp.dtb \
	lahaina-mtp.dtb \
	lahaina-cdp.dtb \
	lahaina-atp.dtb \
	lahaina-hdk.dtb \
	lahaina-qrd-hsp.dtb \
	lahaina-qrd.dtb \
	lahaina-qrd-module.dtb \
	lahainap-mtp.dtb \
	lahainap-cdp.dtb \
	lahainap-atp.dtb \
	lahainap-qrd.dtb \
	lahaina-v2-mtp.dtb \
	lahaina-v2-cdp.dtb \
	lahaina-v2-atp.dtb \
	lahaina-v2-hdk.dtb \
	lahaina-v2-qrd.dtb \
	lahaina-v2-qrd-module.dtb \
	lahainap-v2-mtp.dtb \
	lahainap-v2-cdp.dtb \
	lahainap-v2-atp.dtb \
	lahainap-v2-qrd.dtb \
	lahaina-cdp-v2.2.dtb \
	lahaina-v2.1-mtp.dtb \
	lahaina-v2.1-cdp.dtb \
	lahaina-v2.1-atp.dtb \
	lahaina-v2.1-hdk.dtb \
	lahaina-v2.1-qrd.dtb \
	lahaina-v2.1-qrd-module.dtb \
	lahainap-v2.1-mtp.dtb \
	lahainap-v2.1-cdp.dtb \
	lahainap-v2.1-atp.dtb \
	lahainap-v2.1-qrd.dtb \
	lahaina-mtp-v2.dtb \
	lahaina-mtp-v2.1.dtb
endif

ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_ARCH_SCUBA) += scuba-iot-idp-overlay.dtbo

scuba-iot-idp-overlay.dtbo-base := scuba-iot.dtb
endif

ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_ARCH_HOLI) += holi-rumi-overlay.dtbo \
	holi-mtp-overlay.dtbo \
	holi-cdp-overlay.dtbo \
	holi-cdp-lcd-overlay.dtbo \
	holi-qrd-overlay.dtbo \
	holi-atp-overlay.dtbo \
	holi-mtp-usbc-overlay.dtbo

holi-rumi-overlay.dtbo-base := holi.dtb
holi-mtp-overlay.dtbo-base := holi.dtb
holi-cdp-overlay.dtbo-base := holi.dtb
holi-cdp-lcd-overlay.dtbo-base := holi.dtb
holi-qrd-overlay.dtbo-base := holi.dtb
holi-atp-overlay.dtbo-base := holi.dtb
holi-mtp-usbc-overlay.dtbo-base := holi.dtb
else
dtb-$(CONFIG_ARCH_HOLI) += holi-rumi.dtb \
	holi-mtp.dtb \
	holi-cdp.dtb \
	holi-cdp-lcd.dtb \
	holi-qrd.dtb \
	holi-atp.dtb \
	holi-mtp-usbc.dtb
endif

ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_ARCH_SHIMA) +=    \
	shima-rumi-overlay.dtbo \
	shima-idp-overlay.dtbo  \
	shima-idp-120fps-overlay.dtbo \
	shima-atp-overlay.dtbo \
	shima-qrd-overlay.dtbo \
	shima-idps-digital-mics-overlay.dtbo \
	shima-idps-digital-mics-120fps-overlay.dtbo

shima-rumi-overlay.dtbo-base := shima.dtb
shima-idp-overlay.dtbo-base := shima.dtb
shima-idp-120fps-overlay.dtbo-base := shima.dtb
shima-atp-overlay.dtbo-base := shima.dtb
shima-qrd-overlay.dtbo-base := shima.dtb
shima-idps-digital-mics-overlay.dtbo-base := shima.dtb
shima-idps-digital-mics-120fps-overlay.dtbo-base := shima.dtb
else
dtb-$(CONFIG_ARCH_SHIMA) += shima-rumi.dtb  \
			    shima-idp.dtb   \
			    shima-idp-120fps.dtb  \
			    shima-atp.dtb \
			    shima-qrd.dtb \
			    shima-idps-digital-mics.dtb \
			    shima-idps-digital-mics-120fps.dtb
endif

ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_ARCH_YUPIK) +=    \
	yupik-rumi-overlay.dtbo

yupik-rumi-overlay.dtbo-base := yupik.dtb
else
dtb-$(CONFIG_ARCH_YUPIK) += yupik-rumi.dtb
endif

ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
        dtbo-$(CONFIG_ARCH_SA8155) += \
		sa8155-adp-star-overlay.dtbo \
		sa8155p-adp-star-overlay.dtbo \
		sa8155-v2-adp-air-overlay.dtbo \
		sa8155p-v2-adp-air-overlay.dtbo

sa8155-adp-star-overlay.dtbo-base := sa8155.dtb sa8155-v2.dtb
sa8155p-adp-star-overlay.dtbo-base := sa8155p.dtb sa8155p-v2.dtb
sa8155-v2-adp-air-overlay.dtbo-base := sa8155.dtb sa8155-v2.dtb
sa8155p-v2-adp-air-overlay.dtbo-base := sa8155p.dtb sa8155p-v2.dtb
else
dtb-$(CONFIG_ARCH_SA8155)       += sa8155-adp-star.dtb \
	sa8155p-adp-star.dtb \
	sa8155-v2-adp-star.dtb \
	sa8155p-v2-adp-star.dtb \
	sa8155-v2-adp-air.dtb \
	sa8155p-v2-adp-air.dtb
endif

ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
	dtbo-$(CONFIG_ARCH_SA8195) += \
		sa8195p-adp-star-overlay.dtbo \
		sa8195p-v2-adp-air-overlay.dtbo

sa8195p-adp-star-overlay.dtbo-base := sa8195p.dtb
sa8195p-v2-adp-air-overlay.dtbo-base := sa8195p.dtb
else
dtb-$(CONFIG_ARCH_SA8195)	+= sa8195p-adp-star.dtb \
	sa8195p-v2-adp-air.dtb
endif
endif

ifeq ($(CONFIG_ARCH_LAHAINA), y)
ifeq ($(CONFIG_ARCH_QTI_VM), y)
dtb-$(CONFIG_ARCH_QTI_VM) += lahaina-vm-mtp.dtb \
	lahaina-vm-cdp.dtb \
	lahaina-vm-qrd.dtb \
	lahaina-vm-mtp-v2.1.dtb
endif
endif

dtb-$(CONFIG_ARCH_SDXLEMUR) += sdxlemur-rumi.dtb \
	sdxlemur-cdp.dtb \
	sdxlemur-mtp.dtb \
	sdxlemur-mtp-mbb.dtb \
	sdxlemur-mtp-cpe.dtb \
	sdxlemur-mtp-mbb-ntn3-pcie.dtb \
	sdxlemur-mtp-mbb-pcie-ep.dtb \
	sdxlemur-mtp-mbb-pcie-rc-ep.dtb \
	sdxlemur-mtp-mbb.dtb \
	sdxlemur-rcm-pcie-ep.dtb \
	sdxlemur-rcm-rc.dtb


ifeq ($(CONFIG_ARCH_SHIMA), y)
ifeq ($(CONFIG_ARCH_QTI_VM), y)
dtb-$(CONFIG_ARCH_QTI_VM) += shima-vm-rumi.dtb \
	shima-vm-atp.dtb \
	shima-vm-idp.dtb \
	shima-vm-qrd.dtb
endif
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
