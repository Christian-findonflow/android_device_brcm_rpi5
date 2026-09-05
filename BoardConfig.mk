#
# Copyright (C) 2021-2022 KonstaKANG
#
# SPDX-License-Identifier: Apache-2.0
#

DEVICE_PATH := device/brcm/rpi5

# Architecture
TARGET_ARCH := arm64
TARGET_ARCH_VARIANT := armv8-a
TARGET_CPU_ABI := arm64-v8a
TARGET_CPU_ABI2 :=
TARGET_CPU_VARIANT := cortex-a76

# Bluetooth
BOARD_HAVE_BLUETOOTH := true

# Bootloader
TARGET_NO_BOOTLOADER := true

# Display
TARGET_SCREEN_DENSITY := 120

# Kernel
BOARD_CUSTOM_BOOTIMG := true
BOARD_CUSTOM_BOOTIMG_MK := $(DEVICE_PATH)/mkbootimg.mk
# Kernel cmdline - optimized for fast boot
# quiet: reduce kernel log output
# loglevel=3: only show errors
# logo.nologo: skip raspberry logo
BOARD_KERNEL_CMDLINE := console=ttyAMA10,115200 no_console_suspend root=/dev/ram0 rootwait androidboot.hardware=rpi5 quiet loglevel=3 logo.nologo video=DSI-1:800x480M@60 video=DSI-2:800x480M@60

# Manifest
DEVICE_FRAMEWORK_COMPATIBILITY_MATRIX_FILE := $(DEVICE_PATH)/framework_compatibility_matrix.xml
DEVICE_MANIFEST_FILE := $(DEVICE_PATH)/manifest.xml
PRODUCT_MANIFEST_FILES := $(DEVICE_PATH)/product_manifest.xml

# Partition sizes
BOARD_FLASH_BLOCK_SIZE := 4096
BOARD_USES_METADATA_PARTITION := true
BOARD_BOOTIMAGE_PARTITION_SIZE := 134217728 # 128M
BOARD_METADATAIMAGE_PARTITION_SIZE := 16777216 # 16M
BOARD_SYSTEMIMAGE_PARTITION_SIZE := 3221225472 # 3072M
BOARD_USERDATAIMAGE_PARTITION_SIZE := 134217728 # 128M
BOARD_VENDORIMAGE_PARTITION_SIZE := 402653184 # 384M
BOARD_VENDORIMAGE_FILE_SYSTEM_TYPE := ext4
TARGET_USERIMAGES_SPARSE_EXT_DISABLED := true
TARGET_USERIMAGES_USE_EXT4 := true

# Platform
TARGET_BOARD_PLATFORM := bcm2712
TARGET_BOOTLOADER_BOARD_NAME := bcm2712

# Properties
TARGET_VENDOR_PROP += $(DEVICE_PATH)/vendor.prop

# Recovery
TARGET_NO_RECOVERY := true

# SELinux
BOARD_SEPOLICY_DIRS += device/brcm/rpi5/sepolicy
BOARD_KERNEL_CMDLINE += androidboot.selinux=permissive

# The motorcycle dash has no audio hardware of its own (no HDMI sink, no jack,
# no mic): tell the AIDL audio HAL to back the primary "Speaker"/"Built-In Mic"
# ports with its stub driver. Without this the HAL cannot open card 0 (vc4hdmi
# with nothing attached), the policy manager never initialises, every audio
# track fails, and the Bluetooth stack asserts the moment a phone streams
# (2026-09-05). Real audio goes to Bluetooth earbuds via the bluetooth module.
ifeq ($(TARGET_PRODUCT),aosp_rpi5_motorcycle)
BOARD_KERNEL_CMDLINE += androidboot.audio.tinyalsa.ignore_output=true androidboot.audio.tinyalsa.simulate_input=true
endif

# Treble
TARGET_COPY_OUT_VENDOR := vendor

# Wifi
BOARD_WLAN_DEVICE := bcmdhd
BOARD_HOSTAPD_DRIVER := NL80211
BOARD_HOSTAPD_PRIVATE_LIB := lib_driver_cmd_$(BOARD_WLAN_DEVICE)
BOARD_WPA_SUPPLICANT_DRIVER := NL80211
BOARD_WPA_SUPPLICANT_PRIVATE_LIB := lib_driver_cmd_$(BOARD_WLAN_DEVICE)
WPA_SUPPLICANT_VERSION := VER_0_8_X
