#
# Copyright (C) 2021-2023 KonstaKANG
# Copyright (C) 2024 Motorcycle Dashboard Project
#
# SPDX-License-Identifier: Apache-2.0
#
# Android Automotive build for the NEO motorcycle dashboard.
#
# KNOWN ISSUE (verified 2026-08-29): this product still ships AOSP test/demo apps
# (EmbeddedKitchenSinkApp, AdasLocationTestApp, RotaryPlayground, GarageModeTestApp)
# and phone apps (Calendar, Camera2, DeskClock, Gallery2, PrintSpooler) - they show
# up in the launcher's app grid on a running device.
#
# This file previously used PRODUCT_PACKAGES_REMOVE to drop them. That is a
# LineageOS extension which AOSP's build system does not implement, so it silently
# did nothing (those lines have been removed rather than left as false comfort).
# Post-inheritance $(filter-out ...) on PRODUCT_PACKAGES was also measured and
# removes nothing, because inherit-product flattens variables only after the
# makefile has been read.
#
# RESOLVED 2026-09-03 for the test/demo apps, CarRadioApp and Music: they are
# excluded through soong's `overrides` list on the CarLauncher module
# (packages/apps/Car/Launcher/app/Android.bp) - the mechanism AOSP actually
# supports. The phone apps (Calendar, Camera2, DeskClock, Gallery2,
# PrintSpooler) can go the same way once Christian decides.
#
# The real fix is to inherit a leaner base than full_base.mk (which pulls in the
# phone apps) and to gate car.mk's test-app block. That is a deliberate change
# affecting what ships, so it is left as a follow-up rather than done blind.
#

DEVICE_PATH := device/brcm/rpi5

# Inherit device configuration
$(call inherit-product, device/brcm/rpi5/device.mk)

PRODUCT_AAPT_CONFIG := normal mdpi hdpi
PRODUCT_AAPT_PREF_CONFIG := hdpi
PRODUCT_CHARACTERISTICS := automotive,nosdcard

$(call inherit-product, $(SRC_TARGET_DIR)/product/full_base.mk)
$(call inherit-product, packages/services/Car/car_product/build/car.mk)

# android.car
PRODUCT_PACKAGES += \
    liblargeparcelablejni

# Audio
PRODUCT_PACKAGES += \
    android.hardware.automotive.audiocontrol-service.example

PRODUCT_COPY_FILES += \
    $(DEVICE_PATH)/car/car_audio_configuration.xml:$(TARGET_COPY_OUT_VENDOR)/etc/car_audio_configuration.xml

# Bluetooth - keep for phone/music
PRODUCT_VENDOR_PROPERTIES += \
    bluetooth.device.class_of_device=38,4,8 \
    bluetooth.profile.asha.central.enabled=false \
    bluetooth.profile.bap.broadcast.assist.enabled=false \
    bluetooth.profile.bap.unicast.client.enabled=false \
    bluetooth.profile.bas.client.enabled=false \
    bluetooth.profile.ccp.server.enabled=false \
    bluetooth.profile.csip.set_coordinator.enabled=false \
    bluetooth.profile.hap.client.enabled=false \
    bluetooth.profile.hfp.ag.enabled=false \
    bluetooth.profile.hid.device.enabled=false \
    bluetooth.profile.hid.host.enabled=false \
    bluetooth.profile.map.server.enabled=false \
    bluetooth.profile.mcp.server.enabled=false \
    bluetooth.profile.opp.enabled=false \
    bluetooth.profile.pbap.server.enabled=false \
    bluetooth.profile.sap.server.enabled=false \
    bluetooth.profile.vcp.controller.enabled=false

# NO broadcast radio - no tuner hardware
# PRODUCT_PACKAGES += android.hardware.broadcastradio-service.default

# Camera - disabled
ENABLE_CAMERA_SERVICE := false

# CAN bus support
PRODUCT_PACKAGES += \
    android.hardware.automotive.can-service \
    canhalconfigurator-aidl \
    canhalctrl \
    canhaldump \
    canhalsend

PRODUCT_COPY_FILES += \
    $(DEVICE_PATH)/can/canbus_config.pb:$(TARGET_COPY_OUT_SYSTEM)/etc/canbus_config.pb \
    $(DEVICE_PATH)/can/canhalconfigurator.rc:$(TARGET_COPY_OUT_SYSTEM)/etc/init/canhalconfigurator-rpi.rc

# Display
PRODUCT_COPY_FILES += \
    $(DEVICE_PATH)/car/display_settings.xml:$(TARGET_COPY_OUT_VENDOR)/etc/display_settings.xml

# NO EVS - already removed above

# Keylayout
PRODUCT_COPY_FILES += \
    $(DEVICE_PATH)/keylayout/Generic.kl:$(TARGET_COPY_OUT_VENDOR)/usr/keylayout/Generic.kl

# NO occupant awareness - already removed above

# OsmAnd Navigation
PRODUCT_PACKAGES += \
    OsmAnd

# Pre-granted permissions for bundled apps (no permission dialogs on a bike)
PRODUCT_COPY_FILES += \
    device/brcm/rpi5/permissions/default-permissions-neo.xml:$(TARGET_COPY_OUT_VENDOR)/etc/default-permissions/default-permissions-neo.xml \
    device/brcm/rpi5/permissions/privapp-permissions-neo.xml:$(TARGET_COPY_OUT_SYSTEM)/etc/permissions/privapp-permissions-neo.xml


# Overlays
PRODUCT_PACKAGES += \
    AndroidRpiOverlay \
    BluetoothRpiOverlay \
    CarActivityResolverOverlay \
    CarLauncherRpiOverlay \
    CarServiceRpiOverlay \
    CarSystemUIRpiOverlay \
    PermissionControllerOverlay \
    SettingsProviderRpiOverlay \
    WifiRpiOverlay

# Permissions
PRODUCT_COPY_FILES += \
    frameworks/native/data/etc/android.software.activities_on_secondary_displays.xml:$(TARGET_COPY_OUT_VENDOR)/etc/permissions/android.software.activities_on_secondary_displays.xml \
    frameworks/native/data/etc/car_core_hardware.xml:$(TARGET_COPY_OUT_VENDOR)/etc/permissions/car_core_hardware.xml

# Vehicle - Custom motorcycle HAL that reads from CAN bus
PRODUCT_PACKAGES += \
    android.hardware.automotive.vehicle@V4-motorcycle-service

# Odometer seed: the old dashboard's last persisted reading was 115.413 km
# (rpi android dash misc/Settings.json, "Mileage"). Used only until the HAL
# first persists its own value; update via
#   adb shell setprop persist.vendor.motodash.odometer <metres>
# if the bike has been ridden since that snapshot.
PRODUCT_VENDOR_PROPERTIES += \
    persist.vendor.motodash.odometer=115413

# GPIO pin configuration for turn signals and high beam
PRODUCT_VENDOR_PROPERTIES += \
    persist.vendor.motodash.gpio.left_turn=16 \
    persist.vendor.motodash.gpio.right_turn=20 \
    persist.vendor.motodash.gpio.high_beam=21 \
    persist.vendor.motodash.gpio.active_low=true \
    persist.sys.enable_freeform_support=1 \
    persist.sys.force_resizable_activities=1

# Device identifier
PRODUCT_DEVICE := rpi5
PRODUCT_NAME := aosp_rpi5_motorcycle
PRODUCT_BRAND := Raspberry
PRODUCT_MODEL := Pi 5 Motorcycle
PRODUCT_MANUFACTURER := Raspberry

