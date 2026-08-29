#
# Copyright (C) 2024 MotoDash Project
# SPDX-License-Identifier: Apache-2.0
#
# Cuttlefish-based motorcycle dashboard simulator.
# Runs the full NEO dashboard stack (motorcycle Vehicle HAL, CarLauncher
# cockpit, CarSystemUI bars) locally with a virtual CAN bus - no Pi needed.
#
#   lunch aosp_cf_moto-bp4a-userdebug && m
#   launch_cvd --daemon --x_res=800 --y_res=480 --dpi=120 \
#       -guest_enforce_security=false
#   Browser: https://localhost:8443
#
# The rpi5 sepolicy is not wired into this product; run the guest with
# -guest_enforce_security=false (simulator only - never on the bike image).
#
# Feed traffic:  adb shell moto_can_replay vcan0 10261022#0030B80BD002FF00
#            or  adb push ride.log /data/local/tmp && \
#                adb shell moto_can_replay vcan0 -f /data/local/tmp/ride.log

# Use the motorcycle Vehicle HAL instead of the default Cuttlefish VHAL.
# Must be set before inheriting device_vendor.mk (see LOCAL_VHAL_PRODUCT_PACKAGE
# in device/google/cuttlefish/shared/auto/device_vendor.mk).
LOCAL_VHAL_PRODUCT_PACKAGE := android.hardware.automotive.vehicle@V4-motorcycle-service

$(call inherit-product, device/google/cuttlefish/vsoc_x86_64_only/auto/aosp_cf.mk)

# device/brcm/rpi5 is a soong namespace; without this line every module under
# it (the motorcycle VHAL, the dashboard overlays, moto_can_replay) is
# silently invisible to this product.
PRODUCT_SOONG_NAMESPACES += device/brcm/rpi5

# Local simulator: the rpi5 dashboard overlays install outside the car
# generic-system allowlist, so drop the compliance enforcement here (the
# shipping rpi5 product is unaffected).
PRODUCT_ENFORCE_ARTIFACT_PATH_REQUIREMENTS := false

# Motorcycle dashboard UI configuration - same overlays as the rpi5 build
PRODUCT_PACKAGES += \
    AndroidRpiOverlay \
    CarServiceRpiOverlay \
    CarSystemUIRpiOverlay \
    CarLauncherRpiOverlay \
    SettingsProviderRpiOverlay

# Virtual CAN bus: bring up vcan0 at boot; the HAL binds it via the property
PRODUCT_COPY_FILES += \
    device/brcm/rpi5/cuttlefish/init.moto_cf.rc:$(TARGET_COPY_OUT_VENDOR)/etc/init/init.moto_cf.rc

PRODUCT_VENDOR_PROPERTIES += \
    persist.vendor.motodash.can_interface=vcan0

# On-device CAN frame sender/replayer for test traffic, and the simulator
# vcan/service bring-up script
PRODUCT_PACKAGES += \
    moto_can_replay \
    moto_cf_setup.sh

# Navigation app for the maps tile / maps button
PRODUCT_PACKAGES += OsmAnd

# Pre-granted permissions for bundled apps (no permission dialogs on a bike)
PRODUCT_COPY_FILES += \
    device/brcm/rpi5/permissions/default-permissions-neo.xml:$(TARGET_COPY_OUT_VENDOR)/etc/default-permissions/default-permissions-neo.xml


PRODUCT_NAME := aosp_cf_moto
PRODUCT_MODEL := Cuttlefish Motorcycle Dashboard

