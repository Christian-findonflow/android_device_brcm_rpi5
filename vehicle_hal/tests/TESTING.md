# Testing the motorcycle Vehicle HAL without flashing

Four workflows, fastest first. The first two run entirely on the dev machine.

## 1. Host unit tests (seconds, no hardware)

The decode/config logic builds for the host and is exercised directly:

```bash
source build/envsetup.sh
lunch aosp_rpi5_motorcycle-bp4a-userdebug
atest --host motorcycle_vhal_test
# or: m motorcycle_vhal_test && \
#     out/host/linux-x86/nativetest64/motorcycle_vhal_test/motorcycle_vhal_test
```

Covers: controller status/temps frames, signed regen current, BMS 0x6B1,
OBD2 Mode 0x22 scaling, live config changes (wheel/ratio/CAN IDs), config
validation, gear change-detection. Add a test here whenever a decode
question comes up — captured frame in, expected values out.

## 2. Host replay against virtual CAN (full decode stack, real bus traffic)

Runs the actual `MotorcycleVehicleHardware` class on the dev machine, bound
to a virtual CAN interface. One-time setup:

```bash
sudo apt install can-utils
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

Then:

```bash
m motorcycle_vhal_replay
out/host/linux-x86/bin/motorcycle_vhal_replay vcan0
```

In another terminal, send frames by hand (`cansend` examples are printed at
startup) or replay a real capture from the bike:

```bash
# On the Pi (bike powered): record a ride
candump -l can1                     # writes candump-<date>.log

# On the dev machine: replay it into the HAL at original timing
canplayer -I candump-XXXX.log vcan0=can1
```

Every property update the HAL would deliver to Android is printed as it
happens. This is the place to verify decode changes against real traffic
before anything touches the bike.

## 3. Fast deploy to a bench Pi (no image flash)

With the Pi on the bench reachable over adb (USB or `adb connect <ip>`):

```bash
# one-time on a userdebug build
adb root && adb disable-verity && adb reboot
adb wait-for-device root && adb remount

# HAL: push and restart just the service
m android.hardware.automotive.vehicle@V4-motorcycle-service
adb push out/target/product/rpi5/vendor/bin/hw/android.hardware.automotive.vehicle@V4-motorcycle-service /vendor/bin/hw/
adb shell stop vendor.vehicle-hal-motorcycle
adb shell start vendor.vehicle-hal-motorcycle

# Launcher: install like a normal app update
m CarLauncher
adb install -r out/target/product/rpi5/system/priv-app/CarLauncher/CarLauncher.apk
```

Sepolicy or init.rc changes still need a vendor/boot image flash - but code
changes to the HAL binary or the apps don't.

## 4. Full stack on the Pi without the bike

The kernel already enables CONFIG_CAN_VCAN, so the Pi itself can run the
whole system (HAL -> CarService -> dashboard UI) on fake traffic:

```bash
adb shell ip link add dev vcan0 type vcan
adb shell ip link set up vcan0
adb shell setprop persist.vendor.motodash.can_interface vcan0
adb shell stop vendor.vehicle-hal-motorcycle && adb shell start vendor.vehicle-hal-motorcycle
```

Then stream a capture from the dev machine through adb:

```bash
awk '{print $3}' candump-XXXX.log | while read f; do
  adb shell cansend vcan0 "$f" 2>/dev/null || true; sleep 0.05
done
```

(If `cansend` isn't on the device, workflow 2 on the host covers the same
ground; or push a static-linked cansend.) Remember to set the property back
to `can1` afterwards.
