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

## 3. Fast deploy to a bench Pi (apps only)

With the Pi on the bench reachable over adb (USB or `adb connect <ip>`):

```bash
# Launcher/SystemUI: install like a normal app update
m CarLauncher
adb install -r out/target/product/rpi5/system/priv-app/CarLauncher/CarLauncher.apk
```

CORRECTION (measured 2026-08-29): the HAL binary can NOT be adb-pushed.
/vendor is erofs, mounted read-only - `adb push` to it silently no-ops even
after `adb remount`, and running a copy from /data races the init-managed
service (two HALs, CarService bound to the old one). HAL, sepolicy and
init.rc changes need an image rebuild plus relaunch (simulator) or a vendor
image flash (Pi). Apps still hot-deploy.

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

## Cuttlefish gotcha: do not `adb reboot` the simulator

Rebooting the guest inside a running cvd leaves the host stack (run_cvd,
crosvm, webRTC) in a state that `cvd_internal_stop` no longer tears down, and
the next `cvd_internal_start` fails with "Instance directory files in use".
Recover with:

```bash
pkill -f 'bin/run_cvd'; sleep 2
pkill -f 'bin/x86_64-linux-gnu/crosvm'; pkill -f 'bin/webRTC'
pkill -f 'operator_proxy|openwrt_control_server|kernel_log_monitor|process_restarter|log_tee'
```

For boot-timing work, add `--extra_kernel_cmdline="log_buf_len=4M"` to the
start command so the early kernel/init log survives to be read.

## Ride capture (the fixture that settles the decode questions)

The HAL can log every CAN frame it sees or sends, in `candump -l` format, to
`/data/vendor/motodash/can-<epoch>.log`. Off by default.

Enable: Dash Settings -> Diagnostics -> "Capture CAN traffic" (applies
immediately, persists across reboots), or from adb:
`adb shell setprop persist.vendor.motodash.cfg.can_capture 1` then restart
the HAL (`adb shell stop vendor.vehicle-hal-motorcycle; start ...`).
The file is synced every second, so a key-off power cut loses at most the
last second of a ride. Capture stops itself at 256 MB.

Retrieve (bike on home Wi-Fi, userdebug build):

    adb connect <pi-ip>:5555
    adb root
    adb pull /data/vendor/motodash/ ride-capture/
    adb shell dumpsys android.hardware.automotive.vehicle.IVehicle/default > ride-capture/vhal_dump.txt

Drop the folder into `rpi android dash misc/`. Replay it anywhere:

    moto_can_replay vcan0 -f ride-capture/can-<epoch>.log        # device or host
    motorcycle_vhal_replay vcan0                                   # host, real decode

What the first capture answers: the 0x6B1 broadcast layout (SoC/temp bytes),
whether the controller's current field carries a sign, the gear nibble
encoding above D, and the OBD2 byte order (pack Ah near 68 confirms
big-endian). The OBD2 requests (0x7E0) and responses (0x7E8) are in the log
too, as are our own 0x1026105A display reports.

## Simulated ride (no bike needed)

`make_ride_log.py` synthesizes a full ride (accelerate, 56 km/h cruise, regen
deceleration, 20 km/h stretch, stop) as controller 0x10261022 frames at 10 Hz
plus BMS 0x6B1 at 1 Hz:

    python3 vehicle_hal/tests/make_ride_log.py > ride.log
    adb push ride.log /data/local/tmp/
    adb shell moto_can_replay vcan0 -f /data/local/tmp/ride.log   # adb root first

Verified on the simulator (2026-08-30): live speed + gear D on the cluster,
V/A system-bar chips, SoC from 0x6B1, range EMA re-learning during the ride
(persist.vendor.motodash.whperkm moves, range line grows as the average
settles), odometer + trip advancing by exactly the integrated distance, and
the full trust cascade blanking every surface a few seconds after the last
frame. Note the SoC step (60 -> 59) exercises the arrival-SoC projection too.

The driving-state layer is live as of 2026-08-30: the HAL derives
PARKING_BRAKE_ON from the gear (P = on, boot default on), so the replay
moves the platform through IDLING -> MOVING -> (a single gear-P frame)
PARKED - watch with `dumpsys car_service | grep 'Current Driving State'`.
UX restrictions stay explicitly permissive while moving (motorcycle
car_ux_restrictions_map.xml in car-services): nothing locks at speed by
design - see FOLLOW-UPS "Riding lockout policy" for the reasoning.
