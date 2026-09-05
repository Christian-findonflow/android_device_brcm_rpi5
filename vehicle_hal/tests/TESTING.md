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

**Raw IMU capture (2026-09-04).** The same Workshop switch also writes
`/data/vendor/motodash/imu-<epoch>.log` while the lean sensor is present:
every 100 Hz sample in sensor axes, the CAN speed the estimator was given,
the live roll/pitch/status it produced, and the barometer at 2 Hz (format in
`imu/ImuLog.h`). Pull it with the CAN log and replay on the host:

    motorcycle_imu_replay imu-1757000000.log --up 0.2,-0.3,0.93 --fwd 0.9,0.4,0 \
        --roll-tau 1.5 --band 0.15 --csv ride.csv

`--up/--fwd` are the persisted `persist.vendor.motodash.imu.{up,fwd}` values
(`adb shell getprop`); `--level` instead takes the first second of the log as
the Level capture. The tool prints the sample rate, how long the estimate was
valid, max lean replayed vs what the bike showed live, the rms difference
between the two, the learned gyro bias and the pressure range; the CSV has
raw axes plus live and replayed roll per sample for plotting. Change a
constant on the command line, rerun, compare - that is how the filter gets
tuned on a real ride instead of on synthetic corners.
## Simulated ride (no bike needed)

`e2e_can_test.sh` now also asserts the **ride summary** (VENDOR_RIDE_SEQ /
RIDE_DISTANCE_M) published by the HAL when the controller link dies after the
ride - 27 checks in total. Launcher pure-logic tests (CarLauncherTests,
MotorcycleLogicTest) cover the Open-Meteo weather parse/chip text and the
maintenance-schedule due math alongside units, PIN and solar math (14 tests).
Run them filtered - `atest CarLauncherTests:com.android.car.carlauncher.MotorcycleLogicTest`
- because the rest of the CarLauncherTests module tests the stock AAOS home
screen we replaced, and fails by design. Afterwards
`adb uninstall com.android.car.carlauncher.test`: the test APK registers a
second HOME handler and the HOME key shows a chooser while it is installed.

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

## End-to-end CAN behavior test (one command)

`tests/e2e_can_test.sh [serial]` replays the synthetic ride into a running
emulator (or bench Pi over adb) and asserts the WHOLE stack's behavior:
speed and gear decode, the gear-linked parking brake, driving-state
IDLING -> MOVING -> PARKED, SoC, controller+BMS link bits, range learning,
odometer/trip accumulation, the blanking cascade when the bus goes silent,
and that no tombstones appeared. ~3 minutes, prints PASS/FAIL per check,
exit code 0 only if everything held. Run it before flashing an image to
the bike.

## Launcher logic tests

`atest CarLauncherTests:com.android.car.carlauncher.MotorcycleLogicTest`
(device test - the emulator counts) covers the rider-facing pure logic that
no other suite touched: unit formatting (degC/degF, km/mi), the Workshop PIN
(salted-hash lifecycle + format rules), the shared SoC/temperature/RPM color
thresholds, and the NOAA solar-elevation used for night dimming (checked
against London solstice noon/midnight and equator noon).

## First-ride gear diagnosis

Workshop settings now shows the controller's raw gear/status byte LIVE and
has a "Gear base +1" switch (VENDOR_CFG_GEAR_BASE, persisted, applies
instantly). On the bike: shift through P/R/N/D, hold Sport, flip drive
modes 1/2/3, and read exactly which nibble/bits change - then set the base
to match. No laptop, no rebuild. Sport/drive-mode MAPPING (what the cluster
should display for them) still needs the ride capture to design properly.

## Lean sensor (IMU): tests before the hardware, bring-up after

Software landed 2026-09-04, ahead of the parts (Adafruit 4502 ISM330DHCX +
2651 BMP280 on the CAN HAT's Grove I2C port, ADA4528 Grove-to-QT cable, CAB1015
QT-QT cable). Design notes in `vehicle_hal/imu/` headers and FOLLOW-UPS.md.

**Why the HAL talks I2C itself.** Our 6.12 kernel compiles the lsm6dsx driver
in but has no IIO trigger, and the Qwiic cable carries no interrupt line: the
driver's only path is one-shot sysfs reads, which power the sensor up and down
around every axis (tens of ms each, ~10 Hz for six axes). So the HAL opens
`/dev/i2c-1` (I2C_RDWR) and drives both chips from `imu/Sensors.cpp` at a clean
100 Hz burst read. Nothing in the kernel needs configuring beyond
`dtparam=i2c_arm=on` (boot/config.txt), the ueventd ownership line and the
`i2c_device` sepolicy type.

**Tests (all run on the host, no hardware):**

    atest --host motorcycle_vhal_test      # 41 HAL tests + 4 IMU-through-HAL + 18 imu/ tests

`tests/ImuTest.cpp` drives the estimator with the synthetic IMU
(`imu/SyntheticImu`): upright straight, an established 30 deg corner (where
the accelerometer alone reads ~0 - the test asserts that first, then that the
vehicle-aware filter reads 30), rolling into a corner through the gyro, CAN
link loss mid-corner (lean must hold), an awkward mounting fixed by Level +
learned forward axis (from acceleration and from braking), gyro bias learned
at standstill and held on a long straight, hard braking not tilting roll,
side-stand lean at standstill, sample gaps, Level rejection while moving; the
ISM330DHCX and BMP280 drivers against a fake I2C bus (WHO_AM_I, register
writes, burst decode, BMP280 compensation against the datasheet example,
altitude round trip); and the scenario-file source. The HAL-level tests cover
publishing, ride max-lean (speed-gated so the side stand does not count),
the Level command with persistence, and Level rejection.

**Simulator / e2e.** The HAL reads a scenario file when it exists:

    adb shell "echo 'lean=30 alt=120' > /data/vendor/motodash/imu_sim"   # adb root

Keys: `lean` (deg, + right), `pitch`, `rollrate` (deg/s), `along` (m/s^2),
`alt` (m, drives the fake barometer), optional `speed` (m/s; default = the
bike's live CAN speed, so a corner is always physically consistent with the
speed the HAL believes). The HAL probes for a source every 5 s, so the file
takes effect within seconds; delete it and the "sensor" unplugs. The cluster
shows the lean arc once IMU_STATUS reports PRESENT; Workshop > Lean sensor
shows status, raw axes, live lean/pitch and baro. `e2e_can_test.sh` now runs
37 checks: it adds the simulated sensor, a 30 deg right and 20 deg left corner
at cruise (lean, lateral g, status bits), ride max lean L/R in the summary,
Level/clear through `cmd car_service set-property-value 0x21400068 0 1|0`, and
source loss.

**Bring-up checklist when the parts arrive (10 minutes):**

1. Wire: HAT Grove I2C port -> ADA4528 -> ISM330DHCX -> CAB1015 -> BMP280.
   Mount the IMU rigidly inside the dash enclosure, any orientation; the
   BMP280 anywhere on the chain (keep it out of direct airflow/sun).
2. Flash this image (I2C enabled in config.txt) and boot with the bike off.
3. `adb shell ls -la /dev/i2c-1` must show `crw-rw---- vehicle_network system`.
   `adb logcat -s android.hardware.automotive.vehicle@V4-motorcycle-service`
   must show `IMU: ISM330DHCX family (WHO_AM_I 0x6B) on /dev/i2c-1, barometer
   present`. If it says nothing is on the bus: check the cable seating, then
   `dmesg | grep -i i2c` (bus present?), then `dmesg | grep avc` (sepolicy).
   Address jumpers: `setprop persist.vendor.motodash.imu.addr 0x6b` /
   `persist.vendor.motodash.imu.baro_addr 0x76`, then restart the HAL.
4. Workshop > Lean sensor: the raw row must read ~1.00 g on one accel axis
   and ~0 deg/s on the gyros. Tilt the module by hand and watch the axes.
5. Bike upright on flat ground, held still: press **Level**. Status must say
   `level OK · forward: learning`. (Rejected = the bike or the sensor moved.)
6. Ride: the forward axis learns itself on the first straight pull-away
   (about 12 m/s of accumulated speed change; a few strong accelerations).
   Status flips to `forward OK`, the cluster arc appears and reads lean;
   the Workshop row shows lean/pitch live. On the side stand it should read
   roughly the stand angle; the ride summary ignores standstill lean.
7. Sanity in the first corners: 20-30 deg on a normal bend, marker moving
   the way the bike leans. If it reads mirrored, the learned forward axis is
   backwards: Clear calibration, Level again and pull away hard in a straight
   line (braking also teaches it, with the sign handled).

## First live capture on the bike (2026-09-05) - what the bus really says

Recorded with the Workshop capture switch, bike on, stationary, then a guided
gear/mode/throttle sequence. Facts now built into the HAL:

- Bus: 250 kbit/s on kernel `can1` (the HAT channel on SPI chip-select 0).
  IDs seen: controller 0x10261022 (status, 3.3 Hz), 0x10261023 (temps, 3.3 Hz),
  **0x10261051 (10 Hz, unknown; byte 5 pulses AA->B4 for ~1 s per button
  press)**, **0x1026105A (3.3 Hz, unknown; byte 6 is a rolling counter)**,
  BMS 0x6B1 (21 Hz). Nothing else.
- **0x6B1 is Orion's default second broadcast**, not SoC: DCL (u16 BE, A), CCL
  (u16 BE, A), high cell temp, low cell temp (int8 C), unused, checksum =
  (sum of bytes 0-6 + 8 + 0x6B1) & 0xFF. The earlier "byte 3 = SoC" guess was
  the charge current limit (27 A). SoC/current/voltage are NOT broadcast.
- **The BMS answers OBD2 on 0x7E3 -> 0x7EB** (Orion's second pair), never on
  0x7E0. PID 0xF00F gave 96.5 %, 0xF00D 85.6 V with the 21s pack at 4.11 V/cell.
- Controller status frame: byte 1 gear/status is a bitfield in the high
  nibble (values 00, 10, 30, 70, B0, F0 seen; low nibble 2 = brake), current
  (bytes 6-7) is signed, + = discharge, and reads **-1.7 A at rest** (sensor
  offset; the BMS current via 0xF00C is the trustworthy one). Temps frame
  byte 4 = throttle % (0..43 during a blip).
- Indicators: right and high beam confirmed on the opto inputs (the right one
  toggles at the flasher rate); left not yet seen.
- Gotchas met on the way: a `ip link set can1 down/up` kills the HAL's reader
  thread (fixed to reconnect); with nothing ACKing, the OBD2 polls drove can1
  to bus-off - `restart-ms 100` now in init.rpi5.rc.

Tools (this directory): `bike_live.sh [ip]` = one-shot snapshot over adb (CAN
link state and frame rate, decoded values, raw indicator levels, HAL log);
`can_timeline.py <capture> [epoch]` prints every byte change per ID from a
capture so a rider sequence maps onto bytes; `moto_can_replay` pushed to
/data/local/tmp on the bike sends probe frames (`... can1 7E3#0322F00F00000000`).
