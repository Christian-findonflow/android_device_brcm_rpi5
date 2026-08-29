# NEO dashboard - follow-ups

Living list. Updated 2026-08-29 after the simulator/test-harness session.

## Blocked on Christian (things only you can do)

1. **Ride capture from the bike** - the single most valuable input:
   `candump -l can1` during a short ride (bike on, some throttle, both brakes,
   side stand up/down). This settles, permanently:
   - the real 0x6B1 BMS broadcast layout (two competing inferences today:
     the old dashboard's CanSettings vs one observed frame; the HAL currently
     uses the observed decode - SoC byte3, temp byte7-40)
   - whether gear needs the spec-noted "+1" display offset
   - whether 0x1026105A data5 speed is really a single byte
   - replay fixtures for the harness (`canplayer` / `moto_can_replay -f`)
2. **Current odometer reading** from the old dashboard if the bike has been
   ridden since the Settings.json snapshot (115.413 km). Then:
   `adb shell setprop persist.vendor.motodash.odometer <metres>`
3. **Bar-controls decision (roadmap item 3)** - how the rider should drive the
   UI without touching the screen. Note: the optoisolator has a spare 4th
   input channel; more buttons would need a second board or a resistor-ladder
   ADC approach.
4. **Git pushes** - pushes must run from your shell. `device/brcm/rpi5` is
   typically the one with new commits after a session.
5. *(Optional)* An **Orion BMS profile export** from the Orion utility, if you
   have the cable/software - documents the broadcast layout independently of
   the ride capture.

## Ready to build (not blocked)

### Correctness / hardening
- **HAL shutdown latency**: reader/poll threads use uninterruptible sleeps
  (visible as ~25 s test-suite teardown; worst case slow HAL stop on the
  bike). Replace with condition-variable waits.
- **BMS OBD2 poll cadence**: 5 s x 17 PIDs = ~85 s per value. Do one fast
  initial pass (~100 ms spacing), then 5 s steady state. Testable with a small
  scripted OBD2 responder on vcan.
- **EV_BATTERY_INSTANTANEOUS_CHARGE_RATE units/sign**: HAL publishes Watts,
  positive = discharge; AOSP defines milliwatts, positive = charging. Fix HAL
  + the cockpit power card together.
- **Hot-path INFO logging**: frame dumps every ~10th frame in the HAL and
  per-event logs in the UIs flood logcat on every ride. Demote/gate.
- **GPIO chip selection**: HAL probes gpiochip0 then gpiochip4 and takes
  whichever opens; on Pi 5 both exist. Make the chip path a
  persist.vendor.motodash property alongside the pins.

### Product hygiene
- **Junk packages still ship** (KitchenSink, AdasLocationTestApp, Calendar,
  Camera2, PrintSpooler...). PRODUCT_PACKAGES_REMOVE is a LineageOS-ism and
  post-inheritance filter-out was measured to remove nothing. Real fix:
  inherit a leaner base than full_base.mk and gate car.mk's test-app block.
  See the note in aosp_rpi5_motorcycle.mk.
- **Shared-file leaks**: TARGET_SCREEN_DENSITY 120 in shared BoardConfig.mk
  and the EVS blanking in CarServiceRpiOverlay affect the plain rpi5 tablet /
  car / TV products. Scope both to the motorcycle product.
- **MotoDash legacy app**: SettingsActivity exported without a permission
  guard (dormant - app not shipped). Revert or guard before ever re-enabling;
  or retire the repo.

### Features
- **Side-stand indicator** on the cluster (VENDOR_STATUS_FLAGS bit 3 already
  published and tested; UI only).
- **Real range model (item 4)**: live pack Ah x nominal V derated by SoH from
  the OBD2 data, learned Wh/km from the odometer we now have, instead of the
  hardcoded 5292 Wh / 50 Wh/km in HomeCockpitActivity. Better after the ride
  capture but the plumbing can start now.
- **GPS speed cross-check (item 5)**: compare PERF_VEHICLE_SPEED against GNSS
  speed; flag drift (wrong sprocket config, wheel slip). GNSS HAL already in
  the build.
- **Fault detail screen**: banner tap -> full-screen fault list with
  plain-language explanations and "what to do".
- **Boot time**: measure key-on -> dashboard-visible on the sim, then trim
  (bootanimation length, unused services). Matters for a vehicle.

### Simulator / harness
- **TESTING.md correction**: the "push HAL binary via adb" fast loop does not
  work - /vendor is erofs (read-only); HAL changes need an image rebuild +
  relaunch (sim) or vendor flash (Pi). Apps still hot-deploy via adb install.
- **Scripted ride generator**: synthesize a candump log (accel runs, gear
  changes, SoC drain, a fault event) for demos and regression replays.
- **OBD2 BMS responder** for vcan, so BatteryDetailActivity and poll cadence
  are testable end to end.
- *(Optional)* Cuttlefish kernel with CONFIG_GPIO_SIM=y for true GPIO-path
  emulation (current debug source covers property->UI only).

## Known decode uncertainties (resolved by the ride capture)
| Item | Current implementation | Open question |
|---|---|---|
| 0x6B1 SoC | byte 3, direct % | CanSettings said byte 2 |
| 0x6B1 temp | byte 7 minus 40 | CanSettings said byte 1, no offset |
| Gear display | P/R/N/D direct | spec note: display "needs +1" |
| Controller current | signed int16 x0.1 A | old code assumed 320 A offset |
