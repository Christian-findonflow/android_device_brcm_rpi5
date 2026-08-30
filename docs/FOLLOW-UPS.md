# NEO dashboard - follow-ups

Living list. Updated 2026-08-29 (evening) - the hardening batch (shutdown
latency, BMS cadence + vcan responder, charge-rate units, log gating,
side-stand indicator, TESTING.md correction) is done.

## Recently completed (for the record)

2026-08-29, verified on the simulator/harness unless noted:
- Fault flags property + red/amber pulsing banner UI (critical vs degraded)
- Odometer + trip: accumulation, persistence, trip reset, controller-compliant
  0x1026105A display report (verified on the wire), odometer seed 115.413 km
- Status flags property + SIDE STAND cluster banner
- Turn signal / high beam indicators actually work (two permission mechanisms
  fixed - they had never worked, in launcher AND SystemUI)
- Subscribe seeding: UIs read current values at registration, so restarts
  cannot hide an active fault/indicator
- HAL shutdown made interruptible (test suite 25s -> 1ms)
- BMS polling: all values within ~4s of boot, full refresh ~17s (was 85s
  per value); moto_bms_sim vcan responder for end-to-end testing
- Charge rate to AOSP units/sign; REGEN state on the power card
- Hot-path logging gated behind persist.vendor.motodash.debug.canlog
- Visual refresh: shared palette + MotorcycleTheme (one set of thresholds),
  gradient cards, cluster RPM/battery bars
- Cuttlefish simulator product, host test suite (19 tests), CAN replay tools

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
- **GPIO chip selection**: HAL probes gpiochip0 then gpiochip4 and takes
  whichever opens; on Pi 5 both exist. Make the chip path a
  persist.vendor.motodash property alongside the pins.
- **CarSystemUIRpiOverlay flags.xml is a silent no-op**: the scalable_ui
  booleans it sets to false are only the *fallback* path of CarSystemUI's
  FlagManager - the compile-time aconfig flag
  (com.android.systemui.car.Flags.scalableUi) resolves first and is true, so
  ScalableUI is enabled on both products despite the overlay (confirmed
  2026-08-30: PanelAutoTaskStackTransitionHandlerDelegate handles every
  transition on the simulator). The overlay comment claims it prevents
  TaskView-embedding interference; whatever problem that was, this file is
  not what's holding it off. Either delete the file or, if the interference
  is real, disable the aconfig flag in the product instead. Note: the
  transition-confinement crop (2026-08-30) lives in that delegate's
  startAnimation, so actually disabling ScalableUI would also disable the
  crop and bring the cross-cluster slide back.

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
- **Range model refinements** (base model DONE 2026-08-30: HAL learns a Wh/km
  EMA over 200m chunks, publishes RANGE_REMAINING, pack energy configurable
  via VENDOR_CFG_PACK_ENERGY_WH / settings UI; cluster + cockpit + arrival
  projection consume it). Remaining ideas: derate pack energy by SoH from the
  OBD2 data; seed Wh/km per riding profile; validate the EMA constants
  against the ride capture.
- **GPS speed cross-check (item 5)**: compare PERF_VEHICLE_SPEED against GNSS
  speed; flag drift (wrong sprocket config, wheel slip). GNSS HAL already in
  the build.
- **Night dimming (DONE 2026-08-30, unverified on hardware)**: launcher-side
  NightBrightnessController computes solar elevation from the last GPS fix
  and writes Settings.System screen brightness on band changes; CarService
  forwards to VHAL DISPLAY_BRIGHTNESS -> sysfs backlight. Verify the full
  chain on the Pi (the simulator has no backlight); night level tunable via
  persist.vendor.motodash.night_brightness.
- **Fault detail screen**: banner tap -> full-screen fault list with
  plain-language explanations and "what to do".
- **Boot time**: measured on the simulator 2026-08-29. Warm boot (the key-on
  case) is 9.3 s to screen; cold boot 19 s, and the difference is almost all
  first-boot-only work (userdata format, mount_all --late 3.2 s, apex
  activation ~5 s). Framework is ~6.5 s with 194 packages installed - the
  leaner-base cleanup has a measurable KPI there. VHAL is up at 1.4 s.
  Verdict: acceptable for now; revisit on Pi/NVMe hardware where the numbers
  will differ. Deeper cuts mean kernel/init surgery - measure on hardware
  before spending that effort.

### Simulator / harness
- **Scripted ride generator**: synthesize a candump log (accel runs, gear
  changes, SoC drain, a fault event) for demos and regression replays.
- *(Optional)* Cuttlefish kernel with CONFIG_GPIO_SIM=y for true GPIO-path
  emulation (current debug source covers property->UI only).

### Ghost cluster (priority - rider-visible)
An empty translucent HOME root task in the default (right) TaskDisplayArea
renders a frozen duplicate of the cluster whenever the right area has no app
on top. Two dead ends, both documented in commits: SystemUI's explicit
user-0 HOME launch (removed - it only ever created junk) and
canHostHomeTask=false on the default TDA (REVERTED - system_server NPEs at
RootWindowContainer:1953 on user switch; the framework requires a home root
there). The correct fix: keep the right area occupied - auto-restore
HomeCockpitActivity (as the CURRENT user, with launch-TDA options) whenever
the default TDA's top becomes empty/home. This also fixes the UX gap where
HOME leaves the right panel grey. Implement in MotoSplitDisplayAreaController
via a task stack listener, or extend HomeCockpitLauncherService.

### UX (from the 2026-08-29 evening review)
- **Cluster turn-by-turn: IMPLEMENTED** (CarLauncher ad248b48) - OsmAnd V1
  AIDL client + cluster widget, verified on the simulator up to the last
  joint. Remaining: confirm OsmAnd fires updateNavigationInfo on real
  position updates - needs live GPS (bench Pi or bike), or cuttlefish GPS
  injection. Test hooks: vendor.motodash.debug.navroute="lat,lon,lat,lon"
  and ...navdemo="turnType,distanceMeters".
- **OsmAnd route card overflows the 800x480 panel**: the route-planning
  bottom sheet renders mostly below the fold, leaving destination-setting
  awkward on the bike. Investigate OsmAnd display settings, or drive
  routing through our own UI via the AIDL navigate()/search calls.
- **Speed limit on cluster**: not exposed by the V1 AIDL. The V2 package
  (net.osmand.aidlapi) is now vendored and bound (2026-08-30, for
  getAppInfo route polling), so check its surface for speed limit next -
  the plumbing cost is already paid.
- **Ignition-sense power story**: orderly shutdown (or suspend, if the Pi 5
  can) driven by an ignition input on the spare 4th optoisolator channel.
  Hardware decision needed first - see "Blocked on Christian".
- **Night brightness**: no ambient sensor, but GPS gives sunrise/sunset for
  scheduled dimming. Panel facts from "FNK0078 FAQs.pdf" (the FAQ covers several
  panels; ours is the Freenove 7" DSI, 800x480): it exposes the standard
  /sys/class/backlight/rpi_backlight/brightness (0-255) and the rpi5 lights
  HAL already probes exactly that path, so Android's brightness pipeline
  should just work - the feature is pure Android-side scheduling. The panel
  also has a physical side button (+10% steps, long-press = backlight off),
  a useful rider fallback that adjusts brightness behind Android's back.
- **Screen mounting orientation**: if the case forces the panel upside down,
  the FAQ's Raspbian rotation methods (display_lcd_rotate, X11 touch
  matrices) do NOT apply to Android - use
  ro.surface_flinger.primary_display_orientation plus a touch orientation
  remap instead. Decide when the case design is final.
- **Charging view refinements (DONE 2026-08-30)**: takeover now yields to a
  dead controller link and its SoC blanks without BMS; bottom-bar V/A, temp
  and battery chips link-gate to "--". Cosmetic remainder: the battery and
  charging BAR FILLS keep their last width on a dead link (the text goes
  "--"); blank or dim the fills too.
- **Ride summary** on key-off / next boot (distance, time, average Wh/km) -
  data now exists; also feeds the learned range model.
- **OsmAnd first-run**: pre-seed config or ship a region map (~1 GB image
  cost - product decision).

## Known decode uncertainties (resolved by the ride capture)
| Item | Current implementation | Open question |
|---|---|---|
| 0x6B1 SoC | byte 3, direct % | CanSettings said byte 2 |
| 0x6B1 temp | byte 7 minus 40 | CanSettings said byte 1, no offset |
| Gear display | P/R/N/D direct | spec note: display "needs +1" |
| Controller current | signed int16 x0.1 A | old code assumed 320 A offset |
