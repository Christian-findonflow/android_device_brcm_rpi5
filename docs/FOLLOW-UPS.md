# NEO dashboard - follow-ups

Living list. Updated 2026-08-30 - transition confinement, the rider batch
(range model, arrival SoC, power bar, night dimming, trust patches), display
units on every surface, and the app-grid touch fix are done; see the dated
entries below and the git log of the three forks.

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

2026-08-30 (evening):
- Keyboard (CarLatinIME, new fork neo-car-latinime): key geometry moved to
  %p display fractions so the grid fills the 800px panel edge to edge -
  ~68x47px letter keys (was ~39px huddled in the middle third), wider
  return, 45% spacebar, ~200px total height so the cluster speed readout
  stays visible. Verified typing + symbol layout in OsmAnd search on the
  simulator. Runtime updates need
  `adb install -r --bypass-low-target-sdk-block` (app targets SDK 23);
  image builds unaffected.

## Blocked on Christian (things only you can do)

- **Sport + drive modes 1/2/3 (first ride)**: the gear nibble now has a
  Workshop-settable base (+1 covers the spec's P-offset ambiguity) and the
  Workshop screen shows the raw gear/status byte live - shift through
  P/R/N/D, hold Sport (momentary, <=30 s), flip modes 1/2/3 and note which
  nibble values / status bits appear. With that (plus the ride capture) we
  design real S-gear and drive-mode display. Until then unknown nibbles
  render as N.

- **Riding lockout policy - DECIDED & IMPLEMENTED 2026-08-30 (Christian:
  gear-linked)**: the VHAL now derives PARKING_BRAKE_ON from the gear (P = on,
  R/N/D = off, boot default on), which lets CarDrivingStateService initialize,
  and car-services ships a motorcycle car_ux_restrictions_map.xml that is
  explicitly permissive while idling/moving (requiresDO=false, uxr=baseline) -
  the parser promotes any non-baseline uxr to requiresDO=true, and requiresDO
  blocks every non-distraction-optimized activity (OsmAnd, music, the whole
  dashboard), so config-level restrictions are all-or-nothing. If a keyboard
  lock at speed is ever wanted, do it app-side: CarLatinIME can listen to
  CarDrivingStateManager directly (driving state is now real) and flip its
  existing lockout view.

1. **Ride capture from the bike** - the single most valuable input, and
   now one switch away: Dash Settings -> Diagnostics -> "Capture CAN
   traffic", ride, then `adb root; adb pull /data/vendor/motodash/` (see
   vehicle_hal/tests/TESTING.md "Ride capture"). This settles, permanently:
   - the real 0x6B1 BMS broadcast layout (the HAL now prefers the BMS's
     own 0xF00F SoC PID, so a wrong 0x6B1 guess no longer corrupts the
     battery display - but the capture still tells us the truth)
   - whether the controller's current field is signed (0xF00C from the BMS
     now drives charging detection when fresh, for the same reason)
   - whether gear needs the spec-noted "+1" display offset / what values
     above 3 the nibble carries
   - the OBD2 byte order (decode switched to big-endian 2026-08-30 per the
     UDS standard; pack Ah reading ~68 in the vhal dump confirms it)
   - replay fixtures for the harness (`moto_can_replay -f`)
2. **Current odometer reading** from the old dashboard if the bike has been
   ridden since the Settings.json snapshot (115.413 km). Then:
   `adb shell setprop persist.vendor.motodash.odometer <metres>`
3. **Bar-controls decision (roadmap item 3)** - how the rider should drive the
   UI without touching the screen. Note: the optoisolator has a spare 4th
   input channel; more buttons would need a second board or a resistor-ladder
   ADC approach.
4. *(Optional)* An **Orion BMS profile export** from the Orion utility, if you
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
- **CarLatinIME targetSdk bump (23 -> current)**: only bites runtime installs
  (`adb install -r` needs `--bypass-low-target-sdk-block`, and Android's
  install floor keeps rising - the flashed image is unaffected). Bump on the
  next touch of the app: needs `android:exported="true"` on the IME service
  (31+ gate) and likely a `<queries>` entry for com.android.car (30+ package
  visibility - retest the park-to-type lockout). ~30 min incl. the 7 s
  rebuild loop.

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
- **Rider nice-to-haves (decided 2026-09-04) - IMPLEMENTED same day**: weather/rain
  card (Open-Meteo, keyed off the last GPS fix) + Breezy Weather app, "find
  charger" shortcut on the range card (OsmAnd charging-station POI search),
  ride summary at key-off, maintenance log with odometer reminders, AntennaPod
  for podcasts. Shipped as: header weather chip (2 h rain chance + temp,
  cached with age when offline, tap -> Breezy Weather), header charger bolt
  (OsmAnd geo search "charging station"), Service log screen (7 odometer-keyed
  items, Done stamps the odometer) reachable from native Settings and the
  Motorcycle settings screen - Christian vetoed a dashboard pill, so the only
  dashboard trace is a small amber dot on the settings gear when something is
  due (never-recorded items do not count),
  LAST RIDE line in the range card (HAL VENDOR_RIDE_* at key-off / 5 min
  parked, persisted across reboots), AntennaPod + Breezy Weather bundled like
  OsmAnd (packages/apps/<App>/, presigned prebuilts). Also removed Calendar,
  Messaging (SIM SMS), Gallery, Local Media Player; Clock kept (Christian).
  Parked for later, Christian likes them:
  - **TPMS**: cheap BLE valve-cap sensors -> a BLE service feeding a cockpit
    card with per-wheel pressure/temperature and low-pressure warnings.
  - **Dashcam**: needs a Pi camera module; loop recording + event save on
    hard braking (we already see regen/brake current).
  - **Lean angle / ride telemetry**: needs an IMU HAT (MPU-6050 class);
    live lean on the cluster, max lean in the ride summary.
- **Speed limit on the cluster - NOT feasible via OsmAnd AIDL** (checked
  2026-09-04: neither the V1 nor V2 API exposes the current road's speed
  limit; only GPX max speed). Options: an in-house OSM lookup from offline
  data (heavy), or a future OsmAnd API addition. Parked.

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
  chain on the Pi (the simulator has no backlight); night level is the
  Rider setting (Dim/Normal/Bright) in Settings -> Motorcycle.
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

### Voice assistant (discussed 2026-08-30, parked until the platform is stable)
Google's automotive stack (GAS: Maps, Assistant, Play) is OEM-licensed only;
phone GMS can be sideloaded but is uncertified, RAM-hungry and laid out for
phones - not worth it on this panel, and OsmAnd already beats Google Maps
offline. The assistant we want is buildable without Google, and knows the
bike:
- Framework: a `VoiceInteractionService` app as the system assistant
  (restore the bar's Assistant button, or trigger from a bar/headset PTT).
- Audio: Bluetooth helmet headset (Cardo/Sena) for mic + speaker. Prove
  this path first with OsmAnd voice guidance (recorded voices, no TTS
  engine needed) - cheap and useful on its own.
- Trigger: push-to-talk (headset button or the spare optoisolator channel -
  the bar-controls decision), not a hotword; wind noise kills wake words.
- STT: Vosk or Sherpa-ONNX offline. TTS: Piper via Sherpa-ONNX offline.
- Brains, two tiers: local rules for the commands that must work offline
  (navigate/home, call, play, "how far can I go" from RANGE_REMAINING,
  "will I make it" from the arrival projection, "anything wrong" from
  FAULT_FLAGS) wired to OsmAnd AIDL / dialer / media session / HAL props;
  an optional cloud LLM tier when phone tethering is up, never a dependency.
Order: BT headset + OsmAnd voice -> PTT decision -> offline assistant ->
LLM tier (needs the BT-tethering item). Multi-week; after the ride
capture and decode fixes.

### Settings structure (DONE 2026-08-30, verified on the simulator)
Rider settings (units, night brightness, trip reset) are open to anyone and
sized for gloves; Workshop settings (wheel, drivetrain, battery, CAN, GPIO,
diagnostics) sit behind a passcode set in Workshop itself (open until one is
set; salted SHA-256 in launcher prefs - keeps passengers out, not adb).
Rider is reachable from the cockpit gear and injected into native
CarSettings' homepage device group via settingslib's extra-settings
meta-data on RiderSettingsActivity. Both settings activities carry their
own taskAffinity so a source-less launch can't land them in the cluster
TDA (the app-grid lesson).

### In-place app updates: CarService caveat (found 2026-08-30)
`adb install -r` of CarLauncher over the system copy works (the package
resolves to /data/app afterwards) and is the cheap OTA-lite path - but
CarService crashes on the PACKAGE_ADDED broadcast: VendorServiceController
restarts the launcher's HomeCockpitLauncherService from a background
context and hits BackgroundServiceStartNotAllowedException (it recovers
after restart). Before relying on APK updates in the field, either make the
cockpit launcher a bound vendor service (bind= in the CarService overlay's
config_earlyStartupServices) or move its work elsewhere.

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
- **App grid dead to touch (FIXED 2026-08-30)**: taps and swipes on the
  apps screen did nothing - almost certainly since the split display
  existed. Root cause was not the grid: the cluster's home task runs in
  fullscreen windowing mode, so the ActivityRecordInputSink the framework
  attaches to it (touch-opaque activities, compat change
  ENABLE_TOUCH_OPAQUE_ACTIVITIES) had an unbounded region; whenever the
  cluster was z-ordered above a right-panel task (the grid let it take
  focus, as does a HOME press) the sink ate every right-panel touch.
  Diagnosed via `dumpsys input` window order + the
  "Not sending touch gesture to ... ActivityRecordInputSink" log; confirmed
  with `am compat disable`. Fix: CarLauncher (cluster host) calls
  Activity#setActivityRecordInputSinkEnabled(false) (@hide, flag
  allow_disable_activity_record_input_sink is ENABLED in bp4a). Also: grid
  gets its own taskAffinity (no more cluster focus theft), vertical paging,
  4x3 cells, reorder disabled, gentler snap/fling. Verified on the
  simulator by screenshot diff: tap/fling/drag on fresh boot and after HOME.
  Watch item: the grid resumes on its last page (stock behaviour).
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
  "--"); blank or dim the fills too. Same class (simulated ride, 2026-08-30): the
  cockpit TEMPERATURES and PACK HEALTH cards keep stale values on a dead
  link while the SystemUI chips blank correctly (FIXED 2026-08-30: the
  cards now link-gate to "--" like the chips; bar fills still pending).
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
