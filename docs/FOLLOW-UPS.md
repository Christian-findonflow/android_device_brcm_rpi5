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

2026-09-03/04 (UX pass + rider features, all simulator-verified, bike image
sealed 2026-09-04 13:18):
- Phone button straight to Car Dialer; media button opens Bluetooth Audio
- Glove-sized bottom bar (88dp targets, 12dp gaps, centred under the panel)
- Three exclusion sweeps via soong `overrides`: every AAOS demo/test app,
  Radio, Music, Calendar, Messaging, Gallery, Local Media Player - grid is
  nine real apps (Clock kept)
- Weather chip (Open-Meteo), charger search, LAST RIDE summary (HAL
  VENDOR_RIDE_*), Service log in native Settings, AntennaPod + Breezy Weather
- Test layers: 41 HAL gtests, 14 launcher logic tests, 27-check e2e ride
- Build host fixed: RAM was running 5600 MT/s on a 4800-rated 12700K IMC;
  now 4800 + Intel POR - six clean -j16 image builds, zero corruption events

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

### Hardware shortlist (2026-09-04, from the rider-experience review)
- **Pi 5 RTC battery** (official ML2020 cell, plugs into the Pi 5's own 2-pin
  BAT connector - NOT the CAN HAT's coin-cell holder): rtc-rpi + HCTOSYS are
  in the kernel, so the Pi's clock survives key-off - ride summaries, capture
  timestamps and the weather cache age all depend on it. The Seeed HAT's own
  RTC is an NXP PCF85063 at 0x51, and its driver is NOT in our kernel, so a
  cell in the HAT holder would do nothing here. Cheap, buy.
- **Active Cooler** if not fitted: a sealed dash enclosure in summer will
  throttle a Pi 5.
- **BLE TPMS valve caps** (generic, broadcast manufacturer data): Pi 5 BT
  scans them in a launcher service - no extra hardware beyond the caps.
- **BLE handlebar remote** (HID media/D-pad button pod): arrives as key
  events the launcher can map - glove-safe control without reaching for the
  screen (the parked bar-controls decision).
- **Ambient light sensor**: NOT plug-in today (only TSL2772 has a kernel
  driver; BH1750/APDS9960/TSL2591 would need user-space I2C). Keep the
  solar-elevation dimming.
- **Power hold-up / clean shutdown**: the real reliability item - key-off
  cuts 5 V mid-write (that is how the ESP got its "not properly unmounted").
  Design: CAN silence = shutdown timer + a supercap/UPS on the 5 V rail.
  Needs a proper look, not an impulse buy.

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
    Checked 2026-09-04: the base image already ships a libcamera camera HAL
    (com.android.hardware.camera.libcamera, pisp IPA) and the kernel has the
    imx708/imx219/imx477 drivers, so a Camera Module 3 (Wide) is supported;
    our product just sets ENABLE_CAMERA_SERVICE := false - one-line flip when
    a camera is fitted. Needs the Pi 5 22-pin camera cable and a forward
    housing.
  - **Lean angle / ride telemetry** - hardware decided 2026-09-04: NOT a HAT
    (the header already carries the CAN HAT, plus an NVMe HAT). The Seeed
    CAN-FD HAT v2 has two Grove I2C ports. The Grove IMU 10DOF/9DOF (MPU-9250)
    boards are DISCONTINUED (checked 2026-09-04); in-stock options with drivers
    compiled into our 6.12 Pi kernel: **Seeed Grove 6-Axis LSM6DS3** ($12.90,
    plugs straight in) or, via a $1.95 Qwiic-to-Grove adapter cable (SparkFun
    PRT-15109), **Adafruit ISM330DHCX** (industrial-grade 6-axis, same lsm6dsx
    driver) plus an Adafruit BMP280 for altitude. All 6-axis = lean, pitch,
    g-forces; a magnetometer is not needed for lean.
    CHOSEN 2026-09-04: **Adafruit 4502 ISM330DHCX** (Mouser) + Qwiic-to-Grove
    cable. Bring-up: uncomment `dtparam=i2c_arm=on`; the built-in lsm6dsx driver
    knows ism330dhcx (driver string confirmed in the kernel Image) but the Pi
    i2c-sensor overlay has no param for it, so instantiate at boot from an init
    .rc line: `write /sys/bus/i2c/devices/i2c-1/new_device "ism330dhcx 0x6a"`
    (0x6b if the address jumper is bridged). Barometer: **Adafruit 2651 BMP280 -
    current revision has two STEMMA QT ports** (Mouser shows the old header-only
    photo); NOT Adafruit 4633 (LPS22HB - no driver in our kernel, st_pressure
    unset). `dtoverlay=i2c-sensor,bmp280,addr=0x77` (0x76 with SDO jumper).
    ALTERNATIVE IMU (assessed 2026-09-04): **Adafruit 4754 BNO085** (£22.50,
    Pimoroni) - on-chip sensor fusion (rotation vector, gravity vector,
    calibrated gyro, tilt-compensated heading) so lean/pitch come out ready-made
    instead of us writing and tuning a filter. Cost: NO kernel driver for
    BNO08x - the VHAL would speak SH-2/SHTP itself over /dev/i2c-1
    (CONFIG_I2C_CHARDEV=y, vendor CEVA's Apache-2.0 sh2 C library, poll at
    ~100 Hz, no INT line over Qwiic). Risk: BNO08x I2C clock-stretching broke on
    Pi 4's BCM2835; Pi 5's RP1 DesignWare controller should be fine, and the
    escape hatch is UART-RVC mode through a CP210x/CH341 USB-serial dongle
    (both drivers = y): 100 Hz yaw/pitch/roll as a trivial serial stream.
    DESIGN INSIGHT (2026-09-04, after Christian asked 'are we sure'): in a balanced
    corner the apparent gravity aligns with the bike, so ANY accelerometer-based
    gravity estimate reads ~0 deg lean mid-bend; every IMU-only AHRS (BNO085
    included) survives corners only by gyro integration. The genuinely best
    motorcycle solution is vehicle-aware fusion, which only we can do: lateral
    acceleration = CAN wheel speed x gyro yaw rate, subtracted from the accel
    vector before the gravity estimate (how OEM systems such as Bosch MSC do it),
    gyro-integrated roll corrected only when straight (|a|~g, yaw rate ~0). That
    makes the sensor choice secondary: ISM330DHCX (kernel driver, raw axes) + our
    CAN-aware filter is the cheaper, fully-testable path and beats a generic
    on-chip AHRS in long bends; BNO085 remains the upgrade if a calibrated gyro,
    magnetometer heading or the stability classifier prove worth the SH-2 work.
    Recommendation now: ISM330DHCX + BMP280, vehicle-aware fusion in the VHAL.
    Cabling: STEMMA QT/Qwiic boards carry two PARALLEL connectors, so daisy-chain:
    HAT Grove port -> Qwiic-to-Grove cable -> IMU -> Qwiic-Qwiic cable (Adafruit
    4399 50 mm / 4210 100 mm) -> BMP280. One I2C bus (i2c-1), devices told apart
    by address (0x6a IMU, 0x77 baro); 3V3 from the Grove port is plenty. A custom overlay is also possible with
    prebuilts/misc/linux-x86/dtc. The device then appears as
    /sys/bus/iio/devices/iio:deviceN with in_accel_*/in_anglvel_* raw+scale.
    The DFRobot Fermion 10-DOF (ADXL345/ITG-3205/HMC5883L) would NOT work
    without a kernel rebuild.
    GPS stays USB (decided 2026-09-04): our GNSS HAL (device/brcm/rpi5/gnss_hal)
    reads NMEA GGA/RMC/GSA/GSV/VTG from /dev/ttyACM0 at up to 115200 baud, so any
    USB u-blox puck works unchanged; a Grove/I2C GPS would need a new HAL path
    and puts the antenna inside the enclosure. Upgrade path if wanted: a USB
    u-blox M8N/M9N puck at 10 Hz with an active antenna.
    SOFTWARE SHIPPED 2026-09-04 (parts on order; hardware verification
    pending - see TESTING.md "Lean sensor"): the HAL drives both chips itself
    over /dev/i2c-1 (`vehicle_hal/imu/`: I2C_RDWR transport, ISM330DHCX +
    BMP280 register drivers, 100 Hz) because the kernel's lsm6dsx driver has
    no usable streaming path here (no IIO trigger, no INT line on Qwiic; its
    one-shot sysfs reads cap at ~10 Hz). Vehicle-aware LeanEstimator: CAN
    speed x yaw rate removed as centripetal, dv/dt as longitudinal, roll
    solved as a fixed point (converges from upright into an established
    corner), gyro-integrated, accel-anchored; gyro bias learned at
    standstill; mounting = Level (up axis, Workshop button, rejected if the
    bike moves) + forward axis learned from straight-line acceleration or
    braking; both persisted. Props VENDOR_LEAN_DEG/PITCH/LAT_G/LONG_G/
    BARO_HPA/ALTITUDE_M/IMU_TEMP/IMU_STATUS/IMU_RAW, CFG_IMU_LEVEL command,
    RIDE_MAX_LEAN_L/R in the ride summary (speed-gated). Cluster: lean arc
    with ride max marks (hidden until the sensor is present). Cockpit LAST
    RIDE shows max lean. Simulator: scenario file drives a synthetic IMU at
    the live CAN speed (e2e 37 checks). 22 new host tests.
    Still to do once the sensor is on the bike: verify I2C bring-up (ueventd
    perms, sepolicy, addresses), tune trust band / time constants on a real
    ride capture (log IMU_RAW alongside CAN), then crash/drop detection,
    braking-g ride stats, grade-aware Wh/km from the barometer. A raw IMU
    capture (like the CAN capture) is the natural next tool.
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

- **VHAL restart at runtime blanks the right panel** (seen 2026-09-05 on the
  bike while swapping the HAL over adb): CarService reconnects but
  HomeCockpitActivity is gone until `am start --user 10
  com.android.car.carlauncher/.HomeCockpitActivity` (or a reboot). Boot-time
  restart is fine because the launcher starts afterwards. The cockpit should
  survive a car-service reconnect, and CarLauncher should relaunch it if
  missing. Low priority (only bites during bench work).
- **Backlight sysfs label**: the lights HAL logs an avc denial opening
  `/sys/devices/platform/axi/1000120000.pcie/1f00080000.i2c/i2c-11/11-0045/backlight/11-0045/brightness`
  - our genfscon labels the `/class/backlight/...` symlink path, not the real
  device path. Harmless while SELinux is permissive; fix the genfscon before
  ever going enforcing.
- **Bench loop on the bike (2026-09-05)**: `adb remount` is refused ("Device
  must be bootloader unlocked"); upstream's cmdline `androidboot.verifiedbootstate=orange`
  commit fixes that. Until then: stop the HAL, `mount -o remount,rw /vendor`,
  cp, restorecon, remount ro, start.

## First day on the bike (2026-09-05) - state and open items

Live over adb (192.168.4.73) with the dash on the bike, CAN + optos wired.
Everything below was fixed by pushing binaries to the bike over the LAN; the
flashed image (2026-09-05 mkimg, from 22:19 the day before) predates all of
it, so the NEXT full image build must include the fork tips
(device f9fa15d+, launcher a37cc8bb+) - see TESTING.md "First live capture".

Fixed today: indicators pinned on (active_low parsed with atoi; pull bias
added; raw levels row in Workshop); CAN bus-off with no auto-restart; HAL
reader thread died on interface down/up; BMS OBD2 on 0x7E3/0x7EB; 0x6B1 is
the Orion current-limit message (SoC was wrong); controller current offset
(-1.7 A at rest) replaced by the BMS shunt for display/charging/energy;
gear + ride mode decoded from the status nibble (P/R/D, modes 1-3, Sport);
controller "over voltage" flag gated on the pack's full-charge voltage
(CFG_PACK_MAX_VOLTAGE, Workshop field, default 89.25 V for 21s).

Open:
- **Left indicator**: GPIO 16 never toggles; no free header line toggles
  either, so the opto's left channel/wire is not delivering (Christian has a
  loose wire to check). Pins: left 36 (GPIO16), right 38 (GPIO20), high beam
  40 (GPIO21), GND 34/39. Reassignable in Workshop.
- **High beam input glitch**: GPIO 21 read "on" from 13:22 to ~13:30 with the
  beam off (then recovered). Watch for a flaky opto channel / wire.
- **Unknown controller frames** 0x10261051 (byte 5 AA->B4 while a button is
  pressed) and 0x1026105A (byte 5 tracks motor state 0..7, byte 6 counter).
  Not decoded; capture more rides.
- **Speed calibration**: wheel circumference / gear ratio are defaults; the
  motor rpm field may be motor or wheel rpm. Compare CAN speed with GPS on
  the first ride (a Workshop "GPS vs CAN speed" row would make this a
  one-look job).
- **Runtime VHAL restart kills the cockpit** (see above) - relaunch needed.
- **cf simulator** still carries the old HAL/sim (0x7E0); rebuild before the
  next e2e run.

## Bluetooth phone connection took the bike down (2026-09-05, root-caused)

Symptom: ~5 s after the phone connected the dash reset and then sat on the
NEO logo at every boot; a reflash cured it, `dtoverlay=disable-bt` did not.
UART console + `logcat -b crash` showed system_server dying every 5 s in
`NetworkPolicyManagerService.updateSubscriptions ->
TelephonyManager.getMergedImsisFromGroup`: "unsupported without
android.hardware.telephony.subscription". Trigger: the Bluetooth MAP client
(message access) registers the phone as a REMOTE_SIM subscription
(`MapClientContent.addSubscriptionInfoRecord`), which is persisted in
telephony.db, so the crash loop survives reboots and Bluetooth being off.
This image ships the telephony stack (TelephonyProvider, phone process)
but declares no telephony features, so the feature-enforced API throws
inside system_server.

Fix shipped: `bluetooth.profile.map.client.enabled=false` (we removed the
messaging app; MAP has no consumer). Recovery without reflash: delete the
remote SIM row (`sqlite3 .../telephony.db "delete from siminfo where
subscription_type=1"`) and reboot. Belt-and-braces option not taken:
declaring android.hardware.telephony.subscription would make the call
succeed but turns on more telephony behaviour than a bike wants.

Also seen in the storm: `UsbService.onSwitchUser` NPE (the USB gadget HAL
crashes at boot - tombstones on every boot) - only fatal during the crash
storm's user switches, but the gadget HAL crash itself is a follow-up.

## Audio hub (decided 2026-09-05): phone + earbuds both connect to the dash

Christian: the dash will never have a physical audio output; the rider's
earbuds pair to the DASH, which mixes and routes phone music, calls and
OsmAnd prompts. Findings and plan:

- Legacy audio routing (dynamic car routing off) - the platform routes like a
  phone: media/nav go to a connected A2DP headset automatically.
- The AIDL audio HAL already has the Bluetooth output module (A2DP source,
  AAC) and the HFP AG software-path hook; A2DP source + AVRCP target are on.
- Phase 0 (shipped): `androidboot.audio.tinyalsa.ignore_output/simulate_input`
  on the motorcycle cmdline -> primary output = stub, so the policy always has
  a default output; the Bluetooth A2DP-sink assert (AudioFlinger -19, "Speaker
  unreachable", AudioPolicyManager 0x0) is gone and nav TTS no longer errors.
- Phase 1 (bench): pair earbuds as output; verify music + nav in the helmet,
  ducking, earbud buttons -> phone via AVRCP. Test first with AirPods Pro + iPhone
  but keep it device-agnostic.
- Phase 2 (calls): spike dual voice links (HF client to phone + AG to earbuds
  with the Android 15 HFP software datapath); fallbacks: LE Audio if the
  earbuds do standard LC3, or calls stay phone<->earbuds with the dash doing
  caller ID + answer/decline only. MAP stays off (remote-SIM crash).

### Phase 1 bench test, first attempt (2026-09-05 late evening) - root-caused

Symptoms Christian saw with iPhone + AirPods both connected: music plays only
when the phone takes the AirPods back; the Music screen shows the track but
play/pause/skip do nothing; a call is placed but neither side hears anything.
All three were one fault, proven from the HCI snoop log
(`/data/misc/bluetooth/logs/btsnoop_hci.log.filtered`, parsed with
scratchpad `snoop.py`/`snoop3.py`):

- 22:32:50.583 the dash sent AVDTP START to the AirPods; they ACCEPTED at
  22:32:50.810. The stack never processed the answer: the audio HAL's
  BluetoothAudioPort stayed in STARTING ("start ... failure" every 4.5 s) so
  the earbuds got no audio.
- Everything else queued behind it for 6 min 13 s: the AirPods' own ACL
  supervision-timeout disconnect (22:33:01, HCI) reached BTM only at 22:39:03;
  the phone's SUSPEND/CLOSE/L2CAP disconnects (22:34:37-59) likewise; the
  AVRCP pass-through presses from the Music screen never reached the radio and
  used up all 16 transaction labels ("failed to find free transaction"); the
  ATD dial sat in the queue >10 s so HeadsetClientStateMachine's outgoing-call
  watchdog sent CHUP and Telecom reported the call REMOTE-disconnected.
- Cause: `btif_a2dp_sink_enqueue_buf` (bt_main_thread) takes the sink
  `g_mutex`; the sink worker holds the same mutex around
  `BtifAvrcpAudioTrackWriteData` -> `AAudioStream_write`; the legacy AAudio
  path ignores the timeout ("TODO add timeout to AudioTrack") and blocks
  until the track buffer drains; the buffer only drains once the earbud A2DP
  stream starts; that start needs bt_main_thread. Three-way wait. AOSP never
  hits it because no AOSP device is A2DP sink and source at once.
- Fix: patches/packages_modules_Bluetooth/0001 - non-blocking write with a
  20 ms grace period, drop the remainder ("output not draining" warn, rate
  limited). Local commit 5db0297 on branch neo-motorcycle of
  packages/modules/Bluetooth (not a fork of ours - not pushed).
- Calls, separately: the HF-client role in this stack has NO software SCO
  path (`bta_hf_client_sco.cc` has none; only the AG side has the
  `bluetooth.hfp.software_datapath` hooks). SCO to the phone opens but the
  audio has nowhere to go on a Pi (no PCM wiring). Phase 2 needs either an
  HF-client software datapath (port the AG-side hfp_client_interface use into
  bta_hf_client + a Bluetooth audio HAL session) plus AG->earbuds, or the
  "calls stay phone<->earbuds" fallback.
- Also seen: the AirPods send AVRCP PAUSE to the dash (in-ear detection) -
  the dash's AVRCP target must forward that to the phone's player, check once
  Phase 1 passes.
- Second fault, found on the retest with the deadlock fixed: Music screen said
  "Bluetooth audio disconnected". Snoop showed the phone's AVCTP channel still
  open; the stack had closed it internally. `bta_av_api_set_peer_sep` (coexist
  mode = a2dp source AND sink profiles enabled, which is our config) declares
  "current dut is src" when the earbuds' SNK role becomes known and calls
  `AVRC_UpdateCcb(addr, AVRC_CO_METADATA)`, which sends a synthetic close to
  EVERY legacy (controller) AVRCP connection, i.e. the phone's. Controller
  service -> `setActiveDevice(null)` -> A2DP sink session ended. Fix:
  patches/packages_modules_Bluetooth/0002 (AVCT_GetPeerAddr + skip other
  peers' connections). AOSP's coexist code was written for one peer changing
  role, not two peers in different roles.
- Third fault, same retest: with 0001+0002 the phone stayed active, the
  earbud stream started 0.8 s after the phone's, AVRCP replies came back -
  but the earbuds lost their link after 10 s (reason 8) and the sink dropped
  10-40 % of frames. Snoop: 43 media packets/s TX to the PHONE's link, ~1/s to
  the earbuds. `bta_av_ci_data` hands BTA_AV_SRC_DATA_READY_EVT to every
  started audio stream; the phone->us sink stream is started too, so its SCB
  pulled frames from our encoder and sent them to the phone, duplicating to
  the earbuds only as a copy that the starved link never drained. Fix:
  patches/packages_modules_Bluetooth/0003 (skip streams whose local SEP is
  not SRC in bta_av_data_path and bta_av_dup_audio_buf).
- Still to characterise after 0003: earbud link throughput. The dash is
  peripheral on the phone link and central on the earbud link (scatternet on
  a CYW43455), plus Wi-Fi coexistence (adb/logcat over Wi-Fi during tests
  loads the shared radio - use on-device grep, never stream full logcat).
  Options if the earbud link stays weak: role switch to central on the phone
  link (one piconet), lower AAC bitrate to the earbuds, Wi-Fi off on the bike.
- Fourth fault (found with 0003 in): the dash is actually CENTRAL on both
  links (one piconet, no scatternet), the live sink track in AudioFlinger was
  full and healthy, yet zero media packets left for the earbuds: AVDTP's
  one-current-stream rule (`avdt_scb_event`, "ignore
  AVDT_SCB_API_WRITE_REQ_EVT") counted the phone->us stream as a competing
  transmit stream. The earbuds got silence and sent a remote SUSPEND after
  3 s. Fix: patches/packages_modules_Bluetooth/0004. Also seen: after a
  remote SUSPEND the audio HAL's Bluetooth port stays STARTED and every
  write overflows ("Data 0/512 overflow 1000 ms") until the output goes to
  standby - on a phone the app pauses; here the phone keeps streaming. Needs
  a restart path - DONE as patch 0005 (retry START at 2/5/10 s while our sink
  streams; the earbuds' PLAY key never reaches btif_av on this build because
  the new AVRCP target profile handles it). Bench 2026-09-06 00:00:
  - earbud out/in: NOTHING happens (no AVRCP, no suspend). AirPods report
    in-ear state only to Apple sources; with the dash as source they are
    silent. Not a dash fault; generic earbuds send AVRCP PAUSE, which the
    target forwards to the phone (untested).
  - Siri: the iPhone opens SCO to the dash for Siri; the HF client turns it
    into an ACTIVE "unknown call" in Telecom (TC@1, 5 s) - Telecom takes call
    focus, the sink loses focus; iOS pauses the music itself. When Siri ends
    the phone resumes, our earbud START waits ~4.5 s for the AirPods (still
    held by the iPhone), and the AirPods send AVRCP PAUSE to the dash as the
    iPhone hands them back -> forwarded to the phone -> music stops 500 ms
    after resuming. Fix: patch 0006 (ignore a PAUSE from a device we are not
    playing into, or started < 1.5 s ago). Patch 0005's retry did fire but
    the phone had already paused itself, so it had nothing to do.
  - Retest with 0006 (00:09): music comes back and stays. Siri's SCO shows
    on the dash as the green microphone privacy indicator plus a brief
    screen refresh (the in-call UI starting and finishing with the 2 s
    "unknown call"). Telecom also logs a bogus emergency InCallService
    component (com.android.dialer/com.android.car.dialer...) it cannot find -
    noise, note for the dialer clean-up. Real calls are Phase 2.
- RESULT 23:41 with patches 0001-0004 on the bike: Christian - "music works
  and is stable, play pause skip etc all working well". Snoop: 43 media
  packets/s to the earbuds sustained for minutes, zero AVDTP drops, phone
  pause -> local suspend of the earbud stream -> both restart on play. Only
  residue: an occasional single-packet drop (1020 frames) at the sink write
  every ~5-10 s - the LOW_LATENCY AAudio track buffer was 2052 frames (46 ms)
  and phone packets arrive in bursts. Christian heard it as a tick; fixed in
  patch 0001 (second revision): 8192-frame buffer, performance mode NONE. Phase 1 = DONE. Next: Phase 2 calls
  (HF-client SCO software path + AG to earbuds, or phone<->earbuds fallback),
  remote-SUSPEND restart path, OsmAnd prompt ducking check, v9 image =
  ~/images/RaspberryVanillaAOSP16-20260905-rpi5_motorcycle-v9.img.gz.

## Nav prompts: no speech engine on the dash (found 2026-09-06 00:35)

Christian started a route in OsmAnd with music playing and heard nothing.
Two reasons, neither the audio path: (1) no GPS yet, so no manoeuvre is ever
reached and OsmAnd never speaks; (2) the image has NO text-to-speech engine
(`pm list packages` shows no tts package, `tts_default_synth` is null), so an
OsmAnd "TTS" voice can never speak either. To test prompts and ducking before
the GPS is fitted: pick a RECORDED voice in OsmAnd (Navigation settings ->
Voice prompts -> Voice guidance -> a recorded "en" voice, downloaded over
Wi-Fi), enable the "OsmAnd development" plugin and use its "Simulate your
position" while a route is active. For production either ship a TTS engine
in the image (eSpeak-NG or a Sherpa/Piper-based engine; AOSP has none) or
standardise on recorded voices. The sink handler's focus behaviour is AOSP
car standard: MAY_DUCK prompts duck the phone's music via the track gain,
TRANSIENT focus (assistant/phone) pauses the phone over AVRCP.

Bench 2026-09-06 (recorded en-gb voice + simulated position): prompts DO play
(JsMediaCommandPlayer around_1_mile.ogg / left.ogg) and the sink ducks the
music to 25 % and restores it - the focus path is right. But OsmAnd plays them
on the VOICE-CALL stream (USAGE_VOICE_COMMUNICATION), whose index on the A2DP
device was 3/15, and while a call-stream sound is active AudioService lets
that stream drive the earbuds' AVRCP absolute volume ("absolute volume driving
streams 0"): the prompt was inaudible under the music and the earbuds' volume
was yanked to ~20 % for the duration, occasionally not restored ("music comes
back very low sometimes"). Interim: `media volume --stream 0 --set 12` on the
bike (persisted per device by AudioService). Proper fix: OsmAnd -> Navigation
settings -> Voice prompts -> audio output = Media/music, so prompts share the
music stream and volume (ducking still works, it is focus-based). Image-side
follow-up: a sane default for the voice-call index on Bluetooth A2DP (it
matters again for Phase 2). Map stutter during prompts = Pi CPU: ogg decode +
AAC decode + AAC encode + map render; watch it once the GPS is in.

## Volume panel (shipped 2026-09-06 09:30)

Christian asked for an easy way to set volumes. Facts found: CarAudioService
runs in LEGACY mode here (no dynamic routing), where it exposes exactly three
volume groups = STREAM_MUSIC / STREAM_ALARM / STREAM_RING
(`CarAudioDynamicRouting.STREAM_TYPES`). The stock top-bar sound icon opens
SystemUI's `qc_volume_panel` whose sliders are Car Settings quick controls
keyed by usage; call and navigation usages map to no group in legacy mode, so
only the media slider and the "Sound settings" footer ever showed - and the
voice-call stream (3/15, dragging the earbuds' absolute volume down during
prompts) was not adjustable anywhere.

Shipped: the launcher's `audio/VolumeQCProvider` (content://com.android.car.
carlauncher.qc/volume, allowlisted to SystemUI) serves three sliders bound
straight to streams - Music (STREAM_MUSIC; also the earbuds' AVRCP absolute
volume; subtitle = earbud name), Prompts (STREAM_NOTIFICATION; OsmAnd must be
set to "Notification" output for this to be its knob), Calls
(STREAM_VOICE_CALL; enabled only while a phone is connected over HFP). The
SystemUI fork's `qc_volume_panel.xml` points at it. Two permissions were
needed: BLUETOOTH_CONNECT (runtime; pre-granted via
default-permissions-carlauncher.xml on a fresh flash, `pm grant` on the
bench) and MODIFY_AUDIO_SETTINGS_PRIVILEGED (automotive audio hardening:
`AS.HardeningEnforcer: Preventing volume method` silently drops
setStreamVolume from app uids without it; allowlisted in the Car fork's
com.android.car.carlauncher.xml). Verified by Christian.

Boot race found on the same morning: CarService's early-startup entry for
`HomeCockpitLauncherService` used `bind=start`; startService() at unlock
throws BackgroundServiceStartNotAllowedException when the launcher uid is
momentarily background, and CarService restarted 3x. Now `bind=bind` and the
service launches the cockpit from onCreate() - verified 09:34: clean boot,
CarService up since boot, cockpit resumed (the service gets created twice at
unlock and launches the singleTask cockpit twice; harmless). Also seen at every boot: one
SystemUI `DeadSystemException` on wmshell.main (SystemUI restarts once during
boot) - not investigated, cosmetic so far.

## Now Playing screen (shipped 2026-09-06 10:16)

Christian: "can we improve our music app". The stock car Media app on the
592x352 panel showed a big empty browse area with one "Music" tile, a
truncated title and a thin control strip. Shipped
`launcher/audio/NowPlayingActivity` (MediaBrowser -> BluetoothMediaBrowserService
-> MediaController; art from the AvrcpCoverArtProvider content URI; progress
from PlaybackState position/speed): cover art 210dp, title/artist/album,
seek bar with times, prev / play-pause / next at 72dp height, library button
that opens the old browser (MEDIA_TEMPLATE intent). Bottom-bar music button
now targets it. Gotcha: launcher-package activities can inherit the LEFT
(cluster) task display area when CarLauncher's task was just recreated -
MotoSplitDisplayAreaController now pins NowPlayingActivity to the default
container, same as HomeCockpitActivity. Bench trap: `pm uninstall --user 10`
on the system launcher hides it for the user (FallbackHome appears);
`pm install-existing --user 10 com.android.car.carlauncher` restores it.
Ideas not done: queue/up-next list from the AVRCP browse tree, favourite
button, volume shortcut in the screen.

## Phone reconnect race: phonebook download starves audio and calls (2026-09-06 09:40)

Christian: "sometimes connecting the earbuds doesn't enable the bluetooth
audio app" - Music screen "Bluetooth audio disconnected" although the AirPods
were connected. The earbuds were fine; the PHONE's A2DP sink and HF client had
never come up on that boot. Snoop + logcat: at ACL connect the dash starts
HFP, A2DP sink, PAN and PBAP together; the PBAP contacts download (vCards
WITH photos: base64 blobs, 642 contacts, up to 62 kB/s, 30 s) saturates the
link; the SLC AT round trips take 0.3-0.7 s each and HeadsetClientStateMachine's
10 s CONNECTING timeout fires (three times, then gives up); the AVDTP
capability queries crawl at 0.5 s each and the iPhone closes the signalling
channel ~5 s after opening it. Whether a boot works depends on how fast the
download happens to run. Fix: patch 0007 - PbapClientService.connect() posts
the real connect 15 s later (token = device, cancelled on ACL drop /
disconnect), PROPERTY_PHOTO removed from the vCard filter, HF client
CONNECTING_TIMEOUT_MS 20 s. Verified 09:48: HF client, A2DP sink and AVRCP
controller all Connected within seconds of the ACL, PBAP deferred 15 s and
done 9 s later (no photos), zero crashes on that boot (even the usual SystemUI
DeadSystemException did not occur).

## Phase 2 scoping: call audio through the dash (written 2026-09-06 00:15)

What exists today, from the code (packages/modules/Bluetooth, device audio HAL):

- HF-client role (dash <- phone): `bta/hf_client/bta_hf_client_sco.cc` (663
  lines) opens/closes SCO with BTM_CreateSco and assumes a hardware PCM
  path. It has NO software datapath: no `HfpClientInterface`, no
  Encode/Decode/Offload objects, no `hfp_software_datapath_enabled`.
- AG role (dash -> earbuds): `bta/ag/bta_ag_sco.cc` has the full software
  path (Android 15 `bluetooth.hfp.software_datapath.enabled`): Encode/Decode
  interfaces, ConfirmStreamingRequest/CancelStreamingRequest, Write of RX
  SCO into the decoder, Read of TX PCM from the encoder. This is the template
  to port to the HF client (~200 lines).
- SCO over HCI itself is standard: with offload=false the stack sends
  Enhanced Setup Synchronous Connection with data path = HCI; the CYW43455
  firmware supports it (BlueZ does HFP over the same UART). 3 Mbaud UART is
  plenty for two 64 kbit/s SCO links plus A2DP.
- Audio HAL (device/brcm/rpi5/audio = our copy of the AOSP default AIDL HAL):
  `bluetooth/ModuleBluetooth.cpp` and `DevicePortProxy.cpp` only know the
  A2DP and hearing-aid session types; `Bluetooth.cpp` keeps ScoConfig/HfpConfig
  as flags with no streams behind them; the policy XML declares no BT SCO
  device or mix ports. So even the AG-side software path has nowhere to go
  until the HAL grows HFP ports (HFP_SOFTWARE_ENCODING/DECODING_DATAPATH
  sessions from libbluetooth_audio_session, BT SCO in/out device ports, mix
  ports, routes).
- The bridge: a phone call needs phone-downlink (SCO#1 decode) -> earbuds
  (SCO#2 encode) and earbuds-mic (SCO#2 decode) -> phone-uplink (SCO#1
  encode), full duplex, 8/16 kHz mono. Two SCO links on one CYW43455 is
  within spec but untested here. Cleanest place for the bridge: inside the
  audio HAL process (two BluetoothAudioPort proxies cross-wired, no
  AudioFlinger involvement, lowest latency), with the HAL still exposing the
  HF side to AudioFlinger so OsmAnd/UI sounds can be mixed into the earbud
  leg if wanted.

Plan (each step testable on the bench):
1. HAL: add HFP device/mix ports + the two HFP session types (encode = HAL
   -> stack, decode = stack -> HAL) in ModuleBluetooth/DevicePortProxy; policy
   XML routes. Verify with the AG role alone: dash as AG to the earbuds, play
   a tone to "BT SCO out", hear it in the earbuds (needs
   bluetooth.profile.hfp.ag.enabled=true + hfp.hf coexistence check in
   Config.java - none found).
2. Stack: port the AG software-path hooks into bta_hf_client_sco.cc; verify
   with the phone alone: call audio arrives as "BT SCO in" and a test tone
   sent to "BT SCO out" reaches the caller.
3. Bridge the two inside the HAL; ring/answer/hang-up already work through
   HfpClientConnectionService + Car Dialer (dial went through tonight).
4. Fallback if 2 SCO links misbehave on this controller: calls stay
   phone<->earbuds (the phone owns the AirPods for calls anyway, as Siri
   showed) and the dash keeps caller ID + answer/decline.
Estimate: 2-4 bench days. Not started.

SUPERSEDED 2026-09-06 11:30 by the section below: the audio-HAL route is not
needed. The audio framework can only model one "BT SCO" device, so even with
HFP ports in the HAL the bridge would have been custom code; doing it in the
stack on the raw HCI payloads is ~200 lines and touches nothing else.

## Phase 2 step 1: in-stack SCO bridge (built 2026-09-06 11:30, bench test pending)

Decision: use the native AAOS/AOSP pieces for everything except the voice
bytes. What is stock and untouched:

- HF client (dash <- phone): `HfpClientConnectionService` already publishes
  the phone's calls into Telecom; the car Dialer shows/answers/ends them
  (proven 2026-09-05).
- AG (dash -> earbuds): `HeadsetService` + `BluetoothInCallService` forward
  every Telecom call (the HF client's calls are not "external", so they are
  included) to the earbuds as HFP indicators: the earbuds ring with their own
  tone (`isInbandRingingEnabled()` is already false while an HF client is
  connected - AOSP anticipated a car kit in both roles), a stem press answers
  or hangs up through Telecom -> HF client -> phone, and the AG opens its
  SCO link to the earbuds when the call becomes active.
- Both roles coexist in `Config.java` (independent entries). AG enabled with
  `bluetooth.profile.hfp.ag.enabled=true` (aosp_rpi5_motorcycle.mk).

What was custom (patch 0008, local commit on packages/modules/Bluetooth):

- `bta/hf_client/bta_hf_client_sco.cc` hardcoded offload=true in all seven
  `esco_parameters_for_codec()` calls, so the phone's SCO link was set up
  with data path = PCM (snoop: Enhanced Accept Synchronous Connection with
  input/output data path 0x01) and the voice went to the controller's unwired
  PCM pins. Now `hfp_hal_interface::get_offload_enabled()`, i.e. data path =
  HCI when `bluetooth.hfp.software_datapath.enabled=true`.
- New `stack/btm/btm_sco_bridge.{h,cc}`: when
  `bluetooth.neo.sco_bridge.enabled` is set (sampled at each SCO connect),
  every in-band SCO packet is forwarded to the other connected in-band SCO
  link (byte ring per link, re-packetised to the destination's packet length,
  payload copied verbatim so mSBC H2 framing/sequence numbers and the far
  end's packet-loss concealment are preserved). Same codec required on both
  links (mSBC is what both the iPhone and AirPods negotiate); a mismatch is
  logged once and dropped - transcoding is the follow-up if it ever happens.
  Stats line per link every ~15 s (tag neo_sco_bridge). Hooks:
  `btm_route_sco_data` (before the single-active-SCO check, which would have
  dropped the second link's packets), `btm_sco_connected`,
  `btm_sco_on_disconnected`; `btm_send_sco_packet_to(handle, data)` added.

Controller facts from the snoop (CYW43455, BCM4345C0 fw 003.001.025):
Read Buffer Size reports 1 SCO buffer of 64 bytes; both links are eSCO 2-EV3,
7.5 ms interval, 60-byte payloads, mSBC; the dash is central on both. The
bridge sends one packet out per packet in, so the controller never holds
more than about one queued voice packet per link. UART load for two links:
~34 kB/s of ~300 kB/s (A2DP is suspended during a call by `bta_av`).

Unknowns the bench test answers (in order): (1) does the 43455 route SCO
over HCI when the Enhanced Setup/Accept command says data path 0x00 (BlueZ on
Pi hardware does this; the fallback is the Broadcom Write_SCO_PCM_Int_Param
vendor command, routing=1, sent by our BT HAL at init); (2) does it hold two
eSCO links at once; (3) does `HeadsetService` start cleanly with no telephony
hardware (`HeadsetPhoneState` talks to TelephonyManager).

Bench procedure: `scratchpad/bench_apex_swap_p2.sh` (push apex, reboot, set
the three props, restart BT). Then AirPods out of the case (they should now
also connect HFP to the dash: `Profile: HeadsetService` in dumpsys
bluetooth_manager shows Connected), phone connected, place a call from the
Dialer or the phone. Expect two `neo_sco_bridge: link 0x... connected` lines
and rx/tx counters rising on both. Voice both ways in the AirPods.

Bench bring-up 2026-09-06 10:45 (apex sha bcf138ba..., props set at runtime
with setprop + BT restart; they are baked into the v16 image): HeadsetService
starts cleanly with no telephony hardware; the phone reconnected as before
(HF client + A2DP sink + AVRCP + PBAP); the AirPods connected A2DP at
10:45:50 and then, on their own, opened HFP to the dash's AG at 10:46:14
(they re-read our SDP, no re-pairing needed) and the AG made them the
active headset device, in-band ringing off. Unknowns 1-2 (SCO over HCI, two
eSCO links) still need a real call - waiting on Christian.

## system_server crashes once at EVERY boot (confirmed 2026-09-05)

`UsbService.onSwitchUser` NPE on the android.fg thread at the user-10
switch: UsbService is null because the USB gadget HAL
(com.android.hardware.usb.gadget.rpi5, "bcdVersion must be 0x0100" in
configfs) aborts at boot, twice, before UsbService can bind. The framework
restarts and the boot completes, so it went unnoticed, but it costs ~5 s of
boot, restarts audioserver (which is why the audio HAL rename mattered) and
leaves tombstones on every boot. Fix the gadget HAL crash (or drop the
gadget HAL: the dash never acts as a USB device on the bike; dwc2 is in
peripheral mode only for adb over USB-C on the bench).
