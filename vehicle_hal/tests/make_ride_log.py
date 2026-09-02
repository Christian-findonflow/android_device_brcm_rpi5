#!/usr/bin/env python3
"""Generate a synthetic ride as a candump -l log for moto_can_replay.

Profile: accelerate to cruise, long cruise, regen deceleration, a 20 km/h
town stretch, brake to a stop. Controller status frames (0x10261022) at
10 Hz plus BMS 0x6B1 broadcasts at 1 Hz.

Usage:
    python3 make_ride_log.py > ride.log
    adb push ride.log /data/local/tmp/
    adb shell moto_can_replay vcan0 -f /data/local/tmp/ride.log

Optional args: cruise_kmh duration_s (defaults 56, 150).
Keep wheel/gear in sync with the HAL defaults (or the configured props).
"""
import sys

CIRC_M = 1.894      # DEFAULT_WHEEL_CIRCUMFERENCE_M
RATIO = 4.0         # DEFAULT_GEAR_RATIO
VOLT_RAW = 720      # 72.0 V
SOC_START = 60

cruise = (float(sys.argv[1]) if len(sys.argv) > 1 else 56.0) / 3.6
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 150.0

def rpm_for(mps):
    return int(round(mps * 60.0 * RATIO / CIRC_M))

def speed_at(t):
    if t < 15: return cruise * t / 15.0
    if t < dur - 45: return cruise
    if t < dur - 35: return cruise - (cruise - 5.5) * (t - (dur - 45)) / 10.0
    if t < dur - 15: return 5.5
    if t < dur - 5: return 5.5 * (1 - (t - (dur - 15)) / 10.0)
    return 0.0

def current_at(t):
    if t < 15: return 80.0
    if t < dur - 45: return 45.0
    if t < dur - 35: return -25.0   # regen
    if t < dur - 15: return 15.0
    if t < dur - 5: return -10.0
    return 0.0

base = 1700000000.0
t = 0.0
while t <= dur:
    rpm = rpm_for(speed_at(t))
    cur = int(round(current_at(t) * 10)) & 0xFFFF
    # A controller fault (bit 3 = CONTROLLER) burns for 30 s mid-cruise so the
    # end-to-end test can watch VENDOR_FAULT_FLAGS raise and clear (65-115 s).
    err = 0x08 if 65 <= t < 115 else 0x00
    d = [err, 0x30, rpm & 0xFF, (rpm >> 8) & 0xFF,
         VOLT_RAW & 0xFF, (VOLT_RAW >> 8) & 0xFF, cur & 0xFF, (cur >> 8) & 0xFF]
    print("(%.6f) vcan0 10261022#%s" % (base + t, "".join("%02X" % b for b in d)))
    if abs(t - round(t)) < 1e-9:
        soc = SOC_START if t < dur / 2 else SOC_START - 1
        b = [0x00, 0x63, 0x00, soc, 0x03, 0x02, 0x00, 0x41]   # temp raw 65 = 25 C
        print("(%.6f) vcan0 6B1#%s" % (base + t + 0.005, "".join("%02X" % x for x in b)))
        # Temps frame at 1 Hz: controller/motor warm through the ride,
        # throttle open whenever the motor is pulling current.
        ctl = int(25 + 20.0 * t / dur)
        mot = int(25 + 35.0 * t / dur)
        thr = 40 if current_at(t) > 0 else 0
        d2 = [ctl, mot, 0, 0, thr, 0, 0, 0]
        print("(%.6f) vcan0 10261023#%s" % (base + t + 0.010, "".join("%02X" % x for x in d2)))
    t = round(t + 0.1, 1)
