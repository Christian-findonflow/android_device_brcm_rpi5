#!/bin/bash
# End-to-end CAN behavior test against a running emulator (or bench Pi).
#
# Replays a synthetic ride into the guest's vcan0 and asserts what the whole
# Android stack does with it: speed/gear decode, gear-linked parking brake,
# driving-state transitions, SoC, link status, range learning, odometer/trip
# accumulation, and the blanking trust-cascade when the bus goes silent.
#
# Usage: e2e_can_test.sh [serial]   (default 0.0.0.0:6520; needs adb root)
set -u
SERIAL="${1:-0.0.0.0:6520}"
A="adb -s $SERIAL"
HERE="$(cd "$(dirname "$0")" && pwd)"

PROP_SPEED=291504647       # PERF_VEHICLE_SPEED (m/s)
PROP_ODO=291504644         # PERF_ODOMETER (km)
PROP_BATT=291504905        # EV_BATTERY_LEVEL (%)
PROP_RANGE=291504904       # RANGE_REMAINING (m)
PROP_GEARSEL=289408000     # GEAR_SELECTION
PROP_BRAKE=287310850       # PARKING_BRAKE_ON
PROP_LINK=557842499        # VENDOR_LINK_STATUS (1=ctrl, 2=bms)
PROP_COOLANT=291504897     # ENGINE_COOLANT_TEMP (controller degC)
PROP_OIL=291504900         # ENGINE_OIL_TEMP (motor degC)
PROP_VOLT=559939585        # VENDOR_BATTERY_VOLTAGE
PROP_FAULT=557842496       # VENDOR_FAULT_FLAGS
PROP_CHARGING=557842500    # VENDOR_CHARGING
PROP_RAWGEAR=557842504     # VENDOR_RAW_GEAR_STATUS
PROP_LEAN=559939680        # VENDOR_LEAN_DEG (+ = right)
PROP_LATG=559939682        # VENDOR_LAT_G
PROP_IMUSTATUS=557842535   # VENDOR_IMU_STATUS bitfield
PROP_MAXLEAN_L=559939689   # VENDOR_RIDE_MAX_LEAN_L
PROP_MAXLEAN_R=559939690   # VENDOR_RIDE_MAX_LEAN_R
IMU_SIM="/data/vendor/motodash/imu_sim"
imu_sim() { $A shell "echo '$1' > $IMU_SIM"; }
GEAR_DRIVE=8
GEAR_PARK=4

PASS=0; FAIL=0
say()  { printf '%s\n' "$*"; }
prop() { $A shell "dumpsys android.hardware.automotive.vehicle.IVehicle/default 2>/dev/null" \
           | tr -d '\r' | awk -v p="$1" '$1=="PropId" && $2==p":" {print $3; exit}'; }
dstate() { $A shell "dumpsys car_service 2>/dev/null" | tr -d '\r' \
           | awk -F': ' '/Current Driving State/ {print $2; exit}'; }
check() { # name expr_result expected_desc
  if [ "$2" = "1" ]; then PASS=$((PASS+1)); say "  PASS  $1"
  else FAIL=$((FAIL+1)); say "  FAIL  $1  ($3)"; fi
}
near() { awk -v a="$1" -v b="$2" -v t="$3" 'BEGIN{d=a-b; if(d<0)d=-d; print (d<=t)?1:0}'; }

say "== e2e CAN test against $SERIAL =="
$A root >/dev/null 2>&1; sleep 2

ODO0=$(prop $PROP_ODO); ODO0=${ODO0:-0}
say "baseline: odometer=${ODO0}km  driving_state=$(dstate)"

say "-- generating and pushing ride (150s: accel, 56km/h cruise, regen, stop)"
python3 "$HERE/make_ride_log.py" > /tmp/e2e_ride.log 2>/dev/null || { say "FATAL: make_ride_log.py failed"; exit 2; }
$A push /tmp/e2e_ride.log /data/local/tmp/e2e_ride.log >/dev/null || { say "FATAL: adb push failed"; exit 2; }

say "-- lean sensor: simulated (scenario file drives the synthetic IMU at the live CAN speed)"
imu_sim "lean=0 alt=120"
for i in 1 2 3 4 5 6 7; do   # the HAL probes for a source every 5 s
  ST=$(prop $PROP_IMUSTATUS); [ "$((${ST:-0} & 16))" = "16" ] && break
  sleep 1
done
check "IMU source = simulated (status ${ST:-none})" "$([ "$((${ST:-0} & 17))" = "17" ] && echo 1 || echo 0)" "bits PRESENT|SIMULATED"

say "-- starting BMS responder + replay"
$A shell "nohup moto_bms_sim vcan0 >/dev/null 2>&1 &"
$A shell "nohup moto_can_replay vcan0 -f /data/local/tmp/e2e_ride.log >/dev/null 2>&1 &"
T0=$(date +%s)
at() { local tgt=$1; local now=$(date +%s); local d=$((T0+tgt-now)); [ $d -gt 0 ] && sleep $d; }

at 30   # mid-cruise
say "-- t+30s (cruise):"
V=$(prop $PROP_SPEED)
check "speed ~15.6 m/s (got $V)" "$(near ${V:-0} 15.6 0.7)" "expected 56 km/h cruise"
check "gear = DRIVE (got $(prop $PROP_GEARSEL))" "$([ "$(prop $PROP_GEARSEL)" = "$GEAR_DRIVE" ] && echo 1 || echo 0)" "GEAR_SELECTION should be 8"
check "parking brake OFF" "$([ "$(prop $PROP_BRAKE)" = "0" ] && echo 1 || echo 0)" "brake must release in D"
DS=$(dstate)
check "driving state MOVING (got $DS)" "$([ "$DS" = "2" ] && echo 1 || echo 0)" "CarDrivingStateService should say 2"
L=$(prop $PROP_LINK)
check "controller+BMS links alive (got $L)" "$([ "$L" = "3" ] && echo 1 || echo 0)" "LINK_STATUS should be 3"
B=$(prop $PROP_BATT)
check "battery SoC ~60% (got $B)" "$(near ${B:-0} 60 1.1)" "0x6B1/PID SoC"
check "pack voltage 72.0 (got $(prop $PROP_VOLT))" "$(near $(prop $PROP_VOLT) 72 0.2)" "controller frame voltage"
CT=$(prop $PROP_COOLANT); MT=$(prop $PROP_OIL)
check "controller temp warming (got $CT)" "$(awk -v x="${CT:-0}" 'BEGIN{print (x>25.5 && x<50)?1:0}')" "0x10261023 byte0"
check "motor temp warming (got $MT)" "$(awk -v x="${MT:-0}" 'BEGIN{print (x>25.5 && x<65)?1:0}')" "0x10261023 byte1"
check "raw gear byte 0x30 (got $(prop $PROP_RAWGEAR))" "$([ "$(prop $PROP_RAWGEAR)" = "48" ] && echo 1 || echo 0)" "byte1 verbatim"
check "no fault yet" "$([ "$(prop $PROP_FAULT)" = "0" ] && echo 1 || echo 0)" "fault window starts at t+65"

say "-- t+31..44s: corners at cruise speed (lean must be read through the centripetal term)"
imu_sim "lean=30 alt=120"
at 37
LEAN=$(prop $PROP_LEAN); LG=$(prop $PROP_LATG); ST=$(prop $PROP_IMUSTATUS)
check "lean 30 deg right (got ${LEAN:-none})" "$(near ${LEAN:-0} 30 2.5)" "VENDOR_LEAN_DEG mid-corner"
check "lateral g ~0.58 right (got ${LG:-none})" "$(near ${LG:-0} 0.577 0.08)" "VENDOR_LAT_G = tan(30)"
check "IMU status valid+calibrated (got ${ST:-none})" "$([ "$((${ST:-0} & 61))" = "61" ] && echo 1 || echo 0)" "PRESENT|LEVEL|FORWARD|SIM|VALID"
imu_sim "lean=-20 alt=120"
at 43
LEAN=$(prop $PROP_LEAN)
check "lean 20 deg left (got ${LEAN:-none})" "$(near ${LEAN:-0} -20 2.5)" "sign: + = right"
imu_sim "lean=0 alt=120"

say "-- fault window (65-115 s):"
at 90
F=$(prop $PROP_FAULT)
check "CONTROLLER fault raised (got $F)" "$([ "$F" = "8" ] && echo 1 || echo 0)" "byte0 bit3 during 65-95s"

say "-- waiting for ride to finish"
at 160
check "fault cleared (got $(prop $PROP_FAULT))" "$([ "$(prop $PROP_FAULT)" = "0" ] && echo 1 || echo 0)" "fault window over"
check "no phantom charging after regen stop" "$([ "$(prop $PROP_CHARGING)" = "0" ] && echo 1 || echo 0)" "regen tail must not flash the garage takeover"

say "-- charging test: BMS sim off (its discharge current outranks), charger plugged"
$A shell "pkill moto_bms_sim" 2>/dev/null
sleep 7
ARRIVED=0
for i in 1 2 3 4 5 6 7 8; do   # >5s sustained: the charging dwell must be met
  OUT=$($A shell moto_can_replay vcan0 "10261022#00000000D002B0FF" 2>&1 | tr -d '\r')
  [ -n "$OUT" ] && [ "${OUT#sent}" = "$OUT" ] && say "    (send $i: $OUT)"
  sleep 1
  [ "$(prop $PROP_RAWGEAR)" = "0" ] && ARRIVED=1
done
check "charging frames reach the HAL (raw byte 0x00)" "$ARRIVED" "sends kept failing"
check "charging detected at standstill" "$([ "$(prop $PROP_CHARGING)" = "1" ] && echo 1 || echo 0)" "rpm 0 + current -8A, gear P"
$A shell moto_can_replay vcan0 "10261022#00000000D0020000" >/dev/null
sleep 2
check "charging ends at zero current" "$([ "$(prop $PROP_CHARGING)" = "0" ] && echo 1 || echo 0)" "current back to 0"
sleep 1
check "gear = PARK (got $(prop $PROP_GEARSEL))" "$([ "$(prop $PROP_GEARSEL)" = "$GEAR_PARK" ] && echo 1 || echo 0)" "GEAR_SELECTION should be 4"
check "parking brake ON" "$([ "$(prop $PROP_BRAKE)" = "1" ] && echo 1 || echo 0)" "gear-linked brake"
DS=$(dstate)
check "driving state PARKED (got $DS)" "$([ "$DS" = "0" ] && echo 1 || echo 0)" "should be 0"

say "-- bus silent: links must drop"
sleep 12
L=$(prop $PROP_LINK)
check "links dead (got $L)" "$([ "$L" = "0" ] && echo 1 || echo 0)" "LINK_STATUS should be 0"
V=$(prop $PROP_SPEED)
RS=$(prop 557842505)
check "ride summary published at key-off (seq ${RS:-none})" "$([ "${RS:-0}" -ge 1 ] 2>/dev/null && echo 1 || echo 0)" "VENDOR_RIDE_SEQ"
RM=$(prop 559939664)
check "ride distance ~1.77km (got ${RM:-0}m)" "$(near ${RM:-0} 1770 60)" "VENDOR_RIDE_DISTANCE_M"
check "speed 0 (got $V)" "$(near ${V:-1} 0 0.01)" "last frame was standstill"
ML=$(prop $PROP_MAXLEAN_L); MR=$(prop $PROP_MAXLEAN_R)
check "ride max lean R ~30 (got ${MR:-none})" "$(near ${MR:-0} 30 2.5)" "VENDOR_RIDE_MAX_LEAN_R with the summary"
check "ride max lean L ~20 (got ${ML:-none})" "$(near ${ML:-0} 20 2.5)" "VENDOR_RIDE_MAX_LEAN_L with the summary"

say "-- Workshop Level command (bike still) then clear, then unplug the simulated sensor"
$A shell "cmd car_service set-property-value 0x21400068 0 1" >/dev/null 2>&1
sleep 3
ST=$(prop $PROP_IMUSTATUS)
check "level captured (status ${ST:-none})" "$([ "$((${ST:-0} & 4))" = "4" ] && [ "$((${ST:-0} & 64))" = "0" ] && echo 1 || echo 0)" "LEVEL_SET without LEVEL_FAILED"
$A shell "cmd car_service set-property-value 0x21400068 0 0" >/dev/null 2>&1
sleep 2
ST=$(prop $PROP_IMUSTATUS)
check "calibration cleared (status ${ST:-none})" "$([ "$((${ST:-0} & 12))" = "0" ] && echo 1 || echo 0)" "neither LEVEL_SET nor FORWARD_SET"
say "-- ride capture switch also records the raw IMU stream"
$A shell "cmd car_service set-property-value 0x21400046 0 1" >/dev/null 2>&1
sleep 4
IMULOG=$($A shell "ls /data/vendor/motodash/imu-*.log 2>/dev/null | head -1" | tr -d '\r')
LINES=$($A shell "wc -l < ${IMULOG:-/nonexistent} 2>/dev/null" | tr -d '\r ')
check "imu-*.log written at ~100 Hz (${LINES:-0} lines in 4 s)" "$([ "${LINES:-0}" -ge 200 ] 2>/dev/null && echo 1 || echo 0)" "capture switch must also log the sensor"
$A shell "cmd car_service set-property-value 0x21400046 0 0" >/dev/null 2>&1
$A shell "rm -f /data/vendor/motodash/imu-*.log /data/vendor/motodash/can-*.log"
$A shell "rm -f $IMU_SIM"
sleep 3
ST=$(prop $PROP_IMUSTATUS)
check "sensor gone -> status 0 (got ${ST:-none})" "$([ "${ST:-1}" = "0" ] && echo 1 || echo 0)" "source loss must clear PRESENT"

say "-- ride bookkeeping"
ODO1=$(prop $PROP_ODO)
DELTA=$(awk -v a="${ODO1:-0}" -v b="$ODO0" 'BEGIN{print a-b}')
check "odometer advanced ~1.77km (got +${DELTA}km)" "$(near $DELTA 1.77 0.15)" "integrated ride distance"
R=$(prop $PROP_RANGE)
check "range learned (got ${R}m, want >10000)" "$(awk -v r="${R:-0}" 'BEGIN{print (r>10000)?1:0}')" "RANGE_REMAINING after Wh/km learning"

say "-- health"
CRASH=$($A shell "ls /data/tombstones/ 2>/dev/null | wc -l" | tr -d '\r ')
check "no new tombstones (count=$CRASH)" "$([ "${CRASH:-0}" -eq 0 ] && echo 1 || echo 0)" "native crashes during ride"

say ""
say "== RESULT: $PASS passed, $FAIL failed =="
[ $FAIL -eq 0 ]
