#!/bin/bash
# Live status line for a bike-on-the-bench session (wheel spin, switches,
# optoisolators). Usage: bench_bike_watch.sh [ip] [interval_s]
# Columns: speed km/h | gear/raw | brake | flags | turn | beam | rawGPIO bits
# (bit0 L, bit1 R, bit2 HB electrical level; bits3-5 = line requested) |
# link (1 ctrl, 2 bms) | faults | SoC
IP=${1:-192.168.4.73}; DT=${2:-0.5}; export ANDROID_SERIAL=$IP:5555
adb connect $IP:5555 >/dev/null
adb shell 'while true; do
  d=$(dumpsys android.hardware.automotive.vehicle.IVehicle/default 2>/dev/null | tr -d "\r");
  g() { echo "$d" | awk -v p="$1" "\$1==\"PropId\" && \$2==p\":\" {print \$3; exit}"; };
  v=$(g 291504647); s=$(awk -v v="${v:-0}" "BEGIN{printf \"%5.1f\", v*3.6}");
  printf "%s spd=%s km/h gear=%s raw=%s brake=%s flags=%s turn=%s beam=%s gpio=%s link=%s fault=%s soc=%s\n" \
    "$(date +%H:%M:%S)" "$s" "$(g 289408000)" "$(g 557842504)" "$(g 287310850)" "$(g 557842498)" \
    "$(g 289408016)" "$(g 289412609)" "$(g 557842540)" "$(g 557842499)" "$(g 557842496)" "$(g 291504905)";
  sleep '"$DT"'; done'
