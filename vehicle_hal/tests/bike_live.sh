#!/bin/bash
# Live snapshot of the bike over adb: CAN link, decoded values, indicator inputs, HAL log.
# Usage: bike_live.sh [ip] [serial]   (default 192.168.4.73, adb over TCP port 5555)
export PATH=$PATH:/home/christian/aosp16-rpi/out/host/linux-x86/bin
IP="${1:-192.168.4.73}"; SERIAL="${2:-$IP:5555}"; B="timeout 15 adb -s $SERIAL"
timeout 10 adb connect $SERIAL >/dev/null 2>&1; sleep 1
if ! timeout 10 adb -s $SERIAL get-state >/dev/null 2>&1; then echo "bike $SERIAL not reachable over adb"; exit 1; fi
$B root >/dev/null 2>&1; sleep 2; timeout 10 adb connect $SERIAL >/dev/null 2>&1
prop() { $B shell "dumpsys android.hardware.automotive.vehicle.IVehicle/default 2>/dev/null" | tr -d '\r' | grep -E "PropId $1:" | head -1 | sed 's/.*: //'; }
echo "== bike $IP  $(date +%H:%M:%S)  uptime $($B shell 'cat /proc/uptime' | cut -d' ' -f1 | tr -d '\r')s =="
echo "-- CAN interfaces"
$B shell "ip -d link show can1 2>&1 | head -3; ip -d link show can0 2>&1 | head -1" | tr -d '\r' | sed 's/^/   /' | cut -c1-140
R1=$($B shell "cat /sys/class/net/can1/statistics/rx_packets 2>/dev/null" | tr -d '\r'); sleep 2
R2=$($B shell "cat /sys/class/net/can1/statistics/rx_packets 2>/dev/null" | tr -d '\r')
E=$($B shell "cat /sys/class/net/can1/statistics/rx_errors 2>/dev/null; cat /sys/class/net/can1/statistics/tx_errors 2>/dev/null" | tr -d '\r' | tr '\n' '/')
echo "   can1 rx frames: ${R1:-?} -> ${R2:-?} in 2 s  (rx/tx errors ${E})"
DUMP=$($B shell "dumpsys android.hardware.automotive.vehicle.IVehicle/default 2>/dev/null" | tr -d '\r')
g() { echo "$DUMP" | grep -E "PropId $1:" | head -1 | sed 's/.*: //'; }
echo "-- decoded (VHAL)"
echo "   link bits $(g 557842499) (1=ctrl 2=bms)   speed $(g 291504647) m/s   rpm $(g 291504901)   gear $(g 289408000) (4=P 8=D)   raw gear byte $(g 557842504)"
echo "   pack $(g 559939585) V   $(g 559939586) A   SoC $(g 291504905) %   charging $(g 557842500)   faults $(g 557842496)   status flags $(g 557842498)"
echo "   ctrl temp $(g 291504897) C   motor temp $(g 291504900) C   throttle $(g 559939587) %   raw GPIO $(g 557842540) (bit0 L bit1 R bit2 HB; 8/16/32 configured)"
echo "   IMU status $(g 557842535)   capture $(g 557842502)"
echo "-- HAL log (last 8 relevant lines)"
$B logcat -d -s MotorcycleVehicleHardware 2>/dev/null | tr -d '\r' | grep -iE "link|controller|bms|frame|error|fail|GPIO|Turn|beam|capture|IMU" | tail -8 | cut -c1-150 | sed 's/^/   /'
echo "-- health: tombstones $($B shell 'ls /data/tombstones 2>/dev/null | wc -l' | tr -d '\r ')   vhal $($B shell 'getprop init.svc.vendor.vehicle-hal-motorcycle' | tr -d '\r')"
