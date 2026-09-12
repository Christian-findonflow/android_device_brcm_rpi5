#!/bin/bash
# Push the built CarLauncher to the bench Pi as a system app (no adb install:
# an adb-installed update clamps intent-filter priorities and trips
# CarService's vendor-service starter). Usage: bench_launcher_push.sh [ip]
IP=${1:-192.168.4.73}; export ANDROID_SERIAL=$IP:5555
OUT=/home/christian/aosp16-rpi/out/target/product/rpi5
APK=$OUT/system/priv-app/CarLauncher/CarLauncher.apk
[ -f "$APK" ] || { echo "no $APK - build CarLauncher for rpi5 first"; exit 1; }
adb connect $IP:5555 >/dev/null; adb root >/dev/null 2>&1; sleep 4; adb connect $IP:5555 >/dev/null; adb wait-for-device
adb shell 'mount -o remount,rw / && echo root-rw' || exit 1
adb push "$APK" /system/priv-app/CarLauncher/CarLauncher.apk | tail -1
adb shell 'chmod 644 /system/priv-app/CarLauncher/CarLauncher.apk; chcon u:object_r:system_file:s0 /system/priv-app/CarLauncher/CarLauncher.apk; pm uninstall com.android.car.carlauncher >/dev/null 2>&1; sync'
echo "rebooting $IP"; adb reboot; sleep 30
for i in $(seq 1 30); do adb wait-for-device >/dev/null 2>&1; b=$(adb shell getprop sys.boot_completed 2>/dev/null | tr -d '\r'); [ "$b" = "1" ] && break; sleep 5; done
echo "booted after ${i}x5s"; sleep 20
# The cluster strip has come up black on the first boot after a replacement
# (twice, 2026-09-11 and 09-12): a HOME intent starts it.
adb shell 'am start --user 10 -a android.intent.action.MAIN -c android.intent.category.HOME' >/dev/null 2>&1
sleep 3; adb shell 'dumpsys activity activities 2>/dev/null | grep -c "carlauncher/.CarLauncher"' | tr -d '\r'
