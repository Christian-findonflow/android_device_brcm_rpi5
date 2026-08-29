#!/system/bin/sh
# Simulator vcan bring-up (see init.moto_cf.rc). vcan.ko lives in the
# virtual-device kernel's system_dlkm modules.
# insmod does not resolve dependencies: raw CAN sockets need can.ko and
# can-raw.ko in addition to vcan.ko, in this order.
for m in can.ko can-raw.ko vcan.ko; do
    ko="$(find /system_dlkm/lib/modules /vendor_dlkm/lib/modules /vendor/lib/modules -name "$m" 2>/dev/null | head -1)"
    [ -n "$ko" ] && insmod "$ko" 2>/dev/null
done
ip link add dev vcan0 type vcan 2>/dev/null
ip link set up vcan0
