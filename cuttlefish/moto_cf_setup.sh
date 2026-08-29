#!/system/bin/sh
# Simulator vcan bring-up (see init.moto_cf.rc).
#
# insmod does not resolve dependencies: raw CAN sockets need can.ko and
# can-raw.ko in addition to vcan.ko, in this order. The module tree
# (system_dlkm) is not always ready when post-fs-data fires, so retry for up
# to 60s instead of failing silently - a partial load here left the HAL
# without PF_CAN support for its whole retry window.
i=0
while [ $i -lt 60 ]; do
    ok=1
    for m in can.ko can-raw.ko vcan.ko; do
        ko="$(find /system_dlkm/lib/modules /vendor_dlkm/lib/modules /vendor/lib/modules -name "$m" 2>/dev/null | head -1)"
        if [ -n "$ko" ]; then
            insmod "$ko" 2>/dev/null
        else
            ok=0
        fi
    done
    ip link add dev vcan0 type vcan 2>/dev/null
    ip link set up vcan0 2>/dev/null
    # Done when the interface exists and the raw-CAN protocol is registered
    if [ -d /sys/class/net/vcan0 ] && [ -e /proc/net/can ]; then
        exit 0
    fi
    sleep 1
    i=$((i + 1))
done
exit 1
