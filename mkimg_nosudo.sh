#!/bin/bash
# Same layout as device/brcm/rpi5/mkimg.sh (KonstaKANG), built without root:
# (kept in the tree since 2026-09-11 - the scratchpad copy evaporated once)
# fdisk on the file, dd at partition offsets, mke2fs with -E offset.
set -euo pipefail
OUT=/home/christian/aosp16-rpi/out/target/product/rpi5
# Usage: mkimg_nosudo.sh [tag]   e.g. v23 -> ...-rpi5_motorcycle-v23.img
TAG=${1:+-$1}
IMG=$OUT/RaspberryVanillaAOSP16-$(date +%Y%m%d)-rpi5_motorcycle${TAG}.img
IMGSIZE=15360000000
BOOT_MB=128; SYSTEM_MB=3072; VENDOR_MB=384; METADATA_MB=16
EXT_MB=$((SYSTEM_MB+VENDOR_MB+METADATA_MB+4))
for p in boot system vendor; do [ -f $OUT/$p.img ] || { echo "missing $p.img"; exit 1; }; done
[ -f "$IMG" ] && { echo "$IMG already exists"; exit 1; }
echo "creating $IMG"
fallocate -l $IMGSIZE "$IMG"
(
echo o
echo n; echo p; echo 1; echo; echo +${BOOT_MB}M
echo n; echo e; echo 2; echo; echo +${EXT_MB}M
echo n; echo l; echo; echo +${SYSTEM_MB}M
echo n; echo l; echo; echo +${VENDOR_MB}M
echo n; echo l; echo; echo +${METADATA_MB}M
echo n; echo p; echo 3; echo; echo
echo t; echo 1; echo c
echo a; echo 1
echo w
) | fdisk "$IMG" > /dev/null
sfdisk -d "$IMG"
start() { sfdisk -d "$IMG" | grep "^${IMG}${1} " | sed 's/.*start= *\([0-9]*\),.*/\1/'; }
size()  { sfdisk -d "$IMG" | grep "^${IMG}${1} " | sed 's/.*size= *\([0-9]*\),.*/\1/'; }
for n in 1 3 5 6 7; do echo "p$n start=$(start $n) size=$(size $n) sectors"; done
echo "copying boot";   dd if=$OUT/boot.img   of="$IMG" bs=512 seek=$(start 1) conv=notrunc,fsync status=none
echo "copying system"; dd if=$OUT/system.img of="$IMG" bs=512 seek=$(start 5) conv=notrunc,fsync status=none
echo "copying vendor"; dd if=$OUT/vendor.img of="$IMG" bs=512 seek=$(start 6) conv=notrunc,fsync status=none
echo "formatting metadata"; mke2fs -q -F -t ext4 -I 512 -L metadata -E offset=$(( $(start 7) * 512 )) "$IMG" $(( $(size 7) / 2 ))k
echo "formatting userdata"; mke2fs -q -F -t ext4 -I 512 -L userdata -E offset=$(( $(start 3) * 512 )),lazy_itable_init=1 "$IMG" $(( $(size 3) / 2 ))k
sync
echo "verifying partition contents against the source images"
for pair in "1 boot" "5 system" "6 vendor"; do set -- $pair
  n=$(stat -c %s $OUT/$2.img)
  if cmp -n $n <(dd if="$IMG" bs=512 skip=$(start $1) status=none) $OUT/$2.img; then echo "  p$1 = $2.img OK ($n bytes)"; else echo "  p$1 MISMATCH"; exit 1; fi
done
dumpe2fs -h "$IMG?offset=$(( $(start 3) * 512 ))" 2>/dev/null | grep -E "volume name|Block count" | tr '\n' ';'; echo
echo "DONE $IMG"
