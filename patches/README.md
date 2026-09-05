# Patches to AOSP projects outside this device tree

Changes the NEO dash needs in AOSP modules that are not one of our forks.
Each sub-directory is named after the project path with `/` replaced by `_`,
and holds numbered `git diff` files against the 25Q4 release the tree is on.

Apply from the tree root after `repo sync`:

    device/brcm/rpi5/patches/apply.sh

The script is idempotent (`git apply --check` first, skips patches already in).

| Project | Patch | Why |
|---|---|---|
| packages/modules/Bluetooth | 0001-a2dp-sink-bounded-audio-write | The A2DP sink wrote PCM to its AudioTrack with a blocking write while holding the sink mutex. On the dash the track's output is the earbud A2DP link, whose start needs the stack's main thread, which was itself waiting on that mutex: three-way deadlock, stack frozen for minutes (2026-09-05, see docs/FOLLOW-UPS.md). The write is now non-blocking with a 20 ms grace period and drops what does not fit. |
