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
| packages/modules/Bluetooth | 0002-avrcp-scope-role-switch-close-to-peer | With a phone streaming into the sink and earbuds connecting as our sink, the earbuds' role switch closed the phone's AVRCP controller channel (AVRC_UpdateCcb closed every legacy channel, not just the earbuds'), the controller service dropped the phone as active device and the sink session ended ("Bluetooth audio disconnected"). Adds AVCT_GetPeerAddr and skips other peers' channels. |
| packages/modules/Bluetooth | 0003-bta-av-source-data-only-to-source-streams | With both profiles on, the phone's stream into our sink is "started" too and received the source data-ready event: the dash pushed the encoded earbud stream to the phone at full rate while the earbuds starved and dropped their link (radio log: 43 media packets/s to the phone, ~1/s to the earbuds). Guards bta_av_data_path and bta_av_dup_audio_buf on the local SEP direction. |
| packages/modules/Bluetooth | 0004-avdtp-only-transmit-streams-take-the-current-stream-slot | AVDTP lets one "current" stream transmit when several are streaming; it counted the phone's incoming stream, so the earbud stream never got the slot and every media packet was freed ("ignore AVDT_SCB_API_WRITE_REQ_EVT" 43x/s), the earbuds heard silence and sent SUSPEND after 3 s. Count only streams where our local SEP is the source. |
