#!/bin/bash
# Apply the out-of-tree patches listed in README.md. Run from the AOSP root.
# Patches are `git format-patch` mails; a patch counts as applied when a
# commit with its Subject exists in the project's recent history, so the
# check keeps working after later patches touch the same files. New patches
# are applied with `git am`, which restores the local commit history too.
set -e
root=$(cd "$(dirname "$0")/../../../.." && pwd)
here=$(cd "$(dirname "$0")" && pwd)
for dir in "$here"/*/; do
  proj=$(basename "$dir" | tr _ /)
  [ -d "$root/$proj" ] || { echo "skip $proj: not in tree"; continue; }
  for p in "$dir"/*.patch; do
    # Unfold the (possibly wrapped) Subject header, drop the [PATCH] tag.
    subject=$(awk '/^Subject:/{sub(/^Subject: /,""); s=$0; f=1; next} f&&/^ /{s=s $0; next} f{print s; exit}' "$p" | sed 's/^\[PATCH[^]]*\] //')
    if git -C "$root/$proj" log --format=%s -n 300 | grep -qxF "$subject"; then
      echo "already applied: $proj $(basename "$p")"
    elif git -C "$root/$proj" apply --check "$p" >/dev/null 2>&1; then
      git -C "$root/$proj" -c user.name="NEO dash" -c user.email="neo@dash" am -q "$p" \
        && echo "applied: $proj $(basename "$p")"
    else
      echo "CONFLICT: $proj $(basename "$p")" >&2; exit 1
    fi
  done
done
