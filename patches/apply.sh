#!/bin/bash
# Apply the out-of-tree patches listed in README.md. Run from the AOSP root.
set -e
root=$(cd "$(dirname "$0")/../../../.." && pwd)
here=$(cd "$(dirname "$0")" && pwd)
for dir in "$here"/*/; do
  proj=$(basename "$dir" | tr _ /)
  [ -d "$root/$proj" ] || { echo "skip $proj: not in tree"; continue; }
  for p in "$dir"/*.patch; do
    if git -C "$root/$proj" apply --check --reverse "$p" >/dev/null 2>&1; then
      echo "already applied: $proj $(basename "$p")"
    elif git -C "$root/$proj" apply --check "$p" >/dev/null 2>&1; then
      git -C "$root/$proj" apply "$p" && echo "applied: $proj $(basename "$p")"
    else
      echo "CONFLICT: $proj $(basename "$p")" >&2; exit 1
    fi
  done
done
