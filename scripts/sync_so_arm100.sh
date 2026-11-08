#!/usr/bin/env bash
# Re-sync the vendored SO-101 files in third_party/SO-ARM100 from upstream.
#
# Usage: scripts/sync_so_arm100.sh [git-ref]   (default: main)
#
# Copies only the SO-101-relevant paths listed below, then records the
# resolved commit in third_party/SO-ARM100/UPSTREAM_COMMIT. Review the diff
# before committing: upstream changes can move or rename parts.
set -euo pipefail

REF="${1:-main}"
REPO_URL="https://github.com/TheRobotStudio/SO-ARM100.git"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="$ROOT/third_party/SO-ARM100"

PATHS=(
  /LICENSE /README.md /3DPRINT.md /CHANGELOG.md /CITATION.cff
  /STEP/SO101/ /STL/SO101/ /STL/Gauges/
  /Simulation/README.md /Simulation/SO101/
  /Optional/ /Software/ /media/
)

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

git clone -q --filter=blob:none --no-checkout "$REPO_URL" "$TMP/so-arm100"
git -C "$TMP/so-arm100" sparse-checkout init --no-cone
git -C "$TMP/so-arm100" sparse-checkout set "${PATHS[@]}"
git -C "$TMP/so-arm100" checkout -q "$REF"
SHA="$(git -C "$TMP/so-arm100" rev-parse HEAD)"

mkdir -p "$DEST"
# --delete keeps the vendored copy an exact mirror; our own notes are protected.
rsync -a --delete \
  --exclude .git \
  --exclude UPSTREAM.md --exclude UPSTREAM_COMMIT \
  "$TMP/so-arm100"/ "$DEST"/

echo "$SHA" > "$DEST/UPSTREAM_COMMIT"
echo "Synced third_party/SO-ARM100 to $SHA"
