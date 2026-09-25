#!/usr/bin/env bash
# Build the release firmware the host pushes to the front panel over serial (OTA) and that
# `cdmania flash` writes on a first cable flash: firmware/release/{cdplayer.bin,cdplayer-full.bin,firmware.json}.
# Runs on `npm pack` / `npm publish` (prepack) so a published package always carries an image
# matching its version. Needs arduino-cli with the esp32 core and the sketch libraries.
set -euo pipefail
cd "$(dirname "$0")/.."
export PATH="$HOME/.local/bin:$PATH"
command -v arduino-cli >/dev/null || { echo "arduino-cli is required to build the firmware image" >&2; exit 1; }

VERSION=$(node -p "require('./package.json').version")
OUT=firmware/release
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

rm -rf "$OUT" && mkdir -p "$OUT"
arduino-cli compile -b esp32:esp32:esp32 \
  --build-property "compiler.cpp.extra_flags=-DFW_VERSION=$VERSION" \
  --output-dir "$TMP/out" firmware/CDPlayer >/dev/null
cp "$TMP/out/CDPlayer.ino.bin" "$OUT/cdplayer.bin"
cp "$TMP/out/CDPlayer.ino.merged.bin" "$OUT/cdplayer-full.bin"

SIZE=$(wc -c <"$OUT/cdplayer.bin" | tr -d ' ')
MD5=$( (md5sum "$OUT/cdplayer.bin" 2>/dev/null || md5 -r "$OUT/cdplayer.bin") | cut -d' ' -f1)
printf '{"version":"%s","size":%s,"md5":"%s"}\n' "$VERSION" "$SIZE" "$MD5" >"$OUT/firmware.json"
echo "firmware $VERSION: $SIZE bytes, md5 $MD5"
