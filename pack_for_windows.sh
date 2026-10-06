#!/usr/bin/env bash
# Builds a self-contained nrdemod-vsa bundle for the Windows analysis station.
#
#   ./pack_for_windows.sh [output.zip] [--waveforms <exported-dataset-dir>]
#
# The archive contains the bench tooling plus a full copy of the nrdemod core,
# so the Windows side only needs CMake, MSVC and Python. Pass --waveforms with
# the output directory of the in-tree testDemod_ExportTdcWaveformsForStandaloneWindows
# (NR_DEMOD_REFERENCE_WAVEFORM_EXPORT) to also bundle the exported .npy/.json
# waveform+Config pairs under captures/exported/, replayable with
# tools/replay_exported_waveforms.py once unzipped.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
core="$here/../nrdemod_standalone"
out="$here/../nrdemod-vsa.zip"
waveforms=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --waveforms) waveforms="$2"; shift 2 ;;
    *) out="$1"; shift ;;
  esac
done

if [[ ! -f "$core/CMakeLists.txt" ]]; then
  echo "error: nrdemod_standalone not found next to this workspace" >&2
  exit 1
fi
if [[ -n "$waveforms" && ! -d "$waveforms" ]]; then
  echo "error: --waveforms directory not found: $waveforms" >&2
  exit 1
fi

stage="$(mktemp -d)"
trap 'rm -rf "$stage"' EXIT
root="$stage/nrdemod-vsa"
mkdir -p "$root"

copy() {
  rsync -a --exclude build --exclude .venv --exclude __pycache__ \
        --exclude '*.egg-info' --exclude '_lib' --exclude '*.so' \
        --exclude '*.dll' --exclude '*.pyd' --exclude '*.dylib' \
        "${@:3}" "$1" "$2"
}

copy "$here/" "$root/" --exclude nrdemod
mkdir -p "$root/nrdemod"
copy "$core/" "$root/nrdemod/"

rm -rf "$root/reports" "$root/captures"
mkdir -p "$root/reports" "$root/captures"
touch "$root/reports/.keep" "$root/captures/.keep"
rm -f "$root/pack_for_windows.sh"

if [[ -n "$waveforms" ]]; then
  mkdir -p "$root/captures/exported"
  rsync -a "$waveforms/" "$root/captures/exported/"
  echo "waveforms: $(find "$root/captures/exported" -name '*.npy' | wc -l) captures bundled"
fi

unix2dos_safe() { sed -i 's/\r\?$/\r/' "$1"; }
unix2dos_safe "$root/build.bat"

( cd "$stage" && zip -qr "$out" nrdemod-vsa )
echo "bundle: $out"
echo
echo "On the Windows station:"
echo "  1. unzip to e.g. C:\\nrdemod-vsa"
echo "  2. run build.bat"
echo "  3. open nrdemod-vsa.code-workspace in VS Code"
if [[ -n "$waveforms" ]]; then
  echo "  4. .venv\\Scripts\\python.exe tools\\replay_exported_waveforms.py captures\\exported"
fi
