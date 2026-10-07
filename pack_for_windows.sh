#!/usr/bin/env bash
# Builds a self-contained nrdemod-vsa bundle for the Windows analysis station.
#
#   ./pack_for_windows.sh [output.zip] [--waveforms <exported-dataset-dir>] \
#       [--tdc-reference <TDC-waveform-root>] [--tdc-reference-plaintext <converted-dir>]
#
# The archive contains the bench tooling plus a full copy of the nrdemod core,
# so the Windows side only needs CMake, MSVC and Python. Pass --waveforms with
# the output directory of the in-tree testDemod_ExportTdcWaveformsForStandaloneWindows
# (NR_DEMOD_REFERENCE_WAVEFORM_EXPORT) to also bundle the exported .npy/.json
# waveform+Config pairs under captures/exported/, replayable with
# tools/replay_exported_waveforms.py once unzipped.
#
# Pass --tdc-reference with the TDC waveform root (NR_DEMOD_REFERENCE_WAVEFORM_ROOT,
# e.g. /home/pengzhou/demod/waveforms) to additionally bundle the raw .wfm+.para2
# pairs for WiFi (802.11*), UWB (802.15.4z_HrpUwb), Bluetooth (Bluetooth*) and
# NB-IoT (NbIot*) under captures/tdc_reference/<family>/. These standards have no
# native nrdemod backend (legacy-only), so there is no .npy/.json export for them;
# the .wfm plays back directly in the 89600 (Inputs.Recording.RecallFile detects
# the Signal Studio format) against its own WiFi/BT/UWB/IoT personality, and the
# paired .para2 documents the recorded reference metrics to compare against.
#
# Pass --tdc-reference-plaintext with the output directory of the in-tree
# testDemod_ConvertTdcWaveformsToUnencryptedVsa89600 test (NR_DEMOD_REFERENCE_
# WAVEFORM_CONVERT_OUTPUT) to bundle unencrypted VSA_89600-format .txt copies
# under captures/tdc_reference_plaintext/<family>/ instead of/alongside the raw
# encrypted .wfm. Use this when the Windows/Linux side has no Signal Studio
# decrypt license: tools/vsa.py's load_iq() reads this plaintext format directly.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
core="$here/../nrdemod_standalone"
out="$here/../nrdemod-vsa.zip"
waveforms=""
tdcReference=""
tdcReferencePlaintext=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --waveforms) waveforms="$2"; shift 2 ;;
    --tdc-reference) tdcReference="$2"; shift 2 ;;
    --tdc-reference-plaintext) tdcReferencePlaintext="$2"; shift 2 ;;
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
if [[ -n "$tdcReference" && ! -d "$tdcReference" ]]; then
  echo "error: --tdc-reference directory not found: $tdcReference" >&2
  exit 1
fi
if [[ -n "$tdcReferencePlaintext" && ! -d "$tdcReferencePlaintext" ]]; then
  echo "error: --tdc-reference-plaintext directory not found: $tdcReferencePlaintext" >&2
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

if [[ -n "$tdcReference" ]]; then
  tdcCount=0
  for family in "$tdcReference"/802.11* "$tdcReference"/802.15.4z_HrpUwb "$tdcReference"/Bluetooth* "$tdcReference"/NbIot*; do
    [[ -d "$family" ]] || continue
    name="$(basename "$family")"
    destDir="$root/captures/tdc_reference/$name"
    for wfm in "$family"/*.wfm; do
      [[ -f "$wfm" ]] || continue
      stem="${wfm%.wfm}"
      para="$stem.para2"
      [[ -f "$para" ]] || continue # unpaired captures have no recorded reference metrics, skip
      mkdir -p "$destDir"
      cp "$wfm" "$para" "$destDir/"
      tdcCount=$((tdcCount + 1))
    done
  done
  echo "tdc reference: $tdcCount paired WiFi/UWB/Bluetooth/NbIot waveforms bundled under captures/tdc_reference"
fi

if [[ -n "$tdcReferencePlaintext" ]]; then
  mkdir -p "$root/captures/tdc_reference_plaintext"
  rsync -a "$tdcReferencePlaintext/" "$root/captures/tdc_reference_plaintext/"
  echo "tdc reference plaintext: $(find "$root/captures/tdc_reference_plaintext" -name '*.txt' | wc -l) unencrypted VSA_89600 captures bundled"
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
if [[ -n "$tdcReference" ]]; then
  echo "  5. load a .wfm under captures\\tdc_reference\\<family> into the 89600 (Inputs > Recording"
  echo "     > Recall) with the matching WiFi/BT/UWB/IoT personality and compare against the"
  echo "     RESULT lines in the paired .para2 (no nrdemod replay: these standards are legacy-only)"
fi
if [[ -n "$tdcReferencePlaintext" ]]; then
  echo "  6. no Signal Studio license needed: tools\\vsa.py's load_iq() reads captures\\"
  echo "     tdc_reference_plaintext\\<family>\\*.txt directly (unencrypted VSA_89600 format)"
fi
