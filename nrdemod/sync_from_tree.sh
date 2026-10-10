#!/usr/bin/env bash
# Refreshes the DSP core from the in-tree bazel sources. The standalone project
# intentionally keeps a copy so it can be zipped and built on a Windows host.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
tree="${1:-$here/../vobs/zenith/workspace/services/Demod/Backend}"

if [[ ! -f "$tree/src/nr/NrOfdmCore.cpp" ]]; then
  echo "error: $tree does not look like Demod/Backend" >&2
  exit 1
fi

cp -v "$tree/src/nr/NrOfdmCore.hpp"      "$here/include/nr/NrOfdmCore.hpp"
cp -v "$tree/src/nr/NrOfdmCore.cpp"      "$here/src/NrOfdmCore.cpp"
cp -v "$tree/test/NrOfdmCoreTest.cpp"    "$here/tests/NrOfdmCoreTest.cpp"

# WiFi family / Bluetooth LE / UWB cores (behind demod_c.h, see demod.py).
declare -A families=(
  [wifi/WifiOfdmCore]=WifiOfdmCore
  [wifi11n/Wifi11nOfdmCore]=Wifi11nOfdmCore
  [wifi11ac/Wifi11acOfdmCore]=Wifi11acOfdmCore
  [wifi11ax/Wifi11axOfdmCore]=Wifi11axOfdmCore
  [wifi11ad/Wifi11adOfdmCore]=Wifi11adOfdmCore
  [wifi11b/Wifi11bOfdmCore]=Wifi11bOfdmCore
  [wifi11be/Wifi11beOfdmCore]=Wifi11beOfdmCore
  [bt/BtGfskCore]=BtGfskCore
  [uwb/UwbHrpCore]=UwbHrpCore
)
for relPath in "${!families[@]}"; do
  name="${families[$relPath]}"
  destDir="$here/include/$(dirname "$relPath")"
  mkdir -p "$destDir"
  cp -v "$tree/src/$relPath.hpp" "$destDir/$name.hpp"
  cp -v "$tree/src/$relPath.cpp" "$here/src/$name.cpp"
  cp -v "$tree/test/${name}Test.cpp" "$here/tests/${name}Test.cpp"
done

echo "done - rebuild with: cmake -S '$here' -B '$here/build' && cmake --build '$here/build' -j"
