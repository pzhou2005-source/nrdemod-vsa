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

echo "done - rebuild with: cmake -S '$here' -B '$here/build' && cmake --build '$here/build' -j"
