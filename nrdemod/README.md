# nrdemod — standalone 5G NR demodulator

Portable extract of the in-tree `nrdemod` DSP core
(`vobs/zenith/workspace/services/Demod/Backend/src/nr/NrOfdmCore.*`) with a flat
C ABI and a Python wrapper, so the exact same demodulation can run on a Windows
analysis PC and be correlated against a Keysight 89600 VSA.

No MKL, no libdemod, no UNO — just C++11 and the STL. The Python layer uses
`ctypes` + `numpy`, so there is nothing to compile on the Python side and no
pybind11 / Python headers are needed.

```
include/nr/NrOfdmCore.hpp     DSP core header   (synced from the bazel tree)
include/nrdemod/nrdemod_c.h   flat C ABI
src/NrOfdmCore.cpp            DSP core          (synced from the bazel tree)
src/nrdemod_c.cpp             C ABI implementation
python/nrdemod/               ctypes wrapper + VSA helpers
examples/                     self test and VSA correlation script
tests/NrOfdmCoreTest.cpp      C++ unit test     (synced from the bazel tree)
sync_from_tree.sh             re-copy the core after upstream changes
```

## Build

### Linux

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

### Windows (MSVC)

```bat
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

### Windows (MinGW / MSYS2)

```bash
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

The shared library lands in `python/nrdemod/_lib/` (`nrdemod.dll` on Windows,
`nrdemod.so` on Linux), where the Python package picks it up automatically.
Override with the `NRDEMOD_LIBRARY` environment variable if needed.

## Python

```bash
pip install numpy          # scipy for .mat recordings, matplotlib for --plot
pip install -e python      # or: set PYTHONPATH=...\nrdemod_standalone\python
python examples/selftest.py
```

```python
import numpy as np, nrdemod

cfg = nrdemod.Config(
    numerology=1,                       # SCS = 30 kHz
    frequencyRange=nrdemod.FR1,
    channelBandwidthMHz=100,
    sampleRate=122.88e6,
    modulation=nrdemod.QAM64,           # or nrdemod.AUTO
)

iq = np.fromfile("capture.bin", dtype=np.complex64)
res = nrdemod.demodulate(iq, cfg)

print(res.rmsEvmPercent, res.frequencyErrorHz, res.modulation_name)
print(res.constellation.shape, res.evmPerSubcarrier.shape)
```

`Result` exposes every field of the C++ `nrdemod::Result`: scalars as plain
attributes, vectors as numpy arrays (`constellation`, `reference`,
`errorVector`, `channelEstimate`, `spectrumDb`, `evmPerSymbol`,
`evmPerSubcarrier`, `evmPerSlot`, `evmPerResourceBlock`,
`channelImpulseResponseDb`, ...). `res.as_dict(include_vectors=True)` dumps
them all.

## Correlating with the 89600 VSA

1. Record the burst in the 89600 and save it (`File > Save > Recording`) as
   `.mat` or `.csv`.
2. Note the VSA's 5G NR EVM reading for the same capture.
3. Run:

```bat
python examples\vsa_correlate.py capture.mat --bw 100 --mu 1 --mod 64QAM ^
    --vsa-evm 1.42 --plot
```

The script prints the nrdemod metrics, the EVM delta against the VSA, and can
dump all result vectors to an `.npz` (`--dump out.npz`).

To rule out analyser setup differences, `--export ideal.csv` writes an ideal
synthetic burst in an 89600-importable time-record format; loading that into the
VSA should give ~0 % EVM on both sides.

`nrdemod.vsa.load_iq()` handles `.mat` (scipy), `.csv`/`.txt` (header skipped,
`I,Q` or `t,I,Q`), `.npy` and raw interleaved `.bin`.

## Keeping in sync with the bazel tree

```bash
./sync_from_tree.sh            # copies NrOfdmCore.* and the unit test
```

Only `NrOfdmCore.*` is shared. `NrDemodulation`/`DemodBackend` stay in the main
repository because they depend on the libdemod ABI and MKL.
