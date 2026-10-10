# nrdemod — standalone 5G NR / WiFi / Bluetooth LE / UWB demodulator

Portable extract of the in-tree native demodulator cores
(`vobs/zenith/workspace/services/Demod/Backend/src/...`) with a flat C ABI and
a Python wrapper, so the exact same demodulation can run on a Windows analysis
PC and be correlated against a Keysight 89600 VSA.

No MKL, no libdemod, no UNO — just C++11 and the STL. The Python layer uses
`ctypes` + `numpy`, so there is nothing to compile on the Python side and no
pybind11 / Python headers are needed.

Two independent shared libraries are built:

- `nrdemod` — 5G NR CP-OFDM / DFT-s-OFDM (`nrdemod_c.h`, module `nrdemod`).
- `demod` — WiFi (802.11a/b/n/ac/ax/ad/be), Bluetooth LE (GFSK) and UWB
  (802.15.4z HRP) (`demod_c.h`, module `nrdemod.demod`).

```
include/nr/NrOfdmCore.hpp        5G NR DSP core header      (synced from the bazel tree)
include/nrdemod/nrdemod_c.h      5G NR flat C ABI
include/wifi/WifiOfdmCore.hpp    802.11a DSP core header    (synced from the bazel tree)
include/wifi11n/...              802.11n (HT)               (synced from the bazel tree)
include/wifi11ac/...             802.11ac (VHT)             (synced from the bazel tree)
include/wifi11ax/...             802.11ax (HE)              (synced from the bazel tree)
include/wifi11ad/...             802.11ad (DMG/WiGig)       (synced from the bazel tree)
include/wifi11b/...              802.11b (DSSS/CCK)         (synced from the bazel tree)
include/wifi11be/...             802.11be (EHT)             (synced from the bazel tree)
include/bt/BtGfskCore.hpp        Bluetooth LE1M/LE2M GFSK   (synced from the bazel tree)
include/uwb/UwbHrpCore.hpp       802.15.4z HRP-UWB          (synced from the bazel tree)
include/nrdemod/demod_c.h        WiFi/BT/UWB flat C ABI
src/NrOfdmCore.cpp               5G NR DSP core             (synced from the bazel tree)
src/nrdemod_c.cpp                5G NR C ABI implementation
src/Wifi*.cpp, BtGfskCore.cpp,
  UwbHrpCore.cpp                 WiFi/BT/UWB DSP cores      (synced from the bazel tree)
src/demod_c.cpp                  WiFi/BT/UWB C ABI implementation
python/nrdemod/                  ctypes wrappers + VSA helpers (nrdemod.py / demod.py)
examples/                        self test and VSA correlation script
tests/*CoreTest.cpp              C++ unit tests             (synced from the bazel tree)
sync_from_tree.sh                re-copy every core after upstream changes
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

Both shared libraries land in `python/nrdemod/_lib/` (`nrdemod.dll`/`demod.dll`
on Windows, `nrdemod.so`/`demod.so` on Linux), where the Python package picks
them up automatically. Override with the `NRDEMOD_LIBRARY` / `DEMOD_LIBRARY`
environment variables if needed.

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

### WiFi / Bluetooth LE / UWB

```python
import numpy as np
from nrdemod import demod

iq = np.fromfile("wifi_capture.bin", dtype=np.complex64)

cfg = demod.Config(sampleRate=20e6)
res = demod.demodulate(demod.WIFI_A, iq, cfg)       # 802.11a
print(res.syncFound, res.syncRho, res.rmsEvmPercent)

res_ac = demod.demodulate(demod.WIFI_AC, iq, cfg)   # 802.11ac (20/40/80/160 MHz sync)
res_be = demod.demodulate(demod.WIFI_BE, iq, cfg)   # 802.11be (burst/sync/CFO only)

bt_cfg = demod.Config(sampleRate=8e6, phy=1, accessAddress=0x71764129)
res_bt = demod.demodulate(demod.BT_LE, iq, bt_cfg)  # Bluetooth LE1M
print(res_bt.syncWordFound, res_bt.deltaFAvgHz)

uwb_cfg = demod.Config(sampleRate=999.36e6)
res_uwb = demod.demodulate(demod.UWB_HRP, iq, uwb_cfg)
```

Family constants: `WIFI_A`, `WIFI_N`, `WIFI_AC`, `WIFI_AX`, `WIFI_AD`, `WIFI_B`,
`WIFI_BE`, `BT_LE`, `UWB_HRP`. Each core's scope is documented in its header
under `include/<family>/`; several (802.11ax/ad/be) are deliberately
burst/sync/CFO-only — see the header doc comments for exactly what is and
isn't decoded.

## Correlating with the 89600 VSA
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
./sync_from_tree.sh            # copies every core (nr, wifi*, bt, uwb) + their unit tests
```

Only the pure-DSP cores (`NrOfdmCore.*`, `Wifi*OfdmCore.*`, `BtGfskCore.*`,
`UwbHrpCore.*`) are shared. The `*Demodulation`/`*DemodBackend` adapter and
`DemodBackendRegistry` classes stay in the main repository because they depend
on the libdemod ABI, UNO and MKL.
