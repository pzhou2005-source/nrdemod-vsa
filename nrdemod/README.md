# nrdemod — standalone 5G NR / LTE / WiFi / Bluetooth / UWB demodulator

Portable extract of the in-tree native demodulator cores
(`vobs/zenith/workspace/services/Demod/Backend/src/...`) with a flat C ABI and
a Python wrapper, so the exact same demodulation can run on a Windows analysis
PC and be correlated against a Keysight 89600 VSA.

No MKL, no libdemod, no UNO — just C++11 and the STL. The Python layer uses
`ctypes` + `numpy`, so there is nothing to compile on the Python side and no
pybind11 / Python headers are needed.

Two independent shared libraries are built:

- `nrdemod` — 5G NR CP-OFDM / DFT-s-OFDM (`nrdemod_c.h`, module `nrdemod`).
  LTE/eMTC downlink+uplink reuse this same CP-OFDM/SC-FDMA core with an
  explicit `numResourceBlocks` (see the in-tree `lte/LteDemodulation.cpp` for
  the exact config this maps to); there is no separate standalone LTE module
  since the core itself already covers it. NB-IoT has dedicated
  `nbiot_config` / `demodulate_nbiot` entry points around that core, including
  uplink DFT-s-OFDM pilot exclusion and downlink NPSS frame synchronization.
- `demod` — WiFi (802.11a/b/n/ac/ax/ad/be), Bluetooth (LE1M/LE2M GFSK with full
  sync-word decode; classic BDR, classic EDR's leading GFSK access-code+header,
  and LE Coded S=8 - all three burst/frequency-metrics only) and UWB
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
include/bt/BtGfskCore.hpp        Bluetooth LE1M/LE2M + BDR/EDR/LE-Coded GFSK (synced from the bazel tree)
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

NB-IoT uses numerology 0, 15 kHz SCS and one resource block (12 subcarriers).
Use the platform-equivalent defaults rather than a generic NR configuration:

```python
res = nrdemod.demodulate_nbiot(iq, 1.92e6, uplink=True)
downlink = nrdemod.demodulate_nbiot(iq_dl, fs)
frame = nrdemod.demodulate_nbiot(iq_frame, fs, downlink_frame=True, maxSymbols=28)
```

Uplink defaults enable transform precoding, a 7500 Hz known carrier offset,
continuous input, 14-symbol timing search, guarded CP CFO estimation and
DC removal after CFO correction. Two passes align and exclude NPUSCH pilot
symbol 3 of each 7-symbol slot. This supports the verified 12-subcarrier
captures, not arbitrary single-tone or 3.75 kHz allocations. Downlink frame
mode needs 14..140 symbols in complete 14-symbol subframes, searches a bounded
input prefix for NPSS, and excludes/rotates reference resources for EVM.

Both C libraries and Python bindings now use **ABI 2**. Rebuild both shared
libraries together; old bindings/libraries are rejected, not layout-compatible.
NR settings now expose `cpCfoGuardFraction`, `removeIqOffsetAfterCfo`,
`ssbSearch`, DM-RS settings, `decisionRotations` and `excludedResources`.
In C, rotations are interleaved double real/imaginary pairs and resource masks
are int booleans; counts are complex-value and resource counts respectively.

CTest includes `NrApiTest` when Python and NumPy are available. To also replay
all 14 external full uplink captures at zero and +/-2 kHz CFO:

```bash
PYTHONPATH=python NRDEMOD_NBIOT_UL_CAPTURE_DIR=/path/to/NbIotUl python tests/NrApiTest.py -v
```

The test data are not bundled. These are offline regressions, not a fresh
Windows VSA or RF acceptance run. `tools/native_correlate.py` in the VSA bench
uses the same NB API and retains its existing VSA comparison gates.

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

# phy=3 selects classic BDR (1 Msym/s GFSK): burst + frequency metrics only,
# syncWordFound is always False (the LAP-derived sync word is not decoded).
bdr_cfg = demod.Config(sampleRate=32e6, phy=3, modulationIndex=0.3)
res_bdr = demod.demodulate(demod.BT_LE, iq, bdr_cfg)

# phy=4 selects classic EDR (burst/frequency metrics over the packet's leading
# GFSK access-code+header only, 126 symbols @ 1 Msym/s - the pi/4-DQPSK/8-DPSK
# payload that follows is not demodulated). phy=5 selects LE Coded S=8, which
# is physically still 1 Msym/s GFSK on air (same code path as LE1M, just FEC
# pattern-mapped bits) - both have syncWordFound always False.
edr_cfg = demod.Config(sampleRate=10e6, phy=4, modulationIndex=0.3)
res_edr = demod.demodulate(demod.BT_LE, iq, edr_cfg)
le_coded_cfg = demod.Config(sampleRate=10e6, phy=5)
res_le_coded = demod.demodulate(demod.BT_LE, iq, le_coded_cfg)

uwb_cfg = demod.Config(sampleRate=999.36e6)
res_uwb = demod.demodulate(demod.UWB_HRP, iq, uwb_cfg)
```

For 802.15.4ab draft EHPRF preamble synchronization and CFO estimation, use
`demod.Config(sampleRate=fs, phyMode=demod.UWB_EHPRF, syncLength=64)`.
The native mode values are `UWB_BPRF=0`, `UWB_HPRF=1`, and `UWB_EHPRF=2`;
they are not the waveform sidecar/VSA mode numbers. C callers use
`DEMOD_UWB_BPRF`, `DEMOD_UWB_HPRF`, or `DEMOD_UWB_EHPRF` in `DemodConfig.phyMode`.
EHPRF uses the periodic preamble receiver at the configured chip/sample rates.
That synchronization-only entry point does not decode frame fields. Use the
frame API below for dynamic PHR, BCC/LDPC, keyed STS and PSDU decoding.
The common configuration uses ABI 2 and adds the optional Bluetooth
`steadyFrequencyDeviation` switch (default false). Bluetooth synchronization
can skip an incomplete leading LE packet; UWB frame decoding retries SFD after
a sufficiently long quiet gap without relaxing either correlation threshold.

### Native UWB Frame Decoding

The additive `demodulate_uwb_frame` API recovers SFD timing, keyed STS
correlation, SECDED-protected PHR bits, and decoded PSDU bytes. It does not
consume generator field boundaries or `PayloadHex`:

```python
from nrdemod import demod
from nrdemod.vsa import load_iq

iq, fs = load_iq("captures/generated/uwb/UWB_4a_ch9_CI3_PRF15M6_6M81_PSDU100.txt")
cfg = demod.Config(sampleRate=fs, syncLength=64)
frame = demod.demodulate_uwb_frame(iq, cfg, demod.UwbFrameConfig())
if not frame.ok:
  raise RuntimeError(frame.error)
if frame.psduValid:
  payload = bytes(int(value) for value in frame.psduBytes)
  print(payload.hex(), frame.rsCorrectedSymbols)
```

Frame decoding includes all 32 standardized preamble tables, short/long
legacy SFD and ERDEV SFD0-4, legacy 0.11/0.85/1.7/6.81/27.24 Mbps
rate/PRF-dependent layouts, and integer 2-16 samples/chip. Applicable
code/rate/SFD combinations depend on the PHY. BPRF uses 6.81 Mbps,
up to 127 payload bytes, and one 64-unit STS segment; its AES
implementation uses Windows CNG (`bcrypt`), so non-Windows builds reject
STS requests. For BPRF SP1 use `legacy=False`, `codeIndex=10`,
`stsPacketConfig=1`, and `highRatePhr=True` for the bundled ch5 recording.
SP3 contains STS only: `ok=True` does not imply PHR or PSDU validity.

`stsKeyHex` and `stsIvHex` defaults are public fixture values, not secure
production credentials. Supply the actual session settings for real signals.
STS correlation is not secure-ranging authentication or certification.
The API also returns individual validity flags, PHR bits, and field offsets
and lengths in zero-based input samples, aligned to the acquired pulse
lattice (including receive-filter delay, not generator impulse boundaries).
The existing burst/preamble/CFO API is unchanged. `legacyRateMbps` selects
long-SFD/110-kbps acquisition; other legacy rates are decoded from PHR.
Set `legacyMeanPrfMHz` to 3.9, 15.6 or 62.4 and optionally override
`preambleSpread` (zero selects the normal code/PRF-dependent value).
`legacyVsaHopping=True` reconstructs the bundled compatibility recording;
set it to `False` for unmodified MATLAB legacy hopping.

HPRF frame decoding supports preamble codes 25-32, SFD0-4, PRF124.8/249.6 MHz,
K3 with shortened RS or K7 with separately terminated PHR/PSDU and no RS,
12-bit decoded lengths (up to 4095 bytes), and 1-4 STS segments.
Each segment must pass keyed correlation; `stsRho` is the minimum segment
correlation. `stsSegmentLength` accepts 16/32/64/128/256 units of 512 chips.
SP2 supports `extraStsGapLength` (0-127 units of four chips) and
`extraStsGapIndex` (0-3, checked against PHR where applicable).
Example for the bundled
code-27 recording:

```python
iq, fs = load_iq("captures/generated/uwb/UWB_4z_HPRF_ch5_CI27_124M8_PSDU100.txt")
cfg = demod.Config(sampleRate=fs, phyMode=demod.UWB_HPRF, syncLength=32)
options = demod.UwbFrameConfig(
  legacy=False, codeIndex=27, sfdIndex=1, stsPacketConfig=1,
  meanPrfMHz=124.8, constraintLength=7, numStsSegments=2,
)
frame = demod.demodulate_uwb_frame(iq, cfg, options)
assert frame.ok, frame.error
```

Use code25/PRF249.6/K3/one STS segment for the bundled 127-byte HPRF case.
All new fields extend `DemodUwbFrameConfig` at the end; the DLL accepts
both previous 48-byte and 64-byte Win64 layouts with their original defaults.
The current Win64 structure is 120 bytes; common `DemodConfig` stays unchanged.

#### Draft EHPRF, FCS, and Native EVM

EHPRF supports dynamic PHR1 rate/coding detection, CRC-8-protected PHR2,
K7 BCC or shortened rate-1/2 QC-LDPC (648/1296/1944-bit blocks), and
on-air rates 1.95/7.8/31.2/62.4/124.8 Mbps. With `dynamicPhr=False`,
`bitRateMbps` selects static-PHR rates, including older 6.8/27.2 modes.
This implements the draft format used by the installed reference generator,
not a claim of compliance with every revision of IEEE 802.15.4ab.

```python
iq, fs = load_iq("captures/generated/uwb/UWB_4ab_EHPRF_ch5_7M8_LDPC_PSDU127.txt")
cfg = demod.Config(sampleRate=fs, chipRateHz=fs / 4, phyMode=demod.UWB_EHPRF)
options = demod.UwbFrameConfig(legacy=False, codeIndex=25, stsPacketConfig=1)
frame = demod.demodulate_uwb_frame(iq, cfg, options)
assert frame.ok and frame.phr1Valid and frame.ldpcCoding
print(frame.bitRateMbps, frame.psduEvmRmsPercent)
```

`macFcsLength=0` explicitly skips MAC checking (generated PSDUs contain
random bytes). Set it to 2 for CRC-16/KERMIT or 4 for reflected CRC-32/ISO-HDLC,
with little-endian appended FCS bytes. Inspect `macFcsChecked` and
`macFcsValid`; failed FCS makes `ok=False` without erasing successful PHY
decoding (`psduValid`). `ldpcCorrectedBits` and `rsCorrectedSymbols` report
their respective FEC corrections.

Native `shrEvmRmsPercent`, `stsEvmRmsPercent`, `phrEvmRmsPercent` and
`psduEvmRmsPercent` require `evmValid` and the corresponding stage flag.
The receiver re-encodes corrected PHR/PSDU bits, reconstructs pulse signs,
and fits a complex 33-tap chip-spaced FIR using the first 16 SYNC symbols.
It measures interior chip-lattice residual power relative to reference
power, excluding up to 33 chips at each field edge (at most one quarter
of a short field). Gain/phase are not refitted to the payload. It assumes
a stationary pulse/channel response; it is neither all-sample waveform EVM
nor validated as equivalent to 89600 NRMSE. Absent-field metrics stay zero
and must not be read as successful measurements.

LDPC uses MIT-licensed `tavildar/LDPC`, pinned at
`05ee7f4af36ed5dacf52861315af68b8a17e71e0`, in `third_party/ldpc` with its
license retained. Its half-rate matrices match the reference EHPRF tables;
native code handles EHPRF shortening, block distribution and parity checks.
The decoder currently uses hard-decision-derived LLRs, not a calibrated
soft-channel noise estimate.

Rebuild the shared library and restart an existing Python/notebook kernel
before using the new entry point. The UWB notebook plots native-decoded
fields and compares payload references only after decoding. On Windows,
`ctest --test-dir build -R Uwb --output-on-failure` also runs bundled capture
regressions when recordings are present: byte-exact legacy/BPRF SP0/SP1,
STS-only SP3, both HPRF K3/K7 captures, all three EHPRF BCC/LDPC captures,
+25 kHz CFO, wrong-key/truncation/SFD/missing-segment rejection, K7 and LDPC
error recovery, RS-symbol errors (one/four corrected, tested five-error
pattern rejected), CRC known vectors, and clean/10%-amplitude-error EVM.

Independent expanded-settings regression, with MATLAB required only for
reference generation, writes ignored binary vectors under the build tree:

```text
matlab -batch "addpath('tests'); generate_uwb_frame_vectors('build/frame-tests/vectors')"
python tests/UwbFrameApiTest.py
```

Run from `nrdemod`. This checks 77 generated frames, all 32 preamble tables,
LDPC shortening boundaries, STS lengths/gaps, SFDs, legacy and EHPRF rates,
valid MAC FCS frames, native EVM, and both previous C option layouts.

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
