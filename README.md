# nrdemod-vsa — 89600 correlation bench (Windows)

A separate VS Code workspace that runs on the Windows station where the
Keysight 89600 VSA software lives. It builds the standalone `nrdemod` core,
pulls I/Q from the 89600, demodulates the same samples, and reports the delta.

```
nrdemod-vsa/
  nrdemod/                  the standalone demodulator (C++ core + Python pkg)
  tools/vsa_link.py         SCPI client for the 89600 SCPI server
  tools/correlate.py        live / file / selftest correlation runner
  tools/report.py           CSV trend log + JSON + HTML report with plots
  tools/replay_exported_waveforms.py  batch-replay captures/exported .npy/.json (5GNR, native core)
  configs/scpi_89600.json   editable SCPI command map
  configs/*.json            measurement cases (FR1 100 MHz, FR2 200 MHz, ...)
  captures/                 VSA recordings
  captures/exported/        5GNR TDC captures exported as .npy+.json (native nrdemod replay)
  captures/tdc_reference/   WiFi/UWB/Bluetooth/NB-IoT/LTE/eMTC TDC captures as raw .wfm+.para2 (legacy-only
                            standards: no native nrdemod backend, so play the .wfm directly into
                            the 89600 and compare against the recorded RESULTs in the .para2)
  captures/tdc_reference_plaintext/  same captures as unencrypted VSA_89600 .txt (no Signal Studio
                            decrypt license needed; nrdemod/python/nrdemod/vsa.py's load_iq() reads
                            this format directly)
  reports/                  generated reports
  build.bat                 cmake + venv + dependencies + self test
  nrdemod-vsa.code-workspace
```

Produced by `pack_for_windows.sh [output.zip] [--waveforms <export-dir>] [--tdc-reference <TDC-root>]
[--tdc-reference-plaintext <converted-dir>]` in the main repo: `--waveforms` bundles the 5GNR
native-replay export (`captures/exported/`), `--tdc-reference` bundles the raw encrypted
WiFi/UWB/Bluetooth/NB-IoT/LTE/eMTC TDC pairs (`captures/tdc_reference/`), `--tdc-reference-plaintext` bundles
the same captures converted to unencrypted VSA_89600 `.txt` (`captures/tdc_reference_plaintext/`) via
the in-tree `testDemod_ConvertTdcWaveformsToUnencryptedVsa89600` test
(`NR_DEMOD_REFERENCE_WAVEFORM_CONVERT_OUTPUT`).


## Install on the Windows station

Prerequisites: CMake, Visual Studio 2019/2022 Build Tools (C++), Python 3.8+.

1. Unzip the bundle to e.g. `C:\nrdemod-vsa`.
2. Run `build.bat`. It configures CMake, builds `nrdemod.dll`, runs the C++
   unit test, creates `.venv`, installs numpy/scipy/matplotlib and runs the
   Python self test.
3. Open `nrdemod-vsa.code-workspace` in VS Code. Tasks 1–5 cover the whole
   workflow, and the launch configurations attach the debugger.

The self test must show ~0 % EVM before you involve the instrument.

## Enable the 89600 SCPI server

In the 89600 VSA: **Utilities > SCPI Configuration** → enable the server, note
the port (default **5025**). If the VSA runs on this same PC, the host is
`127.0.0.1`; otherwise use the VSA PC's address and open the port in the
firewall.

Verify the link and discover which mnemonics your VSA release accepts:

```bat
.venv\Scripts\python.exe tools\vsa_link.py --probe --host 127.0.0.1
```

The probe prints every mapped query and whether it returned a value. **Edit
`configs/scpi_89600.json`** to keep what works — no Python changes needed.
Mnemonics differ between VSA releases and measurement personalities, so the
shipped map is a set of candidates, not a guarantee.

Send a single command yourself at any time:

```bat
.venv\Scripts\python.exe tools\vsa_link.py --raw "*IDN?"
```

## Local COM link (no SCPI server)

When the 89600 runs on this PC you can skip the SCPI server entirely: the VSA
registers the `AgtVsaVector.Application` automation object while it is open.

```bat
.venv\Scripts\python.exe tools\vsa_link.py --com
.venv\Scripts\python.exe tools\correlate.py live --link com --points 131072 ^
    --recording captures\ideal.mat --no-configure
```

`tools/vsa_com.py` borrows a spare display trace to read the time record and
the Syms/Errs table, and puts it back afterwards, so the traces on screen are
left alone. `tools/vsa_play.py` backs the current setup up to a `.setx` and
plays a recording through the analyser:

```bat
.venv\Scripts\python.exe tools\vsa_play.py captures\ideal.mat --vector --points 131072
.venv\Scripts\python.exe tools\vsa_play.py --restore reports\vsa_setup_backup.setx
```

Write a recording the VSA accepts with `nrdemod.vsa.save_mat()`; the `.csv`
writer does not carry the sample rate through the VSA's importer.

The 89600 ships 5G NR demo signals with matching setups under
`Help\Signals\5G NR\*.{setx,sdf}`. `configs/vsa_demo_dl_3g5_100mhz.json`
correlates against `DL_at_3_5_GHz_100_MHz`:

```bat
.venv\Scripts\python.exe tools\correlate.py live --link com --no-configure ^
    --config configs\vsa_demo_dl_3g5_100mhz.json
```

Load the signal's `.setx` in the VSA first (it selects the 5G NR personality
and loads the recording).

## Correlation workflows

### A. File based (most reliable — start here)

1. In the 89600, measure the burst and read its 5G NR **EVM**.
2. `File > Save > Recording` → `captures\run01.mat` (or CSV).
3. ```bat
   .venv\Scripts\python.exe tools\correlate.py file captures\run01.mat ^
       --config configs\fr1_100mhz_mu1_64qam.json --vsa-evm 1.42
   ```

### B. Live over SCPI

```bat
.venv\Scripts\python.exe tools\correlate.py live ^
    --host 127.0.0.1 --config configs\fr1_100mhz_mu1_64qam.json
```

The runner sets center/span/points from the case file, triggers one sweep,
reads the VSA EVM and the time trace, demodulates, and compares. Add
`--no-configure` to leave the VSA setup untouched, or `--vsa-evm`/`--vsa-freq-err`
to supply the reference numbers manually when the queries are not mapped.
If the time trace cannot be read over SCPI it falls back to telling the VSA to
save a recording (`vsa.recordingPath` in the case file) and loading it from
`--recording-local`.

### C. Closed loop sanity check

```bat
.venv\Scripts\python.exe nrdemod\examples\vsa_correlate.py --bw 100 --mu 1 ^
    --mod 64qam --fs 122880000 --export captures\ideal.csv
```

Load `ideal.csv` into the 89600 as a recording. Both sides should read ~0 %
EVM; any difference is analyser setup, not the DSP.

## Measurement cases

`configs/*.json` hold the demodulator settings, the VSA setup and the pass
tolerances:

```json
"demod":  { "numerology": 1, "channelBandwidthMHz": 100, "modulation": "64qam",
            "sampleRate": 122880000.0, "equalize": true, "dmrsExclude": true },
"vsa":    { "centerHz": 3.5e9, "spanHz": 122.88e6, "points": 131072 },
"tolerance": { "evmPercentPoints": 0.5, "freqErrorHz": 50.0 }
```

Every key under `demod` maps 1:1 onto `nrdemod.Config`, so anything the core
supports (`transformPrecoding`, `resourceBlockOffset`, `carrierOffsetHz`,
`dmrsAdditionalPositions`, `symbolTimingAdjustmentPercent`, …) can be set here.

## Output

Each run appends one row to `reports/correlation_log.csv` (for trending) and
writes `<case>_<timestamp>.{json,html,npz}`. The HTML has the scalar tables,
the pass/fail verdict and a constellation / EVM / spectrum plot. The `.npz`
holds every result vector for your own analysis.

Exit codes: `0` pass or no reference, `1` demodulation failed, `2` outside
tolerance or setup error — usable directly in a regression script.

## When the numbers disagree

Work down this list; it is almost always one of these:

| Symptom | Likely cause |
|---|---|
| nrdemod EVM much higher | wrong `numerology` / `channelBandwidthMHz` / `sampleRate` |
| EVM ~ random | RB allocation mismatch — set `numResourceBlocks` / `resourceBlockOffset` |
| EVM high only on DM-RS symbols | DM-RS config: `dmrsMappingTypeB`, `dmrsTypeAPosition`, `dmrsAdditionalPositions` |
| `frequencyErrorHz` pinned near half a subcarrier, `flatnessRippleRange2Db` tens of dB | the carrier does not sit on the tuned centre — set `carrierOffsetHz` to ±SCS/2 |
| other large `frequencyErrorHz` | VSA center offset — set `carrierOffsetHz` |
| EVM a few % on a clean signal | the channel estimate is blind, its noise falls as 1/√symbols — raise `maxSymbols` and `equalizerIterations` |
| constant EVM floor | VSA applies different equalizer training; try `--no-equalize` or `symbolTimingAdjustmentPercent` |
| nrdemod much *better* than VSA | VSA includes RF impairments the recording already contains; compare on the same recording, not on separate captures |

## Keeping the core in sync

The `nrdemod/` folder is a copy. After the core changes in the main Linux
repository, re-run `pack_for_windows.sh` there and re-unzip here.

## GitHub upstream in the Linux workspace

### 2026-10-22 Native Core Integration

The standalone library and bench now share the latest NR, NB-IoT, Bluetooth,
UWB and WiFi 7 fixes. NR includes the interleaved-input path, guarded CFO and
post-CFO DC removal, DM-RS power/configuration and reference-resource controls.
Bluetooth can skip incomplete leading LE packets and optionally measure steady
frequency deviation. UWB retries SFD after a quiet gap without lowering its
threshold; WiFi 7 reuses exact LTF scores during synchronization searches.

Both native C libraries use ABI 2; rebuild them together with the Python
bindings. `native_correlate.py` now uses `nrdemod.demodulate_nbiot` for NB-IoT,
including uplink transform precoding and slot-aligned pilot exclusion.
See `nrdemod/README.md` for NPSS frame mode and supported allocation limits.

Validation covers the C++ core regressions, Python API and synthetic NPSS/DC
tests, plus optional external full-capture NB uplink replay at zero and +/-2 kHz
CFO. It does not constitute a new Windows VSA run. Hardware-driver compact
results, async power processing and tester-specific profiling stay outside this
portable repository; no speculative state-pool optimization is included.

The Linux workspace can track GitHub without replacing its existing directories.
The directory mapping is:

| Linux workspace | GitHub `pzhou2005-source/nrdemod-vsa` |
|---|---|
| `nrdemod_standalone/` | `nrdemod/` |
| `nrdemod_vsa_bench/` | Repository root |

In the Linux workspace, `nrdemod_vsa_bench/nrdemod` is a symlink to
`../nrdemod_standalone`. In the GitHub repository, `nrdemod/` contains the
standalone source files. This is a source mapping, not a submodule.

Run this setup once from the SmarTest repository root in each new workspace:

```bash
git remote add nrdemod-vsa https://github.com/pzhou2005-source/nrdemod-vsa.git
git config remote.nrdemod-vsa.fetch '+refs/heads/main:refs/remotes/nrdemod-vsa/main'
git config remote.nrdemod-vsa.tagOpt --no-tags
git config remote.nrdemod-vsa.pushurl disabled://nrdemod-vsa
git fetch nrdemod-vsa
```

The remote configuration is local to `.git/config`, not versioned. If the remote
already exists, inspect `git remote -v` rather than running `remote add` again.
The existing internal `origin` is unchanged. Pushes through `nrdemod-vsa` are
deliberately disabled to prevent publishing the parent SmarTest repository.

To fetch updates and compare the committed standalone snapshot with GitHub:

```bash
git fetch nrdemod-vsa
git log -1 --oneline nrdemod-vsa/main
git diff --stat HEAD:nrdemod_standalone nrdemod-vsa/main:nrdemod
git diff HEAD:nrdemod_standalone nrdemod-vsa/main:nrdemod -- include/uwb src/UwbHrpCore.cpp tests/UwbHrpCoreTest.cpp
```

These comparisons use committed trees; inspect `git status` and `git diff`
separately for uncommitted work. Fetching updates the remote-tracking reference,
not the working files. Import reviewed changes deliberately, preserving local
changes and checking the associated C ABI, Python bindings, build configuration,
tests and dependencies together. Updating standalone also updates what the local
VSA symlink exposes, but does not update the integrated platform cores.

Do not run `git pull nrdemod-vsa main` or merge its root into SmarTest: the
repositories have different roots and histories. Publishing belongs in a
separate checkout of the public GitHub repository, with only reviewed standalone
and VSA files copied into their mapped locations; never push SmarTest history.
Likewise, `sync_from_tree.sh` follows the documented platform-to-standalone
direction, not GitHub-to-standalone synchronization.
