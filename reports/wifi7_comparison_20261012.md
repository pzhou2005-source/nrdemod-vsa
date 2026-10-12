# WiFi 7 waveform comparison - 2026-10-12

Offline component test: `ElementaryTests.testDemod_NativeWifi11beAllTdcReferenceWaveforms`.
Input: `/home/pengzhou/demod/waveforms/802.11be`, 20/40/80/160/320 MHz, 189 waveforms.
Raw log: `/tmp/wifi7_compare_20261012.log`.

Native synchronization passed 189/189; legacy demodulation succeeded on 55/189.
All 134 legacy failures lacked an exact matching `.para2` file and reported synchronization errors. Missing configuration is a possible cause, not a proven algorithm defect; seven successful captures also lacked exact sidecars.

## Matched Successful Subset

Times include one `executeFile` call per waveform (file I/O, conversion and processing), excluding preset/parameter loading. These are single-run measurements, not warmed DSP benchmarks. Legacy executes first; file-cache effects are not controlled.

| Bandwidth MHz | Waveforms | Legacy DATA EVM median % | Legacy total ms | Native total ms | Legacy/native time ratio |
|---|---|---|---|---|---|
| 20 | 13 | 0.367556 | 165.071 | 41.204 | 4.006 |
| 40 | 7 | 0.493480 | 134.672 | 54.135 | 2.488 |
| 80 | 7 | 0.495060 | 151.758 | 38.511 | 3.941 |
| 160 | 7 | 0.552619 | 247.703 | 63.330 | 3.911 |
| 320 | 21 | 0.613475 | 1242.078 | 322.158 | 3.855 |
| Total | 55 | 0.500952 | 1941.281 | 519.337 | 3.738 |

Legacy DATA EVM range: 0.333548% to 0.821462%. Result values from the API are ratios and were multiplied by 100 for this report.

Matched-subset processing rates: legacy 28.332 waveforms/s; native 105.904 waveforms/s. Failed legacy calls are excluded from both sides of this calculation.

## Limitations

Native WiFi 7 only implements burst detection, L-LTF synchronization and CFO recovery. It does not decode EHT-LTF/DATA or produce DATA EVM. Consequently native EVM is unavailable, and the timing ratio is not a comparison of equivalent full demodulators. The component test pass condition covers native synchronization, not legacy success or EVM equivalence. No online RF measurement was performed.

## Subsequent Native DATA Extension

The preceding timing measurements describe the earlier sync-only implementation. Native service version 0.2.0 now optionally measures DATA EVM for 20 MHz, one spatial stream, full RU242, 4x EHT-LTF and MCS0-13. Wider DATA bandwidths, MRU/puncturing, multiple streams, 1x/2x LTF and automatic U-SIG/EHT-SIG decoding remain unsupported. The standalone C++ core mirrors this implementation; the flat C ABI/Python wrapper does not yet expose the new configuration.

Set these native service inputs after preset for the validated 20 MHz MCS7 reference waveform:

| Input | Value |
|---|---|
| DataEvmEnable | true |
| DataBandwidthMHz | 20 |
| NumSpatialStreams | 1 |
| EhtLtfType | 4 |
| DataMcs | 7 |
| EhtLtfFftOffsetSamples | 864 |
| DataFftOffsetSamples | 1184 |
| DataGuardIntervalSamples | 64 |
| DataSymbolCount | 4 |

FFT offsets are relative to PPDU start, in 20 MSa/s samples, not input-clock samples. They must match the actual waveform layout; they are not detected or selected by minimizing DATA EVM. `EvmRmsData` is a ratio (multiply by 100 for percent). Check `DataEvmValid` first; `DataError` explains unsupported/incomplete DATA, and reading invalid EVM throws a service error instead of returning zero.

The EHT-LTF provides the per-tone channel estimate independently of DATA. Pilot phase tracking and nearest-constellation DATA decisions measure residual error without retraining the channel on DATA.

| Real waveform | Native DATA EVM % | DATA symbols |
|---|---|---|
| 20 MHz MCS0 BPSK | 0.00710233 | 4 |
| 20 MHz MCS1 QPSK | 0.00711682 | 4 |
| 20 MHz MCS7 64QAM | 0.00711600 | 4 |
| 20 MHz MCS8 256QAM | 0.00672926 | 4 |
| 20 MHz MCS11 1024QAM | 0.00696891 | 4 |
| 20 MHz MCS13 4096QAM | 0.00673725 | 4 |

Validation: 28 synthetic zero/known-noise MCS0-13 cases agree with known EVM within 1e-9 ratio, including CFO/CPE/channel variation. Truncated DATA and unsupported bandwidth/stream/LTF cases reject DATA results. `Wifi11beOfdmCoreTest` CTest passes; the real component test `ElementaryTests.testDemod_NativeWifi11beDataEvm` runs one registered test covering all six waveforms and invalid-result exceptions. Final log: `/tmp/wifi7_data_final_20261012.log`. These results are not VSA oracle validation or an updated equivalent-workload throughput comparison.

## Other Native WiFi DATA Results

All native WiFi services now declare `EvmRmsData` (ratio, multiply by 100 for percent) and `DataEvmValid`. Reading an unavailable DATA result throws rather than returning a misleading zero. Original result names and measurements are preserved.

| Native standard | DATA support | Verified example EVM % |
|---|---|---|
| 802.11a | Existing OFDM DATA; alias of EvmRms | 0.00950425 |
| 802.11n | Existing 20 MHz single-stream HT DATA; alias of EvmRms | 0.00881826 |
| 802.11ac | Existing 20 MHz single-stream VHT DATA; alias of EvmRms | 0.00770873 |
| 802.11b, 5.5/11 Mbps | Existing CCK best-fit DATA residual; alias of DataEvmRms | 7.66403 at 11 Mbps |
| 802.11b, 1/2 Mbps | New explicit-layout Barker DATA, SYNC-trained gain/CFO/phase | 0.654853 / 0.683378 |
| 802.11ax | New explicit-layout 20 MHz/one stream/RU242/4x HE-LTF/MCS0-11 | 0.00689311 to 0.00780964 |
| 802.11ad | DATA decoder not implemented; DataEvmValid=false, getter rejects | Unavailable |

For the validated HE reference files under `802.11ax_20MHz/4xHELTF_3p2`, enable `DataEvmEnable` and set `DataMcs` to the known waveform MCS, `HeLtfType=4`, `HeLtfFftOffsetSamples=784`, `DataFftOffsetSamples=1104`, `DataGuardIntervalSamples=64`, and `DataSymbolCount=4`. `DataBandwidthMHz=20` and `NumSpatialStreams=1` are required. Offsets are relative to PPDU start in 20 MSa/s samples, not raw input-clock samples. HE uses the same independently trained RU242 DATA calculation as EHT, without duplicating its FFT/equalizer/phase logic. Automatic HE signaling, OFDMA/MRU, wider DATA, multiple streams and 1x/2x LTF remain unsupported.

For validated long-preamble 1/2 Mbps Barker captures, enable `BarkerDataEvmEnable`, set `BarkerDataDqpsk` false/true respectively, `BarkerDataSymbolOffset=192` and `BarkerDataSymbolCount=100`. The offset is relative to the recovered first SYNC symbol; other packet layouts require their actual explicit offset. Only complete DATA symbols before the detected burst end are accepted. Header EVM is never used as the DATA result. CCK keeps its existing, separately documented best-fit residual algorithm.

802.11ad needs an independently verified Golay training sequence, channel estimation and SC symbol recovery before a genuine native DATA EVM can be added. The current capability/error interface makes that gap explicit; it is not an implemented AD decoder. Wideband HT/VHT DATA also remains outside the existing scope. The C++ standalone mirrors are synchronized; new configurable DATA inputs are not exposed by the flat C ABI/Python wrappers.

Final verification: three directly executed component tests pass, covering 19 successful DATA waveform checks plus wideband VHT and AD unavailability checks. Seven real HE waveforms (MCS0/1/3/5/7/9/11) each measure four DATA symbols. Twenty-four synthetic HE zero/known-noise cases match expected EVM within 1e-9 ratio, plus HE MCS-limit/truncation and shared-result reset/empty-capture checks. Barker DBPSK/DQPSK synthetic known-noise and truncation checks pass. All 10 standalone CTests pass. Raw final log: `/tmp/wifi_other_data_final_20261012.log`. No new throughput benchmark, RF bench run or VSA accuracy oracle is claimed.

## Native 802.11ad SC DATA Extension

This later update supersedes the AD-unavailable statements above only for the explicitly configured scope below. Default operation remains burst-only.

Service v0.2.0 and the standalone C++ core optionally measure SC DATA for MCS1-9 at exactly one sample per chip, 1.76 GSa/s. The caller must supply an independent known 128-chip STF reference encoded as `+`/`-`, repeated four times at the specified training location. Repetition estimates CFO; reference correlation estimates static phase and scalar channel gain, independently of DATA. Each 512-chip block excludes its first 64-chip guard and measures the remaining 448 BPSK/QPSK symbols against nearest constellation decisions. `EvmRmsData` is a ratio, not percent, and is available only when `DataEvmValid=true`; invalid getters throw and `DataError` explains the reason.

The two validated 1x Signal Studio fixtures use:

```text
DataEvmEnable = true
DataMcs = 1 or 6
ScTrainingReference = ++-------+-++--+++--++++-+-+-++---+++++++-+--++-++--++++-+-+-++-++-------+-++--+++--++++-+-+-++-++-------+-++--+--++----+-+-+--+
ScTrainingOffsetSamples = 1760
ScDataOffsetSamples = 6112
DataBlockCount = 4
```

Offsets are absolute raw capture sample indices, not relative to the power detector's burst window. This stored reference belongs to the ideal test fixtures; it is not a built-in, independently standardized Golay table or an automatic packet detector. Other captures require their independently known transmitted training reference and actual layout.

| Real capture | Sample rate | DATA result |
|---|---|---|
| scWaveform_MCS1_SC.wfm | 1.76 GSa/s | 0% EVM, 1792 DATA symbols |
| scWaveform_MCS6_SC.wfm | 1.76 GSa/s | 0% EVM, 1792 DATA symbols |
| scWaveform_MCS06_from11adGenerator_IdleTime1uS_80Blocks.wfm | 7.04 GSa/s | Explicitly unsupported |
| scWaveform_MCS12_from11adGenerator_4xWithRRC.wfm | 7.04 GSa/s | Explicitly unsupported |

Zero EVM is appropriate for these two ideal generator fixtures. It is not evidence of measured RF performance or aligned legacy/VSA accuracy: measurement intervals, guards, filtering and equalizer settings differ, and old `WiGigDataEvmRms` sentinel values are not valid references. Legacy `WiGigEvmRms` sidecars use percent here.

Verification: 27 zero/known-noise cases across MCS1-9 with 400 kHz CFO and complex channel gain match expected EVM within 1e-10 ratio. Guards deliberately contain large errors and do not affect DATA EVM. Noise training, wrong/malformed reference, truncation, overlap, huge block count, nonfinite DATA, MCS12 and 4x sampling reject. Two directly executed component tests pass, including the existing default AD-unavailable contract and other WiFi DATA interfaces (`/tmp/ad_data_final_20261012.log`). The three AD C++ mirrors are synchronized; rebuilt standalone `demod` and `Wifi11adOfdmCoreTest` pass the focused CTest (`/tmp/ad_standalone_20261012.log`).

Remaining limits: flat channel only; no automatic Golay/CE/header acquisition, frequency-selective channel equalizer, RRC matched filtering, oversampled input, MCS10-12, OFDM or payload bits. CFO from 128-chip repetition is unambiguous only within +/-6.875 MHz. New configuration is exposed through the production service and standalone C++ Config, not the flat C ABI/Python wrappers. No new throughput benchmark, online RF or VSA oracle run, commit or push.

## Online WiFi Attempt: 802.11a Only

MENDOCINO/socbm782 was reserved (270675), configured online successfully and run through VNC display `:4`. The existing flow transmits 54 Mbps/64QAM at 5.18 GHz, -20 dBm, using internal cross-connect and 20 MHz receive bandwidth. It does not transmit/capture the 1.76 GSa/s AD SC waveform; **AD online EVM and throughput remain unverified**. No hardware maximum-bandwidth claim is inferred from this narrower test.

The final RF capture is 25 MSa/s, shared-resampled to 40 MSa/s before selecting identical inputs for legacy/native. Shared conversion takes 76.136304 ms and is excluded from demod execution timing. Nineteen activity regions were found; eighteen pass native complete-packet checks with observed leading idle. The capture-boundary packet is excluded because legacy returns `NO BURST FOUND` without a leading idle region. The selected single packet contains 7936 samples, raw grid indices [10752,18688).

| Single complete RF packet | Legacy | Native |
|---|---|---|
| Reported EVM, % | EvmRms 87.9476607 | EvmRmsData/EvmRms 1.40764748 |
| CFO estimate, Hz | 0.54158717 | 129.94359229 |
| Warm execution median, ms | 1.346351 | 1.947350 |
| Executions/s | 742.748 | 513.518 |
| Measured result length | 41 | 40 DATA symbols |

Each backend uses one warmup and five measured executions of the same IQ. Native is about 44.6% slower in this single-packet execution boundary (legacy/native ratio 0.691376); the legacy EVM failure means this is not an equivalent-accuracy throughput result. Legacy does not expose `EvmRmsData` here; its overall EVM and native DATA-only EVM do not have identical statistics.

Native property checks reached and passed: `DataEvmValid`, `SyncFound`, EVM alias equality, 54 Mbps rate and 40-symbol count, finite frequency, SyncRho=0.9999693959, peak EVM=8.89391109%, peak DATA symbol=5 and DATA subcarrier=44. These establish valid output/sanity, not independent EVM or CFO accuracy. The CFO disagreement remains unvalidated.

**Overall test FAILED**: legacy EVM exceeds the unchanged 10% validity gate. The multiple-packet timing case was therefore not executed and the system-log gate was NOT EXECUTED. No full-green online, AD, VSA precision or throughput-parity claim. Final runner: `/tmp/wifi_online_verify_20261012.log`; report: `vobs/zenith/workspace/integtests/tests/marlin/basecard/TestBed.socbm782.pengzhou.online-INTEG_TEST-model_marlin.T20261012-190739-159630298/log/ReportWindow.dump`.

Local test fixes retained: distinguish `WifiThroughputLoopbackFlow.flow` from the Java testmethod name, use the actual int setter overload for three legacy inputs (not long-to-double widening), require observed leading idle, and do not request a nonexistent legacy DATA field. No production DSP algorithm or EVM gate changed. ATE stopped, reservation closed, monitor stopped; user VNC retained. No commit/push.