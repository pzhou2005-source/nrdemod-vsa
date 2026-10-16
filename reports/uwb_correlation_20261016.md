# UWB correlation follow-up: 2026-10-16

Measurements with Keysight 89600 VSA 2026_U2: **12 pass, 0 fail,
3 EHPRF unsupported**. The final legacy case uses an explicitly labeled
VSA-compatible hopping export; its original MATLAB recording is archived.
All 15 native repetition-synchronization and injected-CFO checks pass.
Native checks do not validate STS, FEC, or payload decoding.

## Verified corrections

- MATLAB-generated BPRF SP1/SP3 recordings now use the generator's STS key,
  upper counter, and initial count instead of the VSA demo's values.
- Generated HPRF uses an explicit supported SFD and the active low/high
  data-rate mode selected by mean PRF, not the inactive PHR-rate property.
- HPRF recordings are exported on a 499.2 MHz chip clock. The 124.8 MHz
  configuration previously inherited a 249.6 MHz clock from MATLAB R2025a.
  Only the two HPRF recordings were regenerated; other capture samples
  remain unchanged.
- Sidecars export active BPRF rates, burst/hop parameters, chip clock, and
  STS initialization. Legacy mode explicitly disables its inactive STS flag.

## Original Legacy Discrepancy

The original MATLAB `UWB_4a_ch9_CI3_PRF15M6_6M81_PSDU100` failed: SHR NRMSE was
0.000006%, PHR NRMSE is 13.380928%, and PSDU NRMSE is 343.824291%.
VSA decodes the header's 6.81 Mbps rate, 100-byte length, and SECDED correctly.

Comparing VSA's IQ reference with MATLAB's unfiltered chip sequence gives
944/944 matching payload hopping positions at PN offset 335, versus 98/944
at MATLAB's offset 336. These are zero-based offsets after the PHR starts.
The diagnostic used the correct 2-chip payload bursts and 8 hop positions;
increasing inverse-pulse delay spread from 8 to 32 chips did not resolve it.
This establishes an implementation disagreement, not which implementation
conforms to the standard. A subsequent polarity comparison showed that all
944 payload spreading-chip pairs agree at MATLAB's original PN offset 336:
the difference is only the hopping window, not PN advancement or spreading.

## Legacy Interoperability Export

`tools/uwb/uwb_vsa_legacy_hopping.m` now selects the trailing hopping window
for the qualified two-chip/eight-hop legacy case. Header chips, spreading
polarities, data halves, payload coding, and sample count are preserved.
Other configurations are unchanged. The regenerated sidecar records
`LegacyHoppingConvention=VsaTrailingPnWindow` and names the adjusted generator.

A fresh VSA measurement of the saved export gives SHR NRMSE 0.000006%,
PHR NRMSE 0.002532%, and PSDU NRMSE 0.00000843%, with negligible CFO.
The original capture, sidecar, hash, and failed measurement are retained via
`uwb_legacy_original_20261016.json`. The original recording still disagrees
with VSA; the passing export is not independent IEEE conformance certification.

`gen_uwb(outRoot)` enables this bench convention; `gen_uwb(outRoot, false)`
retains unmodified MATLAB hopping. Comparing both exports confirms that only
the intended legacy waveform changes. Three MATLAB regression tests pass,
including measured hopping positions, preserved header/spreading/data halves,
and unaffected legacy rates/BPRF. Run them with
`runtests('tools/uwb/test_uwb_vsa_legacy_hopping.m')`.

## Evidence

- `uwb_vsa_all_20261016.csv`: fresh raw VSA tables, UTC timestamps, capture SHA256.
- `uwb_all_20261016.json` and `.csv`: strict native/VSA comparison.
- `../nrdemod/python/examples/uwb_demod.ipynb`: executed 11 code cells, eight figures.
- `uwb_legacy_original_20261016.json`: archived original legacy capture provenance.

Limits are unchanged: CFO agreement <= 100 Hz, SHR/STS NRMSE <= 0.5%,
PHR/PSDU NRMSE <= 2%. Missing, nonfinite, and no-result sentinel values
are rejected. EHPRF remains native-only in this VSA release.