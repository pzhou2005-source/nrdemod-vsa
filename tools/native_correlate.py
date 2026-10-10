"""Correlate the new native WiFi / Bluetooth LE / UWB demodulators (nrdemod.demod) with the 89600 VSA.

The VSA side is read from reports/legacy_correlation.csv (produced by legacy_correlate.py), the native side is
run here on the same plaintext waveforms. Families without a native core (NB-IoT, Bluetooth BDR/EDR/LR) are listed
as unsupported.

    python tools/native_correlate.py --list
    python tools/native_correlate.py                       # everything, writes reports/native_correlation.csv
    python tools/native_correlate.py --family 802.11a --limit 3
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import sys
from fractions import Fraction

import numpy as np
from scipy.signal import resample_poly

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, ".."))
sys.path.insert(0, os.path.join(_ROOT, "nrdemod", "python"))
sys.path.insert(0, _HERE)
os.environ.setdefault("DEMOD_LIBRARY", os.path.join(_ROOT, "nrdemod", "python", "nrdemod", "_lib", "demod.dll"))

import legacy_correlate as L  # noqa: E402  (parsing helpers shared with the VSA batch)
from nrdemod import demod  # noqa: E402

_VSA_CSV = os.path.join(_ROOT, "reports", "legacy_correlation.csv")
EVM_TOL_PP = 0.1  # near-ideal waveforms: native and VSA EVM must agree within 0.1 percentage points
DEV_TOL_PCT = 5.0
FREQ_TOL_HZ = 100.0
BT_FREQ_TOL_HZ = 300.0  # the VSA's own FskCarrOffs scatters by up to ~220 Hz on these ideal bursts
NO_NATIVE = {
    "NbIotDl": "no native NB-IoT core",
    "NbIotUl": "no native NB-IoT core",
    "BluetoothBdr": "native BT core is LE only (no BR GFSK)",
    "BluetoothEdr": "native BT core is LE only (no EDR DPSK)",
    "BluetoothLeCodedS8": "native BT core handles uncoded LE1M/LE2M only",
}
FAMILY_ID = {"802.11a": demod.WIFI_A, "802.11b": demod.WIFI_B, "802.11n": demod.WIFI_N, "802.11ac": demod.WIFI_AC,
             "BluetoothLe1M": demod.BT_LE, "BluetoothLe2M": demod.BT_LE, "802.15.4z_HrpUwb": demod.UWB_HRP}


def _vsa_rows() -> dict:
    with open(_VSA_CSV, newline="") as fh:
        return {(r["family"], r["waveform"]): r for r in csv.DictReader(fh)}


def _num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def _resample(iq, fs, fs_new):
    if fs == fs_new:
        return iq
    fr = Fraction(fs_new / fs).limit_denominator(1000)
    return resample_poly(iq, fr.numerator, fr.denominator)


def _run(fid, iq, fs, **kw):
    res = demod.demodulate(fid, iq, demod.Config(sampleRate=fs, **kw))
    return res, res.as_dict()


def _evm_compare(row, native_evm, vsa_evm, decode_ok):
    row["native_EvmRms"], row["vsa_EvmRms"] = native_evm, vsa_evm
    if vsa_evm is None:
        row["pass"], row["note"] = None, "no VSA result"
        return
    row["delta_pp"] = native_evm - vsa_evm
    row["pass"] = bool(abs(row["delta_pp"]) <= EVM_TOL_PP and decode_ok)
    if not decode_ok:
        row["note"] = "native header decode wrong"


def correlate_one(family, stem, wav, vsa) -> dict:
    row = {"family": family, "waveform": stem}
    if family in NO_NATIVE:
        row.update(status="unsupported", note=NO_NATIVE[family])
        return row
    iq, fs, _center = L.parse_waveform(wav)
    vall = json.loads(vsa["vsa_all"]) if vsa and vsa.get("vsa_all") else {}
    vsa_evm = _num(vsa.get("vsa_EvmRms")) if vsa else None
    vsa_freq = _num(vsa.get("vsa_FrequencyError")) if vsa else None
    fid = FAMILY_ID[family]
    row["status"] = "ok"

    if family == "802.11a":
        res, d = _run(fid, iq, fs)
        rate = int(re.search(r"(\d+)Mbps", stem).group(1))
        row.update(native_info=f"rate {d.get('dataRateMbps')} Mbps, {d.get('psduLengthBytes')} B", native_FreqErr=d.get("frequencyErrorHz"))
        _evm_compare(row, d.get("rmsEvmPercent"), vsa_evm, d.get("syncFound") and d.get("dataRateMbps") == rate)
    elif family == "802.11b":
        res, d = _run(fid, iq, fs)
        row.update(native_info=f"cck={d.get('isCckDataRate')} dataEVM={d.get('dataRmsEvmPercent'):.2f} %",
                   native_FreqErr=d.get("frequencyErrorHz"))
        _evm_compare(row, d.get("rmsEvmPercent"), vsa_evm, bool(d.get("syncFound")))
    elif family in ("802.11n", "802.11ac"):
        bw = int(re.search(r"(\d+)MHz", stem).group(1))
        mcs = int(re.search(r"MCS(\d+)", stem).group(1))
        if family == "802.11n" and bw != 20:
            row.update(status="unsupported", note="native 802.11n core is 20 MHz only")
            return row
        # the cores take the capture as recorded: 20 MHz is 2x oversampled, and the 802.11ac core channelises
        # wider captures itself (it needs the recorded 4/5/10x oversampling, a rate equal to BW makes it
        # treat the signal as 20 MHz)
        x, fs_in = iq, fs
        res, d = _run(fid, x, fs_in)
        row.update(native_info=f"mcs {d.get('mcsIndex')} mod {d.get('modulation')} bw {d.get('bandwidthMHz')}",
                   native_FreqErr=d.get("frequencyErrorHz"))
        if bw == 20:
            if family == "802.11n":
                decode_ok = d.get("mcsIndex") == mcs
            else:  # the TDC 11ac VHT-SIG-A fails its CRC, so only the modulation order is checked
                decode_ok = d.get("modulation") == {"BPSK": 0, "QPSK": 1, "16QAM": 2, "64QAM": 3, "256QAM": 4}[stem.split("_")[-1]]
            _evm_compare(row, d.get("rmsEvmPercent"), vsa_evm, bool(d.get("syncFound")) and decode_ok)
        else:  # the native cores only synchronise / estimate CFO above 20 MHz
            row["note"] = "native core has no EVM above 20 MHz; compared on sync and frequency error"
            ok = bool(d.get("syncFound")) and d.get("bandwidthMHz") == bw
            if vsa_freq is not None and d.get("frequencyErrorHz") is not None:
                row["delta_freq_hz"] = d["frequencyErrorHz"] - vsa_freq
                ok = ok and abs(row["delta_freq_hz"]) <= FREQ_TOL_HZ
            row["vsa_FreqErr"] = vsa_freq
            row["pass"] = bool(ok)
    elif family.startswith("BluetoothLe"):
        phy = 2 if family.endswith("2M") else 1
        res, d = _run(fid, iq, fs, phy=phy)
        vec = res.frequencyDeviationHz
        k = max(int(d.get("samplesPerSymbol", 10) / 4), 1)
        f = np.convolve(vec, np.ones(k) / k, mode="valid") if vec.size > k else vec
        dev = float((np.percentile(f, 99.5) - np.percentile(f, 0.5)) / 2) if f.size else None
        vsa_dev = vall.get("FskDev")
        row.update(native_dev_hz=dev, vsa_dev_hz=vsa_dev, native_deltaFAvg_hz=d.get("deltaFAvgHz"),
                   native_FreqErr=d.get("frequencyErrorHz"), vsa_FreqErr=vall.get("FskCarrOffs"),
                   native_info=f"syncWord={d.get('syncWordFound')} rho={d.get('syncWordRho')}")
        if dev is not None and vsa_dev:
            row["delta_dev_pct"] = (dev / vsa_dev - 1) * 100
            row["pass"] = bool(abs(row["delta_dev_pct"]) <= DEV_TOL_PCT and d.get("syncWordFound")
                               and abs(d.get("frequencyErrorHz", 0) - (vall.get("FskCarrOffs") or 0)) <= BT_FREQ_TOL_HZ)
    elif family == "802.15.4z_HrpUwb":
        res, d = _run(fid, iq, fs, phyMode=1 if "HPRF" in stem else 0)
        row.update(native_info=f"syncRho={d.get('syncRho')}", native_FreqErr=d.get("frequencyErrorHz"), vsa_FreqErr=vall.get("FreqErr"))
        ok = bool(d.get("syncFound"))
        if vall.get("FreqErr") is not None and d.get("frequencyErrorHz") is not None:
            row["delta_freq_hz"] = d["frequencyErrorHz"] - vall["FreqErr"]
            ok = ok and abs(row["delta_freq_hz"]) <= FREQ_TOL_HZ
        row["pass"] = bool(ok)
    if not res.ok:
        row.update(status="error", pass_=None, note=res.error)
        row["pass"] = None
    return row


COLUMNS = ["family", "waveform", "status", "pass", "native_EvmRms", "vsa_EvmRms", "delta_pp", "native_dev_hz", "vsa_dev_hz",
           "delta_dev_pct", "native_deltaFAvg_hz", "native_FreqErr", "vsa_FreqErr", "delta_freq_hz", "native_info", "note"]


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--family", action="append")
    p.add_argument("--limit", type=int)
    p.add_argument("--list", action="store_true")
    p.add_argument("--csv", default=os.path.join(_ROOT, "reports", "native_correlation.csv"))
    args = p.parse_args(argv)

    vsa_rows = _vsa_rows()
    items = L.discover(set(args.family) if args.family else None)
    if args.limit:
        seen, kept = {}, []
        for it in items:
            seen[it[0]] = seen.get(it[0], 0) + 1
            if seen[it[0]] <= args.limit:
                kept.append(it)
        items = kept
    if args.list:
        for family, stem, _w, _p in items:
            print(f"{family}\t{stem}\t{NO_NATIVE.get(family, 'native')}")
        return 0

    with open(args.csv, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=COLUMNS, extrasaction="ignore")
        writer.writeheader()
        for family, stem, wav, _para in items:
            try:
                row = correlate_one(family, stem, wav, vsa_rows.get((family, stem)))
            except Exception as exc:  # noqa: BLE001
                row = {"family": family, "waveform": stem, "status": "error", "note": f"{type(exc).__name__}: {exc}"[:300]}
            writer.writerow(row)
            fh.flush()
            print(f"{family}/{stem}: {row.get('status')} pass={row.get('pass')} {row.get('note') or ''}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
