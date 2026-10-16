#!/usr/bin/env python3
"""Run every HRP UWB waveform through the native demodulators and the 89600 VSA and correlate the two.

Waveforms: captures/generated/uwb/*.txt (MATLAB gen_uwb.m) and captures/tdc_reference_plaintext/802.15.4z_HrpUwb/*.txt.
Per waveform three receivers are run:

* native C++ demod (nrdemod.demod UWB_HRP): acquisition and BPRF/4a/HPRF PHR/PSDU decoding
* Python HRP PHY (examples/uwb_hrp.py, BPRF and 802.15.4a only): full PHR/PSDU decode, EVM, payload check
* 89600 VSA (HrpUwb personality, demo setx patched from the .para2 INPUTs): FreqErr, TimeOffset, NRMSE, ...

and the results are written to reports/uwb_correlation.csv.

    python tools/uwb/uwb_correlate.py --list
    python tools/uwb/uwb_correlate.py                     # everything (restarts the VSA once)
    python tools/uwb/uwb_correlate.py --match BPRF_SP0    # subset by substring
    python tools/uwb/uwb_correlate.py --no-vsa            # native/Python only
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import re
import subprocess
import sys
import threading
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, os.path.join(_ROOT, "tools"))
sys.path.insert(0, os.path.join(_ROOT, "nrdemod", "python"))
sys.path.insert(0, os.path.join(_ROOT, "nrdemod", "python", "examples"))
os.environ.setdefault("DEMOD_LIBRARY", os.path.join(_ROOT, "nrdemod", "python", "nrdemod", "_lib", "demod.dll"))

import legacy_correlate as L  # noqa: E402  (recording writer, VSA restart, table reader)

_OUT = os.path.join(_ROOT, "reports")
_TMP = os.path.join(_OUT, "uwb_tmp")
_SETX = os.path.join(L._SIGNALS, "HrpUwb", "HrpUwb_Ch1_Code2_L64_Sync64_Sfd8.setx")
SOURCES = {
    "generated": os.path.join(_ROOT, "captures", "generated", "uwb"),
    "tdc": os.path.join(_ROOT, "captures", "tdc_reference_plaintext", "802.15.4z_HrpUwb"),
}
_TDC_PARA = os.path.join(_ROOT, "captures", "tdc_reference", "802.15.4z_HrpUwb")
CHILD_TIMEOUT_S = 900
LEAD_S = 100e-6  # write_recording puts the burst this far into the recording

# VSA enum values (Agilent.SA.Vsa.HrpUwb.Interfaces)
_SYNC_LEN = {16: 0, 64: 1, 1024: 2, 4096: 3, 24: 4, 32: 5, 48: 6, 96: 7, 128: 8, 256: 9, 512: 10}
_DELTA_LEN = {4: 0, 16: 1, 64: 2}
_STS_SEG_LEN = {32: 0, 64: 1, 128: 2, 256: 3, 16: 4}
_CHANNEL_FC = {1: 3494.4e6, 2: 3993.6e6, 3: 4492.8e6, 4: 3993.6e6, 5: 6489.6e6, 6: 6988.8e6, 7: 6489.6e6,
               8: 7488.0e6, 9: 7987.2e6, 10: 8486.4e6, 11: 7987.2e6, 12: 8985.6e6, 13: 9484.8e6, 14: 9984.0e6,
               15: 9484.8e6}
# Table 15-3 (15.6 MHz and 62.4 MHz mean PRF): chips per burst by data rate in Mb/s, hopping bursts
_NCPB = {15.6: {0.11: 128, 0.85: 16, 6.81: 2, 27.24: 1}, 62.4: {0.11: 512, 0.85: 64, 6.81: 8, 27.24: 2}}
_NHOP = {15.6: 8, 62.4: 2}
# lrwpanHRPConfig builds the STS from the IEEE test-vector key/counter; the VSA default is a different key
_MATLAB_STS = {"DemodHrpUwbStsKey": "4A5572BC90798C8E518D2449092F1B55",
               "DemodHrpUwbStsUpper96": "68DEBD3A599939DD57FDBB0E", "DemodHrpUwbStsInitialCount": "2A10FAC0"}

# VSA pass criteria on ideal waveforms (VSA NRMSE is in %): a correct lock stays well below 1 %, while a
# wrong lock / undecoded frame gives tens to hundreds of %
NRMSE_DATA_MAX = 0.5
NRMSE_SHR_MAX = 0.5
CFO_TOL_HZ = 1e3


# --------------------------------------------------------------------------- discovery / parameters
def read_inputs(path: str) -> dict:
    p = {}
    with open(path, errors="strict") as fh:
        for line in fh:
            parts = [x.strip() for x in line.split(",")]
            if len(parts) >= 4 and parts[0] in ("INPUT", "INFO"):
                p[parts[1]] = parts[3]
    return p


def discover(match=None):
    items = []
    for source, folder in SOURCES.items():
        for txt in sorted(glob.glob(os.path.join(folder, "*.txt"))):
            stem = os.path.splitext(os.path.basename(txt))[0]
            if match and not any(m in stem for m in match):
                continue
            para = os.path.join(folder if source == "generated" else _TDC_PARA, stem + ".para2")
            items.append({"source": source, "stem": stem, "txt": txt, "para": para})
    return items


def params(item) -> dict:
    """Normalised waveform parameters from the .para2 INPUTs (plus the TDC file name for the frame length)."""
    p = read_inputs(item["para"])
    stem = item["stem"]
    phy = int(p.get("PhyMode", 1))
    mode = {0: "4a", 1: "BPRF", 2: "HPRF", 3: "EHPRF"}[phy]
    octets = int(p["PSDULength"]) if "PSDULength" in p else None
    m = re.search(r"FrameLeng(\d+)", stem)
    if octets is None and m:
        octets = int(m.group(1))
    sp = int(p.get("StsPacketConfig", 0))
    if mode == "4a":
        sp = 0  # lrwpanHRPConfig reports its default STSPacketConfiguration=1 even though 802.15.4a has no STS
    if sp == 3:
        octets = 0
    data_rate = float(p["DataRate"]) if "DataRate" in p else None
    hprf_high = None
    if mode == "HPRF":
        m = re.search(r"_(\d+)M(\d)_", stem)
        if m:  # Generated names encode mean PRF in MHz, not payload bit rate.
            hprf_high = float(f"{m.group(1)}.{m.group(2)}") > 200
        else:  # TDC: PDR-L/PDR-H
            hprf_high = "PDR-H" in stem
    return {
        "mode": mode,
        "code": int(p.get("CodeIndex", 9)),
        "sfd": int(p.get("SfdIndex", 0)),
        "sync": int(p.get("SyncLength", 64)),
        "delta": int(p.get("DeltaLength", 4)),
        "sp": sp,
        "sts_seg_num": int(p.get("NumSTSSegments", p.get("StsNumSegments", 1))),
        "sts_seg_len": int(p.get("STSSegmentLength", p.get("StsSegmentLength", 64))),
        "octets": octets,
        "pdr": int(p.get("PhrDataRateMode", 0)),
        "channel": int(p["Channel"]) if "Channel" in p else None,
        "mean_prf": float(p.get("MeanPRF", 62.4)),
        "data_rate": data_rate,
        "hprf_high": hprf_high,
        "payload_hex": p.get("PayloadHex"),
        "generator": "matlab" if "MatlabVersion" in p else "keysight",
        "constraint_length": int(p.get("ConstraintLength", 3)),
    }


def vsa_patches(pr: dict, center: float, fs: float, n_samples: int) -> dict:
    mode = pr["mode"]
    patches = {
        # the demo setx is tuned to channel 1 (3.4944 GHz); the recording's own center must be analysed
        "InputCenter": center, "MeasCenter": center,
        "InputSpan": min(998.4e6, fs / 1.25),
        # the demo searches 318 us; the whole frame (plus the recording lead-in) has to fit in the search window
        "DemodHrpUwbSearchLength": LEAD_S + n_samples / fs + 20e-6,
        "DemodHrpUwbPhyMode": {"4a": 0, "BPRF": 1, "HPRF": 2}[mode],
        "DemodHrpUwbSHRCodeIndex": pr["code"],
        "DemodHrpUwbSHRSFDSelector": pr["sfd"],
        "DemodHrpUwbSHRSyncLength": _SYNC_LEN[pr["sync"]],
        "DemodHrpUwbSHRSpreading": _DELTA_LEN.get(pr["delta"], 0),
        "DemodHrpUwbStsPacketConfig": pr["sp"],
        "DemodHrpUwbSegmentLength": _STS_SEG_LEN.get(pr["sts_seg_len"], 1),
        "DemodHrpUwbSegmentNumber": max(pr["sts_seg_num"], 1) - 1,
        "DemodHrpUwbDataNOctets": pr["octets"] or 0,
        "DemodHrpUwbDecodingLevel": 2,  # PSDU after convolutional/RS decoding
    }
    if pr["channel"] is not None:
        patches["DemodHrpUwbChannel"] = pr["channel"]
    if pr["generator"] == "matlab" and pr["sp"]:
        patches.update(_MATLAB_STS)
    if mode == "BPRF":
        patches["DemodHrpUwbDataRate"] = 1 if pr["pdr"] else 0  # BprfLow/BprfHigh = PHR at 0.85/6.81 Mb/s
        patches["DemodHrpUwbDataNCpb"], patches["DemodHrpUwbDataNHop"] = 8, 2
    elif mode == "HPRF":
        patches["DemodHrpUwbDataRate"] = 3 if pr["hprf_high"] else 2
        patches["DemodHrpUwbDataK"] = 1 if pr["constraint_length"] == 7 else 0  # CL3 / CL7
    elif mode == "4a":
        prf = 15.6 if abs(pr["mean_prf"] - 15.6) < 1 else 62.4
        rate = min(_NCPB[prf], key=lambda r: abs(r - (pr["data_rate"] or 6.81)))
        patches["DemodHrpUwbDataNCpb"], patches["DemodHrpUwbDataNHop"] = _NCPB[prf][rate], _NHOP[prf]
    patches.update(json.loads(os.environ.get("SETX_PATCH", "{}")))
    return patches


def write_setx(patches: dict, stem: str) -> str:
    with open(_SETX, encoding="utf-8-sig") as fh:
        text = fh.read()
    for key, value in patches.items():
        text, n = re.subn(r'(key="%s" value=")[^"]*(")' % key, r"\g<1>%s\g<2>" % value, text)
        if n == 0 and key.startswith("DemodHrpUwb"):
            # keys the demo setup omits (STS key/V, ...) are accepted when added next to the other demod keys
            kind = "String" if isinstance(value, str) else "Int32"
            anchor = '<Double key="DemodHrpUwbSymbolRate"'
            text, n = text.replace(anchor, f'<{kind} key="{key}" value="{value}" />\n      {anchor}', 1), \
                text.count(anchor)
        if n != 1:
            raise KeyError(f"{key} found {n} times in {os.path.basename(_SETX)}")
    dst = os.path.join(_TMP, stem + ".setx")
    os.makedirs(_TMP, exist_ok=True)
    with open(dst, "w", encoding="utf-8") as fh:
        fh.write(text)
    return dst


# --------------------------------------------------------------------------- native / Python receivers
def run_native(iq, fs, pr) -> dict:
    from nrdemod import demod

    phy = {"BPRF": 0, "4a": 2, "HPRF": 1}.get(pr["mode"])
    if phy is None:
        return {"native_status": "unsupported (EHPRF not implemented)"}
    try:
        mean_prf = pr["mean_prf"] if pr["mode"] == "4a" else 62.4
        if pr["mode"] == "HPRF":
            mean_prf = 249.6 if pr["hprf_high"] else 124.8
        data = demod.UwbHrpDataConfig(
            codeIndex=pr["code"], sfdNumber=pr["sfd"],
            sfdLength=64 if pr["mode"] == "4a" and pr["data_rate"] == 0.11 else 8,
            preambleSpreadingFactor=pr["delta"], meanPrfMHz=mean_prf,
            phrDataRateKbps=(6810 if pr["pdr"] else 850) if pr["mode"] == "BPRF" else 0,
            stsPacketConfiguration=pr["sp"], constraintLength=pr["constraint_length"],
            numStsSegments=pr["sts_seg_num"], stsSegmentLength=pr["sts_seg_len"])
        res = demod.demodulate(demod.UWB_HRP, iq,
                              demod.Config(sampleRate=fs, phyMode=phy, syncLength=pr["sync"]),
                              data_config=data)
        d = res.as_dict(include_vectors=True)
    except Exception as exc:  # noqa: BLE001
        return {"native_status": f"error {type(exc).__name__}: {exc}"[:200]}
    out = {"native_status": "sync" if d.get("syncFound") else ("burst" if d.get("burstFound") else "no-burst")}
    if not d["ok"]:
        out["native_status"] = "error: " + d["error"]
    if data is not None:
        out["native_data_status"] = (
            ("shr-ok" if d["sfdFound"] else "shr-mismatch") if pr["sp"] == 3 else
            ("decoded" if d["psduDecoded"] and (d["rsOk"] or data.constraintLength == 7) else "failed"))
        out["native_data_error"] = d["dataError"]
        out["native_evm_db"] = d["evmDb"]
        out["native_rs_corrected"] = d["rsCorrectedSymbols"]
        if d["psduDecoded"]:
            out["native_payload_hex"] = bytes(map(int, d["psduBytes"])).hex()
    for k_out, k_in in (("native_burst_start", "burstStart"), ("native_sync_pos", "syncPositionSamples"),
                        ("native_rho", "syncRho"), ("native_cfo_hz", "frequencyErrorHz"),
                        ("native_period", "preambleSymbolPeriodSamples")):
        v = d.get(k_in)
        out[k_out] = float(v) if isinstance(v, (int, float, np.number)) else v
    return out


def run_python(iq, fs, pr) -> dict:
    if pr["mode"] not in ("BPRF", "4a"):
        return {"py_status": "unsupported (" + pr["mode"] + ")"}
    import uwb_hrp as hrp

    cfg = hrp.HrpConfig(
        mode="BPRF" if pr["mode"] == "BPRF" else "802.15.4a", code_index=pr["code"], sfd_number=pr["sfd"],
        mean_prf=pr["mean_prf"] if pr["mode"] == "4a" else 62.4, preamble_duration=pr["sync"],
        preamble_spreading_factor=pr["delta"], sts_packet_configuration=pr["sp"],
        phr_data_rate_kbps=(6810 if pr["pdr"] else 850) if pr["mode"] == "BPRF" else None,
        samples_per_pulse=int(round(fs / hrp.CHIP_RATE_HZ)))
    try:
        r = hrp.demodulate(iq, cfg, fs)
    except Exception as exc:  # noqa: BLE001
        return {"py_status": f"error {type(exc).__name__}: {exc}"[:200]}
    out = {"py_timing": r.timing, "py_cfo_hz": float(r.cfo_hz), "py_sfd_symbol": r.sfd_symbol}
    if pr["sp"] == 3:
        out["py_status"] = "shr-ok" if r.sfd_symbol >= 0 and r.preamble_symbols == pr["sync"] else "shr-mismatch"
        return out
    out["py_evm_db"] = float(r.evm_db) if r.evm_db is not None else None
    if r.psdu_bits is None:
        out["py_status"] = f"no-psdu ({r.status})"
        return out
    psdu = bytes(r.psdu_bytes)
    exp = bytes.fromhex(pr["payload_hex"]) if pr["payload_hex"] else None
    out["py_psdu_len"] = len(psdu)
    out["py_payload_hex"] = psdu.hex()
    out["py_payload_match"] = (psdu == exp) if exp is not None else None
    out["py_status"] = "decoded" if r.ok else f"decoded ({r.status})"
    return out


# --------------------------------------------------------------------------- VSA child
def decoded_payload(raw) -> bytes:
    """Convert VSA's LSB-first decoded octets to the capture's MSB-first convention."""
    values = np.asarray(raw, dtype=float)
    if values.ndim != 1 or not np.all(np.isfinite(values)) or np.any(
            (values < 0) | (values > 255) | (values != np.floor(values))):
        raise ValueError("VSA decoded trace must contain finite integer octets")
    bits = np.unpackbits(values.astype(np.uint8)).reshape(-1, 8)
    return np.packbits(bits[:, ::-1].ravel()).tobytes()


def payload_bit_errors(actual: bytes, expected: bytes) -> int:
    common = sum((a ^ b).bit_count() for a, b in zip(actual, expected))
    return common + 8 * abs(len(actual) - len(expected))


def vsa_measure(item, iq, fs, center, pr) -> dict:
    from vsa_com import Vsa89600Com, VsaCom, _BorrowedTrace

    mat = os.path.join(_TMP, item["stem"] + ".mat")
    os.makedirs(_TMP, exist_ok=True)
    duration = L.write_recording(mat, iq, fs, center)
    duration = max(duration, LEAD_S + iq.size / fs + 40e-6)
    setx = write_setx(vsa_patches(pr, center, fs, iq.size), item["stem"])
    with VsaCom() as link:
        vsa = Vsa89600Com(link)
        meas = link.measurement
        meas.Frequency.Points = 801
        vsa.load_recording(mat)
        meas.RecallFile(setx)
        meas.Frequency.Points = 801
        vsa.load_recording(mat)
        meas.Frequency.Center = center
        span = float(meas.Frequency.Span)
        target = 1.28 * span * duration * 1.05
        for _ in range(60):
            meas.Frequency.Points = int(target)
            if float(meas.Time.MainLen) >= 1.05 * duration:
                break
            target *= 1.08
        meas.Continuous = False
        meas.Initialize()
        meas.Start()
        meas.WaitForMeasDone(300000)
        tables = L.read_tables(link)
        if pr["sp"] != 3:
            with _BorrowedTrace(link.traces, "Demod Decoded Bits1") as trace:
                payload = decoded_payload(trace.RawData(4))
            tables["DecodedPayloadHex"] = payload.hex()
        tables["_span"] = span
        tables["_center"] = float(meas.Frequency.Center)
    return tables


def vsa_child(spec: str, cfo_hz: float = 0.0) -> int:
    threading.Thread(target=lambda: [(L.dismiss_error_dialogs(), time.sleep(1)) for _ in iter(int, 1)],
                     daemon=True).start()
    item = next(it for it in discover() if f"{it['source']}/{it['stem']}" == spec)
    try:
        pr = params(item)
        iq, fs, center = L.parse_waveform(item["txt"])
        iq = apply_cfo(iq, fs, cfo_hz)
        print(json.dumps(vsa_measure(item, iq, fs, center, pr)))
    except Exception as exc:  # noqa: BLE001
        print(json.dumps({"_error": f"{type(exc).__name__}: {exc}"[:300]}))
    return 0


def apply_cfo(iq, fs, cfo_hz):
    return iq if not cfo_hz else iq * np.exp(2j * np.pi * cfo_hz * np.arange(iq.size) / fs)


def run_vsa(item, cfo_hz: float = 0.0) -> dict:
    if params(item)["mode"] == "EHPRF":
        return {"_error": "unsupported: the 89600 HrpUwb personality has no 802.15.4ab EHPRF mode"}
    cmd = [sys.executable, "-u", os.path.abspath(__file__), "--vsa-one", f"{item['source']}/{item['stem']}",
           "--cfo-hz", repr(cfo_hz)]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=CHILD_TIMEOUT_S)
        if result.returncode:
            return {"_error": f"VSA child exit {result.returncode}: {result.stderr[-500:]}"}
        return json.loads(result.stdout.strip().splitlines()[-1])
    except subprocess.TimeoutExpired:
        L.dismiss_error_dialogs()
        return {"_error": f"timeout after {CHILD_TIMEOUT_S}s"}
    except Exception as exc:  # noqa: BLE001
        return {"_error": f"child failed: {exc}"[:300]}


# --------------------------------------------------------------------------- correlation
def correlate(item, pr, fs, burst_start, nat, py, vsa, expected_cfo_hz=0.0) -> dict:
    row = {}
    err = vsa.get("_error")
    required = {"ShrNrmse": NRMSE_SHR_MAX}
    if pr["sp"]:
        required["StsNrmse"] = NRMSE_SHR_MAX
    if pr["sp"] != 3:
        required.update(PhrNrmse=NRMSE_DATA_MAX, PsduNrmse=NRMSE_DATA_MAX)
    vsa_lock = not err and all(
        isinstance(vsa.get(k), (int, float)) and math.isfinite(vsa[k]) and 0 <= vsa[k] <= limit
        for k, limit in required.items())
    row["vsa_lock"] = None if err else vsa_lock
    row["vsa_FreqErr"] = vsa.get("FreqErr")
    row["vsa_TimeOffset"] = vsa.get("TimeOffset")
    expected_hex = pr.get("payload_hex") or py.get("py_payload_hex") or nat.get("native_payload_hex")
    if not err and pr["sp"] != 3 and expected_hex is not None:
        actual_hex = vsa.get("DecodedPayloadHex")
        expected = bytes.fromhex(expected_hex)
        if actual_hex is None:
            row["vsa_payload_match"] = False
            row["vsa_payload_bit_errors"] = None
        else:
            actual = bytes.fromhex(actual_hex)
            row["vsa_payload_match"] = actual == expected
            row["vsa_payload_bit_errors"] = payload_bit_errors(actual, expected)
    for k in ("ShrNrmse", "PhrNrmse", "PsduNrmse", "StsNrmse", "OverallNrmse", "ChipClkErr", "ChPower"):
        row["vsa_" + k] = vsa.get(k)
    nat_lock = nat.get("native_status") == "sync"
    row["native_lock"] = None if "unsupported" in str(nat.get("native_status")) else nat_lock
    checks, notes = [], []
    if pr["mode"] in ("BPRF", "4a", "HPRF"):
        expected_status = "shr-ok" if pr["sp"] == 3 else "decoded"
        checks.append(nat.get("native_data_status") == expected_status and not nat.get("native_data_error"))
        if not checks[-1]:
            notes.append("native packet decode failed: " + str(nat.get("native_data_error") or
                                                              nat.get("native_data_status", "missing result")))
        if pr["sp"] != 3:
            actual_hex = nat.get("native_payload_hex")
            native_expected = pr.get("payload_hex") or py.get("py_payload_hex") or vsa.get("DecodedPayloadHex")
            row["native_payload_match"] = actual_hex is not None and native_expected is not None and (
                bytes.fromhex(actual_hex) == bytes.fromhex(native_expected))
            row["native_payload_bit_errors"] = payload_bit_errors(
                bytes.fromhex(actual_hex), bytes.fromhex(native_expected)) if (
                    actual_hex is not None and native_expected is not None) else None
            if native_expected is None and pr["mode"] == "HPRF" and vsa.get("_error") == "VSA skipped (--no-vsa)":
                row["native_payload_match"] = None
                notes.append("native HPRF payload decoded; independent payload reference unavailable without VSA")
            else:
                checks.append(row["native_payload_match"])
            if not checks[-1]:
                notes.append("native PSDU missing or differs from transmitter/Python payload")
    if row.get("vsa_payload_match") is False:
        checks.append(False)
        notes.append(f"VSA PSDU missing or mismatched ({row['vsa_payload_bit_errors']} bit errors)")
    if row["native_lock"] is not None:
        checks.append(nat_lock)
        if not nat_lock:
            notes.append(f"native receiver failed: {nat.get('native_status', 'no result')}")
    if row["native_lock"] is not None and row["vsa_lock"] is not None:
        checks.append(nat_lock and vsa_lock)
        if nat_lock != vsa_lock:
            notes.append(f"lock disagrees (native {nat_lock}, VSA {vsa_lock})")
        elif not nat_lock:
            notes.append("neither receiver acquired the frame")
    if err and not str(err).startswith(("unsupported:", "VSA skipped")):
        checks.append(False)
    if not err:
        invalid = [k for k in required if not isinstance(vsa.get(k), (int, float))
                   or not math.isfinite(vsa[k]) or vsa[k] < 0]
        if invalid:
            notes.append("missing/invalid VSA metrics: " + ", ".join(invalid))
    py_status = py.get("py_status", "")
    if pr["mode"] in ("4a", "BPRF"):
        checks.append(py_status == ("shr-ok" if pr["sp"] == 3 else "decoded"))
        if not checks[-1]:
            notes.append(f"Python receiver failed: {py_status or 'no result'}")
    valid_cfo = {}
    for label, value, active in (
            ("native", nat.get("native_cfo_hz"), nat_lock),
            ("VSA", vsa.get("FreqErr"), vsa_lock),
            ("Python", py.get("py_cfo_hz"), py_status in ("decoded", "shr-ok"))):
        valid = isinstance(value, (int, float)) and math.isfinite(value)
        valid_cfo[label] = active and valid
        if active:
            checks.append(valid and abs(value - expected_cfo_hz) <= CFO_TOL_HZ)
            if not checks[-1]:
                notes.append(f"{label} CFO {value} differs from injected {expected_cfo_hz:g} Hz")
    if valid_cfo["native"] and valid_cfo["VSA"]:
        row["delta_cfo_hz"] = nat["native_cfo_hz"] - row["vsa_FreqErr"]
        checks.append(abs(row["delta_cfo_hz"]) <= CFO_TOL_HZ)
        if abs(row["delta_cfo_hz"]) > CFO_TOL_HZ:
            notes.append(f"CFO native {nat['native_cfo_hz']:.0f} Hz vs VSA {row['vsa_FreqErr']:.0f} Hz")
    if valid_cfo["Python"] and valid_cfo["VSA"]:
        row["delta_py_vsa_cfo_hz"] = py["py_cfo_hz"] - row["vsa_FreqErr"]
        checks.append(abs(row["delta_py_vsa_cfo_hz"]) <= CFO_TOL_HZ)
    if vsa_lock and row["vsa_TimeOffset"] is not None and burst_start is not None:
        # time of the first preamble chip from the waveform envelope vs the VSA's time offset (recording has LEAD_S)
        row["wave_burst_s"] = burst_start / fs
        row["delta_timing_s"] = row["vsa_TimeOffset"] - (LEAD_S + burst_start / fs)
    if py.get("py_payload_match") is False or (
            pr.get("payload_hex") and pr["sp"] != 3 and pr["mode"] in ("BPRF", "4a")
            and py.get("py_payload_match") is not True):
        checks.append(False)
        notes.append("Python PSDU missing or differs from PayloadHex")
    unavailable = err and str(err).startswith(("unsupported:", "VSA skipped"))
    row["pass"] = None if unavailable and all(checks) else (all(checks) if checks else None)
    if err:
        notes.append(err)
    if not vsa_lock and not err:
        notes.extend(note for cond, note in _KNOWN_VSA_GAPS if cond(pr, fs))
    row["note"] = "; ".join(notes)
    return row


# Observed failures, not exemptions from the pass criteria.
_KNOWN_VSA_GAPS = (
    (lambda pr, fs: pr["mode"] == "HPRF" and pr["generator"] == "matlab"
     and abs(pr["mean_prf"] - 124.8) < 1 and abs(fs - 998.4e6) < 1,
     "incorrect legacy export clock: HPRF124M8 mapper already inserts half-rate pulse gaps; "
     "regenerate with gen_uwb to use the 499.2 MHz chip grid"),
    (lambda pr, fs: pr["mode"] == "4a" and pr["generator"] == "matlab",
     "unresolved: installed 89600 NonErdev reference hopping is one PN chip behind MATLAB/Python "
     "for NCpb=2, NHop=8 (944/944 reference symbols verified); native/Python payloads match PayloadHex"),
)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--match", action="append", help="substring filter on the waveform name (repeatable)")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--no-vsa", action="store_true", help="skip the 89600")
    ap.add_argument("--skip-unsupported", action="store_true",
                    help="exclude EHPRF captures (not implemented by the local receivers or VSA personality)")
    ap.add_argument("--no-restart", action="store_true", help="use the running 89600 instead of restarting it")
    ap.add_argument("--cfo-hz", type=float, default=0.0,
                    help="frequency offset applied to every waveform before all receivers (|f| < ~490 kHz)")
    ap.add_argument("--vsa-one", help="internal: measure one source/stem on the VSA and print the tables as JSON")
    ap.add_argument("--csv", default=os.path.join(_OUT, "uwb_correlation.csv"))
    args = ap.parse_args(argv)
    if not math.isfinite(args.cfo_hz) or abs(args.cfo_hz) >= 490e3:
        ap.error("--cfo-hz must be finite and have magnitude below 490000 Hz")

    if args.vsa_one:
        return vsa_child(args.vsa_one, args.cfo_hz)
    items = discover(args.match)
    skipped = 0
    if args.skip_unsupported:
        supported = [it for it in items if params(it)["mode"] != "EHPRF"]
        skipped = len(items) - len(supported)
        items = supported
        if skipped:
            print(f"skipping {skipped} unsupported EHPRF waveforms", flush=True)
    if args.list:
        for it in items:
            pr = params(it)
            print(f"{it['source']:9s} {pr['mode']:5s} {it['stem']}")
        print(f"{len(items)} waveforms")
        return 0
    if not items:
        if skipped:
            print("no supported UWB waveforms selected")
            return 0
        print("no UWB waveforms found; run tools/uwb/gen_uwb.m in MATLAB to generate captures/generated/uwb")
        return 1
    if not args.no_vsa and not args.no_restart:
        print("restarting the 89600 ...", flush=True)
        L.restart_vsa()

    cols = ["source", "waveform", "mode", "pass", "native_lock", "vsa_lock", "native_status", "py_status",
            "native_data_status", "native_data_error", "native_payload_hex", "native_payload_match",
            "native_payload_bit_errors", "native_evm_db", "native_rs_corrected",
            "cfo_injected_hz", "native_cfo_hz", "py_cfo_hz", "vsa_FreqErr", "delta_cfo_hz", "delta_py_vsa_cfo_hz",
            "native_sync_pos", "native_rho",
            "py_timing", "wave_burst_s", "vsa_TimeOffset", "delta_timing_s", "py_evm_db", "py_payload_match",
            "py_payload_hex", "vsa_payload_match", "vsa_payload_bit_errors",
            "vsa_ShrNrmse", "vsa_PhrNrmse", "vsa_PsduNrmse", "vsa_StsNrmse", "vsa_OverallNrmse",
            "vsa_ChipClkErr", "vsa_ChPower", "seconds", "note", "vsa_all"]
    os.makedirs(os.path.dirname(os.path.abspath(args.csv)), exist_ok=True)
    rows = []
    with open(args.csv, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for it in items:
            t0 = time.time()
            pr = params(it)
            iq, fs, _center = L.parse_waveform(it["txt"])
            iq = apply_cfo(iq, fs, args.cfo_hz)
            on = np.flatnonzero(np.abs(iq) > 0.1 * np.abs(iq).max())
            burst_start = int(on[0]) if on.size else None
            print(f"{it['source']}/{it['stem']} ({pr['mode']}, {iq.size} samples)", flush=True)
            nat = run_native(iq, fs, pr)
            py = run_python(iq, fs, pr)
            vsa = {"_error": "VSA skipped (--no-vsa)"} if args.no_vsa else run_vsa(it, args.cfo_hz)
            row = {"source": it["source"], "waveform": it["stem"], "mode": pr["mode"], "cfo_injected_hz": args.cfo_hz,
                   **nat, **py}
            row.update(correlate(it, pr, fs, burst_start, nat, py, vsa, args.cfo_hz))
            row["vsa_all"] = json.dumps({k: v for k, v in vsa.items() if not k.startswith("_")})
            row["seconds"] = round(time.time() - t0, 1)
            w.writerow(row)
            fh.flush()
            rows.append(row)
            print(f"  native {row.get('native_status')}/{row.get('native_data_status')} cfo={row.get('native_cfo_hz')} | python {row.get('py_status')}"
                  f" evm={row.get('py_evm_db')} | VSA lock={row.get('vsa_lock')} FreqErr={row.get('vsa_FreqErr')}"
                  f" PSDU NRMSE={row.get('vsa_PsduNrmse')} | pass={row['pass']} {row['note']}", flush=True)
    n_pass = sum(r["pass"] is True for r in rows)
    n_fail = sum(r["pass"] is False for r in rows)
    print(f"\n{n_pass} pass, {n_fail} fail, {len(rows) - n_pass - n_fail} not comparable -> {args.csv}")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
