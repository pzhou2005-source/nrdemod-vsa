"""Correlate the 89600 VSA with the TDC reference results for the non-5GNR waveforms.

Each plaintext waveform in captures/tdc_reference_plaintext is padded, written as a .mat
recording, played through the running 89600 (local COM) and the VSA summary table is compared
with the scalar RESULT entries of the matching captures/tdc_reference/*.para2. WLAN is driven through the
COM demod objects; UWB and NB-IoT recall a patched demo setup, restarting the VSA before each family.

    python tools/legacy_correlate.py --list
    python tools/legacy_correlate.py --family 802.11a --limit 2
    python tools/legacy_correlate.py --resume
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import subprocess
import sys
import threading
import time

import numpy as np
from scipy.io import savemat

from vsa_com import Vsa89600Com, VsaCom, _BorrowedTrace

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, ".."))
_PLAIN = os.path.join(_ROOT, "captures", "tdc_reference_plaintext")
_REF = os.path.join(_ROOT, "captures", "tdc_reference")
_OUT = os.path.join(_ROOT, "reports")
_SIGNALS = r"C:\Program Files\Keysight\89600 Software 2026_U2\89600 VSA Software\Help\Signals"
_VSA_EXE = r"C:\Program Files\Keysight\89600 Software 2026_U2\89600 VSA Software\Agilent.SA.Vsa.Vector-x64.exe"

_DEMOD_OFDM, _DEMOD_DSSS, _DEMOD_OFDM11N, _DEMOD_DIGITAL = 5, 6, 13, 2
BLUETOOTH = ("BluetoothBdr", "BluetoothEdr", "BluetoothLe1M", "BluetoothLe2M", "BluetoothLeCodedS8")
DEV_TOL_PCT = 5.0  # GFSK deviation: the VSA's FskDev against the reference's DeltaFAvg

# family dir -> how the VSA is put into the right personality
FAMILIES = {
    "802.11a": {"demod": _DEMOD_OFDM},
    "802.11b": {"demod": _DEMOD_DSSS},
    "802.11n": {"demod": _DEMOD_OFDM11N},
    "802.11ac": {"demod": _DEMOD_OFDM11N},
    "802.15.4z_HrpUwb": {"setx": os.path.join(_SIGNALS, "HrpUwb", "HrpUwb_Ch9_Code10_L4_Sync64_Sfd8_33octet.setx")},
    "NbIotDl": {"setx": os.path.join(_SIGNALS, "NBIoT", "NBIoT_DL_StandAlone.setx")},
    "NbIotUl": {"setx": os.path.join(_SIGNALS, "NBIoT", "NBIoT_UL_NPUSCH_1T_2RU_2Rep.setx")},
    **{d: {"demod": _DEMOD_DIGITAL} for d in BLUETOOTH},
}
UNSUPPORTED = {}
# plugin personalities are reached by recalling a demo setup (patched per waveform)
SETX_FAMILIES = ("802.15.4z_HrpUwb", "NbIotDl", "NbIotUl")

# para2 RESULT name -> VSA table entries, first hit wins
COMPARE = {
    "EvmRms": ("EVM", "EvmRms", "EVMrms"),
    "EvmPeak": ("EVMPk", "EvmPeak"),
    "FrequencyError": ("FreqErr", "FrequencyError"),
    "IQOffset": ("IQOffset", "IqOffset"),
    "IQQuadError": ("IQQuadErr", "QuadErr"),
    "IQGainImbalance": ("IQGainImb", "IqGainImbalance"),
    "SyncRho": ("SyncCorr", "SyncRho"),
    "DeltaFAvg": ("FskDev",),
}
EVM_TOL_PP = 0.1  # percent points, the references are near-ideal waveforms
CHILD_TIMEOUT_S = 900  # the HPRF UWB files are 100 MB of text

_SUMMARY_RE = re.compile(r"(Syms/Errs|(?<!Acp )(?<!Obw )(?<!CCDF )Summary)\d*$")


def parse_waveform(path: str):
    with open(path, "rb") as fh:
        blob = fh.read()
    cut = blob.index(b"\nY\n") + 3
    hdr = {}
    for line in blob[:cut].decode("ascii", "replace").splitlines():
        key, _, val = line.partition("\t")
        hdr[key.strip()] = val.strip()
    flat = np.fromstring(blob[cut:].decode("ascii"), dtype=np.float64, sep=" ")
    iq = flat[0::2] + 1j * flat[1::2]
    return iq, 1.0 / float(hdr["XDelta"]), float(hdr.get("InputCenter", 0.0))


def parse_para2(path: str) -> dict:
    out = {}
    with open(path, "r", errors="replace") as fh:
        for line in fh:
            parts = [p.strip() for p in line.split(",")]
            if len(parts) >= 4 and parts[0] == "RESULT" and parts[2] in ("Double", "Int", "Bool"):
                try:
                    out[parts[1]] = float(parts[3])
                except ValueError:
                    pass
    return out


def discover(family_filter=None):
    items = []
    for family in sorted(os.listdir(_PLAIN)):
        if family_filter and family not in family_filter:
            continue
        fdir = os.path.join(_PLAIN, family)
        for name in sorted(os.listdir(fdir)):
            if name.endswith(".txt"):
                stem = name[:-4]
                items.append((family, stem, os.path.join(fdir, name), os.path.join(_REF, family, stem + ".para2")))
    return items


_RESULT_LEN = {}  # EDR payload symbols for the VSA, set by run_one


def _burst(iq):
    a = np.abs(iq)
    return np.flatnonzero(a > 0.1 * a.max())


def waveform_deviation(iq, fs) -> float:
    """Peak GFSK frequency deviation in Hz read straight off the (ideal) waveform."""
    on = _burst(iq)
    seg = iq[on[0] + int(8e-6 * fs): on[-1] - int(8e-6 * fs)]
    f = np.diff(np.unwrap(np.angle(seg))) * fs / (2 * np.pi)
    k = max(int(fs / 1e6 / 4), 1)
    f = np.convolve(f, np.ones(k) / k, mode="valid")
    return float((np.percentile(f, 99.5) - np.percentile(f, 0.5)) / 2)


def _rrc(alpha, sps, span):
    t = np.arange(-span * sps, span * sps + 1) / sps
    with np.errstate(all="ignore"):
        h = (np.sin(np.pi * t * (1 - alpha)) + 4 * alpha * t * np.cos(np.pi * t * (1 + alpha))) / (np.pi * t * (1 - (4 * alpha * t) ** 2))
    h[np.abs(t) < 1e-9] = 1 - alpha + 4 * alpha / np.pi
    sing = np.abs(np.abs(t) - 1 / (4 * alpha)) < 1e-9
    h[sing] = alpha / np.sqrt(2) * ((1 + 2 / np.pi) * np.sin(np.pi / (4 * alpha)) + (1 - 2 / np.pi) * np.cos(np.pi / (4 * alpha)))
    return h / np.sqrt(np.sum(h ** 2))


def edr_payload(iq, fs, psk_order):
    """Crops the DPSK payload (the GFSK header has a flat envelope) and measures its EVM independently of the VSA.

    Returns (cropped samples, EVM percent from the differential phase error, VSA result length in symbols).
    """
    sps = int(round(fs / 1e6))
    a = np.abs(iq)
    on = _burst(iq)
    flat = np.abs(a[on[0] + 50:] - a[on[0] + 100]) < 1e-3
    brk = on[0] + 50 + int(np.argmin(flat))
    start = brk + int(25e-6 * fs)  # past the guard time and the DPSK sync word
    y = np.convolve(iq[brk: on[-1] + 1], _rrc(0.4, sps, 6), mode="same")
    best = np.inf
    for off in range(sps):
        ph = np.angle(y[off::sps][1:] * np.conj(y[off::sps][:-1]))
        if psk_order == 4:
            q = np.round((ph - np.pi / 4) / (np.pi / 2)) * (np.pi / 2) + np.pi / 4
        else:
            q = np.round(ph / (np.pi / 4)) * (np.pi / 4)
        best = min(best, float(np.sqrt(np.mean(np.angle(np.exp(1j * (ph - q)))[10:-10] ** 2))))
    crop = iq[start - int(2e-6 * fs): on[-1] + int(2e-6 * fs)]
    return crop, 100 * best / np.sqrt(2), int((on[-1] - start) / sps) - 24  # keep clear of the ramp-down symbols


def write_recording(path: str, iq: np.ndarray, fs: float, center: float) -> float:
    # The VSA measures the first MainLen of the recording, so the burst sits right after a short idle lead-in.
    # MainLen snaps to a coarse grid and has a minimum length; the recording is three times the needed length so
    # the snapped value never overshoots (that raises a modal "recording is not long enough" dialog).
    lead = 100e-6
    need = max(lead + iq.size / fs, 1.5e-3)
    pre = int(round(lead * fs))
    post = int(round(3 * need * fs)) - pre - iq.size
    rng = np.random.default_rng(1)
    noise = lambda n: 1e-6 * (rng.standard_normal(n) + 1j * rng.standard_normal(n))  # noqa: E731
    padded = np.concatenate([noise(pre), iq, noise(post)])
    savemat(path, {"Y": padded.reshape(-1, 1), "XStart": 0.0, "XDelta": 1.0 / fs, "XDomain": 2, "XUnit": "Sec",
                   "YUnit": "V", "InputCenter": center, "InputZoom": 1, "InputRefImped": 50.0}, do_compression=False)
    return need


def read_tables(link) -> dict:
    values = {}
    names = set(link.traces.Item(1).DataNames)
    for table in sorted(n for n in names if _SUMMARY_RE.search(n)):
        try:
            with _BorrowedTrace(link.traces, table) as trace:
                entries = list(trace.RawDataValueNames)
                for i, entry in enumerate(entries):
                    try:
                        values[entry if entry not in values else f"{table}:{entry}"] = float(trace.RawDataValue(i))
                    except (TypeError, ValueError):
                        continue
        except Exception:  # noqa: BLE001 - table not readable in this personality
            continue
    return values


def apply_demod_settings(meas, family, stem) -> None:
    if family == "802.11b":
        d = meas.DigDemodDSSS
        d.Equalize, d.TrackPhase, d.RefFilter = True, False, 0  # the reference run: EQ on, no phase tracking, rect
    elif family in ("802.11n", "802.11ac"):
        mhz = int(re.search(r"_(\d+)MHz_", stem).group(1))
        d = meas.DigDemodOFDM11n
        d.Standard = {20: 0, 40: 1, 80: 2, 160: 4}[mhz]  # Standard also sets the FFT size
        d.EqualizerTraining = 1  # the reference run trains the equaliser on channel estimate plus data
        if family == "802.11ac":
            # take modulation, length, guard interval and stream count from the VHT-SIG field
            d.SubCarrierModType = d.ResultLenType = d.GuardIntervalType = d.StreamsType = 2
    elif family in BLUETOOTH:
        d = meas.DigDemod
        if family == "BluetoothEdr":  # DPSK payload: 2DH = pi/4-DQPSK, 3DH = D8PSK, root-raised-cosine filters
            d.Format = 6 if "_2DH" in stem else 9  # this VSA rejects D8PSK (23); plain 8PSK still reads the 8 states
            d.MeasFilter, d.RefFilter, d.FilterAlpha = 2, 1, 0.4  # RRC receive filter against a raised-cosine reference
            d.ResultLen = _RESULT_LEN["n"]  # stop before the burst-end ramp symbols
        else:  # GFSK, Gaussian BT 0.5 reference
            d.Format = 12
            d.MeasFilter, d.RefFilter, d.FilterAlpha = 0, 3, 0.5
            d.ResultLen = _RESULT_LEN["n"]
        d.SymbolRate = 2e6 if family == "BluetoothLe2M" else 1e6
        d.PulseSearch = True  # without it the demod never finds the short burst


def patch_setx(family, stem, src) -> str:
    """Copies a demo setup with the keys the waveform name (or its reference) dictates."""
    patches = {}
    if family == "802.15.4z_HrpUwb":
        m = re.match(r"802154HrpUwb_(BPRF|HPRF)_SP(\d)_Code(\d+)_Sync(\d+)_Delta(\d)_SFD(\d)(?:_PDR-\w)?(?:_FrameLeng(\d+))?", stem)
        mode, sp, code, _sync, _delta, sfd, octets = m.groups()
        patches = {"DemodHrpUwbPhyMode": 1 if mode == "BPRF" else 2, "DemodHrpUwbStsPacketConfig": sp,
                   "DemodHrpUwbSHRCodeIndex": code, "DemodHrpUwbSHRSFDSelector": sfd,
                   "DemodHrpUwbDataNOctets": octets or 0}
    if family in ("NbIotDl", "NbIotUl"):
        patches = {"DemodNBIoTEqTraining": 3}  # the reference run's equaliser training
    if family == "NbIotUl":
        # the UL waveforms are 12-tone NPUSCH, the demo setup is single tone
        patches.update({"DemodNBIoTNumTones": 3, "DemodNBIoTNpuschScOffset": 0})
    patches.update(json.loads(os.environ.get("SETX_PATCH", "{}")))  # experiment hook: extra setup keys as JSON
    if not patches:
        return src
    with open(src, encoding="utf-8-sig") as fh:
        text = fh.read()
    for key, value in patches.items():
        text, n = re.subn(r'(key="%s" value=")[^"]*(")' % key, r"\g<1>%s\g<2>" % value, text)
        if n != 1:
            raise KeyError(f"{key} found {n} times in {os.path.basename(src)}")
    dst = os.path.join(_OUT, "legacy_tmp", f"{family}_{stem}.setx")
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    with open(dst, "w", encoding="utf-8") as fh:
        fh.write(text)
    return dst


def measure(link, vsa, family, stem, mat, duration, state) -> dict:
    meas = link.measurement
    cfg = FAMILIES[family]
    meas.Frequency.Points = 801  # a stale long time record trips "recording is not long enough" on any later change
    if "setx" in cfg:
        if state.get("family") != family:
            setx = patch_setx(family, stem, cfg["setx"])
            print(f"  recalling {os.path.basename(setx)}", flush=True)
            meas.RecallFile(setx)
            meas.Frequency.Points = 801  # the setup brings its own long time record, too long for our recording
    else:
        # RecallFile of a recording blocks after a finished measurement unless the demod is re-selected
        meas.DemodConfig = 0
        meas.DemodConfig = cfg["demod"]
    state["family"] = family
    apply_demod_settings(meas, family, stem)
    print("  loading recording", flush=True)
    vsa.load_recording(mat)
    span = float(meas.Frequency.Span)
    target = 1.28 * span * duration * 1.05
    for _ in range(60):
        meas.Frequency.Points = int(target)
        main_len = float(meas.Time.MainLen)
        if main_len >= 1.05 * duration:
            break
        target *= 1.08
    print(f"  measuring (span {span / 1e6:.3f} MHz, points {meas.Frequency.Points}, "
          f"main {main_len * 1e3:.3f} ms, needed {duration * 1e3:.3f} ms)", flush=True)
    meas.Continuous = False
    meas.Initialize()
    meas.Start()
    meas.WaitForMeasDone(300000)
    tables = read_tables(link)
    if family == "BluetoothEdr":
        # the VSA's pulse search can end the burst early on a deep envelope dip; symbols past that point read ~100 % EVM
        peak, sym = tables.get("EvmPeak"), tables.get("EvmPeakSym")
        if peak and peak > 50 and sym and sym > 0.9 * _RESULT_LEN["n"]:
            meas.DigDemod.ResultLen = int(sym) - 12
            print(f"  burst ended early, retrying with {int(sym) - 12} symbols", flush=True)
            meas.Initialize()
            meas.Start()
            meas.WaitForMeasDone(300000)
            tables = read_tables(link)
    return tables


def compare(ref: dict, vsa: dict, stem: str = "") -> dict:
    row = {}
    for key, aliases in COMPARE.items():
        got = next((vsa[a] for a in aliases if a in vsa), None)
        row["ref_" + key] = ref.get(key)
        row["vsa_" + key] = got
    if stem.startswith("802154HrpUwb"):
        # the .para2 carry inputs only; the waveform is ideal, so a correct lock means the data fields decode
        # with a tiny NRMSE (the SHR/STS NRMSE stays ~0.3 with the reference pulse filter) and no frequency error
        data = [vsa[k] for k in ("PhrNrmse", "PsduNrmse") if vsa.get(k)]
        pre = [vsa[k] for k in ("ShrNrmse", "StsNrmse") if vsa.get(k)]
        ok = bool(pre) and all(x <= 0.5 for x in pre) and all(x <= 0.02 for x in data) \
            and abs(vsa.get("FreqErr", 0)) <= 100
        row["pass"] = ok
        row["note"] = ("ideal waveform: VSA locks, PHR/PSDU NRMSE <= 2 %" if data else
                       "ideal waveform: VSA locks on SHR/STS (no payload)") if ok else "VSA did not lock"
        return row
    if ref.get("BurstFound") == 0:  # the reference run itself found no burst, its values are placeholders
        row["delta_evm_pp"], row["pass"], row["note"] = None, None, "reference run found no burst"
        return row
    dev_ref, dev = ref.get("DeltaFAvg"), row["vsa_DeltaFAvg"]
    if dev_ref:
        if dev is None or dev > 5 * dev_ref:  # an unlocked FSK demod reports MHz-scale deviation
            row["pass"], row["note"] = False, "VSA did not lock"
        else:
            row["delta_dev_pct"] = (dev / dev_ref - 1) * 100
            row["pass"] = abs(row["delta_dev_pct"]) <= DEV_TOL_PCT
            row["note"] = ref.get("_note", "")
        return row
    r, v = ref.get("EvmRms"), row["vsa_EvmRms"]
    sync = next((vsa[a] for a in COMPARE["SyncRho"] if a in vsa), None)
    if r is None:  # e.g. the UWB .para2 files carry inputs only
        row["delta_evm_pp"], row["pass"], row["note"] = None, None, "no reference result in the .para2"
        return row
    if v is None or (v == 0.0 and not sync):  # an all-zero table means the demod never found the burst
        row["delta_evm_pp"], row["pass"], row["note"] = None, False, "VSA did not lock"
        return row
    row["delta_evm_pp"] = v - r
    row["pass"] = None if row["delta_evm_pp"] is None else bool(abs(row["delta_evm_pp"]) <= (1.0 if stem.startswith("BT_EDR") else EVM_TOL_PP))
    row["note"] = ref.get("_note", "")
    return row


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--family", action="append", help="restrict to this directory name (repeatable)")
    p.add_argument("--limit", type=int, help="max waveforms per family")
    p.add_argument("--list", action="store_true")
    p.add_argument("--resume", action="store_true", help="skip waveforms already in the CSV")
    p.add_argument("--only", help="internal: measure one family/stem and print its row as JSON")
    p.add_argument("--csv", default=os.path.join(_OUT, "legacy_correlation.csv"))
    args = p.parse_args(argv)

    items = discover(set(args.family) if args.family else None)
    unsupported = dict(UNSUPPORTED)
    if args.limit:
        seen, kept = {}, []
        for it in items:
            seen[it[0]] = seen.get(it[0], 0) + 1
            if seen[it[0]] <= args.limit:
                kept.append(it)
        items = kept
    if args.list:
        for family, stem, _, _ in items:
            status = "unsupported: " + unsupported[family] if family in unsupported else "vsa"
            print(f"{family}\t{stem}\t{status}")
        print(f"{len(items)} waveforms")
        return 0

    if args.only:
        return run_one(args.only, unsupported)

    done = set()
    if args.resume and os.path.exists(args.csv):
        with open(args.csv, newline="") as fh:
            done = {(r["family"], r["waveform"]) for r in csv.DictReader(fh)}
    cols = ["family", "waveform", "status", "seconds", "delta_evm_pp", "delta_dev_pct", "pass"]
    cols += [f"{s}_{k}" for k in COMPARE for s in ("ref", "vsa")] + ["vsa_all", "note"]
    new = not (args.resume and os.path.exists(args.csv))
    os.makedirs(_OUT, exist_ok=True)
    with open(args.csv, "w" if new else "a", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
        if new:
            writer.writeheader()
        last_family = None
        for family, stem, _wav, _para in items:
            if (family, stem) in done:
                continue
            t0 = time.time()
            if family in unsupported:
                row = {"family": family, "waveform": stem, "status": "unsupported", "note": unsupported[family]}
            else:
                if family != last_family and (family in SETX_FAMILIES or last_family in SETX_FAMILIES):
                    restart_vsa()  # a recalled plugin setup leaves the VSA unable to lock later COM-driven demods
                last_family = family
                # one process per waveform: a long-lived COM session stalled in Recording.RecallFile
                cmd = [sys.executable, "-u", os.path.abspath(__file__), "--only", f"{family}/{stem}"]
                try:
                    out = subprocess.run(cmd, capture_output=True, text=True, timeout=CHILD_TIMEOUT_S).stdout
                    row = json.loads(out.strip().splitlines()[-1])
                except subprocess.TimeoutExpired:
                    dismiss_error_dialogs()
                    row = {"family": family, "waveform": stem, "status": "timeout",
                           "note": f"VSA did not answer within {CHILD_TIMEOUT_S}s"}
                except Exception as exc:  # noqa: BLE001
                    row = {"family": family, "waveform": stem, "status": "error", "note": f"child failed: {exc}"[:300]}
            row["seconds"] = round(time.time() - t0, 1)
            writer.writerow(row)
            fh.flush()
            print(f"{family}/{stem}: {row['status']} dEVM={row.get('delta_evm_pp')} pass={row.get('pass')} "
                  f"({row['seconds']}s)", flush=True)
    return 0


def restart_vsa(timeout_s: int = 240) -> None:
    """Kills and relaunches the 89600 and waits until its COM object is registered."""
    subprocess.run(["taskkill", "/F", "/IM", os.path.basename(_VSA_EXE)], capture_output=True)
    subprocess.Popen([_VSA_EXE])
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        time.sleep(5)
        try:
            VsaCom().open()
            return
        except Exception:  # noqa: BLE001 - not registered yet
            continue
    raise RuntimeError("the 89600 did not come up")


def dismiss_error_dialogs() -> None:
    """Closes the modal 'Keysight 89600 VSA : Error' boxes that otherwise block every COM call."""
    import ctypes
    from ctypes import wintypes

    user32 = ctypes.windll.user32
    proc = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)

    def visit(hwnd, _):
        buf = ctypes.create_unicode_buffer(256)
        user32.GetWindowTextW(hwnd, buf, 256)
        if user32.IsWindowVisible(hwnd) and buf.value.endswith("VSA : Error"):
            user32.PostMessageW(hwnd, 0x0010, 0, 0)  # WM_CLOSE
        return True

    user32.EnumWindows(proc(visit), 0)


def run_one(spec: str, unsupported: dict) -> int:
    # a setup recall can pop "recording is not long enough" for the previous recording; it only needs an OK
    threading.Thread(target=lambda: [(dismiss_error_dialogs(), time.sleep(1)) for _ in iter(int, 1)], daemon=True).start()
    family, stem = spec.split("/", 1)
    wav = os.path.join(_PLAIN, family, stem + ".txt")
    para = os.path.join(_REF, family, stem + ".para2")
    row = {"family": family, "waveform": stem}
    try:
        ref = parse_para2(para) if os.path.exists(para) else {}
        iq, fs, center = parse_waveform(wav)
        if family in BLUETOOTH:
            # the TDC runs are unusable (burst not found) or off by ~3 %: the waveforms are ideal, so read the truth off them
            ref = dict(ref, BurstFound=1)
            if family == "BluetoothEdr":
                iq, ref["EvmRms"], _RESULT_LEN["n"] = edr_payload(iq, fs, 4 if "_2DH" in stem else 8)
                ref["_note"] = "reference EVM measured from the ideal waveform (numpy differential demod)"
            else:
                ref["DeltaFAvg"] = waveform_deviation(iq, fs)
                on = _burst(iq)
                _RESULT_LEN["n"] = int((on[-1] - on[0]) / fs * (2e6 if family == "BluetoothLe2M" else 1e6)) - 24
                ref["_note"] = "reference deviation measured from the ideal waveform"
        # a fresh file name each time: the VSA keeps reading the loaded recording from disk
        mat = os.path.join(_OUT, "legacy_tmp", f"{family}_{stem}.mat")
        os.makedirs(os.path.dirname(mat), exist_ok=True)
        duration = write_recording(mat, iq, fs, center)
        with VsaCom() as link:
            vsa = Vsa89600Com(link)
            tables = measure(link, vsa, family, stem, mat, duration, {})
        row.update(compare(ref, tables, stem))
        row["vsa_all"] = json.dumps(tables)
        row["status"] = "ok" if tables else "no-results"
    except Exception as exc:  # noqa: BLE001
        row.update(status="error", note=f"{type(exc).__name__}: {exc}"[:300])
    print(json.dumps(row))
    return 0


if __name__ == "__main__":
    sys.exit(main())
