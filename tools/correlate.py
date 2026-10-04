#!/usr/bin/env python3
"""Correlate the native nrdemod core against a Keysight 89600 VSA.

Three modes:

  live      talk to the 89600 SCPI server, trigger one measurement, pull the
            time record and the VSA's own EVM, demodulate the same samples
            with nrdemod and compare.

  file      use a recording already exported from the VSA (.mat/.csv/.npy/.bin)
            plus the EVM you read off the VSA screen (--vsa-evm).

  selftest  demodulate an ideal synthetic burst; proves the toolchain works
            before any instrument is involved.

Examples
--------
  python tools\\correlate.py selftest
  python tools\\correlate.py live --host 127.0.0.1 --config configs\\fr1_100mhz_mu1_64qam.json
  python tools\\correlate.py file captures\\run01.mat --config configs\\fr1_100mhz_mu1_64qam.json --vsa-evm 1.42
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "nrdemod", "python"))

import nrdemod  # noqa: E402
from nrdemod import vsa as vsa_io  # noqa: E402

from report import write_reports  # noqa: E402
from vsa_com import ComError, Vsa89600Com, VsaCom  # noqa: E402
from vsa_link import CommandMap, ScpiError, Vsa89600, VsaScpi  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, ".."))

_MODS = {
    "auto": nrdemod.AUTO,
    "qpsk": nrdemod.QPSK,
    "16qam": nrdemod.QAM16,
    "64qam": nrdemod.QAM64,
    "256qam": nrdemod.QAM256,
}


def load_case(path: str) -> dict:
    with open(path, "r") as fh:
        return json.load(fh)


def build_config(case: dict, sample_rate: float = None) -> nrdemod.Config:
    raw = dict(case.get("demod", {}))
    mod = raw.pop("modulation", "auto")
    cfg = nrdemod.Config(**raw)
    cfg.modulation = _MODS[str(mod).lower()] if isinstance(mod, str) else int(mod)
    if sample_rate:
        cfg.sampleRate = float(sample_rate)
    return cfg


_INT_KEYS = {
    "numResourceBlocks",
    "carrierResourceBlocks",
    "numSymbols",
    "numSlots",
    "fftSize",
    "timingOffset",
    "burstStart",
    "burstLength",
}


def summarise(res) -> dict:
    keys = (
        "rmsEvmPercent",
        "peakEvmPercent",
        "rmsEvmDb",
        "frequencyErrorHz",
        "numResourceBlocks",
        "carrierResourceBlocks",
        "numSymbols",
        "numSlots",
        "fftSize",
        "timingOffset",
        "burstStart",
        "burstLength",
        "dataPowerDb",
        "burstPowerDb",
        "iqOffsetDb",
        "iqGainImbalanceDb",
        "iqQuadratureErrorDeg",
        "commonPhaseErrorDeg",
        "dmrsEvmPercent",
        "flatnessRippleRange1Db",
        "flatnessRippleRange2Db",
    )
    out = {k: (int(getattr(res, k)) if k in _INT_KEYS else float(getattr(res, k))) for k in keys}
    out["modulation"] = res.modulation_name
    return out


def verdict(nrd: dict, vsa_evm, vsa_freq_err, tol: dict) -> dict:
    out = {"vsaEvmPercent": vsa_evm, "vsaFreqErrorHz": vsa_freq_err, "pass": None}
    if vsa_evm is not None:
        delta = nrd["rmsEvmPercent"] - vsa_evm
        out["evmDeltaPoints"] = delta
        out["evmDeltaRelPercent"] = (delta / vsa_evm * 100.0) if vsa_evm else float("inf")
        out["evmWithinTolerance"] = abs(delta) <= tol.get("evmPercentPoints", 0.5)
        out["pass"] = bool(out["evmWithinTolerance"])
    if vsa_freq_err is not None:
        fdelta = nrd["frequencyErrorHz"] - vsa_freq_err
        out["freqDeltaHz"] = fdelta
        out["freqWithinTolerance"] = abs(fdelta) <= tol.get("freqErrorHz", 50.0)
        if out["pass"] is not None:
            out["pass"] = bool(out["pass"] and out["freqWithinTolerance"])
    return out


def run_demod(iq, cfg, case, vsa_evm, vsa_freq_err, args) -> int:
    print(f"demodulating {len(iq)} samples at {cfg.sampleRate / 1e6:.4f} MHz ...")
    res = nrdemod.demodulate(iq, cfg)
    if not res.ok:
        print(f"nrdemod failed: {res.error}", file=sys.stderr)
        return 1

    nrd = summarise(res)
    cmp_ = verdict(nrd, vsa_evm, vsa_freq_err, case.get("tolerance", {}))

    print("\n--- nrdemod ---")
    for key, value in nrd.items():
        print(f"  {key:26s} {value}")
    if vsa_evm is not None or vsa_freq_err is not None:
        print("\n--- vs 89600 ---")
        for key, value in cmp_.items():
            print(f"  {key:26s} {value}")

    record = {
        "timestamp": _dt.datetime.now().isoformat(timespec="seconds"),
        "case": case.get("name", "unnamed"),
        "mode": args.mode,
        "source": getattr(args, "capture", None) or "live",
        "sampleRateHz": cfg.sampleRate,
        "numSamples": int(len(iq)),
        "nrdemod": nrd,
        "comparison": cmp_,
    }
    paths = write_reports(os.path.join(_ROOT, "reports"), record, res, plot=not args.no_plot)
    print("\nreports:")
    for path in paths:
        print(f"  {path}")

    if cmp_["pass"] is False:
        return 2
    return 0


def mode_selftest(args) -> int:
    case = load_case(args.config) if args.config else {"name": "selftest", "demod": {
        "numerology": 1, "frequencyRange": 1, "channelBandwidthMHz": 100,
        "modulation": "64qam", "sampleRate": 122880000.0}}
    cfg = build_config(case)
    iq = nrdemod.generate_test_signal(cfg, num_symbols=28, leading_samples=128, seed=11)
    print(f"nrdemod {nrdemod.version()} - synthetic burst, expecting ~0 % EVM")
    return run_demod(iq, cfg, case, None, None, args)


def mode_file(args) -> int:
    case = load_case(args.config)
    iq, fs = vsa_io.load_iq(args.capture, sample_rate=args.fs)
    if fs is None:
        fs = case.get("demod", {}).get("sampleRate")
    if not fs:
        print("error: sample rate unknown, pass --fs or set demod.sampleRate", file=sys.stderr)
        return 2
    cfg = build_config(case, fs)
    return run_demod(iq, cfg, case, args.vsa_evm, args.vsa_freq_err, args)


def mode_live(args) -> int:
    case = load_case(args.config)
    cmds = CommandMap.load(args.map)
    vcfg = case.get("vsa", {})

    if args.link == "com":
        opener = VsaCom(timeout=args.timeout)
        make_vsa = Vsa89600Com
    else:
        opener = VsaScpi(args.host, args.port, timeout=args.timeout)
        make_vsa = lambda link: Vsa89600(link, cmds)  # noqa: E731

    with opener as link:
        print(f"connected: {link.idn()}")
        link.errors()
        vsa = make_vsa(link)

        if args.recording:
            print(f"loading recording {args.recording} into the VSA ...")
            vsa.load_recording(os.path.abspath(args.recording))
        if args.points:
            vsa.set_points(args.points)

        if not args.no_configure and vcfg.get("centerHz"):
            vsa.configure(vcfg["centerHz"], vcfg.get("spanHz", 0) or 0, vcfg.get("points", 0) or 0)
        vsa.single_measure(timeout=args.timeout)

        vsa_evm = args.vsa_evm if args.vsa_evm is not None else vsa.read_evm_percent()
        vsa_freq_err = args.vsa_freq_err if args.vsa_freq_err is not None else vsa.read_scalar("query_freq_error")
        print(f"VSA EVM: {vsa_evm}   VSA freq error: {vsa_freq_err}")

        iq = vsa.fetch_time_iq()
        fs = args.fs or vsa.sample_rate() or case.get("demod", {}).get("sampleRate")

        if iq is None and vcfg.get("recordingPath"):
            print("no time trace over SCPI, falling back to a saved recording ...")
            if vsa.save_recording(vcfg["recordingPath"]) and args.recording_local:
                iq, fs_file = vsa_io.load_iq(args.recording_local, sample_rate=args.fs)
                fs = fs or fs_file

        for err in link.errors():
            print(f"VSA error: {err}", file=sys.stderr)

    if iq is None:
        print(
            "error: could not retrieve samples. Run 'python tools/vsa_link.py --probe' "
            "and fix configs/scpi_89600.json, or export a recording and use file mode.",
            file=sys.stderr,
        )
        return 2
    if not fs:
        print("error: sample rate unknown, pass --fs", file=sys.stderr)
        return 2

    cfg = build_config(case, fs)
    if args.save_capture:
        np.save(args.save_capture, iq)
        print(f"capture saved to {args.save_capture}")
    return run_demod(iq, cfg, case, vsa_evm, vsa_freq_err, args)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="mode", required=True)

    def common(sp):
        sp.add_argument("--config", default=os.path.join(_ROOT, "configs", "fr1_100mhz_mu1_64qam.json"))
        sp.add_argument("--fs", type=float, help="override the sample rate in Hz")
        sp.add_argument("--vsa-evm", type=float, help="EVM (%%) reported by the 89600")
        sp.add_argument("--vsa-freq-err", type=float, help="frequency error (Hz) reported by the 89600")
        sp.add_argument("--no-plot", action="store_true", help="skip the PNG in the HTML report")

    sp = sub.add_parser("selftest", help="demodulate an ideal synthetic burst")
    common(sp)
    sp.set_defaults(func=mode_selftest)

    sp = sub.add_parser("file", help="use a recording exported from the VSA")
    sp.add_argument("capture")
    common(sp)
    sp.set_defaults(func=mode_file)

    sp = sub.add_parser("live", help="drive the 89600 SCPI server")
    sp.add_argument("--link", choices=("scpi", "com"), default=os.environ.get("VSA_LINK", "scpi"),
                    help="'com' drives a 89600 running on this PC without the SCPI server")
    sp.add_argument("--recording", help="I/Q recording to play back through the VSA first")
    sp.add_argument("--points", type=int, help="VSA frequency points (sets the time record length)")
    sp.add_argument("--host", default=os.environ.get("VSA_HOST", "127.0.0.1"))
    sp.add_argument("--port", type=int, default=int(os.environ.get("VSA_PORT", 5025)))
    sp.add_argument("--map", help="SCPI command map JSON")
    sp.add_argument("--timeout", type=float, default=60.0)
    sp.add_argument("--no-configure", action="store_true", help="do not touch center/span/points")
    sp.add_argument("--recording-local", help="path where the VSA recording is visible to this PC")
    sp.add_argument("--save-capture", help="save the retrieved samples to an .npy")
    common(sp)
    sp.set_defaults(func=mode_live)

    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if not hasattr(args, "capture"):
        args.capture = None
    try:
        return args.func(args)
    except (ScpiError, ComError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
