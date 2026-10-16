#!/usr/bin/env python3
"""Verify the Python HRP UWB PHY (nrdemod/python/examples/uwb_hrp.py) against MATLAB references.

Reference vectors come from tools/uwb/gen_hrp_reference.m (Communications Toolbox
lrwpanWaveformGenerator); generate them first with
    matlab -batch "cd tools/uwb; gen_hrp_reference"

For every reference packet this script checks:
  1. TX stages, bit-exact: PHR+SECDED, RS(63,55), convolutional code, scrambler, chips
     (and the Python pulse-shaped waveform for SP0 packets).
  2. RX on the clean MATLAB waveform (random leading/trailing zeros + phase): PHR fields
     and PSDU bit-exact.
  3. RX with impairments (CFO, phase, AWGN): PSDU bit-exact.
  4. Optional (--native): the native nrdemod UWB_HRP core's burst/sync/CFO estimates.
Exit code is non-zero if any check fails.
"""
from __future__ import annotations

import argparse
import glob
import os
import sys

import numpy as np
import scipy.io as sio

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "nrdemod", "python", "examples"))
sys.path.insert(0, os.path.join(ROOT, "nrdemod", "python"))

import uwb_hrp as hrp  # noqa: E402


def cfg_from_ref(m) -> hrp.HrpConfig:
    mode = str(m["mode"])
    cpb = np.atleast_1d(m["chips_per_burst"])
    phr_rate = None
    if mode == "BPRF":
        phr_rate = 850 if int(cpb[0]) == 64 else 6810
    return hrp.HrpConfig(
        mode=mode,
        code_index=int(m["code_index"]),
        mean_prf=62.4 if mode == "BPRF" else float(m["mean_prf"]),
        data_rate_kbps=int(m["data_rate_kbps"]),
        phr_data_rate_kbps=phr_rate,
        sfd_number=int(m["sfd_number"]),
        preamble_duration=int(m["preamble_duration"]),
        preamble_spreading_factor=int(m["preamble_spreading_factor"]),
        sts_packet_configuration=int(m["sts_packet_configuration"]),
        ranging=bool(m["ranging"]),
        samples_per_pulse=int(m["samples_per_pulse"]),
    )


class Checker:
    def __init__(self, verbose: bool):
        self.verbose = verbose
        self.failed = 0
        self.passed = 0

    def check(self, name: str, ok: bool, detail: str = ""):
        if ok:
            self.passed += 1
        else:
            self.failed += 1
        if self.verbose or not ok:
            print(f"    [{'PASS' if ok else 'FAIL'}] {name} {detail}")
        return ok


def bits_equal(a, b) -> tuple[bool, str]:
    a, b = np.asarray(a).astype(int).ravel(), np.asarray(b).astype(int).ravel()
    if a.shape != b.shape:
        return False, f"length {a.size} vs {b.size}"
    n = int(np.count_nonzero(a != b))
    return n == 0, f"({n} errors / {a.size})"


def impair(wave, fs, rng, snr_db=None, cfo_hz=0.0, lead=0, tail=0):
    x = np.concatenate([np.zeros(lead, complex), wave, np.zeros(tail, complex)])
    n = np.arange(len(x))
    x = x * np.exp(1j * (2 * np.pi * cfo_hz * n / fs + rng.uniform(-np.pi, np.pi)))
    if snr_db is not None:
        p = np.mean(np.abs(wave) ** 2)
        sigma = np.sqrt(p / 10 ** (snr_db / 10) / 2)
        x = x + sigma * (rng.standard_normal(len(x)) + 1j * rng.standard_normal(len(x)))
    return x


def run_native(x, cfg: hrp.HrpConfig, fs):
    try:
        from nrdemod import demod
    except Exception as exc:  # native library not built / not importable
        return f"native unavailable ({exc.__class__.__name__})"
    r = demod.demodulate(demod.UWB_HRP, x, demod.Config(
        sampleRate=fs, syncLength=cfg.preamble_duration, chipRateHz=hrp.CHIP_RATE_HZ,
        phyMode=0 if cfg.mode == "802.15.4a" else 1))
    d = r.as_dict()
    keys = ("burstFound", "burstStart", "syncFound", "syncRho", "frequencyErrorHz")
    return "native " + ", ".join(f"{k}={d[k]:.4g}" if isinstance(d.get(k), float) else f"{k}={d.get(k)}"
                                 for k in keys if k in d)


def verify_file(path, args, chk: Checker, rng):
    m = sio.loadmat(path, squeeze_me=True)
    name = str(m["name"])
    cfg = cfg_from_ref(m)
    fs = float(m["sample_rate"])
    wave = np.asarray(m["waveform"], dtype=complex).ravel()
    psdu = np.asarray(m["psdu_bits"], dtype=np.uint8).ravel()
    print(f"{name}: {cfg.mode}, CI{cfg.code_index}, {cfg.data_rate_kbps} kb/s, SP{cfg.sts_packet_configuration}, "
          f"PSDU {len(psdu) // 8} B")
    n_fail0 = chk.failed

    # 1) TX stage checks
    phr13 = hrp.build_phr(cfg, len(psdu) // 8)
    chk.check("PHR+SECDED", *bits_equal(hrp.secded_encode(phr13), m["phr_bits"]))
    chk.check("RS(63,55) encode", *bits_equal(hrp.rs_encode(psdu), m["rs_psdu_bits"]))
    cw = hrp.conv_encode(np.concatenate([m["phr_bits"], m["rs_psdu_bits"], [0, 0]]))
    chk.check("conv. encode K=3", *bits_equal(cw, m["conv_codeword"]))
    pn_ref = np.asarray(m["pn_sequence"]).ravel()
    chk.check("scrambler", *bits_equal(hrp.scrambler(cfg.code_index, len(pn_ref)), pn_ref))
    if cfg.sts_packet_configuration == 0:
        g = hrp.generate(psdu, cfg)
        chips_ref = np.asarray(m["chips"], dtype=float).ravel()
        same = g["chips"].shape == chips_ref.shape and np.array_equal(g["chips"], chips_ref)
        chk.check("chips (SHR+PHR+PSDU)", same, f"({len(g['chips'])} vs {len(chips_ref)})")
        if g["waveform"].shape == wave.shape:
            err = np.max(np.abs(g["waveform"] - wave)) / np.max(np.abs(wave))
            chk.check("pulse-shaped waveform", err < 1e-9, f"(max rel err {err:.1e})")
        else:
            chk.check("pulse-shaped waveform", False, "(length mismatch)")
        # round trip through our own modulator too
        r = hrp.demodulate(g["waveform"], hrp.HrpConfig(**{**cfg.__dict__, "phr_data_rate_kbps": None}), fs)
        chk.check("python TX -> python RX", r.ok and bits_equal(r.psdu_bits, psdu)[0], r.status)

    # 2) clean MATLAB waveform with unknown start/phase, PHR rate auto-detected
    rx_cfg = hrp.HrpConfig(**{**cfg.__dict__, "phr_data_rate_kbps": None})
    lead = int(rng.integers(500, 5000))
    x = impair(wave, fs, rng, lead=lead, tail=3000)
    r = hrp.demodulate(x, rx_cfg, fs)
    ok_psdu, det = bits_equal(r.psdu_bits if r.psdu_bits is not None else [], psdu)
    chk.check("clean: PSDU", r.ok and ok_psdu, f"{det} status={r.status}")
    chk.check("clean: PHR SECDED", r.phr_status == "ok", r.phr_status)
    if r.phr:
        exp = {"data_rate_kbps": cfg.data_rate_kbps, "frame_length": len(psdu) // 8,
               "ranging": int(cfg.ranging), "preamble_duration": cfg.preamble_duration}
        chk.check("clean: PHR fields", all(r.phr[k] == v for k, v in exp.items()), str(r.phr))
    exp_start = lead + int(m[f"idx_{'PHR'}"][0]) - 1
    chk.check("clean: PHR timing", r.phr_start == exp_start, f"({r.phr_start} vs {exp_start})")
    if args.verbose:
        print(f"      CFO {r.cfo_hz:.1f} Hz, EVM {r.evm_db:.1f} dB, preamble syms {r.preamble_symbols}, "
              f"SFD @ {r.sfd_symbol}")
    if args.plot:
        _save_plots(r, x, fs, psdu, os.path.join(args.plot, name + "_clean"))

    # 3) impaired
    for snr in args.snr:
        cfo = rng.uniform(-1, 1) * args.cfo_hz
        x = impair(wave, fs, rng, snr_db=snr, cfo_hz=cfo, lead=int(rng.integers(500, 5000)), tail=3000)
        r = hrp.demodulate(x, rx_cfg, fs)
        ok_psdu, det = bits_equal(r.psdu_bits if r.psdu_bits is not None else [], psdu)
        chk.check(f"SNR {snr:+.0f} dB CFO {cfo/1e3:+.0f} kHz: PSDU", r.ok and ok_psdu,
                  f"{det} est CFO {r.cfo_hz/1e3:+.1f} kHz, RS corr {r.rs_corrected}, status={r.status}")
        if args.plot and snr == args.snr[-1]:
            _save_plots(r, x, fs, psdu, os.path.join(args.plot, f"{name}_snr{snr:g}"))

    if args.native:
        print("    " + run_native(impair(wave, fs, rng, lead=2000, tail=2000), cfg, fs))
    print(f"    -> {'OK' if chk.failed == n_fail0 else 'FAILED'}")


def _save_plots(res, x, fs, ref, stem):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(os.path.dirname(stem), exist_ok=True)
    for i, fig in enumerate(hrp.plot_stages(res, x, fs, ref, os.path.basename(stem))):
        fig.savefig(f"{stem}_{i}.png", dpi=90)
        plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="*", help="reference .mat files (default: captures/generated/uwb_hrp/*.mat)")
    ap.add_argument("--snr", type=float, nargs="*", default=[10.0, 0.0], help="AWGN SNR list in dB")
    ap.add_argument("--cfo-hz", type=float, default=200e3, help="max |CFO| for impaired runs")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--plot", help="directory to save per-stage PNG plots")
    ap.add_argument("--native", action="store_true", help="also run the native nrdemod UWB_HRP core")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    files = args.files or sorted(glob.glob(os.path.join(ROOT, "captures", "generated", "uwb_hrp", "*.mat")))
    if not files:
        print("no reference files; run: matlab -batch \"cd tools/uwb; gen_hrp_reference\"")
        return 2
    rng = np.random.default_rng(args.seed)
    chk = Checker(args.verbose)
    for f in files:
        verify_file(f, args, chk, rng)
    print(f"\n{chk.passed} passed, {chk.failed} failed")
    return 1 if chk.failed else 0


if __name__ == "__main__":
    sys.exit(main())
