"""Sanity-check captures/generated/sparklink: readable by the existing loaders, rate/length/deviation as configured."""
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
sys.path.insert(0, TOOLS)
from legacy_correlate import parse_waveform  # noqa: E402


def para2(path):
    out = {}
    for line in open(path):
        p = [x.strip() for x in line.split(",", 3)]
        if len(p) == 4 and p[0] in ("INPUT", "INFO"):
            out[p[1]] = p[3]
    return out


ok = True
for txt in sorted(glob.glob(os.path.join(TOOLS, "..", "captures", "generated", "sparklink", "*.txt"))):
    meta = para2(txt[:-4] + ".para2")
    iq, fs, fc = parse_waveform(txt)
    rs = float(meta["SymbolRate"])
    on = np.abs(iq) > 0.1 * np.abs(iq).max()
    f = np.diff(np.unwrap(np.angle(iq))) * fs / (2 * np.pi)
    dev = np.percentile(np.abs(f[on[1:] & on[:-1]]), 99)
    exp_dev = float(meta["ModulationIndex"]) * rs / 2
    good = (abs(fs - 8 * rs) < 1 and iq.size == int(meta["NumSamples"]) and abs(dev / exp_dev - 1) < 0.05)
    ok &= good
    print("%-22s fs=%5.1f MS/s n=%6d fc=%.0f MHz dev=%7.1f kHz (exp %6.1f) %s"
          % (os.path.basename(txt), fs / 1e6, iq.size, fc / 1e6, dev / 1e3, exp_dev / 1e3, "OK" if good else "FAIL"))
sys.exit(0 if ok else 1)
