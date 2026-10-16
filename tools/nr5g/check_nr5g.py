"""Demodulate captures/generated/nr5g with the native nrdemod core, configured from each .para2."""
import dataclasses
import glob
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "nrdemod", "python"))
import nrdemod  # noqa: E402
from nrdemod import vsa  # noqa: E402

FIELDS = {f.name: f.type for f in dataclasses.fields(nrdemod.Config)}
EVM_LIMIT_PCT = 1.0


def config_from_para2(path):
    kw = {}
    for line in open(path):
        p = [x.strip() for x in line.split(",", 3)]
        if len(p) == 4 and p[0] == "INPUT" and p[1] in FIELDS:
            kw[p[1]] = float(p[3]) if p[2] == "Double" else int(p[3])
    return nrdemod.Config(**kw)


ok = True
for txt in sorted(glob.glob(os.path.join(ROOT, "captures", "generated", "nr5g", "*.txt"))):
    iq, fs = vsa.load_iq(txt)
    res = nrdemod.demodulate(iq, config_from_para2(txt[:-4] + ".para2"))
    good = res.ok and res.rmsEvmPercent < EVM_LIMIT_PCT
    ok &= good
    detail = ("EVM %.4f %%  freqErr %.2f Hz  %s  %d sym" % (res.rmsEvmPercent, res.frequencyErrorHz,
              res.modulation_name, res.numSymbols)) if res.ok else res.error
    print("%-48s %s  %s" % (os.path.basename(txt), "OK  " if good else "FAIL", detail))
sys.exit(0 if ok else 1)
