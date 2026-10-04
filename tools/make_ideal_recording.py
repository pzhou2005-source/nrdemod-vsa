"""Regenerate the ideal 89600 reference recording (3GPP carrier raster)."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "nrdemod", "python"))

import nrdemod
from nrdemod import vsa as vsa_io

FS = 122.88e6
cfg = nrdemod.Config(numerology=1, frequencyRange=1, channelBandwidthMHz=100,
                     modulation=nrdemod.QAM64, sampleRate=FS)
iq = nrdemod.generate_test_signal(cfg, num_symbols=112, leading_samples=128, seed=11)
out = os.path.join(HERE, "..", "captures", "ideal.mat")
vsa_io.save_mat(out, iq, FS)

r = nrdemod.demodulate(iq, cfg)
print("%d samples (%.3f ms) -> %s" % (iq.size, iq.size / FS * 1e3, os.path.normpath(out)))
print("round trip EVM %.6g %%, freq error %.3f Hz" % (r.rmsEvmPercent, r.frequencyErrorHz))
