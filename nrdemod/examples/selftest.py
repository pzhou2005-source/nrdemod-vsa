#!/usr/bin/env python3
"""Self test: generate an ideal NR burst, demodulate it, print the metrics."""

import sys

import nrdemod


def run(label, **cfg_kwargs):
    cfg = nrdemod.Config(**cfg_kwargs)
    iq = nrdemod.generate_test_signal(cfg, num_symbols=28, leading_samples=64, seed=7)
    res = nrdemod.demodulate(iq, cfg)
    if not res.ok:
        print(f"{label:28s} FAILED: {res.error}")
        return False
    print(
        f"{label:28s} rmsEVM={res.rmsEvmPercent:7.4f}%  peakEVM={res.peakEvmPercent:7.4f}%  "
        f"mod={res.modulation_name:6s} RB={res.numResourceBlocks:3d}  fft={res.fftSize:5d}  "
        f"dF={res.frequencyErrorHz:8.1f}Hz  symbols={res.numSymbols}"
    )
    return res.rmsEvmPercent < 1.0


def main():
    print(f"nrdemod {nrdemod.version()}\n")
    ok = True
    ok &= run(
        "FR1 100 MHz mu=1 64QAM",
        numerology=1,
        frequencyRange=nrdemod.FR1,
        channelBandwidthMHz=100,
        sampleRate=122.88e6,
        modulation=nrdemod.QAM64,
    )
    ok &= run(
        "FR1 100 MHz mu=1 256QAM",
        numerology=1,
        frequencyRange=nrdemod.FR1,
        channelBandwidthMHz=100,
        sampleRate=122.88e6,
        modulation=nrdemod.QAM256,
    )
    ok &= run(
        "FR1 100 MHz DFT-s-OFDM",
        numerology=1,
        frequencyRange=nrdemod.FR1,
        channelBandwidthMHz=100,
        sampleRate=122.88e6,
        modulation=nrdemod.QPSK,
        transformPrecoding=True,
    )
    ok &= run(
        "FR2 200 MHz mu=3 16QAM",
        numerology=3,
        frequencyRange=nrdemod.FR2,
        channelBandwidthMHz=200,
        sampleRate=245.76e6,
        modulation=nrdemod.QAM16,
    )
    print("\nPASS" if ok else "\nFAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
