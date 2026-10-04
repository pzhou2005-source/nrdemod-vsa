#!/usr/bin/env python3
"""Correlate nrdemod against a Keysight 89600 VSA measurement.

Typical flow on the Windows analysis PC:

  1. In the 89600 VSA, record the same burst you want to analyse and save it
     (File > Save > Recording) as ``capture.mat`` or ``capture.csv``.
  2. Read the VSA's own 5G NR EVM number from the summary table.
  3. Run:

       python vsa_correlate.py capture.mat --bw 100 --mu 1 --mod 64QAM \
           --vsa-evm 1.42

Use ``--export ideal.csv`` to write an ideal reference burst that can be
loaded back into the VSA, which isolates analyser-side setup differences.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

import nrdemod
from nrdemod import vsa

_MODS = {
    "auto": nrdemod.AUTO,
    "qpsk": nrdemod.QPSK,
    "16qam": nrdemod.QAM16,
    "64qam": nrdemod.QAM64,
    "256qam": nrdemod.QAM256,
}


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("capture", nargs="?", help="89600 recording (.mat/.csv/.txt/.npy/.bin)")
    p.add_argument("--fs", type=float, help="sample rate in Hz (overrides the file header)")
    p.add_argument("--bw", type=int, default=100, help="channel bandwidth in MHz")
    p.add_argument("--mu", type=int, default=1, help="numerology (SCS = 15 kHz * 2^mu)")
    p.add_argument("--fr", type=int, default=1, choices=(1, 2), help="frequency range")
    p.add_argument("--mod", default="auto", type=str.lower, choices=sorted(_MODS), help="modulation")
    p.add_argument("--rb", type=int, default=0, help="allocated resource blocks (0 = whole carrier)")
    p.add_argument("--rb-offset", type=int, default=0, help="first allocated RB")
    p.add_argument("--dft-s", action="store_true", help="DFT-s-OFDM (transform precoding)")
    p.add_argument("--carrier-offset", type=float, default=0.0, help="carrier offset from DC in Hz")
    p.add_argument("--no-burst-search", action="store_true")
    p.add_argument("--no-equalize", action="store_true")
    p.add_argument("--vsa-evm", type=float, help="EVM (%%) reported by the 89600 for the same capture")
    p.add_argument("--tolerance", type=float, default=0.5, help="allowed EVM delta in %% points")
    p.add_argument("--export", metavar="CSV", help="write an ideal reference burst for the VSA")
    p.add_argument("--plot", action="store_true", help="show constellation and EVM traces")
    p.add_argument("--dump", metavar="NPZ", help="save all result vectors to an .npz")
    return p


def make_config(args, sample_rate: float) -> nrdemod.Config:
    return nrdemod.Config(
        numerology=args.mu,
        frequencyRange=args.fr,
        channelBandwidthMHz=args.bw,
        numResourceBlocks=args.rb,
        resourceBlockOffset=args.rb_offset,
        modulation=_MODS[args.mod],
        sampleRate=sample_rate,
        transformPrecoding=args.dft_s,
        carrierOffsetHz=args.carrier_offset,
        burstSearch=not args.no_burst_search,
        equalize=not args.no_equalize,
    )


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    if args.capture:
        iq, fs = vsa.load_iq(args.capture, sample_rate=args.fs)
        if fs is None:
            print("error: sample rate unknown, pass --fs", file=sys.stderr)
            return 2
        print(f"loaded {len(iq)} samples from {args.capture} at {fs / 1e6:.4f} MHz")
    else:
        fs = args.fs or 122.88e6
        cfg = make_config(args, fs)
        iq = nrdemod.generate_test_signal(cfg, num_symbols=28, leading_samples=128, seed=11)
        print(f"no capture given, using a synthetic burst: {len(iq)} samples at {fs / 1e6:.4f} MHz")

    cfg = make_config(args, fs)

    if args.export:
        ideal_cfg = make_config(args, fs)
        ideal = nrdemod.generate_test_signal(ideal_cfg, num_symbols=28, leading_samples=128, seed=11)
        vsa.save_csv(args.export, ideal, fs)
        print(f"ideal reference burst written to {args.export}")

    res = nrdemod.demodulate(iq, cfg)
    if not res.ok:
        print(f"demodulation failed: {res.error}", file=sys.stderr)
        return 1

    print("\n--- nrdemod ---")
    for key in (
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
        "iqOffsetDb",
        "iqGainImbalanceDb",
        "iqQuadratureErrorDeg",
        "commonPhaseErrorDeg",
        "dmrsEvmPercent",
        "flatnessRippleRange1Db",
        "flatnessRippleRange2Db",
    ):
        print(f"  {key:28s} {getattr(res, key)}")
    print(f"  {'modulation':28s} {res.modulation_name}")

    if args.vsa_evm is not None:
        print("\n--- correlation vs 89600 ---")
        for key, value in vsa.compare_evm(res, args.vsa_evm, args.tolerance).items():
            print(f"  {key:28s} {value}")

    if args.dump:
        np.savez_compressed(args.dump, **{k: v for k, v in res.as_dict(True).items() if isinstance(v, np.ndarray)})
        print(f"\nvectors saved to {args.dump}")

    if args.plot:
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(2, 2, figsize=(11, 8))
        c = res.constellation
        ax[0][0].plot(c.real, c.imag, ".", markersize=2)
        ax[0][0].set_title(f"Constellation ({res.modulation_name})")
        ax[0][0].set_aspect("equal")
        ax[0][1].plot(res.evmPerSubcarrier)
        ax[0][1].set_title("EVM per subcarrier (%)")
        ax[1][0].plot(res.evmPerSymbol)
        ax[1][0].set_title("EVM per symbol (%)")
        ax[1][1].plot(res.spectrumDb)
        ax[1][1].set_title("Spectrum (dB)")
        fig.suptitle(f"nrdemod  rmsEVM={res.rmsEvmPercent:.3f}%")
        fig.tight_layout()
        plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main())
