#!/usr/bin/env python3
"""Replays a batch of exported TDC waveforms (.npy + .json Config sidecar) through the
standalone nrdemod core, e.g. on the Windows VSA bench where the UNO/legacy A/B regression
cannot run.

The .npy/.json pairs are produced on Linux by the in-tree test
``testDemod_ExportTdcWaveformsForStandaloneWindows`` (set NR_DEMOD_REFERENCE_WAVEFORM_EXPORT).
Fields prefixed ``linux`` in the sidecar are the Linux-native reference values for comparison
only; they are not passed to nrdemod.Config.

Usage:
  python tools\\replay_exported_waveforms.py <dataset_dir> [--csv reports\\exported_replay.csv]
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "nrdemod", "python"))

import numpy as np

import nrdemod


def replay(dataset_dir: str, csv_path: str) -> int:
    json_paths = sorted(glob.glob(os.path.join(dataset_dir, "**", "*.json"), recursive=True))
    if not json_paths:
        print(f"no .json sidecars found under {dataset_dir}", file=sys.stderr)
        return 1

    os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)
    passed = failed = 0
    with open(csv_path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["waveform", "ok", "rmsEvmPercent", "linuxRmsEvmPercent", "deltaEvmPoints",
                          "frequencyErrorHz", "linuxFrequencyErrorHz", "modulation", "numSymbols", "error"])
        for json_path in json_paths:
            npy_path = json_path[:-5] + ".npy"
            name = os.path.splitext(os.path.basename(json_path))[0]
            if not os.path.isfile(npy_path):
                writer.writerow([name, False, "", "", "", "", "", "", "", "missing .npy"])
                failed += 1
                continue

            import json

            meta = json.load(open(json_path))
            iq = np.load(npy_path)
            cfg_fields = {k: v for k, v in meta.items() if not k.startswith("linux")}
            cfg = nrdemod.Config(**cfg_fields)
            res = nrdemod.demodulate(iq, cfg)

            linux_evm = meta.get("linuxRmsEvmPercent")
            delta = (res.rmsEvmPercent - linux_evm) if (res.ok and linux_evm is not None) else ""
            # linuxFrequencyErrorHz is the pre-refinement estimate; compare against the matching
            # nrdemod stage instead of the final (phase-tracking refined) frequencyErrorHz.
            freq_err = res.acquisitionFrequencyErrorHz if res.ok else ""
            writer.writerow([name, res.ok, res.rmsEvmPercent if res.ok else "", linux_evm, delta,
                              freq_err, meta.get("linuxFrequencyErrorHz"),
                              res.modulation_name if res.ok else "", res.numSymbols if res.ok else "",
                              res.error])
            if res.ok:
                passed += 1
                print(f"{name}: rmsEvm={res.rmsEvmPercent:.6f}% (linux={linux_evm}%) mod={res.modulation_name}")
            else:
                failed += 1
                print(f"{name}: FAILED: {res.error}", file=sys.stderr)

    print(f"\n{passed} ok, {failed} failed -> {csv_path}")
    return 0 if failed == 0 else 2


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("dataset_dir", help="directory with exported *.npy/*.json pairs (searched recursively)")
    parser.add_argument("--csv", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "reports",
                                                        "exported_replay.csv"))
    args = parser.parse_args(argv)
    return replay(args.dataset_dir, args.csv)


if __name__ == "__main__":
    raise SystemExit(main())
