"""Back up the live 89600 setup and play an I/Q recording through it.

    python tools/vsa_play.py captures/ideal.mat --vector --points 262144
    python tools/vsa_play.py --restore reports/vsa_setup_backup.setx

Needed because this bench drives the VSA over its local COM automation
interface rather than the SCPI server.
"""

from __future__ import annotations

import argparse
import os
import sys

from vsa_com import Vsa89600Com, VsaCom

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, ".."))

_VECTOR_MODE = 0  # Measurement.DemodConfig: no demodulator, plain vector measurement


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("recording", nargs="?", help="I/Q recording to load (.mat/.csv/.sdf/...)")
    p.add_argument("--points", type=int, help="frequency points, drives the time record length")
    p.add_argument("--vector", action="store_true", help="switch the VSA to vector mode (no demod)")
    p.add_argument("--backup", default=os.path.join(_ROOT, "reports", "vsa_setup_backup.setx"))
    p.add_argument("--restore", help="recall a saved setup instead of loading a recording")
    args = p.parse_args(argv)

    with VsaCom() as link:
        vsa = Vsa89600Com(link)
        meas = link.measurement
        print(f"connected: {link.idn()}")

        if args.restore:
            meas.RecallFile(os.path.abspath(args.restore))
            print(f"setup restored from {args.restore}")
            return 0

        if os.path.exists(args.backup):
            # never overwrite: the first backup is the only one holding the operator's setup
            print(f"keeping the existing setup backup {args.backup}")
        elif vsa.save_setup(os.path.abspath(args.backup)):
            print(f"current setup backed up to {args.backup}")
        else:
            print("warning: could not back up the current setup", file=sys.stderr)

        if args.vector:
            meas.DemodConfig = _VECTOR_MODE

        if args.recording:
            path = os.path.abspath(args.recording)
            print(f"loading recording {path} ...")
            vsa.load_recording(path)
            rec = meas.Inputs.Recording
            print(f"recording  : {rec.PlaySampleRate / 1e6:.4f} MSa/s, "
                  f"center {rec.PlayCenter / 1e6:.4f} MHz, span {rec.PlaySpan / 1e6:.4f} MHz")
            meas.Frequency.Span = float(rec.PlaySpan)
            meas.Frequency.Center = float(rec.PlayCenter)

        if args.points:
            meas.Frequency.Points = int(args.points)

        meas.Continuous = False
        meas.Initialize()
        meas.WaitForMeasDone(120000)

        freq = meas.Frequency
        print(f"measurement: center {freq.Center / 1e6:.4f} MHz  span {freq.Span / 1e6:.4f} MHz  "
              f"points {freq.Points}")
        iq = vsa.fetch_time_iq()
        if iq is None:
            print("time trace : not available in this measurement mode")
        else:
            peak = float(abs(iq).max()) if iq.size else 0.0
            print(f"time trace : {iq.size} samples at {vsa.sample_rate() / 1e6:.4f} MHz, peak |x| = {peak:.6g}")
        print(f"VSA EVM    : {vsa.read_evm_percent()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
