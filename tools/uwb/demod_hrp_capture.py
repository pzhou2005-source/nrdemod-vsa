#!/usr/bin/env python3
"""Demodulate an HRP UWB (802.15.4z BPRF / 802.15.4a) capture with the Python PHY.

Reads the 89600-style plaintext export (header, 'Y', then 'I<TAB>Q' lines) and the
matching .para2 file (CodeIndex, SfdIndex, StsPacketConfig, SyncLength, PhrDataRateMode),
prints the PHR/PSDU, checks the MAC FCS (CRC-16, ITU-T, LSB first) and can save
per-stage plots.

    python tools/uwb/demod_hrp_capture.py captures/tdc_reference_plaintext/802.15.4z_HrpUwb/*BPRF_SP0*.txt --plot out
"""
from __future__ import annotations

import argparse
import glob
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "nrdemod", "python", "examples"))

import uwb_hrp as hrp  # noqa: E402


def load_plaintext(path):
    with open(path) as f:
        lines = f.read().split("\n")
    hdr = {}
    k = 0
    while lines[k].strip() != "Y":
        parts = lines[k].split("\t")
        if len(parts) == 2:
            hdr[parts[0]] = parts[1]
        k += 1
    a = np.loadtxt(lines[k + 1:])
    fs = 1.0 / float(hdr.get("XDelta", 1 / 1.9968e9))
    return a[:, 0] + 1j * a[:, 1], fs


def find_para2(txt_path):
    stem = os.path.splitext(os.path.basename(txt_path))[0]
    for cand in (os.path.splitext(txt_path)[0] + ".para2",
                 *glob.glob(os.path.join(ROOT, "captures", "**", stem + ".para2"), recursive=True)):
        if os.path.exists(cand):
            return cand
    return None


def config_from_para2(path) -> hrp.HrpConfig:
    p = {}
    with open(path) as f:
        for line in f:
            parts = [x.strip() for x in line.split(",")]
            if parts[0] == "INPUT" and len(parts) >= 4:
                p[parts[1]] = parts[3]
    phy = int(p.get("PhyMode", 1))
    if phy == 2:
        raise NotImplementedError("HPRF captures are not supported by uwb_hrp.py")
    mode = "BPRF" if phy == 1 else "802.15.4a"
    pdr = int(p.get("PhrDataRateMode", 0))
    return hrp.HrpConfig(
        mode=mode,
        code_index=int(p.get("CodeIndex", 9)),
        sfd_number=int(p.get("SfdIndex", 0)),
        preamble_duration=int(p.get("SyncLength", 64)),
        preamble_spreading_factor=int(p["DeltaLength"]) if "DeltaLength" in p else None,
        sts_packet_configuration=int(p.get("StsPacketConfig", 0)),
        phr_data_rate_kbps=(850 if pdr == 0 else 6810) if mode == "BPRF" else None,
        samples_per_pulse=int(p.get("OSR", 4)),
    )


def fcs16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+")
    ap.add_argument("--plot", help="directory for per-stage PNGs")
    args = ap.parse_args()
    rc = 0
    for path in args.files:
        para = find_para2(path)
        if para is None:
            print(f"{path}: no .para2 found")
            rc = 1
            continue
        try:
            cfg = config_from_para2(para)
        except NotImplementedError as exc:
            print(f"{os.path.basename(path)}: skipped ({exc})")
            continue
        iq, fs = load_plaintext(path)
        r = hrp.demodulate(iq, cfg, fs)
        print(f"{os.path.basename(path)}")
        print(f"  config : {cfg.mode} CI{cfg.code_index} SFD{cfg.sfd_number} SP{cfg.sts_packet_configuration}, "
              f"{len(iq)} samples @ {fs/1e9:.4f} GS/s")
        print(f"  sync   : first preamble sample {r.timing}, {r.preamble_symbols} preamble symbols, "
              f"SFD at symbol {r.sfd_symbol}, CFO {r.cfo_hz:.1f} Hz, phase {np.degrees(r.phase_rad):.1f} deg")
        if cfg.sts_packet_configuration == 3:
            sfd_ok = r.sfd_symbol >= 0 and r.preamble_symbols == cfg.preamble_duration
            print(f"  SP3    : no PHR/PSDU; SHR {'OK' if sfd_ok else 'mismatch'} "
                  f"({r.preamble_symbols}/{cfg.preamble_duration} preamble symbols)")
            rc |= 0 if sfd_ok else 1
            continue
        print(f"  PHR    : {r.phr_status} {r.phr} (PHR {r.phr_cpb} / PSDU {r.data_cpb} chips per burst)")
        if r.psdu_bits is None:
            print(f"  status : {r.status}")
            rc = 1
            continue
        raw_err = int(np.sum(hrp.conv_encode(r.decoded).reshape(-1, 2) != r.hard))
        psdu = r.psdu_bytes
        fcs_ok = len(psdu) >= 2 and fcs16(psdu[:-2]) == int.from_bytes(psdu[-2:], "little")
        print(f"  PSDU   : {len(psdu)} B, channel bit errors {raw_err}, RS corrected {r.rs_corrected}, "
              f"EVM {r.evm_db:.1f} dB, MAC FCS {'OK' if fcs_ok else 'mismatch'}")
        print(f"  bytes  : {psdu.hex(' ')}")
        rc |= 0 if r.ok else 1
        if args.plot:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            os.makedirs(args.plot, exist_ok=True)
            stem = os.path.join(args.plot, os.path.splitext(os.path.basename(path))[0])
            for i, fig in enumerate(hrp.plot_stages(r, iq, fs, title=cfg.mode)):
                fig.savefig(f"{stem}_{i}.png", dpi=90)
                plt.close(fig)
    return rc


if __name__ == "__main__":
    sys.exit(main())
