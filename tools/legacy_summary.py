"""Merges re-run rows into legacy_correlation.csv and prints a per-family summary.

    python tools/legacy_summary.py [--merge reports/legacy_11n.csv]
"""

from __future__ import annotations

import argparse
import csv
import os
from collections import defaultdict

_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_MAIN = os.path.join(_ROOT, "reports", "legacy_correlation.csv")


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--merge", help="CSV whose rows replace the same (family, waveform) rows in the main CSV")
    args = p.parse_args()

    with open(_MAIN, newline="") as fh:
        reader = csv.DictReader(fh)
        cols, rows = reader.fieldnames, list(reader)
    if args.merge:
        with open(args.merge, newline="") as fh:
            reader = csv.DictReader(fh)
            new = {(r["family"], r["waveform"]): r for r in reader}
            cols = list(dict.fromkeys(cols + list(reader.fieldnames)))
        rows = [new.pop((r["family"], r["waveform"]), r) for r in rows]
        rows += list(new.values())
        with open(_MAIN, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=cols, restval="")
            w.writeheader()
            w.writerows(rows)

    stats = defaultdict(lambda: defaultdict(int))
    worst = defaultdict(float)
    for r in rows:
        s = stats[r["family"]]
        s["total"] += 1
        s[r["status"] if r["status"] != "ok" else {"True": "pass", "False": "fail"}.get(r["pass"], "noref")] += 1
        if r["delta_evm_pp"]:
            worst[r["family"]] = max(worst[r["family"]], abs(float(r["delta_evm_pp"])))
    print(f"{'family':20} {'total':>5} {'pass':>5} {'fail':>5} {'noref':>6} {'unsup':>6} {'other':>6}  max|dEVM| [pp]")
    for fam, s in sorted(stats.items()):
        other = s["total"] - s["pass"] - s["fail"] - s["noref"] - s["unsupported"]
        tail = f"{worst[fam]:.5f}" if fam in worst else "-"
        print(f"{fam:20} {s['total']:5} {s['pass']:5} {s['fail']:5} {s['noref']:6} {s['unsupported']:6} {other:6}  {tail}")


if __name__ == "__main__":
    main()
