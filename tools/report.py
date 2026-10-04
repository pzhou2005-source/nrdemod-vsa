"""Report writers for the nrdemod / 89600 correlation runs.

Produces three artefacts per run in ``reports/``:
  * ``correlation_log.csv``  one appended row per run, for trending
  * ``<case>_<stamp>.json``  the full record
  * ``<case>_<stamp>.html``  human readable summary with an embedded plot
"""

from __future__ import annotations

import base64
import csv
import datetime as _dt
import io
import json
import os
import re
from typing import List

import numpy as np

_CSV_COLUMNS = [
    "timestamp",
    "case",
    "mode",
    "source",
    "sampleRateHz",
    "numSamples",
    "modulation",
    "rmsEvmPercent",
    "peakEvmPercent",
    "frequencyErrorHz",
    "numResourceBlocks",
    "numSymbols",
    "vsaEvmPercent",
    "evmDeltaPoints",
    "evmDeltaRelPercent",
    "vsaFreqErrorHz",
    "freqDeltaHz",
    "pass",
]


def _slug(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", str(text))[:60]


def _append_csv(path: str, record: dict) -> None:
    row = {k: "" for k in _CSV_COLUMNS}
    row.update({k: record.get(k, "") for k in ("timestamp", "case", "mode", "source", "sampleRateHz", "numSamples")})
    for key, value in record.get("nrdemod", {}).items():
        if key in _CSV_COLUMNS:
            row[key] = value
    for key, value in record.get("comparison", {}).items():
        if key in _CSV_COLUMNS:
            row[key] = value

    exists = os.path.isfile(path)
    with open(path, "a", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=_CSV_COLUMNS)
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def _plot_png(result) -> str:
    """Returns a base64 PNG, or an empty string when matplotlib is absent."""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return ""

    fig, ax = plt.subplots(2, 2, figsize=(11, 7.5))
    c = result.constellation
    if c.size:
        ax[0][0].plot(c.real, c.imag, ".", markersize=1.5)
        ax[0][0].set_aspect("equal")
    ax[0][0].set_title(f"Constellation ({result.modulation_name})")
    ax[0][1].plot(result.evmPerSubcarrier)
    ax[0][1].set_title("EVM per subcarrier (%)")
    ax[1][0].plot(result.evmPerSymbol)
    ax[1][0].set_title("EVM per symbol (%)")
    ax[1][1].plot(result.spectrumDb)
    ax[1][1].set_title("Spectrum (dB)")
    fig.tight_layout()

    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=96)
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode("ascii")


_HTML = """<!doctype html>
<meta charset="utf-8">
<title>nrdemod vs 89600 - {case}</title>
<style>
 body {{ font-family: Segoe UI, sans-serif; margin: 2rem; color: #222; }}
 h1 {{ font-size: 1.3rem; }}
 table {{ border-collapse: collapse; margin-bottom: 1.5rem; }}
 td, th {{ border: 1px solid #ccc; padding: 3px 10px; font-size: 0.86rem; text-align: left; }}
 th {{ background: #f3f3f3; }}
 .pass {{ color: #0a0; font-weight: bold; }}
 .fail {{ color: #c00; font-weight: bold; }}
 .na   {{ color: #888; }}
 img {{ max-width: 100%; border: 1px solid #ddd; }}
</style>
<h1>nrdemod vs Keysight 89600 &mdash; {case}</h1>
<p>{timestamp} &middot; mode <b>{mode}</b> &middot; source <code>{source}</code> &middot;
   {num_samples} samples @ {fs_mhz:.4f} MHz &middot; verdict {verdict}</p>
{tables}
{image}
"""


def _table(title: str, data: dict) -> str:
    rows = "".join(
        f"<tr><th>{k}</th><td>{v if not isinstance(v, float) else f'{v:.6g}'}</td></tr>"
        for k, v in data.items()
    )
    return f"<h2 style='font-size:1rem'>{title}</h2><table>{rows}</table>"


def write_reports(directory: str, record: dict, result, plot: bool = True) -> List[str]:
    os.makedirs(directory, exist_ok=True)
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = os.path.join(directory, f"{_slug(record.get('case', 'run'))}_{stamp}")
    written = []

    csv_path = os.path.join(directory, "correlation_log.csv")
    _append_csv(csv_path, record)
    written.append(csv_path)

    json_path = stem + ".json"
    with open(json_path, "w") as fh:
        json.dump(record, fh, indent=2, default=float)
    written.append(json_path)

    passed = record.get("comparison", {}).get("pass")
    verdict = (
        "<span class='pass'>PASS</span>"
        if passed is True
        else "<span class='fail'>FAIL</span>"
        if passed is False
        else "<span class='na'>n/a (no VSA reference)</span>"
    )

    tables = _table("nrdemod", record.get("nrdemod", {}))
    if record.get("comparison"):
        tables += _table("comparison", record["comparison"])

    png = _plot_png(result) if plot else ""
    image = f"<img src='data:image/png;base64,{png}'>" if png else "<p class='na'>matplotlib not installed - no plot.</p>"

    html_path = stem + ".html"
    with open(html_path, "w") as fh:
        fh.write(
            _HTML.format(
                case=record.get("case", "run"),
                timestamp=record.get("timestamp", stamp),
                mode=record.get("mode", "?"),
                source=record.get("source", "?"),
                num_samples=record.get("numSamples", 0),
                fs_mhz=float(record.get("sampleRateHz", 0)) / 1e6,
                verdict=verdict,
                tables=tables,
                image=image,
            )
        )
    written.append(html_path)

    npz_path = stem + ".npz"
    vectors = {k: v for k, v in result.as_dict(True).items() if isinstance(v, np.ndarray) and v.size}
    if vectors:
        np.savez_compressed(npz_path, **vectors)
        written.append(npz_path)

    return written
