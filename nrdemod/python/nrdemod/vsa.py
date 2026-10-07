"""Keysight 89600 VSA interoperability helpers.

Loads I/Q recordings exported by the 89600 VSA so the same samples can be
pushed through :func:`nrdemod.demodulate`, and writes captures back out in a
format the VSA can re-import for A/B comparison.

Supported inputs
----------------
``.mat``   89600 recording saved as MATLAB (needs ``scipy``). Uses ``Y`` for
           the complex samples and ``XDelta``/``InputZoom`` for the rate.
``.csv``   89600 trace or recording export; header lines are skipped, then
           ``I,Q`` or ``t,I,Q`` columns.
``.txt``   same parsing as ``.csv``.
``.npy``   numpy array, complex or interleaved real.
``.bin``   raw interleaved samples, dtype given by ``dtype=``.
"""

from __future__ import annotations

import os
import re
from typing import Optional, Tuple

import numpy as np

__all__ = ["load_iq", "save_csv", "save_mat", "compare_evm"]

_NUMERIC = re.compile(r"^[\s+\-0-9.eE,;\t]+$")


def _parse_text(path: str) -> Tuple[np.ndarray, Optional[float]]:
    sample_rate = None
    rows = []
    with open(path, "r", errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            if not _NUMERIC.match(line):
                m = re.search(r"(XDelta|SampleInterval)\s*[,;:=\t ]\s*([0-9.eE+\-]+)", line, re.I)
                if m:
                    delta = float(m.group(2))
                    if delta > 0:
                        sample_rate = 1.0 / delta
                    continue
                m = re.search(r"(SampleRate|Fs)\s*[,;:=\t ]\s*([0-9.eE+\-]+)", line, re.I)
                if m:
                    sample_rate = float(m.group(2))
                continue
            parts = [p for p in re.split(r"[,;\t ]+", line) if p]
            try:
                rows.append([float(p) for p in parts])
            except ValueError:
                continue

    if not rows:
        raise ValueError(f"no numeric data found in {path}")

    arr = np.asarray(rows, dtype=np.float64)
    if arr.shape[1] >= 3:
        if sample_rate is None and arr.shape[0] > 1:
            dt = arr[1, 0] - arr[0, 0]
            if dt > 0:
                sample_rate = 1.0 / dt
        iq = arr[:, 1] + 1j * arr[:, 2]
    elif arr.shape[1] == 2:
        iq = arr[:, 0] + 1j * arr[:, 1]
    else:
        flat = arr.ravel()
        if flat.size % 2:
            raise ValueError("single column file needs an even number of interleaved values")
        iq = flat[0::2] + 1j * flat[1::2]
    return iq.astype(np.complex64), sample_rate


def _parse_mat(path: str) -> Tuple[np.ndarray, Optional[float]]:
    try:
        from scipy.io import loadmat
    except ImportError as exc:  # pragma: no cover
        raise ImportError("reading 89600 .mat recordings requires scipy") from exc

    mat = loadmat(path, squeeze_me=True, struct_as_record=False)
    data = None
    for key in ("Y", "y", "data", "IQ", "iq"):
        if key in mat:
            data = np.asarray(mat[key]).ravel()
            break
    if data is None:
        candidates = [v for k, v in mat.items() if not k.startswith("__") and np.size(v) > 16]
        if not candidates:
            raise ValueError(f"no sample vector found in {path}")
        data = np.asarray(candidates[0]).ravel()

    sample_rate = None
    for key in ("XDelta", "xDelta", "dt"):
        if key in mat:
            delta = float(np.ravel(mat[key])[0])
            if delta > 0:
                sample_rate = 1.0 / delta
            break
    if sample_rate is None:
        for key in ("Fs", "SampleRate", "InputZoom"):
            if key in mat:
                sample_rate = float(np.ravel(mat[key])[0])
                break

    if not np.iscomplexobj(data):
        if data.size % 2:
            raise ValueError("real .mat vector needs an even number of interleaved values")
        data = data[0::2] + 1j * data[1::2]
    return data.astype(np.complex64), sample_rate


def load_iq(
    path: str,
    sample_rate: Optional[float] = None,
    dtype: str = "float32",
) -> Tuple[np.ndarray, Optional[float]]:
    """Returns ``(complex64 samples, sample_rate_or_None)``.

    ``sample_rate`` overrides whatever is found in the file.
    """
    ext = os.path.splitext(path)[1].lower()
    if ext == ".mat":
        iq, fs = _parse_mat(path)
    elif ext == ".npy":
        arr = np.load(path)
        if np.iscomplexobj(arr):
            iq = arr.astype(np.complex64).ravel()
        else:
            flat = arr.astype(np.float64).ravel()
            iq = (flat[0::2] + 1j * flat[1::2]).astype(np.complex64)
        fs = None
    elif ext in (".bin", ".raw", ".iq"):
        flat = np.fromfile(path, dtype=np.dtype(dtype))
        if flat.size % 2:
            raise ValueError("raw file needs an even number of interleaved values")
        iq = (flat[0::2] + 1j * flat[1::2]).astype(np.complex64)
        fs = None
    else:
        iq, fs = _parse_text(path)

    return iq, (sample_rate if sample_rate is not None else fs)


def save_csv(path: str, iq: np.ndarray, sample_rate: float) -> None:
    """Writes an 89600-importable time record (header + ``I,Q`` rows)."""
    iq = np.asarray(iq).ravel()
    if not np.iscomplexobj(iq):
        iq = iq[0::2] + 1j * iq[1::2]
    iq = iq.astype(np.complex128)
    with open(path, "w") as fh:
        fh.write("XStart,0\n")
        fh.write(f"XDelta,{1.0 / float(sample_rate)!r}\n")
        fh.write("XDomain,2\n")
        fh.write("XUnit,Sec\n")
        fh.write("YUnit,V\n")
        fh.write(f"NumPoints,{iq.size}\n")
        fh.write("Y\n")
        for s in iq:
            fh.write(f"{float(s.real)!r},{float(s.imag)!r}\n")


def save_mat(path: str, iq: np.ndarray, sample_rate: float, center_hz: float = 0.0) -> None:
    """Writes an 89600 style MATLAB recording (``Y`` + the X/input metadata)."""
    try:
        from scipy.io import savemat
    except ImportError as exc:  # pragma: no cover
        raise ImportError("writing 89600 .mat recordings requires scipy") from exc

    iq = np.asarray(iq).ravel()
    if not np.iscomplexobj(iq):
        iq = iq[0::2] + 1j * iq[1::2]
    savemat(path, {
        "Y": iq.astype(np.complex128).reshape(-1, 1),
        "XStart": 0.0,
        "XDelta": 1.0 / float(sample_rate),
        "XDomain": 2,          # time domain
        "XUnit": "Sec",
        "YUnit": "V",
        "InputCenter": float(center_hz),
        "InputZoom": 1,        # complex baseband rather than a real band
        "InputRefImped": 50.0,
    }, do_compression=False)


def compare_evm(result, vsa_evm_percent: float, tolerance_percent: float = 0.5) -> dict:
    """Compares a :class:`nrdemod.Result` against a VSA EVM reading."""
    ours = float(result.rmsEvmPercent)
    delta = ours - vsa_evm_percent
    rel = delta / vsa_evm_percent * 100.0 if vsa_evm_percent else float("inf")
    return {
        "nrdemod_evm_percent": ours,
        "vsa_evm_percent": vsa_evm_percent,
        "delta_percent_points": delta,
        "delta_relative_percent": rel,
        "within_tolerance": abs(delta) <= tolerance_percent,
        "nrdemod_freq_error_hz": float(result.frequencyErrorHz),
        "nrdemod_modulation": result.modulation_name,
    }
