"""ctypes binding to the demod shared library (WiFi/Bluetooth LE/UWB cores).

Keeps the raw C ABI in one place; the friendly API lives in ``nrdemod.demod``.
Mirrors ``_ffi.py`` (the nrdemod/5G NR binding) in structure and conventions.
"""

from __future__ import annotations

import ctypes
import os
import sys
from ctypes import POINTER, c_char_p, c_double, c_float, c_int, c_long, c_size_t, c_uint32

ABI_VERSION = 1

_CANDIDATE_NAMES = {
    "win32": ("demod.dll", "libdemod.dll"),
    "darwin": ("demod.dylib", "libdemod.dylib"),
}.get(sys.platform, ("demod.so", "libdemod.so"))


class DemodConfig(ctypes.Structure):
    """Mirror of ``DemodConfig`` in demod_c.h (field order must match)."""

    _fields_ = [
        ("abiVersion", c_int),
        ("sampleRate", c_double),
        ("burstSearch", c_int),
        ("burstSearchThresholdDb", c_double),
        ("syncThreshold", c_double),
        ("phy", c_int),
        ("modulationIndex", c_double),
        ("accessAddress", c_uint32),
        ("phyMode", c_int),
        ("syncLength", c_int),
        ("chipRateHz", c_double),
    ]


_ResultPtr = ctypes.c_void_p


def _search_paths() -> list:
    here = os.path.dirname(os.path.abspath(__file__))
    paths = []
    env = os.environ.get("DEMOD_LIBRARY")
    if env:
        paths.append(env)
    for root in (os.path.join(here, "_lib"), here, os.path.join(here, "..", "..", "build")):
        for name in _CANDIDATE_NAMES:
            paths.append(os.path.join(root, name))
            paths.append(os.path.join(root, "Release", name))
    paths.extend(_CANDIDATE_NAMES)  # fall back to the loader search path
    return paths


def _load():
    tried = []
    kwargs = {"winmode": 0} if sys.platform == "win32" else {}
    for path in _search_paths():
        if os.path.sep in path and not os.path.isfile(path):
            continue
        try:
            return ctypes.CDLL(path, **kwargs)
        except OSError as exc:  # pragma: no cover - platform dependent
            tried.append(f"{path}: {exc}")
    raise OSError(
        "demod shared library not found. Build it with CMake or set "
        "DEMOD_LIBRARY to the full path.\nTried:\n  " + "\n  ".join(tried or _CANDIDATE_NAMES)
    )


lib = _load()

lib.demod_version.restype = c_char_p
lib.demod_abi_version.restype = c_int

lib.demod_config_default.argtypes = [c_int, POINTER(DemodConfig)]
lib.demod_config_default.restype = c_int

lib.demod_demodulate.argtypes = [c_int, POINTER(c_float), c_size_t, POINTER(DemodConfig)]
lib.demod_demodulate.restype = _ResultPtr

lib.demod_demodulate64.argtypes = [c_int, POINTER(c_double), c_size_t, POINTER(DemodConfig)]
lib.demod_demodulate64.restype = _ResultPtr

lib.demod_result_free.argtypes = [_ResultPtr]
lib.demod_result_free.restype = None

lib.demod_result_ok.argtypes = [_ResultPtr]
lib.demod_result_ok.restype = c_int

lib.demod_result_error.argtypes = [_ResultPtr]
lib.demod_result_error.restype = c_char_p

lib.demod_result_get_bool.argtypes = [_ResultPtr, c_char_p, POINTER(c_int)]
lib.demod_result_get_bool.restype = c_int

lib.demod_result_get_int.argtypes = [_ResultPtr, c_char_p, POINTER(c_int)]
lib.demod_result_get_int.restype = c_int

lib.demod_result_get_double.argtypes = [_ResultPtr, c_char_p, POINTER(c_double)]
lib.demod_result_get_double.restype = c_int

for _stem, _dtype in (("vector", c_double), ("ivector", c_int)):
    getattr(lib, f"demod_result_{_stem}_size").argtypes = [_ResultPtr, c_char_p]
    getattr(lib, f"demod_result_{_stem}_size").restype = c_long
    getattr(lib, f"demod_result_{_stem}_copy").argtypes = [
        _ResultPtr,
        c_char_p,
        POINTER(_dtype),
        c_size_t,
    ]
    getattr(lib, f"demod_result_{_stem}_copy").restype = c_int

lib.demod_field_names.argtypes = [c_int, c_int]
lib.demod_field_names.restype = ctypes.c_void_p

if lib.demod_abi_version() != ABI_VERSION:
    raise RuntimeError(
        f"demod ABI mismatch: library {lib.demod_abi_version()}, binding {ABI_VERSION}"
    )


def field_names(family: int, kind: int) -> list:
    """kind: 0=bool 1=int 2=double 3=vector<double> 4=vector<int>."""
    addr = lib.demod_field_names(family, kind)
    if not addr:
        return []
    out = []
    offset = 0
    while True:
        name = ctypes.string_at(addr + offset)
        if not name:
            break
        out.append(name.decode("ascii"))
        offset += len(name) + 1
    return out
