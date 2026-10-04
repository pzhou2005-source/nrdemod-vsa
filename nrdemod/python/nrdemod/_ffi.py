"""ctypes binding to the nrdemod shared library.

Keeps the raw C ABI in one place; the friendly API lives in ``nrdemod``.
"""

from __future__ import annotations

import ctypes
import os
import sys
from ctypes import POINTER, c_char_p, c_double, c_float, c_int, c_long, c_size_t, c_uint

ABI_VERSION = 1

_CANDIDATE_NAMES = {
    "win32": ("nrdemod.dll", "libnrdemod.dll"),
    "darwin": ("nrdemod.dylib", "libnrdemod.dylib"),
}.get(sys.platform, ("nrdemod.so", "libnrdemod.so"))


class NrdConfig(ctypes.Structure):
    """Mirror of ``NrdConfig`` in nrdemod_c.h (field order must match)."""

    _fields_ = [
        ("abiVersion", c_int),
        ("numerology", c_int),
        ("fftSize", c_int),
        ("frequencyRange", c_int),
        ("channelBandwidthMHz", c_int),
        ("numResourceBlocks", c_int),
        ("resourceBlockOffset", c_int),
        ("extendedCp", c_int),
        ("modulation", c_int),
        ("maxSymbols", c_int),
        ("burstSearch", c_int),
        ("burstSearchThresholdDb", c_double),
        ("timingSearch", c_int),
        ("syncSearchSymbols", c_int),
        ("cfoCorrection", c_int),
        ("carrierOffsetHz", c_double),
        ("equalize", c_int),
        ("equalizerIterations", c_int),
        ("phaseTracking", c_int),
        ("timingTracking", c_int),
        ("amplitudeTracking", c_int),
        ("transformPrecoding", c_int),
        ("dcPunctured", c_int),
        ("removeIqOffset", c_int),
        ("symbolTimingAdjustmentPercent", c_double),
        ("resample", c_int),
        ("sampleRate", c_double),
        ("dmrsExclude", c_int),
        ("dmrsMappingTypeB", c_int),
        ("dmrsTypeAPosition", c_int),
        ("dmrsAdditionalPositions", c_int),
        ("dmrsDoubleSymbol", c_int),
        ("excludedSymbols", POINTER(c_int)),
        ("numExcludedSymbols", c_int),
    ]


_ResultPtr = ctypes.c_void_p


def _search_paths() -> list:
    here = os.path.dirname(os.path.abspath(__file__))
    paths = []
    env = os.environ.get("NRDEMOD_LIBRARY")
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
    for path in _search_paths():
        if os.path.sep in path and not os.path.isfile(path):
            continue
        try:
            return ctypes.CDLL(path)
        except OSError as exc:  # pragma: no cover - platform dependent
            tried.append(f"{path}: {exc}")
    raise OSError(
        "nrdemod shared library not found. Build it with CMake or set "
        "NRDEMOD_LIBRARY to the full path.\nTried:\n  " + "\n  ".join(tried or _CANDIDATE_NAMES)
    )


lib = _load()

lib.nrd_version.restype = c_char_p
lib.nrd_abi_version.restype = c_int

lib.nrd_config_default.argtypes = [POINTER(NrdConfig)]
lib.nrd_config_default.restype = c_int

lib.nrd_demodulate.argtypes = [POINTER(c_float), c_size_t, POINTER(NrdConfig)]
lib.nrd_demodulate.restype = _ResultPtr

lib.nrd_demodulate64.argtypes = [POINTER(c_double), c_size_t, POINTER(NrdConfig)]
lib.nrd_demodulate64.restype = _ResultPtr

lib.nrd_result_free.argtypes = [_ResultPtr]
lib.nrd_result_free.restype = None

lib.nrd_result_ok.argtypes = [_ResultPtr]
lib.nrd_result_ok.restype = c_int

lib.nrd_result_error.argtypes = [_ResultPtr]
lib.nrd_result_error.restype = c_char_p

lib.nrd_result_get_double.argtypes = [_ResultPtr, c_char_p, POINTER(c_double)]
lib.nrd_result_get_double.restype = c_int

lib.nrd_result_get_int.argtypes = [_ResultPtr, c_char_p, POINTER(c_int)]
lib.nrd_result_get_int.restype = c_int

for _stem, _dtype in (("vector", c_double), ("ivector", c_int), ("cvector", c_double)):
    getattr(lib, f"nrd_result_{_stem}_size").argtypes = [_ResultPtr, c_char_p]
    getattr(lib, f"nrd_result_{_stem}_size").restype = c_long
    getattr(lib, f"nrd_result_{_stem}_copy").argtypes = [
        _ResultPtr,
        c_char_p,
        POINTER(_dtype),
        c_size_t,
    ]
    getattr(lib, f"nrd_result_{_stem}_copy").restype = c_int

lib.nrd_field_names.argtypes = [c_int]
lib.nrd_field_names.restype = ctypes.c_void_p

lib.nrd_carrier_resource_blocks.argtypes = [c_int, c_int, c_int]
lib.nrd_carrier_resource_blocks.restype = c_int

lib.nrd_generate_test_signal.argtypes = [
    POINTER(NrdConfig),
    c_int,
    c_int,
    c_uint,
    POINTER(c_size_t),
]
lib.nrd_generate_test_signal.restype = POINTER(c_float)

lib.nrd_free_signal.argtypes = [POINTER(c_float)]
lib.nrd_free_signal.restype = None

if lib.nrd_abi_version() != ABI_VERSION:
    raise RuntimeError(
        f"nrdemod ABI mismatch: library {lib.nrd_abi_version()}, binding {ABI_VERSION}"
    )


def field_names(kind: int) -> list:
    """kind: 0=int 1=double 2=vector 3=cvector 4=ivector."""
    addr = lib.nrd_field_names(kind)
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
