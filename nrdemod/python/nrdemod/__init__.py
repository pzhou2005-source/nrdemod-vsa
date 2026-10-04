"""Native 5G NR CP-OFDM / DFT-s-OFDM demodulator.

Standalone extract of the Advantest in-tree ``nrdemod`` core, callable from
Python on Linux and Windows for offline analysis and correlation against a
Keysight 89600 VSA.

    >>> import numpy as np, nrdemod
    >>> cfg = nrdemod.Config(numerology=1, channelBandwidthMHz=100,
    ...                      sampleRate=122.88e6, modulation=nrdemod.QAM64)
    >>> iq = nrdemod.generate_test_signal(cfg, num_symbols=28)
    >>> res = nrdemod.demodulate(iq, cfg)
    >>> round(res.rmsEvmPercent, 3)
    0.0
"""

from __future__ import annotations

import ctypes
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

from . import _ffi
from ._ffi import NrdConfig, lib

__all__ = [
    "Config",
    "Result",
    "demodulate",
    "generate_test_signal",
    "carrier_resource_blocks",
    "version",
    "AUTO",
    "QPSK",
    "QAM16",
    "QAM64",
    "QAM256",
    "FR1",
    "FR2",
]

AUTO, QPSK, QAM16, QAM64, QAM256 = 0, 1, 2, 3, 4
FR1, FR2 = 1, 2

_MOD_NAMES = {AUTO: "AUTO", QPSK: "QPSK", QAM16: "16QAM", QAM64: "64QAM", QAM256: "256QAM"}

_INT_FIELDS = _ffi.field_names(0)
_DOUBLE_FIELDS = _ffi.field_names(1)
_VECTOR_FIELDS = _ffi.field_names(2)
_CVECTOR_FIELDS = _ffi.field_names(3)
_IVECTOR_FIELDS = _ffi.field_names(4)


def version() -> str:
    return lib.nrd_version().decode("ascii")


def carrier_resource_blocks(frequency_range: int, bandwidth_mhz: int, numerology: int) -> int:
    """3GPP TS 38.101 carrier RB count, 0 when the combination is undefined."""
    return int(lib.nrd_carrier_resource_blocks(frequency_range, bandwidth_mhz, numerology))


def _defaults() -> NrdConfig:
    raw = NrdConfig()
    if lib.nrd_config_default(ctypes.byref(raw)) != 0:
        raise RuntimeError("nrd_config_default failed")
    return raw


_DEFAULTS = _defaults()


@dataclass
class Config:
    """Demodulator settings. Defaults come straight from the C++ core."""

    numerology: int = _DEFAULTS.numerology
    fftSize: int = _DEFAULTS.fftSize
    frequencyRange: int = _DEFAULTS.frequencyRange
    channelBandwidthMHz: int = _DEFAULTS.channelBandwidthMHz
    numResourceBlocks: int = _DEFAULTS.numResourceBlocks
    resourceBlockOffset: int = _DEFAULTS.resourceBlockOffset
    extendedCp: bool = bool(_DEFAULTS.extendedCp)
    modulation: int = _DEFAULTS.modulation
    maxSymbols: int = _DEFAULTS.maxSymbols
    burstSearch: bool = bool(_DEFAULTS.burstSearch)
    burstSearchThresholdDb: float = _DEFAULTS.burstSearchThresholdDb
    timingSearch: bool = bool(_DEFAULTS.timingSearch)
    syncSearchSymbols: int = _DEFAULTS.syncSearchSymbols
    cfoCorrection: bool = bool(_DEFAULTS.cfoCorrection)
    carrierOffsetHz: float = _DEFAULTS.carrierOffsetHz
    equalize: bool = bool(_DEFAULTS.equalize)
    equalizerIterations: int = _DEFAULTS.equalizerIterations
    phaseTracking: bool = bool(_DEFAULTS.phaseTracking)
    timingTracking: bool = bool(_DEFAULTS.timingTracking)
    amplitudeTracking: bool = bool(_DEFAULTS.amplitudeTracking)
    transformPrecoding: bool = bool(_DEFAULTS.transformPrecoding)
    dcPunctured: bool = bool(_DEFAULTS.dcPunctured)
    removeIqOffset: bool = bool(_DEFAULTS.removeIqOffset)
    symbolTimingAdjustmentPercent: float = _DEFAULTS.symbolTimingAdjustmentPercent
    resample: bool = bool(_DEFAULTS.resample)
    sampleRate: float = _DEFAULTS.sampleRate
    dmrsExclude: bool = bool(_DEFAULTS.dmrsExclude)
    dmrsMappingTypeB: bool = bool(_DEFAULTS.dmrsMappingTypeB)
    dmrsTypeAPosition: int = _DEFAULTS.dmrsTypeAPosition
    dmrsAdditionalPositions: int = _DEFAULTS.dmrsAdditionalPositions
    dmrsDoubleSymbol: bool = bool(_DEFAULTS.dmrsDoubleSymbol)
    excludedSymbols: Sequence[int] = field(default_factory=list)

    def _to_c(self):
        raw = NrdConfig()
        lib.nrd_config_default(ctypes.byref(raw))
        for name, _ctype in NrdConfig._fields_:
            if name in ("abiVersion", "excludedSymbols", "numExcludedSymbols"):
                continue
            value = getattr(self, name)
            setattr(raw, name, int(value) if isinstance(value, bool) else value)

        keep = None
        if len(self.excludedSymbols):
            keep = (ctypes.c_int * len(self.excludedSymbols))(*self.excludedSymbols)
            raw.excludedSymbols = keep
            raw.numExcludedSymbols = len(self.excludedSymbols)
        return raw, keep


class Result:
    """Demodulation output. Scalars are attributes, vectors are numpy arrays."""

    __slots__ = ("_scalars", "_vectors", "ok", "error")

    def __init__(self, handle):
        self.ok = bool(lib.nrd_result_ok(handle))
        self.error = lib.nrd_result_error(handle).decode("utf-8", "replace")
        self._scalars: Dict[str, float] = {}
        self._vectors: Dict[str, np.ndarray] = {}

        iv, dv = ctypes.c_int(), ctypes.c_double()
        for name in _INT_FIELDS:
            if lib.nrd_result_get_int(handle, name.encode(), ctypes.byref(iv)) == 0:
                self._scalars[name] = iv.value
        for name in _DOUBLE_FIELDS:
            if lib.nrd_result_get_double(handle, name.encode(), ctypes.byref(dv)) == 0:
                self._scalars[name] = dv.value

        for name in _VECTOR_FIELDS:
            self._vectors[name] = self._copy_real(handle, name)
        for name in _IVECTOR_FIELDS:
            self._vectors[name] = self._copy_int(handle, name)
        for name in _CVECTOR_FIELDS:
            self._vectors[name] = self._copy_complex(handle, name)

    @staticmethod
    def _copy_real(handle, name) -> np.ndarray:
        n = lib.nrd_result_vector_size(handle, name.encode())
        if n <= 0:
            return np.empty(0, dtype=np.float64)
        out = np.empty(n, dtype=np.float64)
        lib.nrd_result_vector_copy(
            handle, name.encode(), out.ctypes.data_as(ctypes.POINTER(ctypes.c_double)), n
        )
        return out

    @staticmethod
    def _copy_int(handle, name) -> np.ndarray:
        n = lib.nrd_result_ivector_size(handle, name.encode())
        if n <= 0:
            return np.empty(0, dtype=np.int32)
        out = np.empty(n, dtype=np.int32)
        lib.nrd_result_ivector_copy(
            handle, name.encode(), out.ctypes.data_as(ctypes.POINTER(ctypes.c_int)), n
        )
        return out

    @staticmethod
    def _copy_complex(handle, name) -> np.ndarray:
        n = lib.nrd_result_cvector_size(handle, name.encode())
        if n <= 0:
            return np.empty(0, dtype=np.complex128)
        buf = np.empty(2 * n, dtype=np.float64)
        lib.nrd_result_cvector_copy(
            handle, name.encode(), buf.ctypes.data_as(ctypes.POINTER(ctypes.c_double)), n
        )
        return buf.view(np.complex128)

    def __getattr__(self, name):
        if name in self._scalars:
            return self._scalars[name]
        if name in self._vectors:
            return self._vectors[name]
        raise AttributeError(name)

    def __dir__(self):
        return list(super().__dir__()) + list(self._scalars) + list(self._vectors)

    @property
    def modulation_name(self) -> str:
        return _MOD_NAMES.get(self._scalars.get("detectedModulation", 0), "?")

    def as_dict(self, include_vectors: bool = False) -> dict:
        out = dict(self._scalars)
        out["ok"] = self.ok
        out["error"] = self.error
        out["modulation"] = self.modulation_name
        if include_vectors:
            out.update(self._vectors)
        return out

    def __repr__(self) -> str:
        if not self.ok:
            return f"<Result failed: {self.error}>"
        s = self._scalars
        return (
            "<Result ok rmsEvm={:.4f}%% peakEvm={:.3f}%% mod={} rb={} symbols={} "
            "fft={} freqErr={:.1f}Hz>".format(
                s.get("rmsEvmPercent", float("nan")),
                s.get("peakEvmPercent", float("nan")),
                self.modulation_name,
                s.get("numResourceBlocks", 0),
                s.get("numSymbols", 0),
                s.get("fftSize", 0),
                s.get("frequencyErrorHz", 0.0),
            )
        )


def _as_interleaved(iq) -> np.ndarray:
    a = np.asarray(iq)
    if np.iscomplexobj(a):
        return np.ascontiguousarray(a.astype(np.complex64)).view(np.float32)
    a = np.ascontiguousarray(a.astype(np.float32).ravel())
    if a.size % 2:
        raise ValueError("interleaved I/Q input needs an even number of floats")
    return a


def demodulate(iq, config: Optional[Config] = None, **overrides) -> Result:
    """Demodulates a complex baseband capture.

    ``iq`` may be a complex numpy array or an interleaved real array.
    Keyword overrides are applied on top of ``config``.
    """
    cfg = Config() if config is None else config
    if overrides:
        cfg = Config(**{**cfg.__dict__, **overrides})

    flat = _as_interleaved(iq)
    num_samples = flat.size // 2
    raw, _keep = cfg._to_c()

    ptr = flat.ctypes.data_as(ctypes.POINTER(ctypes.c_float)) if num_samples else None
    handle = lib.nrd_demodulate(ptr, num_samples, ctypes.byref(raw))
    if not handle:
        raise RuntimeError("nrd_demodulate returned NULL")
    try:
        return Result(handle)
    finally:
        lib.nrd_result_free(handle)


def generate_test_signal(
    config: Config,
    num_symbols: int,
    leading_samples: int = 0,
    seed: int = 1,
) -> np.ndarray:
    """Synthesises an ideal NR burst at ``config.sampleRate`` (complex64)."""
    raw, _keep = config._to_c()
    n = ctypes.c_size_t(0)
    ptr = lib.nrd_generate_test_signal(
        ctypes.byref(raw), num_symbols, leading_samples, seed, ctypes.byref(n)
    )
    if not ptr or n.value == 0:
        raise RuntimeError("test signal generation failed (check the configuration)")
    try:
        buf = np.ctypeslib.as_array(ptr, shape=(2 * n.value,))
        return buf.copy().view(np.complex64)
    finally:
        lib.nrd_free_signal(ptr)
