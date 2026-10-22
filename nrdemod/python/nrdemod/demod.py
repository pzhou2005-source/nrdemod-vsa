"""Native WiFi (802.11a/b/n/ac/ax/ad/be) / Bluetooth LE / UWB demodulator.

Standalone extract of the Advantest in-tree native cores, callable from
Python on Linux and Windows, mirroring ``nrdemod`` (the 5G NR module) in
structure and conventions.

    >>> import numpy as np
    >>> from nrdemod import demod
    >>> cfg = demod.Config(sampleRate=20e6)
    >>> res = demod.demodulate(demod.WIFI_A, iq, cfg)
    >>> res.syncFound, res.syncRho
"""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np

from . import _demod_ffi as _ffi
from ._demod_ffi import DemodConfig, lib

__all__ = [
    "Config",
    "WifiBeDataConfig",
    "UwbFrameConfig",
    "Result",
    "demodulate",
    "demodulate_uwb_frame",
    "version",
    "WIFI_A",
    "WIFI_N",
    "WIFI_AC",
    "WIFI_AX",
    "WIFI_AD",
    "WIFI_B",
    "WIFI_BE",
    "BT_LE",
    "UWB_HRP",
    "UWB_BPRF",
    "UWB_HPRF",
    "UWB_EHPRF",
]

(
    WIFI_A,
    WIFI_N,
    WIFI_AC,
    WIFI_AX,
    WIFI_AD,
    WIFI_B,
    WIFI_BE,
    BT_LE,
    UWB_HRP,
) = range(9)

UWB_BPRF, UWB_HPRF, UWB_EHPRF = range(3)

_FAMILY_NAMES = {
    WIFI_A: "802.11a",
    WIFI_N: "802.11n",
    WIFI_AC: "802.11ac",
    WIFI_AX: "802.11ax",
    WIFI_AD: "802.11ad",
    WIFI_B: "802.11b",
    WIFI_BE: "802.11be",
    BT_LE: "BluetoothLE",
    UWB_HRP: "UWB-HRP",
}

_FIELD_CACHE: Dict[int, Dict[int, list]] = {}


def _fields(family: int, kind: int) -> list:
    cache = _FIELD_CACHE.setdefault(family, {})
    if kind not in cache:
        cache[kind] = _ffi.field_names(family, kind)
    return cache[kind]


def version() -> str:
    return lib.demod_version().decode("ascii")


@dataclass
class Config:
    """Demodulator settings, shared across every family (see demod_c.h for
    which fields each family actually reads).

    UWB phyMode selects UWB_BPRF, UWB_HPRF, or UWB_EHPRF. All three
    provide burst/preamble synchronization and CFO, not payload decoding.
    These native mode values differ from waveform sidecar/VSA numbering.
    """

    sampleRate: float = 0.0
    burstSearch: bool = True
    burstSearchThresholdDb: float = 15.0
    syncThreshold: float = 0.5
    phy: int = 1  # LE_1M
    modulationIndex: float = 0.5
    accessAddress: int = 0x71764129
    phyMode: int = UWB_BPRF
    syncLength: int = 64
    chipRateHz: float = 499.2e6
    steadyFrequencyDeviation: bool = False

    def _to_c(self, family: int) -> DemodConfig:
        raw = DemodConfig()
        lib.demod_config_default(family, ctypes.byref(raw))
        raw.sampleRate = self.sampleRate
        raw.burstSearch = int(self.burstSearch)
        raw.burstSearchThresholdDb = self.burstSearchThresholdDb
        raw.syncThreshold = self.syncThreshold
        raw.phy = self.phy
        raw.modulationIndex = self.modulationIndex
        raw.accessAddress = self.accessAddress
        raw.phyMode = self.phyMode
        raw.syncLength = self.syncLength
        raw.chipRateHz = self.chipRateHz
        raw.steadyFrequencyDeviation = int(self.steadyFrequencyDeviation)
        return raw


class Result:
    """Demodulation output. Scalars are attributes, vectors are numpy arrays."""

    __slots__ = ("_scalars", "_vectors", "ok", "error", "family", "dataError")

    def __init__(self, family: int, handle):
        self.family = family
        self.ok = bool(lib.demod_result_ok(handle))
        self.error = lib.demod_result_error(handle).decode("utf-8", "replace")
        self.dataError = (lib.demod_result_data_error(handle).decode("utf-8", "replace")
                  if hasattr(lib, "demod_result_data_error") else "")
        self._scalars: Dict[str, object] = {}
        self._vectors: Dict[str, np.ndarray] = {}

        bv, iv, dv = ctypes.c_int(), ctypes.c_int(), ctypes.c_double()
        for name in _fields(family, 0):
            if lib.demod_result_get_bool(handle, name.encode(), ctypes.byref(bv)) == 0:
                self._scalars[name] = bool(bv.value)
        for name in _fields(family, 1):
            if lib.demod_result_get_int(handle, name.encode(), ctypes.byref(iv)) == 0:
                self._scalars[name] = iv.value
        for name in _fields(family, 2):
            if lib.demod_result_get_double(handle, name.encode(), ctypes.byref(dv)) == 0:
                self._scalars[name] = dv.value

        for name in _fields(family, 3):
            self._vectors[name] = self._copy_real(handle, name)
        for name in _fields(family, 4):
            self._vectors[name] = self._copy_int(handle, name)

    @staticmethod
    def _copy_real(handle, name) -> np.ndarray:
        n = lib.demod_result_vector_size(handle, name.encode())
        if n <= 0:
            return np.empty(0, dtype=np.float64)
        out = np.empty(n, dtype=np.float64)
        lib.demod_result_vector_copy(
            handle, name.encode(), out.ctypes.data_as(ctypes.POINTER(ctypes.c_double)), n
        )
        return out

    @staticmethod
    def _copy_int(handle, name) -> np.ndarray:
        n = lib.demod_result_ivector_size(handle, name.encode())
        if n <= 0:
            return np.empty(0, dtype=np.int32)
        out = np.empty(n, dtype=np.int32)
        lib.demod_result_ivector_copy(
            handle, name.encode(), out.ctypes.data_as(ctypes.POINTER(ctypes.c_int)), n
        )
        return out

    def __getattr__(self, name):
        if name in self._scalars:
            return self._scalars[name]
        if name in self._vectors:
            return self._vectors[name]
        raise AttributeError(name)

    def __dir__(self):
        return list(super().__dir__()) + list(self._scalars) + list(self._vectors)

    def as_dict(self, include_vectors: bool = False) -> dict:
        out = dict(self._scalars)
        out["ok"] = self.ok
        out["error"] = self.error
        if self.family == WIFI_BE:
            out["dataError"] = self.dataError
        out["family"] = _FAMILY_NAMES.get(self.family, "?")
        if include_vectors:
            out.update(self._vectors)
        return out

    def __repr__(self) -> str:
        fam = _FAMILY_NAMES.get(self.family, "?")
        if not self.ok:
            return f"<Result[{fam}] failed: {self.error}>"
        return f"<Result[{fam}] ok {self._scalars}>"


def _as_interleaved(iq) -> np.ndarray:
    a = np.asarray(iq)
    if np.iscomplexobj(a):
        return np.ascontiguousarray(a.astype(np.complex64)).view(np.float32)
    a = np.ascontiguousarray(a.astype(np.float32).ravel())
    if a.size % 2:
        raise ValueError("interleaved I/Q input needs an even number of floats")
    return a


@dataclass
class WifiBeDataConfig:
    """Explicit RU242 DATA layout; FFT offsets use 20 MSa/s samples from PPDU start."""

    dataBandwidthMHz: int = 20
    numSpatialStreams: int = 1
    ehtLtfType: int = 4
    dataMcs: int = 7
    ehtLtfFftOffsetSamples: int = 864
    dataFftOffsetSamples: int = 1184
    dataGuardIntervalSamples: int = 64
    numDataSymbols: int = 4


@dataclass
class UwbFrameConfig:
    """Legacy/BPRF/HPRF/EHPRF frame settings; no payload reference.

    STS key/IV defaults are public test-vector values, not production keys.
    Frame positions follow the acquired pulse lattice, including filter delay.
    STS correlation is a waveform check, not a secure-ranging certification.
    Dynamic EHPRF headers signal rate and BCC/LDPC selection on the air.
    macFcsLength is 0 (not checked), 2 (CRC-16), or 4 (CRC-32).
    Native EVM uses a SYNC-trained pulse model on the chip lattice; it is
    not an assertion of equivalence to VSA's EVM measurement.
    """

    codeIndex: int = 3
    sfdIndex: int = 0
    stsPacketConfig: int = 0
    samplesPerPulse: int = 4
    legacy: bool = True
    highRatePhr: bool = False
    stsKeyHex: str = "4A5572BC90798C8E518D2449092F1B55"
    stsIvHex: str = "68DEBD3A599939DD57FDBB0E2A10FAC0"
    constraintLength: int = 3
    numStsSegments: int = 1
    meanPrfMHz: float = 249.6
    dynamicPhr: bool = True
    bitRateMbps: float = 27.2
    stsSegmentLength: int = 64
    extraStsGapLength: int = 0
    macFcsLength: int = 0
    extraStsGapIndex: int = 0
    legacyRateMbps: float = 6.81
    legacyMeanPrfMHz: float = 15.6
    preambleSpread: int = 0
    legacyVsaHopping: bool = True


def demodulate_uwb_frame(iq, config: Config, frame_config: UwbFrameConfig) -> Result:
    """Recover native SFD/STS/PHR/PSDU results; check stage validity flags.

    Payload length is decoded from PHR. No PayloadHex or field offsets are
    passed to the native receiver. Optional MAC FCS rejection is separate
    from psduValid (PHY decoding). stsRho is the minimum normalized keyed
    correlation across all STS segments. Inspect evmValid and stage flags
    before reading native field EVM, which is reported in percent.
    """
    if not hasattr(lib, "demod_uwb_frame"):
        raise RuntimeError("Rebuild demod.dll and restart the kernel to enable native UWB frames")
    flat = _as_interleaved(iq)
    raw = config._to_c(UWB_HRP)
    settings = frame_config.__dict__.copy()
    settings["stsKeyHex"] = frame_config.stsKeyHex.encode("ascii")
    settings["stsIvHex"] = frame_config.stsIvHex.encode("ascii")
    frame_raw = _ffi.UwbFrameConfig(structSize=ctypes.sizeof(_ffi.UwbFrameConfig), **settings)
    count = flat.size // 2
    pointer = flat.ctypes.data_as(ctypes.POINTER(ctypes.c_float)) if count else None
    handle = lib.demod_uwb_frame(pointer, count, ctypes.byref(raw), ctypes.byref(frame_raw))
    if not handle:
        raise RuntimeError("native UWB frame decoder returned NULL")
    try:
        return Result(UWB_HRP, handle)
    finally:
        lib.demod_result_free(handle)


def demodulate(family: int, iq, config: Optional[Config] = None,
               *, data_config: Optional[WifiBeDataConfig] = None, **overrides) -> Result:
    """Demodulates a complex baseband capture with the given family's native core.

    ``family`` is one of the module-level constants (WIFI_A, WIFI_N, ...).
    ``iq`` may be a complex numpy array or an interleaved real array.
    ``data_config`` enables the explicit-layout WIFI_BE DATA measurement.
    Check ``dataEvmValid`` before reading ``evmRmsData`` (a ratio, not percent);
    invalid EVM is absent and ``dataError`` describes the DATA failure.
    This path does not decode EHT signaling or recover payload bits.
    """
    cfg = Config() if config is None else config
    if overrides:
        cfg = Config(**{**cfg.__dict__, **overrides})

    flat = _as_interleaved(iq)
    num_samples = flat.size // 2
    raw = cfg._to_c(family)

    ptr = flat.ctypes.data_as(ctypes.POINTER(ctypes.c_float)) if num_samples else None
    if data_config is not None:
        if family != WIFI_BE:
            raise ValueError("data_config is supported only for WIFI_BE")
        if not hasattr(lib, "demod_wifi_be_data"):
            raise RuntimeError("Rebuild the demod library to enable Wi-Fi 7 DATA EVM")
        data_raw = _ffi.WifiBeDataConfig(**data_config.__dict__)
        handle = lib.demod_wifi_be_data(ptr, num_samples, ctypes.byref(raw), ctypes.byref(data_raw))
    else:
        handle = lib.demod_demodulate(family, ptr, num_samples, ctypes.byref(raw))
    if not handle:
        raise RuntimeError("demod_demodulate returned NULL")
    try:
        return Result(family, handle)
    finally:
        lib.demod_result_free(handle)
