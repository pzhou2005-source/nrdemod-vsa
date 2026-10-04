"""Local COM link to the Keysight 89600 VSA running on this PC.

The 89600 registers the ``AgtVsaVector.Application`` automation object whenever
it runs, so the whole measurement can be driven locally even when the SCPI
server (Utilities > SCPI Configuration, TCP 5025) is switched off. The surface
mirrors :class:`vsa_link.Vsa89600` so ``correlate.py live`` can use either.

Only a spare display trace is borrowed to read data; it is restored afterwards,
so the traces the operator has on screen are left alone.
"""

from __future__ import annotations

from typing import List, Optional

import numpy as np

PROG_ID = "AgtVsaVector.Application"

# Trace.RawData(vType): 4 returns the unscaled doubles, interleaved I/Q when
# Trace.RawDataComplex is set.
_RAW_DOUBLES = 4
_NO_DATA = "No Data"

# in preference order: the vector mode record, then the digital demod equivalents
_TIME_TRACES = ("Main Time1", "Raw Main Time1", "IQ Meas Time1", "Time1", "Search Time1")
_SUMMARY_TRACES = ("Summary1", "CC0 Summary1", "Syms/Errs1")

# command-map keys -> summary table entries, 5G NR name first then digital demod
_SCALARS = {
    "query_evm": ("EVM", "EvmRms"),
    "query_peak_evm": ("EVMPk", "EvmPeak"),
    "query_freq_error": ("FrequencyError", "FreqErr"),
    "query_iq_offset": ("IQOffset", "IqOffset"),
    "query_rho": ("Rho",),
    "query_quadrature_error": ("IQQuadErr", "QuadErr"),
    "query_gain_imbalance": ("IQGainImb", "IqGainImbalance"),
    "query_timing_skew": ("IQTimingSkew",),
    "query_sync_correlation": ("SyncCorrelation",),
    "query_symbol_clock_error": ("SymbolClockError", "SymClkErr"),
}


class ComError(RuntimeError):
    pass


class _BorrowedTrace:
    """Points a spare trace at a data name and puts it back afterwards."""

    def __init__(self, traces, data_name: str):
        self._traces = traces
        self._data_name = data_name
        self._trace = None
        self._previous = None

    def __enter__(self):
        count = int(self._traces.Count)
        for index in range(1, count + 1):  # the VSA collections are 1 based
            trace = self._traces.Item(index)
            if trace.DataName == _NO_DATA:
                self._trace, self._previous = trace, _NO_DATA
                break
        if self._trace is None:
            self._trace = self._traces.Item(count)
            self._previous = self._trace.DataName
        self._trace.DataName = self._data_name
        if self._trace.DataName != self._data_name:
            raise ComError(f"the VSA does not offer the trace {self._data_name!r} in this mode")
        return self._trace

    def __exit__(self, *exc) -> None:
        try:
            self._trace.DataName = self._previous
        except Exception:  # noqa: BLE001 - never mask the original error
            pass


class VsaCom:
    """Connection to the local 89600, API compatible with ``VsaScpi``."""

    def __init__(self, prog_id: str = PROG_ID, timeout: float = 60.0):
        self.prog_id = prog_id
        self.timeout = timeout
        self.app = None

    def __enter__(self) -> "VsaCom":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def open(self) -> None:
        try:
            import comtypes.client as cc
        except ImportError:
            raise ComError("the COM link needs 'comtypes' (pip install comtypes)") from None
        try:
            self.app = cc.GetActiveObject(self.prog_id)
        except OSError:
            raise ComError(
                f"no running 89600 VSA found ({self.prog_id} is not in the running object table). "
                "Start the 89600 VSA software on this PC."
            ) from None

    def close(self) -> None:
        self.app = None  # the VSA keeps running, we only drop our reference

    @property
    def measurement(self):
        if self.app is None:
            raise ComError("not connected")
        return self.app.Measurement

    @property
    def traces(self):
        return self.app.Display.Traces

    def idn(self) -> str:
        return f"Keysight,89600 VSA,{self.app.Version},local COM"

    def errors(self) -> List[str]:
        return []  # the automation interface raises instead of queueing errors


class Vsa89600Com:
    """High level operations, same names as the SCPI flavoured ``Vsa89600``."""

    def __init__(self, link: VsaCom):
        self.link = link

    def configure(self, center_hz: float, span_hz: float, points: int) -> None:
        freq = self.link.measurement.Frequency
        if center_hz:
            freq.Center = float(center_hz)
        if span_hz:
            freq.Span = float(span_hz)
        if points:
            freq.Points = int(points)

    def single_measure(self, timeout: float = 60.0) -> None:
        meas = self.link.measurement
        meas.Continuous = False
        meas.Initialize()
        meas.Start()  # Initialize only arms the measurement, Start runs it
        meas.WaitForMeasDone(int(timeout * 1000.0))

    def _summary(self, entries) -> Optional[float]:
        available = set(self.link.traces.Item(1).DataNames)
        for table in _SUMMARY_TRACES:
            if table not in available:
                continue
            try:
                with _BorrowedTrace(self.link.traces, table) as trace:
                    names = list(trace.RawDataValueNames)
                    for entry in entries:
                        if entry not in names:
                            continue
                        # the VSA reports '***' for anything it could not measure
                        try:
                            return float(trace.RawDataValue(names.index(entry)))
                        except (TypeError, ValueError):
                            continue
            except Exception:  # noqa: BLE001 - table not readable in this mode
                continue
        return None

    def read_evm_percent(self) -> Optional[float]:
        return self._summary(_SCALARS["query_evm"])

    def read_scalar(self, key: str) -> Optional[float]:
        entries = _SCALARS.get(key)
        return self._summary(entries) if entries else None

    def read_summary(self) -> dict:
        """Every mapped scalar the current personality can supply."""
        return {key: self._summary(entries) for key, entries in _SCALARS.items()}

    def _time_trace_name(self) -> Optional[str]:
        available = set(self.link.traces.Item(1).DataNames)
        for name in _TIME_TRACES:
            if name in available:
                return name
        return None

    def fetch_time_iq(self) -> Optional[np.ndarray]:
        name = self._time_trace_name()
        if name is None:
            return None
        with _BorrowedTrace(self.link.traces, name) as trace:
            raw = trace.RawData(_RAW_DOUBLES)
            if not raw:
                return None
            flat = np.asarray(raw, dtype=np.float64)
            if not trace.RawDataComplex:
                return None
            self._last_x_delta = float(trace.RawDataXDelta)
            return (flat[0::2] + 1j * flat[1::2]).astype(np.complex64)

    def sample_rate(self) -> Optional[float]:
        delta = getattr(self, "_last_x_delta", None)
        if delta:
            return 1.0 / delta
        name = self._time_trace_name()
        if name is None:
            return None
        with _BorrowedTrace(self.link.traces, name) as trace:
            delta = float(trace.RawDataXDelta)
        return 1.0 / delta if delta else None

    def load_recording(self, path: str) -> None:
        """Plays a saved I/Q recording instead of the hardware input."""
        recording = self.link.measurement.Inputs.Recording
        recording.RecallFile(path, "")  # "" = detect the format from the file
        # a recording brings its own rate and centre, the measurement has to follow
        freq = self.link.measurement.Frequency
        freq.Span = float(recording.PlaySpan)
        freq.Center = float(recording.PlayCenter)

    def set_points(self, points: int) -> None:
        self.link.measurement.Frequency.Points = int(points)

    def save_recording(self, path: str) -> bool:
        try:
            self.link.measurement.Inputs.Recording.SaveFile(path, "")
            return True
        except Exception:  # noqa: BLE001
            return False

    def save_setup(self, path: str) -> bool:
        """Stores the current VSA setup so an experiment can be undone."""
        try:
            self.link.measurement.SaveFile(path)
            return True
        except Exception:  # noqa: BLE001
            return False
