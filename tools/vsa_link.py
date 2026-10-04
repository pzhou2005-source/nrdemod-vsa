"""SCPI link to the Keysight 89600 VSA software (and X-Series analysers).

The 89600 VSA exposes a SCPI server (Utilities > SCPI Configuration, default
TCP port 5025). Mnemonics differ between VSA releases and measurement
personalities, so every command used here comes from an editable command map
(``configs/scpi_89600.json``) instead of being hard coded. Run

    python tools/vsa_link.py --probe

to confirm connectivity and check which mapped commands your installation
actually accepts.
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time
from typing import Dict, List, Optional

import numpy as np

DEFAULT_PORT = 5025
_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_MAP = os.path.join(_HERE, "..", "configs", "scpi_89600.json")

_COM_SCALARS = ("query_evm", "query_peak_evm", "query_freq_error", "query_iq_offset",
                "query_rho", "query_quadrature_error", "query_gain_imbalance",
                "query_timing_skew", "query_sync_correlation", "query_symbol_clock_error")


class ScpiError(RuntimeError):
    pass


class VsaScpi:
    """Minimal line based SCPI client, good enough for the 89600 server."""

    def __init__(self, host: str, port: int = DEFAULT_PORT, timeout: float = 20.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._sock: Optional[socket.socket] = None
        self._buf = b""

    def __enter__(self) -> "VsaScpi":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def open(self) -> None:
        try:
            self._sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        except OSError as exc:
            raise ScpiError(
                f"cannot reach the 89600 SCPI server at {self.host}:{self.port} ({exc}). "
                "Enable it under Utilities > SCPI Configuration in the VSA and check the firewall."
            ) from None
        self._sock.settimeout(self.timeout)
        self._buf = b""

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    # --- transport -------------------------------------------------------

    def write(self, command: str) -> None:
        if self._sock is None:
            raise ScpiError("not connected")
        self._sock.sendall(command.strip().encode("ascii") + b"\n")

    def _read_line(self) -> bytes:
        while b"\n" not in self._buf:
            chunk = self._sock.recv(65536)
            if not chunk:
                raise ScpiError("connection closed by the analyser")
            self._buf += chunk
        line, _, self._buf = self._buf.partition(b"\n")
        return line

    def query(self, command: str) -> str:
        self.write(command)
        return self._read_line().decode("ascii", "replace").strip()

    def _read_exact(self, n: int) -> bytes:
        while len(self._buf) < n:
            chunk = self._sock.recv(min(1 << 20, n - len(self._buf)))
            if not chunk:
                raise ScpiError("connection closed while reading a block")
            self._buf += chunk
        out, self._buf = self._buf[:n], self._buf[n:]
        return out

    def query_block(self, command: str, dtype=np.float32) -> np.ndarray:
        """Reads an IEEE 488.2 definite length block (``#<n><len><bytes>``)."""
        self.write(command)
        head = self._read_exact(1)
        if head != b"#":
            rest = self._read_line()
            raise ScpiError(f"expected a binary block, got {(head + rest)[:80]!r}")
        ndigits = int(self._read_exact(1))
        if ndigits == 0:
            raise ScpiError("indefinite length blocks are not supported")
        length = int(self._read_exact(ndigits))
        payload = self._read_exact(length)
        if self._buf[:1] == b"\n":
            self._buf = self._buf[1:]
        return np.frombuffer(payload, dtype=dtype)

    # --- conveniences ----------------------------------------------------

    def idn(self) -> str:
        return self.query("*IDN?")

    def opc(self, timeout: Optional[float] = None) -> bool:
        old = self._sock.gettimeout()
        if timeout:
            self._sock.settimeout(timeout)
        try:
            return self.query("*OPC?").startswith("1")
        finally:
            self._sock.settimeout(old)

    def errors(self) -> List[str]:
        out = []
        for _ in range(32):
            err = self.query(":SYSTem:ERRor?")
            if not err or err.startswith("0,") or "No error" in err:
                break
            out.append(err)
        return out

    def try_query(self, command: str) -> Optional[str]:
        """Query that returns None instead of raising when unsupported."""
        try:
            value = self.query(command)
        except (ScpiError, socket.timeout, OSError):
            return None
        return None if self.errors() else value


class CommandMap:
    """Editable SCPI mnemonics, so a VSA revision change is a config change."""

    def __init__(self, data: Dict):
        self.data = data

    @classmethod
    def load(cls, path: Optional[str] = None) -> "CommandMap":
        with open(path or _DEFAULT_MAP, "r") as fh:
            return cls(json.load(fh))

    def get(self, key: str) -> Optional[str]:
        return self.data.get("commands", {}).get(key) or None

    def get_list(self, key: str) -> List[str]:
        value = self.data.get("commands", {}).get(key)
        if not value:
            return []
        return value if isinstance(value, list) else [value]

    def format(self, key: str, **kwargs) -> Optional[str]:
        template = self.get(key)
        return template.format(**kwargs) if template else None


class Vsa89600:
    """High level operations expressed through the command map."""

    def __init__(self, link: VsaScpi, cmds: CommandMap):
        self.link = link
        self.cmds = cmds

    def apply(self, key: str, **kwargs) -> None:
        for template in self.cmds.get_list(key):
            self.link.write(template.format(**kwargs))

    def configure(self, center_hz: float, span_hz: float, points: int) -> None:
        self.apply("set_center", center=center_hz)
        self.apply("set_span", span=span_hz)
        self.apply("set_points", points=points)

    def single_measure(self, timeout: float = 60.0) -> None:
        self.apply("abort")
        self.apply("single")
        self.apply("initiate")
        self.link.opc(timeout)

    def read_evm_percent(self) -> Optional[float]:
        for template in self.cmds.get_list("query_evm"):
            raw = self.link.try_query(template)
            if raw:
                try:
                    return float(raw.split(",")[0])
                except ValueError:
                    continue
        return None

    def read_scalar(self, key: str) -> Optional[float]:
        for template in self.cmds.get_list(key):
            raw = self.link.try_query(template)
            if raw:
                try:
                    return float(raw.split(",")[0])
                except ValueError:
                    continue
        return None

    def fetch_time_iq(self) -> Optional[np.ndarray]:
        """Reads the main time trace as interleaved I/Q -> complex64."""
        binary = self.cmds.get("query_time_trace_binary")
        if binary:
            self.apply("set_binary_format")
            try:
                flat = self.link.query_block(binary, dtype=np.float32)
            except (ScpiError, socket.timeout, OSError):
                flat = None
            if flat is not None and flat.size >= 2:
                return (flat[0::2] + 1j * flat[1::2]).astype(np.complex64)

        ascii_cmd = self.cmds.get("query_time_trace_ascii")
        if ascii_cmd:
            raw = self.link.try_query(ascii_cmd)
            if raw:
                values = np.fromstring(raw, sep=",") if hasattr(np, "fromstring") else None
                if values is None or values.size == 0:
                    values = np.array([float(v) for v in raw.split(",") if v.strip()])
                if values.size >= 2:
                    return (values[0::2] + 1j * values[1::2]).astype(np.complex64)
        return None

    def sample_rate(self) -> Optional[float]:
        fs = self.read_scalar("query_sample_rate")
        if fs:
            return fs
        delta = self.read_scalar("query_x_delta")
        return 1.0 / delta if delta else None

    def save_recording(self, remote_path: str) -> bool:
        cmd = self.cmds.format("save_recording", path=remote_path)
        if not cmd:
            return False
        self.link.write(cmd)
        self.link.opc(120.0)
        return not self.link.errors()


def probe_com() -> int:
    from vsa_com import Vsa89600Com, VsaCom

    print("attaching to the 89600 running on this PC (COM) ...")
    with VsaCom() as link:
        print(f"identity   : {link.idn()}")
        vsa = Vsa89600Com(link)
        freq = link.measurement.Frequency
        print(f"center     : {freq.Center / 1e6:.6f} MHz")
        print(f"span       : {freq.Span / 1e6:.6f} MHz")
        print(f"points     : {freq.Points}")
        print(f"input rate : {freq.InpSampleRate / 1e6:.4f} MHz")

        print("\nsummary table:")
        for key in sorted(k for k in _COM_SCALARS):
            print(f"  {key:24s} -> {vsa.read_scalar(key)}")

        iq = vsa.fetch_time_iq()
        fs = vsa.sample_rate()
        print(f"\nsample rate: {fs}")
        if iq is None:
            print("time trace : not available in this measurement mode")
        else:
            peak = float(np.max(np.abs(iq))) if iq.size else 0.0
            print(f"time trace : {iq.size} samples, peak |x| = {peak:.6g}")
            if peak == 0.0:
                print("             (all zero - the VSA has no input signal)")
    return 0


def probe(host: str, port: int, map_path: Optional[str]) -> int:
    cmds = CommandMap.load(map_path)
    print(f"connecting to {host}:{port} ...")
    with VsaScpi(host, port) as link:
        print(f"*IDN? -> {link.idn()}")
        link.errors()
        vsa = Vsa89600(link, cmds)

        print("\nchecking mapped queries (None = not supported by this VSA):")
        for key in sorted(k for k in cmds.data.get("commands", {}) if k.startswith("query_")):
            for template in cmds.get_list(key):
                t0 = time.time()
                value = link.try_query(template)
                shown = value if value is None or len(value) < 70 else value[:67] + "..."
                print(f"  {key:28s} {template:40s} -> {shown}   ({time.time() - t0:.2f}s)")

        fs = vsa.sample_rate()
        print(f"\nsample rate: {fs}")
        iq = vsa.fetch_time_iq()
        print(f"time trace : {None if iq is None else f'{iq.size} samples'}")
        evm = vsa.read_evm_percent()
        print(f"VSA EVM    : {evm}")

        remaining = link.errors()
        if remaining:
            print("\nerror queue:")
            for err in remaining:
                print(f"  {err}")
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default=os.environ.get("VSA_HOST", "127.0.0.1"))
    p.add_argument("--port", type=int, default=int(os.environ.get("VSA_PORT", DEFAULT_PORT)))
    p.add_argument("--map", help="path to the SCPI command map JSON")
    p.add_argument("--probe", action="store_true", help="connect and report which commands work")
    p.add_argument("--com", action="store_true",
                   help="use the local COM automation interface instead of the SCPI server")
    p.add_argument("--raw", metavar="SCPI", help="send one raw command (query if it ends with ?)")
    args = p.parse_args(argv)

    try:
        if args.com:
            return probe_com()

        if args.raw:
            with VsaScpi(args.host, args.port) as link:
                if args.raw.strip().endswith("?"):
                    print(link.query(args.raw))
                else:
                    link.write(args.raw)
                    for err in link.errors():
                        print(err, file=sys.stderr)
            return 0

        return probe(args.host, args.port, args.map)
    except ScpiError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
