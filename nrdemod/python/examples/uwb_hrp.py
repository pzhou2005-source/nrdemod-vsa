"""IEEE 802.15.4a / 802.15.4z BPRF HRP UWB PHY in pure Python.

Implements the full chain of IEEE Std 802.15.4-2020 clause 15 (non-HPRF) in both
directions so every stage can be inspected and plotted:

    TX: PSDU -> RS(63,55) -> PHR+SECDED -> K=3 rate-1/2 conv. code -> BPM-BPSK
        (scrambler-driven burst hopping + spreading) -> SHR (SYNC+SFD) -> pulse shaping
    RX: pulse matched filter -> preamble correlation / timing -> CFO + phase estimate ->
        SFD search -> BPM-BPSK soft demod (with phase tracking) -> Viterbi ->
        SECDED PHR check -> RS(63,55) decode -> PSDU

STS (802.15.4z SP1/SP2) is skipped on receive; the Python modulator only builds SP0.
HPRF is not supported.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import reedsolo
from scipy import signal

CHIP_RATE_HZ = 499.2e6
PHR_BITS = 19
PHR_SYMBOLS = 21  # PHR bits + 2 convolutional memory bits are sent at the PHR rate
STS_BPRF_CHIPS = 512 + 64 * 512 + 512  # gap + 64x512 active + gap (15.4z, BPRF)

# Ternary preamble codes, Table 15-6 (length 31) and 15-7 (length 127).
_CODES = {
    1: "-0000+0-0+++0+-000+-+++00-+0-00",
    2: "0+0+-0+0+000-++0-+---00+00++000",
    3: "-+0++000-+-++00++0+00-0000-0+0-",
    4: "0000+-00-00-++++0+-+000+0-0++0-",
    5: "-0+-00+++-+000-+0+++0-0+0000-00",
    6: "++00+00---+-0++-000+0+0-+0+0000",
    7: "+0000+-0+0+00+000+0++---0-+00-+",
    8: "0+00-0-0++0000--+00-+0++-++0+00",
    9: "+00+000-0--00--+0+0+00-+-++0+0000++-000+00-00--0-+0+0--0-+++0++000+-0+00-0++-0+++00-+00+0+0-0++-+--+000000+00000-+0000-0-000--+",
    10: "++00+0-+00+00+000000-000-00--000-0+-+0-0+-0-+00000+-00++0-0+00--+00++-+0+-0+0000-0-0-0-++-+0+00+0+000-+0+++000----+++0000+++0--",
    11: "-+-0000+00--00000-0+0+0+-0+00+00+0-00-+++00+000-+0+0-0000+++++-+0+--0+-0++--0-000+0-+00+0+----000-000000-+00+-0++000++-00++-0-0",
    12: "-+0++000000-0+0-+0---+-++00-+0++0+0+0+000-00-00-+00+-++000-+-0-++0-0++++0-00-0++00+0+00++-00+000+-000-0--+0000-0000--0+00000+--",
    13: "+000--0000--++0-++++0-0++0+0-00-+0++00++-0++0+-+0-00+00-0--000-+-00+0000-0++-00000+-0-000000-00-+-++-+000-0+0+0+++-00--00+0+000",
    14: "+000++0-0+0-00+-0-+0-00+0+0000+0+-0000++00+0+++++-+0-0+-0--+0++--000---0+000+0+0-+-000000+-+-0--00++000-00+00++-00--++-00-00000",
    15: "0+-00+0-000-++0000---++000+0+-0-+00-+000--0-00--0--+++-+0-++00+-++0+00000+0-0+++-00+00+000-0000+00--+0++0+0+0-00-0-+-0+0++00000",
    16: "++0000+000+00+--0+-++0-000--00+-0+00++000+++00+0+0-0-+-0-0+00+00+0++----+00++--+0+-0--+000000-0-0000-+0--00+00000+-++000-0-+0+0",
    17: "+--000-0-0000+-00000+000000+--+-++0-0+0+00+-00+++0-++0-00+0-+000++0+++-0--0+0+-0--00-00+000-++0000+0++-+-00+0+0+--00--0-000+00+",
    18: "--0+++0000+++----000+++0+-000+0+00+0+-++-0-0-0-0000+0-+0+-++00+--00+0-0++00-+00000+-0-+0-0+-+0-000--00-000-000000+00+00+-0+00++",
    19: "-0-++00-++000++0-+00+-000000-000----+0+00+-0+000-0--++0-+0--+0+-+++++0000-0+0+-000+00+++-00-0+00+00+0-+0+0+0-00000--00+0000-+-0",
    20: "--+00000+0--0000-0000+--0-000-+000+00-++00+0+00++0-00-0++++0-0++-0-+-000++-+00+-00-00-000+0+0+0++0+-00++-+---0+-0+0-000000++0+-",
    21: "+0+00--00-+++0+0+0-000+-++-+-00-000000-0-+00000-++0-0000+00-+-000--0-00+00-0+-+0++0-++00++0+-00-0+0++0-0++++-0++--0000--000+000",
    22: "0-00-++--00-++00+00-000++00--0-+-+000000-+-0+0+000+0---000--++0+--0-+0-0+-+++++0+00++0000-+0+0000+0+00-0+-0-+00-0+0-0++000+0000",
    23: "000++0+0-+-0-00-0+0+0++0+--00+0000-000+00+00-+++0-0+00000+0++-+00++-0+-+++--0--00-0--000+-00+-0-+0+000++---0000++-000-0+00-+000",
    24: "+0+-0-000++-+00000+00--0+-0000-0-000000+--0-+0+--++00+----++0+00+00+0-0-+-0-0+0+00+++000++00+0-+00--000-0++-+0--+00+000+0000++0",
}

_SFD_SHORT = [0, 1, 0, -1, 1, 0, 0, -1]
_SFD_LONG_4A = _SFD_SHORT + _SFD_SHORT + [
    -1, 0, 0, 1, 0, -1, 0, 1, 0, 1, 0, 0, 0, -1, 0, -1, 0, -1, 0, 0, 1, 0, -1, -1, 0, -1, 1, 0, 0, 0, 0, 1,
    1, 0, 0, -1, -1, -1, 1, -1, 1, 1, 0, 0, 0, 0, 1, 1]
# Table 15-7c (802.15.4z); BPRF allows SFD 0 and 2.
_SFD_4Z = {
    0: _SFD_SHORT,
    1: [-1, -1, 1, -1],
    2: [-1, -1, -1, 1, -1, -1, 1, -1],
    3: [-1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1],
    4: [-1, -1, -1, -1, -1, -1, -1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, -1, -1, -1, 1, 1, -1, -1, -1, 1, -1,
        1, 1, -1, -1],
}

# Table 15-3: mean PRF (MHz) -> (bursts per symbol, hopping bursts, {data rate kb/s: chips per burst})
_PRF_TABLE = {
    3.9: (128, 32, {110: 32, 850: 4, 1700: 2, 6810: 1}),
    15.6: (32, 8, {110: 128, 850: 16, 6810: 2, 27240: 1}),
    62.4: (8, 2, {110: 512, 850: 64, 6810: 8, 27240: 2}),
}
# PHR data-rate field (R1, R0), Table 15-8
_RATE_FIELD = {
    3.9: {(0, 0): 110, (0, 1): 850, (1, 0): 1700, (1, 1): 6810},
    "default": {(0, 0): 110, (0, 1): 850, (1, 0): 6810, (1, 1): 27240},
}
_PREAMBLE_FIELD = {(0, 0): 16, (0, 1): 64, (1, 0): 1024, (1, 1): 4096}


def preamble_code(code_index: int) -> np.ndarray:
    return np.array([{"+": 1, "-": -1, "0": 0}[c] for c in _CODES[code_index]], dtype=np.int8)


@dataclass
class HrpConfig:
    """Non-HPRF HRP configuration (mirrors the relevant lrwpanHRPConfig properties)."""

    mode: str = "BPRF"  # 'BPRF' (802.15.4z) or '802.15.4a'
    code_index: int = 9
    mean_prf: float = 62.4  # data mean PRF in MHz (BPRF is always 62.4)
    data_rate_kbps: int = 6810  # transmitter only; the receiver reads it from the PHR
    phr_data_rate_kbps: int | None = None  # BPRF: 850 or 6810; None = default (TX) / auto (RX)
    sfd_number: int = 0
    preamble_duration: int = 64
    preamble_spreading_factor: int | None = None
    sts_packet_configuration: int = 0
    ranging: bool = False
    samples_per_pulse: int = 4

    @property
    def code(self) -> np.ndarray:
        return preamble_code(self.code_index)

    @property
    def L(self) -> int:
        if self.preamble_spreading_factor:
            return self.preamble_spreading_factor
        return 4 if len(_CODES[self.code_index]) == 127 else 16

    @property
    def sfd(self) -> np.ndarray:
        if self.mode == "802.15.4a":
            seq = _SFD_LONG_4A if self.data_rate_kbps == 110 else _SFD_SHORT
        else:
            seq = _SFD_4Z[self.sfd_number]
        return np.array(seq, dtype=np.int8)

    @property
    def prf_key(self) -> float:
        return 62.4 if self.mode == "BPRF" else float(self.mean_prf)

    @property
    def bursts_per_symbol(self) -> int:
        return _PRF_TABLE[self.prf_key][0]

    @property
    def num_hop_bursts(self) -> int:
        return _PRF_TABLE[self.prf_key][1]

    def chips_per_burst(self, rate_kbps: int) -> int:
        return _PRF_TABLE[self.prf_key][2][rate_kbps]

    def phr_rate_kbps(self, data_rate_kbps: int | None = None) -> int:
        rate = self.data_rate_kbps if data_rate_kbps is None else data_rate_kbps
        if rate == 110:
            return 110
        if self.mode == "BPRF":
            return self.phr_data_rate_kbps or 850
        return 850

    @property
    def sample_rate(self) -> float:
        return CHIP_RATE_HZ * self.samples_per_pulse


# --------------------------------------------------------------------------- coding
def secded_encode(phr13) -> np.ndarray:
    """13 PHR bits -> 19 bits with SECDED parity (15.2.7)."""
    b = np.zeros(19, dtype=np.uint8)
    b[:13] = np.asarray(phr13, dtype=np.uint8) & 1
    par = lambda idx: int(np.sum(b[np.array(idx) - 1]) & 1)  # noqa: E731 (1-based, as in the standard)
    b[18] = par([2, 1, 9, 7, 5, 4, 11, 12])
    b[17] = par([1, 7, 6, 4, 3, 10, 11, 13])
    b[16] = par([2, 9, 8, 4, 3, 10, 11])
    b[15] = par([9, 8, 7, 6, 5, 10, 11])
    b[14] = par([13, 12])
    b[13] = int(np.sum(b) & 1)
    return b


def secded_decode(bits19):
    """Return (13 PHR bits, status) where status is 'ok', 'corrected' or 'error'."""
    b = np.asarray(bits19, dtype=np.uint8) & 1
    if np.array_equal(secded_encode(b[:13]), b):
        return b[:13].copy(), "ok"
    for i in range(19):
        t = b.copy()
        t[i] ^= 1
        if np.array_equal(secded_encode(t[:13]), t):
            return t[:13].copy(), "corrected"
    return b[:13].copy(), "error"


def build_phr(cfg: HrpConfig, psdu_len_bytes: int) -> np.ndarray:
    rate_map = _RATE_FIELD.get(cfg.prf_key if cfg.prf_key == 3.9 else "default")
    inv_rate = {v: k for k, v in rate_map.items()}
    inv_pre = {v: k for k, v in _PREAMBLE_FIELD.items()}
    phr = np.zeros(13, dtype=np.uint8)
    phr[0:2] = inv_rate[cfg.data_rate_kbps]
    phr[2:9] = [(psdu_len_bytes >> (6 - i)) & 1 for i in range(7)]
    phr[9] = int(cfg.ranging)
    phr[11:13] = inv_pre[cfg.preamble_duration]
    return phr


def parse_phr(phr13, cfg: HrpConfig) -> dict:
    phr = [int(x) for x in phr13]
    rate_map = _RATE_FIELD.get(cfg.prf_key if cfg.prf_key == 3.9 else "default")
    return {
        "data_rate_kbps": rate_map[(phr[0], phr[1])],
        "frame_length": int("".join(map(str, phr[2:9])), 2),
        "ranging": phr[9],
        "reserved": phr[10],
        "preamble_duration": _PREAMBLE_FIELD[(phr[11], phr[12])],
    }


_RS_BLOCK = 330  # 55 symbols x 6 bits
_RS_CODEC = None


def _rs():
    global _RS_CODEC
    if _RS_CODEC is None:
        # GF(2^6), primitive 1+x+x^6, roots alpha^1..alpha^8 -> x^8+55x^7+61x^6+...+22 (15.3.3.2)
        _RS_CODEC = reedsolo.RSCodec(8, nsize=63, fcr=1, prim=0x43, generator=2, c_exp=6)
    return _RS_CODEC


def _bits_to_sym6(bits):
    return (np.asarray(bits, dtype=np.int64).reshape(-1, 6) << np.arange(6)).sum(axis=1)


def _sym6_to_bits(sym):
    return ((np.asarray(sym, dtype=np.int64)[:, None] >> np.arange(6)) & 1).astype(np.uint8).ravel()


def rs_encoded_length(n_bits: int) -> int:
    return n_bits + 48 * int(np.ceil(n_bits / _RS_BLOCK))


def rs_encode(bits) -> np.ndarray:
    bits = np.asarray(bits, dtype=np.uint8)
    out = []
    for start in range(0, len(bits), _RS_BLOCK):
        blk = bits[start:start + _RS_BLOCK]
        pad = _RS_BLOCK - len(blk)
        msg = _bits_to_sym6(np.concatenate([np.zeros(pad, np.uint8), blk]))
        cw = np.frombuffer(bytes(_rs().encode(bytearray(msg.tolist()))), dtype=np.uint8)
        out.append(_sym6_to_bits(cw)[pad:])
    return np.concatenate(out) if out else np.zeros(0, np.uint8)


def rs_decode(bits, n_psdu_bits: int):
    """Decode RS-coded PSDU bits. Returns (psdu bits, corrected symbol count, all_blocks_ok)."""
    bits = np.asarray(bits, dtype=np.uint8)
    out, n_corr, ok = [], 0, True
    pos = 0
    for start in range(0, n_psdu_bits, _RS_BLOCK):
        n_msg = min(_RS_BLOCK, n_psdu_bits - start)
        pad = _RS_BLOCK - n_msg
        enc = bits[pos:pos + n_msg + 48]
        pos += n_msg + 48
        cw = _bits_to_sym6(np.concatenate([np.zeros(pad, np.uint8), enc]))
        try:
            msg, _, err_pos = _rs().decode(bytearray(cw.tolist()))
            n_corr += len(err_pos)
            msg = np.frombuffer(bytes(msg), dtype=np.uint8)
        except reedsolo.ReedSolomonError:
            ok = False
            msg = cw[:55]
        out.append(_sym6_to_bits(msg)[pad:])
    return (np.concatenate(out) if out else np.zeros(0, np.uint8)), n_corr, ok


def conv_encode(bits) -> np.ndarray:
    """K=3 rate-1/2 code, generators (2, 5) octal: g0 = u[n-1], g1 = u[n] ^ u[n-2]."""
    u = np.concatenate([[0, 0], np.asarray(bits, dtype=np.uint8)])
    g0 = u[1:-1]
    g1 = u[2:] ^ u[:-2]
    return np.column_stack([g0, g1]).ravel().astype(np.uint8)


def viterbi_k3(soft, terminated: bool = True) -> np.ndarray:
    """Soft Viterbi for conv_encode(); soft > 0 means bit 0. soft has 2 values per input bit."""
    soft = np.asarray(soft, dtype=float).reshape(-1, 2)
    n = len(soft)
    ns = np.arange(4)
    u_in, a = ns >> 1, ns & 1  # next state = (u[n], u[n-1]); state index = 2*u[n-1] + u[n-2]
    pm = np.array([0.0, -np.inf, -np.inf, -np.inf])
    prev = np.zeros((n, 4), dtype=np.int8)
    for k in range(n):
        y0, y1 = soft[k]
        cand = []
        for b in (0, 1):
            s = 2 * a + b
            cand.append(pm[s] + y0 * (1 - 2 * a) + y1 * (1 - 2 * (u_in ^ b)))
        pick = cand[1] > cand[0]
        pm = np.where(pick, cand[1], cand[0])
        prev[k] = 2 * a + pick
    state = 0 if terminated else int(np.argmax(pm))
    out = np.zeros(n, dtype=np.uint8)
    for k in range(n - 1, -1, -1):
        out[k] = state >> 1
        state = prev[k, state]
    return out


def scrambler(code_index: int, n: int) -> np.ndarray:
    """15.3.2 LFSR 1+D^14+D^15 seeded with the first 15 non-zero preamble-code chips (-1 -> 0)."""
    code = preamble_code(code_index)
    seed = (code[code != 0][:15] > 0).astype(np.uint8)
    x = np.zeros(n + 15, dtype=np.uint8)
    x[:15] = seed
    for i in range(15, n + 15):
        x[i] = x[i - 14] ^ x[i - 15]
    return x[15:]


# --------------------------------------------------------------------------- modulation
def _symbol_layout(cfg: HrpConfig, num_sym: int, cpb_phr: int, cpb_data: int):
    """Per-symbol (chip start, PN offset, chips per burst)."""
    bps = cfg.bursts_per_symbol
    k = np.arange(num_sym)
    is_phr = k < PHR_SYMBOLS
    cpb = np.where(is_phr, cpb_phr, cpb_data)
    chip_start = np.where(is_phr, k * bps * cpb_phr,
                          PHR_SYMBOLS * bps * cpb_phr + (k - PHR_SYMBOLS) * bps * cpb_data)
    pn_off = np.where(is_phr, k * cpb_phr, PHR_SYMBOLS * cpb_phr + (k - PHR_SYMBOLS) * cpb_data)
    return chip_start, pn_off, cpb


def _burst_position(pn, off, m):
    return int(np.dot(pn[off:off + m], 1 << np.arange(m))) if m else 0


def bpm_bpsk_modulate(codeword, cfg: HrpConfig, cpb_phr: int, cpb_data: int) -> np.ndarray:
    cw = np.asarray(codeword, dtype=np.uint8).reshape(-1, 2)
    num_sym = len(cw)
    starts, pn_offs, cpbs = _symbol_layout(cfg, num_sym, cpb_phr, cpb_data)
    m = int(np.log2(cfg.num_hop_bursts))
    pn = scrambler(cfg.code_index, int(pn_offs[-1] + cpbs[-1] + m + 64))
    bps = cfg.bursts_per_symbol
    total = int(starts[-1] + bps * cpbs[-1])
    chips = np.zeros(total, dtype=float)
    for k in range(num_sym):
        sys_bit, par_bit = cw[k]
        cpb, off = int(cpbs[k]), int(pn_offs[k])
        pos = _burst_position(pn, off, m) + sys_bit * bps // 2
        spread = 1 - 2 * pn[off:off + cpb].astype(float)
        s = int(starts[k]) + pos * cpb
        chips[s:s + cpb] = (1 - 2 * par_bit) * spread
    return chips


def shr_chips(cfg: HrpConfig) -> np.ndarray:
    spread = np.zeros(cfg.L * len(cfg.code))
    spread[::cfg.L] = cfg.code
    sync = np.tile(spread, cfg.preamble_duration)
    sfd = (cfg.sfd[:, None] * spread[None, :]).ravel()
    return np.concatenate([sync, sfd])


def pulse_filter(spc: int):
    """4th-order Butterworth pulse (3 dB at Fc with Fs = Fc*spc), as in the MATLAB reference."""
    return signal.butter(4, 1.0 / spc)


def pulse_shape(chips, spc: int) -> np.ndarray:
    b, a = pulse_filter(spc)
    imp = np.zeros(len(chips) * spc, dtype=complex)
    imp[::spc] = chips
    return signal.lfilter(b, a, imp)


def pulse_response(spc: int, length: int | None = None) -> np.ndarray:
    length = length or 16 * spc
    b, a = pulse_filter(spc)
    imp = np.zeros(length)
    imp[0] = 1.0
    return signal.lfilter(b, a, imp)


def generate(psdu_bits, cfg: HrpConfig) -> dict:
    """Build an HRP packet (SP0 only). Returns waveform plus every intermediate stage."""
    if cfg.mode == "BPRF" and cfg.sts_packet_configuration != 0:
        raise NotImplementedError("Python modulator builds SP0 packets only (no STS)")
    psdu = np.asarray(psdu_bits, dtype=np.uint8)
    if len(psdu) % 8:
        raise ValueError("PSDU length must be a multiple of 8 bits")
    rs_bits = rs_encode(psdu)
    phr = secded_encode(build_phr(cfg, len(psdu) // 8))
    cw = conv_encode(np.concatenate([phr, rs_bits, [0, 0]]))
    cpb_phr = cfg.chips_per_burst(cfg.phr_rate_kbps())
    cpb_data = cfg.chips_per_burst(cfg.data_rate_kbps)
    data = bpm_bpsk_modulate(cw, cfg, cpb_phr, cpb_data)
    shr = shr_chips(cfg)
    chips = np.concatenate([shr, data])
    return {
        "psdu_bits": psdu, "rs_psdu_bits": rs_bits, "phr_bits": phr, "conv_codeword": cw,
        "shr_chips": shr, "data_chips": data, "chips": chips,
        "waveform": pulse_shape(chips, cfg.samples_per_pulse), "sample_rate": cfg.sample_rate,
    }


# --------------------------------------------------------------------------- demodulation
@dataclass
class HrpResult:
    ok: bool = False
    status: str = ""
    sps: int = 4
    symbol_period: int = 0  # preamble symbol period in samples
    corr: np.ndarray | None = None  # |preamble correlation| (samples)
    timing: int = 0  # sample index of the first detected preamble symbol
    symbol_corr: np.ndarray | None = None  # complex preamble-symbol correlations (after CFO)
    preamble_symbols: int = 0
    cfo_hz: float = 0.0
    phase_rad: float = 0.0
    sfd_symbol: int = -1
    sfd_metric: np.ndarray | None = None
    sts_skipped_chips: int = 0
    phr_start: int = 0  # sample index of the first PHR chip
    data_chips: np.ndarray | None = None  # normalised, phase-corrected chip samples (PHR+PSDU)
    z: np.ndarray | None = None  # (num_sym, 2) burst correlations at the two BPM positions
    burst_pos: np.ndarray | None = None
    phase_track: np.ndarray | None = None
    soft: np.ndarray | None = None  # (num_sym, 2) soft bits, >0 means 0
    hard: np.ndarray | None = None
    phr_cpb: int = 0
    data_cpb: int = 0
    phr_bits: np.ndarray | None = None
    phr_status: str = ""
    phr: dict = field(default_factory=dict)
    decoded: np.ndarray | None = None  # Viterbi output (PHR + RS PSDU + tail)
    rs_psdu_bits: np.ndarray | None = None
    rs_corrected: int = 0
    rs_ok: bool = False
    psdu_bits: np.ndarray | None = None
    psdu_bytes: bytes = b""
    evm_db: float = float("nan")


def _matched_filter(iq, spc):
    p = pulse_response(spc)
    return signal.correlate(iq, p, mode="full", method="fft")[len(p) - 1:] / np.sum(p ** 2)


def _preamble_corr(mf, cfg: HrpConfig, spc: int):
    code = cfg.code.astype(float)
    tpl = np.zeros(len(code) * cfg.L * spc)
    tpl[::cfg.L * spc] = code
    r = signal.correlate(mf, tpl, mode="full", method="fft")[len(tpl) - 1:]
    return r / np.count_nonzero(code)


def _symbol_corrs(r, start, period):
    n = (len(r) - start) // period
    return r[start + np.arange(n) * period]


def _find_sfd(b, sfd, first, last):
    """Metric over candidate SFD starts using real symbol correlations b (preamble ~ +1)."""
    zero = sfd == 0
    n = len(sfd)
    metric = np.full(len(b), -np.inf)
    for k in range(first, min(last, len(b) - n) + 1):
        seg = b[k:k + n]
        metric[k] = np.dot(seg, sfd) - np.sum(np.abs(seg[zero]))
    return int(np.argmax(metric)), metric


def demodulate(iq, cfg: HrpConfig, fs: float | None = None, track_alpha: float = 0.05) -> HrpResult:
    iq = np.asarray(iq, dtype=complex)
    fs = fs or cfg.sample_rate
    spc = int(round(fs / CHIP_RATE_HZ))
    if abs(spc * CHIP_RATE_HZ - fs) > 1e3:
        raise ValueError("sample rate must be an integer multiple of 499.2 MHz")
    res = HrpResult(sps=spc)
    period = len(cfg.code) * cfg.L * spc
    res.symbol_period = period

    # 1) acquisition: correlate with one preamble symbol, align to the strongest periodic phase
    mf = _matched_filter(iq, spc)
    r = _preamble_corr(mf, cfg, spc)
    peak = int(np.argmax(np.abs(r)))
    phase0 = peak % period
    s = _symbol_corrs(r, phase0, period)
    thr = 0.5 * np.abs(r[peak])
    on = np.flatnonzero(np.abs(s) > thr)
    if len(on) < 4:
        res.status = "no preamble"
        return res
    k0 = int(on[0])
    run = k0
    while run + 1 < len(s) and np.abs(s[run + 1]) > thr:
        run += 1
    pre = np.arange(k0, run + 1)

    # 2) CFO from symbol-to-symbol rotation of the preamble correlations, then redo on corrected IQ
    d = np.sum(s[pre[1:]] * np.conj(s[pre[:-1]]))
    res.cfo_hz = float(np.angle(d) / (2 * np.pi * period / fs))
    n = np.arange(len(iq))
    iq_c = iq * np.exp(-2j * np.pi * res.cfo_hz * n / fs)
    mf = _matched_filter(iq_c, spc)
    r = _preamble_corr(mf, cfg, spc)
    # refine sample timing around the earlier phase using energy over the detected preamble
    cands = [(phase0 + dlt) % period for dlt in range(-spc, spc + 1)]
    energy = [np.sum(np.abs(_symbol_corrs(r, c, period)[pre]) ** 2) for c in cands]
    phase0 = cands[int(np.argmax(energy))]
    s = _symbol_corrs(r, phase0, period)
    res.corr = np.abs(r)
    res.timing = phase0 + k0 * period
    res.symbol_corr = s

    # 3) phase reference from the leading preamble symbols, SFD search on real projections
    n_ref = max(2, min(16, len(pre) // 2))
    h = np.mean(s[k0:k0 + n_ref])
    res.phase_rad = float(np.angle(h))
    b = np.real(s * np.conj(h)) / np.abs(h) ** 2
    sfd = cfg.sfd.astype(float)
    k_sfd, metric = _find_sfd(b, sfd, k0 + 1, run + 1)
    res.sfd_symbol, res.sfd_metric = k_sfd, metric
    res.preamble_symbols = k_sfd - k0
    # refine the phase with all preamble symbols before the SFD
    h = np.mean(s[k0:k_sfd])
    res.phase_rad = float(np.angle(h))

    # 4) chip-level samples after SHR (and STS for SP1)
    start = phase0 + (k_sfd + len(sfd)) * period
    if cfg.mode == "BPRF" and cfg.sts_packet_configuration == 1:
        res.sts_skipped_chips = STS_BPRF_CHIPS
        start += STS_BPRF_CHIPS * spc
    res.phr_start = start
    chip_amp = np.abs(h)
    chips = mf[start::spc] * np.exp(-1j * res.phase_rad) / chip_amp
    res.data_chips = chips

    # 5) PHR: try candidate PHR rates, keep the one whose SECDED check passes
    if cfg.mode == "BPRF":
        phr_rates = [cfg.phr_data_rate_kbps] if cfg.phr_data_rate_kbps else [850, 6810]
    else:
        phr_rates = [110] if len(sfd) == 64 else [850]
    best = None
    for rate in phr_rates:
        cpb = cfg.chips_per_burst(rate)
        z, pos, ph = _demod_symbols(chips, cfg, PHR_SYMBOLS, cpb, cpb, track_alpha)
        if z is None:
            continue
        soft = _soft_bits(z, ph)
        u = viterbi_k3(soft, terminated=False)
        phr13, st = secded_decode(u[:PHR_BITS])
        if best is None or st != "error":
            best = (rate, cpb, phr13, st)
        if st != "error":
            break
    if best is None:
        res.status = "capture too short for PHR"
        return res
    _, cpb_phr, phr13, st = best
    res.phr_cpb, res.phr_bits, res.phr_status = cpb_phr, phr13, st
    if st == "error":
        res.status = "PHR SECDED error"
        return res
    res.phr = parse_phr(phr13, cfg)
    try:
        cpb_data = cfg.chips_per_burst(res.phr["data_rate_kbps"])
    except KeyError:
        res.status = f"unsupported data rate {res.phr['data_rate_kbps']} kb/s"
        return res
    res.data_cpb = cpb_data

    # 6) full PHR + PSDU demod, Viterbi, RS
    n_psdu = 8 * res.phr["frame_length"]
    n_rs = rs_encoded_length(n_psdu)
    num_sym = PHR_BITS + n_rs + 2
    z, pos, ph = _demod_symbols(chips, cfg, num_sym, cpb_phr, cpb_data, track_alpha)
    if z is None:
        res.status = "capture too short for PSDU"
        return res
    res.z, res.burst_pos, res.phase_track = z, pos, ph
    res.soft = _soft_bits(z, ph)
    res.hard = (res.soft < 0).astype(np.uint8)
    u = viterbi_k3(res.soft, terminated=True)
    res.decoded = u
    res.rs_psdu_bits = u[PHR_BITS:PHR_BITS + n_rs]
    res.psdu_bits, res.rs_corrected, res.rs_ok = rs_decode(res.rs_psdu_bits, n_psdu)
    res.psdu_bytes = np.packbits(res.psdu_bits, bitorder="little").tobytes()
    sel = np.where(res.hard[:, 0] == 0, z[:, 0], z[:, 1]) * np.exp(-1j * ph)
    ideal = np.sign(sel.real)
    res.evm_db = float(10 * np.log10(np.mean(np.abs(sel - ideal) ** 2) + 1e-30))
    res.ok = res.rs_ok and res.phr_status != "error"
    res.status = "ok" if res.ok else "RS decode failure"
    return res


def _demod_symbols(chips, cfg: HrpConfig, num_sym, cpb_phr, cpb_data, alpha):
    """Coherent burst correlations at both BPM halves, with decision-directed phase tracking."""
    starts, pn_offs, cpbs = _symbol_layout(cfg, num_sym, cpb_phr, cpb_data)
    bps = cfg.bursts_per_symbol
    if starts[-1] + bps * cpbs[-1] > len(chips):
        return None, None, None
    m = int(np.log2(cfg.num_hop_bursts))
    pn = scrambler(cfg.code_index, int(pn_offs[-1] + cpbs[-1] + m + 64))
    z = np.zeros((num_sym, 2), dtype=complex)
    pos = np.zeros(num_sym, dtype=int)
    ph = np.zeros(num_sym)
    theta = 0.0
    for k in range(num_sym):
        cpb, off = int(cpbs[k]), int(pn_offs[k])
        pos[k] = _burst_position(pn, off, m)
        spread = 1 - 2 * pn[off:off + cpb].astype(float)
        for half in (0, 1):
            s = int(starts[k]) + (pos[k] + half * bps // 2) * cpb
            z[k, half] = np.dot(chips[s:s + cpb], spread) / cpb
        ph[k] = theta
        sel = z[k, int(np.abs(z[k, 1]) > np.abs(z[k, 0]))] * np.exp(-1j * theta)
        theta += alpha * np.angle(sel * np.sign(sel.real or 1.0))
    return z, pos, ph


def _soft_bits(z, ph):
    zr = (z * np.exp(-1j * ph)[:, None]).real
    sys_soft = np.abs(zr[:, 0]) - np.abs(zr[:, 1])
    par_soft = np.where(sys_soft >= 0, zr[:, 0], zr[:, 1])
    return np.column_stack([sys_soft, par_soft])


# --------------------------------------------------------------------------- plotting
def plot_stages(res: HrpResult, iq=None, fs: float | None = None, ref_bits=None, title: str = ""):
    """One figure per receiver stage. Returns the list of matplotlib figures."""
    import matplotlib.pyplot as plt

    figs = []
    fs = fs or CHIP_RATE_HZ * res.sps

    fig, ax = plt.subplots(2, 1, figsize=(11, 6))
    if iq is not None:
        t = np.arange(len(iq)) / fs * 1e6
        ax[0].plot(t, np.abs(iq), lw=0.5)
        for x, lbl in [(res.timing, "preamble"), (res.timing + res.preamble_symbols * res.symbol_period, "SFD"),
                       (res.phr_start, "PHR")]:
            ax[0].axvline(x / fs * 1e6, color="r", ls="--", lw=0.8)
            ax[0].text(x / fs * 1e6, ax[0].get_ylim()[1] * 0.9, lbl, color="r")
        ax[0].set(xlabel="time (us)", ylabel="|x|", title=f"{title} captured envelope and field boundaries")
    if res.corr is not None:
        ax[1].plot(np.arange(len(res.corr)) / fs * 1e6, res.corr, lw=0.5)
        ax[1].set(xlabel="time (us)", ylabel="|r|", title="Preamble-symbol correlation")
    fig.tight_layout()
    figs.append(fig)

    if res.symbol_corr is not None:
        s = res.symbol_corr
        fig, ax = plt.subplots(1, 3, figsize=(14, 4))
        ax[0].plot(np.abs(s), ".-")
        ax[0].axvline(res.sfd_symbol, color="r", ls="--")
        ax[0].set(xlabel="symbol", ylabel="|corr|", title="Per-symbol preamble correlation (SFD marked)")
        ax[1].plot(np.unwrap(np.angle(s[np.abs(s) > 0.5 * np.abs(s).max()])), ".")
        ax[1].set(xlabel="detected symbol", ylabel="rad", title=f"Phase after CFO corr. (CFO {res.cfo_hz/1e3:.1f} kHz)")
        m = res.sfd_metric
        ax[2].plot(np.where(np.isfinite(m), m, np.nan), ".-")
        ax[2].axvline(res.sfd_symbol, color="r", ls="--")
        ax[2].set(xlabel="candidate SFD start", ylabel="metric", title="SFD search")
        fig.tight_layout()
        figs.append(fig)

    if res.z is not None:
        zr = res.z * np.exp(-1j * res.phase_track)[:, None]
        fig, ax = plt.subplots(2, 2, figsize=(12, 8))
        ax[0, 0].plot(zr[:, 0].real, zr[:, 0].imag, ".", ms=3, label="half 0")
        ax[0, 0].plot(zr[:, 1].real, zr[:, 1].imag, ".", ms=3, label="half 1")
        ax[0, 0].set(title=f"BPM-BPSK burst correlations (EVM {res.evm_db:.1f} dB)", xlabel="I", ylabel="Q")
        ax[0, 0].legend()
        ax[0, 0].axis("equal")
        ax[0, 1].plot(np.abs(res.z[:, 0]), lw=0.7, label="|z| half 0")
        ax[0, 1].plot(np.abs(res.z[:, 1]), lw=0.7, label="|z| half 1")
        ax[0, 1].axvline(PHR_SYMBOLS - 0.5, color="k", ls=":")
        ax[0, 1].set(xlabel="symbol", title="Burst energy per BPM half (PHR | PSDU)")
        ax[0, 1].legend()
        ax[1, 0].plot(res.soft[:, 0], ".", ms=3, label="systematic")
        ax[1, 0].plot(res.soft[:, 1], ".", ms=3, label="parity")
        ax[1, 0].set(xlabel="symbol", title="Soft bits into Viterbi (>0 = 0)")
        ax[1, 0].legend()
        ax[1, 1].plot(np.degrees(res.phase_track))
        ax[1, 1].set(xlabel="symbol", ylabel="deg", title="Decision-directed phase track")
        fig.tight_layout()
        figs.append(fig)

        n = min(len(res.data_chips), 4 * 512)
        fig, ax = plt.subplots(1, 1, figsize=(12, 3))
        ax.stem(res.data_chips[:n].real, markerfmt=" ", basefmt=" ")
        ax.set(xlabel="chip", title="First PHR chips (phase-corrected, normalised)")
        fig.tight_layout()
        figs.append(fig)

    if res.psdu_bits is not None:
        fig, ax = plt.subplots(1, 2, figsize=(12, 4))
        nb = len(res.psdu_bits)
        w = 64
        rows = int(np.ceil(nb / w))
        img = np.zeros(rows * w)
        img[:nb] = res.psdu_bits
        ax[0].imshow(img.reshape(rows, w), aspect="auto", cmap="gray_r", interpolation="nearest")
        ax[0].set(title=f"Decoded PSDU bits ({nb}), RS corrected {res.rs_corrected}", xlabel="bit", ylabel="row")
        if ref_bits is not None and len(ref_bits) == nb:
            err = np.flatnonzero(np.asarray(ref_bits) != res.psdu_bits)
            ax[1].plot(err, np.ones_like(err), "rx")
            ax[1].set(title=f"Bit errors vs reference: {len(err)}", xlabel="bit", xlim=(0, nb))
        else:
            ax[1].text(0.1, 0.5, "\n".join(f"{k}: {v}" for k, v in res.phr.items()), fontsize=11)
            ax[1].set_axis_off()
            ax[1].set_title(f"PHR ({res.phr_status})")
        fig.tight_layout()
        figs.append(fig)
    return figs
