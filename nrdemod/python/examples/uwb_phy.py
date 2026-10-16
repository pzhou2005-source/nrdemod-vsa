"""Reference HRP-UWB data-path helpers used by ``uwb_demod.ipynb``.

This module keeps configurable waveform and FEC assumptions visible to
notebook users, independently of the native packet decoder.
"""

from __future__ import annotations

import numpy as np


def bits_to_bytes_lsb(bits: np.ndarray) -> bytes:
    bits = np.asarray(bits, dtype=np.uint8).ravel()
    return bytes(
        sum(int(bits[i + j]) << j for j in range(8))
        for i in range(0, len(bits) - 7, 8)
    )


def bytes_to_bits_lsb(data: bytes) -> np.ndarray:
    return np.asarray(
        [(byte >> bit) & 1 for byte in data for bit in range(8)], dtype=np.uint8
    )


def descramble(bits: np.ndarray, seed: int = 0x1FF) -> np.ndarray:
    """IEEE 802.15.4-style x^9+x^5+1 self-synchronising scrambler."""
    state = int(seed) & 0x1FF
    out = np.empty(len(bits), dtype=np.uint8)
    for i, bit in enumerate(np.asarray(bits, dtype=np.uint8)):
        feedback = ((state >> 8) ^ (state >> 4)) & 1
        out[i] = int(bit) ^ (state & 1)
        state = ((state << 1) | feedback) & 0x1FF
    return out


def viterbi_k7_rate_half(
    llr: np.ndarray, generators: tuple[int, int] = (0x6D, 0x4F)
) -> tuple[np.ndarray, float]:
    """Decode terminated K=7 rate-1/2 bits from two soft values per bit."""
    pairs = np.asarray(llr, dtype=float).reshape(-1, 2)
    inf = 1e30
    metric = np.full(64, inf)
    metric[0] = 0.0
    previous = np.full((len(pairs), 64), -1, dtype=np.int16)
    decisions = np.zeros((len(pairs), 64), dtype=np.uint8)
    for time, pair in enumerate(pairs):
        new = np.full(64, inf)
        for state in range(64):
            if metric[state] >= inf:
                continue
            for bit in (0, 1):
                register = ((state << 1) | bit) & 0x7F
                next_state = register & 0x3F
                expected = [
                    (register & generators[index]).bit_count() & 1
                    for index in range(2)
                ]
                branch = sum(
                    (1.0 if expected[index] == 0 else -1.0) * pair[index]
                    for index in range(2)
                )
                candidate = metric[state] - branch
                if candidate < new[next_state]:
                    new[next_state] = candidate
                    previous[time, next_state] = state
                    decisions[time, next_state] = bit
        metric = new
    state = 0 if metric[0] < inf else int(np.argmin(metric))
    output = np.zeros(len(pairs), dtype=np.uint8)
    for time in range(len(pairs) - 1, -1, -1):
        output[time] = decisions[time, state]
        state = max(0, int(previous[time, state]))
    return output, float(metric[0] if metric[0] < inf else metric.min())


def crc16_ccitt_lsb(bits: np.ndarray, polynomial: int = 0x1021) -> int:
    value = 0
    for bit in np.asarray(bits, dtype=np.uint8):
        value ^= int(bit) << 15
        value = (
            ((value << 1) ^ polynomial) & 0xFFFF
            if value & 0x8000
            else (value << 1) & 0xFFFF
        )
    return value


def extract_bpm_bpsk(
    iq: np.ndarray,
    start: int,
    symbol_samples: int,
    count: int,
    polarity: int = 1,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return hard bits, soft values, slot energies, and rotated metrics."""
    half = symbol_samples // 2
    if half < 2 or start + count * 2 * half > len(iq):
        raise ValueError("capture does not contain the requested BPM-BPSK symbols")
    metrics = []
    slot_energy = []
    for symbol in range(count):
        base = start + symbol * 2 * half
        slots = [iq[base + index * half : base + (index + 1) * half] for index in (0, 1)]
        coherent = np.asarray([np.sum(slot) for slot in slots])
        energy = np.asarray([np.sum(np.abs(slot) ** 2) for slot in slots])
        metrics.append(coherent[int(np.argmax(energy))] * polarity)
        slot_energy.append(energy)
    metrics = np.asarray(metrics)
    reference = np.angle(metrics[int(np.argmax(np.abs(metrics)))])
    rotated = metrics * np.exp(-1j * reference)
    soft = np.real(rotated) / np.maximum(np.abs(rotated), 1e-12)
    return (soft < 0).astype(np.uint8), soft, np.asarray(slot_energy), rotated
