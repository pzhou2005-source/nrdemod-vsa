"""Compare uncorrected sample-clock and carrier-frequency errors on QPSK."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import resample_poly


def make_constellations(clock_error_ppm=200.0, carrier_error_hz=800.0):
    """Return ideal QPSK and two independent impairments at 1 Msymbol/s.

    Positive clock error means the receiver samples faster than nominal.
    Timing starts aligned and then drifts; no timing or carrier recovery runs.
    """
    symbol_count = 2000
    symbol_rate = 1e6
    samples_per_symbol = 32
    guard_symbols = 64
    generator = np.random.default_rng(42)
    bits = generator.integers(0, 2, (symbol_count + 2 * guard_symbols, 2))
    symbols = ((2 * bits[:, 0] - 1) + 1j * (2 * bits[:, 1] - 1)) / np.sqrt(2)
    waveform = resample_poly(symbols, samples_per_symbol, 1)
    nominal_positions = (guard_symbols + np.arange(symbol_count)) * samples_per_symbol
    clock_ratio = 1 + clock_error_ppm * 1e-6
    if clock_ratio <= 0:
        raise ValueError("Receiver sample rate must be positive.")
    clock_positions = (
        guard_symbols * samples_per_symbol
        + np.arange(symbol_count) * samples_per_symbol / clock_ratio
    )
    if clock_positions[-1] > len(waveform) - 1:
        raise ValueError("Clock error exceeds the available waveform duration.")
    sample_positions = np.arange(len(waveform))
    clock_samples = np.interp(clock_positions, sample_positions, waveform)
    ideal = symbols[guard_symbols:guard_symbols + symbol_count]
    time_seconds = np.arange(symbol_count) / symbol_rate
    carrier_samples = ideal * np.exp(2j * np.pi * carrier_error_hz * time_seconds)
    reference_samples = waveform[nominal_positions]
    clock_samples /= np.mean(np.abs(reference_samples))
    return ideal, clock_samples, carrier_samples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clock-ppm", type=float, default=200.0)
    parser.add_argument("--carrier-hz", type=float, default=800.0)
    parser.add_argument("--output", type=Path, default=Path(__file__).with_suffix(".png"))
    parser.add_argument("--show", action="store_true", help="Also open the figure window")
    args = parser.parse_args()
    ideal, clock_samples, carrier_samples = make_constellations(args.clock_ppm, args.carrier_hz)
    figure, axes = plt.subplots(1, 2, figsize=(11, 5), layout="constrained")
    elapsed_ms = np.arange(len(ideal)) / 1000
    titles = (
        f"Sample-clock error: {args.clock_ppm:g} ppm\nCarrier-frequency error: 0 Hz",
        f"Carrier-frequency error: {args.carrier_hz:g} Hz\nSample-clock error: 0 ppm",
    )
    for axis, samples, title in zip(axes, (clock_samples, carrier_samples), titles):
        points = axis.scatter(
            samples.real, samples.imag, c=elapsed_ms, cmap="viridis", s=8, alpha=0.65
        )
        axis.scatter(
            ideal[:0].real, ideal[:0].imag, marker="x", color="black", label="Ideal QPSK"
        )
        references = np.unique(ideal)
        axis.scatter(references.real, references.imag, marker="x", color="black", s=65)
        axis.set(title=title, xlabel="In-phase", ylabel="Quadrature", xlim=(-1.6, 1.6), ylim=(-1.6, 1.6))
        axis.set_aspect("equal", adjustable="box")
        axis.grid(alpha=0.25)
        axis.legend(loc="upper right")
    figure.colorbar(points, ax=list(axes), label="Time (ms)", shrink=0.75)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=160)
    print(f"Saved {args.output.resolve()}")
    if args.show:
        plt.show()
    plt.close(figure)


if __name__ == "__main__":
    main()