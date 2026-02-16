#!/usr/bin/env python3
"""
Read ADS1298 sample output from samples.txt and plot channel data.
- First 3 bytes per line = header (ignored); samples.txt has 27 hex bytes per line.
- Then 8 channels × 3 bytes each, MSB first, two's complement.
- LSB = 5 µV => voltage (V) = raw * 5e-6
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# Config: 3 header + 8*3 channel bytes = 27 bytes per line (file format)
HEADER_BYTES = 3
CHANNELS = 8
BYTES_PER_CHANNEL = 3
LSB_UV = 5  # µV per LSB
LSB_V = LSB_UV * 1e-6

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_INPUT = SCRIPT_DIR / "samples.txt"


def parse_24bit_twos_complement(b0: int, b1: int, b2: int) -> int:
    """Three bytes MSB first -> signed 24-bit (two's complement). Use Python ints to avoid numpy uint8 overflow."""
    b0, b1, b2 = int(b0) & 0xFF, int(b1) & 0xFF, int(b2) & 0xFF
    raw = (b0 << 16) | (b1 << 8) | b2
    if raw >= 0x800000:  # negative in 24-bit two's complement
        raw -= 0x1000000
    return raw


def parse_line(line: str) -> np.ndarray:
    """Parse one line: skip header bytes, then 8 channels × 3 bytes -> voltages (V)."""
    parts = line.strip().split()
    need = HEADER_BYTES + CHANNELS * BYTES_PER_CHANNEL  # 27 for 3-byte header
    if len(parts) < need:
        return None
    try:
        bytes_arr = np.array([int(p, 16) for p in parts[:need]], dtype=np.uint8)
    except ValueError:
        return None
    # Skip header
    data = bytes_arr[HEADER_BYTES : HEADER_BYTES + CHANNELS * BYTES_PER_CHANNEL]
    voltages = np.zeros(CHANNELS)
    for ch in range(CHANNELS):
        i = ch * BYTES_PER_CHANNEL
        code = parse_24bit_twos_complement(data[i], data[i + 1], data[i + 2])
        voltages[ch] = code * LSB_V
    return voltages


def load_samples(path: Path) -> np.ndarray:
    """Load all lines into (N, 8) array of channel voltages in volts."""
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            v = parse_line(line)
            if v is not None:
                rows.append(v)
    return np.array(rows) if rows else np.empty((0, CHANNELS))


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Plot ADS1298 channel data from samples.txt")
    parser.add_argument("file", nargs="?", default=str(DEFAULT_INPUT), help="Input samples file (hex bytes per line)")
    parser.add_argument("--uv", action="store_true", help="Plot in µV instead of V")
    args = parser.parse_args()
    path = Path(args.file)
    if not path.is_file():
        print(f"Error: file not found: {path}")
        return 1
    data = load_samples(path)
    if data.size == 0:
        print("No valid samples found.")
        return 1
    n_samples = data.shape[0]
    t = np.arange(n_samples)  # sample index (no time base; use index as x)
    if args.uv:
        data_plot = data * 1e6  # V -> µV
        ylabel = "Voltage (µV)"
    else:
        data_plot = data
        ylabel = "Voltage (V)"
    fig, axes = plt.subplots(CHANNELS, 1, sharex=True, figsize=(10, 2 * CHANNELS))
    if CHANNELS == 1:
        axes = [axes]
    for ch in range(CHANNELS):
        axes[ch].plot(t, data_plot[:, ch], linewidth=0.8)
        axes[ch].set_ylabel(f"Ch{ch+1}")
        axes[ch].grid(True, alpha=0.3)
    axes[-1].set_xlabel("Sample index")
    axes[0].set_ylabel(axes[0].get_ylabel() + f"\n({ylabel})")
    fig.suptitle(f"ADS1298 — {n_samples} samples, LSB = {LSB_UV} µV")
    plt.tight_layout()
    plt.show()
    return 0


if __name__ == "__main__":
    exit(main())
