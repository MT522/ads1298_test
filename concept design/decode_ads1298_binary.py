#!/usr/bin/env python3
"""
Decode and plot ADS1298 data from RAW BINARY capture (ESP32 UART output).

The ESP32 sends raw 27-byte packets (no hex, no line breaks):
  - 3 status bytes (first byte high nibble 0xC = valid packet)
  - 8 channels × 3 bytes each, MSB first, 24-bit two's complement
  - LSB = VREF / (2^23 - 1) with VREF = 2.4 V

Usage:
  1. Capture serial at 115200 baud to a file (raw binary), e.g. capture.bin
  2. python decode_ads1298_binary.py capture.bin
  3. Or: python decode_ads1298_binary.py capture.bin -o samples.txt  # also export decoded

Why you see garbled text: Serial terminal was set to wrong baud rate or text mode.
  Set terminal to 115200 baud to read log messages; use raw binary capture for data.
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# Match main.cpp: ADS1298_VREF_V, ADS1298_POSITIVE_FS_CODE
VREF_V = 2.4
POSITIVE_FS_CODE = (1 << 23) - 1  # 8388607
LSB_V = VREF_V / POSITIVE_FS_CODE

PACKET_SIZE = 27
HEADER_BYTES = 3
CHANNELS = 8
BYTES_PER_CHANNEL = 3
VALID_STATUS_NIBBLE = 0xC0  # high nibble of first byte for valid packet


def parse_24bit_twos_complement(b0: int, b1: int, b2: int) -> int:
    """Three bytes MSB first -> signed 24-bit."""
    b0, b1, b2 = int(b0) & 0xFF, int(b1) & 0xFF, int(b2) & 0xFF
    raw = (b0 << 16) | (b1 << 8) | b2
    if raw >= 0x800000:
        raw -= 0x1000000
    return raw


def decode_packet(packet: bytes) -> np.ndarray | None:
    """Decode one 27-byte packet to 8 channel voltages (V). Returns None if invalid."""
    if len(packet) < PACKET_SIZE:
        return None
    if (packet[0] & 0xF0) != VALID_STATUS_NIBBLE:
        return None
    data = packet[HEADER_BYTES : HEADER_BYTES + CHANNELS * BYTES_PER_CHANNEL]
    voltages = np.zeros(CHANNELS)
    for ch in range(CHANNELS):
        i = ch * BYTES_PER_CHANNEL
        code = parse_24bit_twos_complement(data[i], data[i + 1], data[i + 2])
        voltages[ch] = code * LSB_V
    return voltages


def load_raw_binary(path: Path, sync: bool = True) -> np.ndarray:
    """Load raw binary file; split into 27-byte packets; return (N, 8) voltages.
    If sync=True, search for 0xCx header to align (handles mixed log + binary capture)."""
    with open(path, "rb") as f:
        raw = f.read()
    rows = []
    i = 0
    if sync:
        # Align to packet boundary: find first byte with high nibble 0xC
        while i < len(raw) and (raw[i] & 0xF0) != VALID_STATUS_NIBBLE:
            i += 1
    while i + PACKET_SIZE <= len(raw):
        packet = bytes(raw[i : i + PACKET_SIZE])
        v = decode_packet(packet)
        if v is not None:
            rows.append(v)
            i += PACKET_SIZE
        else:
            if sync:
                # Resync: step by 1 until next 0xCx
                i += 1
                while i < len(raw) and (raw[i] & 0xF0) != VALID_STATUS_NIBBLE:
                    i += 1
            else:
                i += PACKET_SIZE
    return np.array(rows) if rows else np.empty((0, CHANNELS))


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Decode and plot ADS1298 raw binary capture (27-byte packets)"
    )
    parser.add_argument("file", help="Raw binary capture file (e.g. capture.bin)")
    parser.add_argument("-o", "--output", help="Save decoded samples to text file (hex bytes per line)")
    parser.add_argument("--uv", action="store_true", help="Plot in µV instead of V")
    args = parser.parse_args()
    path = Path(args.file)
    if not path.is_file():
        print(f"Error: file not found: {path}")
        return 1
    data = load_raw_binary(path)
    if data.size == 0:
        print("No valid 27-byte packets found. Check baud rate (115200) and that file is raw binary.")
        return 1
    n_samples = data.shape[0]
    print(f"Decoded {n_samples} samples from {path}")

    if args.output:
        # Export as hex lines for compatibility with original plot_ads1298_samples.py format
        # We don't have original bytes here, so export as "index ch1 ch2 ... ch8" in scientific
        out_path = Path(args.output)
        with open(out_path, "w", encoding="utf-8") as f:
            for i in range(n_samples):
                line = " ".join(f"{data[i, ch]:.6e}" for ch in range(CHANNELS))
                f.write(line + "\n")
        print(f"Saved decoded voltages to {out_path}")

    t = np.arange(n_samples)
    if args.uv:
        data_plot = data * 1e6
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
    fig.suptitle(f"ADS1298 — {n_samples} samples (raw binary), VREF=2.4V")
    plt.tight_layout()
    plt.show()
    return 0


if __name__ == "__main__":
    exit(main())
