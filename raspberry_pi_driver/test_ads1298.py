#!/usr/bin/env python3
"""
Test script for ADS1298 on Raspberry Pi 4.
Usage:
  sudo python3 test_ads1298.py              # basic: read 5 samples, print voltages
  sudo python3 test_ads1298.py --continuous --duration 5  # sample 5 seconds
  sudo python3 test_ads1298.py --continuous --plot       # sample 5 s and plot
"""

import argparse
import sys
import time

try:
    from ads1298_driver import ADS1298Driver, packet_to_voltages
except ImportError:
    print("Run from raspberry_pi_driver directory or add it to PYTHONPATH.")
    sys.exit(1)


def main():
    p = argparse.ArgumentParser(description="Test ADS1298 on Raspberry Pi")
    p.add_argument("--count", type=int, default=5, help="Number of samples for basic test")
    p.add_argument("--continuous", action="store_true", help="Run continuous sampling")
    p.add_argument("--duration", type=float, default=5.0, help="Duration in seconds (continuous)")
    p.add_argument("--plot", action="store_true", help="Plot channels (requires matplotlib)")
    p.add_argument("--quiet", action="store_true", help="Less logging")
    args = p.parse_args()

    driver = ADS1298Driver(verbose=not args.quiet)
    if not driver.begin():
        print("ADS1298 init failed. Check wiring and run with sudo.")
        return 1

    try:
        if not args.continuous:
            driver.start_sampling()
            time.sleep(0.5)
            samples = driver.read_samples(args.count, timeout=5.0)
            driver.stop_sampling()
            if not samples:
                print("No samples received. Check DRDY connection and SPI.")
                return 1
            print(f"Received {len(samples)} samples (8 channels, volts):")
            for i, s in enumerate(samples):
                print(f"  {i+1}: " + " ".join(f"Ch{j+1}={v:.6f}" for j, v in enumerate(s)))
            if len(samples) >= 2:
                import statistics
                for ch in range(8):
                    vals = [s[ch] for s in samples]
                    print(f"  Ch{ch+1} mean={statistics.mean(vals):.6f} V std={statistics.stdev(vals):.6e}")
            print("Basic test OK.")
            return 0

        driver.start_sampling()
        deadline = time.monotonic() + args.duration
        all_voltages = []
        step = max(1, int(250 * args.duration / 20))
        n = 0
        while time.monotonic() < deadline:
            batch = driver.read_samples(step, timeout=1.0)
            for s in batch:
                all_voltages.append(s)
                n += 1
            if not args.quiet and n > 0 and n % 250 == 0:
                print(f"  {n} samples...")
        driver.stop_sampling()

        if not all_voltages:
            print("No samples in continuous run. Check DRDY/SPI.")
            return 1
        print(f"Continuous test: {len(all_voltages)} samples in {args.duration} s")

        if args.plot:
            try:
                import matplotlib.pyplot as plt
                import numpy as np
            except ImportError:
                print("Install matplotlib for plotting: pip install matplotlib")
                return 0
            arr = np.array(all_voltages)
            t = np.arange(len(arr)) / 250.0
            fig, axes = plt.subplots(8, 1, sharex=True, figsize=(10, 10))
            for ch in range(8):
                axes[ch].plot(t, arr[:, ch], linewidth=0.8)
                axes[ch].set_ylabel(f"Ch{ch+1} (V)")
                axes[ch].grid(True, alpha=0.3)
            axes[-1].set_xlabel("Time (s)")
            plt.suptitle(f"ADS1298 — {len(arr)} samples")
            plt.tight_layout()
            plt.show()
        return 0
    finally:
        driver.close()


if __name__ == "__main__":
    sys.exit(main())
