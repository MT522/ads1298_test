#!/usr/bin/env python3
"""
Check SPI on Raspberry Pi (device node and optional loopback).
Usage:
  sudo python3 check_spi.py                    # Check /dev/spidev exists
  sudo python3 check_spi.py --loopback         # Loopback test (short MOSI to MISO first!)
  sudo python3 check_spi.py --ads1298          # Quick ADS1298 init (Device ID + reg verify)
"""

import argparse
import os
import sys

def check_device():
    """Verify SPI device node exists."""
    path = "/dev/spidev0.0"
    if os.path.exists(path):
        print(f"OK: {path} exists (SPI is enabled).")
        return True
    print(f"Missing: {path}. Enable SPI: sudo raspi-config -> Interface -> SPI -> Enable, then reboot.")
    return False

def loopback_test():
    """Send bytes and read back (requires MOSI shorted to MISO)."""
    try:
        import spidev
    except ImportError:
        print("Install spidev: pip3 install spidev")
        return False
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 1_000_000
    spi.mode = 1
    try:
        # CE0 is used by spidev; we only test the bus
        tx = [0x55, 0xAA, 0xF0, 0x0F]
        rx = spi.xfer2(tx)
        spi.close()
        if tx == rx:
            print("Loopback OK: sent and received bytes match.")
            return True
        print(f"Loopback MISMATCH: sent {[hex(x) for x in tx]}, got {[hex(x) for x in rx]}")
        print("Ensure MOSI (GPIO 10) is shorted to MISO (GPIO 9), then run again.")
        return False
    except Exception as e:
        spi.close()
        print(f"Loopback error: {e}")
        return False

def ads1298_quick_check():
    """Run driver begin() to verify SPI with ADS1298 (Device ID + register verify)."""
    try:
        from ads1298_driver import ADS1298Driver
    except ImportError:
        print("Run from raspberry_pi_driver directory: python3 check_spi.py --ads1298")
        return False
    driver = ADS1298Driver(verbose=True)
    ok = driver.begin()
    driver.close()
    if ok:
        print("ADS1298 check OK: Device ID and register read-back passed.")
    else:
        print("ADS1298 check failed. Check wiring (MOSI, MISO, SCLK, CS, RST, power).")
    return ok

def main():
    p = argparse.ArgumentParser(description="Check SPI on Raspberry Pi")
    p.add_argument("--loopback", action="store_true", help="Run loopback test (short MOSI to MISO first)")
    p.add_argument("--ads1298", action="store_true", help="Quick ADS1298 init and register verify")
    args = p.parse_args()

    if not check_device():
        return 1

    if args.loopback:
        print("Running loopback test (MOSI must be shorted to MISO)...")
        return 0 if loopback_test() else 1
    if args.ads1298:
        return 0 if ads1298_quick_check() else 1

    print("Use --loopback to test SPI bus (short MOSI to MISO), or --ads1298 to test with ADS1298.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
