#!/usr/bin/env python3
"""
Capture raw UART output from ESP32 to a binary file for decode_ads1298_binary.py.
Requires: pip install pyserial

Usage:
  python capture_serial_binary.py COM3 capture.bin
  python capture_serial_binary.py /dev/ttyUSB0 capture.bin --duration 10
"""

import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    print("Install pyserial: pip install pyserial")
    sys.exit(1)


def main():
    import argparse
    p = argparse.ArgumentParser(description="Capture ESP32 UART to binary file (115200 baud)")
    p.add_argument("port", help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
    p.add_argument("output", help="Output binary file (e.g. capture.bin)")
    p.add_argument("--duration", type=float, default=5.0, help="Capture duration in seconds")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = p.parse_args()
    out_path = Path(args.output)
    print(f"Opening {args.port} at {args.baud} baud. Capturing for {args.duration} s to {out_path}")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        print(f"Error opening port: {e}")
        return 1
    start = time.monotonic()
    total = 0
    with open(out_path, "wb") as f:
        while (time.monotonic() - start) < args.duration:
            n = ser.in_waiting
            if n:
                data = ser.read(n)
                f.write(data)
                total += len(data)
            else:
                time.sleep(0.01)
    ser.close()
    print(f"Captured {total} bytes to {out_path}")
    print(f"Run: python decode_ads1298_binary.py {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
