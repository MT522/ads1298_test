#!/bin/bash
# One-time setup for ADS1298 on Raspberry Pi 4 (Raspbian).
# Run: chmod +x setup_raspberry_pi.sh && ./setup_raspberry_pi.sh

set -e
echo "=== ADS1298 Raspberry Pi setup ==="

# Enable SPI (if not already)
if ! grep -q "dtparam=spi=on" /boot/config.txt 2>/dev/null; then
    echo "Enabling SPI in /boot/config.txt"
    echo "dtparam=spi=on" | sudo tee -a /boot/config.txt
else
    echo "SPI already enabled"
fi

# Add user to spi group (so we can use /dev/spidev* without root for some distros)
# On Raspbian, SPI device is often root:spi; running with sudo is typical.
if groups | grep -q spi 2>/dev/null; then
    echo "User already in group spi"
else
    echo "Add your user to group spi: sudo usermod -aG spi $USER"
fi

# Python deps
echo "Installing Python dependencies..."
pip3 install --user -r requirements.txt 2>/dev/null || true

echo ""
echo "Setup done. Reboot if SPI was just enabled: sudo reboot"
echo "Then run: sudo python3 test_ads1298.py"
