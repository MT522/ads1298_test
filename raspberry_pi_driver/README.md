# ADS1298 driver for Raspberry Pi 4 (Raspbian)

SPI + GPIO driver for the Texas Instruments ADS1298 8-channel ADC. Uses **spidev** for SPI and **RPi.GPIO** for CS, START, RST, and DRDY (interrupt-driven data ready).

## Hardware wiring (Raspberry Pi 4 — BCM GPIO numbering)

All GPIO numbers below are **BCM GPIO numbers** (the “BCM” column on `pinput.xyz`), not physical header pin numbers.

Notes about SPI chip-select pins:
- **BCM 8 = CE0**
- **BCM 7 = CE1**

To avoid conflicts, this wiring does **not** use CE0/CE1 for ADS1298 control pins.

| ADS1298 pin | Raspberry Pi   | Notes           |
|-------------|----------------|-----------------|
| DIN         | GPIO 10 (MOSI) | SPI0 MOSI       |
| DOUT        | GPIO 9 (MISO)  | SPI0 MISO       |
| SCLK        | GPIO 11 (SCLK) | SPI0 SCLK       |
| CS          | GPIO 5         | Manual chip select |
| START       | GPIO 17        | Start conversion |
| RST         | GPIO 25        | Reset (active low) |
| DRDY        | GPIO 24        | Data ready (falling edge) |
| GND         | GND            |                 |
| DVDD/AVDD   | 3.3 V          |                 |
| PWDN        | 3.3 V          | Power enable   |

SPI is used at **4 MHz**, mode **1** (CPOL=0, CPHA=1). CS is toggled in software (datasheet: wait 4 tCLK after transfer before deasserting CS).

## Setup on Raspberry Pi

```bash
cd raspberry_pi_driver
chmod +x setup_raspberry_pi.sh
./setup_raspberry_pi.sh
# Reboot if SPI was enabled for the first time
sudo reboot
```

## Install Python dependencies

```bash
pip3 install --user -r requirements.txt
```

Or: `make install`

## Run tests (use sudo for GPIO/SPI access)

```bash
# Basic: read 5 samples and print voltages
sudo python3 test_ads1298.py

# Continuous: sample for 5 seconds
sudo python3 test_ads1298.py --continuous --duration 5

# Continuous + plot (requires matplotlib)
sudo python3 test_ads1298.py --continuous --plot
```

Or: `make test`, `make test-continuous`, `make test-plot`

## Using the driver in your code

```python
from ads1298_driver import ADS1298Driver, packet_to_voltages

driver = ADS1298Driver(
    cs_pin=5,
    drdy_pin=24,
    start_pin=17,
    rst_pin=25,
    verbose=True,
)
if not driver.begin():
    raise RuntimeError("ADS1298 init failed")

driver.start_sampling()
samples = driver.read_samples(100, timeout=5.0)  # list of (v1..v8) tuples
driver.stop_sampling()
driver.close()
```

## Data format

- Each sample is **27 bytes**: 3 status bytes + 8 channels × 3 bytes (24-bit two's complement, MSB first).
- Valid packet: first byte high nibble = `0xC`.
- Voltage: `V = code * (2.4 / 8388607)` (VREF = 2.4 V).

## Notes

- **DMA**: Raspberry Pi SPI does not expose the same DMA API as the ESP32; data is read in the DRDY callback via `spidev.xfer2()`, which is efficient for 27-byte frames.
- **Root**: Access to `/dev/spidev0.0` and GPIO usually requires `sudo` unless udev rules and group membership are set up.
- **RPi.GPIO**: Uses BCM numbering. Ensure no other process uses the same GPIOs.
