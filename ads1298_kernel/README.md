# ADS1298 Linux kernel driver (Raspberry Pi)

SPI + GPIO kernel module for the Texas Instruments ADS1298 8-channel 24-bit ADC. Data is read on DRDY (GPIO interrupt), buffered in a kernel FIFO, and exposed via the character device **/dev/ads1298**.

## Hardware (BCM GPIO)

| ADS1298 pin | Raspberry Pi (BCM) | Notes        |
|-------------|--------------------|-------------|
| DIN         | 10 (MOSI)          | SPI0        |
| DOUT        | 9 (MISO)           | SPI0        |
| SCLK        | 11                 | SPI0        |
| CS          | 5                  | Manual CS   |
| START       | 17                 | Start conv. |
| RST         | 25                 | Reset       |
| DRDY        | 24                 | Data ready  |

SPI: 4 MHz, mode 1 (CPOL=0, CPHA=1). Each sample is **27 bytes** (3 status + 8×3 channel).

## Build (on Raspberry Pi)

### 1. Install kernel headers

```bash
sudo apt update
sudo apt install raspberrypi-kernel-headers build-essential
```

If the package name differs on your image, use the one that provides `/lib/modules/$(uname -r)/build`.

### 2. Build the module

```bash
cd ads1298_kernel
make
```

You should get `ads1298.ko`.

### 3. Compile Device Tree overlay (optional, for auto-probe)

```bash
sudo dtc -I dts -O dtb -o ads1298.dtbo -@ ads1298-overlay.dts
sudo cp ads1298.dtbo /boot/overlays/
```

Add to `/boot/config.txt` (or `/boot/firmware/config.txt` on Bookworm):

```
dtoverlay=ads1298
```

Reboot so the overlay is applied and the SPI device is created. Then load the driver (see below).

## Load / unload

- **Load** (if DT overlay is already applied and device exists):
  ```bash
  sudo insmod ads1298.ko
  ```

- **Load at boot**: install the module and add it to modules-load:
  ```bash
  sudo make modules_install
  sudo depmod -a
  echo ads1298 | sudo tee /etc/modules-load.d/ads1298.conf
  ```

- **Unload**:
  ```bash
  sudo rmmod ads1298
  ```

If you have not applied the overlay, the SPI device won’t exist and `insmod` will succeed but the driver won’t probe. Ensure the overlay is in `/boot/overlays/` and `dtoverlay=ads1298` is in the config and reboot.

## Usage

- **Device node**: `/dev/ads1298`
- **Format**: each `read()` returns **27-byte** frames. Read multiples of 27 (e.g. 27, 54, 1350 for 50 packets).
- **Blocking**: `read()` blocks until at least one packet is available (or EOF if the driver is stopped).

### Example: read 50 packets (1350 bytes)

```bash
sudo cat /dev/ads1298 | head -c 1350 > sample.bin
```

### Example: C

```c
int fd = open("/dev/ads1298", O_RDONLY);
uint8_t buf[27];
while (read(fd, buf, sizeof(buf)) == (ssize_t)sizeof(buf)) {
    /* process packet: buf[0..2] status, buf[3..26] 8×24-bit channels */
}
close(fd);
```

### Example: Python (decode with existing helper)

Use the same packet format as the userspace Python driver (27 bytes, status nibble 0xC0). You can reuse `packet_to_voltages()` from `raspberry_pi_driver/ads1298_driver.py`:

```python
with open("/dev/ads1298", "rb") as f:
    while True:
        packet = f.read(27)
        if len(packet) != 27:
            break
        from ads1298_driver import packet_to_voltages
        vols = packet_to_voltages(packet)
        if vols:
            print(vols)
```

(Ensure the driver package is on `PYTHONPATH` or run from `raspberry_pi_driver`.)

## Notes

- The driver uses a **threaded IRQ** for DRDY, so the 27-byte SPI read runs in kernel context and can achieve higher and more reliable sample rates (e.g. 4 kSPS) than a userspace Python callback.
- FIFO holds 512 packets (~13.5 kB). If userspace doesn’t read fast enough, older samples are overwritten (ring buffer).
- After probe, the driver sends SDATAC, configures registers (CONFIG1/2/3, channels, etc.), then RDATAC and START. Channels are set to normal input (0x00); change in `ads1298_init_device()` in `ads1298.c` if you need test signals or different gain.
