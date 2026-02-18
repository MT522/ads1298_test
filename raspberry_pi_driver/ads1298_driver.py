#!/usr/bin/env python3
"""
ADS1298 driver for Raspberry Pi 4 (Raspbian) using SPI + GPIO.
Uses spidev for SPI and RPi.GPIO for CS, START, RST, and DRDY (interrupt).
Data is read on DRDY falling edge; 27-byte packets are validated and queued.
"""

import time
import threading
import queue
from typing import Optional, List, Tuple

# Constants from ADS1298 datasheet / ESP32 driver
WAKEUP = 0x02
STANDBY = 0x04
RESET = 0x06
START = 0x08
STOP = 0x0A
RDATAC = 0x10
SDATAC = 0x11
RDATA = 0x12
RREG = 0x20
WREG = 0x40

REG_DEVID = 0x00
REG_CONFIG1 = 0x01
REG_CONFIG2 = 0x02
REG_CONFIG3 = 0x03
REG_LOFF = 0x04
REG_CH1SET = 0x05
REG_CH2SET = 0x06
REG_CH3SET = 0x07
REG_CH4SET = 0x08
REG_CH5SET = 0x09
REG_CH6SET = 0x0A
REG_CH7SET = 0x0B
REG_CH8SET = 0x0C
REG_RLD_SENSP = 0x0D
REG_RLD_SENSN = 0x0E
REG_LOFF_SENSP = 0x0F
REG_LOFF_SENSN = 0x10
REG_LOFF_FLIP = 0x11
REG_LOFF_STATP = 0x12
REG_LOFF_STATN = 0x13
REG_GPIO = 0x14
REG_PACE = 0x15
REG_RESP = 0x16
REG_CONFIG4 = 0x17
REG_WCT1 = 0x18
REG_WCT2 = 0x19

PACKET_SIZE = 27  # 3 status + 8*3 channel bytes
HEADER_BYTES = 3
CHANNELS = 8
BYTES_PER_CHANNEL = 3
VALID_STATUS_NIBBLE = 0xC0  # high nibble of first byte = valid packet
WAIT_AFTER_SPI_US = 2e-6    # 4 tCLK before deassert CS
VREF_V = 2.4
POSITIVE_FS_CODE = (1 << 23) - 1
LSB_V = VREF_V / POSITIVE_FS_CODE

# Default Raspberry Pi 4 pins (BCM)
# SPI0: MOSI=10, MISO=9, SCLK=11 (fixed). We use GPIO for CS (not CE0).
DEFAULT_CS_PIN = 5
DEFAULT_DRDY_PIN = 24
DEFAULT_START_PIN = 17
DEFAULT_RST_PIN = 25
DEFAULT_SPI_BUS = 0
DEFAULT_SPI_DEVICE = 0
DEFAULT_SPI_SPEED_HZ = 4_000_000


def _parse_24bit(b0: int, b1: int, b2: int) -> int:
    b0, b1, b2 = b0 & 0xFF, b1 & 0xFF, b2 & 0xFF
    raw = (b0 << 16) | (b1 << 8) | b2
    if raw >= 0x800000:
        raw -= 0x1000000
    return raw


def packet_to_voltages(packet: bytes) -> Optional[Tuple[float, ...]]:
    """Decode 27-byte packet to 8 channel voltages in volts. None if invalid."""
    if len(packet) < PACKET_SIZE:
        return None
    if (packet[0] & 0xF0) != VALID_STATUS_NIBBLE:
        return None
    data = packet[HEADER_BYTES : HEADER_BYTES + CHANNELS * BYTES_PER_CHANNEL]
    vols = []
    for ch in range(CHANNELS):
        i = ch * BYTES_PER_CHANNEL
        code = _parse_24bit(data[i], data[i + 1], data[i + 2])
        vols.append(code * LSB_V)
    return tuple(vols)


class ADS1298Driver:
    """ADS1298 driver for Raspberry Pi: SPI + GPIO (CS, START, RST, DRDY)."""

    def __init__(
        self,
        cs_pin: int = DEFAULT_CS_PIN,
        drdy_pin: int = DEFAULT_DRDY_PIN,
        start_pin: int = DEFAULT_START_PIN,
        rst_pin: int = DEFAULT_RST_PIN,
        spi_bus: int = DEFAULT_SPI_BUS,
        spi_device: int = DEFAULT_SPI_DEVICE,
        spi_speed_hz: int = DEFAULT_SPI_SPEED_HZ,
        verbose: bool = True,
    ):
        self.cs_pin = cs_pin
        self.drdy_pin = drdy_pin
        self.start_pin = start_pin
        self.rst_pin = rst_pin
        self.spi_bus = spi_bus
        self.spi_device = spi_device
        self.spi_speed_hz = spi_speed_hz
        self.verbose = verbose
        self._spi = None
        self._gpio_ready = False
        self._sample_queue: queue.Queue = queue.Queue()
        self._drdy_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

    def _log(self, msg: str) -> None:
        if self.verbose:
            print(msg)

    def _assert_cs(self) -> None:
        try:
            import RPi.GPIO as GPIO
            GPIO.output(self.cs_pin, GPIO.LOW)
        except Exception:
            pass

    def _deassert_cs(self) -> None:
        time.sleep(WAIT_AFTER_SPI_US)
        try:
            import RPi.GPIO as GPIO
            GPIO.output(self.cs_pin, GPIO.HIGH)
        except Exception:
            pass

    def _spi_transfer(self, tx: List[int]) -> List[int]:
        """Transfer bytes over SPI with manual CS. Returns list of received bytes (same length as tx)."""
        self._assert_cs()
        try:
            rx = self._spi.xfer2(tx)
            time.sleep(WAIT_AFTER_SPI_US)
            return list(rx) if rx else []
        finally:
            self._deassert_cs()

    def _send_command(self, cmd: int) -> None:
        self._spi_transfer([cmd])
        self._log(f"Command 0x{cmd:02X} sent")

    def _reg_write(self, addr: int, data: int) -> None:
        with self._lock:
            self._assert_cs()

            try:
                self._spi.xfer2([addr | WREG])
                time.sleep(WAIT_AFTER_SPI_US)

                self._spi.xfer2([0x00, data])
                time.sleep(WAIT_AFTER_SPI_US)

                self._log(f"\033[35mWrite reg 0x{addr:02X} = 0x{data:02X}\033[37m")
            finally:
                self._deassert_cs()

    def _reg_read(self, addr: int) -> int:
        with self._lock:
            self._assert_cs()

            try:
                self._spi.xfer2([addr | RREG])
                time.sleep(WAIT_AFTER_SPI_US)

                rx = self._spi.xfer2([0x00, 0x00])
                time.sleep(WAIT_AFTER_SPI_US)

                self._log(f"\033[35mRead reg 0x{addr:02X} = 0x{rx[1]:02X}\033[37m")
                return rx[1]
            finally:
                self._deassert_cs()

    def _hardware_reset(self) -> None:
        import RPi.GPIO as GPIO
        GPIO.output(self.rst_pin, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(self.rst_pin, GPIO.LOW)
        time.sleep(0.000002)
        GPIO.output(self.rst_pin, GPIO.HIGH)
        time.sleep(0.00001)
        self._log("Hardware reset done")

    def _drdy_callback(self, channel: int) -> None:
        """Called on DRDY falling edge: read 27 bytes and queue if valid."""
        try:
            import RPi.GPIO as GPIO
        except Exception:
            return
        self._assert_cs()
        try:
            rx = self._spi.xfer2([0xFF] * PACKET_SIZE)
            self._log(f"\033[35mReceived packet, first header byte: 0x{rx[0]:02X}\033[37m")
        except Exception:
            return
        finally:
            self._deassert_cs()
            time.sleep(WAIT_AFTER_SPI_US)

        if len(rx) != PACKET_SIZE:
            self._log(f"\033[31mReceived packet with length {len(rx)} != {PACKET_SIZE}\033[37m")
            return
        packet = bytes(rx)
        if (packet[0] & 0xF0) != VALID_STATUS_NIBBLE:
            self._log(f"\033[31mReceived packet with invalid status nibble: 0x{packet[0]:02X}\033[37m")
            return
        try:
            self._sample_queue.put_nowait(packet)
        except queue.Full:
            self._log(f"\033[31mSample queue is full, dropping packet\033[37m")
            pass
        self._log(f"\033[32mQueued packet with {len(packet)} bytes\033[37m")

    def _drdy_poll_thread(self) -> None:
        """Poll DRDY in a loop (fallback if edge detection is flaky)."""
        import RPi.GPIO as GPIO
        last = GPIO.input(self.drdy_pin)
        while not self._stop_event.is_set():
            time.sleep(0.0001)
            v = GPIO.input(self.drdy_pin)
            if last == 1 and v == 0:  # falling edge
                self._drdy_callback(self.drdy_pin)
            last = v

    def begin(self) -> bool:
        """Initialize SPI, GPIO, reset ADS1298, and load config. Returns True if ID read is 0x92."""
        import RPi.GPIO as GPIO
        import spidev

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in (self.cs_pin, self.start_pin, self.rst_pin):
            GPIO.setup(pin, GPIO.OUT)
        GPIO.setup(self.drdy_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self._gpio_ready = True

        GPIO.output(self.cs_pin, GPIO.HIGH)
        GPIO.output(self.start_pin, GPIO.LOW)
        GPIO.output(self.rst_pin, GPIO.HIGH)

        # SPI settings matching ADS1298 reference (concept design/main.c): 8-bit, CPOL=0, CPHA=1, MSB first
        self._spi = spidev.SpiDev()
        self._spi.open(self.spi_bus, self.spi_device)
        self._spi.max_speed_hz = self.spi_speed_hz
        self._spi.mode = 1           # CPOL=0 (SPI_POLARITY_LOW), CPHA=1 (SPI_PHASE_2EDGE)
        self._spi.bits_per_word = 8  # SPI_DATASIZE_8BIT
        self._spi.lsbfirst = False   # SPI_FIRSTBIT_MSB
        self._log(f"SPI opened: 8-bit, mode 1 (CPOL=0 CPHA=1), MSB first, {self.spi_speed_hz // 1_000_000} MHz")

        self._hardware_reset()
        time.sleep(0.01)

        self._send_command(SDATAC)
        time.sleep(0.002)

        dev_id = self._reg_read(REG_DEVID)
        self._log(f"Device ID: 0x{dev_id:02X}")
        if dev_id != 0x92:
            self._log("Expected 0x92 for ADS1298. Check wiring.")
            return False

        # Power-on style config (internal test signal for testing)
        config_regs = [
            (REG_CONFIG1, 0xA4, "CONFIG1 (HR, 2kSPS)"),
            (REG_CONFIG2, 0x31, "CONFIG2 (internal test 2 Hz)"),
            (REG_CONFIG3, 0xCC, "CONFIG3"),
            (REG_RLD_SENSP, 0xFF, "RLD_SENSP"),
            (REG_RLD_SENSN, 0xFF, "RLD_SENSN"),
            (REG_PACE, 0x01, "PACE"),
            (REG_WCT1, 0x09, "WCT1"),
            (REG_WCT2, 0xD0, "WCT2"),
            (REG_LOFF, 0x07, "LOFF"),
            (REG_CONFIG4, 0x02, "CONFIG4"),
            (REG_LOFF_SENSP, 0xFF, "LOFF_SENSP"),
            (REG_LOFF_SENSN, 0xFF, "LOFF_SENSN"),
        ]
        for addr, val, _ in config_regs:
            self._reg_write(addr, val)
        for i in range(8):
            self._reg_write(REG_CH1SET + i, 0x35)
        ch_regs = [(REG_CH1SET + i, 0x35, f"CH{i+1}SET") for i in range(8)]
        all_regs = config_regs + ch_regs

        # Register read-back verification
        self._log("Register verification (write then read-back):")
        ok = 0
        fail = 0
        for addr, expected, desc in all_regs:
            read_val = self._reg_read(addr)
            match = read_val == expected
            if match:
                ok += 1
                self._log(f" \033[32m Reg 0x{addr:02X} ({desc}): wrote 0x{expected:02X}, read 0x{read_val:02X} OK\033[37m")
            else:
                fail += 1
                self._log(f"  \033[33mReg 0x{addr:02X} ({desc}): wrote 0x{expected:02X}, read 0x{read_val:02X} MISMATCH\033[37m")
        self._log(f"Verification: {ok} OK, {fail} mismatch(es).")
        if fail > 0:
            self._log("Check SPI wiring (MOSI/MISO/SCLK/CS) and that ADS1298 is powered.")

        self._log("ADS1298 init done.")
        return True

    def start_sampling(self) -> None:
        """Start conversions and install DRDY callback to fill sample queue."""
        import RPi.GPIO as GPIO
        self._stop_event.clear()
        self._send_command(START)
        self._send_command(RDATAC)
        GPIO.output(self.start_pin, GPIO.HIGH)
        time.sleep(0.01)
        try:
            GPIO.add_event_detect(
                self.drdy_pin,
                GPIO.FALLING,
                callback=self._drdy_callback,
                bouncetime=1,
            )
            self._log("Start ECG sampling interrupt mode.")
        except Exception:
            self._drdy_thread = threading.Thread(target=self._drdy_poll_thread, daemon=True)
            self._drdy_thread.start()
            self._log("Start ECG sampling polling mode.")

    def stop_sampling(self) -> None:
        """Stop conversions and remove DRDY callback."""
        import RPi.GPIO as GPIO
        try:
            GPIO.remove_event_detect(self.drdy_pin)
        except Exception:
            pass
        self._stop_event.set()
        GPIO.output(self.start_pin, GPIO.LOW)
        self._send_command(STOP)
        self._send_command(SDATAC)
        self._log("Stop ECG sampling.")

    def read_samples(self, count: int, timeout: float = 5.0) -> List[Tuple[float, ...]]:
        """Read up to `count` decoded samples (8 voltages each). Blocks up to `timeout` s."""
        samples = []
        deadline = time.monotonic() + timeout
        while len(samples) < count and time.monotonic() < deadline:
            try:
                packet = self._sample_queue.get(timeout=0.1)
                v = packet_to_voltages(packet)
                if v is not None:
                    samples.append(v)
            except queue.Empty:
                continue
        return samples

    def close(self) -> None:
        """Release SPI and GPIO."""
        self.stop_sampling()
        if self._spi is not None:
            try:
                self._spi.close()
            except Exception:
                pass
            self._spi = None
        if self._gpio_ready:
            try:
                import RPi.GPIO as GPIO
                GPIO.cleanup()
            except Exception:
                pass
            self._gpio_ready = False
        self._log("ADS1298 driver closed.")
