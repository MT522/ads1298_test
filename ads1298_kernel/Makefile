# Out-of-tree build for ADS1298 kernel module (Raspberry Pi / Linux)
# Build on the Pi: make
# Load: sudo insmod ads1298.ko  (or use udev + overlay)

obj-m += ads1298.o

# Kernel build path: use running kernel by default
KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build

# For cross-compile (example):
# export ARCH=arm
# export CROSS_COMPILE=arm-linux-gnueabihf-
# KERNEL_SRC = /path/to/linux

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(CURDIR) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(CURDIR) modules_install
	depmod -a

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(CURDIR) clean

.PHONY: all modules_install clean
