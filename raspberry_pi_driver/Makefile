# ADS1298 Raspberry Pi driver — convenience targets
# Run from raspberry_pi_driver/

.PHONY: test test-basic test-continuous test-plot install setup help

help:
	@echo "ADS1298 Raspberry Pi driver"
	@echo "  make install   - pip install requirements"
	@echo "  make setup    - run setup_raspberry_pi.sh"
	@echo "  make test     - sudo python3 test_ads1298.py (basic 5 samples)"
	@echo "  make test-continuous - sudo python3 test_ads1298.py --continuous --duration 5"
	@echo "  make test-plot - sudo python3 test_ads1298.py --continuous --plot"

install:
	pip3 install --user -r requirements.txt

setup:
	chmod +x setup_raspberry_pi.sh && ./setup_raspberry_pi.sh

test:
	sudo python3 test_ads1298.py

test-basic:
	sudo python3 test_ads1298.py --count 10

test-continuous:
	sudo python3 test_ads1298.py --continuous --duration 5

test-plot:
	sudo python3 test_ads1298.py --continuous --plot
