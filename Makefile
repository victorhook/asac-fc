HAL ?= rp2040

all:
	$(MAKE) -C hal/hal_${HAL}
