TARGET = build/asac-fc
# Depends on your system...
RP2040_USB_PATH = /media/victor/RPI-RP21/
DEFAULT_SERIAL_PORT = /dev/ttyACM0

# Builds firmware using Raspberry Pis build tools
all:
	make -C build -j4

cmake:
	cd build && cmake -DCMAKE_BUILD_TYPE=DEBUG -DCMAKE_C_FLAGS_DEBUG="-g -O0 -Wall" -DCMAKE_CXX_FLAGS_DEBUG="-g -O0" .. && cd ..

mavlink: force
	rm -rf include/mavlink
	tools/gen_mavlink

reboot:
	picotool reboot -f

# Flashes rp2040 over serial.
# Note that in order for this to work the rp2040 must be configured to use
# serial on usb.
flash_serial:
	picotool load -f ${TARGET}.uf2 && picotool reboot -f

# Flashes rp2040 over SWD using a pico debugger.
flash:
	openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 500" -c "program ${TARGET}.elf verify reset exit"

# Flashes rp2040 when it's registered as USB device.
flash_usb:
	sudo cp ${TARGET}.uf2 ${RP2040_USB_PATH} && sudo sync

# Monitor serial port
monitor:
	tools/monitor.py ${DEFAULT_SERIAL_PORT}

clean:
	rm -rf build/*


.PHONY: all flash_serial flash flash_usb scripts force
