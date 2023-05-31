CC = gcc
CFLAGS = -g
TARGET = build/asac-fc

all: scripts
	cd build && cmake -DCMAKE_BUILD_TYPE=DEBUG -DCMAKE_C_FLAGS_DEBUG="-g -O0 -Wall" -DCMAKE_CXX_FLAGS_DEBUG="-g -O0" .. && make -j4 && cd ..

flash_serial:
	picotool load -f ${TARGET}.uf2 && picotool reboot -f

flash:
	openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 500" -c "program ${TARGET}.elf verify reset exit"

flash_usb:
	sudo cp ${TARGET}.uf2 /media/victor/RPI-RP21/ && sudo sync

debug:
	${CC} ${CFLAGS} src/main.c src/msp.c -Iinclude -o msp_test -Wl,-Map,output.map

scripts:
	tools/gen_log_params.py

clean:
	rm -rf build/*