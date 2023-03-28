CC = gcc
CFLAGS = -g


all:
	cd build && make -j4 && cd ..

flash:
	cd build && make -j4 && cd .. && openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 500" -c "program build/asac-fc.elf verify reset exit"

flash_usb: $(TARGET)
	sudo cp build/asac-fc.uf2 /media/victor/RPI-RP21/ && sudo sync

#$(TARGET):
#	cd build && cmake .. && make -j4

debug:
	${CC} ${CFLAGS} src/main.c src/msp.c -Iinclude -o msp_test -Wl,-Map,output.map
