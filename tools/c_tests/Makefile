CC = gcc
INCLUDES = -I../../include
CFLAGS = -DLINUX $(INCLUDES)



test_elrs:
	$(CC) $(CFLAGS) \
	../../src/rc/crsf.c \
	../../src/asac_fc.c \
	elrs_test.c -o run && ./run


test_smartaudio:
	$(CC) $(CFLAGS) \
	../../src/asac_fc.c \
	../../src/vtx/smartaudio.c \
	smartaudio_test.c -o run && ./run

#gcc -DLINUX -I../../include ../../src/rc/crsf.c ../../src/asac_fc.c elrs_test.c -o run && ./run