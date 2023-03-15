CC = gcc
CFLAGS = -g

all:
	${CC} ${CFLAGS} src/main.c src/msp.c -Iinclude -o msp_test -Wl,-Map,output.map
