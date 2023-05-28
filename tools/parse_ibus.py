#!/usr/bin/env python3
from serial import Serial
from serial.serialutil import SerialException
import sys
import time


HEADER_FIRST_BYTE = 0
HEADER_SECOND_BYTE = 1
PAYLOAD = 2
CHECKSUM_FIRST_BYTE = 3
CHECKSUM_SECOND_BYTE = 4


ibus_state = HEADER_FIRST_BYTE
parse_errors = 0
payload = [0 for i in range(28)]
buf = []
data_bytes = 0
rx_checksum: int
checksum: int
pkt_rate = 0
pkt_cnt = 0
t0 = 0
j = 0


if __name__ == '__main__':
    try:
        port = sys.argv[1]
    except Exception:
        print('Supply serial port!')
        sys.exit(0)

    connected = False
    while not connected:
        try:
            serial = Serial(port, baudrate=115200)
            connected = True
        except SerialException:
            try:
                time.sleep(.05)
            except KeyboardInterrupt:
                sys.exit(0)

    print('** Connected **')

    def ibus_parse(byte: bytes) -> None:
        global ibus_state
        global parse_errors
        global payload
        global data_bytes
        global rx_checksum
        global buf
        global checksum
        global pkt_rate
        global pkt_cnt
        global t0
        global j

        if isinstance(byte, bytes):
            byte = ord(byte)

        next_state = ibus_state
        buf.append(byte)

        if ibus_state == HEADER_FIRST_BYTE:
            if byte == 0x20:
                next_state = HEADER_SECOND_BYTE
                buf = [byte]
            else:
                parse_errors += 1
        elif ibus_state == HEADER_SECOND_BYTE:
            if byte == 0x40:
                next_state = PAYLOAD
                checksum = 0xFFFF - 0x40 - 0x20
                data_bytes = 0
            else:
                next_state = HEADER_FIRST_BYTE
                parse_errors += 1
        elif ibus_state == PAYLOAD:
            payload[data_bytes] = byte
            data_bytes += 1
            checksum -= byte
            if len(buf) == 30:
                next_state = CHECKSUM_FIRST_BYTE
        elif ibus_state == CHECKSUM_FIRST_BYTE:
            rx_checksum = byte
            next_state = CHECKSUM_SECOND_BYTE
        elif ibus_state == CHECKSUM_SECOND_BYTE:
            rx_checksum |= (byte << 8)
            next_state = HEADER_FIRST_BYTE

            channels = []
            i = 0
            while i < 28:
                channels.append(
                    (payload[i+1] << 8) | payload[i]
                )
                i += 2

            a = 0xffff
            for i in channels:
                a -= i
            #print(' '.join(hex(a)[2:].zfill(2) for a in buf))

            pkt_cnt += 1

            now = time.time()
            if ((now - t0) > 1):
                pkt_rate = pkt_cnt
                pkt_cnt = 0
                t0 = now
                j += 1
                print(f'[{j}] Errs: {parse_errors}', end=' : ')
                print(f'Rate: {pkt_rate} : ')
                print(hex(buf[0]) + ' ' + hex(buf[1]), end=' ')
                print(channels, end=' -> ')
                print(f'{rx_checksum} - {checksum} -- {a}')

        ibus_state = next_state

    try:
        while True:
            #d = serial.read(32)
            #print(' '.join(hex(a)[2:].zfill(2) for a in d))
            d = serial.read(1)
            ibus_parse(d)
    except KeyboardInterrupt:
        sys.exit(0)
    finally:
        try:
            serial.close()
        except Exception:
            pass
