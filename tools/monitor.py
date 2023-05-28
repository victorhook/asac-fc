#!/usr/bin/env python3
from serial import Serial
from serial.serialutil import SerialException
import sys
import time


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

    try:
        while True:
            d = serial.read(1)
            string = d.decode('ascii')
            sys.stdout.write(string)
            sys.stdout.flush()
    except KeyboardInterrupt:
        sys.exit(0)
    finally:
        try:
            serial.close()
        except Exception:
            pass
