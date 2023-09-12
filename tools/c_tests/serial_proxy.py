import serial
import socket


if __name__ == '__main__':
    server_port = 9090
    ser = serial.Serial(port='/dev/ttyUSB0', baudrate=420000)
    print('Connected to serial port!')

    while True:
        print('Connecting to server...')
        sock = socket.socket()
        sock.connect(('127.0.0.1', server_port))
        print('Connected to server')

        try:
            while True:
                b = ser.read(256)
                if b:
                    sock.send(b)

                #sock.close()
                #import sys
                #sys.exit(0)
        except BrokenPipeError:
            sock.close()
