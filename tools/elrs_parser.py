from serial import Serial
from serial.tools.list_ports import comports
import sys
from dataclasses import dataclass
from queue import Queue
from threading import Thread
import bitstruct



def print_thread(tx):
    print('Print thread started')
    pkt = bytearray()
    while True:
        d = tx.get()

        if d == b'\xc8':
            sys.stdout.write('\n')
            sys.stdout.write(" ".join(hex(b)[2:].zfill(2) for b in pkt))
            sys.stdout.flush()

            if len(pkt) > 2 and pkt[2] == 0x16:
                # RC CHANNELS
                data = pkt[3:-1]
                if len(data) < 22:
                    print('Not 22 bytes data...')
                else:
                    r = bitstruct.unpack('<' + 'u11' * 16, data)
                    print(r)


            pkt = bytearray()

        pkt.extend(d)


        #d = hex(ord(d))[2:].zfill(2)
        #sys.stdout.write(d)
        #sys.stdout.write(' ')
        #sys.stdout.flush()
    print('Print thread ended')


if __name__ == '__main__':
    print('Serial ports: ', ', '.join([p.device for p in comports()]))
    port = sys.argv[1]
    baud = 420000
    print(f'Using port: {port}, baud: {baud}')
    serial = Serial(port, baudrate=baud)
    serial.rtscts = False
    serial.dsrdtr = False

    tx = Queue()
    Thread(target=print_thread, args=(tx, ), daemon=True).start()

    while True:
        d = serial.read(1)
        if d:
            tx.put(d)
