from pymavlink.dialects.v10 import common
from pymavlink import mavutil
from serial import Serial
import sys
from threading import Event, Thread
from io import StringIO
from queue import Queue


class ASAC:

    def __init__(self, port: str) -> None:
        self.port = port
        self._stop_flag = Event()
        self._stop_flag.set()
        self._serial = Serial(baudrate=115200, timeout=1, write_timeout=1)
        self._fake_file = StringIO()
        self._mav = common.MAVLink(self._fake_file)
        self._rx = Queue()

    def start(self) -> None:
        if not self._stop_flag.is_set():
            return

        self._serial.port = self.port
        self._serial.open()
        self._stop_flag.clear()
        Thread(target=self._receive_thread, daemon=True).start()

    def stop(self) -> None:
        if self._stop_flag.is_set():
            return

        self._stop_flag.stop()
        self._serial.close()

    def _receive_thread(self) -> None:
        print('RX Thread started')
        while not self._stop_flag.is_set():
            byte = self._serial.read(1)
            if byte:
                try:
                    res = self._mav.parse_char(byte)
                    if res:
                        self._rx.put(res)
                        print(res)
                except common.MAVError:
                    pass
        print('RX Thread ended')

if __name__ == '__main__':
    PORT = sys.argv[1]
    asac = ASAC(PORT)
    asac.start()

    while 1:
        import time
        time.sleep(11)