#!/usr/bin/env python3

import socket
import threading
import time
import os
import sys


import queue
import sys
from dataclasses import fields

from PyQt6.QtWidgets import QApplication, QLabel, QWidget, QMainWindow, QFrame, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QScrollArea, QSizePolicy, QGridLayout, QLineEdit
from PyQt6.QtCore import QTimer, Qt, QThread
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import time

from typing import Dict
from queue import Queue
from serial import Serial


class Gui(QMainWindow):

    def __init__(self) -> None:
        super().__init__()


        # -- Graph content -- #
        self.graph = pg.PlotWidget()
        self.setCentralWidget(self.graph)
        self.graph.setBackground('w')
        self.x = []
        self.y = []
        self.line = self.graph.plot(
            self.x,
            self.y,
            pen=pg.mkPen('r', width=1)
        )

        # Timer and FPS counter
        self._graphics_update_timer = QTimer()
        self._graphics_update_timer.setInterval(33)
        self._graphics_update_timer.timeout.connect(self._update_graphics)
        self._graphics_update_timer.start()
        self._fps_counter = 0
        self._fps = 0
        self._fps_t0 = 0

    def _update_graphics(self) -> None:
        self.line.setData(self.x, self.y)

    def add(self, x, y) -> None:
        self.x.append(x)
        self.y.append(y)



if __name__ == '__main__':
    target_ssid = 'telemetrynode'
    ssid_name = os.popen('iwgetid -r').read().strip()
    if ssid_name != target_ssid:
        print(f'Not connected to target SSID "{target_ssid}", connecting...')
        os.system(f'nmcli con up {target_ssid}')

    #ip, port = '192.168.4.1', 80
    #'nmcli con up telemetrynode'
    #print(f'Trying to connect to ip: {ip}:{port}')


    app = QApplication([])
    gui = Gui()
    gui.show()

    def go(gui):
        use_tcp = True
        ip, port = '192.168.4.1', 80

        if use_tcp:
            sock = socket.socket()
            sock.connect((ip, port))
            sock.settimeout(.05)
        else:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            #sock.bind(('192.168.4.2', 1234))
            sock.bind(('', 1234))
            sock.settimeout(.05)

        print('Connected!')
        t0 = time.time()
        i = 0
        samples = []

        x = 0
        s = 3

        while True:
            try:
                data = sock.recv(4096)
                if data:
                    i += len(data)
            except TimeoutError:
                pass

            now = time.time()
            if (now - t0) > (1 / s):
                bps = round((i*s)/1000, 3)
                samples.append(bps)
                sys.stdout.write(f'\rRX: {bps} kB/s')
                sys.stdout.flush()
                i = 0
                t0 = now

                x += 1/s
                gui.add(x, bps)

    def go_serial(gui, port: str):
        # Need this weird wait of opening serial because of d1 mini...
        serial = Serial(baudrate=2000000)
        serial._rts_state = False
        serial._dtr_state = False
        serial.port = '/dev/ttyUSB0'
        serial.open()
        serial.timeout = .5
        print(f'Connected to serial {serial.port}, baudrate: {serial.baudrate}')

        t0 = time.time()
        rx = 0
        samples = []

        x = 0
        s = 3

        while True:
            try:
                data = serial.read(1024)
                if data:
                    #sys.stdout.write(str(data))
                    #sys.stdout.flush()
                    rx += len(data)
            except TimeoutError:
                rx = 0
                pass

            now = time.time()
            if (now - t0) > (1 / s):
                bps = round((rx*s)/1000, 3)
                samples.append(bps)
                sys.stdout.write(f'\rRX: {bps} kB/s')
                sys.stdout.flush()
                rx = 0
                t0 = now

                x += 1/s
                gui.add(x, bps)


    from threading import Thread
    Thread(target=go, args=(gui, ), daemon=True).start()

    sys.exit(app.exec())
