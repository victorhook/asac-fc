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
    print('Plotter started!')
    app = QApplication([])
    gui = Gui()
    gui.show()

    def go_serial(gui, port: str = '/dev/ttyACM0'):
        # Need this weird wait of opening serial because of d1 mini...
        serial = Serial(port, baudrate=115200)
        serial.timeout = .5
        print(f'Connected to serial {serial.port}, baudrate: {serial.baudrate}')

        t0 = time.time()
        rx = 0
        samples = []

        x = 0
        s = 3

        while True:
            try:
                data = serial.readline()
                if data:
                    var = int(str(data, encoding='ascii').split(':')[-1].strip())
                    #var = int(str(data).split(':')[-1].strip())
                    #sys.stdout.write(str(data))
                    #sys.stdout.flush()
                    now = time.time()
                    x = now - t0
                    gui.add(x, var)
            except TimeoutError:
                pass


    from threading import Thread
    Thread(target=go_serial, args=(gui, ), daemon=True).start()

    sys.exit(app.exec())
