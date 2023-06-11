#!/usr/bin/env python3

import queue
import sys
import os
from dataclasses import fields

from PyQt6.QtWidgets import QApplication, QLabel, QWidget, QMainWindow, QFrame, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QScrollArea, QSizePolicy, QGridLayout, QLineEdit
from PyQt6.QtCore import QTimer, Qt, QThread
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import time

from typing import Dict
from queue import Queue
from telemetry_client import TelemetryClient


import log_types
from log_types import log_block_t


DEFAULT_IP = '192.168.4.1'
#DEFAULT_IP = '192.168.10.205'
DEFAULT_PORT = 80


class ScrollableWidget(QScrollArea):
    def __init__(self):
        super().__init__()
        self._widget = QWidget()
        layout = QGridLayout(self._widget)
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.setWidget(self._widget)
        self.setWidgetResizable(True)


class Settings(ScrollableWidget):
    
    def __init__(self) -> None:
        super().__init__()
        


class Gui(QMainWindow):

    def __init__(self) -> None:
        super().__init__()

        # Attributes
        self._params: Dict[str, QLabel] = {}
        self._log_blocks = Queue()
        self._telem_client = TelemetryClient(DEFAULT_IP, DEFAULT_PORT,
                                             self._log_blocks.put)
        self._wanted_params = [
            'raw_gyro_x',
            'raw_gyro_y',
            'raw_gyro_z',
        ]

        # Data
        self._plot_x = []
        self._plot_data = {param: [] for param in self._wanted_params}

        # -- UI -- #

        self.main = QWidget()
        self.main.setLayout(QVBoxLayout())

        self.setWindowTitle("PyQt App")
        self.setCentralWidget(self.main)
        self.setGeometry(100, 100, 1400, 800)

        self.frame_ctrl = QWidget()
        self.frame_ctrl.setLayout(QHBoxLayout())
        self.frame_main = QWidget()
        self.frame_main.setLayout(QHBoxLayout())
        self.frame_sidebar = ScrollableWidget()
        self.frame_plot = QWidget()
        self.frame_plot.setLayout(QVBoxLayout())

        # Add to main
        self.main.layout().addWidget(self.frame_ctrl)
        self.main.layout().addWidget(self.frame_main)
        # Add frames to main content frame
        self.frame_main.layout().addWidget(self.frame_sidebar)
        self.frame_main.layout().addWidget(self.frame_plot)

        # -- Control content -- #

        # Connection config
        self.con_info = QWidget()
        self.con_info.setLayout(QGridLayout())

        self.ip = QLineEdit(DEFAULT_IP)
        self.port = QLineEdit(str(DEFAULT_PORT))

        self.con_info.layout().addWidget(QLabel('IP'), 0, 0, Qt.AlignmentFlag.AlignLeft)
        self.con_info.layout().addWidget(QLabel('PORT'), 1, 0, Qt.AlignmentFlag.AlignLeft)
        self.con_info.layout().addWidget(self.ip, 0, 1, Qt.AlignmentFlag.AlignLeft)
        self.con_info.layout().addWidget(self.port, 1, 1, Qt.AlignmentFlag.AlignLeft)

        self.btn_connect = QPushButton(text='Connect')
        self.btn_disconnect = QPushButton(text='Disconnect')
        self.btn_connect.clicked.connect(self._telem_client.start)
        self.btn_disconnect.clicked.connect(self._telem_client.stop)
        self.fps = QLabel(text='')
        self.bw = QLabel(text='')

        self.frame_ctrl.layout().addWidget(self.con_info)
        self.frame_ctrl.layout().addWidget(self.btn_connect)
        self.frame_ctrl.layout().addWidget(self.btn_disconnect)
        self.frame_ctrl.layout().addWidget(self.fps)
        self.frame_ctrl.layout().addWidget(self.bw)

        # -- Sidebar content -- #
        self.frame_sidebar.setMaximumWidth(300)
        self.frame_sidebar.setSizePolicy(QSizePolicy.Policy.MinimumExpanding, QSizePolicy.Policy.Expanding)

        # Populate sidebar
        grid_layout = self.frame_sidebar._widget.layout()
        grid_layout.addWidget(QLabel(text='Parameter'), 0, 0, Qt.AlignmentFlag.AlignCenter)
        grid_layout.addWidget(QLabel(text='Value'), 0, 1, Qt.AlignmentFlag.AlignCenter)
        for row, f in enumerate(fields(log_types.log_block_data_control_loop_t)):
            param_name = QLabel(text=f.name)
            param_value = QLabel(text='0')
            grid_layout.addWidget(param_name, row+1, 0, Qt.AlignmentFlag.AlignLeft)
            grid_layout.addWidget(param_value, row+1, 1, Qt.AlignmentFlag.AlignLeft)
            self._params[f.name] = param_value


        # -- Graph content -- #
        self.graph = pg.PlotWidget()
        self.frame_plot.layout().addWidget(self.graph)
        self.graph.setBackground('w')

        colors = ['r', 'g', 'b', 'o', 'p']
        self._lines = {}
        for i, param in enumerate(self._wanted_params):
            color = colors[i]
            self.graph.addLegend()
            line = self.graph.plot(
                self._plot_x,
                self._plot_data[param],
                pen=pg.mkPen(color, width=1),
                name=param
            )
            self._lines[param] = line

        # Timer and FPS counter
        self._graphics_update_timer = QTimer()
        self._graphics_update_timer.setInterval(33)
        self._graphics_update_timer.timeout.connect(self._update_graphics)
        self._graphics_update_timer.start()
        self._fps_counter = 0
        self._fps = 0
        self._fps_t0 = 0

    def _update_graphics(self) -> None:
        last_log_block: log_block_t = None

        # Fetch all log blocks from queue
        while not self._log_blocks.empty():
            log_block: log_block_t = self._log_blocks.get()
            last_log_block = log_block

            # Append data to X axis
            self._plot_x.append(log_block.timestamp)

            # Iterate over the wanted parameters and get the data for each
            for param in self._wanted_params:
                value = getattr(log_block, param)
                self._plot_data[param].append(value)

        # Update plot data
        for param, line in self._lines.items():
            line.setData(self._plot_data[param])

        # Update sidebar data
        if last_log_block is not None:
            for param_name, param_label in self._params.items():
                value = getattr(last_log_block, param_name)
                # Format value
                if type(value) == float:
                    value = f'{value:.4f}'
                else:
                    value = str(value)
                param_label.setText(value)

        self._fps_counter += 1
        now = time.time()
        if (now - self._fps_t0) > 1:
            self._fps = self._fps_counter
            self._fps_t0 = now
            self._fps_counter = 0
            self.fps.setText(f'FPS: {self._fps}')
            bw = self._telem_client.get_bandwidth()
            self.bw.setText(f'Bandwidth: {round(bw / 1000.0, 3)} kB/s')


if __name__ == '__main__':
    app = QApplication([])
    gui = Gui()
    gui.show()
    sys.exit(app.exec())