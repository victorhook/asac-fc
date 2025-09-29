#!/usr/bin/env python3
from pymavlink import mavutil
from pymavlink.mavutil import mavtcp, mavserial
from pymavlink.dialects.v20 import ardupilotmega as ap
from pymavlink.dialects.v20.ardupilotmega import MAVLink, MAVLink_message

from argparse import ArgumentParser, Namespace
import typing as t
from threading import Thread, Event
import time
from datetime import datetime
from serial.serialutil import SerialException

params = {}
t0 = time.time()
last_heartbeat = 0

def parse_args() -> Namespace:
    parser = ArgumentParser()
    parser.add_argument('-p', '--port', default='/dev/ttyACM0', help='Serial port')
    parser.add_argument('-b', '--baud', default=921600, type=int,help='Baud rate')
    parser.add_argument('-o', '--out', default=None, help='Output file to write results to')
    return parser.parse_args()


def reader(master: t.Union[mavtcp, mavserial], done_flag: Event) -> None:
    global last_heartbeat
    heatbeat_log = True

    while not done_flag.is_set():
        msg: MAVLink_message = master.recv_match(blocking=True, timeout=1)
        uptime_ms = int((time.time() - t0) * 1000)
        
        if msg:
            msg_type = msg.get_type()
            if msg_type == 'STATUSTEXT':
                print(f'[{msg.severity}] {msg.text}')
            elif msg_type == 'HEARTBEAT':
                last_heartbeat = time.time()
                if heatbeat_log:
                    print(f'{uptime_ms} HEARTBEAT')
                    heatbeat_log = False
            else:
                print(f'[{uptime_ms}] {msg_type}')
        else:
            if (time.time() - last_heartbeat) > 3:
                print('Long time since heartbeat!')
                heatbeat_log = True

            


if __name__ == '__main__':
    args = parse_args()
    port = args.port
    baud = args.baud
    out = args.out

    print(f'Trying to connect to {port}')
    master = mavutil.mavlink_connection(port, baud=baud, autoreconnect=True)
    mav: MAVLink = master.mav

    # Wait for the heartbeat from the autopilot (tells us system ID, etc.)
    print('Connected, waiting for initial heartbeat')
    master.wait_heartbeat()
    print(f'Heartbeat from system (system {master.target_system} component {master.target_component})')

    done_flag = Event()

    rx = Thread(target=reader, args=(master, done_flag), name='MAVLink reader', daemon=True)
    rx.start()

    i = 0
    while not done_flag.is_set():
        if i % 100 == 0:
            mav.heartbeat_send(0, 0, 0, 0, 0)
        i += 1
        time.sleep(.01)

    done_flag.wait()

    rx.join()
