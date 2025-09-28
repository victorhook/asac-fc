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

params = {}
t0 = time.time()


def parse_args() -> Namespace:
    parser = ArgumentParser()
    parser.add_argument('port', default='/dev/ttyACM0', help='Serial port')
    parser.add_argument('-b', '--baud', default=921600, type=int,help='Baud rate')
    parser.add_argument('-o', '--out', default=None, help='Output file to write results to')
    return parser.parse_args()


def reader(master: t.Union[mavtcp, mavserial], done_flag: Event) -> None:
    while not done_flag.is_set():
        msg: MAVLink_message = master.recv_match(blocking=True)
        
        if msg:
            msg_type = msg.get_type()
            uptime_ms = int((time.time() - t0) * 1000)
            print(f'[{uptime_ms}] {msg_type}')


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
