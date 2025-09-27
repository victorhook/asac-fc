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


def parse_args() -> Namespace:
    parser = ArgumentParser()
    parser.add_argument('-p', '--port', required=True, help='Serial port')
    parser.add_argument('-b', '--baud', help='Baud rate')
    parser.add_argument('-o', '--out', default=None, help='Output file to write results to')
    return parser.parse_args()


def reader(master: t.Union[mavtcp, mavserial], done_flag: Event) -> None:
    while not done_flag.is_set():
        msg: MAVLink_message = master.recv_match(blocking=True)
        
        if msg:
            msg_type = msg.get_type()
            if msg_type == 'PARAM_VALUE':
                params[msg.param_id] = msg.param_value
                if msg.param_index == (msg.param_count - 1):
                    done_flag.set()


if __name__ == '__main__':
    args = parse_args()
    port = args.port
    baud = args.baud
    out = args.out

    print(f'Trying to connect to {port}')
    master = mavutil.mavlink_connection(port, baud=baud)
    mav: MAVLink = master.mav

    # Wait for the heartbeat from the autopilot (tells us system ID, etc.)
    print('Connected, waiting for initial heartbeat')
    master.wait_heartbeat()
    print(f'Heartbeat from system (system {master.target_system} component {master.target_component})')

    done_flag = Event()

    rx = Thread(target=reader, args=(master, done_flag), name='MAVLink reader', daemon=True)
    rx.start()
    
    # Send a param request
    mav.param_request_list_send(master.target_system, master.target_component)

    i = 0
    while not done_flag.is_set():
        if i % 100 == 0:
            mav.heartbeat_send(0, 0, 0, 0, 0)
        i += 1
        time.sleep(.01)

    done_flag.wait()

    rx.join()

    out_str = ''
    for name, value in params.items():
        out_str += f'{name} {value}\n'

    if out:
        print(f'Writing results to {out}')
        timestamp_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        with open(out, 'w') as f:
            f.write(f'# {timestamp_str}\n')
            f.write(out_str)
    else:
        print(out_str)

