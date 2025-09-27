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


def parse_args() -> Namespace:
    parser = ArgumentParser()
    parser.add_argument('-p', '--port', required=True, help='Serial port')
    parser.add_argument('-b', '--baud', help='Baud rate')
    parser.add_argument('file', help='Parameter file to load from')
    return parser.parse_args()


def reader(master: t.Union[mavtcp, mavserial], params: dict, done_flag: Event) -> None:
    while not done_flag.is_set():
        msg: MAVLink_message = master.recv_match(blocking=True)
        
        if msg:
            msg_type = msg.get_type()
            if msg_type == 'PARAM_VALUE':
                desired_param_value = params.get(msg.param_id)
                
                if desired_param_value == msg.param_value:
                    params.pop(msg.param_id)
                else:
                    print(f'Wrong parameter value for {msg.param_id}, got {msg.param_value}, expected {desired_param_value}') 

                if len(params) == 0:
                    done_flag.set()


if __name__ == '__main__':
    args = parse_args()
    port = args.port
    baud = args.baud
    file = args.file

    params = {}
    with open(file) as f:
        for line_nbr, line in enumerate(f.readlines()):
            split = list(filter(len, line.split('#')[0].split(' ')))
            if not split:
                continue
            try:
                name, value = split
                params[name] = float(value.strip())
            except Exception as e:
                print(f'Failed to parse parameter on line {line_nbr} ({line}): {e}')

    print(f'Found {len(params)} parameters in {file}')

    print(f'Trying to connect to {port}')
    master = mavutil.mavlink_connection(port, baud=baud)
    mav: MAVLink = master.mav

    # Wait for the heartbeat from the autopilot (tells us system ID, etc.)
    print('Connected, waiting for initial heartbeat')
    master.wait_heartbeat()
    print(f'Heartbeat from system (system {master.target_system} component {master.target_component})')

    done_flag = Event()
    rx = Thread(target=reader, args=(master, params.copy(), done_flag), name='MAVLink reader', daemon=True)
    rx.start()

    for name, value in params.items():
        mav.param_set_send(master.target_system, master.target_component, name.encode('ascii'), value, ap.MAV_PARAM_TYPE_REAL32)
    
    done_flag.wait()

    rx.join()

    print(f'Parameters set successful')
