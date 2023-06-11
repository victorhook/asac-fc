from dataclasses import dataclass, fields
import struct
from enum import IntEnum

@dataclass
class log_block_t:
    pass

class log_type_t(IntEnum):
    LOG_TYPE_PID = 0
    LOG_TYPE_BATTERY = 1

@dataclass
class log_block_header_t(log_block_t):
    type: float = 0 # log_type_t
    timestamp: float = 0 # uint32_t
    id: float = 0 # uint32_t

    fmt = '<BII'
    size = struct.calcsize(fmt)

    def to_bytes(self) -> bytes:
        """ Returns a log_block_header_t in bytes. """
        fmt = self.fmt
        raw = struct.pack(fmt, *[getattr(self, f.name) for f in fields(self)])
        return raw

@dataclass
class log_block_data_control_loop_t(log_block_header_t):
    raw_gyro_x: float = 0 # float
    raw_gyro_y: float = 0 # float
    raw_gyro_z: float = 0 # float
    filtered_gyro_x: float = 0 # float
    filtered_gyro_y: float = 0 # float
    filtered_gyro_z: float = 0 # float
    rc_in_roll: float = 0 # uint16_t
    rc_in_pitch: float = 0 # uint16_t
    rc_in_yaw: float = 0 # uint16_t
    rc_in_throttle: float = 0 # uint16_t
    setpoint_roll: float = 0 # float
    setpoint_pitch: float = 0 # float
    setpoint_yaw: float = 0 # float
    setpoint_throttle: float = 0 # float
    is_connected: float = 0 # bool
    is_armed: float = 0 # bool
    can_run_motors: float = 0 # bool
    roll_error: float = 0 # float
    roll_error_integral: float = 0 # float
    roll_p: float = 0 # float
    roll_i: float = 0 # float
    roll_d: float = 0 # float
    roll_pid: float = 0 # float
    pitch_error: float = 0 # float
    pitch_error_integral: float = 0 # float
    pitch_p: float = 0 # float
    pitch_i: float = 0 # float
    pitch_d: float = 0 # float
    pitch_pid: float = 0 # float
    yaw_error: float = 0 # float
    yaw_error_integral: float = 0 # float
    yaw_p: float = 0 # float
    yaw_i: float = 0 # float
    yaw_d: float = 0 # float
    yaw_pid: float = 0 # float
    m1_non_restricted: float = 0 # float
    m2_non_restricted: float = 0 # float
    m3_non_restricted: float = 0 # float
    m4_non_restricted: float = 0 # float
    m1_restricted: float = 0 # float
    m2_restricted: float = 0 # float
    m3_restricted: float = 0 # float
    m4_restricted: float = 0 # float
    battery: float = 0 # float
    successful_packets: float = 0 # uint32_t
    parse_errors: float = 0 # uint32_t
    last_received_packet: float = 0 # uint32_t
    packet_rate: float = 0 # uint32_t

    fmt = '<ffffffHHHHffff???fffffffffffffffffffffffffffIIII'
    size = struct.calcsize(fmt)

    def to_bytes(self) -> bytes:
        """ Returns a log_block_data_control_loop_t in bytes. """
        fmt = self.fmt
        fmt = super().fmt + fmt.replace("<", "")
        raw = struct.pack(fmt, *[getattr(self, f.name) for f in fields(self)])
        return raw

@dataclass
class log_block_data_battery_t(log_block_header_t):
    voltage: float = 0 # float

    fmt = '<f'
    size = struct.calcsize(fmt)

    def to_bytes(self) -> bytes:
        """ Returns a log_block_data_battery_t in bytes. """
        fmt = self.fmt
        fmt = super().fmt + fmt.replace("<", "")
        raw = struct.pack(fmt, *[getattr(self, f.name) for f in fields(self)])
        return raw

