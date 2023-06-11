from log_types import log_block_data_control_loop_t, log_block_header_t, log_type_t


DESIRED_LOG_PARAMS = [
    'roll_error',
    'roll_error_integral',
    'roll_p',
    'roll_i',
    'roll_d',
    'roll_pid',
    #'raw_gyro_x',
    #'raw_gyro_y',
    #'raw_gyro_z',
    #'successful_packets',
    #'parse_errors',
    #'last_received_packet',
    #'packet_rate'
    #'filtered_gyro_x',
    #'filtered_gyro_y',
    #'filtered_gyro_z',
]

DESIRED_LOG_PARAMS2 = [
    'rc_in_roll',
    'rc_in_pitch',
    'rc_in_yaw',
    'rc_in_throttle',
]


class TelemetryClientLogger:

    def log(self, log_block: log_block_data_control_loop_t) -> None:
        print(f'[{log_block.id}] ', end='')
        for param in DESIRED_LOG_PARAMS:
            value = getattr(log_block, param)
            value_fmt = '{:+0.3f}'.format(value)
            print(f'{param}: {value_fmt}', end=' ')
        print()
