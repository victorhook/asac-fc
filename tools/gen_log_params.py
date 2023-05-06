#!/usr/bin/env python3

from dataclasses import dataclass
from pathlib import Path
import re
from typing import List, Union

PROJECT_BASE_DIR = Path(__file__).absolute().parent.parent
INCLUDE_DIR = PROJECT_BASE_DIR.joinpath('include')
JS_TARGET_DIR = PROJECT_BASE_DIR.joinpath('tools', 'grapher-frontend', 'src')
PYTHON_TARGET_DIR = PROJECT_BASE_DIR.parent.joinpath('telemetry-node', 'tools', 'client')
JS_TARGET_FILE = 'LogTypes.js'
PYTHON_TARGET_FILE = 'log_types.py'

JS_TARGET_FILEPATH = JS_TARGET_DIR.joinpath(JS_TARGET_FILE)
PYTHON_TARGET_FILEPATH = PYTHON_TARGET_DIR.joinpath(PYTHON_TARGET_FILE)
LOG_HEADER_FILE = 'log.h'
LOG_HEADER_PATH = INCLUDE_DIR.joinpath(LOG_HEADER_FILE)
LOG_TYPEDEF_NAME = 'log_block_control_loop_t'

C_TYPE_FORMATS = {
    'uint8_t': 'B',
    'bool': '?',
    'uint16_t': 'H',
    'uint32_t': 'I',
    'int': 'i',
    'float': 'f',
}

@dataclass
class Param:
    name: str

@dataclass
class StructParam(Param):
    type: str

@dataclass
class EnumParam(Param):
    value: str


def find_params_for(c_code: str, decl_type: str, decl_name: str) -> List[Param]:
    '''
    Finds all "parameters" in the given c code of declaration type "decl_type"
    decl_type can be any of "struct", "union", "enum" etc.

    Example: find_params_for(c_code, 'struct', 'my_struct_t')
        Expected format of c code:
            typedef struct
            {
                int x;
                int y;
            } my_struct_t;

        This will results in: [
            Param(name=x, type=int),
            Param(name=y, type=int)
        ]
    '''

    regex = 'typedef %s\s*\{(.*?)\} %s;' % (decl_type, decl_name)
    res = re.search(regex, c_code, flags=re.DOTALL)
    print(f'{decl_type} {decl_name} -> Using regex: {regex}')
    _params = res.group(1)

    params = []

    for p in filter(len, _params.splitlines()):
        p = p.strip()
        if p.startswith('//'):
            continue

        p = re.sub('\s+', ' ', p)
        split = p.split(' ')

        if decl_type.lower() == 'enum':
            # ENUM
            enum_name = split[0]
            if len(split) == 3:
                enum_value = split[2]
                if enum_value.endswith(','):
                    enum_value = enum_value[:-1]
            else:
                # No specific enum value gives, we'll deduce it
                enum_value = len(params)
            param = EnumParam(enum_name, enum_value)

        elif decl_type.lower() == 'struct':
            # STRUCT
            param_type, param_name = split
            if param_name.endswith(';'):
                param_name = param_name[:-1]

            param = StructParam(param_name, param_type)
        else:
            raise RuntimeError(f'Dont have support for type {decl_type} yet!')

        params.append(param)

    return params



if __name__ == '__main__':
    print(f'Reading log typedef from {LOG_HEADER_PATH}')

    with open(LOG_HEADER_PATH) as f:
        c_code = f.read()

    log_ctrl_loop_params = find_params_for(c_code, 'struct', 'log_block_control_loop_t')
    log_block_types = find_params_for(c_code, 'enum', 'log_type_t')

    # Write JS output
    name = 'AVAILABLE_LOG_TYPES'
    output = f'const {name} = [\n'
    for param in log_ctrl_loop_params:
        output += '    {name: "%s", type: "%s"},\n' % (param.name, param.type)
    output = output[:-2] # Remove last comma
    output += '\n]\n'
    output += f'\nexport default {name};\n'

    with open(JS_TARGET_FILEPATH, 'w') as f:
        f.write(output)

    print(f'Written log typedef to {JS_TARGET_FILEPATH}')


    # Create log_block_control_loop_t python dataclass
    struct_fmt = '<'

    output = 'from dataclasses import dataclass\n'
    output += 'import struct\n'
    output += 'from enum import IntEnum\n\n'
    output += '@dataclass\n'
    output += 'class log_block_control_loop_t:\n'

    for param in log_ctrl_loop_params:
        fmt = C_TYPE_FORMATS.get(param.type)
        if fmt is None:
            raise RuntimeError('Failed to find struct type format for '
                               f'{param.name}, type: {param.type}')

        output += f'    {param.name}: float = 0 # {param.type}\n'
        struct_fmt += fmt

    output += '\n    fmt = \'' + struct_fmt + '\'\n'
    output += f'    size = struct.calcsize(fmt)\n\n'

    # Add log_type_t python IntEnum
    output += 'class log_type_t(IntEnum):\n'
    for enum_type in log_block_types:
        output += f'    {enum_type.name} = {enum_type.value}\n'
    output += '\n'

    with open(PYTHON_TARGET_FILEPATH, 'w') as f:
        f.write(output)

    print(f'Written log typedef to {PYTHON_TARGET_FILEPATH}')

    # Write python output
