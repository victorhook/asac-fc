#!/usr/bin/env python3

from dataclasses import dataclass
from pathlib import Path
import re
from typing import List, Union
from datetime import datetime

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

AUTO_GENERATED_FILE_COMMENT = '''\
Auto-generated file. Please don\'t modify directly!
Generated: {}
'''

C_TYPE_FORMATS = {
    'log_type_t': 'B',
    'uint8_t': 'B',
    'bool': '?',
    'uint16_t': 'H',
    'uint32_t': 'I',
    'uint64_t': 'Q',
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


@dataclass
class Typedef:
    type: str
    name: str
    params: List[Param]


def find_all_typedefs(c_code: str) -> None:
    typedefs = []
    current_typedef = None
    current_params = []
    prev_line = ''

    for i, line in enumerate(c_code.splitlines()):
        line = line.strip()
        line = re.sub('\s+', ' ', line)
        if not line or line.startswith('//') or line.startswith('{'):
            continue

        split = line.split(' ')

        #print(f'LINE: {line}, TYPEDEF IS: {current_typedef}')

        if current_typedef is None:
            if 'typedef' in line:
                typedef_type = split[1]
                #print('NEW TYPEDEF', typedef_type)
                current_typedef = typedef_type.strip()
        else:
            res = re.search('\s*\}.*?\s(.*?);', line)
            if res:
                # Check if we've reached end of typedef
                typedef_name = res.group(1)
                typedefs.append(Typedef(
                    current_typedef,
                    typedef_name,
                    current_params
                ))
                current_typedef = None
                current_params = []
            else:
                # Parse params
                if current_typedef == 'enum':
                    # ENUM
                    enum_name = split[0]
                    if len(split) == 3:
                        enum_value = split[2]
                        if enum_value.endswith(','):
                            enum_value = enum_value[:-1]
                    else:
                        # No specific enum value gives, we'll deduce it
                        enum_value = len(current_params)
                    param = EnumParam(enum_name, enum_value)
                elif current_typedef == 'struct':
                    # STRUCT
                    #print(prev_line, '|', line, '> ', split)
                    param_type, param_name = split
                    if param_name.endswith(';'):
                        param_name = param_name[:-1]

                    param = StructParam(param_name, param_type)
                else:
                    print(f'Dont have support for type {current_typedef} yet!')
                    continue

                current_params.append(param)


    return typedefs

def now() -> str:
    return datetime.now().strftime('%Y-%m-%d %H:%M:%S')


def write_js_output(typedefs: List[Typedef], output_path: Path) -> None:
    header = AUTO_GENERATED_FILE_COMMENT.format(now())
    output = f'/*\n{header}*/\n\n'

    for typedef in typedefs:
        if typedef.name == 'log_block_data_control_loop_t':
            name = 'AVAILABLE_LOG_TYPES'
            output += f'const {name} = [\n'
            for param in typedef.params:
                output += '    {name: "%s", type: "%s"},\n' % (param.name, param.type)
            output = output[:-2] # Remove last comma
            output += '\n]\n'
            output += f'\nexport default {name};\n'

    with open(output_path, 'w') as f:
        f.write(output)

    print(f'Written log typedef to {output_path}')


def write_python_output(typedefs: List[Typedef], output_path: Path) -> None:
    header = AUTO_GENERATED_FILE_COMMENT.format(now())
    output = f'"""\n{header}"""\n\n'

    output = 'from dataclasses import dataclass, fields\n'
    output += 'import struct\n'
    output += 'from enum import IntEnum\n\n'

    output += '@dataclass\n'
    output += 'class log_block_t:\n'
    output += '    pass\n\n'

    def format_struct(typedef: Typedef, parent: str = None) -> str:
        struct_fmt = '<'
        output = ''
        output += '@dataclass\n'
        if parent is not None:
            output += f'class {typedef.name}({parent}):\n'
        else:
            output += f'class {typedef.name}:\n'

        for param in typedef.params:
            fmt = C_TYPE_FORMATS.get(param.type)
            if fmt is None:
                raise RuntimeError('Failed to find struct type format for '
                                  f'{param.name}, type: {param.type}')

            output += f'    {param.name}: float = 0 # {param.type}\n'
            struct_fmt += fmt

        output += '\n    fmt = \'' + struct_fmt + '\'\n'
        output += f'    size = struct.calcsize(fmt)\n\n'

        # Add to_bytes() function
        output += '    def to_bytes(self) -> bytes:\n'
        output += f'        """ Returns a {typedef.name} in bytes. """\n'
        output += f'        fmt = self.fmt\n'
        if typedef.name != 'log_block_header_t':
            output += '        fmt = super().fmt + fmt.replace("<", "")\n'
        output += '        raw = struct.pack(fmt, *[getattr(self, f.name) for f in fields(self)])\n'
        output += '        return raw\n\n'

        return output

    def format_enum(typedef: Typedef) -> str:
        output = ''
        output += f'class {typedef.name}(IntEnum):\n'
        for enum_type in typedef.params:
            output += f'    {enum_type.name} = {enum_type.value}\n'
        output += '\n'
        return output

    for typedef in typedefs:
        if typedef.type == 'struct':
            # All log blocks that are not header should inherit header.
            if typedef.name.startswith('log_block_data'):
                output += format_struct(typedef, parent='log_block_header_t')
            elif typedef.name.startswith('log_block_header'):
                output += format_struct(typedef, parent='log_block_t')
            else:
                output += format_struct(typedef)
        elif typedef.type == 'enum':
            output += format_enum(typedef)

    with open(PYTHON_TARGET_FILEPATH, 'w') as f:
        f.write(output)

    print(f'Written log typedef to {PYTHON_TARGET_FILEPATH}')



if __name__ == '__main__':
    print(f'Reading log typedef from {LOG_HEADER_PATH}')

    with open(LOG_HEADER_PATH) as f:
        c_code = f.read()

    typedefs = find_all_typedefs(c_code)
    print(f'Found {len(typedefs)} typedefs!')

    write_js_output(typedefs, JS_TARGET_FILEPATH)
    write_python_output(typedefs, PYTHON_TARGET_FILEPATH)
