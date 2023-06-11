from argparse import ArgumentParser, Namespace
import os
from pathlib import Path
from typing import List, Tuple
import re
from dataclasses import dataclass, asdict
import logging
import json


logger = logging.getLogger(__name__)


FILE_EXTENSIONS_TO_SEARCH = [
    '.h',
    '.c'
]

MEASUREMENT = 'meas'
CALIBRATION = 'cal'


@dataclass
class MemParam:
    name: str
    dtype: str
    initial_value: str
    address: int
    type: str # [Measurement, Calibration]
    line_number: int
    source_file: str
    children: List[str]

def parse_param_from_string(string: str, type: str) -> MemParam:
    string = re.sub('\s+', ' ', string)
    split = string.split(' ')
    dtype = split[0]
    name = split[1]

    if split[2] == '=':
        initial_value = split[3]
    else:
        initial_value = None

    return MemParam(name, dtype, initial_value, 0, type, 0, '', [])


def get_param_strings_from_file(filepath: Path) -> List[MemParam]:
    with open(filepath) as f:
        data = f.read()

    params = []

    def get_param_with_regex(line: str, regex: str) -> MemParam:
        # Search for measurement parameters
        match = re.search(f'(.*)@{regex}', line)
        if match:
            decl = match.group(1).split(';')[0]
            param = parse_param_from_string(decl, regex)
            return param
        else:
            return None

    for line_nbr, line in enumerate(data.splitlines()):
        for param_type in [MEASUREMENT, CALIBRATION]:
            param = get_param_with_regex(line, param_type)
            if param is not None:
                param.line_number = line_nbr + 1
                param.source_file = str(filepath.absolute())
                params.append(param)

    return params

def lookup_memory_addresses(mem_params: List[MemParam]) -> List[MemParam]:
    for mem_param in mem_params:
        addr = re.search(f'0x(.*)\s*{mem_param.name}', map_data)
        if addr:
            addr = int(addr.group(1), 16)
            mem_param.address = addr
    return mem_params


def parse_args() -> Namespace:
    parser = ArgumentParser()
    parser.add_argument('-m', '--map_file', type=str, required=True,
                        help='Filepath to memory file')
    parser.add_argument('-s', '--source_dir', type=str, required=True,
                        help='Directory of where to search for source code')
    parser.add_argument('-o', '--output', type=str, required=True,
                        help='Output directory to write results to')
    return parser.parse_args()


mem_params: List[MemParam] = []

def get_filepaths_to_search(source_dirs: List[str]) -> List[Path]:
    filepaths = []
    for src in source_dirs:
        for dirpath, dirnames, filenames in os.walk(src):
            for filename in filenames:
                abspath = Path(dirpath).joinpath(filename)
                filepaths.append(abspath)
    return filepaths


def calc_param_size(param: MemParam) -> int:
    pass


PARAM_TYPES = {
    'int': 4,
    'float': 4
}


@dataclass
class ParamTypeDef:
    pass


def is_primitive_type(param_type: str) -> bool:
    return param_type in PARAM_TYPES


def find_param_typedef_in_sourcefiles(source_files: List[str], param_type: str) -> ParamTypeDef:
    for src_file in source_files:
        with open(src_file) as f:
            src_code = f.read()
            print('typedef\s+struct\s*\{(.*?)}(.*?)%s' % param_type)
            #m = re.search('typedef\s+struct\s*\{(.*?)}(.*?)%s' % param_type, src_code, flags=re.DOTALL)
            #if m:
            #    print('HIT', m.group())

def build_param_children(filepaths: List[str], param: MemParam) -> int:

    if is_primitive_type(param.dtype):
        return

    find_param_typedef_in_sourcefiles(filepaths, param.dtype)


if __name__ == '__main__':
    args = parse_args()

    logging.basicConfig(level=logging.DEBUG)

    source_dirs = args.source_dir.split(' ')
    filepaths = get_filepaths_to_search(source_dirs)

    print(f'Found {len(filepaths)} files to search')
    print('\t', ' '.join(str(path) for path in filepaths))

    mem_params = []

    for filepath in filepaths:
        params = get_param_strings_from_file(filepath)
        mem_params.extend(params)

    print(f'Found {len(mem_params)} parameters')
    for param in mem_params:
        print('\t', f'{param.name}: {param.source_file}:{param.line_number}')
        build_param_children(filepaths, param)

    # Evaluate types
    #for param in mem_params:
    #    if is_primitive_type(param):
    #        continue
    #    for filepath in filepaths:
            

    # Read addresses in map file
    with open(args.map_file) as f:
        map_data = f.read()

    # Use the params we just found in source code and use map-file to find the correct addresses.
    mem_params = lookup_memory_addresses(mem_params)

    # Dump output
    mem_params_json = [asdict(mem_param) for mem_param in mem_params]
    print(f'Writing output result to: "{args.output}"')
    with open(args.output, 'w') as f:
        json.dump(mem_params_json, f, indent=4)
