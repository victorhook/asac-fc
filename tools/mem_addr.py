from argparse import ArgumentParser, Namespace
import os
from pathlib import Path
from typing import List
import re
from dataclasses import dataclass, asdict
import logging
import json


logger = logging.getLogger(__name__)


FILE_EXTENSIONS_TO_SEARCH = [
    '.h',
    '.c'
]


@dataclass
class MemParam:
    name: str
    dtype: str
    initial_value: str = None
    address: int = None


def get_param_strings_from_file(filepath: str) -> List[str]:
    with open(filepath) as f:
        data = f.read()

    params = []

    for match in re.finditer('(.*)@mem', data):
        decl = match.group(1).split(';')[0]
        params.append(decl)

    return params


def parse_param_from_string(string: str) -> MemParam:
    string = re.sub('\s+', ' ', string)
    split = string.split(' ')
    dtype = split[0]
    name = split[1]

    if split[2] == '=':
        initial_value = split[3]
    else:
        initial_value = None

    return MemParam(name, dtype, initial_value)


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


if __name__ == '__main__':
    args = parse_args()

    logging.basicConfig(level=logging.DEBUG)


    param_strings = []

    source_dirs = args.source_dir.split(' ')
    for src in source_dirs:
        for dirpath, dirnames, filenames in os.walk(src):
            for filename in filenames:
                abspath = Path(dirpath).joinpath(filename)
                more_param_strings = get_param_strings_from_file(abspath)
                if more_param_strings:
                    param_strings.extend(more_param_strings)


    logger.debug(f'Found {len(param_strings)} param strings:')
    for param_string in param_strings:
        logger.debug(param_string)

    for param_string in param_strings:
        mem_param = parse_param_from_string(param_string)
        mem_params.append(mem_param)


    for mem_param in mem_params:
        print(mem_param)


    # Read addresses in map file
    with open(args.map_file) as f:
        map_data = f.read()

    for mem_param in mem_params:
        addr = re.search(f'0x(.*)\s*{mem_param.name}', map_data)
        if addr:
            addr = int(addr.group(1), 16)
            mem_param.address = addr

    # Dump output
    mem_params_json = [asdict(mem_param) for mem_param in mem_params]
    logger.debug(f'Writing output result to: "{args.output}"')
    with open(args.output, 'w') as f:
        json.dump(mem_params_json, f, indent=4)
