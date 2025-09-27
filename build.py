#!/usr/bin/env python3
from argparse import ArgumentParser, Namespace
import os
import sys
from pathlib import Path

def parse_args() -> Namespace:
    parser = ArgumentParser()
    parser.add_argument('board', help='Board to build')
    parser.add_argument('-c', '--clean', action='store_true', help='Cleans before building')
    return parser.parse_args()


ROOT_DIR = Path(__file__).absolute().parent
BUILD_DIR = str(ROOT_DIR / 'build')

AVAILABLE_BOARDS = [
    'rp2040',
    'sitl'
]

if __name__ == '__main__':
    args = parse_args()
    board = args.board
    clean = args.clean

    if clean:
        print('Cleaning build dir')
        os.system(f'rm -rf {BUILD_DIR}/*')

    if board not in AVAILABLE_BOARDS:
        print(f'Board not valid, {board} must be one of: [{", ".join(AVAILABLE_BOARDS)}]')
        sys.exit(0)

    w = 80
    print('*' * w)
    print(f'Building for board {board}')

    hal_flag = f'-DHAL={board.upper()}'
    os.system(f'cd build && cmake {hal_flag} .. && make -j$(nproc)')

    print('Done')
    print('*' * w)