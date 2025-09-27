#!/usr/bin/env python3
from pathlib import Path
from dataclasses import dataclass
import re

ROOT_DIR = Path(__file__).absolute().parent.parent
PARAMS_FILE = ROOT_DIR / 'src/param/param.c'
OUTPUT_FILE = ROOT_DIR / 'docs/params.yaml'

@dataclass
class Param:
    mavlink_name: str
    code_name: str
    default_value: float = None
    description: str = None


class ParamParser:

    def __init__(self) -> None:
        self.state = 0

    def parse_line(self, line: str) -> Param:
        line = line.strip()
        if not line or line.startswith('/*') or line.startswith('//'):
            return

        if line.startswith('mav_param_t mav_params'):
            self.state = 1
            return
        
        if self.state == 1:
            if line.startswith('};'):
                self.state = 0
                return

            # Extract mavlink param name, C variable name, and (optional) description
            match = re.search(r'{"(.*?)",\s*&(.*?)}\s*,\s*(//.*)?', line)
            desc = match.group(3)
            if desc:
                desc = desc.split('//')[-1].strip()
            else:
                desc = ''

            return Param(
                mavlink_name=match.group(1).strip(),
                code_name=match.group(2).strip(),
                description=desc,
            )
            


if __name__ == '__main__':
    params: list[Param] = []
    parser = ParamParser()

    with open(PARAMS_FILE) as f:
        lines = f.readlines() 

        for line in lines:
            param = parser.parse_line(line)
            if param:
                params.append(param)

        for line in lines:
            for param in params:
                match = re.search(f'{param.code_name}\s*=\s*(.*?);', line)
                if match:
                    param.default_value = match.group(1)
                
    print(f'Analyzed {len(params)} parameters, writing results to {OUTPUT_FILE}')
    with open(OUTPUT_FILE, 'w') as f:
        for param in params:
            f.write(f'- name: {param.mavlink_name}\n')
            f.write(f'  default: {param.default_value}\n')
            f.write(f'  description: {param.description}\n')
            f.write('\n')