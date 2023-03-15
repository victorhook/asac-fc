#!/bin/bash

cd build && make -j4 && cd .. && openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "program build/rc-car.elf verify reset exit"