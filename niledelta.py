#!/usr/bin/env python

# Copyright (C) 2015, Jason S. McMullan
# All right reserved.
# Author: Jason S. McMullan <jason.mcmullan@gmail.com>
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
import math
import serial
import time

import SmoothieDelta

def main():
    port = serial.Serial(port="/dev/ttyUSB0", baudrate=250000,
                         parity=serial.PARITY_NONE,
                         stopbits=serial.STOPBITS_ONE,
                         bytesize=serial.EIGHTBITS)
    gcode = GCode(port)

    delta = SmoothieDelta(gcode)

    delta.calibrate()


if __name__ == "__main__": main()
# vim: set shiftwidth=4 expandtab: 
