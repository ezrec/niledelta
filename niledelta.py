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
import sys

import McCalibrate

def main(port = "/dev/ttyUSB0", baudrate = 250000):
    port = serial.Serial(port=port, baudrate=baudrate,
                         parity=serial.PARITY_NONE,
                         stopbits=serial.STOPBITS_ONE,
                         bytesize=serial.EIGHTBITS)

    delta = McCalibrate.McCalibrate(port)

    delta.calibrate()


if __name__ == "__main__": main(sys.argv[1], sys.argv[2])
# vim: set shiftwidth=4 expandtab: 
