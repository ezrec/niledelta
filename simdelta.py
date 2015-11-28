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

import DC42Delta
import Delta

class sim_probe:
    def __init__(self, filename = "plot.plt"):
        fd = open(filename, "r")
        self.points = []
        for line in fd:
           point = line.split(" ")
           self.points.append((float(point[0]), float(point[1]), float(point[2])))
        fd.close()

    def probe(self, delta = None, point = None):
        offset = delta.zprobe_offset()
        point = (point[0] - offset[0], point[1] - offset[1])
        for i in range(0, 13):
            fake = self.points[i]
            if abs(fake[0]-point[0]) < 0.01 and abs(fake[1]-point[1]) < 0.01:
                return fake[2]
        print "Probe Point not in database: ",point
        assert 0 == 1

def main():
    delta = DC42Delta.DC42Delta(probe=sim_probe(filename="plot.plt"), eeprom = "machine.epr")

    delta.calibrate()


if __name__ == "__main__": main()
# vim: set shiftwidth=4 expandtab: 
