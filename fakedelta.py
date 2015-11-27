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

class fake_probe(Delta.Delta):
    def __init__(self, eeprom = None):
        Delta.Delta.__init__(self, eeprom = eeprom)
        self.bed_screw[0][2]=0
        self.bed_screw[1][2]=0.43
        self.bed_screw[2][2]=0.63
        self.recalc()

    def probe(self, delta = None, point = None):
        # Get the motor position of the point from the testing model
        offset = [0,0] #delta.zprobe_offset()
        point = [point[0]-offset[0], point[1]-offset[1], 0]
        motor = delta.delta_to_motor(point)
        motor[0] -= delta.endstop[0]
        motor[1] -= delta.endstop[1]
        motor[2] -= delta.endstop[2]
        # Get the actual position of the motor from the reference model
        motor[0] += self.endstop[0]
        motor[1] += self.endstop[1]
        motor[2] += self.endstop[2]
        actual = self.motor_to_delta(motor)
        print "Offset: %.3f, %.3f, %.3f" % (actual[0], actual[1], self.bed_offset(actual))
        return self.bed_offset(actual)-actual[2]

def main():
    delta = DC42Delta.DC42Delta(probe=fake_probe(eeprom = "fake.epr"))

    delta.calibrate()


if __name__ == "__main__": main()
# vim: set shiftwidth=4 expandtab: 
