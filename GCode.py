#
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
import serial
import time

class GCode:
    """GCode Sender"""
    port = None
    x = None
    y = None
    e = None
    z = None
    f = 4000
    eeprom = None

    def __init__(self, serial_port):
        self.port = serial_port
        self.reset()
        pass

    def reset(self):
        if self.port is not None:
            self.port.setDTR(0)
            time.sleep(1)
            self.port.setDTR(1)
            self.read("start") # Wait for start

        self.home()
        pass

    def write(self, gcode):
        gcode = gcode.strip()

        if len(gcode) == 0:
            return

        self.ser.write(gcode + "\n")
        self.read("ok")
        pass

    def read(self, expected=None):
        if self.port is None:
            return self.fake_response

        response = []
        while True:
            line = self.port.readline().strip()
            if expected is None:
                return line

            response.append(line)

            if expected.lower() in line.lower():
                return response

            # Keep lookin...
            pass
        pass

    def move(self, point = None, e = None, f = None):
        """G1 move """
        if point is None:
            point = self.position
        if e is None:
            e = self.e
        if f is None:
            f = self.f

        if self.port is None:
            self.position = point
            self.e = e
            self.f = f
            return

        self.write("G1 X%.2f Y%.2f Z%.2f E%.2f F%d" % (x, y, z, e, f))
        self.read("ok")
        pass

    def home(self):
        """G28 home """
        if self.port is None:
            self.position = 0, 0, 150
            self.e = 0
            return

        self.write("G28")
        self.read("ok")

        self.axis_report()
        pass

    def _parse_axis_report(self, report):
        x, y, z, e = self.x, self.y, self.z, self.e
        for axis in report.split(" "):
            av = axis.split(":")

            if av[0].lower() is "x":
                x = float(av[1])
                continue

            if av[0].lower() is "y":
                y = float(av[1])
                continue

            if av[0].lower() is "z":
                z = float(av[1])
                continue

            if av[0].lower() is "e":
                e = float(av[1])
                continue
            pass

        return x, y, z, e

    def axis_report(self):
        """M114 axis report"""
        if self.port is None:
            return self.position + (self.e, self.f)

        self.write("M114")
        pos = self._parse_axis_report(self.read("ok")[0])

        self.position = pos[0:3]
        self.e = pos[3]
        return self.position + (self.e, self.f)

    def zprobe(self):
        """G30 single-probe"""
        if self.port is None:
            if self.position[0] == 0 and self.position[1] == 0:
                return 0.5
            else:
                return 0.1

        self.write("G30")
        pos = self._parse_axis_report(self.read("ok")[0])
        return pos[2]

    # REPETIER
    def endstop_trim_clear(self):
        """G131 Remove endstop offsets"""
        if self.port is None:
            return

        self.write("G131")
        self.read("ok")
        pass

    # REPETIER
    def endstop_trim(self, trim = None):
        if self.port is None:
            return [1.1, 2.2, 3.3]

        steps_per_mm = int(self.repetier_eeprom("Steps per mm"))

        old_trim = [0, 0, 0]
        old_trim[0] = self.repetier_eeprom("Tower X endstop offset") / float(steps_per_mm)
        old_trim[1] = self.repetier_eeprom("Tower Y endstop offset") / float(steps_per_mm)
        old_trim[2] = self.repetier_eeprom("Tower Z endstop offset") / float(steps_per_mm)

        if trim is None:
            return old_trim

        self.repetier_eeprom("Tower X endstop offset", trim[0] * steps_per_mm)
        self.repetier_eeprom("Tower Y endstop offset", trim[1] * steps_per_mm)
        self.repetier_eeprom("Tower Z endstop offset", trim[2] * steps_per_mm)

        return old_trim

    # REPETIER
    def delta_radius(self, radius = None):
        if self.port is None:
            return [100.0, 100.0, 100.0]

        dradius = None
        dcorr = (None, None, None)
        if radius is not None:
            dradius = min(radius)
            for i in range(0, 3):
                dcorr[i] = radius[i] - dradius

        horiz_radius = float(self.repetier_eeprom("Horizontal radius", dradius))
        a_radius = float(self.repetier_eeprom("Delta Radius A(0)", dcorr[0])) + horiz_radius
        b_radius = float(self.repetier_eeprom("Delta Radius B(0)", dcorr[1])) + horiz_radius
        c_radius = float(self.repetier_eeprom("Delta Radius C(0)", dcorr[2])) + horiz_radius

        return [a_radius, b_radius, c_radius]

    # REPETIER
    def delta_diagonal(self, diagonal = None):
        if self.port is None:
            return [190.0, 190.0, 190.0]

        drod = None
        dcorr = (None, None, None)

        if diagonal is not None:
            drod = min(diagonal)
            for i in range(0, 3):
                dcorr[i] = diagonal[i] - drod

        rod = float(self.repetier_eeprom("Diagonal rod length", drod))
        a_rod = float(self.repetier_eeprom("Corr. diagonal A", dcorr[0])) + rod
        b_rod = float(self.repetier_eeprom("Corr. diagonal B", dcorr[1])) + rod
        c_rod = float(self.repetier_eeprom("Corr. diagonal C", dcorr[2])) + rod

        return [a_rod, b_rod, c_rod]

    # REPETIER
    def delta_angle(self, angle = None):
        if self.port is None:
            return [210, 330, 90]

        dangle = (None, None, None)
        for i in range(0, 3):
            if angle is not None:
                dtower[i] = angle[i]


        a_angle = float(self.repetier_eeprom("Corr. diagonal A", dangle[0]))
        b_angle = float(self.repetier_eeprom("Corr. diagonal B", dangle[1]))
        c_angle = float(self.repetier_eeprom("Corr. diagonal C", dangle[2]))

        return [a_angle, b_angle, c_angle]


    # REPETIER
    def delta_bed(self, bed_radius = None):
        if self.port is None:
            return 90.0

        return float(self.repetier_eeprom("Max. radius", bed_radius))

    # REPETIER
    def zprobe_offset(self, offset = None):
        if self.port is None:
            return [0, 0, 0]

        if offset is None:
            offset = None, None, None

        x = float(self.repetier_eeprom("Z-probe offset x", offset[0]))
        y = float(self.repetier_eeprom("Z-probe offset y", offset[1]))
        z = float(self.repetier_eeprom("Z-probe height", offset[2]))

        return [x, y, z]

    # REPETIER
    def repetier_eeprom(self, key, value = None):
        if self.eeprom is None:
            # Fetch the EEPROM table
            self.eeprom = {}
            self.gcode.write("M205")
            for line in self.gcode.read("ok"):
                if not "EPR:" in line:
                    continue
                epr, rest = line.split(":",2)
                etype, epos, val, name = rest.split(" ", 4)
                self.eeprom[name] = (val, etype, epos)
                pass
            pass
        eset = self.eeprom.get(key)
        if eset is None:
            return None

        val = eset[0]
        if value is not None:
            if eset[1] == 3:
                self.gcode.write("M206 T%d P%d X%.3f" % (eset[1], eset[2], float(value)))
            else:
                self.gcode.write("M206 T%d P%d S%d" % (eset[1], eset[2], int(value)))
            self.eeprom[key] = (value, eset[1], eset[2])

        return val
