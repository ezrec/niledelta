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

repetier_eeprom = {
                "Horizontal radius [mm]": 100.0,
                "Delta Radius A(0):": 0.0,
                "Delta Radius B(0):": 0.0,
                "Delta Radius C(0):": 0.0,
                "Steps per mm": 80,
                "Tower X endstop offset [steps]": 0,
                "Tower Y endstop offset [steps]": 0,
                "Tower Z endstop offset [steps]": 0,
                "Diagonal rod length [mm]": 190.0,
                "Corr. diagonal A [mm]": 0.0,
                "Corr. diagonal B [mm]": 0.0,
                "Corr. diagonal C [mm]": 0.0,
                "Alpha A(210):": 210,
                "Alpha B(330):": 330,
                "Alpha C(90):": 90,
                "Max. radius [mm]": 90,
                "Z max length [mm]": 190,
                }

class GCode:
    """GCode Sender"""
    port = None
    position = (0, 0, 0)
    z_probe = 0
    e = 0.0
    f = 4000
    eeprom = None

    def __init__(self, serial_port):
        self.port = serial_port
        self.reset()
        pass

    def reset(self):
        self.eeprom = None
        if self.port is not None:
            self.port.setDTR(0)
            time.sleep(2)
            self.port.setDTR(1)
            time.sleep(2)
            self.read("start") # Wait for start
            self.read("wait")  # Wait for wait

        pass

    def write(self, gcode):
        gcode = gcode.strip()

        if len(gcode) == 0:
            return

        self.port.write(gcode + "\n")
        self.read("ok")
        self.read("wait")
        pass

    def _parse_EPR(self, line):
        epr, rest = line.split(":",1)
        etype, epos, val, name = rest.split(" ", 3)
        self.eeprom[name] = (val, int(etype), int(epos))

    def _parse_XYZE(self, report):
        x = self.position[0]
        y = self.position[1]
        z = self.position[2]
        z_probe = self.z_probe
        e = self.e

        for axis in report.split(" "):
            av = axis.split(":")

            if av[0] == "X":
                x = float(av[1])
                continue

            if av[0] == "Y":
                y = float(av[1])
                continue

            if av[0] == "Z":
                z = float(av[1])
                continue

            if av[0] == "Z-probe":
                z_probe = float(av[1])
                continue

            if av[0] == "E":
                e = float(av[1])
                continue
            pass

        self.position = (x, y, z)
        self.e = e
        self.z_probe = z_probe

    def read(self, expected=None):
        while True:
            line = self.port.readline().strip()

            if expected is None:
                return line

            if line.startswith(expected):
                return line

            if line.startswith("EPR:"):
                self._parse_EPR(line)
                continue

            if line.startswith("X:"):
                self._parse_XYZE(line)
                continue

            if line.startswith("Z-probe:"):
                self._parse_XYZE(line)
                continue

            # Keep lookin...
            pass
        pass

    def move(self, point = None, e = None, f = None):
        """G1 move """

        if point is None and e is None and f is None:
            return

        if point is None:
            point = self.position[:]
        if e is None:
            e = self.e
        if f is None:
            f = self.f

        point = [point[0], point[1], point[2]]
        for i in range(0,3):
            if point[i] is None:
                point[i] = self.position[i]

        if self.port is None:
            self.position = point[:]
            self.e = e
            self.f = f
            return

        self.write("G1 X%.2f Y%.2f Z%.2f E%.2f F%d" % (point[0], point[1], point[2], e, f))
        pass

    def home(self):
        """G28 home """
        if self.port is None:
            self.position = 0, 0, 150
            self.e = 0
            return

        self.write("G28")
        pass

    def axis_report(self):
        """M114 axis report"""
        if self.port is None:
            return self.position + (self.e, self.f)

        self.write("M114")

        return self.position + (self.e, self.f)

    def zprobe(self, point = None, first = False, last = False):
        """G30 single-probe"""
        if self.port is None:
            probes = ( -0.98, -0.48, 0.77, 0.64, 0.26, -0.02, -0.81, -1.35, -0.93)
            if point is not None:
                z = probes[self.z_probe]
                self.z_probe += 1
            else:
                z = -1.23
            return z

        p = 0
        if first:
            p |= 1
        if last:
            p |= 2

        self.move(point)
        self.write("G30 P%d" % (p))

        return self.position[2] - self.z_probe

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
        steps_per_mm = float(self.repetier_eeprom("Steps per mm"))

        old_trim = [0, 0, 0]
        old_trim[0] = float(self.repetier_eeprom("Tower X endstop offset [steps]")) / steps_per_mm
        old_trim[1] = float(self.repetier_eeprom("Tower Y endstop offset [steps]")) / steps_per_mm
        old_trim[2] = float(self.repetier_eeprom("Tower Z endstop offset [steps]")) / steps_per_mm

        if trim is None:
            return old_trim

        self.repetier_eeprom("Tower X endstop offset [steps]", trim[0] * steps_per_mm)
        self.repetier_eeprom("Tower Y endstop offset [steps]", trim[1] * steps_per_mm)
        self.repetier_eeprom("Tower Z endstop offset [steps]", trim[2] * steps_per_mm)

        return old_trim

    # REPETIER
    def delta_radius(self, radius = None):
        dradius = None
        dcorr = [None] * 3
        if radius is not None:
            dradius = min(radius)
            for i in range(0, 3):
                dcorr[i] = radius[i] - dradius

        horiz_radius = float(self.repetier_eeprom("Horizontal radius [mm]", dradius))
        a_radius = float(self.repetier_eeprom("Delta Radius A(0):", dcorr[0])) + horiz_radius
        b_radius = float(self.repetier_eeprom("Delta Radius B(0):", dcorr[1])) + horiz_radius
        c_radius = float(self.repetier_eeprom("Delta Radius C(0):", dcorr[2])) + horiz_radius

        return [a_radius, b_radius, c_radius]

    # REPETIER
    def delta_diagonal(self, diagonal = None):
        drod = None
        dcorr = [None] * 3

        if diagonal is not None:
            drod = min(diagonal)
            for i in range(0, 3):
                dcorr[i] = diagonal[i] - drod

        rod = float(self.repetier_eeprom("Diagonal rod length [mm]", drod))
        a_rod = float(self.repetier_eeprom("Corr. diagonal A [mm]", dcorr[0])) + rod
        b_rod = float(self.repetier_eeprom("Corr. diagonal B [mm]", dcorr[1])) + rod
        c_rod = float(self.repetier_eeprom("Corr. diagonal C [mm]", dcorr[2])) + rod

        return [a_rod, b_rod, c_rod]

    # REPETIER
    def delta_angle(self, angle = None):
        dangle = [None, None, None]
        for i in range(0, 3):
            if angle is not None:
                dangle[i] = angle[i]

        a_angle = float(self.repetier_eeprom("Alpha A(210):", dangle[0]))
        b_angle = float(self.repetier_eeprom("Alpha B(330):", dangle[1]))
        c_angle = float(self.repetier_eeprom("Alpha C(90):", dangle[2]))

        return [a_angle, b_angle, c_angle]


    # REPETIER
    def delta_bed(self, radius = None, height = None):
        old_radius = float(self.repetier_eeprom("Max. radius [mm]", radius))
        old_height = float(self.repetier_eeprom("Z max length [mm]", height))

        return old_radius, old_height

    # REPETIER
    def zprobe_offset(self, offset = None):
        if self.port is None:
            return [0, 0, 0]

        if offset is None:
            offset = None, None, None

        x = float(self.repetier_eeprom("Z-probe offset x [mm]", offset[0]))
        y = float(self.repetier_eeprom("Z-probe offset y [mm]", offset[1]))
        z = float(self.repetier_eeprom("Z-probe height [mm]", offset[2]))

        return [x, y, z]

    # REPETIER
    def repetier_eeprom(self, key, value = None):
        if self.port is None:
            if self.eeprom is None:
                self.eeprom = repetier_eeprom.copy()
            val = self.eeprom[key]
            if value is not None:
                self.eeprom[key] = value
                print "EPR: %s %.3f" % (key, value)
            return val

        if self.eeprom is None:
            # Fetch the EEPROM table
            self.eeprom = {}
            self.write("M205")

        eset = self.eeprom.get(key)
        if eset is None:
            return None

        val = eset[0]
        if value is not None:
            if eset[1] == 3:
                self.write("M206 T%d P%d X%.3f" % (eset[1], eset[2], float(value)))
            else:
                self.write("M206 T%d P%d S%d" % (eset[1], eset[2], int(value)))
            self.eeprom[key] = (value, eset[1], eset[2])

        return val
