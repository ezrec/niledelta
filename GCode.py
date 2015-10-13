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
    z = None
    f = 4000
    eeprom = None

    def __init__(self, serial_port):
        self.port = serial_port
        self.reset()
        pass

    def reset(self):
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

    def move(self, x = None, y = None, z = None, e = None, f = None):
        """G1 move """
        if x is None:
            x = self.x
        if y is None:
            y = self.y
        if z is None:
            z = self.z
        if e is None:
            e = self.e
        if f is None:
            f = self.f

        self.write("G1 X%.2f Y%.2f Z%.2f E%.2f F%d" % (x, y, z, e, f))
        self.read("ok")
        pass

    def home(self):
        """G28 home """
        self.write("G28")
        self.read("ok")

        self.report_axes()
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
        self.write("M114")
        self.x, self.y, self.z, self.e = self._parse_axis_report(self.read("ok")[0])

    def zprobe(self):
        """G30 single-probe"""
        self.write("G30")
        x, y, z, e = self._parse_axis_report(self.read("ok")[0])
        return z

    # REPETIER
    def endstop_trim_clear(self):
        """G131 Remove endstop offsets"""
        self.write("G131")
        self.read("ok")
        pass

    # REPETIER
    def endstop_trim(self, x_trim, y_trim, z_trim):
        steps_per_mm = int(self.repetier_eeprom("Steps per mm"))
        self.repetier_eeprom("Tower X endstop offset", x_trim * steps_per_mm)
        self.repetier_eeprom("Tower Y endstop offset", y_trim * steps_per_mm)
        self.repetier_eeprom("Tower Z endstop offset", z_trim * steps_per_mm)
        pass

    # REPETIER
    def delta_radius(self, value = None):
        return float(self.repetier_eeprom("Horizontal radius", value))

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
