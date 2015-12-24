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
# Implementation of the DC42 delta calibration technique.
#
# Derived from:
#  https://github.com/dc42/RepRapFirmware/blob/dev/DeltaProbe.cpp
#
import math
import serial
import time
import GCode
import Delta

class McCalibrate(Delta.Delta):
    """ McCalibrate  """

    # Solve for the following factors:
    #  Endstop A
    #  Endstop B
    #  Radius Overall
    #  Diagonal Overall
    #  Radius A Offset
    #  Radius B Offset
    #  Diagonal A Offset
    #  Diagonal B Offset
    #  Bed Level Screw A (Diagonal from A)
    #  Bed Level Screw B (Diagnoal from B)
    #
    #  Note that Endstop C, Bed Screw C, and Angle C are held constant.
    #
    numFactors = 1
    numPoints = 13

    def __init__(self, port = None, probe = None, eeprom = None):
        Delta.Delta.__init__(self, port = port, probe = probe, eeprom = eeprom)

    def _apply_value(self, delta = None, index = None, value = None):
        if index == 0:
            for i in range(0, 3):
                delta.radius[i] += value
            return
        index -= 1

        if index in range(0, 2):
            delta.endstop[index] += value
            return
        index -= 2

        if index == 0:
            for i in range(0, 3):
                delta.diagonal[i] += value
            return
        index -= 1

        if index in range(0, 2):
            delta.radius[index] += value
            return
        index -= 2

        if index in range(0, 2):
            delta.diagonal[index] += value
            return
        index -= 2

        if index in range(0, 2):
            delta.bed_screw[index][2] += value
            return
        index -= 2

        return

    def _dist2(self, a, b):
        dist = [float(a[i]-b[i]) for i in range(0, 3)]
        return sum([d*d for d in dist])/3

    def _perturb(self, index = 0, pos = (0, 0, 0), motor = (0, 0, 0), perturb = 0.1):
        delta = self.copy()

        self._apply_value(delta, index, perturb)
        delta.recalc()

        alt_pos = delta.motor_to_delta(motor)
        return self._dist2(pos, alt_pos)

    def _print_parms(self):
        print "Bed Height: %.3fmm" % (self.bed_height)

        print "Steps per mm: %.3f" % (self.steps)

        for i in range(0, 3):
            print "Bed Level %c: %.3fmm (%.3f turns)" % (ord('A') + i, self.bed_screw[i][2], self.bed_screw[i][2]/0.5)

        # Adjust all the endstops
        for i in range(0, 3):
            print "Endstop %c: %.3fmm (%d steps)" % (ord('X') + i, self.endstop[i], self.endstop[i] * self.steps_per_mm())

        for i in range(0, 3):
            print "Radius %c: %.3fmm" % (ord('A') + i, self.radius[i])

        for i in range(0, 3):
            print "Angle %c: %.3f deg" % (ord('A') + i, self.angle[i])

        for i in range(0, 3):
            print "Diagonal Rod %c: %.3fmm" % (ord('A') + i, self.diagonal[i])

    def calibrate(self, target = 0.05):
        print "Original parameters:"
        self._print_parms()

        target = target * target

        delta_points = self.delta_probe(self.numPoints)

        # Collect motor points
        motor_points = [p[3::] for p in delta_points]

        for i in range(0, len(delta_points)):
            point = delta_points[i]
            pass

        #self.view(delta_points)

        err = 0
        for p in range(0, self.numPoints):
             err += self._perturb(0, delta_points[p], motor_points[p], 0)

        print "OldErr: %.3f" % (err)

        converged = err < target

        perturb = 0.1
        oldFactor = -1
        oldValue = 0
        while not converged:
            sumOfSquares = [0] * self.numFactors * 2
            minFactor = -1

            # Perturb positive...
            for i in range(0, self.numFactors):
                sumOfSquares[i] = 0
                for p in range(0, self.numPoints):
                    sumOfSquares[i] += self._perturb(i, delta_points[p], motor_points[p], perturb)
                if minFactor < 0:
                    minFactor = i
                elif sumOfSquares[minFactor] > sumOfSquares[i]:
                    minFactor = i

            # Perturb negaitive...
            for n in range(0, self.numFactors):
                i = n + self.numFactors
                sumOfSquares[i] = 0
                for p in range(0, self.numPoints):
                    sumOfSquares[i] += self._perturb(n, delta_points[p], motor_points[p], -perturb)
                if minFactor < 0:
                    minFactor = i
                elif sumOfSquares[minFactor] > sumOfSquares[i]:
                    minFactor = i

            err = sumOfSquares[minFactor]
            value = perturb
            if minFactor >= self.numFactors:
                minFactor -= self.numFactors
                value = -value

            print sumOfSquares
            print "MinFactor: %d %.3f (err = %.3f)\n" % (minFactor, value, err)

            if oldFactor == minFactor and oldValue == -value:
                break

            oldFactor = minFactor
            oldValue = value

            self._apply_value(self, minFactor, value)
            self.recalc()

            if err < target:
                converged = True
                break

            self._print_parms()
            pass

        # Display updated parameters
        if converged:
            minstop = self.endstop[0]
            for i in range(1, 3):
                if self.endstop[i] < minstop:
                    minstop = self.endstop[i]
            for i in range(0, 3):
                self.endstop[i] -= minstop

            print "Converged solution found:"
            self._print_parms()

        return converged

# vim: set shiftwidth=4 expandtab: 
