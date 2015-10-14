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
import numpy

class DC42Delta(Delta.Delta):
    """ DC42Delta Calibrator """

    # Solve for the following factors:
    #  0 - Endstop A trim
    #  1 - Endstop B trim
    #  2 - Endstop C trim
    #  3 - Radius A
    #  4 - Radius B
    #  5 - Radius C
    #  6 - Diagonal A
    #  7 - Diagonal B
    #  8 - Diagonal C
    #
    # The assumption is that the bed is flat and horizontal, and
    # that the tower angles are correct.
    #
    numFactors = 9
    numPoints = numFactors

    def __init__(self, gcode = None):
        Delta.Delta.__init__(self, gcode)

    def _derivative(self, deriv = 0, pos = (0, 0, 0)):
        perturb = 0.2;
        hi = self.copy()
        lo = self.copy()

        if deriv == 0:
            hi.endstop[0] += perturb
            lo.endstop[0] -= perturb
        elif deriv == 1:
            hi.endstop[1] += perturb
            lo.endstop[1] -= perturb
        elif deriv == 2:
            hi.endstop[2] += perturb
            lo.endstop[2] -= perturb
        elif deriv == 3:
            hi.radius[0] += perturb
            lo.radius[0] -= perturb
        elif deriv == 4:
            hi.radius[1] += perturb
            lo.radius[1] -= perturb
        elif deriv == 5:
            hi.radius[2] += perturb
            lo.radius[2] -= perturb
        elif deriv == 6:
            hi.diagonal[0] += perturb
            lo.diagonal[0] -= perturb
        elif deriv == 7:
            hi.diagonal[1] += perturb
            lo.diagonal[1] -= perturb
        elif deriv == 8:
            hi.diagonal[2] += perturb
            lo.diagonal[2] -= perturb
            pass

        hi.recalc()
        lo.recalc()

        pos_hi = hi.motor_to_delta(pos)
        pos_lo = lo.motor_to_delta(pos)

        return (pos_hi[2] - pos_lo[2])/(2 * perturb);


    def calibrate(self, target = 0.03):
        delta_points = self.probe_points(self.numPoints)
        probe_offset = self.zprobe_offset()

        self.home()

        # Collect probe points
        motor_points = []
        initialSumOfSquares = 0
        for i in range(0, len(delta_points)):
            point = delta_points[i]

            self.move((point[0], point[1], 20.0))
            zpoint = self.zprobe()

            # Convert from probe to nozzle position
            pos = (point[0] - probe_offset[0], point[1] - probe_offset[1], 0.0)

            # Convert from delta to motor position
            motor_points.append(self.delta_to_motor(pos))

            initialSumOfSquares += math.pow(zpoint, 2)
            pass

        # Do a Newton-Raphson iterations

        # Build a Nx7 matrix of derivatives

        dMatrix = numpy.zeros((self.numPoints, self.numFactors))
        for i in range(0, len(delta_points)):
            for j in range(0, self.numFactors):
                dMatrix[i, j] = self._derivative(j, motor_points[i])
                pass
            pass

        print "dMatrix: ", dMatrix

        # Build the equations = values
        eMatrix = numpy.zeros((self.numFactors, self.numFactors))
        vMatrix = numpy.zeros((self.numFactors))
        for i in range(0, self.numFactors):
            for j in range(0, self.numFactors):
                temp = dMatrix[0, i] * dMatrix[0, j]
                for k in range(1, len(delta_points)):
                    temp += dMatrix[k, i] * dMatrix[k, j]
                eMatrix[i, j] = temp
                pass

            temp = dMatrix[0, i]
            for k in range(1, self.numPoints):
                temp += dMatrix[k, i]
                pass
            vMatrix[i] = temp
            pass

        print "eMatrix: ", eMatrix
        print "vMatrix: ", vMatrix
        solution = numpy.linalg.solve(eMatrix, vMatrix)
        print "Solution:", solution

        # Apply solution to endstop trims
        eav = 0
        for i in range(0, 3):
            self.endstop[i] += solution[i]
            print self.endstop[i]
            if i == 0 or self.endstop[i] < eav:
                eav = self.endstop[i]

        # Adjust all the endstops
        for i in range(0, 3):
            self.endstop[i] -= eav

        self.radius += solution[3]
        self.diagonal[0] += solution[4]
        self.diagonal[1] += solution[5]
        self.diagonal[2] += solution[6]

        self.recalc()

        # Update firmware
        self.update()

        pass

# vim: set shiftwidth=4 expandtab: 
