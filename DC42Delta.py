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

class DC42Delta(Delta.Delta):
    """ DC42Delta Calibrator """

    # Solve for the following factors:
    #  0 - Endstop A trim
    #  1 - Endstop B trim
    #  2 - Endstop C trim
    #  3 - Radius A
    #  4 - Radius B
    #  5 - Radius C
    #  6 - Angle A
    #  7 - Angle B
    #  8 - Diagonal A
    #  9 - Diagonal B # FIXME
    # 10 - Diagonal C # FIXME
    #
    # The assumption is that the bed is flat,
    # and that Angle C is correct.
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
            hi.angle[0] += perturb
            lo.angle[0] -= perturb
        elif deriv == 7:
            hi.angle[1] += perturb
            lo.angle[1] -= perturb
        elif deriv == 8:
            hi.diagonal[0] += perturb
            lo.diagonal[0] -= perturb
        # elif deriv == 9:
            hi.diagonal[1] += perturb
            lo.diagonal[1] -= perturb
        # elif deriv == 10:
            hi.diagonal[2] += perturb
            lo.diagonal[2] -= perturb
            pass

        hi.recalc()
        lo.recalc()

        pos_hi = hi.motor_to_delta(pos)
        pos_lo = lo.motor_to_delta(pos)

        return (pos_hi[2] - pos_lo[2])/(2 * perturb)

    def _print_matrix(self, name, matrix, rows, cols):
        print name
        for i in range(0, rows):
            for j in range(0, cols):
                print "%7.3f" % (matrix[i][j]),
                pass
            print
            pass



    def _gauss_jordan(self, mat = [0], n = 0):
        for i in range(0,n):
            vmax = math.fabs(mat[i][i])
            for j in range(i+1,n):
                rmax = math.fabs(mat[i][j])
                if rmax > vmax:
                    row = mat[i]
                    mat[i] = mat[j]
                    mat[j] = row
                    vmax = rmax
                    pass
                pass

            # self._print_matrix("Gauss%d:" % i, mat, n, n+1)

            v = mat[i][i]
            for j in range(0, n):
                if j == i:
                    continue

                factor = mat[j][i]/v
                mat[j][i] = 0
                for k in range(i+1, n+1):
                    mat[j][k] -= mat[i][k] * factor
                    pass
                pass

            pass

        solution = [0] * n

        for i in range(0, n):
            solution[i] = mat[i][n]/mat[i][i]

        return solution


    def calibrate(self, target = 0.03):
        delta_points = self.probe_points(self.numPoints)
        probe_offset = self.zprobe_offset()

        self.home()

        # Collect probe points
        motor_points = []
        zpoints = [0] * len(delta_points)
        initialSumOfSquares = 0
        for i in range(0, len(delta_points)):
            point = delta_points[i]

            self.move((point[0], point[1], 20.0))
            zpoints[i]= self.zprobe()

            # Convert from probe to nozzle position
            pos = (point[0] - probe_offset[0], point[1] - probe_offset[1], 0)

            # Convert from delta to motor position
            motor = self.delta_to_motor(pos)
            print "probe %.2f, %.2f => %.2f" % (point[0], point[1], zpoints[i])

            motor_points.append(motor)
            initialSumOfSquares += math.pow(zpoints[i], 2)
            pass

        # Do a Newton-Raphson iterations

        # Build a Nx7 matrix of derivatives

        dMatrix = [[0] * self.numPoints for _ in xrange(self.numFactors)]
        for i in range(0, len(delta_points)):
            for j in range(0, self.numFactors):
                dMatrix[i][j] = self._derivative(j, motor_points[i])
                pass
            pass

        self._print_matrix("dMatrix:", dMatrix, self.numPoints, self.numFactors );

        # Build the equations = values
        nMatrix = [[0] * (self.numFactors + 1) for _ in xrange(self.numFactors)]
        for i in range(0, self.numFactors):
            for j in range(0, self.numFactors):
                temp = dMatrix[0][i] * dMatrix[0][j]
                for k in range(1, len(delta_points)):
                    temp += dMatrix[k][i] * dMatrix[k][j]
                nMatrix[i][j] = temp
                pass

            temp = 0
            for k in range(0, self.numPoints):
                temp += dMatrix[k][i] * -zpoints[k]
                pass
            nMatrix[i][self.numFactors] = temp
            pass

        self._print_matrix("nMatrix:", nMatrix, self.numFactors, self.numFactors + 1);
        solution = self._gauss_jordan(nMatrix, self.numFactors)

        #self._print_matrix("nMatrix:", nMatrix, self.numFactors, self.numFactors + 1);

        self._print_matrix("Solution:", [solution], 1, self.numFactors);

        # Apply solution to endstop trims
        eav = 0
        for i in range(0, 3):
            self.endstop[i] += solution[i]
            if i == 0 or self.endstop[i] < eav:
                eav = self.endstop[i]

        # Adjust all the endstops
        for i in range(0, 3):
            self.endstop[i] -= eav
            print "Endstop %c: %.2fmm" % (ord('X') + i, self.endstop[i])

        for i in range(0, 3):
            self.radius[i] += solution[3 + i]
            print "Radius %c: %.2fmm" % (ord('A') + i, self.radius[i])

        for i in range(0, 2):
            self.angle[i] += solution[6 + i]
            print "Angle %c: %.2f deg" % (ord('A') + i, self.angle[i])
        print "Angle C: %.2f deg" % (self.angle[2])

        for i in range(0, 3):
            #self.diagonal[i] += solution[8 + i]
            self.diagonal[i] += solution[8]
            print "Diagonal Rod %c: %.2fmm" % (ord('A') + i, self.diagonal[i])

        self.recalc()

        # Update firmware
        self.update()

        pass

# vim: set shiftwidth=4 expandtab: 
