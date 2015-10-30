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
    #  0 - Endstop A
    #  1 - Endstop B
    #  2 - Endstop C
    #  3 - Radius A
    #  4 - Radius B
    #  5 - Radius C
    #  6 - Diagonal A/B/C
    #
    # The assumption is that the bed is flat,
    # and that Angles A/B/C are correct.
    #
    numFactors =7
    numPoints = 13

    def __init__(self, gcode = None):
        Delta.Delta.__init__(self, gcode)

    def _apply_value(self, delta = None, index = None, value = None):
        if index in range(0, 3):
            delta.radius[index] += value
            return
        index -= 3

        if index == 0:
            for i in range(0,3):
                delta.diagonal[i + index] += value
            return
        index -= 1

        if index in range(0, 3):
            delta.endstop[index] += value
            return
        index -= 3

        return 0

    def _apply_factor(self, factor = [0] * numFactors, delta = None):
        if delta is None:
            delta = self

        for i in range(0, len(factor)):
            self._apply_value(delta, i, factor[i])

        delta.recalc()

    def _derivative(self, deriv = 0, pos = (0, 0, 0)):
        perturb = 0.2;
        hi = self.copy()
        lo = self.copy()
        factor = [0] * self.numFactors

        factor[deriv] = perturb
        self._apply_factor(factor = factor, delta = hi)

        factor[deriv] = -perturb
        self._apply_factor(factor = factor, delta = lo)

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

    def _print_parms(self):
        print "Bed Height: %.3fmm" % (self.bed_height)

        # Adjust all the endstops
        for i in range(0, 3):
            print "Endstop %c: %.3fmm (%d steps)" % (ord('X') + i, self.endstop[i], self.endstop[i] * self.steps_per_mm())

        for i in range(0, 3):
            print "Radius %c: %.3fmm" % (ord('A') + i, self.radius[i])

        for i in range(0, 3):
            print "Angle %c: %.3f deg" % (ord('A') + i, self.angle[i])

        for i in range(0, 3):
            print "Diagonal Rod %c: %.3fmm" % (ord('A') + i, self.diagonal[i])

    def calibrate(self, target = 0.03):
        delta_points = self.delta_probe(self.numPoints)

        # Collect probe points
        motor_points = []
        zmin = min([x[2] for x in delta_points])

        # Adjust bed height such that all points are above 0
        self.bed_height += zmin

        initialSumOfSquares = 0
        for i in range(0, len(delta_points)):
            point = delta_points[i]
            point[2] -= zmin

            # Convert from delta to motor position
            motor = self.delta_to_motor(point)
            print "probe %.2f, %.2f, [%.2f] => %.2f, %.2f, %.2f" % (point[0], point[1], point[2], motor[0], motor[1], motor[2])

            motor_points.append(motor)
            initialSumOfSquares += math.pow(point[2], 2)
            pass

        # Do Newton-Raphson iterations until we converge (or fail to converge)
        converged = False
        zCorrection = [0] * self.numPoints

        for attempt in range(0, 2):
            # Build a Nx7 matrix of derivatives

            dMatrix = [[0] * self.numFactors for _ in xrange(self.numPoints)]
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
                    temp += dMatrix[k][i] * -(delta_points[k][2] + zCorrection[k])
                    pass
                nMatrix[i][self.numFactors] = temp
                pass

            self._print_matrix("nMatrix:", nMatrix, self.numFactors, self.numFactors + 1);
            solution = self._gauss_jordan(nMatrix, self.numFactors)
            self._print_matrix("nMatrix:", nMatrix, self.numFactors, self.numFactors + 1);

            #self._print_matrix("nMatrix:", nMatrix, self.numFactors, self.numFactors + 1);

            self._print_matrix("Solution:", [solution], 1, self.numFactors);

            # Determine the residuals
            residuals = [0] * self.numPoints
            for i in range(0, len(delta_points)):
                residuals[i] = delta_points[i][2]
                for j in range(0, self.numFactors):
                    residuals[i] += solution[j] * dMatrix[i][j]

            self._print_matrix("Residuals:", [residuals], 1, self.numPoints)

            for i in range(0, self.numFactors):
                if solution[i] > 20 or solution[i] < -20:
                    print "BOGUS SOLUTION"
                    break

            self._apply_factor(solution)

            # Calculate the expected probe heights with this new set of adjustments
            expectedResiduals = [0] * self.numPoints
            sumOfSquares = 0

            for i in range(0, len(delta_points)):
                newPosition = self.motor_to_delta(motor_points[i])
                zCorrection[i] = newPosition[2]
                expectedResiduals[i] = delta_points[i][2] + newPosition[2]
                sumOfSquares += math.pow(expectedResiduals[i], 2)

            expectedRmsError = math.sqrt(sumOfSquares / len(delta_points))
            self._print_matrix("Expected probe error %.3f:" % (expectedRmsError), [expectedResiduals], 1, self.numPoints)
            if expectedRmsError < 0.2:
                if attempt == 0:
                    print "Calibrated - no corrections needed"
                    return True
                converged = True
                break


        # Update EEPROM
        if converged:
            print "Converged solution found:"
            self._print_parms()

        return converged

# vim: set shiftwidth=4 expandtab: 
