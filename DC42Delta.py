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
import mpl_toolkits.mplot3d.axes3d
import matplotlib.pyplot
import matplotlib.tri
import matplotlib.cm

class DC42Delta(Delta.Delta):
    """ DC42Delta Calibrator """

    # Solve for the following factors:
    #  0 - Endstop A
    #  1 - Endstop B
    #  2 - Endstop C
    #  6 - Radius A
    #  7 - Radius B
    #  8 - Radius C
    #  9 - Steps per mm
    #  9 - Diagonal rod
    #  3 - Bed Level Screw A (Diagonal from A)
    #  4 - Bed Level Screw B (Diagnoal from B)
    #  5 - Bed Level Screw C (Digaonal from C)
    #
    # Assuming that Angles A/B/C are correct.
    #
    numFactors = 6
    numPoints = 13

    def __init__(self, port = None, probe = None, eeprom = None):
        Delta.Delta.__init__(self, port = port, probe = probe, eeprom = eeprom)

    def _apply_value(self, delta = None, index = None, value = None):
        if index in range(0, 3):
            delta.endstop[index] += value
            return
        index -= 3

        if index in range(0, 3):
            delta.radius[index] += value
            return
        index -= 3

        if index == 0:
            delta.steps += value
            return
        index -= 1

        if index == 0:
            for i in range(0, 3):
                delta.diagonal[i] += value
            return
        index -= 1

        if index in range(0, 3):
            delta.bed_screw[index][2] += value
            return
        index -= 3

        return

    def _apply_factor(self, factor = [0] * numFactors, delta = None):
        if delta is None:
            delta = self

        for i in range(0, len(factor)):
            self._apply_value(delta, i, factor[i])

        delta.recalc()

    def _derivative(self, deriv = 0, pos = (0, 0, 0)):
        perturb = 0.2
        hi = self.copy()
        lo = self.copy()
        factor = [0] * self.numFactors

        factor[deriv] = perturb
        self._apply_factor(factor = factor, delta = hi)

        factor[deriv] = -perturb
        self._apply_factor(factor = factor, delta = lo)

        pos_hi = hi.motor_to_delta(pos)
        pos_hi[2] -= hi.bed_offset(pos_hi)
        pos_lo = lo.motor_to_delta(pos)
        pos_lo[2] -= lo.bed_offset(pos_lo)

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

        self._print_matrix("Solved matrix", mat, n, n + 1)

        return solution

    def _print_parms(self):
        print "Bed Height: %.3fmm" % (self.bed_height)

        print "Steps per mm: %.3f" % (self.steps)

        for i in range(0, 3):
            print "Bed Level %c: %.3fmm" % (ord('A') + i, self.bed_screw[i][2])

        # Adjust all the endstops
        for i in range(0, 3):
            print "Endstop %c: %.3fmm (%d steps)" % (ord('X') + i, self.endstop[i], self.endstop[i] * self.steps_per_mm())

        for i in range(0, 3):
            print "Radius %c: %.3fmm" % (ord('A') + i, self.radius[i])

        for i in range(0, 3):
            print "Angle %c: %.3f deg" % (ord('A') + i, self.angle[i])

        for i in range(0, 3):
            print "Diagonal Rod %c: %.3fmm" % (ord('A') + i, self.diagonal[i])

    def _view(self, points, correction):
        x = numpy.array([p[0] for p in points])
        y = numpy.array([p[1] for p in points])
        z = numpy.array([(points[i][2]+correction[i]) for i in range(0,len(points))])

        tri = matplotlib.tri.Triangulation(x, y)
        ref = matplotlib.tri.UniformTriRefiner(tri)
        new, new_z = ref.refine_field(z, subdiv=4)

        norm = matplotlib.pyplot.Normalize(vmax=1, vmin=-1)
        kwargs = dict(triangles=new.triangles, cmap=matplotlib.cm.jet, norm=norm, linewidth=0.2)

        fig = matplotlib.pyplot.figure()
        ax = mpl_toolkits.mplot3d.axes3d.Axes3D(fig)
        ax.set_zlim3d(-0.5, 0.5)
        plot = ax.plot_trisurf(new.x, new.y, new_z, **kwargs)
        matplotlib.pyplot.show()

    def calibrate(self, target = 0.03):
        print "Original parameters:"
        self._print_parms()

        delta_points = self.delta_probe(self.numPoints)

        # Collect probe points
        motor_points = []
        zavg = sum([x[2] for x in delta_points]) / len(delta_points)

        initialSumOfSquares = 0
        offset = self.zprobe_offset()

        for i in range(0, len(delta_points)):
            point = delta_points[i]
            point[2] -= zavg
            perfect = (point[0]+offset[0], point[1]+offset[1], 0)

            # Convert from delta to motor position
            motor = self.delta_to_motor(perfect)

            print "probe %.2f, %.2f, [%.2f] => %.2f, %.2f, %.2f" % (point[0], point[1], 0.0, motor[0], motor[1], motor[2])

            motor_points.append(motor)
            initialSumOfSquares += math.pow(point[2], 2)
            pass

        # Do Newton-Raphson iterations until we converge (or fail to converge)
        converged = False
        zCorrection = [0] * self.numPoints

        for attempt in range(0, 10):
            # Build a Nx7 matrix of derivatives

            self._view(delta_points, zCorrection)

            dMatrix = [[0] * self.numFactors for _ in xrange(self.numPoints)]
            for i in range(0, len(delta_points)):
                for j in range(0, self.numFactors):
                    dMatrix[i][j] = self._derivative(j, motor_points[i])
                    pass
                pass

            self._print_matrix("Derivative matrix", dMatrix, self.numPoints, self.numFactors );

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

            self._print_matrix("Normal matrix", nMatrix, self.numFactors, self.numFactors + 1);
            solution = self._gauss_jordan(nMatrix, self.numFactors)

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
                    return

            self._apply_factor(solution)
            self._print_parms()

            # Calculate the expected probe heights with this new set of adjustments
            expectedResiduals = [0] * self.numPoints
            sumOfSquares = 0

            for i in range(0, len(delta_points)):
                print ("[ %.3f, %.3f, %.3f ] " % (motor_points[i][0], motor_points[i][1], motor_points[i][2])),

                newPosition = self.motor_to_delta(motor_points[i])
                newPosition[2] -= self.bed_offset(newPosition)
                print ("[ %.3f, %.3f, %.3f ] => [ %.3f, %.3f, %.3f ]" % (motor_points[i][0], motor_points[i][1], motor_points[i][2], newPosition[0], newPosition[1], newPosition[2]))
                zCorrection[i] = newPosition[2]
                expectedResiduals[i] = delta_points[i][2] + newPosition[2]
                sumOfSquares += math.pow(expectedResiduals[i], 2)

            expectedRmsError = math.sqrt(sumOfSquares / len(delta_points))
            self._print_matrix("Expected probe error %.3f:" % (expectedRmsError), [expectedResiduals], 1, self.numPoints)
            if attempt > 1 and expectedRmsError < 0.10:
                converged = True
                break


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
            self._view(delta_points, zCorrection)

        return converged

# vim: set shiftwidth=4 expandtab: 
