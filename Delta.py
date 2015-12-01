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

import math
import GCode
import copy

class Delta(GCode.GCode):
    """ Delta Machine """
    def __init__(self, port = None, eeprom = None, probe = None):
        GCode.GCode.__init__(self, port = port, eeprom = eeprom, probe = probe)
        self.bed_radius, self.bed_height = self.delta_bed()
        self.radius = self.delta_radius()
        self.diagonal = self.delta_diagonal()
        self.angle = self.delta_angle()
        self.endstop = self.endstop_trim()
        self.bed_factor = 0.8
        self.bed_screw = [[0,0,0],[0,0,0],[0,0,0]]
        self.bed_matrix = [0, 0, 0, 0]
        self.recalc()

    def copy(self):
        delta = Delta(None)
        delta.bed_radius = self.bed_radius
        delta.bed_height = self.bed_height
        delta.radius = self.radius[:]
        delta.diagonal = self.diagonal[:]
        delta.angle = self.angle[:]
        delta.endstop = self.endstop[:]
        delta.bed_factor = self.bed_factor
        delta.bed_screw = self.bed_screw[:]
        delta.recalc()
        return delta

    def update(self):
        self.delta_radius(self.radius)
        self.delta_diagonal(self.diagonal)
        self.delta_radius(self.radius)
        self.delta_angle(self.angle)
        self.endstop_trim(self.endstop)
        self.delta_bed(radius = None, height = self.bed_height)

    def recalc(self):
        # Get the positions of the 3 towers
        deg = math.pi/180.0

        self.tower = [(), (), ()]
        for i in range(0,3):
            px = math.cos(self.angle[i] * deg)
            py = math.sin(self.angle[i] * deg)
            self.tower[i] = (px * self.radius[i], py * self.radius[i])

        for i in range(0,3):
            px = math.cos(self.angle[i] * deg)
            py = math.sin(self.angle[i] * deg)
            self.bed_screw[i][0] = -px * 100
            self.bed_screw[i][1] = -py * 100

        v1 = [self.bed_screw[1][0] - self.bed_screw[0][0],
              self.bed_screw[1][1] - self.bed_screw[0][1],
              self.bed_screw[1][2] - self.bed_screw[0][2]]
        v2 = [self.bed_screw[2][0] - self.bed_screw[0][0],
              self.bed_screw[2][1] - self.bed_screw[0][1],
              self.bed_screw[2][2] - self.bed_screw[0][2]]

        norm = [v1[1]*v2[2]-v1[2]*v2[1],
                v1[2]*v2[0]-v1[0]*v2[2],
                v1[0]*v2[1]-v1[1]*v2[0]]
        self.bed_matrix = [norm[0], norm[1], norm[2],
                           norm[0]*self.bed_screw[0][0] +
                           norm[1]*self.bed_screw[0][1] +
                           norm[1]*self.bed_screw[0][2]]

    def bed_offset(self, point = (0, 0)):
        return (point[0]*self.bed_matrix[0] +
                point[1]*self.bed_matrix[1] -
                self.bed_matrix[3])/-self.bed_matrix[2]

    def plot_points(self, points):
        f = open("plot.plt", "w+")
        for point in points:
            f.write("%.3f %.3f %.3f\n" % (point[0], point[1], point[2]));
        f.close()

    def probe_points(self, count = 7):
        deg = math.pi/180.0

        # Canonical points (towerA, towerB, towerC, center, midBC, midAC, midAB)
        if count == 7:
            tower = [0] * 7
            for i in range(0,3):
                px = math.cos(self.angle[i] * deg) * self.bed_radius * self.bed_factor
                py = math.sin(self.angle[i] * deg) * self.bed_radius * self.bed_factor
                tower[i] = (px, py)
            return tower[0], tower[1], tower[2], (0, 0), (-tower[0][0], -tower[0][1]), (-tower[1][0], -tower[1][1]), (-tower[2][0], -tower[2][1])

        if count == 13:
            tower = [ (0,0), (0,0), (0,0) ]
            for i in range(0,3):
                px = math.cos(self.angle[i] * deg) * self.bed_radius * self.bed_factor
                py = math.sin(self.angle[i] * deg) * self.bed_radius * self.bed_factor
                tower[i] = (px, py)
            tower = [ tower[0],
                     (-tower[2][0], -tower[2][1]),
                     tower[1],
                     (-tower[0][0], -tower[0][1]),
                     tower[2],
                     (-tower[1][0], -tower[1][1])]
            retval = [0] * 13
            for i in range(0, 6):
                retval[i] = tower[i]
                retval[i+6] = (tower[i][0]/2, tower[i][1]/2)
            retval[12] = (0,0)
            return retval



        # Make a circle around the bed, ending at the center.
        points = []
        for i in range(0, count-1):
            x = math.cos(2*math.pi * i/(count-1)) * self.bed_radius * self.bed_factor
            y = math.sin(2*math.pi * i/(count-1)) * self.bed_radius * self.bed_factor
            points.append((x, y))
        points.append((0, 0))

        return points

    def delta_probe(self, points = 7):
        probe_points = self.probe_points(self.numPoints)
        probe_offset = self.zprobe_offset()

        self.home()
        self.move((0, 0, 20))
        self.zprobe(None, first = True)

        # Collect probe points
        delta_points = [[x[0], x[1], 0] for x in probe_points]
        for point in delta_points:
            point[2] = self.zprobe((point[0], point[1], 20))
            # Convert from probe to nozzle
            point[0] -= probe_offset[0]
            point[1] -= probe_offset[1]

        self.zprobe(None, last = True)

        if self.port is not None:
            self.plot_points(delta_points)

        return delta_points

    def delta_to_motor(self, point = (0, 0, 0)):
        motor = [0, 0, 0]
        z = point[2]
        for i in range(0, 3):
            D2 = math.pow(self.diagonal[i], 2)
            motor[i] = z + math.sqrt(D2 - math.pow(point[0] - self.tower[i][0], 2) - math.pow(point[1] - self.tower[i][1], 2))
            motor[i] += self.endstop[i]

        return motor

    def motor_to_delta(self, pos = (0, 0, 0)):
        coreF = [0, 0, 0]
        F = [0, 0, 0]
        motor = pos[:]
        for i in range(0, 3):
            motor[i] -= self.endstop[i]
            coreF[i] = math.pow(self.tower[i][0], 2) + math.pow(self.tower[i][1], 2)
            F[i] = coreF[i] + math.pow(motor[i], 2)
            pass

        # Setup PQRSU such that x = -(S - uz)/P, y = (P - Rz)/Q
        P = S = R = U = 0
        for i in range(0, 3):
            a = self.tower[(i + 1) % 3]
            b = self.tower[(i + 2) % 3]
            chord = ((b[0] - a[0]), (b[1] - a[1]))

            P += chord[0] * F[i]
            S += chord[1] * F[i]
            R += chord[0] * motor[i]
            U += chord[1] * motor[i]
            pass

        R *= 2.0
        U *= 2.0
        Q = 2 * ((self.tower[0][0] - self.tower[2][0]) * (self.tower[1][1] - self.tower[0][1]) - (self.tower[1][0] - self.tower[0][0]) * (self.tower[0][1] - self.tower[2][1]))

        D2 = math.pow(self.diagonal[0], 2)  # FIXME
        R2 = math.pow(R, 2)
        U2 = math.pow(U, 2)
        Q2 = math.pow(Q, 2)

        A = U2 + R2 + Q2
        minusHalfB = S * U + P * R + motor[0] * Q2 + self.tower[0][0] * U * Q - self.tower[0][1] * R * Q
        C = math.pow(S + self.tower[0][0] * Q, 2) + math.pow(P - self.tower[0][1] * Q, 2) + (math.pow(motor[0],2) - D2) * Q2

        z = (minusHalfB - math.sqrt(math.pow(minusHalfB, 2) - A * C)) / A
        x = (U * z - S) / Q
        y = (P - R * z) / Q

        return [x, y, z]

# vim: set shiftwidth=4 expandtab: #
