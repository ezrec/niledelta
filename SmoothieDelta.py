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
import serial
import time
import GCode

class SmoothieDelta:
    """ SmootheDelta Calibrator """
    def __init__(self, gcode = None, radius = 100):
        self.radius = radius
        self.gcode = gcode

    def probe_points(self):
        # Get the positions of the 3 towers
        deg = 180.0/math.pi
        px = -math.sin(60*deg) * self.radius
        py = math.cos(60*deg) * self.radius
        tower_A = -px, -py
        tower_B = px, -py
        tower_C = 0, self.radius

        return tower_A, tower_B, tower_C, (0, 0), (-tower_A[0], -tower_A[1]), (-tower_B[0], -tower_B[1]), (-tower_C[0], -tower_C[1])


    # Calibration routines adapted from Smoothieware
    def probe_delta_points(self):
        probe = self.probe_points()

        max_delta = 0
        last_z = None
        for point in probe:
            # Move to x, y = probe, z = 20mm
            gcode.move(x = point[0], y = point[1], z=20.0)
            z = self.gcode.zprobe()
            if last_z is None:
                last_z = z
            else:
                delta = math.fabs(z - last_z)
                if delta > max_delta:
                    max_delta = delta

            pass

        return max_delta

    def calibrate_endstops(self, target = 0.03):
        """ Run the endstop calibration """
        """   target is in mm """
        """ 0. Clear endstop trim """
        """ 1. Home """
        """ 2. Probe for Z Bed """
        """ 3. Probe tower positions """
        """ 4. Set minimal initial trims """
        """ 5. Home, and probe the towers again """
        """ 6. Calculate and apply new trims """
        """ 7. Go to 5 until converged """

        trim = self.gcode.endstop_trim()
        print("Trim Start: A:%f, B:%f, C:%f" % (trim[0], trim[1], trim[2]))

        # Get probe points
        points = self.probe_points()

        # Clear endstop trims
        trim = [ 0, 0, 0]

        # Probe up to 10 times...
        for i in range(0,10):
            # Set trim
            self.gcode.endstop_trim(trim[0], trim[1], trim[2])

            # Home
            self.home()

            # Get the Z at the base of the 3 towers
            tower_z = []
            for point in probe[0:3]:
                self.gcode.move(x = point[0], y = point[1], z = 20)
                tower_z.append(self.gcode.zprobe())
                pass

            trimscale = 1.2522 # Emperically determined

            minz = min(tower_z)
            maxz = max(tower_z)

            if (maxz - minz) < target:
                print("  Pass: A:%f, B:%f, C:%f" % (trim[0], trim[1], trim[2]))
                return true

            # Worst case trims
            trim = [0, 0, 0]
            for i in range(0,3):
                trim[i] = (minz - tower_z[i]) * trimscale
                pass

            pass

        print("  FAIL: A:%f, B:%f, C:%f" % (trim[0], trim[1], trim[2]))
        return false

    def calibrate_delta_radius(self, target = 0.03):

        points = self.probe_points()

        # Find the bed at 0,0
        self.gcode.home()
        cmm = self.gcode.zprobe()

        delta_radius = self.gcode.delta_radius()

        print("Radius: %fmm" % (delta_radius))

        drinc = 2.5

        for i in range(0,10):
            # Probe the three towers, and average
            zsum = 0
            for t in range(0, 3):
                self.gcode.move(x=points[t][0], y=points[t][1], z=20)
                zsum += self.gcode.zprobe()

            m = zsum/3.0
            d = cmm - m

            if math.fabs(d) < target:
                print("  Pass: %fmm" % (delta_radius))
                return True

            # increase delta radius to adjust for low center
            # decrease delta radius to adjust for high center
            delta_radius += d * drinc

            self.gcode.delta_radius(delta_radius)

            self.gcode.home()
            cmm = self.gcode.zprobe()
            pass

        print("  FAIL: %fmm" % (delta_radius))
        pass

    def calibrate(self):
        self.calibrate_endstops()
        self.calibrate_delta_radius()
        pass

# vim: set shiftwidth=4 expandtab: 
