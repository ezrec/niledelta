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
import Delta

class SmoothieDelta(Delta.Delta):
    """ SmootheDelta Calibrator """
    def __init__(self, port = None, probe = None, eeprom = None):
        Delta.Delta.__init__(self, port = port, probe = probe, eeprom = eeprom)

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

        trim = self.endstop_trim()
        print("Trim Start: A:%f, B:%f, C:%f" % (trim[0], trim[1], trim[2]))

        # Get probe points
        points = self.probe_points()

        # Clear endstop trims
        trim = [ 0, 0, 0]

        # Probe up to 10 times...
        for i in range(0,10):
            # Set trim
            self.endstop_trim([trim[0], trim[1], trim[2]])

            # Home
            self.home()

            # Get the Z at the base of the 3 towers
            tower_z = [0] * 3
            for i in range(0, 3):
                point = points[i]
                tower_z[i] = self.zprobe((point[0], point[1], 20))
                pass

            trimscale = 1.2522 # Emperically determined

            minz = min(tower_z)
            maxz = max(tower_z)

            if (maxz - minz) < target:
                print("  Pass: A:%f, B:%f, C:%f" % (trim[0], trim[1], trim[2]))
                return True

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
        self.home()
        cmm = self.zprobe([0,0,20])

        delta_radius = self.delta_radius()

        for i in range(0, 3):
            print("Radius %c: %.3fmm" % (i+65, delta_radius[i]))

        drinc = 2.5

        for p in range(0,10):
            for i in range(0, 3):
                # Probe the three towers, and average
                zsum = 0
                for t in range(0, 3):
                    zsum += self.zprobe((points[t][0], points[t][1], 20))

                m = zsum/3.0
                d = cmm - m

                if math.fabs(d) < target:
                    print("  Pass: Radius %c: %fmm" % (65+i, delta_radius[i]))
                    return True

                # increase delta radius to adjust for low center
                # decrease delta radius to adjust for high center
                delta_radius[i] += d * drinc
                print("  Pass: Radius %c: %fmm" % (65+i, delta_radius[i]))

                self.radius = delta_radius[:]
                self.update()

                self.home()
                cmm = self.zprobe()
                pass
            pass

        for i in range(0, 3):
            print("  FAIL: Radius %c %fmm" % (65+i, delta_radius[i]))
        pass

    def calibrate(self):
        self.calibrate_endstops()
        self.calibrate_delta_radius()
        pass

# vim: set shiftwidth=4 expandtab: 
