# NileDelta - External Delta 3D Calibrator

NileDelta is a Python program that attempts to calibrate
your Delta 3D printer using Z-Probing and the David Crocker
(dc42) Newton's Approximation method.

Unlike the built-in version in RepRap firmware, this
version will attempt to calibrate Arduino 8-bit firmwares
(currently Repetier) via the GCode interface, instead of
as an on-firmware feature.

## Usage

```
$ python niledelta.py /dev/ttyUSB0 250000
probe 81.00, 0.00 => 0.10
probe 57.28, 57.28 => 0.10
probe 0.00, 81.00 => 0.10
probe -57.28, 57.28 => 0.10
probe -81.00, 0.00 => 0.10
probe -57.28, -57.28 => 0.10
probe -0.00, -81.00 => 0.10
probe 57.28, -57.28 => 0.10
probe 0.00, 0.00 => 0.50
Endstop X: 0.00mm
Endstop Y: 1.10mm
Endstop Z: 2.20mm
Radius A: 99.02mm
Radius B: 99.02mm
Radius C: 99.02mm
Angle A: 210.00 deg
Angle B: 330.00 deg
Angle C: 90.00 deg
Diagonal Rod A: 189.82mm
Diagonal Rod B: 189.82mm
Diagonal Rod C: 189.82mm
```

## What It Solves For

We attempt to solve for the following factors, using 9 probe points:

0. Endstop A trim
1. Endstop B trim
2. Endstop C trim
3. Radius A
4. Radius B
5. Radius C
6. Angle A
7. Angle B
8. Diagonal

The assumption is that the bed is flat,
and that Angle C is correct.

## Author

Jason S. McMullan
