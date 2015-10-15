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
```

## Author

Jason S. McMullan
