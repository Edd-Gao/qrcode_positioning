#!/usr/bin/env python
'''
 name: camera_calibration.py
 usage: used to test the camera calibration info parsers
'''

import camera_calibration_parsers as parser

try:
    name, info = parser.readCalibration('rpi_cam_v2_640_480.yml')
except TypeError:
    print "error reading calibrations."

print name