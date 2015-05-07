#!/usr/bin/env python
import serial
import time
ser = serial.Serial('/dev/ttyUSB0', 2400, timeout=1)
while True:
    string = ser.read(12)
    if len(string) == 0:
        print "Please insert a tag"
        continue
    else:
        print string
