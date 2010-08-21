#!/usr/bin/env python
#Serial Port Initializer for TinyG
import sys
import ctypes
import os
import platform
import serial

def scan():
    """ Scan for available ports. return a list of device names. """
    if platform.system() == 'Windows':
        print "[*]Scanning for Windows Serial Ports"
        import scanwin32
        ports = []
        for order, port, desc, hwid in sorted(scanwin32.comports()):
            ports.append(port)
        return ports
    elif platform.system() == "Darwin":
        print "[*]Scanning for OSX Serial Ports"
        ports = []
        import scanosx
        return scanosx.scan()
    else:
        print "[*]Scanning for Linux Serial Ports"
        import scanlinux
        return scanlinux.scan()
 
def connect(port, speed):
    print "[*]Connecting to %s at %s" % (port, speed)
    ser = serial.Serial(port, speed, timeout=1)
    return ser
    
    
def test_port(port):
    """ Detects whether NMEA GPS sentences can be read from the port. """
    nmea_gps_pattern = '^\$GP[A-Z]{3},'
    gps_detected = False
    try:
        s = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT)
        for i in range(1,10):   # try 10 lines in a row
            line = s.readline().strip()
            m = re.match(nmea_gps_pattern, line)
            if m:
                gps_detected = True
                break
        s.close()
        return gps_detected
    except serial.SerialException:
        return False