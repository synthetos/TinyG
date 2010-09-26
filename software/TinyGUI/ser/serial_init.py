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
        #print "[*]Scanning for Windows Serial Ports"
        import scanwin32
        ports = []
        for order, port, desc, hwid in sorted(scanwin32.comports()):
            ports.append(port)
        return ports
    elif platform.system() == "Darwin":
        #print "[*]Scanning for OSX Serial Ports"
        ports = []
        import scanosx
        return scanosx.scan()
    else:
        #print "[*]Scanning for Linux Serial Ports"
        import scanlinux
        return scanlinux.scan()
 
def connect(port, speed):
    #print "[*]Connecting to %s at %s" % (port, speed)
    ser = serial.Serial(port, speed, timeout=.3)
    return ser
    