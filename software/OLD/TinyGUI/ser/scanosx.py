#!/usr/bin/env python
#OSX Serial port detection
#TinyG
#Riley Porter

import serial
import glob


def scan():
    """scan for available ports. return a list of device names."""
    # return USB devices first, as they are more likely
    return glob.glob('/dev/ttyusb*') + glob.glob('/dev/cu.*')

if __name__=='__main__':
    print "Found ports:"
    for name in scan():
        print name
