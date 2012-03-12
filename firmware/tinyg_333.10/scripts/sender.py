#!/usr/bin/env python
import sys
from optparse import OptionParser

try:
    import serial
except ImportError:
    print "You need the serial lib for python\n Go here: http://pypi.python.org/pypi/pyserial"
    sys.exit()
    
    
parser = OptionParser()
parser.add_option("-p", "--port", dest="port", help="Serial port to use")
parser.add_option("-f", "--file", dest="file", help="File to send to TinyG")

(options, args) = parser.parse_args()

print "Opening Port"
if options.port != None:
    try:
        s = serial.Serial(options.port, 115200)
    except serial.serialutil.SerialException as e:
        print "Error: %s" % e
        sys.exit()
        
else:
    print "Please Specify a port"
    sys.exit()
    
if options.file != None:
    file = options.file
else:
    print "Please specify a Gcode file to send"
    sys.exit()

s.timeout = .25
with open(file) as data:
    for line in data:
        res = raw_input("Hit Enter to send\t %s" %line.rstrip())
        s.writelines(line)
        
        
        
        
    
    
    
    
