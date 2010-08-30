#!/usr/bin/env python
#Gcode Command Line Loader for TinyG
#synthetos.com
#Riley Porter

import serial, sys, getopt

print "############################"
print "[*]TinyG Gcode Loader v1.0a"
print "############################"


def usage():
    print "USAGE:"
    print "GcodeLoader.py <serial port> <speed> <gcode file>"
    print "EXAMPLE:"
    print "GcodeLoader.py COM4 115200 circles.gcode"
    print "NOTE: An optional -d as the LAST argument will enable debug mode"
    
    #sys.exit(3)

def main(SPEED=115200):
    global debug
    debug  = 0
    try:
#        PORT = sys.argv[1]
#        SPEED = sys.argv[2]
#        FILE = sys.argv[3]
#        PORT = '/dev/cu.usbserial-A700fhW6'      # board #8 (Alden)
        PORT = '/dev/cu.usbserial-A700fhWe'      # board #1 (Alden)
        SPEED = 115200
#        FILE = "gcode_files/circles.gcode"
        FILE = "gcode_files/song.gcode"
#        FILE = "gcode_files/hacdc.gcode"
#        FILE = "gcode_files/pcb.gcode"
        debug = 1
    except:
        print "[ERROR]"
        usage()	
    
    try:
        ser = serial.Serial(PORT, SPEED)
    except:
        print "[ERROR] Opening Serial Port"
        sys.exit(3)
        
    try: #check file exists
        f = open(FILE, 'r')
    except:
        print "[ERROR] Opening Gcode File."
        sys.exit(4)
    
    try:
        dbg_check = sys.argv[4]
        if dbg_check == "-d":
            print "[!]Enabling Verbose Debug Mode"
            debug = 1
    except:
        pass
    
    
    checkSettings(PORT, SPEED, FILE) #Make sure the user entered the right arguments
    processFile(ser, f) #f is an opened gcode file from above. Send the gcode down the line
    print "Job Complete"
    ser.close() #close the serial port

def choice():
    
    while True:
        confirm = raw_input("Please Enter (Y/N)")
        if confirm == "Y" or confirm == "y":
            print "Processing Job - Press CTRL+C to Abort Job."
            break
        elif confirm == "N" or confirm == "n":
            print "Job Aborted, Exiting..."
            sys.exit()
        
def checkSettings(PORT, SPEED, FILE):
    print "-----------"
    print "   :JOB:"
    print "-----------"
    print "FILE = %s" % FILE
    print "PORT = %s" % PORT
    print "SPEED = %s" % SPEED
    print "Are these settings correct?"
    choice()
    
def processFile(ser, f ):
    try:
        ser.write(3*"\n") #This might not be need anymore but it just sends a few \n down the wire to clear anything before sending gcode
    except serial.serialutil.SerialException:
        print "[ERROR] Write Failed"
        print "System Exiting"
        sys.exit(6)
    delim = ""
    for line in f:
        line = line.rstrip() #remove trailing new line character
        if debug == 1:
#            print "[*]Writing: %s" % line
            print "%s" % line
        ser.write(line+"\n") #send a line of gcode
        delim = ser.read() #check to see if we found our delimeter '*'
        response = delim
        while delim != "*":  #Loop until we find the delim
            delim = ser.read()
            response += delim
#        print "%s" % response
#        print

if __name__ == "__main__":
    main()

