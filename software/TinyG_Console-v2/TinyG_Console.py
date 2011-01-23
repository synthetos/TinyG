#!/usr/bin/env python
#Gcode Command Line Loader for TinyG
#synthetos.com
#Riley Porter

try:
    import serial
except:
    print "[!]Error Loading the Serial Module"
    print "  [!]Please install pyserial onto your system"
    sys.exit(1)
    
import  sys, getopt
from optparse import OptionParser

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
        PORT = sys.argv[1]
        SPEED = sys.argv[2]
        FILE = sys.argv[3]
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
    
    
class Job(object):
    """Main Class for Gcode File Job"""
    def __init__(self, options, args):

        print "############################"
        print "[*]TinyG Gcode Loader v1.0a"
        print "############################"
        
        self.speed = options.speed
        self.port = options.port

        """Check to see if we are in debug mode"""
        if options.debug == None:
            self.debug = False
        else:
            self.debug = True
            
        """Check to see if a gcode directory was passed"""
        if options.gfiles == None:  #Check to see if they gave us an Directory of files
            self.gfile = options.gfile
        else:
            """Do dir code here"""
            pass
        self.Connect()
        self.CheckFile()
        self.ProcessFile()
        
        
    def Connect(self):
        """Connect to TinyG with the parameters passed by command line"""
        print "[*] Connecting to %s" % self.port
        try:
            self.sconnection = serial.Serial(self.port, self.speed)
            print "  [*] Connection Successful"
            return self
        
        except:
            print "[ERROR] Connecting to %s" % self.port
            print "  [!] Check to make sure TinyG is connected and you selected the correct Serial port / speed."
            print "  [!] Exiting...."
            sys.exit(3)
        
   
    def CheckFile(self):        
        try: #check file exists
            self.f = open(self.gfile, 'r')
            return self  #Returns the f as a file object to the gcode file
        except:
            print "[ERROR] Opening Gcode File."
            print "  [!] Exiting...."
            sys.exit(4)
    
    
    

    
        def ProcessFile(ser, f ):
            """Process the Gcode file/s"""
            if len(self.gfile) > 1:
                """We were given a directory of Gcode files to run.. Running them now"""
                pass
            else:
                """We were given a single file.  Running it now"""
                try:
                    self.connection.write(3*"\n")
                    """Clean up the command line if needed"""
                except:
                    print  "[ERROR] Writing to Serial Port: %s failed.." % self.port
                    print  "   [!] Aborting the Job..."
                    print  "  [!] System Exiting..."
                    self.sconnection.close()
                    
                """We were abled to write to the serial port... Moving on now"""
                delim = ""
            for line in self.f.readlines():
                line = line.rstrip() #remove trailing new line character
                if self.debug == True:
                    print "[*]Writing: %s" % line
                self.connection.write(line+"\n") #send a line of gcode
                delim = self.connection.read() #check to see if we found our delimeter '*' 
                while delim != "*":  #Loop until we find the delim
                    delim = self.connection.read()
            print "[*] Gcode File Processed"
            print "[*] Job Complete"
            print "[*] Exiting"
            self.sconnection.close()
        
                
    
def main():
    parser = OptionParser()
    parser.add_option("--port", dest="port", help="Serial Port to be used.  IE: Windows = COM3, linux/Mac = /dev/ttyUSB0")
    parser.add_option("--file", dest="gfile", help="Gcode file to run")
    parser.add_option("--dir", dest="gfiles", help="Path/To/File full of gcode files to be ran")
    parser.add_option("--speed",dest="speed",help="Serial port baud rate... Normally 115200")
    parser.add_option("--debug",dest="debug",help="Enters a overly voberse debug mode")
    (options, args) = parser.parse_args()
    
    job = Job(options, args) 

if __name__ == "__main__":
    main()