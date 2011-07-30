#!/usr/bin/python
#Console Loader for TinyG v5-v6
#Riley Porter
#Synthetos.com

import serial

import serial, sys, time
from ser.serial_init import scan

#Terminal Color Module
from colorama import init
from termcolor import colored

import sys
import random
import time
global SPEED

SPEED = 115200

logo = """
  _______ _              _____ 
 |__   __(_)            / ____|
    | |   _ _ __  _   _| |  __ 
    | |  | | '_ \| | | | | |_ |
    | |  | | | | | |_| | |__| |
    |_|  |_|_| |_|\__, |\_____|
    CONSOLE        __/ |       
                  |___/        """




def usage():
    print """[?] Usage: 
         Typing #FILE# filename will run a gcode file
         Everything else will operate as you were "Jacked" into TinyG
        
EXAMPLE:
           #FILE# ./files/gcodefile.nc
        """


def IdentifyBoard(port_selected):
    SPEEDS = {"TINYG":115200,
              "GRBL":9600}
    print "[*] Running Auto Board Identification Now!"
    for tmpPort,tmpSpeed in SPEEDS.items():
        print "\t[#]Trying %s at Speed %s" % (tmpPort, tmpSpeed)
        try:
            s = serial.Serial(port_selected, tmpSpeed, timeout=.5)
            s.writelines("\n")
            if s.inWaiting > 1:
                readbuf = s.readall()
                if readbuf == "":
                    s.writelines("$\n")
                    readbuf = s.readall()
                if "ok" in readbuf:
                    print colored("\t[*]Auto Board Identification selected %s:%s\n" % (tmpPort,tmpSpeed), "magenta")
                    return(s)
        except Exception, e:
            print e
    
    
    s = serial.Serial(ports[port_selected], SPEED, timeout=.5)
    s.flush()  #Removes any trailing char's from previous sessions
    print colored(("TinyG successfully connected to %s" % ports[port_selected]), 'magenta')
    return s, tmpPort
    

def init_serial():
    #Get Serial Port Options
    def pick_port():
        print colored("Scanning Ports:\n", 'magenta')
        while(1):
            ports = scan()
            if len(ports) == 0:
                self.scanports_mode()
    
            print "Select a Serial Port:"
            count = 1
            for port in ports:
                print "[%s] %s " % (count, port)
                count = count + 1
            print colored(("Default Selection: [%s]  "% (ports[-1])), 'blue', attrs=['bold'])
    
            try:
                port_choice =(raw_input("Choice: "))
                if port_choice == "":
                    port_choice = -1 #Enter was hit as the default port was selected
                    return port_choice, ports
    
                else: #A port number was selected
                    port_choice = (int(port_choice)-1)
                    if port_choice >= len(ports):
                        raise ValueError()
                    return port_choice, ports
                
            except ValueError:
                print colored("Invalid Choice.... Try Again...\n", 'red')
            
    port_selected, ports = pick_port()
    
    
    try:
        s = IdentifyBoard(ports[port_selected])
        return(s)
    except:
        print colored("[ERROR] Opening Serial Port", 'red')
        pick_port()
    

def ser_term():
    """Main Serial terminal function"""
    s.writelines("\n") #Get a prompt in the serial recv buffer
    
    tmpBuf = s.readall().rstrip().lstrip()  #This is to capture the recv buffer to a var to put it on the same line as the raw_input
    if tmpBuf == "":
        #Print a prompt to identify GRBL is ready
        print "GRBL Ready>"
    while 1 :

        #Simple prompt fix to catch the right type of prompt string
        if type(tmpBuf) == str:
            prompt = tmpBuf
        else:
            tmpBuf = tmpBuf[-1]


        # get keyboard input
        input = raw_input(colored(tmpBuf, 'green'))
        
        ########################################################
        ## Parse Commandline Commands
        if input == 'exit':
            """Exit the loader"""
            print colored("Exiting....", "grey")
            sys.exit()
            
        elif input.find("load") >= 0:  #Look for the #FILE# string
            """loading a gcode file function called"""
            loadFile(input)
        ########################################################    
            
        else:
            s.write(input + "\n")
            if s.inWaiting > 0:
                tmpBuf =  s.readall().split('\n')
                for bufLine in tmpBuf:
                    if bufLine == tmpBuf[-1]:
                        pass
                    else:
                        print bufLine
def loadFile(input):
    
    def processFileXOFF(f):
        """Used Grep Flow Control"""
        try:
            for line in f.readlines():
                #time.sleep(.25) #Not cool man
                s.writelines(line)
                y = s.readline()
                #print y.rstrip()
                if y != "":
                    if "ok" in y:
                        print "\t"+y.rstrip().lstrip()
                    else:
                        print "\t"+y.rstrip().lstrip()
        except KeyboardInterrupt: #catch a CTRL-C
            s.writelines("!\n")
            print colored("EMERGENCY STOP SENT!", "magenta")
            ser_term()
                
    def processFileXON(f):
        """Use TinyG Flow Control"""
        for line in f.readlines():
            s.writelines(line)
            y = s.readline()
            print y.rstrip()

        
    try:
        input = input.split("load")[-1].rstrip().lstrip() #Parse out the path/to/file.gc
        f = open(input, 'r')
    except IOError:
        print colored("[!] Error opening file... Check your file path....", "red")
    #Begin Processing the Gcode File
    print colored("[*] Begin Processing Gcode File", "magenta")
    #Check to see if flow control is enabled
    if XON == True:
        processFileXON(f)

    elif XON == False:
        processFileXOFF(f)

    
def main():
    try:
        
        init()  #Windows Terminal Color Support.. Just ignore this
        print colored(logo, 'blue') #Print the TinyG Console Logo
        global s, XON
        XON = False  #Set this to test out XON XOFF MODE (not tested)
        
        s = init_serial()
        #s = serial.Serial(ports[port_choice], 115200, timeout=.5) #Setup Serial port COnnection
        #s.xonxoff = True #Turn on software flow control.  TinyG supports this
        usage()  #display #FILE# usage
        ser_term() #Run the terminal function
    except OSError:
        print colored("TinyG disconnected or powered down during operations... Exiting Hard...", "red")
        sys.exit()
        

if __name__ == "__main__":
    main()
