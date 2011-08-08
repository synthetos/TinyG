#!/usr/bin/python

import sys
try:
    import serial
except ImportError:
    print("[ERROR] Importing the serial module.\n\tGo to http://pyserial.sourceforge.net/ and install the serial python package.")
    print("Exiting...")
    sys.exit()
    

import ctypes
import os
import platform

from cmds.cmds import ls, fgcode, loadFile, zero

from logo import logo
from ser.serial_init import Serial

global OPTIONS
OPTIONS = {
           "SPEEDS":[9600,115200],
           "DEFAULT_GCODE_DIR":"./gcode_samples"  }



class Console(object):
    def __init__(self):
        print logo
        self.usage()
        self.ser = Serial()
        self.ser.pick_port()
        self.autoConfigure()
        self.prompt()
    
    def usage(self):
        print """\nTinyG / Grbl Console Loader..... 
[*]Commands Supported:
    load filename.gcode - Runs a file through TinyG / Grbl
    fgcode ./gcode_samples - Scans a directory for gcode files and lets the user select a file to be ran.
    zero - shortcut for G92 x0 y0 z0
    ls dir_name - lists all files (like ls in *nix)\n"""
    
    def autoConfigure(self):
        """These commands are used to get TinyG applied with default settings at start"""
        
        if self.ser.board.lower() == "tinyg":
            """TinyG Board Auto Configure"""
            print("[*]Running Autoconfigure Commands")
            print "\t[#]Disabling Local Command Echo"
            print "\t[#]Disabling XON Flow Control"
            print "[*]Done"
            self.ser.s.writelines("$ee0\n") #Disable Local Echo
            self.ser.s.writelines("$ex1\n") #Disable Flow Control
        
        elif self.ser.board.lower() == "grbl":
            """Placeholder for auto configuration GRBL"""
            pass
    
    def prompt(self):
        """Main Serial terminal function"""
        s = self.ser.s
        s.writelines("\n") #Get a prompt in the serial recv buffer
        
        tmpBuf = s.readall().rstrip().lstrip()  #This is to capture the recv buffer to a var to put it on the same line as the raw_input
        if tmpBuf == "":
            #Print a prompt to identify GRBL is ready
            s.flush()
            print "GRBL Ready>"
        
            
        #Simple prompt fix to catch the right type of prompt string
        if type(tmpBuf) == str:
            
            #This hides the autoConfigure response for the TinyG board.
            if tmpBuf.startswith("Enable"):
                tmpBuf = tmpBuf.split("> ")[-1]
           #Done Hiding
           
            
           
            prompt = tmpBuf
        else:
            tmpBuf = tmpBuf[-1]
            
        while 1 :
    
            
    
    
            # get keyboard input
            input = raw_input(tmpBuf)
            input = input.lower()
            ########################################################
            ## Parse Commandline Commands
            if input == 'exit':
                """Exit the loader"""
                print colored("Exiting....", "grey")
                sys.exit()
                
            elif input.find("load") >= 0:  #Look for the load string
                """loading a gcode file function called"""
                loadFile(input, s)
            
                
            elif input.find("fgcode") >= 0: #look for gcode dir
                fgcode(input,s)
                
            elif input.find("ls") >= 0:
                ls(input,s)
                
            elif input.find("zero") >= 0:
                zero(input, s)
                
            ########################################################    
                
            else:
                s.write(input + "\n")
                if s.inWaiting > 0:
                    tmpBuf =  s.readall().split('\n')
                    for bufLine in tmpBuf:
                        if bufLine == tmpBuf[-1]:
                            tmpBuf = bufLine
                        else:
                            print bufLine
                    
        

        
x = Console()