#!/usr/bin/env python
#Serial Port Initializer for TinyG
import sys
import ctypes
import os
import platform
import serial
import time
        
    
class Serial(object):
    def __init__(self):
        self.PORTS = self.scan()
        
    
    def scan(self):
        """ Scan for available self.PORTS. return a list of device names. """
        if platform.system() == 'Windows':
            #print "[*]Scanning for Windows Serial self.PORTS"
            import scanwin32
            self.PORTS = []
            for order, port, desc, hwid in sorted(scanwin32.comself.PORTS()):
                self.PORTS.append(port)
            return self.PORTS
        elif platform.system() == "Darwin":
            #print "[*]Scanning for OSX Serial self.PORTS"
            self.PORTS = []
            import scanosx
            return scanosx.scan()
        else:
            #print "[*]Scanning for Linux Serial self.PORTS"
            import scanlinux
            return scanlinux.scan()

        
        
    def pick_port(self,msg=""):  #msg is used to prepend "Re-" when the serial ports are needed to be scanned again
        print("[#]%sScanning Serial Ports:" % msg)
        while(1):
            if len(self.PORTS) == 0:
                self.scan()
                #self.PORTS_mode()
    
            print "[?]Select a Serial Port:"
            count = 1
            for port in self.PORTS:
                print "    [%s] %s " % (count, port)
                count = count + 1
            print("Default Selection: [%s]  " % (self.PORTS[-1]))
    
            try:
                self.port_choice =(raw_input("Choice#> "))
                if self.port_choice == "":
                    self.port_choice = -1 #Enter was hit as the default port was selected
                    self.port_selected = self.PORTS[self.port_choice]
                    break
                
                else: #A port number was selected
                    self.port_choice = (int(self.port_choice)-1)
                    

                    if self.port_choice >= len(self.PORTS):
                        raise ValueError()
                    self.port_selected = self.PORTS[self.port_choice]
                    break
                    
            except ValueError:
                print("\t[!]Invalid Choice.... Try Again...\n")
                
        while(1): 
            try:
                self.s, self.board, self.speed = self.IdentifyBoard()
                return
            except serial.SerialException:
                print("[ERROR]Opening Serial Port")
                self.pick_port()
            except TypeError:
                print "[ERROR]Opening %s... Select a different port....\n" % self.port_selected
                sys.exit()
            
                
                
                
                
                
    def IdentifyBoard(self):
        """Need to cycle thorugh all speeds in a list for each board  TODO"""
                
        SPEEDS = {"GRBL":9600,
                  "TINYG":115200
                  }
        
        for tmpPort,tmpSpeed in SPEEDS.items():
    
            print "\t[#]Trying %s at Speed %s" % (tmpPort, tmpSpeed)
            try:
                s = serial.Serial(self.port_selected, tmpSpeed, timeout=.025)
            except serial.SerialException:
                print "Error connecting to %s at %s" % (self.port_selected, tmpSpeed)
                continue
            
            s.writelines("\n")
            readbuf = s.readall()
            if readbuf == "":  #Possibly GRBL
                s.writelines("$\n")
                readbuf = s.readall()
                if "ok" in readbuf:
                    print("\t[#]Auto Board Identification selected %s:%s\n" % (tmpPort,tmpSpeed))
                    return(s, tmpPort, tmpSpeed)
            elif "ok" in readbuf:
                print("\t[#]Auto Board Identification selected %s:%s\n" % (tmpPort,tmpSpeed))
                return(s, tmpPort, tmpSpeed)
            
        
            if s is None:
                #No TinyG / GRBL board was found.
                print "[#]Auto Detection has failed."
                print "[!]System will now exit.. Verify your board connections are secure and power is on."
                #sys.exit()
                return
    
        
    