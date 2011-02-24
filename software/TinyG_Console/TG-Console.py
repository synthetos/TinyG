#!/usr/bin/env python
#Gcode Command Line Loader for TinyG
#synthetos.com
#Riley Porter

import serial, sys, getopt, time
from ser.serial_init import scan
print "############################"
print "[*]TinyG Console v1"
print "############################"


class Console(object):
    def __init__(self):
        self.SPEED = 115200
        self.init_serial()
        self.choose_mode()


    def scanports_mode(self):
        """This mode is called if no serial ports were detected during startup"""
        print "\n--------------------------"     
        print "Scan Ports Mode:"
        print "--------------------------"
        print "No serial ports were detected on your system or TinyG was disconnected."
        print "If you would like to scan again press enter.  Otherwise type quit to end the program"
        while(1):

            scan_choice =(raw_input("Choice: "))
            if scan_choice == "":
                self.init_serial()
                break
            elif scan_choice == 'quit':
                print "Exiting Program"
                sys.exit()
            else:
                print "Invalid Choice: try again"  
        return port_choice, ports #return the port_choice number and all the ports

    def choose_mode(self):
        """Root mode choice menu"""
        print "\n--------------------------"     
        print "Mode Selection Menu:"
        print "--------------------------"
        print "1 = Manual Mode"
        print "2 = Config Mode"
        print "3 = Gcode Mode"
        print "Default [1] - enter to confirm:"


        mode_choice =raw_input("TinyG[IDLE]#> ").lstrip().rstrip()
        #Select Which Mode to go to
        if mode_choice == "":
            self.manual_mode()
        elif mode_choice.rstrip() == "1":
            self.manual_mode()
        elif mode_choice.rstrip() == "2":
            self.config_mode()
        elif mode_choice.rstrip() == "3":
            self.gcode_mode()


    def config_mode(self):
        MODE = "CONFIG"  #Used to track which mode we are in
        self.change_modes(MODE)  #Change the mode
        
        print "\n--------------------------"     
        print "TinyG Config Mode:"
        print "--------------------------"     

        print "Type ? to list current settings"
        print "Type ?x ?y ?z or ?a to get axis specific settings"
        print "--------"
        print "Example:"
        print "--------"
        print "ASE700\nWould change the A axis's Seek Steps / sec to 700\n"
        self.usage()
        print "Enter a Command:"

        self.ser_term(MODE)


    def display_help(self):
        self.ser.write("h" + "\n")
        # send the characters to TinyG
        out = ""
        time.sleep(.5) #Sleep a bit for an answer
        while self.ser.inWaiting() > 0:
            cur = self.ser.read(1)
            if cur == "\r":
                pass
            else:                        
                out += cur
        print out
        
    def usage(self):
        print "Basic Commands are:"
        print "\texit - Leave current mode and return to mode selection."
        print "\tquit - Quit the program"
        print "\thelp or h - Display the help menu"
        print "\t? - for general mode information"

    def gcode_mode(self):
        MODE = "GCODE"
        self.change_modes(MODE)

        print "\n----------------------------------------"     
        print "TinyG Gcode File Loader Mode:"
        print "----------------------------------------"     
        print "Type exit to quit to mode selection"
        print "Specify Gcode File to Run"
        print "\tExample:" 
        print "\t\t/home/user/test.gcode or ../test.gcode or test.gcode"
        print "\t\tC:\gcode_files\girl.ngc or ../test.gcode or test.gcode"
        self.usage()
        print "Please enter a path/to/gcode/file"
        while(1):
            """GCODE MODE FILE SELECTION LOOP"""
            self.FILE = raw_input("TinyG[GCODE]#> ").lstrip().rstrip()
            if self.FILE == "exit": #Leave the gcode loader mode.  Return to the root choice menu
                self.choose_mode()
            elif self.FILE == "help" or self.FILE == "h":
                self.display_help()
                self.gcode_mode()
            elif self.FILE == 'quit':
                print "Quitting Program"
                self.ser.close()
                sys.exit()

            try: #check file exists
                f = open(self.FILE, 'r')
                break
            except:
                print "Error Opening %s Gcode File" % self.FILE
                print "Please try again or type exit to return to main choice menu."
        self.processFile() #Run the gcode file through tinyg
        
        while(1):
            print "\n-------------------------"     
            print "File Processing Complete..."
            print "Run Again? [Y/n]"
            again = raw_input("TinyG[GCODE]#> ").lstrip().rstrip()
            if again == "":
                self.processFile()
            elif again == "Y" or again == "y":
                self.processFile()
            else:
                self.gcode_mode()

    def processFile(self):
        ser = self.ser
        f = open(self.FILE, 'r')
        debug = 1
        """
        DO NOT REMOVE - Start Important Code
        """
        try:
            ser.write(3*"\n") #Initializes GCODE MODE to start accepting commands
        except Exception, e:
            print "[ERROR] Write Failed"
            print "Kicking you into scanport mode to re-connect to TinyG"
            self.scanports_mode()
        """
        END Important Code
        """
        delim = ""
        for line in f:
            line = line.rstrip() #remove trailing new line character
            if debug == 1:
                print "%s" % line
            ser.write(line+"\n") #send a line of gcode
            delim = ser.read() #check to see if we found our delimeter '*'
            response = delim
            while delim != "*":  #Loop until we find the delim
                delim = ser.read()
                response += delim
        return

    def get_coords(self):
        self.ser.write("?\n")
        coord_lines = []
        out = ''
        time.sleep(.2) #Sleep a bit for an answer
        while self.ser.inWaiting() > 0:
            cur = self.ser.read(1)
            if cur == "\r":
                pass
            else:                        
                out += cur
        out = out.split("\n")
        print "\n--------------------"
        print "TinyG Axis Positions"
        for line in out:

            if line.find("Position") != -1:         
                coord_lines.append(line)
        for coord in coord_lines:
            print coord
        pass

    def init_serial(self):
        #Get Serial Port Options
        port_selected, ports = self.pick_port()
        if port_selected == False:
            while(port_choice == False):            
                port_selected, ports = self.pick_port()
        try:
            self.ser = serial.Serial(ports[port_selected], self.SPEED, timeout=1)
        except:
            print "[ERROR] Opening Serial Port"
            sys.exit()
        print "Selected: %s" % ports[port_selected]
        print "Connection Established Successfully"
        return self.ser
    
    def pick_port(self):
        print "Scanning Ports:\n"
        while(1):
            ports = scan()
            if len(ports) == 0:
                self.scanports_mode()
                
            print "Select a Serial Port:"
            count = 1
            for port in ports:
                print "[%s] %s " % (count, port)
                count = count + 1
            print "Default Selection: [%s]  "% (ports[-1])
            try:
                port_choice =(raw_input("Choice: "))
                if port_choice == "":
                    port_choice = -1 #Enter was hit as the default port was selected
                    return port_choice, ports

                else: #A port number was selected
                    port_choice = (int(port_choice)-1)
                    if port_choice <= len(ports):
                        return port_choice, ports
            except ValueError:
                print "Invalid port_choice\n"
                print "Try Again"
                

    def ser_term(self, MODE):
        while 1 :
            input=1
            # get keyboard input
            input = raw_input("TinyG[%s]*>"%MODE).lstrip().rstrip()
            if input == 'exit':
                print ""
                self.ser.write("q\n")
                time.sleep(.2)
                self.choose_mode()
            elif input == "help" or input == "h":
                self.display_help()
            elif input == 'quit':
                print "Quitting Program"
                self.ser.close()
                sys.exit()

            else:
                # send the characters to TinyG
                self.ser.write(input + "\n")
                if MODE == "GCODE":
                    self.get_coords()  #Run this after every command to get location
                out = ''
                time.sleep(.2) #Sleep a bit for an answer
                while self.ser.inWaiting() > 0:
                    cur = self.ser.read(1)
                    if cur == "\r":
                        pass
                    else:                        
                        out += cur
                print out

    def change_modes(self, MODE):
        # Not sure why this is needed however its the only way 
        # That changing modes in TinyG is working from python?
        if MODE == "MANUAL":
            lower_mode = 'g'
            MODE = "GCODE"  #Dumb but its need.
        elif MODE == "GCODE":
            lower_mode = 'g'
        elif MODE == "CONFIG":
            lower_mode = 'c'

        #Send the initial quit char
        self.ser.write("q"+"\n")

        # The meat of the changing modes function
        while(1):
            tmp = self.ser.read(48)
            if tmp == "":
                self.ser.write(lower_mode+"\n")
            if MODE  in tmp:
                break

    def manual_mode(self):
        MODE = "MANUAL"
        self.change_modes(MODE)

        print "\n--------------------------"     
        print "Entered TinyG Manual Mode:"
        print "--------------------------"     
        self.usage()
        print "Enter a Gcode Command: IE: g0x10y10z5"
        MODE = "GCODE"
        self.ser_term(MODE)


if __name__ == "__main__":
    Console()
