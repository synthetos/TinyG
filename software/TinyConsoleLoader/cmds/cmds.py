#!/usr/bin/python
import os
import time
#from TinyConsole import DEFAULT_GCODE_DIR

global DEFAULT_GCODE_DIR
DEFAULT_GCODE_DIR = "./gcode_files"



def loadFile(input, self):
    s = self
    def processDelimiter(f):
        """Used Grep Flow Control"""
        totalLines = 0
        count = 0
        
        #Get file stats
        for l in f.readlines():
            count = count + 1
        totalLines = count
        f.seek(0)  #reset file to zero'th line
        
        
        statsCount = 0
        for line in f.readlines():
            try:
                statsCount = statsCount + 1
                time.sleep(.1) #Not cool man
                s.writelines(line)
                y = s.readline()
               
                #print y.rstrip()
                
                if y != "":
                    percentageComplete =  "%.f" % (100 * float(statsCount)/float(totalLines))
                    if "ok" in y:
                        print "\t"+y.rstrip().lstrip() + " - %s%s of Job Complete" % (percentageComplete,"%")
                    else:
                        print "\t"+y.rstrip().lstrip() + " - %s%s of Job Complete" % (percentageComplete,"%")

        
            except KeyboardInterrupt: #catch a CTRL-C
                s.writelines("!\n")
                print("EMERGENCY STOP SENT!")
                return
            
        print "[*]Sending File Complete:"
    
    def processFileXON(f):
        """Use TinyG Flow Control XON/XOFF"""
        for line in f.readlines():
            s.writelines(line)
            y = s.readline()
            print y.rstrip()
            
            
    #Check to see if Flow control is enabled
    XON = s.xonxoff
    #Check file exists and is abled to be read
    try:
        input = input.split("load")[-1].rstrip().lstrip() #Parse out the path/to/file.gc
        f = open(input, 'r') #Attempt to open file
    except IOError:
        print ("\t[ERROR]Opening %s... Check your file path...." % input)
        return
    
    #Begin Processing the Gcode File
    print ("[*] Begin Processing Gcode File")
    
    if XON == False:  #if xon is off call processDelimiter
        processDelimiter(f)
    elif XON == True: #if xon is on call processXON
        processFileXON(f) 

        
def ls(cmd,s ):
    def printFiles(files):
        print ("[#]File Listing:")
        for f in files:
            if f.startswith("."): #Skip hidden files
                pass
            else:
                print ("\t%s" % f)

    cmd = cmd.split(" ")
    if len(cmd) > 1:
        c = cmd[0] #Capture the ls command
        path = cmd[1] #Capture the path if one was supplied
        files =  os.listdir(path)
    else:
        files =  os.listdir(".")
    printFiles(files)
    
    
def zero(cmd, s):
    """Shortcut for a g19 zero command"""
    try:
        print("[*]Zeroing..")
        s.writelines("g92 x0 y0 z0 a0\n")
    except Exception, e:
        print "[ERROR]Running the zero command"
        
        
    

def fgcode(cmd, s):
    """load gcode files in a dir"""

    try:
        
        def selectFileToRun(fileToCount):
            """Gets a file from the user and checks to see if its valid"""
            fileToRun = raw_input("Select a gcode file to run:\nType quit to return to main menu.\n\tChoice: ")
            if fileToRun.isdigit() and int(fileToRun) in range(0,len(fileToCount)):  #check to see if input was a number
                #selection was a number and was in the range of files found
                loadFile(fileToCount[int(fileToRun)], s)    
            
        cmd = cmd.split(" ") #Parse command (if needed)
        path = DEFAULT_GCODE_DIR 
        if len(cmd) > 1:
            path = cmd[1] #Capture the path of gcode files
        gfiles = os.listdir(path)  #List the files
        newGcodeFiles = []
        for f in gfiles:
            if f.endswith(".gc") or f.endswith(".gcode") or f.endswith(".txt"):
                newGcodeFiles.append(path+"/"+f)
        count = 0
        
        fileToCount = {} #Dict to hold files and map to count number
        if len(newGcodeFiles) > 0:
            print ("[%s]Gcode Found Files:" % len(newGcodeFiles))
            for gcodefile in newGcodeFiles:
                print "\t[%s] - %s" % (count, gcodefile)
                fileToCount[count] = gcodefile
                count = count +1
            return(selectFileToRun(fileToCount))
        
        else:
            print("[0] Gcode Files Found:")
    except Exception, e:
        print("[ERROR] Running fgcode Command: %s" % e)
        
            