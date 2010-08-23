#!/usr/bin/env python
#Serial Port File Loader Program

def printMsg(msg, msgbox):
	msgbox.AppendText(msg)
	return

def processFile(DebugTxtBox, ser, f ):
	"""Pass a serialport and the file object"""
	try:
		ser.write(3*"\n") #This might not be need anymore but it just sends a few \n down the wire to clear anything before sending gcode
	except serial.serialutil.SerialException:
		print "[ERROR] Write Failed"
		print "System Exiting"
		sys.exit(6)
	delim = ""
	for line in f:
		line = line.rstrip() #remove trailing new line character
		printMsg(("[*]Writing: %s" % line), DebugTxtBox)
		ser.write(line+"\n") #send a line of gcode
		delim = ser.read() #check to see if we found our delimeter '*' 
		while delim != "*":  #Loop until we find the delim
			delim = ser.read()


