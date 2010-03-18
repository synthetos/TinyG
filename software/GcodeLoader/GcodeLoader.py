#Python Gcode loader Script:

import serial, sys
from random import randint
from time import sleep

ser = serial.Serial('COM12',  115200, timeout=.01)
print "[*]TinyG Gcode Loader v1.0a"

def breakIn(STEPS, COUNT, AXIS):
	STATE = 0
	STEPS_CONST = STEPS
	
	"""This moves whatever axis back and forth the specififed count"""
	delim = "*"
	for tmp in range(0,COUNT):
		if STATE == 0:
			STEPS = STEPS * -1
			STATE = 1
		else:
			STEPS = STEPS_CONST
		print "[*]Running: %s on %s" % (STEPS, AXIS)
		ser.write("g1 %s %s F333 \n" % (AXIS, STEPS) )
		delim = ser.read()
		while "*" not in delim:
			delim = ser.readline()
			if delim == "":
				#print "[*]Sending Newline"
				ser.write("\n")
				delim = ser.readline()
			
def randCode(count):
	"""Will create random g0 gcode lines"""
	COUNT = 1
	delim = ""
	for x in range(count):
		print "[*]Running: %s" % COUNT
		COUNT += 1
		x = randint(0,20)
		y = randint(-10,10)
		z = randint(0,10)
		#print "[*]Gcode Command:\n"
		print "g0 x%s, y%s, z%s" % (x,y,z)
		ser.write("g0 x%s y%s z%s F333 \n" % (x,y,z))
		delim = ser.readline()
		while "*" not in delim:
			delim = ser.readline()
			if delim == "":
				#print "[*]Sending Newline"
				ser.write("\n")
				delim = ser.readline()
			
		

def Run():
	"""THIS IS NOT WORKING RIGHT, ALDEN IS CHANGING THE SERIAL RESPONSE A BIT"""
	#filename = "hacdc.gcode"
	filename = "song.gcode"
	
	f = open(filename, 'r')
	delim = ""
	for line in f:
		print "[*]Writing: %s" % line
		ser.write(line)
		delim = ser.read()
		print delim
		while delim != "*":
			#print "Delim Not Found"
			delim = ser.read()
			
		
#Run()		
randCode(50)
#breakIn(10, 10, "X")
