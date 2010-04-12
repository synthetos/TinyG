import serial, sys
from random import randint
from time import sleep

SERIAL_PORT = "COM17"


ser = serial.Serial(SERIAL_PORT,  115200, timeout=.01)
print "[*]TinyG Gcode Loader v1.0a"


def randCode(count):
	COUNT = 1
	delim = ""
	for x in range(count):
		print "[*]Running: %s" % COUNT
		COUNT += 1
		x = randint(0,10)
		y = randint(-10,10)
		z = randint(0,10)
		#print "[*]Gcode Command:\n"
		print "g0 x%s, y%s, z%s" % (x,y,z)
		ser.write("g0 x%s y%s z%s \n" % (x,y,z))
		delim = ser.readline()
		while "*" not in delim:
			delim = ser.readline()
			if delim == "":
				#print "[*]Sending Newline"
				ser.write("\n")
				delim = ser.readline()
			
		

def Run():
	ser.write(3*"\n")
	#filename = "circles.gcode"
	#filename = "freeze.gcode"
	#filename = "hacdc.gcode"
	#filename = "song.gcode"
	filename = "test.NC"
	
	f = open(filename, 'r')
	delim = ""
	for line in f:
		line = line.rstrip()
		print "[*]Writing: %s" % line
		ser.write(line+"\n")
		delim = ser.readline()
		while "*" not in delim:
			delim = ser.readline()
		


Run()		
#randCode(10)
