#Python Gcode loader Script:

import serial, sys
from random import randint
from time import sleep

ser = serial.Serial('COM12',  115200, timeout=.1)
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
		x = randint(-20,20)
		y = randint(-20,20)
		z = randint(-20,20)
		#print "[*]Gcode Command:\n"
		print "g1 x%s, y%s, z%s" % (x,y,z)
		ser.write("g0 x%s y%s z%s F300 \n" % (x,y,z))
		delim = ser.readline()
		while "*" not in delim:
			delim = ser.readline()
			if delim == "":
				#print "[*]Sending Newline"
				ser.write("\n")
				delim = ser.readline()

def sendCode(code, axis, steps, feedrate):		
	print "[#]SENDING: %s %s %s %s" % (code, axis, steps, feedrate)
	ser.write("%s %s%s %s\n" % (code, axis, steps, feedrate))
			
			
			
class Walker(object):
	def __init__(self, MAXWALK=4):
		self.DELIM = ""
		self.MAXWALK = MAXWALK #This is how many times it will go one way then the other.. Defaults to 500
		self.COUNT = 1
		
		
	def walk(self, steps, code, feedrate):
		"""<steven tyler voice>Will walk this way</steven tyler voice> then the other"""
		self.DELIM = ""
		self.DIR = 1
		self.STEPS = steps
		while 1:
			"""Checks to see if its walked the MAXWALK number of times."""
			if self.COUNT >= self.MAXWALK:  
				print "[#]Walking Complete\n"
				break
				#sys.exit()
				
			print "[#]Walking %s:%s" % (self.COUNT, self.MAXWALK)
			self.COUNT = self.COUNT + 1
			
			"""Run the initial Command."""
			if self.DIR == 0:
				#self.COUNT = self.COUNT + 1
				steps = steps ^ self.STEPS
				self.DIR = 1 #Change the directions
				print "[#]LEFT: %s x%s y%s z%s %s" % (code, steps, steps, steps, feedrate)
				ser.write("%s x%s y%s z%s %s \n" % (code, steps, steps, steps, feedrate))
				self.DELIM = ser.readline()
			else:
				#self.COUNT = self.COUNT + 1
				self.DIR = 0 #Change the directions
				steps = steps ^ self.STEPS
				print "[#]RIGHT: %s x%s y%s z%s %s" % (code, steps, steps, steps, feedrate)
				ser.write("%s x%s y%s z%s %s \n" % (code, steps, steps, steps, feedrate))
				self.DELIM = ser.readline()
			#sleep(.2)
			while 1:
				if self.DELIM.find("*") == 0:
					print "FOUND %s" % self.DELIM
					break
					
				else:
					self.DELIM = ser.read()
					



def Run():
	"""THIS IS NOT WORKING RIGHT, ALDEN IS CHANGING THE SERIAL RESPONSE A BIT"""
	#filename = "hacdc.gcode"
	#filename = "song.gcode"
	#filename = "pcb.gcode"
	filename = "circles.gcode"
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
			


x = Walker(232)
x.walk(10, "g1", "f300")
#sendCode("G0", "x",-85, "F300")
#Run()		
#randCode(80)
#breakIn(10, 10, "X")
