#!/usr/bin/env python
import extern.serial as serial
import sys
import getopt

#Diagnostic Python Program
#TinyG


class Walker(object):
	def __init__(self, PORT, MAXWALK):
		self.DELIM = ""
		self.MAXWALK = MAXWALK #This is how many times it will go one way then the other.. Defaults to 500
		self.COUNT = 1
		self.SPEED=115200
		self.SER = serial.Serial(PORT, self.SPEED, timeout=.1)
		
		
	def walk(self, steps, cmd, feedrate):
		"""<steven tyler voice>Will walk this way</steven tyler voice>
		 then the other"""
		 
		self.DELIM = ""
		self.DIR = 1
		self.STEPS = steps
		self.COUNT = 1
		self.FEEDRATE = feedrate
		while 1:
			"""Checks to see if its walked the MAXWALK number of times."""
			if self.COUNT == self.MAXWALK:  
				print "[#]Walking Complete\n"
				break
				
			print "[#]Walking %s:%s" % (self.COUNT, self.MAXWALK)
			self.COUNT = self.COUNT + 1
			
			"""Run the initial Command."""
			if self.DIR == 0:
				steps = steps ^ self.STEPS #XOR steps so that is will go do something
				self.DIR = 1 #Change the directions
				print "[#]LEFT: %s x%s y%s z%s %s" % (cmd, steps, steps, steps, self.FEEDRATE)
				self.SER.write("%s x%s y%s z%s %s \n" % (cmd, steps, steps, steps, self.FEEDRATE))
			else:
				self.DIR = 0 #Change the directions
				steps = steps ^ self.STEPS #XOR steps so that is will go do something
				print "[#]RIGHT: %s x%s y%s z%s %s" % (cmd, steps, steps, steps, self.FEEDRATE)
				self.SER.write("%s x%s y%s z%s %s \n" % (cmd, steps, steps, steps, self.FEEDRATE))
			#sleep(.2)
			while 1:
				if self.DELIM != "*":
					#print "Delim Not Found"
					self.DELIM = self.SER.read()
				else:
					print "FOUND %s" % self.DELIM
					break
def usage():

	print "[INFO] Invalid Commandline Arguments"
	print "[USAGE] python Walker.py <serialport> <speed>"
	print "[EXAMPLE] python Walker.py COM12 115200"
	
def main(argv=None):
	print "[#]TinyG Python Walker Diagnostic Program\n"
	try:
		serport= sys.argv[1]
		
	except IndexError:
		usage()
	
	
	#Serial Port Config
	SPEED = 115200
	PORT = serport #passed from cmdline
	
	
	x = Walker(PORT, 2)  #Init the object
	x.walk(6, "g1", 300) #This method can be overloaded like so:
	#x.walk(steps_to_take, gcodeCmd, feedrate)
	#IE:
	#x.walk(20, "g0", 300)
	
			
if __name__ == "__main__":
	main()
	

	

