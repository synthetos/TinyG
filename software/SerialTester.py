#!/usr/bin/env python
import serial, sys, time
from random import randint
from time import sleep
#TinyG Serial Testing Program
#Synthetos.com
#Alden Hart, Riley Porter


"""This program is to diagnose sending MANY gcode lines to the TinyG Board.  If the compiled version of TinyG was
built with the RX_BUFF set to 18 you can send about 5 lines of gcode (provide the TestingRig with a count of 5.. IE: TestingRig(5, ser)  )
without any problems.  Once you send more.. Say 25 then you will execute 5 lines then stop.  I am not sure why this is at the moment.

KNOWN:
If you are to recompile TinyG to use a RX_BUFF as 255 then you can do more commands (counts) before locking up.  Many more.  This seems to point to my logic for detecting
the delimeter or something going on in the control loop.  Not sure which.
"""

#Set your's Here
SERIAL_PORT = '/dev/cu.usbserial-A700eUQq'

try:
	#Open the serial Port.  Set the serial port for your system in the SERIAL_PORT var above.
	ser = serial.Serial(SERIAL_PORT,  115200, timeout=.01)
except:
	print "[!!]ERROR Opening Serial Port.. Exiting..."
	sys.exit()

print "[*]TinyG Serial Tester v1.0a"
			
def TestingRig(count, ser):
	"""This is a testing function that will go one way then back to 0 on  all axis.  The count varible is how many times you want the loop to run.
	Each count represents 1 gcode command"""
	current  = 1 #Start loop counter
	DIR = 0      #A flag to keep state of which direction was just sent.
	while current <= count:	
		if DIR == 0:
			x = 0
			y = 0
			z = 0
			DIR = 1
		else:
			x = -5
			y = -5
			z = -5
			DIR = 0

		feed = 333  #Static set feed
		print "g1 x%s, y%s, z%s f%s" % (x,y,z, feed)     #Print the gcode line to stdout that will be sent
		ser.write("g1 x%s y%s z%s f%s\n" % (x,y,z,feed)) #Send the gcode line
		current = current+1 #Increment loop counter
		#time.sleep(.2) #This was a sanity testing delay.  By enabling this it seems to make it through the whole loop
		delim = ser.read()
		while delim != "*":  #while * is not in the delimeter
			delim = ser.read() #read a char from the serial port.. In theory if it * it should break the for loop
	else:  #Will run when all "counts" have been sent thus exiting the program
		print "Done"			
		sys.exit()


#This will run the TestingRig program.  Provide Various "counts" by changing the number: IE: TestingRig(255)
#You can also speed things up by changing how many "steps" it will take by chaning -5 to say -2 or so on.
TestingRig(10, ser)

