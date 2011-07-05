#!/usr/bin/python
from commands import *
"""Psuedo CRC"""



        
    
    ##def updateCRC(self):
        ##data = self.Payload
        ##crc = 0
        ##for data in byte_array_input:
            ##crc = (crc^data) & 0xff
            ##for y in range(8):  
                ##if (crc & 0x01) != 0:
                    ##crc = ((crc >> 1) ^ 0x8c) & 0xff
                ##else:
                    ##crc = (crc >> 1) & 0xff
           
    ##print "CRC for %s is: %s" %  (byte_array_input,(hex(crc)))
    

    

def getCRC(byte_array_input):
    crc = 0
    for data in byte_array_input:
        crc = (crc^data) & 0xff
        for y in range(8):  
            if (crc & 0x01) != 0:
                crc = ((crc >> 1) ^ 0x8c) & 0xff
            else:
                crc = (crc >> 1) & 0xff
    return crc

##class interact(object):
    ##try:
        ##import rlcompleter, readline
        ##READLINE = 1
    ##except ImportError:
        ##print("Unable to import the python libreadline or completer")
        ##READLINE = 0
        
    ##print "Welcome To the Python Command Interface for the Reprap Extruder"
    ##print "or... PCIRE"
    
    ##try:
        ##import IPython
        ##IPYTHON = True
    ##except ImportError:
        ##print ("Ipython Not Available.")
        
    ##if IPYTHON:
        ##ipshell = IPython.Shell.IPShellEmbed(banner="Using the IPYTHON Shell")
        ##ipshell()
        



##IN = bytearray(3)
##IN[0] = 0xff
##IN[1] = 0x70
##IN[2] = 0xff  
##getCRC(IN)

#print "DONE"
##x = interact()

##START_BYTE = 0xd5
##SLAVE_ID = 0xff
##MSG_LENGTH = 0x04
##COMMAND  = 0x76
##REGISTER = 0x00
##BLINK_NUMB = 0xff

##SLAVE_PAYLOAD = [SLAVE_ID, COMMAND, REGISTER, BLINK_NUMB]
###PACKET = [START_BYTE, MSG_LENGTH, SLAVE_PAYLOAD, CRC]


##tmp = bytearray(4)
##tmp[0] = SLAVE_ID
##tmp[1] = COMMAND
##tmp[2] = REGISTER
##tmp[3] = BLINK_NUMB
##getCRC(tmp)


#fanOn = bytearray(3)
#fanOn[0] = 0xff
#fanOn[1] = 12
#fanOn[2] = 1 #0b00000001 
#getCRC(fanOn)
