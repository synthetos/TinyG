#!/usr/bin/python
#Packet
from crc import getCRC
import serial

"""
PAYLOAD = SLAVE_ID, COMMAND, ARGS
"""

EQ_TOGGLE_FAN = {"name":"Toggle Fan",
                 "command":12, "args":1,
                 "p_length":1, 
                 "r_length":0,
                 "args_choices":[0b00000001, 0b00000000]}

class Packet(object):

    def createPayload(self, SLAVE_ID=0xff):
        self.payload = bytearray()
        self.payload.append(SLAVE_ID)
        self.payload.append(self.command.code)
        self.payload.append(self.command.args)
        return self.payload


    def __init__(self, command=None, payload=None):
        self.START_BYTE = 0xd5  #This is set for the protocol default
        self.command = Command(EQ_TOGGLE_FAN)
        self.payload = self.createPayload()
        self.crc = getCRC(self.payload)
        self.s = serial.Serial("/dev/cu.usbserial-A700eE6u", 38400)
        self.s.timeout = 2



    def getLength(self):
        self.length = COMMANDS["self.name"]



    def sendPacket(self):
        pay = ""
        psend = []
        psend.append(self.START_BYTE)
        psend.append(len(self.payload))#lenght of payload
        for b in self.payload:
            psend.append(b)
        psend.append(self.crc)
        for a in psend:
            print hex(a)
            self.s.write(chr(a))
        print "DONE"





class Command(object):

    def __init__(self, cmd):
        self.name = cmd["name"]
        self.code = cmd["command"]
        self.args = cmd["args"]
        self.p_length = cmd["p_length"]
        self.r_length = cmd["r_length"]







    def __repr__(self):
        return('<Command Name: %s, Command: %s, Args: %s, Payload Length: %s, Response Length: %s>' 
               % (self.name, self.code, self.args, self.p_length, self.r_length))



p = Packet(EQ_TOGGLE_FAN)
p.sendPacket()
print 23