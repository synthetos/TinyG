##!/usr/bin/python

#EQ_TOGGLE_FAN = {"name":"Toggle Fan","code":12,"p_length":[0x01], "r_length":0}
#EQ_GET_VERSION = {"name":"Get Version", "code":0,"p_length":2,"r_length":2}
#EQ_INIT = ("Extruder Init", 1, 0)
#EQ_GET_TEMP = ("Get Temperature", 2, 1)
#EQ_SET_MOTOR1_RPM = 4
#EQ_SET_MOTOR2_RPM = 5

#class Payload(object):
    #def __init__(self):
        #self.payload = ""
        
#class Command(object):

    #def __init__(self, cmd):
        #self.name = cmd["name"]
        #self.code = cmd["code"]
        #self.p_length = cmd["p_length"]
        #self.r_length = cmd["r_length"]
        
    #def __repr__(self):
        #return('<Command Name: %s, Command: %s, Payload Length: %s, Response Length: %s>' % (self.name, self.command, self.p_length, self.r_length))
                             
##Test Commands
##C_ECHO_TEST = 0x70
##C_GENERATE_BAD_RESPONSE = 0x71
##C_SIMULATE_BAD_PACKET_RECEPTION = 0x72
##C_NO_SUCH_COMMAND = 0x75
##C_SET_DEBUG_CODE = 0x76
##C_GET_DEBUG_CODE = 0x77
##C_COMMAND_QUEUE_FILLER = 0xF0

###Extruder Query Commands

##EQ_SET_MOTOR1_SPEED = 6
##EQ_SET_MOTOR2_SPEED = 7
##EQ_SET_MOTOR1_DIR = 8
##EQ_SET_MOTOR2_DIR = 9
##EQ_TOGGLE_MOTOR1 = 10
##EQ_TOGGLE_MOTOR2 = 11

##EQ_TOGGLE_VALVE = 13
##EQ_SET_SERVO1_POSITON = 14
##EQ_SET_SERVO2_POSITION = 15
##EQ_FILAMENT_STATUS = 16
##EQ_GET_MOTOR1_RPMSPEED = 17
##EQ_GET_MOTOR2_RPMSPEED = 18
##EQ_GET_MOTOR1_PWMSPEED = 19
##EQ_GET_MOTOR2_PWMSPEED = 20
##EQ_SELECT_TOOL = 21
##EQ_IS_TOOL_READY = 22
##EQ_PAUSE_UNPAUSE_EXTRUSION = 23
##EQ_ABORT = 24
##EQ_READ_FROM_EEPROM = 25
##EQ_WRITE_TO_EEPROM = 26
##EQ_GET_BUILD_PLATFORM_TEMP = 30
##EQ_SET_BUILD_PLATFORM_TARGET_TEMP = 31
##EQ_GET_BUILD_PLATFORM_TARGET_TEMP = 33

##EQ_GET_EXTRUDER_TARGET_TEMP = 32
##EQ_GET_FIRMWARE_BUILD_NAME = 34
##EQ_IS_BUILD_PLATFORM_READY = 35
##EQ_GET_TOOL_STATUS = 36
##EQ_GET_PID_STATE = 37
##EQ_SET_MOTOR1_DDA_SPEED = 38
##EQ_SET_MOTOR2_DDA_SPEED = 39

    
    ##"""Query Commands
    ##QC_GET_VERSION = 0
    ##C_INIT = 1
    ##C_GET_AVAILABLE_BUFFER_SIZE = 2
    ##C_CLEAR_BUFFER = 3
    ##QC_GET_POSITION = 4
    ##QC_ABORT_IMMEDIATELY = 7
    ##QC_PAUSE_UNPAUSE = 8
    ##QC_TOOL_QUERY = 10
    ##QC_IS_FINISHED = 11
    ##QC_READ_EEPROM = 12
    ##QC_WRITE_EEPROM = 13
    ##QC_CAPTURE_TO_FILE = 14
    ##QC_END_CAPTURE = 15
    ##QC_PLAYBACK_CAPTURE = 16
    ##QC_RESET = 17
    ##QC_GET_NEXT_FILENAME = 18
    ##QC_GET_FIRMWARE_BUILD_NAME = 20
    ##QC_GET_EXTENDED_POSITION = 21
    ##QC_EXTENDED_STOP = 22"""
    
    ##def __init__(self, command=C_ECHO_TEST):
        ##self.command = hex(command)
        ###ECHO TEST is the default of nothing is passed
        ##pass
    ##def setCommand(self, command):
        ##self.command = command
        
##print 'done'
        
        