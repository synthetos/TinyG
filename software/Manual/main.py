# flexgridsizer.py

import wx
import ser.serial_init as serinit
import ser.loader
import time



class FlexGridSizer(wx.Frame):
    def __init__(self, parent, id, title):
	self.Frame = wx.Frame.__init__(self, parent, id, title, size=(600,300))
	

	#Serial Port Speeds:
	self.SPORT_SPEEDS = ["115200", "57600", "38400","19200"]
	self.SERIAL_PORTS = []

	#MenuBars and Menus
	menubar = wx.MenuBar()
	file = wx.Menu()
	file.Append(wx.ID_EXIT,'Quit', 'Quit Application')
	menubar.Append(file, '&File')
	self.SetMenuBar(menubar)

	#Status Bar
	self.status_bar = self.CreateStatusBar(1,style=wx.EXPAND|wx.ALL)
	self.status_bar.SetStatusText("TinyG")

	#self.panel
	self.panel = wx.Panel(self, -1)

	#Text Ctrl Boxes
	self.DebugMsg = wx.TextCtrl(self.panel, style=wx.TE_MULTILINE | wx.ALL)
	
	self.CmdInput = wx.TextCtrl(self.panel, id=30, style = wx.ALL | wx.TE_MULTILINE | wx.TE_PROCESS_ENTER )
	self.CmdInput.Value = "(Example) \ng0 x100 y100 z100"
	self.CmdInput.SetToolTip(wx.ToolTip("Type a Gcode command and hit enter or click execute"))
	self.CmdInput.SetEditable(True)
	
	self.GcodeTxt = wx.TextCtrl(self.panel) #TextCtrl Box that displays the Loaded Gcode File
	
	#Sizers
	hbox = wx.BoxSizer(wx.HORIZONTAL) #Create a master hbox then add everything into it
	gbs = wx.GridBagSizer(5, 5)

	#Combo Boxes
	self.FindInitPorts()
	self.cmbSports = wx.ComboBox(self.panel, -1, value=self.SERIAL_PORTS[-1], pos=wx.Point(10,30), choices=self.SERIAL_PORTS)
	self.cmbSpeeds = wx.ComboBox(self.panel, -1, value=self.SPORT_SPEEDS[0], pos=wx.Point(10,30), choices=self.SPORT_SPEEDS)

	#buttons 
	#Dbtn = wx.Button(self.panel, 1001, "D", (10,10), size=(30,20))
	#Ubtn = wx.Button(self.panel, 1002, "U", (10,10), size=(30,20))
	#Lbtn = wx.Button(self.panel, 1003, "L", (10,10), size=(30,20))
	#Rbtn = wx.Button(self.panel, 1004, "R", (10,10), size=(30,20))
	LoadBtn = wx.Button(self.panel,   17,  "Load Gcode")
	RunBtn  = wx.Button(self.panel,   18,   "Run Gcode")
	RefBtn  = wx.Button(self.panel,   19,   "Refresh",   size=(75,25))
	ConBtn  = wx.Button(self.panel,   20,   "Connect",   size=(75,25))
	ExeBtn  = wx.Button(self.panel,   21,   "Execute",   size=(75,25))
	ClrBrn  = wx.Button(self.panel,   22,   "Clear",     size=(75,25))
	StopBtn = wx.Button(self.panel,   23,   "Stop",      size=(75,25))



	self.FindInitPorts() #Scan for ports on program start

	#Static Boxes Connect | Refresh Button
	#self.staticBox1 = wx.StaticBox(self.panel, label="Serial")
	self.Box1 = wx.BoxSizer(wx.VERTICAL)
	self.Box2 = wx.BoxSizer(wx.VERTICAL)

	self.Box2.Add(ClrBrn)
	self.Box2.Add(ExeBtn)


	#self.Box1 = wx.StaticBoxSizer(self.staticBox1, wx.VERTICAL)
	self.Box1.Add(ConBtn)
	self.Box1.Add(RefBtn)
	self.Box1.Add(StopBtn)

	#Hbox2 Create
	hbox2 = wx.BoxSizer(wx.HORIZONTAL) #Box for the run and load gcode buttons


	#Add the Text Input Boxes to Flex Grid
	#gbs.Add(self.CmdInput,  pos=(1, 3),span=(1,3), flag=wx.ALL | wx.EXPAND)  #Execute  Button follows this one

	#Static Text
	self.Gtext = wx.StaticText(self.panel,   -1, "Gcode File:")
	self.Sertext = wx.StaticText(self.panel, -1, "Serial Port:")
	self.Cmdtext = wx.StaticText(self.panel, -1, "Command:")

	#Add buttons run / load to hbox2
	hbox2.Add(LoadBtn)
	hbox2.Add(RunBtn)


	#Add Combo Boxes to Flex Grid
	gbs.Add(self.cmbSports,  pos=(0,1),   span=(1,5),   flag=wx.ALL | wx.EXPAND)
	gbs.Add(self.cmbSpeeds,  pos=(0,7),   span=(1,2),   flag=wx.ALL | wx.EXPAND)
	#Add Buttons to Flex Grid
	gbs.Add(self.Box1,       pos=(0,9),   span=(2,1),   flag=wx.ALL)
	#gbs.Add(ExeBtn,          pos=(1,7),   span=(1,1),   flag=wx.ALL)
	#gbs.Add(ClrBrn,          pos=(1,8),   span=(1,1),   flag=wx.ALL)
	gbs.Add(self.Box2,       pos=(1,7),   span=(1,2),   flag=wx.ALL)
	gbs.Add(RunBtn,          pos=(2,9),   span=(1,1),   flag=wx.ALL)
	gbs.Add(LoadBtn,         pos=(3,9),   span=(1,1),   flag=wx.ALL)
	gbs.Add(self.Sertext,    pos=(0,0),   span=(1,1),   flag=wx.ALL)
	gbs.Add(self.Gtext,      pos=(2,0),   span=(1,1),   flag=wx.ALL)
	gbs.Add(self.GcodeTxt,   pos=(2,1),   span=(1,8),   flag=wx.ALL | wx.EXPAND) 
	gbs.Add(self.DebugMsg,   pos=(4,0),   span=(1,10),  flag=wx.ALL | wx.EXPAND)
	gbs.Add(self.CmdInput,   pos=(1,1),   span=(1,5),   flag=wx.ALL | wx.EXPAND)
	gbs.Add(self.Cmdtext,    pos=(1,0),   span=(1,1),   flag=wx.ALL | wx.EXPAND)
	
	#gbs.Add(RefBtn, pos=(1,9), span=(1,1), flag=wx.ALL)
	#gbs.Add(Dbtn, pos=(3,2), span=(1,1), flag=wx.ALL)
	#gbs.Add(Ubtn, pos=(1,2), span=(1,1), flag=wx.ALL)        
	#gbs.Add(Lbtn, pos=(2,1), span=(1,1), flag=wx.ALL)
	#gbs.Add(Rbtn, pos=(2,3), span=(1,1), flag=wx.ALL)

	gbs.AddGrowableCol(3)
	gbs.AddGrowableRow(3)
	hbox.Add(gbs, -1, wx.ALL | wx.EXPAND, 15)
	self.panel.SetSizer(hbox)

	#Bind Events
	#Text Ctrl Events
	#wx.EVT_COMMAND_ENTER(30, self.OnExecute)
	wx.EVT_TEXT_ENTER(self, 30, self.OnExecute)
	#Menu Events
	wx.EVT_MENU(self, wx.ID_EXIT, self.OnQuit)

	#Button Events
	wx.EVT_BUTTON(self, 17, self.OnLoad)
	wx.EVT_BUTTON(self, 18, self.OnRun)
	wx.EVT_BUTTON(self, 19, self.OnRefresh)
	wx.EVT_BUTTON(self, 20, self.OnConnect)
	wx.EVT_BUTTON(self, 21, self.OnExecute)
	wx.EVT_BUTTON(self, 22, self.OnClear)
	#wx.EVT_BUTTON(self,  22, self.ProcessCode)  #While debugging threading code I attached to the clear button
	wx.EVT_BUTTON(self,  23, self.OnStop)
	
	#Show the Frame Code
	self.Centre()
	self.Show(True)

    def OnLoad(self, event):
	self.PrintDebug("Loading Gcode Files")
	tmpFile = self.GcodeTxt.Value  #Stores the filename that was previously selectev in case of cancel being clicked on dialog
	newFile = wx.FileSelector("Select the Gcode File", default_path="", default_filename="",default_extension="", wildcard="*.gcode", flags=0, parent=None,x=-1, y=-1)
	if newFile == "":
	    pass
	else:
	    self.GcodeTxt.Value = newFile

    def OnRun(self, event):
	self.PrintDebug("Running Gcode Files")
	if self.GcodeTxt.Value == "":  #Make sure that a file has been loaded
	    self.PrintDebug("[ERROR] Load a Gcode File First")
	    return
	try:
	    if self.connection: #Check to make sure the serial connection to tinyg is open
		f = open(self.GcodeTxt.Value, "r")
		ser.loader.processFile(self.DebugMsg, self.connection, f)
	except AttributeError:
	    self.PrintDebug("[ERROR] Open a Serial Connection to TinyG First")

	    
	    
	    
    def ProcessCode(self, evt):
	self.f = open(self.GcodeTxt.Value, "r")
	test = WriteSerial(self, 1, self.connection, self.f, self.DebugMsg)
	test.start()
	
    def OnClear(self, event):
	"""Clear the execute box."""
	self.CmdInput.Value = ""
	
    def OnStop(self, event):
	"""Stops all motion"""
	try:
	    self.connection.write('\x03')
	    self.PrintDebug("[*]Sending Stop Signal")
	except AttributeError:
	    pass
	
    def OnExecute(self, event):
	"""Execute the gcode command that is typed into the Cmd Input Box"""
	CMD = self.CmdInput.Value
	if CMD == "":
	    self.PrintDebug("[!!]ERROR: Invalid Gcode Command")
	else:
	    try:
		self.connection.write(CMD+"\n")
		self.PrintDebug("[*]Sending: %s" % CMD)
		fullResponse = ""
		while(1):
		    res = self.connection.read()
		    if res == "\n":
			break
		    else:
			fullResponse = fullResponse+res
		self.CmdInput.Value = ""
	    except:
		self.PrintDebug("[!!]ERROR: Serial Port Not Connected")
		return
	    self.PrintDebug("[TINYG RESPONSE]: %s " % fullResponse.lstrip())

    def PrintDebug(self, msg):
	"""Accepts a MSG you want to display"""
	return self.DebugMsg.AppendText(msg+"\n")

    def FindInitPorts(self):
	"""Initial Run to find connected ports"""
	self.SERIAL_PORTS = serinit.scan() #Find the serial ports connected to a system (OS independent)
	self.status_bar.SetStatusText("%s Serial Port/s Found." % str(len(self.SERIAL_PORTS))) 
	if self.SERIAL_PORTS == []:  #If no serial ports were detected we return a msg where the port should be
	    msg = "No Serial Ports Found"
	    self.SERIAL_PORTS.append(msg)

    def OnRefresh(self, event):
	try:
	    self.connection.close()
	    self.status_bar.SetStatusText("DISCONNECTED:")
	except:
	    pass
	self.PrintDebug("[*]Scanning For New Serial Ports")
	self.FindInitPorts()
	self.cmbSports.Items = self.SERIAL_PORTS #This updates the combo box with the list of ports returned

    def OnConnect(self, event):
	try:
	    self.connection.close() #Try to close an existing connection.  Perhaps you connected to the wrong port first
	except:
	    pass

	try:
	    self.connection = serinit.connect(self.cmbSports.Value, self.cmbSpeeds.Value)
	    self.connection.write("\n")
	    #print self.connection.readline()
	    self.status_bar.SetStatusText("CONNECTED:  %s:%s" % (self.connection.port, self.connection._baudrate))
	except:
	    self.PrintDebug("[!!]ERROR: Connecting to the Serial Port")
	    self.status_bar.SetStatusText("DISCONNECTED:")

    def OnQuit(self, event):
	self.Destroy()

"""THREADING CODE NOT WORKING RIGHT"""
import threading
class WriteSerial(threading.Thread):
    def __init__(self, parent, value, ser, f, DebugMsg):
	threading.Thread.__init__(self)
	self._parent = parent
	self._value = value
	self.f = f
	self.ser = ser
	self.DebugMsg = DebugMsg

    def run(self):
	"""Pass a serialport and the file object"""
	f = self.f
	ser = self.ser
	try:
	    ser.write(3*"\n") #This might not be need anymore but it just sends a few \n down the wire to clear anything before sending gcode
	except serial.serialutil.SerialException:
	    print "[ERROR] Write Failed"
	    print "System Exiting"
	    sys.exit(6)
	delim = ""
	for line in f:
	    line = line.rstrip() #remove trailing new line character
	    #self.printMsg("[*]Writing: %s" % line)
	    ser.write(line+"\n") #send a line of gcode
	    delim = ser.read() #check to see if we found our delimeter '*' 
	    while delim != "*":  #Loop until we find the delim
		delim = ser.read()

	evt = WriteEvent(myEVT_COUNT, -1, self._value)
	wx.PostEvent(self._parent, evt)
	
    def printMsg(self, msg):
	return self.DebugMsg.AppendText(msg+"\n")
myEVT_COUNT = wx.NewEventType()
EVT_COUNT = wx.PyEventBinder(myEVT_COUNT, 1)

class WriteEvent(wx.PyCommandEvent):
    def __init__(self, etype, eid, value=None):
	wx.PyCommandEvent.__init__(self, etype, eid)
	self._value = value

""" THREADING CODE END - STILL NOT WORKING"""



app = wx.App()
FlexGridSizer(None, -1, 'TinyG Controller')
app.MainLoop()