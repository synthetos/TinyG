#TinyG Manual / Loader Application
#www.synthetos.com/tinyG
#Riley Porter [ a t ] Gmail <dot> com


import wx
import ser.serial_init as serinit
import time
from threading import *


class FlexGridSizer(wx.Frame):
    def __init__(self, parent, id, title):
        self.Frame = wx.Frame.__init__(self, parent, id, title, (-1,-1), size=(650,350))

        #Needed for Threading
        self.worker = 0
        #Needed for Threading

        #MenuBars and Menus
        menubar = wx.MenuBar()
        file = wx.Menu()
        file.Append(wx.ID_EXIT,'Quit', 'Quit Application')
        menubar.Append(file, '&File')
        self.SetMenuBar(menubar)
        menubar.Show()

        #Status Bar
        self.status_bar = self.CreateStatusBar(1,style=wx.EXPAND|wx.ALL)
        self.status_bar.SetStatusText("TinyG")

        #self.panel
        self.panel = wx.Panel(self, -1)

        #Text Ctrl Boxes
        self.DebugMsg = wx.TextCtrl(self.panel, style=wx.TE_MULTILINE | wx.ALL)
        self.CmdInput = wx.TextCtrl(self.panel, id=30, style =wx.TE_PROCESS_ENTER )
        self.CmdInput.Value = "g1 x100 y100 z100 f450"
        self.CmdInput.SetToolTip(wx.ToolTip("Type a Gcode command and hit enter to execute the command."))
        self.CmdInput.SetEditable(True)
        self.GcodeTxt = wx.TextCtrl(self.panel) #TextCtrl Box that displays the Loaded Gcode File

        #Sizers
        hbox = wx.BoxSizer(wx.VERTICAL) #Create a master hbox then add everything into it
        gbs = wx.GridBagSizer(5, 5)
        hbox2 = wx.BoxSizer(wx.HORIZONTAL)
        hbox3 = wx.BoxSizer(wx.HORIZONTAL)
        hbox4 = wx.BoxSizer(wx.HORIZONTAL)

        #Serial Port Speeds:
        self.SPORT_SPEEDS = ["115200", "57600", "38400","19200"]
        self.SERIAL_PORTS = []
        self.FindInitPorts() #Scan for ports on program start

        #Combo Boxes
        self.FindInitPorts()
        self.cmbSports = wx.ComboBox(self.panel, -1, value=self.SERIAL_PORTS[-1], pos=wx.Point(10,30), choices=self.SERIAL_PORTS)
        self.cmbSpeeds = wx.ComboBox(self.panel, -1, value=self.SPORT_SPEEDS[0], pos=wx.Point(10,30), choices=self.SPORT_SPEEDS)

        #buttons 
        #Dbtn = wx.Button(self.panel, 1001, "D", (10,10), size=(30,20))
        #Ubtn = wx.Button(self.panel, 1002, "U", (10,10), size=(30,20))
        #Lbtn = wx.Button(self.panel, 1003, "L", (10,10), size=(30,20))
        #Rbtn = wx.Button(self.panel, 1004, "R", (10,10), size=(30,20))
        LoadBtn = wx.Button(self.panel,        17,  "Load Gcode")
        self.RunBtn  = wx.Button(self.panel,   18,   "Run Gcode")
        RefBtn  = wx.Button(self.panel,        19,   "Refresh",   size=(75,25))
        ConBtn  = wx.Button(self.panel,        20,   "Connect",   size=(75,25))
        #ExeBtn  = wx.Button(self.panel,       21,   "Execute",   size=(75,25)) #Just hit enter no need for this anymore
        #ClrBrn  = wx.Button(self.panel,       22,   "Clear",     size=(75,25)) #Not super functional just hit enter
        StopBtn = wx.Button(self.panel,        23,   "Stop",      size=(75,25))
        HardBtn = wx.Button(self.panel,        24,   "Hard Stop", size=(80,25))

        #Static Text
        self.Gtext =   wx.StaticText(self.panel,   -1, "Gcode File:")
        self.Sertext = wx.StaticText(self.panel,   -1, "Serial Port:")
        self.Cmdtext = wx.StaticText(self.panel,   -1, "Gcode Command:")
        self.OutText = wx.StaticText(self.panel,   -1, "Ouput Text:")

        #Add Items to hbox2
        hbox2.Add(HardBtn, border=5, flag=wx.ALL)
        hbox2.Add(StopBtn, border=5, flag=wx.ALL)

        #Add Items to hbox3
        hbox3.Add(RefBtn, border=5, flag=wx.ALL)
        hbox3.Add(ConBtn, border=5, flag=wx.ALL)

        #Add Items to hbox4
        hbox4.Add(LoadBtn, border=5, flag=wx.ALL)
        hbox4.Add(self.RunBtn, border=5, flag=wx.ALL)


        #Add Items to the Flex Grid
        gbs.Add(self.Sertext,    pos=(0,0),   span=(1,1),   flag=wx.ALL)               #Row 0
        gbs.Add(self.cmbSports,  pos=(0,1),   span=(1,5),   flag=wx.ALL | wx.EXPAND)   #Row 0
        gbs.Add(self.cmbSpeeds,  pos=(0,6),   span=(1,3),   flag=wx.ALL | wx.EXPAND)   #Row 0
        gbs.Add(hbox3,           pos=(0,9),   span=(1,2))                              #Row 0


        gbs.Add(self.Cmdtext,    pos=(1,0),   span=(1,1),   flag=wx.ALL | wx.EXPAND)   #Row 1
        gbs.Add(self.CmdInput,   pos=(2,0),   span=(1,11),  flag=wx.ALL | wx.EXPAND)   #Row 2
        gbs.Add(self.Gtext,      pos=(3,0),   span=(1,13),  flag=wx.ALL|wx.EXPAND)     #Row 3

        gbs.Add(self.GcodeTxt,   pos=(4,0),   span=(1,9),   flag=wx.ALL|wx.EXPAND)     #Row 4
        gbs.Add(hbox4,           pos=(4,9),   span=(1,2),   flag=wx.ALL|wx.EXPAND)     #Row 4

        gbs.Add(self.OutText,    pos=(5,0),   span=(1,1))                              #Row 5
        gbs.Add(self.DebugMsg,   pos=(6,0),   span=(3,13), flag=wx.ALL|wx.EXPAND )           #Row 6

        gbs.Add(hbox2,           pos=(9,9),   span=(1,2),  flag=wx.ALL|wx.EXPAND)      #Row 9



        hbox.Add(gbs, -1, wx.ALL | wx.CENTER)  #Add the gbs to the main hbox
        self.panel.SetSizer(hbox)

        #Bind Events
        #Text Ctrl Events
        wx.EVT_TEXT_ENTER(self, 30, self.OnExecute)

        #Menu Events
        wx.EVT_MENU(self, wx.ID_EXIT, self.OnQuit)

        #Button Events
        EVT_RESULT(self,self.OnResult) #THREADING
        wx.EVT_BUTTON(self, 17, self.OnLoad)
        wx.EVT_BUTTON(self, 18, self.OnStart)
        wx.EVT_BUTTON(self, 19, self.OnRefresh)
        wx.EVT_BUTTON(self, 20, self.OnConnect)
        wx.EVT_BUTTON(self, 21, self.OnExecute)
        wx.EVT_BUTTON(self, 22, self.OnClear)
        wx.EVT_BUTTON(self, 23, self.OnStop)
        wx.EVT_BUTTON(self, 24, self.OnStopHARD)

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

    def processFile(self, line):
        """Pass a serialport and the file object"""
        try:
            self.connection.write(3*"\n") #This might not be need anymore but it just sends a few \n down the wire to clear anything before sending gcode
        except serial.serialutil.SerialException:
            print "[ERROR] Write Failed"
            print "System Exiting"
            sys.exit(6)
        delim = ""
        line = line.rstrip() #remove trailing new line character
        self.PrintDebug("[*]Writing: %s" % line)
        self.connection.write(line+"\n") #send a line of gcode
        delim = self.connection.read() #check to see if we found our delimeter '*' 
        while delim != "*":  #Loop until we find the delim
            delim = self.connection.read()
        return

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

    def OnClear(self, event):
        """Clear the execute box."""
        self.CmdInput.Value = ""

    def OnExecute(self, event):
        """Execute the gcode command that is typed into the Cmd Input Box"""
        CMD = self.CmdInput.Value
        if CMD == "":
            self.PrintDebug("[!!]ERROR: Invalid Gcode Command")
        else:
            try:
                """Section is still under development,
		we are waiting to standardize a feed back standard
		with TinyG.  For now this will just echo what command was ran"""
                self.connection.write(CMD+"\n")
                self.PrintDebug("[*]Sending: %s" % CMD)		
                #while(1):
                #    res = self.connection.read()
                #    if res == "\n":
                #        break
                #    else:
                #        fullResponse = fullResponse+res
                self.CmdInput.Value = ""  #Clear the execute command after enter was pressed.
            except:
                self.PrintDebug("[!!]ERROR: Serial Port Not Connected")
                return
            #self.PrintDebug("[TINYG RESPONSE]: %s " % fullResponse.lstrip())

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

    def OnStart(self, event):
        if not self.worker:
            self.RunBtn.SetLabel("Running")
            """Worker takes self, serial port, debug text box and the loaded file"""
            self.worker = WorkerThread(self, self.connection, self.GcodeTxt.Value)

    def OnResult(self, event):
        if event.data is None:
            pass
        else:
            #event.data = str(event.data)
            self.PrintDebug(event.data)

    def OnStopHARD(self, event):
        self.PrintDebug("[!!]HARD STOP!\n You most likely will need to reboot TinyG now.")
        self.connection.write('\x03')  #Send the CTRL+C control character


    def OnStop(self, event):
        """Stop Processing the Gcode File and kill the tinyG"""
        # Flag the worker thread to stop if running

        if self.worker:
            self.RunBtn.SetLabel("Run Gcode")  #Change the Run button label back from Running
            self.PrintDebug('"[!!]Abort Signal Sent,  Trying to Abort Now!"')
            self.worker.abort()
            self.worker = None
            #self.connection.close()  #Try to close the serial port connection



"""THREADING CODE SHOULD BE WORKING CORRECTLY"""
EVT_RESULT_ID = wx.NewId()

def EVT_RESULT(win, func):
    """Define Result Event."""
    win.Connect(-1, -1, EVT_RESULT_ID, func)

class ResultEvent(wx.PyEvent):
    """Simple event to carry arbitrary result data."""
    def __init__(self, data):
        """Init Result Event."""
        wx.PyEvent.__init__(self)
        self.SetEventType(EVT_RESULT_ID)
        self.data = data

class WorkerThread(Thread):
    """Worker Thread Class."""
    def __init__(self, notify_window, ser, gcodeFile):
        """Init Worker Thread Class."""
        Thread.__init__(self)
        self._notify_window = notify_window
        self._want_abort = 0
        self.connection = ser        #Give the worker object access to the serial port
        self.gcodeFile = gcodeFile   #Give the worker object access to the file
        self.start()                 #Starts the thread

    def abort(self):
        """abort worker thread."""
        # Method for use by main thread to signal an abort
        self._want_abort = 1

    def run(self):
        """Run Worker Thread."""
        if self._want_abort:
            """Catch Aborts"""
            wx.PostEvent(self._notify_window, ResultEvent(None))
            return

        if self.connection:                        #check to see if the serial port is connected
            f = open(self.gcodeFile, 'r')          #Open the gcode file 
            fread =  f.readlines()                 #Read the whole gcode file
            for line in fread:                     #Loop through gcode file
                line = line.strip()                #Strip \n at the beggining of the line
                delim = self.processFile(line)     #Send one line of gcode to ProcessGcode get a readline() back
                while delim != "*":                #Loop until we find the delim
                    try:
                        delim = self.connection.read() #Read a char again to find delim
                    except Exception, e:
                        #wx.PostEvent(self._notify_window, ResultEvent(str(e)))
                        return
                wx.PostEvent(self._notify_window, ResultEvent(line))

                if self._want_abort:
                    """Catch Aborts and break the loop!"""
                    wx.PostEvent(self._notify_window, ResultEvent("[!!]Aborted"))
                    return

    def processFile(self, line):
        """Pass a line of the file object (aka line of gcode)"""
        self.connection.write(line+"\n")   #send a line of gcode
        return self.connection.read()      #check to see if we found our delimeter '*' 

    def abort(self):
        """abort worker thread."""
        # Method for use by main thread to signal an abort
        self._want_abort = 1



app = wx.App()
FlexGridSizer(None, -1, 'TinyG Controller / Loader')
app.MainLoop()
