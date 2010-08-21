# flexgridsizer.py

import wx
import ser.serial_init as serinit

class FlexGridSizer(wx.Frame):
    def __init__(self, parent, id, title):
        wx.Frame.__init__(self, parent, id, title, size=(600,400))
        
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
        self.DebugMsg = wx.TextCtrl(self.panel, 205, style=wx.TE_MULTILINE | wx.ALL | wx.TE_PROCESS_ENTER)
        self.CmdInput = wx.TextCtrl(self.panel, wx.ALL)
        
        
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
        RefBtn = wx.Button(self.panel, 19, "Refresh", size=(75,25))
        ConBtn = wx.Button(self.panel, 20, "Connect", size=(75,25))
        ExeBtn = wx.Button(self.panel, 21, "Execute", size=(75,25))
        
     
        self.FindInitPorts() #Scan for ports on program start
        
        #Static Boxes Connect | Refresh Button
        #self.staticBox1 = wx.StaticBox(self.panel, label="Serial")
        self.Box1 = wx.BoxSizer(wx.VERTICAL)
     
        
        #self.Box1 = wx.StaticBoxSizer(self.staticBox1, wx.VERTICAL)
        self.Box1.Add(ConBtn)
        self.Box1.Add(RefBtn)
        
        #Add Combo Boxes to Flex Grid
        gbs.Add(self.cmbSports, pos=(0,3), span=(1,3), flag=wx.ALL | wx.EXPAND)
        gbs.Add(self.cmbSpeeds, pos=(0,7), span=(1,2), flag=wx.ALL | wx.EXPAND)

        #Add the Text Input Boxes to Flex Grid
        gbs.Add(self.DebugMsg,  pos=(7, 1),span=(6,9), flag=wx.ALL | wx.EXPAND)
        gbs.Add(self.CmdInput,  pos=(1, 3),span=(1,3), flag=wx.ALL | wx.EXPAND)  #Execute  Button follows this one
        
        
        #Add Buttons to Flex Grid
        gbs.Add(self.Box1, pos=(0,9), span=(2,1), flag=wx.ALL)
        gbs.Add(ExeBtn,    pos=(1,7), span=(1,1), flag=wx.ALL)
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
        #Menu Events
        wx.EVT_MENU(self, wx.ID_EXIT, self.OnQuit)
        #Button Events
        wx.EVT_BUTTON(self, 20, self.OnConnect)
        wx.EVT_BUTTON(self, 19, self.OnRefresh)
        wx.EVT_BUTTON(self, 21, self.OnExecute)
        
        #Key Events
        
        #Show the Frame Code
        self.Centre()
        self.Show(True)
        
    def OnExecute(self, event):
        """Execute the gcode command that is typed into the Cmd Input Box"""
        CMD = self.CmdInput.Value
        if CMD == "":
            self.PrintDebug("[!!]ERROR: Invalid Gcode Command")
        else:
            self.PrintDebug("[*]Sending: %s" % CMD)
            try:
                self.connection.write(CMD+"\n")
                fullResponse = ""
                while(1):
                    res = self.connection.read()
                    if res == "\n":
                        break
                    else:
                        fullResponse = fullResponse+res
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
   


app = wx.App()
FlexGridSizer(None, -1, 'TinyG Manual Controller')
app.MainLoop()