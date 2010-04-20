# flexgridsizer.py

import wx

class FlexGridSizer(wx.Frame):
    def __init__(self, parent, id, title):
        wx.Frame.__init__(self, parent, id, title, size=(400,400))
        
        #MenuBars and Menus
        menubar = wx.MenuBar()
        file = wx.Menu()
        file.Append(wx.ID_EXIT,'Quit', 'Quit Application')
        menubar.Append(file, '&File')
        self.SetMenuBar(menubar)
        
        #Status Bar
        status_bar = self.CreateStatusBar(4,style=wx.EXPAND|wx.ALL)
        status_bar.SetStatusText("TinyG")
        
        panel = wx.Panel(self, -1)
       
        #Sizers
        hbox = wx.BoxSizer(wx.HORIZONTAL) #Create a master hbox then add everything into it
        
        fgs = wx.GridBagSizer(5, 5)
        
        #Combo Boxes
        Values = ["One","Two","Three"]
        #self.combo1 = wx.ComboBox(self, -1, value=areaList[0], pos=wx.Point(10, 30),size=wx.Size(120, 150), choices=areaList)
        combo1 = wx.ComboBox(panel, -1, value=Values[0], pos=wx.Point(10,30), choices=Values)
        
        
        #Text Ctrl Boxes
        DebugMsg = wx.TextCtrl(panel, wx.ALL)
       
        #buttons 
        Dbtn = wx.Button(panel, 1001, "D", (10,10), size=(30,20))
        Ubtn = wx.Button(panel, 1002, "U", (10,10), size=(30,20))
        Lbtn = wx.Button(panel, 1003, "L", (10,10), size=(30,20))
        Rbtn = wx.Button(panel, 1004, "R", (10,10), size=(30,20))
        ConBtn = wx.Button(panel, 1005, "Connect")
        
        fgs.Add(DebugMsg, pos=(5, 0), span=(5,5), flag=wx.ALL|wx.EXPAND)
        fgs.Add(combo1, pos=(0,0), span=(1,4), flag=wx.ALL|wx.EXPAND)
        fgs.Add(ConBtn, pos=(0,4), span=(1,1), flag=wx.ALL)
        fgs.Add(Dbtn, pos=(3,2), span=(1,1), flag=wx.ALL)
        fgs.Add(Ubtn, pos=(1,2), span=(1,1), flag=wx.ALL)        
        fgs.Add(Lbtn, pos=(2,1), span=(1,1), flag=wx.ALL)
        fgs.Add(Rbtn, pos=(2,3), span=(1,1), flag=wx.ALL)
        
        fgs.AddGrowableCol(3)
        fgs.AddGrowableRow(3)
        hbox.Add(fgs, -1, wx.ALL | wx.EXPAND, 15)
        panel.SetSizer(hbox)

        #Bind Events
        wx.EVT_MENU(self, wx.ID_EXIT, self.OnQuit)
        #self.Bind(self, wx.EVT_MENU, self.OnQuit(), wx.ID_EXIT )        
        
        self.Centre()
        self.Show(True)
        
        
    def OnQuit(self, event):
        print "Destroyed"
        self.Destroy()
   
        

app = wx.App()
FlexGridSizer(None, -1, 'TinyG Manual Controller')
app.MainLoop()