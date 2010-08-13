#!/usr/bin/env python
#TinyG Controller GUI
#synthetos.com
#Riley Porter

from wx import *
import os

class Controller(wx.Frame):
    def __init__(self, parent, id, title):
        wx.Frame.__init__(self, parent, id, title, wx.DefaultPosition, wx.Size(800, 600))
      
        
        #====================WIDGETS================="
        
        #MenuBar Stuff
        menubar = wx.MenuBar() #Create MenuBar
        file = wx.Menu() #Create Menu
        edit = wx.Menu() #Create Menu
        help = wx.Menu() #Create Menu
        
        file.Append(101, '&Load Gcode', 'Open a new Gcode File')
        file.Append(102, '&Process Job', 'Execute the Gcode')
        file.AppendSeparator()
        
        help.Append(103, '&About', 'Open About Information')
        quit = wx.MenuItem(file, 104, '&Quit\tCtrl+Q', 'Quit the Application')
        edit.Append(105, '&Preferences', 'Edit Prefrences')
        
        file.AppendItem(quit)
        menubar.Append(file, '&File')
        menubar.Append(edit, '&Edit')
        menubar.Append(help, '&Help')
        self.SetMenuBar(menubar)
        self.CreateStatusBar()
        
        #Panel Widgets
        panel = wx.Panel(self, -1)
        
        #Sizer Box Widgets
        box = wx.BoxSizer(wx.HORIZONTAL)
        #box.Add(btnConnect)
        #box.Add(btnQuit,1 )
        
        #Buttons Widgets
        box.Add(wx.Button(panel, 201, 'Connect'), 1, wx.ALL, 5)
        box.Add(wx.Button(panel, 202, 'Quit'), 1, wx.ALL, 5 )
        #box.Add(btnQuit)
        panel.SetSizer(box)
        self.Centre()
        
        
        #======================BIND EVENTS========================#
        #Button Binds
        self.Bind(wx.EVT_BUTTON, self.OnCon,   id=201)   #Attaches the Connect Button
        self.Bind(wx.EVT_BUTTON, self.OnQuit,  id=202)  #Attaches the Quit Button
        #Menu Binds
        self.Bind(wx.EVT_MENU, self.OnOpen,    id=101)    #Attaches the Open menu option
        self.Bind(wx.EVT_MENU, self.OnProcess, id=102)    #Attaches the Process menu option
        self.Bind(wx.EVT_MENU, self.OnAbout,   id=103)    #Attaches the About menu option
        self.Bind(wx.EVT_MENU, self.OnQuit,    id=104)    #Attaches the Quit menu option
        self.Bind(wx.EVT_MENU, self.OnPrefs,   id=105)    #Attaches the Prefs menu option
       
   
        
    #=========================EVENT METHODS=========================#
    def OnQuit(self, event):
        self.Close()
        
    def OnCon(self, event):
        print "[*]Connecting To Serial Port"
        
    def OnOpen(self, event):
        print "[*]Opening Gcode File Chooser"
        """
        Create and show the Open FileDialog
        """ 
        wildcard = "Gcode Files (*.gcode)|*.nc| All files (*.*)|*.*"  #Set the file types listed
        currentDIR = os.getcwd() #get the current directory
        
        DIAopen = wx.FileDialog(
            self, message="Choose a gcode file",
            defaultDir=currentDIR,
            defaultFile="",
            wildcard=wildcard,
            style=wx.OPEN
            )
        if DIAopen.ShowModal() == wx.ID_OK:
            paths = DIAopen.GetPaths()
            print "You chose the following file(s):"
            for path in paths:
                print path
        DIAopen.Destroy()
        
        
    def OnAbout(self, event):
        """ 
        Create and show the About Dialog
        """
        print "[*]TinyG Controller Program By Riley Porter and Alden Hart"  
        DIAabout = wx.MessageDialog(parent=None,
                                    message = "About TinyG Controller", 
                                    caption = "About TinyG",
                                    style = wx.OK|wx.ICON_INFORMATION)
        DIAabout.ShowModal() #Show Message Dialog
        DIAabout.Destroy()   #Close Message Dialog once OK clicked
        return True
    
    def OnProcess(self, event):
        print "[*]Sending Gcode file to TinyG... Please Wait for job to finish."
    
    def OnPrefs(self, event):
        print "[*]Opening the prefrences dialog"
        
        
class MyApp(wx.App):
    def OnInit(self):
        frame = Controller(None, -1, 'TinyG Controller')
        frame.Show(True)
        return True

app = MyApp(0)
app.MainLoop()