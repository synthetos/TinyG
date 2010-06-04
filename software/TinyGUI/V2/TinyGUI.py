#! /usr/bin/env python

import wx
import coms
import gcode


class App(wx.App):

    def OnInit(self):
        frame = MainFrame()
        frame.Show()
        self.SetTopWindow(frame)
        return True

class MainFrame(wx.Frame):

    title = "TinyGUI - Motion Controller"

    def __init__(self):
        wx.Frame.__init__(self, None, 1, self.title) #id = 5
        self.panel = wx.Panel(self, -1)
        self.menuFile = wx.Menu()

        #Modes Menu
        self.menuAbout = wx.Menu()
        self.menuAbout.Append(1, "&Mode...", "Select which mode windows are displayed")

        #About Menu
        self.menuAbout = wx.Menu()
        self.menuAbout.Append(2, "&About...", "About this program")

        #Prefrences Menu
        self.menuPrefs = wx.Menu()
        self.menuPrefs.Append(3, "&Preferences","TinyG Prefrences")



        self.menuBar = wx.MenuBar()
        self.menuBar.Append(self.menuPrefs, "&Edit")
        self.menuBar.Append(self.menuAbout, "&Help")
        self.SetMenuBar(self.menuBar)

        self.CreateStatusBar()

                #Bind Functions Here
        self.Bind(wx.EVT_MENU, self.OnAbout, id=2)
        self.Bind(wx.EVT_MENU, self.OnPrefs, id=3)


    def OnQuit(self, event):
        self.Close()

    def OnAbout(self, event):
        AboutFrame().Show()

    def OnPrefs(self, event):
        PrefsFrame().Show()
        
    def OnModeTerminal(self, event):
        ModeTerminalFrame().Show()

    def OnModeStatus(self, event):
        ModeStatusFrame().Show()


#Create Frames the Needed Windows
class PrefsFrame(wx.Frame):
    title = "TinyG Prefrences"
    def __init__(self):
        wx.Frame.__init__(self, wx.GetApp().TopWindow, title=self.title, size=(300,700))
        self.panel = wx.Panel(self, -1)

class ModeTerminalFrame(wx.Frame):
    title = "TinyGUI Terminal"
    def __init__(self):
        wx.Frame.__init__(self, wx.GetApp().TopWindow, title=self.title)
        self.panel = wx.Panel(self, -1)

class ModeStatusFrame(wx.Frame):
    title = "TinyGUI Status Output"
    def __init__(self):
        wx.Frame.__init__(self, wx.GetApp().TopWindow, title=self.title)
        self.panel = wx.Panel(self, -1)        

class AboutFrame(wx.Frame):
    title = "About this program"
    def __init__(self):
        wx.Frame.__init__(self, wx.GetApp().TopWindow, title=self.title)
        self.panel = wx.Panel(self, -1)


#Start the Application
if __name__ == '__main__':
    app = App(False)
    app.MainLoop()
