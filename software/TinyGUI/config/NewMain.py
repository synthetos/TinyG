#!/usr/bin/env python

#New GUI XML Test


import  wx
import  wx.xrc  as  xrc


class Xml(wx.Frame):
    def __init__(self, parent, id, title):
        wx.Frame.__init__(self, parent, id, title)

        res = xrc.XmlResource('resource.xrc')
        res.LoadPanel(self, 'MyPanel')
        
        self.Bind(wx.EVT_BUTTON, self.OnClose,  id=xrc.XRCID('CloseButton'))
        self.Center()
        self.Show(True)


    def OnClose(self, event):
        self.Close()

app = wx.App()
Xml(None, -1, 'xml.py')
app.MainLoop()