import wx, sys
import options



#WX Python Imports
from wx import xrc


#Twisted Imports
from twisted.internet import wxreactor
from twisted.internet.serialport import SerialPort
from twisted.protocols.basic import LineReceiver
from twisted.internet.protocol import Protocol, Factory
from twisted.internet.defer import Deferred

#from events import Events
wxreactor.install()

# import twisted reactor *only after* installing wxreactor
from twisted.internet import reactor, protocol
from twisted.protocols import basic

class TinyG_Protocol(LineReceiver):
    def processData(self, data):
        print data
        
    def connectionMade(self):
        print "TinyG Connected!"
        
    def lineReceived(self, line):
        print line

class TinyG_Factory(Factory):
    protocol = TinyG_Protocol
    sport = options.TG_Options.options["serialport"]
    s = SerialPort(TinyG_Protocol(), sport, reactor, 115200)


class SettingPage(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent)
        t = wx.StaticText(self, -1, "This is Settings", (20,20))
        


class GUI_Frame(wx.App):
    def OnInit(self):
        self.res = xrc.XmlResource("TG.xrc")
        self.init_frame()
        return True
        
    def init_frame(self):
        self.frame = self.res.Load( 'NOTE_TINYG')
        self.panel = xrc.XRCCTRL(self.frame, 'NoteBook_TINYG')
        self.settings_panel = xrc.XRCCTRL(self.frame, "tab_settings")
        self.frame.Show()

if __name__ == '__main__':
    app = GUI_Frame(False)
    app.MainLoop()
    #reactor.registerWxApp(app)
    #reactor.connectTCP("localhost", 5001, ChatFactory(frame))
    #reactor.run()
    
