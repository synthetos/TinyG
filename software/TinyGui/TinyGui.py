#!/usr/bin/python


import gtk
import TinyGSerial


class TinyG(gtk.Window):
    def __init__(self):
        super(TinyG, self).__init__()
        
        #Window
        window = gtk.Window(gtk.WINDOW_TOPLEVEL)
        window.set_title("TinyG Test Terminal")
        window.set_size_request(600, 300)
        window.set_position(gtk.WIN_POS_CENTER)
        window.connect("destroy", self.on_destroy)
        window.set_border_width(10)
        
        vbox = gtk.VBox(False, 5)
        hbox = gtk.HBox(True, 3)
        
        valign = gtk.Alignment(0, 1, 0, 0)
        vbox.pack_start(valign)
        
        
        cb = gtk.combo_box_new_text()
        cb.append_text("one")
        cb.append_text("two")
        cb.append_text("three")
      
        
        ok = gtk.Button("OK")
        ok.set_size_request(70, 30)
        ok.connect("clicked", self.on_ok_clicked)
        
        close = gtk.Button("Close")
        close.connect("clicked",self.on_close_clicked) 
        
        
        buf = gtk.TextBuffer()
        buf.set_text("Hello World\n hello again")
        textview = gtk.TextView(buf)
        #textview.set_wrap_mode(gtk.TextView.set_wrap_mode(GTK_WRAP_WORD))
        textview.set_editable(True)
        hbox.add(ok)
        hbox.add(close)
        vbox.add(textview)
        vbox.add(cb)
        halign = gtk.Alignment(1, 0, 0, 0)
        halign.add(hbox)
        
        vbox.pack_start(halign, False, False, 3)

        self.add(vbox)

        self.connect("destroy", gtk.main_quit)
        self.show_all()
        
        
        #fixed = gtk.Fixed()
        #box = gtk.VBox(False, 0)
        #window.add(box)
        ##box.show()
        
        #box2 = gtk.VBox(False, 0)
        #box2.set_border_width(10)
        #box2.pack_start(box2, True, True, 0)
        #box2.show()
        
        #sw = gtk.ScrolledWindow()
        #sw.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        #box2.pack_start(box2, False, True, 0)
        #box2.show()
        #window.show()
        ##WIDGETS - buttons, combo boxes etc
        
        ##Buttons
        #btn_Execute = gtk.Button("Execute")
        #btn_Execute.connect("clicked", self.on_clicked)
        #btn_Execute.set_size_request(80, 35)

        #btn_Clear = gtk.Button("Clear")
        #btn_Clear.connect("clicked", self.on_clicked_clear)
        #btn_Clear.set_size_request(80, 35)
        

        #menubar_Main = gtk.Menu()
        #menubar_Main_File = gtk.MenuItem("File")
        #menubar_Main_File.set_submenu(menubar_Main)
        
        ##Text View
        #textview_Status = gtk.TextView()
        #buf = gtk.TextBuffer()
        #buf.set_text("This is the text")
        #textview_Status.set_border_width(24)
        #textview_Status.set_buffer(buf)
        #textview_Status.set_editable(True)
        #textview_Status.set_left_margin(50)
        #textview_Status.set_right_margin(50)
        #textview_Status.set_border_width(10)
        #sw.add(textview_Status)
        #sw.show()
        #textview_Status.show()
        ##fixed.put(vbox, 20, 40)
        
        ##Combo Box
        
        #cmb_serial_Device = gtk.combo_box_new_text()
        #ports = TinyGSerial.scanLinux()
        
        ##Enumerates OS and scans for Possible Serial Ports
        #for x in ports:
            #cmb_serial_Device.append_text(x)

        
        ##Place Widgets on Container
        ##fixed.put(menubar_Main)
        #fixed.put(btn_Execute, 5, 50)
        #fixed.put(btn_Clear, 5, 100)
        #fixed.put(cmb_serial_Device, 90, 50)
        #fixed.put(textview_Status,150,200)
        ##self.add(fixed)
        #self.show_all()
        
    #Callback Connectors
    def on_ok_clicked(self, widget):
        self.set_title("Word")
    
    def on_destroy(self, widget):
        gtk.main_quit()
    def on_close_clicked(self, widget):
        gtk.main_quit()
    #Enc Callback Connectors
    
        
    #def on_clicked(self, widget):
        
        #print "Clicked Connect"
        ##ser.connect("/dev/ttyUSB0")
        
    #def on_clicked_clear(self, widget):
        #print("Clear Clicked")
        #buf = ""
        ##textview_Status.set_buffer(buf)
    


TinyG()
gtk.main()
