## gcode.py is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by the
## Free Software Foundation; either version 2 of the License, or (at your
## option) any later version.  gcode.py is distributed in the hope that it
## will be useful, but WITHOUT ANY WARRANTY; without even the implied
## warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
## the GNU General Public License for more details.  You should have
## received a copy of the GNU General Public License along with gcode.py; if
## not, write to the Free Software Foundation, Inc., 59 Temple Place,
## Suite 330, Boston, MA 02111-1307 USA
##
## gcode.py is Copyright (C) 2005 Chris Radek
## chris@timeguy.com
#
# $Id: gcode.py 633 2010-08-24 01:39:35Z sumpfralle $


class gcode:

    def __init__(self, safetyheight=1.0, tool_id=1):
        self.tool_id = tool_id
        self.safetyheight = safetyheight
        self.lastfeed = None
        self.lastx = None
        self.lasty = None
        self.lastz = None
        self.lasta = None
        self.lastgcode = None
        self.lastz = None

    def delay(self, seconds):
        return "G04 P%d" % seconds

    def begin(self):
        """
        G40: disable tool radius compensation
        G49: disable tool length compensation
        G54: select coordinate system 1
        G80: cancel model motion
        G90: use absolute coordinates instead of axis increments
        G00 Z?: no x/y positioning - just go up to safety height
        """
        return ("G40 G49 G54 G80 G90",
                "G04 P3 T%d M6" % self.tool_id,
                "G00 Z%.4f" % self.safetyheight)

    def exactpath(self):
        return "G61"

    def continuous(self):
        return "G64"

    def rapid(self, x = None, y = None, z = None, a = None, gcode = "G00",
            feed=None):
        gcodestring = feedstring = xstring = ystring = zstring = astring = ""
        if x == None:
            x = self.lastx
        if y == None:
            y = self.lasty
        if z == None:
            z = self.lastz
        if a == None:
            a = self.lasta
        if gcode != self.lastgcode:
            gcodestring = gcode
            self.lastgcode = gcode
        if x != self.lastx:
            xstring = " X%.4f" % (x)
            self.lastx = x
        if y != self.lasty:
            ystring = " Y%.4f" % (y)
            self.lasty = y
        if z != self.lastz:
            zstring = " Z%.4f" % (z)
            self.lastz = z
        if a != self.lasta:
            astring = " A%.4f" % (a)
            self.lasta = a
        if (gcode == "G01") and feed and (feed != self.lastfeed):
            feedstring = " F%.4f" % (feed)
            self.lastfeed = feed
        positionstring = xstring + ystring + zstring + astring
        if len(positionstring) > 0:
            return gcodestring + feedstring + positionstring
        else:
            return ""

    def cut(self, x = None, y = None, z = None, a = None, feed=None):
        if x == None: x = self.lastx
        if y == None: y = self.lasty
        if z == None: z = self.lastz
        if a == None: a = self.lasta
        return self.rapid(x, y, z, a, gcode="G01", feed=feed)

    def safety(self):
        return self.rapid(z=self.safetyheight)

