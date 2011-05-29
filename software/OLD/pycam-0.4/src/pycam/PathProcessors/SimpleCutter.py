# -*- coding: utf-8 -*-
"""
$Id: SimpleCutter.py 703 2010-09-27 12:19:06Z sumpfralle $

Copyright 2008 Lode Leroy

This file is part of PyCAM.

PyCAM is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

PyCAM is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with PyCAM.  If not, see <http://www.gnu.org/licenses/>.
"""

from pycam.Geometry.Path import Path
from pycam.Toolpath import simplify_toolpath

class SimpleCutter:
    def __init__(self, reverse=False):
        self.paths = []
        self.curr_path = None
        self.reverse = reverse

    def append(self, p):
        curr_path = None
        if self.curr_path == None:
            curr_path = Path()
            self.curr_path = curr_path
        else:
            curr_path = self.curr_path
            self.curr_path = None
        curr_path.append(p)
        if self.curr_path == None:
            simplify_toolpath(curr_path)
            if self.reverse:
                self.paths.insert(0, curr_path)
            else:
                self.paths.append(curr_path)

    def new_direction(self, direction):
        pass

    def end_direction(self):
        pass

    def new_scanline(self):
        if self.curr_path:
            print "ERROR: curr_path expected to be empty"
            self.curr_path = None

    def end_scanline(self):
        if self.curr_path:
            print "ERROR: curr_path expected to be empty"
            self.curr_path = None

    def finish(self):
        pass
