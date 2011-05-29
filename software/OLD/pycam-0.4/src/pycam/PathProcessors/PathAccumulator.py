# -*- coding: utf-8 -*-
"""
$Id: PathAccumulator.py 703 2010-09-27 12:19:06Z sumpfralle $

Copyright 2010 Lars Kruse <devel@sumpfralle.de>
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

from pycam.Toolpath import simplify_toolpath
from pycam.Geometry.Path import Path


class PathAccumulator:
    def __init__(self, zigzag=False, reverse=False):
        self.paths = []
        self.curr_path = None
        self.zigzag = zigzag
        self.scanline = None
        self.reverse = reverse

    def append(self, p):
        if self.curr_path == None:
            self.curr_path = Path()
        self.curr_path.append(p)

    def new_direction(self, direction):
        self.scanline = 0

    def end_direction(self):
        pass

    def new_scanline(self):
        self.scanline += 1
        if self.curr_path:
            print "ERROR: curr_path expected to be empty"
            self.curr_path = None

    def end_scanline(self):
        if self.curr_path:
            if self.zigzag and (self.scanline % 2 == 0):
                self.curr_path.reverse()
            if self.reverse:
                self.curr_path.reverse()
            simplify_toolpath(self.curr_path)
            self.paths.append(self.curr_path)
            self.curr_path = None

    def finish(self):
        pass

