# -*- coding: utf-8 -*-
"""
$Id: ContourCutter.py 772 2010-10-14 11:45:38Z sumpfralle $

Copyright 2008-2010 Lode Leroy

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

from pycam.Geometry.PolygonExtractor import PolygonExtractor
from pycam.Geometry.Point import Point
from pycam.Toolpath import simplify_toolpath

class ContourCutter:
    def __init__(self, reverse=False):
        self.paths = []
        self.curr_path = None
        self.scanline = None
        self.pe = None
        self.points = []
        self.reverse = reverse
        self.__forward = Point(1, 1, 0)

    def append(self, p):
        # Sort the points in positive x/y direction - otherwise the
        # PolygonExtractor breaks.
        if self.points and (p.sub(self.points[0]).dot(self.__forward) < 0):
            self.points.insert(0, p)
        else:
            self.points.append(p)

    def new_direction(self, direction):
        if self.pe == None:
            self.pe = PolygonExtractor(PolygonExtractor.CONTOUR)

        self.pe.new_direction(direction)

    def end_direction(self):
        self.pe.end_direction()

    def new_scanline(self):
        self.pe.new_scanline()
        self.points = []

    def end_scanline(self):
        for i in range(1, len(self.points)-1):
            self.pe.append(self.points[i])
        self.pe.end_scanline()

    def finish(self):
        self.pe.finish()
        if self.pe.merge_path_list:
            paths = self.pe.merge_path_list
        elif self.pe.hor_path_list:
            paths = self.pe.hor_path_list
        else:
            paths = self.pe.ver_path_list
        if paths:
            for p in paths:
                p.append(p.points[0])
                simplify_toolpath(p)
        if paths:
            if self.reverse:
                paths.reverse()
            self.paths.extend(paths)
        self.pe = None

