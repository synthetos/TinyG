# -*- coding: utf-8 -*-
"""
$Id: PolygonCutter.py 703 2010-09-27 12:19:06Z sumpfralle $

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

from pycam.Geometry.Path import Path
from pycam.Geometry.PolygonExtractor import PolygonExtractor
from pycam.Toolpath import simplify_toolpath


class PolygonCutter:
    def __init__(self, reverse=False):
        self.paths = []
        self.curr_path = None
        self.scanline = None
        self.pe = PolygonExtractor(PolygonExtractor.MONOTONE)
        self.reverse = reverse

    def append(self, p):
        self.pe.append(p)

    def new_direction(self, direction):
        self.pe.new_direction(direction)

    def end_direction(self):
        self.pe.end_direction()

    def new_scanline(self):
        self.pe.new_scanline()

    def end_scanline(self):
        self.pe.end_scanline()

    def finish(self):
        self.pe.finish()
        paths = []
        source_paths = []
        if self.pe.hor_path_list:
            source_paths.extend(self.pe.hor_path_list)
        if self.pe.ver_path_list:
            source_paths.extend(self.pe.ver_path_list)
        for path in source_paths:
            points = path.points
            for i in range(0, (len(points)+1)/2):
                p = Path()
                if i % 2 == 0:
                    p.append(points[i])
                    p.append(points[-i-1])
                else:
                    p.append(points[-i-1])
                    p.append(points[i])
                paths.append(p)
        if paths:
            for p in paths:
                simplify_toolpath(p)
                if self.reverse:
                    p.reverse()
            self.paths.extend(paths)

