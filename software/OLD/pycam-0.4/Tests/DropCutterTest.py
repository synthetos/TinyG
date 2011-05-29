#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
$Id: DropCutterTest.py 629 2010-08-23 16:53:06Z sumpfralle $

Copyright 2009 Lode Leroy

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

import sys
sys.path.insert(0,'.')

from pycam.Geometry import *
from pycam.Cutters.SphericalCutter import *
from pycam.Cutters.CylindricalCutter import *
from pycam.Cutters.ToroidalCutter import *

from pycam.Gui.Visualization import ShowTestScene

from pycam.Importers.TestModel import TestModel


from pycam.PathGenerators.DropCutter import DropCutter
from pycam.Exporters.SimpleGCodeExporter import SimpleGCodeExporter

if __name__ == "__main__":

    #c = SphericalCutter(1, Point(0,0,7))
    #c = CylindricalCutter(1, Point(0,0,7))
    c = ToroidalCutter(1, 0.1, Point(0,0,7))
    print "c=", c

    #model = TestModel()
    model = Model()
    model.append(Triangle(Point(-3,-4,1),Point(-3,4,1),Point(3,0,1)))


    if True:
        samples = 50
        lines = 50
        x0 = -7.0
        x1 = +7.0
        y0 = -7.0
        y1 = +7.0
        z0 = 0
        z1 = 4
        dx = (x1-x0)/samples
        dy = (y1-y0)/lines
        pg = DropCutter(c, model)

        pathlist = pg.GenerateToolPath(x0, x1, y0, y1, z0, z1, dx, dy, 0)

        ShowTestScene(model, c, pathlist)

