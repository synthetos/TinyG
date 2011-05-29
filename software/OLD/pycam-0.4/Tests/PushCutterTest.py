#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
$Id: PushCutterTest.py 629 2010-08-23 16:53:06Z sumpfralle $

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

import sys
sys.path.insert(0,'.')

from pycam.Geometry import *
from pycam.Cutters.SphericalCutter import *
from pycam.Cutters.CylindricalCutter import *
from pycam.Cutters.ToroidalCutter import *

from pycam.Gui.Visualization import ShowTestScene

from pycam.Importers import STLImporter

from pycam.PathGenerators.PushCutter import PushCutter
from pycam.PathProcessors import *
from pycam.Exporters.SimpleGCodeExporter import SimpleGCodeExporter

if __name__ == "__main__":

    c = SphericalCutter(0.1, Point(0,0,7))
    #c = CylindricalCutter(1, Point(0,0,7))
    #c = ToroidalCutter(1, 0.25, Point(0,0,7))
    print "c=", c

    #model = TestModel()
    #model = STLImporter.ImportModel("Samples/STL/Box0.stl")
    #model = STLImporter.ImportModel("Samples/STL/Box1.stl")
    model = STLImporter.ImportModel("Samples/STL/Box0+1.stl")
    #model = Model()
    #model.append(Triangle(Point(0,0,0),Point(0,5,4),Point(0,-5,4)))
    #model.append(Triangle(Point(2,0,0),Point(2,-5,4),Point(2,5,4)))

    if True:
        lines = 20
        layers = 4
        x0 = -7.0
        x1 = +7.0
        y0 = -7.0
        y1 = +7.0
        z0 = 2.0
        z1 = 4.0
        pc = PushCutter(c, model, SimpleCutter())
        #pc = PushCutter(c, model, ZigZagCutter())
        #pc = PushCutter(c, model, PolygonCutter())

        dx = 0
        if lines>1:
            dy = float(y1-y0)/(lines-1)
        else:
            dy = INFINITE
        if layers>1:
            dz = float(z1-z0)/(layers-1)
        else:
            dz = INFINITE

        pathlist = pc.GenerateToolPath(x0,x1,y0,y1,z0,z1,dx,dy,dz)
        c.moveto(Point(x0,y0,z0))
        ShowTestScene(model, c, pathlist)
