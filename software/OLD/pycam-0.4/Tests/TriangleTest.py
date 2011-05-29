#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
$Id: TriangleTest.py 629 2010-08-23 16:53:06Z sumpfralle $

Copyright 2010 Lode Leroy

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

import math

from pycam.Geometry import *
from pycam.Gui.Visualization import Visualization


p1 = Point(1,0,0)
p2 = Point(0,1,0)
p3 = Point(0,0,1)
t = Triangle(p1,p2,p3)
t.id=1
t.calc_circumcircle()

def DrawScene():
    t.to_OpenGL()

if __name__ == "__main__":

    print "p1=" + str(p1);
    print "p2=" + str(p2);
    print "p3=" + str(p3);

    print "p2-p1=" + str(p2.sub(p1))
    print "p3-p2=" + str(p3.sub(p2))
    print "p1-p3=" + str(p1.sub(p3))

    print "p2.p1=" + str(p2.dot(p1))
    print "p3.p2=" + str(p3.dot(p2))
    print "p1.p3=" + str(p1.dot(p3))

    print "p1xp2=" + str(p1.cross(p2))
    print "p2xp3=" + str(p2.cross(p3))
    print "p3xp1=" + str(p3.cross(p1))

    print t

    print "circ(t) = %s@%s" % (t.radius(),t.center())


    Visualization("VisualizationTest", DrawScene)
