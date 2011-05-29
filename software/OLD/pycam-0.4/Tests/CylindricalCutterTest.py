#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
$Id: CylindricalCutterTest.py 629 2010-08-23 16:53:06Z sumpfralle $

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
from pycam.Cutters.CylindricalCutter import *

from pycam.Gui.Visualization import ShowTestScene

if __name__ == "__main__":

    dir = Point(0,0,-1)
    #c = CylindricalCutter(1, Point(0,0,6))
    c = CylindricalCutter(1, Point(2,0,6))
    #c = CylindricalCutter(1, Point(-2.2,0.2,6))
    #c = CylindricalCutter(1, Point(-1.7,-0.2,6))
    print "c=", c
    #t = Triangle(Point(-3,0,2), Point(2,2,3), Point(2,-2,4))
    #t = Triangle(Point(-2,0,2), Point(2,-1,4), Point(2,1,3))
    #t = Triangle(Point(2,0,4), Point(2,-1,2), Point(2,1,2))
    #t = Triangle(Point(2,0,4), Point(2,1,2), Point(2,-1,2))
    #t = Triangle(Point(-3,0,4), Point(2,2,2), Point(2,-2,2))
    t = Triangle(Point(-3,0,2.5), Point(3,0,2.5), Point(0,1,1.5))
    print "t=", t

    if False:
        print "plane:"
        (cl_p,ccp_p,cp_p,d_p) = c.intersect_circle_plane(dir,t)
        print "ccp=", ccp_p
        print "cp=", cp_p
        print "cl=", cl_p
        print "d=", d_p

        print "triangle:"
        (cl_t,d_t) = c.intersect_circle_triangle(dir,t)
        print "cl=", cl_t
        print "d=", d_t

        print "point:"
        (cl_v,ccp_v,cp_v,d_v) = c.intersect_circle_point(dir,t.p1)
        print "ccp=", ccp_v
        print "cp=", cp_v
        print "cl=", cl_v
        print "d=", d_v

        l = Line(t.p2,t.p3)
        print "line:", l
        (cl_l,ccp_l,cp_l,d_l) = c.intersect_circle_line(dir,l)
        print "ccp=", ccp_l
        print "cp=", cp_l
        print "cl=", cl_l
        print "d=", d_l

        print "edge:", l
        (cl_e,d_e) = c.intersect_circle_edge(dir,l)
        print "cl=", cl_e
        print "d=", d_e

        print "piece:"
        (cl,d) = c.intersect(dir,t)
        print "cl=", cl

    if False:
        samples = 50
        x0 = -5.0
        x1 = +5.0
        y0 = -5.0
        y1 = +5.0
        z = 10
        dir = Point(0,0,-1)
        pathlist = []
        for i in range(0,samples):
            x = x0 + i * ((x1-x0) / samples)
            p = Path()
            for j in range(0,samples):
                y = y0 + j * ((y1-y0) / samples)
                c.moveto(Point(x,y,z))
                cl = c.drop(t)
                if cl:
                    p.append(cl)
                else:
                    p.append(Point(x,y,0))
            pathlist.append(p)
        c.moveto(Point(x0,y0,z))
        ShowTestScene(t, c, pathlist)

    if True:
        samples = 50
        layers = 10
        x0 = -5.0
        x1 = +5.0
        y0 = -5.0
        y1 = +5.0
        z0 = 0
        z1 = 4
        dir = Point(1,0,0)
        pathlist = []
        for i in range(0, layers):
            z = z1-i*float(z1-z0)/layers
            for j in range(0,samples):
                y = y0 + j * ((y1-y0) / samples)
                p = Point(x0,y,z)
                c.moveto(p)
                (cl,l) = c.intersect(dir,t)
                if cl:
                    p = Path()
                    #p.append(c.center)
                    #p.append(ccp)
                    #p.append(cp)
                    p.append(cl)
                    p.append(cl.sub(dir.mul(l)))
                    pathlist.append(p)

        c.moveto(Point(x0,y0,z1))
        ShowTestScene(t, c, pathlist)
