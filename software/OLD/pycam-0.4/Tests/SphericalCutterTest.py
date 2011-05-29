#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
$Id: SphericalCutterTest.py 629 2010-08-23 16:53:06Z sumpfralle $

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

import math

from pycam.Geometry import *
from pycam.Cutters.SphericalCutter import *

from pycam.Gui.Visualization import ShowTestScene

if __name__ == "__main__":

    if False:
        cutter = SphericalCutter(1, Point(-10,0.5,0))
        edge = Line(Point(0,0,-10),Point(0,0,10))
        dir = Point(1,0,0)
        print "cutter=", cutter
        print "edge=", edge
        print "dir=", dir
        (cl,ccp,cp,d) = cutter.intersect_sphere_line(dir, edge)
        print "cp=", cp
        print "ccp=", ccp
        print "d=", d
        print "cl=", cl
        exit()

    if False:
        cutter = SphericalCutter(1, Point(-10,0,0))
        edge = Line(Point(0,-5,1),Point(3,+5,1))
        dir = Point(1,-0.2,0)
        print "cutter=", cutter
        print "edge=", edge
        print "dir=", dir
        (cl,ccp,cp,d) = cutter.intersect_cylinder_line(dir, edge)
        print "cp=", cp
        print "ccp=", ccp
        print "d=", d
        print "cl=", cl

    dir = Point(0,0,-1)
    c = SphericalCutter(1, Point(0,0,6))
    #c = SphericalCutter(1, Point(-2.2,0.2,6))
    #c = SphericalCutter(1, Point(-1.7,0.5,6))
    print "c=", c
    t = Triangle(Point(-2,0,2), Point(2,1,3), Point(2,-1,4))
    #t = Triangle(Point(-2,0,2), Point(2,-1,4), Point(2,1,3))
    #t = Triangle(Point(2,0,4), Point(2,-1,2), Point(2,1,2))
    #t = Triangle(Point(2,0,2), Point(-2,1,2), Point(-2,-1,2))
    print "t=", t

    if False:
        (cl_p,ccp_p,cp_p,d_p) = c.intersect_sphere_plane(dir,t)
        print "ccp=", ccp_p
        print "cp=", cp_p
        print "cl=", cl_p
        print "d=", d_p

        (cl_v,ccp_v,cp_v,d_v) = c.intersect_sphere_point(dir,t.p1)
        print "ccp=", ccp_v
        print "cp=", cp_v
        print "cl=", cl_v
        print "d=", d_v

        (cl_e,ccp_e,cp_e,d_e) = c.intersect_sphere_line(dir,Line(t.p1,t.p2))
        print "ccp=", ccp_e
        print "cp=", cp_e
        print "cl=", cl_e
        print "d=", d_e

        (cl,d) = c.intersect(dir,t)
        print "cl=", cl

    if False:
        samples = 50
        x0 = -5.0
        x1 = +5.0
        y0 = -5.0
        y1 = +5.0
        z = 10
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
        y0 = -3.0
        y1 = +3.0
        z0 = 0
        z1 = 3
        dir = Point(1,0,0)
        pathlist = []
        for i in range(0, layers):
            z = z1-i*float(z1-z0)/layers
            for j in range(0,samples):
                y = y0 + j * ((y1-y0) / samples)
                c.moveto(Point(x0,y,z))
                (cl,l) = c.intersect(dir,t)
                if cl:
                    p = Path()
                    #p.append(c.center)
                    #p.append(ccp)
                    #p.append(cp)
                    p.append(cl)
                    p.append(cl.sub(dir.mul(l)))
                    pathlist.append(p)

        c.moveto(Point(x0,y0,z))
        ShowTestScene(t, c, pathlist)
