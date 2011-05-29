#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
$Id: SimulationTest.py 629 2010-08-23 16:53:06Z sumpfralle $

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

from pycam.Gui.Visualization import Visualization
from pycam.Simulation.ZBuffer import ZBuffer
from pycam.Importers.TestModel import TestModel
from pycam.Geometry.Triangle import Triangle
from pycam.Geometry.Point import Point
from pycam.Cutters.SphericalCutter import SphericalCutter

from OpenGL.GL import *

model = TestModel()

zbuffer = ZBuffer(-5,+5,150, -5,+5,150, 1,5)

#zbuffer.add_wave()

#zbuffer.add_triangle(Triangle(Point(-4,0,0),Point(3,5,2),Point(4,-3,4)))

c = SphericalCutter(0.25)

p = Point(-5,-5,2)
c.moveto(p)

zbuffer.add_triangles(model.triangles())

#zbuffer.add_cutter(c)


def DrawScene():
    size=1
    # axes
    glBegin(GL_LINES)
    glColor3f(1,0,0)
    glVertex3f(0,0,0)
    glVertex3f(size,0,0)
    glEnd()
    glBegin(GL_LINES)
    glColor3f(0,1,0)
    glVertex3f(0,0,0)
    glVertex3f(0,size,0)
    glEnd()
    glBegin(GL_LINES)
    glColor3f(0,0,1)
    glVertex3f(0,0,0)
    glVertex3f(0,0,size)
    glEnd()

    glColor3f(1,1,1)
    c.to_OpenGL()


    glColor3f(0.9,0.8,0.7)
#    glMaterial(GL_FRONT_AND_BACK, GL_AMBIENT, (0.9, 0.8, 0.7, 0.2))
#    glMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE, (0.8, 0.8, 0.8, 0.2))
    glMaterial(GL_FRONT_AND_BACK, GL_SPECULAR, (1.0, 1.0, 1.0, 1.0))
    glMaterial(GL_FRONT_AND_BACK, GL_SHININESS, (0.5))
    zbuffer.to_OpenGL()

dy = 0.1
dx = 0.23
dz = -0.01

def HandleKey(key, x, y):
    global dx,dy,dz
    p.x += dx
    if p.x>5 or p.x<-5:
        dx = -dx
        p.x += dx * 2
    p.y += dy
    if p.y>5 or p.y<-5:
        dy = -dy
        p.y += dy * 2
    p.z += dz

    c.moveto(p)
    zbuffer.add_cutter(c)

Visualization("VisualizationTest", DrawScene, handleKey = HandleKey)

