# -*- coding: utf-8 -*-
"""
$Id: ZBuffer.py 629 2010-08-23 16:53:06Z sumpfralle $

Copyright 2009-2010 Lode Leroy

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

from pycam.Geometry.utils import sqrt
from pycam.Geometry.Point import Point
import ctypes


try:
    import OpenGL.GL as GL
    GL_enabled = True
except ImportError:
    GL_enabled = False


EPSILON = 1e-8

NUM_PER_CELL_X = 10
NUM_PER_CELL_Y = 10
NUM_CELL_X = 0
NUM_CELL_Y = 0


class ZBufferItem:
    def __init__(self, z=0.0):
        self.z = float(z)
        self.changed = True
        self.list = -1

class ZCellItem:
    def __init__(self):
        self.list = -1
        self.array = None

class ZBuffer:
    def __init__(self, minx, maxx, xres, miny, maxy, yres, minz, maxz):
        global NUM_CELL_X, NUM_CELL_Y
        self.minx = float(minx)
        self.maxx = float(maxx)
        self.miny = float(miny)
        self.maxy = float(maxy)
        self.minz = float(minz)
        self.maxz = float(maxz)
        self.xres = int(xres)
        self.yres = int(yres)
        self.changed = True

        NUM_CELL_X = self.xres / NUM_PER_CELL_X
        NUM_CELL_Y = self.yres / NUM_PER_CELL_Y

        self.x = [0.0] * self.xres
        for i in range(0, self.xres):
            self.x[i] = self.minx+(i * (self.maxx-self.minx)/self.xres)
        self.y = [0.0] * self.yres
        for i in range(0, self.yres):
            self.y[i] = self.miny+(i * (self.maxy-self.miny)/self.yres)
        self.buf = [ [] ] * self.yres
        for y in range(0, self.yres):
            self.buf[y] = [ None ] * self.xres
            for x in range(0, self.xres):
                self.buf[y][x] = ZBufferItem(self.minz)

        self.list = [ [] ] * self.yres
        for y in range(0, self.yres):
            self.list[y] = [ None ] * self.xres
            for x in range(0, self.xres):
                self.list[y][x] = -1

        self.cell = [ None ] * NUM_CELL_Y
        for y in range(0, NUM_CELL_Y):
            self.cell[y] = [ None ] * NUM_CELL_X
            for x in range(0, NUM_CELL_X):
                self.cell[y][x] = ZCellItem()

    def add_wave(self, freq=8, damp=3.0):
        self.changed = True
        rmax = sqrt(self.y[0]*self.y[0]+self.x[0]*self.x[0])
        for y in range(0, self.yres):
            for x in range(0, self.xres):
                r = sqrt(self.y[y]*self.y[y]+self.x[x]*self.x[x])
                self.buf[y][x].z = 1 + math.cos(r / rmax * r / rmax * math.pi \
                        * freq) / (1 + damp * (r / rmax))
                self.buf[y][x].changed = True

    def add_triangles(self, triangles):
        for t in triangles:
            self.add_triangle(t)

    def add_triangle(self, t):
        minx = int((t.minx - self.minx) / (self.maxx - self.minx) \
                * self.xres) - 1
        maxx = int((t.maxx - self.minx) / (self.maxx - self.minx) \
                * self.xres) + 1
        miny = int((t.miny - self.miny) / (self.maxy - self.miny) \
                * self.yres) - 1
        maxy = int((t.maxy - self.miny) / (self.maxy - self.miny) \
                * self.yres) + 2
        if minx < 0: 
            minx = 0
        if maxx > self.xres - 1: 
            maxx = self.xres - 1
        if miny < 0: 
            miny = 0
        if maxy > self.yres - 1:
            maxy = self.yres - 1

        for y in range(miny, maxy):
            py = self.y[y]
            for x in range(minx, maxx):
                px = self.x[x]
                v0x = t.p3.x - t.p1.x
                v0y = t.p3.y - t.p1.y
                v1x = t.p2.x - t.p1.x
                v1y = t.p2.y - t.p1.y
                v2x = px - t.p1.x
                v2y = py - t.p1.y
                dot00 = v0x*v0x + v0y*v0y
                dot01 = v0x*v1x + v0y*v1y
                dot02 = v0x*v2x + v0y*v2y
                dot11 = v1x*v1x + v1y*v1y
                dot12 = v1x*v2x + v1y*v2y
                invDenom = 1 / (dot00 * dot11 - dot01 * dot01)
                u = (dot11 * dot02 - dot01 * dot12) * invDenom
                v = (dot00 * dot12 - dot01 * dot02) * invDenom
                if (u >= -EPSILON) and (v >= -EPSILON) and (u + v <= 1-EPSILON):
                    v0z = t.p3.z - t.p1.z
                    v1z = t.p2.z - t.p1.z
                    pz = t.p1.z + v0z * u + v1z * v
                    if pz > self.buf[y][x].z:
                        self.buf[y][x].z = pz
                        self.buf[y+0][x+0].changed = True
                        self.buf[y+0][x+1].changed = True
                        self.buf[y+1][x+0].changed = True
                        self.buf[y+1][x+1].changed = True
                        self.changed = True

    def add_cutter(self, c):
        cx = c.location.x
        cy = c.location.y
        rsq = c.radiussq
        minx = int((c.minx - self.minx) / (self.maxx - self.minx) * self.xres) \
                - 1
        maxx = int((c.maxx - self.minx) / (self.maxx - self.minx) * self.xres) \
                + 1
        miny = int((c.miny - self.miny) / (self.maxy - self.miny) * self.yres) \
                - 1
        maxy = int((c.maxy - self.miny) / (self.maxy - self.miny) * self.yres) \
                + 1
        if minx < 0: 
            minx = 0
        if maxx < 0: 
            maxx = 0
        if minx > self.xres - 1: 
            minx = self.xres - 1
        if maxx > self.xres - 1: 
            maxx = self.xres - 1
        if miny < 0: 
            miny = 0
        if maxy < 0: 
            maxy = 0
        if maxy > self.yres - 1: 
            maxy = self.yres - 1
        if miny > self.yres - 1: 
            miny = self.yres - 1
        p = Point(0, 0, 0)
        zaxis = Point(0, 0, -1)

        for y in range(miny, maxy):
            p.y = py = self.y[y]
            for x in range(minx, maxx):
                p.x = px = self.x[x]
                if (px - cx) * (px - cx) + (py - cy) * (py - cy) \
                        <= rsq + EPSILON:
                    (cl, ccp, cp, l) = c.intersect_point(zaxis, p)
                    if ccp:
                        pz = l
                        if pz < self.buf[y][x].z:
                            self.buf[y][x].z = pz
                            self.buf[y+0][x+0].changed = True
                            self.buf[y+0][x+1].changed = True
                            self.buf[y+1][x+0].changed = True
                            self.buf[y+1][x+1].changed = True
                            self.changed = True

    def to_OpenGL(self):
        if GL_enabled:
            self.to_OpenGL_6()
        self.changed = False

    def normal(self, z0, z1, z2):
        nx = 1.0 / self.xres
        ny = 1.0 / self.yres
        nz = 1.0 / (self.maxz - self.minz)
        return (-ny * (z1 - z0) * nz / nx, -nx * (z2 - z1) * nz / ny,
                nx * ny / nz * 100)
        
    # the naive way (quads)
    def to_OpenGL_1(self):
        GL.glBegin(GL.GL_QUADS)
        for y in range(self.yres - 1):
            for x in range(self.xres - 1):
                n = self.normal(self.buf[y+0][x+0].z, self.buf[y+0][x+1].z,
                        self.buf[y+1][x+0].z)
                GL.glNormal3f(n[0], n[1], n[2])
                GL.glVertex3f(self.x[x+0], self.y[y+0], self.buf[y+0][x+0].z)
                GL.glVertex3f(self.x[x+1], self.y[y+0], self.buf[y+0][x+1].z)
                GL.glVertex3f(self.x[x+1], self.y[y+1], self.buf[y+1][x+1].z)
                GL.glVertex3f(self.x[x+0], self.y[y+1], self.buf[y+1][x+0].z)

        GL.glEnd()

    # use display lists (per quad)
    def to_OpenGL_2(self):        
        for y in range(self.yres - 1):
            for x in range(self.xres - 1):
#                print "z[%f][%f]=%f" % (self.y[y],self.x[x],self.buf[y][x])
                if self.buf[y+0][x+0].changed or self.buf[y+0][x+1].changed \
                        or self.buf[y+1][x+0].changed \
                        or self.buf[y+1][x+1].changed:
                    if self.buf[y][x].list == -1:
                        self.buf[y][x].list = GL.glGenLists(1)

                    GL.glNewList(self.buf[y][x].list, GL.GL_COMPILE)
                    GL.glBegin(GL.GL_QUADS)
                    n = self.normal(self.buf[y+0][x+0].z, self.buf[y+0][x+1].z,
                            self.buf[y+1][x+0].z)
                    GL.glNormal3f(n[0], n[1], n[2])
                    GL.glVertex3f(self.x[x+0], self.y[y+0],
                            self.buf[y+0][x+0].z)
                    GL.glVertex3f(self.x[x+1], self.y[y+0],
                            self.buf[y+0][x+1].z)
                    GL.glVertex3f(self.x[x+1], self.y[y+1],
                            self.buf[y+1][x+1].z)
                    GL.glVertex3f(self.x[x+0], self.y[y+1],
                            self.buf[y+1][x+0].z)
                    GL.glEnd()
                    GL.glEndList()
                
        for y in range(0, self.yres-1):
            for x in range(0, self.xres-1):
                self.buf[y][x].changed = False
                GL.glCallList(self.buf[y][x].list)

    # use display list per cell (cell = group of quads)
    def to_OpenGL_3(self):
        dy = self.yres/NUM_CELL_Y
        dx = self.xres/NUM_CELL_X

        for y in range(NUM_CELL_Y):
            y0 = y * dy
            y1 = y0 + dy + 1
            if y1 > self.yres:
                y1 = self.yres
            for x in range(NUM_CELL_X):
                x0 = x * dx
                x1 = x0 + dx + 1
                if x1 > self.xres:
                    x1 = self.xres

                changed = False

                if self.changed:
                    for yi in range(y0, y1):
                        for xi in range(x0, x1):
                            if self.buf[yi][xi].changed:
                                changed = True
                                break

                if changed:
                    if self.list[y][x] == -1:
                        self.list[y][x] = GL.glGenLists(1)

                    if False:
                        GL.glNewList(self.list[y][x], GL.GL_COMPILE)
                        for yi in range(y0, y1-1):
                            GL.glBegin(GL.GL_TRIANGLES)
                            for xi in range(x0, x1-1):
                                n = self.normal(self.buf[yi+0][xi+0].z,
                                        self.buf[yi+0][xi+1].z,
                                        self.buf[yi+1][xi+0].z)
                                GL.glNormal3f(n[0], n[1], n[2])
                                GL.glVertex3f(self.x[xi+0], self.y[yi+0],
                                        self.buf[yi+0][xi+0].z)
                                GL.glVertex3f(self.x[xi+0], self.y[yi+1],
                                        self.buf[yi+1][xi+0].z)
                                GL.glVertex3f(self.x[xi+1], self.y[yi+1],
                                        self.buf[yi+1][xi+1].z)
                                n = self.normal(self.buf[y+1][x+1].z,
                                        self.buf[y+0][x+1].z,
                                        self.buf[y+1][x+0].z)
                                GL.glNormal3f(n[0], n[1], n[2])
                                GL.glVertex3f(self.x[xi+0], self.y[yi+0],
                                        self.buf[yi+0][xi+0].z)
                                GL.glVertex3f(self.x[xi+1], self.y[yi+1],
                                        self.buf[yi+1][xi+1].z)
                                GL.glVertex3f(self.x[xi+1], self.y[yi+0],
                                        self.buf[yi+0][xi+1].z)
                                self.buf[yi][xi].changed = False
                            GL.glEnd()
                        GL.glEndList()
                    else:
                        GL.glNewList(self.list[y][x], GL.GL_COMPILE)
                        for yi in range(y0, y1-1):
                            GL.glBegin(GL.GL_QUADS)
                            for xi in range(x0, x1-1):
                                n = self.normal(self.buf[yi+0][xi+0].z,
                                        self.buf[yi+0][xi+1].z,
                                        self.buf[yi+1][xi+0].z)
                                GL.glNormal3f(n[0], n[1], n[2])
                                GL.glVertex3f(self.x[xi+0], self.y[yi+0],
                                        self.buf[yi+0][xi+0].z)
                                GL.glVertex3f(self.x[xi+0], self.y[yi+1],
                                        self.buf[yi+1][xi+0].z)
                                GL.glVertex3f(self.x[xi+1], self.y[yi+1],
                                        self.buf[yi+1][xi+1].z)
                                GL.glVertex3f(self.x[xi+1], self.y[yi+0],
                                        self.buf[yi+0][xi+1].z)
                                self.buf[yi][xi].changed = False
                            GL.glEnd()
                        GL.glEndList()


                GL.glCallList(self.list[y][x])

    # use display list with vertex buffers per cell (cell = group of quads) 
    def to_OpenGL_4(self):
        num_cell_x = NUM_CELL_X
        num_cell_y = NUM_CELL_Y

        dy = self.yres / num_cell_y
        if dy < 2:
            num_cell_y = 1
            dy = self.yres
        dx = self.xres/num_cell_x
        if dx < 2:
            num_cell_x = 1
            dx = self.xres

        for y in range(0, num_cell_y):
            y0 = y * dy
            y1 = y0 + dy + 1
            if y1 > self.yres:
                y1 = self.yres
            for x in range(0, num_cell_x):
                x0 = x * dx
                x1 = x0 + dx + 1
                if x1 > self.xres:
                    x1 = self.xres

                changed = False

                if self.changed:
                    for yi in range(y0, y1):
                        for xi in range(x0, x1):
                            if self.buf[yi][xi].changed:
                                changed = True
                                break

                if changed:
#                    print "cell[",y,"][",x,"]=",self.cell[y][x].list
                    if self.cell[y][x].list == -1:
                        self.cell[y][x].list = GL.glGenLists(1)
                        self.cell[y][x].array = (ctypes.c_float * 3 \
                                * ((y1 - y0) * (x1 - x0) * 2))()

                    GL.glEnableClientState(GL.GL_VERTEX_ARRAY)
                    GL.glVertexPointerf(self.cell[y][x].array)
                    GL.glNewList(self.cell[y][x].list, GL.GL_COMPILE)
                    idx = 0
                    for yi in range(y0, y1-1):
                        lineidx = idx
                        for xi in range(x0, x1):
                            self.buf[yi][xi].changed = False
                            self.buf[yi+1][xi].changed = False
                            self.cell[y][x].array[idx+0][0] = self.x[xi+0]
                            self.cell[y][x].array[idx+0][1] = self.y[yi+0]
                            self.cell[y][x].array[idx+0][2] = \
                                    self.buf[yi+0][xi+0].z
                            self.cell[y][x].array[idx+1][0] = self.x[xi+0]
                            self.cell[y][x].array[idx+1][1] = self.y[yi+1]
                            self.cell[y][x].array[idx+1][2] = \
                                    self.buf[yi+1][xi+0].z
                            idx += 2
                        GL.glDrawArrays(GL.GL_QUAD_STRIP, lineidx,
                                idx - lineidx + 1)
                    GL.glEndList()

                GL.glCallList(self.cell[y][x].list)

    # use display list with vertex and normal buffers per cell
    # (cell = group of quads) 
    def to_OpenGL_5(self):
        num_cell_x = NUM_CELL_X
        num_cell_y = NUM_CELL_Y

        dy = self.yres / num_cell_y
        if dy < 2:
            num_cell_y = 1
            dy = self.yres
        dx = self.xres / num_cell_x
        if dx < 2:
            num_cell_x = 1
            dx = self.xres

        GL.glEnableClientState(GL.GL_VERTEX_ARRAY)
        GL.glEnableClientState(GL.GL_NORMAL_ARRAY)

        for y in range(num_cell_y):
            y0 = y * dy
            y1 = y0 + dy + 1
            if y1 > self.yres:
                y1 = self.yres
            for x in range(num_cell_x):
                x0 = x * dx
                x1 = x0 + dx + 1
                if x1 > self.xres:
                    x1 = self.xres

                changed = False

                if self.changed:
                    for yi in range(y0, y1):
                        for xi in range(x0, x1):
                            if self.buf[yi][xi].changed:
                                changed = True
                                break

                if changed:
#                    print "cell[",y,"][",x,"]=",self.cell[y][x].list
                    if self.cell[y][x].list == -1:
                        self.cell[y][x].list = GL.glGenLists(1)
                        self.cell[y][x].vertex = (ctypes.c_float * 3 \
                                * ((y1 - y0) * (x1 - x0) * 4))()
                        self.cell[y][x].normal = (ctypes.c_float * 3 \
                                * ((y1 - y0) * (x1 - x0) * 4))()

                    GL.glVertexPointerf(self.cell[y][x].vertex)
                    GL.glNormalPointerf(self.cell[y][x].normal)
                    GL.glNewList(self.cell[y][x].list, GL.GL_COMPILE)
                    idx = 0
                    for yi in range(y0, y1-1):
                        lineidx = idx
                        for xi in range(x0, x1-1):
                            self.buf[yi][xi].changed = False
                            self.buf[yi+1][xi].changed = False
                            self.cell[y][x].vertex[idx+0][0] = self.x[xi+0]
                            self.cell[y][x].vertex[idx+0][1] = self.y[yi+0]
                            self.cell[y][x].vertex[idx+0][2] = \
                                    self.buf[yi+0][xi+0].z
                            self.cell[y][x].vertex[idx+1][0] = self.x[xi+0]
                            self.cell[y][x].vertex[idx+1][1] = self.y[yi+1]
                            self.cell[y][x].vertex[idx+1][2] = \
                                    self.buf[yi+1][xi+0].z
                            self.cell[y][x].vertex[idx+2][0] = self.x[xi+1]
                            self.cell[y][x].vertex[idx+2][1] = self.y[yi+1]
                            self.cell[y][x].vertex[idx+2][2] = \
                                    self.buf[yi+1][xi+1].z
                            self.cell[y][x].vertex[idx+3][0] = self.x[xi+1]
                            self.cell[y][x].vertex[idx+3][1] = self.y[yi+0]
                            self.cell[y][x].vertex[idx+3][2] = \
                                    self.buf[yi+0][xi+1].z

                            n = self.normal(self.buf[yi+0][xi+0].z,
                                    self.buf[yi+0][xi+1].z,
                                    self.buf[yi+1][xi+0].z)
                            self.cell[y][x].normal[idx+0][0] = n[0]
                            self.cell[y][x].normal[idx+0][1] = n[1]
                            self.cell[y][x].normal[idx+0][2] = n[2]
                            self.cell[y][x].normal[idx+1][0] = n[0]
                            self.cell[y][x].normal[idx+1][1] = n[1]
                            self.cell[y][x].normal[idx+1][2] = n[2]
                            self.cell[y][x].normal[idx+2][0] = n[0]
                            self.cell[y][x].normal[idx+2][1] = n[1]
                            self.cell[y][x].normal[idx+2][2] = n[2]
                            self.cell[y][x].normal[idx+3][0] = n[0]
                            self.cell[y][x].normal[idx+3][1] = n[1]
                            self.cell[y][x].normal[idx+3][2] = n[2]

                            idx += 4
                        GL.glDrawArrays(GL.GL_QUADS, lineidx, idx-lineidx+1)
                    GL.glEndList()

                GL.glCallList(self.cell[y][x].list)

        GL.glDisableClientState(GL.GL_VERTEX_ARRAY)
        GL.glDisableClientState(GL.GL_NORMAL_ARRAY)

    # use display list with vertex and normal and index buffers per cell
    # (cell = group of quads) 
    def to_OpenGL_6(self):
        num_cell_x = NUM_CELL_X
        num_cell_y = NUM_CELL_Y

        dy = self.yres/num_cell_y
        if dy < 2:
            num_cell_y = 1
            dy = self.yres
        dx = self.xres/num_cell_x
        if dx < 2:
            num_cell_x = 1
            dx = self.xres

        GL.glEnableClientState(GL.GL_INDEX_ARRAY)
        GL.glEnableClientState(GL.GL_VERTEX_ARRAY)
        GL.glEnableClientState(GL.GL_NORMAL_ARRAY)

        for y in range(num_cell_y):
            y0 = y * dy
            y1 = y0 + dy + 1
            if y1 > self.yres:
                y1 = self.yres
            for x in range(num_cell_x):
                x0 = x * dx
                x1 = x0 + dx + 1
                if x1 > self.xres:
                    x1 = self.xres

                changed = False

                if self.changed:
                    for yi in range(y0, y1 - 1):
                        for xi in range(x0, x1 - 1):
                            if self.buf[yi][xi].changed:
                                changed = True
                                break

                if changed:
                    #print "cell[",y,"][",x,"]=",self.cell[y][x].list
                    if self.cell[y][x].list == -1:
                        self.cell[y][x].list = GL.glGenLists(1)
                        self.cell[y][x].vertex = (ctypes.c_float * 3 \
                                * ((y1-y0) * (x1 - x0)))()
                        self.cell[y][x].normal = (ctypes.c_float * 3 \
                                * ((y1-y0) * (x1 - x0)))()
                        self.cell[y][x].index  = (ctypes.c_ushort \
                                * (4 * (y1 - y0 - 1) * (x1 - x0 - 1)))()
                    GL.glIndexPointers(self.cell[y][x].index)
                    GL.glVertexPointerf(self.cell[y][x].vertex)
                    GL.glNormalPointerf(self.cell[y][x].normal)

                    GL.glNewList(self.cell[y][x].list, GL.GL_COMPILE)
                    idx = 0
                    for yi in range(y0, y1):
                        for xi in range(x0, x1):
                            self.buf[yi][xi].changed = False
                            self.cell[y][x].vertex[idx][0] = self.x[xi]
                            self.cell[y][x].vertex[idx][1] = self.y[yi]
                            self.cell[y][x].vertex[idx][2] = self.buf[yi][xi].z

                            if (xi==self.xres - 1) or (yi == self.yres - 1):
                                n = self.normal(self.buf[yi-1][xi-1].z,
                                        self.buf[yi-1][xi].z,
                                        self.buf[yi][xi-1].z)
                            else:
                                n = self.normal(self.buf[yi][xi].z,
                                        self.buf[yi][xi+1].z,
                                        self.buf[yi+1][xi].z)

                            self.cell[y][x].normal[idx][0] = n[0]
                            self.cell[y][x].normal[idx][1] = n[1]
                            self.cell[y][x].normal[idx][2] = n[2]

                            idx += 1

                    idx = 0
                    for yi in range(y0, y1 - 1):
                        for xi in range(x0, x1 - 1):
                            self.cell[y][x].index[idx+0] = (yi - y0 + 0) \
                                    * (x1 - x0) + (xi - x0 + 0)
                            self.cell[y][x].index[idx+1] = (yi - y0 + 1) \
                                    * (x1 - x0) + (xi - x0 + 0)
                            self.cell[y][x].index[idx+2] = (yi - y0 + 1) \
                                    * (x1 - x0) + (xi - x0 + 1)
                            self.cell[y][x].index[idx+3] = (yi - y0 + 0) \
                                    * (x1 - x0) + (xi - x0 + 1)
                            idx += 4

                    GL.glDrawElements(GL.GL_QUADS, idx, GL.GL_UNSIGNED_SHORT,
                            self.cell[y][x].index)
                    GL.glEndList()
                GL.glCallList(self.cell[y][x].list)

        GL.glDisableClientState(GL.GL_VERTEX_ARRAY)
        GL.glDisableClientState(GL.GL_NORMAL_ARRAY)
        GL.glDisableClientState(GL.GL_INDEX_ARRAY)

