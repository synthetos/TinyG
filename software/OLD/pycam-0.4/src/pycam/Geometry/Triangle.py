# -*- coding: utf-8 -*-
"""
$Id: Triangle.py 711 2010-10-02 01:41:14Z sumpfralle $

Copyright 2008-2010 Lode Leroy
Copyright 2010 Lars Kruse <devel@sumpfralle.de>

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

from pycam.Geometry.Point import Point, Vector
from pycam.Geometry.Plane import Plane
from pycam.Geometry.Line import Line
from pycam.Geometry import TransformableContainer


try:
    import OpenGL.GL as GL
    import OpenGL.GLU as GLU
    import OpenGL.GLUT as GLUT
    GL_enabled = True
except ImportError:
    GL_enabled = False


class Triangle(TransformableContainer):
    id = 0

    # points are expected to be in ClockWise order
    def __init__(self, p1=None, p2=None, p3=None, n=None):
        self.id = Triangle.id
        Triangle.id += 1
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.normal = n
        self.reset_cache()

    def reset_cache(self):
        self.minx = min(self.p1.x, self.p2.x, self.p3.x)
        self.miny = min(self.p1.y, self.p2.y, self.p3.y)
        self.minz = min(self.p1.z, self.p2.z, self.p3.z)
        self.maxx = max(self.p1.x, self.p2.x, self.p3.x)
        self.maxy = max(self.p1.y, self.p2.y, self.p3.y)
        self.maxz = max(self.p1.z, self.p2.z, self.p3.z)
        self.e1 = Line(self.p1, self.p2)
        self.e2 = Line(self.p2, self.p3)
        self.e3 = Line(self.p3, self.p1)
        # calculate normal, if p1-p2-pe are in clockwise order
        if self.normal is None:
            self.normal = self.p3.sub(self.p1).cross(self.p2.sub( \
                    self.p1)).normalized()
        if not isinstance(self.normal, Vector):
            self.normal = self.normal.get_vector()
        self.center = self.p1.add(self.p2).add(self.p3).div(3)
        self.plane = Plane(self.center, self.normal)
        # calculate circumcircle (resulting in radius and middle)
        denom = self.p2.sub(self.p1).cross(self.p3.sub(self.p2)).norm
        self.radius = (self.p2.sub(self.p1).norm \
                * self.p3.sub(self.p2).norm * self.p3.sub(self.p1).norm) \
                / (2 * denom)
        self.radiussq = self.radius ** 2
        denom2 = 2 * denom * denom
        alpha = self.p3.sub(self.p2).normsq \
                * self.p1.sub(self.p2).dot(self.p1.sub(self.p3)) / denom2
        beta  = self.p1.sub(self.p3).normsq \
                * self.p2.sub(self.p1).dot(self.p2.sub(self.p3)) / denom2
        gamma = self.p1.sub(self.p2).normsq \
                * self.p3.sub(self.p1).dot(self.p3.sub(self.p2)) / denom2
        self.middle = Point(
                self.p1.x * alpha + self.p2.x * beta + self.p3.x * gamma,
                self.p1.y * alpha + self.p2.y * beta + self.p3.y * gamma,
                self.p1.z * alpha + self.p2.z * beta + self.p3.z * gamma)

    def __repr__(self):
        return "Triangle%d<%s,%s,%s>" % (self.id, self.p1, self.p2, self.p3)

    def next(self):
        yield self.p1
        yield self.p2
        yield self.p3
        yield self.normal

    def get_points(self):
        return (self.p1, self.p2, self.p3)

    def get_children_count(self):
        # tree points per triangle
        return 7

    def to_OpenGL(self):
        if not GL_enabled:
            return
        GL.glBegin(GL.GL_TRIANGLES)
        # use normals to improve lighting (contributed by imyrek)
        normal_t = self.normal
        GL.glNormal3f(normal_t.x, normal_t.y, normal_t.z)
        # The triangle's points are in clockwise order, but GL expects
        # counter-clockwise sorting.
        GL.glVertex3f(self.p1.x, self.p1.y, self.p1.z)
        GL.glVertex3f(self.p3.x, self.p3.y, self.p3.z)
        GL.glVertex3f(self.p2.x, self.p2.y, self.p2.z)
        GL.glEnd()
        if False: # display surface normals
            n = self.normal
            c = self.center
            d = 0.5
            GL.glBegin(GL.GL_LINES)
            GL.glVertex3f(c.x, c.y, c.z)
            GL.glVertex3f(c.x+n.x*d, c.y+n.y*d, c.z+n.z*d)
            GL.glEnd()
        if False: # display bounding sphere
            GL.glPushMatrix()
            middle = self.middle
            GL.glTranslate(middle.x, middle.y, middle.z)
            if not hasattr(self, "_sphere"):
                self._sphere = GLU.gluNewQuadric()
            GLU.gluSphere(self._sphere, self.radius, 10, 10)
            GL.glPopMatrix()
        if False: # draw triangle id on triangle face
            GL.glPushMatrix()
            cc = GL.glGetFloatv(GL.GL_CURRENT_COLOR)
            c = self.center
            GL.glTranslate(c.x, c.y, c.z)
            p12 = self.p1.add(self.p2).mul(0.5)
            p3_12 = self.p3.sub(p12).normalized()
            p2_1 = self.p1.sub(self.p2).normalized()
            pn = p2_1.cross(p3_12)
            GL.glMultMatrixf((p2_1.x, p2_1.y, p2_1.z, 0, p3_12.x, p3_12.y,
                    p3_12.z, 0, pn.x, pn.y, pn.z, 0, 0, 0, 0, 1))
            n = self.normal.mul(0.01)
            GL.glTranslatef(n.x, n.y, n.z)
            GL.glScalef(0.003, 0.003, 0.003)
            w = 0
            id_string = "%s." % str(self.id)
            for ch in id_string:
                w += GLUT.glutStrokeWidth(GLUT.GLUT_STROKE_ROMAN, ord(ch))
            GL.glTranslate(-w/2, 0, 0)
            GL.glColor4f(1, 1, 1, 1)
            for ch in id_string:
                GLUT.glutStrokeCharacter(GLUT.GLUT_STROKE_ROMAN, ord(ch))
            GL.glPopMatrix()
            GL.glColor4f(cc[0], cc[1], cc[2], cc[3])
        if False: # draw point id on triangle face
            cc = GL.glGetFloatv(GL.GL_CURRENT_COLOR)
            c = self.center
            p12 = self.p1.add(self.p2).mul(0.5)
            p3_12 = self.p3.sub(p12).normalized()
            p2_1 = self.p1.sub(self.p2).normalized()
            pn = p2_1.cross(p3_12)
            n = self.normal.mul(0.01)
            for p in (self.p1, self.p2, self.p3):
                GL.glPushMatrix()
                pp = p.sub(p.sub(c).mul(0.3))
                GL.glTranslate(pp.x, pp.y, pp.z)
                GL.glMultMatrixf((p2_1.x, p2_1.y, p2_1.z, 0, p3_12.x, p3_12.y,
                        p3_12.z, 0, pn.x, pn.y, pn.z, 0, 0, 0, 0, 1))
                GL.glTranslatef(n.x, n.y, n.z)
                GL.glScalef(0.001, 0.001, 0.001)
                w = 0
                for ch in str(p.id):
                    w += GLUT.glutStrokeWidth(GLUT.GLUT_STROKE_ROMAN, ord(ch))
                    GL.glTranslate(-w/2, 0, 0)
                GL.glColor4f(0.5, 1, 0.5, 1.0)
                for ch in str(p.id):
                    GLUT.glutStrokeCharacter(GLUT.GLUT_STROKE_ROMAN, ord(ch))
                GL.glPopMatrix()
            GL.glColor4f(cc[0], cc[1], cc[2], cc[3])

    def is_point_inside(self, p):
        # http://www.blackpawn.com/texts/pointinpoly/default.html
        # Compute vectors
        v0 = self.p3.sub(self.p1)
        v1 = self.p2.sub(self.p1)
        v2 = p.sub(self.p1)
        # Compute dot products
        dot00 = v0.dot(v0)
        dot01 = v0.dot(v1)
        dot02 = v0.dot(v2)
        dot11 = v1.dot(v1)
        dot12 = v1.dot(v2)
        # Compute barycentric coordinates
        denom = dot00 * dot11 - dot01 * dot01
        if denom == 0:
            return False
        invDenom = 1.0 / denom
        # Originally, "u" and "v" are multiplied with "1/denom".
        # We don't do this to avoid division by zero (for triangles that are
        # "almost" invalid).
        u = (dot11 * dot02 - dot01 * dot12) * invDenom
        v = (dot00 * dot12 - dot01 * dot02) * invDenom
        # Check if point is in triangle
        return (u > 0) and (v > 0) and (u + v < 1)

    def subdivide(self, depth):
        sub = []
        if depth == 0:
            sub.append(self)
        else:
            p4 = self.p1.add(self.p2).div(2)
            p5 = self.p2.add(self.p3).div(2)
            p6 = self.p3.add(self.p1).div(2)
            sub += Triangle(self.p1, p4, p6).subdivide(depth - 1)
            sub += Triangle(p6, p5, self.p3).subdivide(depth - 1)
            sub += Triangle(p6, p4, p5).subdivide(depth - 1)
            sub += Triangle(p4, self.p2, p5).subdivide(depth - 1)
        return sub

