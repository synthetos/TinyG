# -*- coding: utf-8 -*-
"""
$Id: Line.py 697 2010-09-24 17:17:25Z sumpfralle $

Copyright 2008-2009 Lode Leroy
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

from pycam.Geometry import TransformableContainer
from pycam.Geometry.Point import Point
from pycam.Geometry.Plane import Plane
from pycam.Geometry.utils import epsilon, sqrt
import math


try:
    import OpenGL.GL as GL
    GL_enabled = True
except ImportError:
    GL_enabled = False


class Line(TransformableContainer):
    id = 0

    def __init__(self, p1, p2):
        self.id = Line.id
        Line.id += 1
        self.p1 = p1
        self.p2 = p2
        self.reset_cache()

    def __repr__(self):
        return "Line<%g,%g,%g>-<%g,%g,%g>" % (self.p1.x, self.p1.y, self.p1.z,
                self.p2.x, self.p2.y, self.p2.z)

    def __cmp__(self, other):
        """ Two lines are equal if both pairs of points are at the same
        locations.
        Otherwise the result is based on the comparison of the first and then
        the second point.
        """
        if self.__class__ == other.__class__:
            if (self.p1 == other.p1) and (self.p2 == other.p2):
                return 0
            elif self.p1 != other.p1:
                return cmp(self.p1, other.p1)
            else:
                return cmp(self.p2, other.p2)
        else:
            return cmp(str(self), str(other))

    def next(self):
        yield self.p1
        yield self.p2

    def get_children_count(self):
        # a line always contains two points
        return 2

    def reset_cache(self):
        self.dir = self.p2.sub(self.p1).normalized()
        self.len = self.p2.sub(self.p1).norm
        self.minx = min(self.p1.x, self.p2.x)
        self.miny = min(self.p1.y, self.p2.y)
        self.minz = min(self.p1.z, self.p2.z)
        self.maxx = max(self.p1.x, self.p2.x)
        self.maxy = max(self.p1.y, self.p2.y)
        self.maxz = max(self.p1.z, self.p2.z)

    def get_points(self):
        return (self.p1, self.p2)

    def point_with_length_multiply(self, l):
        return self.p1.add(self.dir.mul(l*self.len))

    def get_length_line(self, length):
        """ return a line with the same direction and the specified length
        """
        return Line(self.p1, self.p1.add(self.dir.mul(length)))

    def closest_point(self, p):
        v = self.dir
        if v is None:
            # for zero-length lines
            return self.p1
        l = self.p1.dot(v) - p.dot(v)
        return self.p1.sub(v.mul(l))

    def dist_to_point_sq(self, p):
        return p.sub(self.closest_point(p)).normsq

    def dist_to_point(self, p):
        return sqrt(self.dist_to_point_sq(p))
    
    def is_point_inside(self, p):
        if (p == self.p1) or (p == self.p2):
            # these conditions are not covered by the code below
            return True
        dir1 = p.sub(self.p1).normalized()
        dir2 = self.p2.sub(p).normalized()
        # True if the two parts of the line have the same direction or if the
        # point is self.p1 or self.p2.
        return (dir1 == dir2 == self.dir) or (dir1 is None) or (dir2 is None)

    def to_OpenGL(self):
        if GL_enabled:
            GL.glBegin(GL.GL_LINES)
            GL.glVertex3f(self.p1.x, self.p1.y, self.p1.z)
            GL.glVertex3f(self.p2.x, self.p2.y, self.p2.z)
            # (optional) draw arrows for visualizing the direction of each line
            if False:
                line = (self.p2.x - self.p1.x, self.p2.y - self.p1.y)
                if line[0] == 0:
                    ortho = (1.0, 0.0)
                elif line[1] == 0:
                    ortho = (0.0, 1.0)
                else:
                    ortho = (1.0 / line[0], -1.0 / line[1])
                line_size = sqrt((line[0] ** 2) + (line[1] ** 2))
                ortho_size = sqrt((ortho[0] ** 2) + (ortho[1] ** 2))
                ortho_dest_size = line_size / 10.0
                ortho = (ortho[0] * ortho_dest_size / ortho_size,
                        ortho[1] * ortho_dest_size / ortho_size)
                line_back = (-line[0] * ortho_dest_size / line_size,
                        -line[1] * ortho_dest_size / line_size)
                p3 = (self.p2.x + ortho[0] + line_back[0],
                        self.p2.y + ortho[1] + line_back[1], self.p2.z)
                p4 = (self.p2.x - ortho[0] + line_back[0],
                        self.p2.y - ortho[1] + line_back[1], self.p2.z)
                GL.glVertex3f(p3[0], p3[1], p3[2])
                GL.glVertex3f(self.p2.x, self.p2.y, self.p2.z)
                GL.glVertex3f(p4[0], p4[1], p4[2])
                GL.glVertex3f(self.p2.x, self.p2.y, self.p2.z)
            GL.glEnd()

    def get_intersection(self, line, infinite_lines=False):
        """ Get the point of intersection between two lines. Intersections
        outside the length of these lines are ignored.
        Returns (None, None) if no valid intersection was found.
        Otherwise the result is (CollisionPoint, distance). Distance is between
        0 and 1.
        """
        x1, x2, x3, x4 = self.p1, self.p2, line.p1, line.p2
        a = x2.sub(x1)
        b = x4.sub(x3)
        c = x3.sub(x1)
        # see http://mathworld.wolfram.com/Line-LineIntersection.html (24)
        try:
            factor = c.cross(b).dot(a.cross(b)) / a.cross(b).normsq
        except ZeroDivisionError:
            # lines are parallel
            # check if they are _one_ line
            if a.cross(c).norm != 0:
                # the lines are parallel with a distance
                return None, None
            # the lines are on one straight
            candidates = []
            if self.is_point_inside(x3):
                candidates.append((x3, c.norm / a.norm))
            elif self.is_point_inside(x4):
                candidates.append((x4, line.p2.sub(self.p1).norm / a.norm))
            elif line.is_point_inside(x1):
                candidates.append((x1, 0))
            elif line.is_point_inside(x2):
                candidates.append((x2, 1))
            else:
                return None, None
            # return the collision candidate with the lowest distance
            candidates.sort(key=lambda (cp, dist): dist)
            return candidates[0]
        if infinite_lines or (-epsilon <= factor <= 1 + epsilon):
            intersection = x1.add(a.mul(factor))
            # check if the intersection is between x3 and x4
            if infinite_lines:
                return intersection, factor
            elif (min(x3.x, x4.x) - epsilon <= intersection.x <= max(x3.x, x4.x) + epsilon) \
                    and (min(x3.y, x4.y) - epsilon <= intersection.y <= max(x3.y, x4.y) + epsilon) \
                    and (min(x3.z, x4.z) - epsilon <= intersection.z <= max(x3.z, x4.z) + epsilon):
                return intersection, factor
            else:
                # intersection outside of the length of line(x3, x4)
                return None, None
        else:
            # intersection outside of the length of line(x1, x2)
            return None, None

    def get_cropped_line(self, minx, maxx, miny, maxy, minz, maxz):
        if self.is_completely_inside(minx, maxx, miny, maxy, minz, maxz):
            return self
        elif self.is_completely_outside(minx, maxx, miny, maxy, minz, maxz):
            return None
        else:
            # the line needs to be cropped
            # generate the six planes of the cube for possible intersections
            minp = Point(minx, miny, minz)
            maxp = Point(maxx, maxy, maxz)
            planes = [
                    Plane(minp, Point(1, 0, 0)),
                    Plane(minp, Point(0, 1, 0)),
                    Plane(minp, Point(0, 0, 1)),
                    Plane(maxp, Point(1, 0, 0)),
                    Plane(maxp, Point(0, 1, 0)),
                    Plane(maxp, Point(0, 0, 1)),
            ]
            # calculate all intersections
            intersections = [plane.intersect_point(self.dir, self.p1)
                    for plane in planes]
            # remove all intersections outside the box and outside the line
            valid_intersections = [(cp, dist) for cp, dist in intersections
                    if cp and (-epsilon <= dist <= self.len + epsilon) and \
                            cp.is_inside(minx, maxx, miny, maxy, minz, maxz)]
            # sort the intersections according to their distance to self.p1
            valid_intersections.sort(
                    cmp=lambda (cp1, l1), (cp2, l2): cmp(l1, l2))
            # Check if p1 is within the box - otherwise use the closest
            # intersection. The check for "valid_intersections" is necessary
            # to prevent an IndexError due to floating point inaccuracies.
            if self.p1.is_inside(minx, maxx, miny, maxy, minz, maxz) \
                    or not valid_intersections:
                new_p1 = self.p1
            else:
                new_p1 = valid_intersections[0][0]
            # Check if p2 is within the box - otherwise use the intersection
            # most distant from p1.
            if self.p2.is_inside(minx, maxx, miny, maxy, minz, maxz) \
                    or not valid_intersections:
                new_p2 = self.p2
            else:
                new_p2 = valid_intersections[-1][0]
            if new_p1 == new_p2:
                # no real line
                return None
            else:
                return Line(new_p1, new_p2)

