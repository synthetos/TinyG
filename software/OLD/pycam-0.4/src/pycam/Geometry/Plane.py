# -*- coding: utf-8 -*-
"""
$Id: Plane.py 758 2010-10-12 17:00:37Z sumpfralle $

Copyright 2008-2009 Lode Leroy

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
from pycam.Geometry.utils import INFINITE, epsilon
from pycam.Geometry.Point import Point, Vector
# "Line" is imported later to avoid circular imports
#from pycam.Geometry.Line import Line


class Plane(TransformableContainer):
    id = 0
    def __init__(self, p, n):
        self.id = Plane.id
        Plane.id += 1
        self.p = p
        self.n = n
        if not isinstance(self.n, Vector):
            self.n = self.n.get_vector()

    def __repr__(self):
        return "Plane<%s,%s>" % (self.p, self.n)

    def __cmp__(self, other):
        if self.__class__ == other.__class__:
            if self.p == other.p:
                return cmp(self.n, other.n)
            else:
                return cmp(self.p, other.p)
        else:
            return cmp(str(self), str(other))

    def next(self):
        yield self.p
        yield self.n

    def get_children_count(self):
        # a plane always consists of two points
        return 2

    def reset_cache(self):
        # nothing to be done (but required for TransformableContainer)
        pass

    def intersect_point(self, direction, point):
        if (not direction is None) and (direction.norm != 1):
            # calculations will go wrong, if the direction is not a unit vector
            direction = direction.normalized()
        if direction is None:
            return (None, INFINITE)
        denom = self.n.dot(direction)
        if denom == 0:
            return (None, INFINITE)
        l = -(self.n.dot(point) - self.n.dot(self.p)) / denom
        cp = point.add(direction.mul(l))
        return (cp, l)

    def intersect_triangle(self, triangle, counter_clockwise=False):
        """ Returns the line of intersection of a triangle with a plane.
        "None" is returned, if:
            - the triangle does not intersect with the plane
            - all vertices of the triangle are on the plane
        The line always runs clockwise through the triangle.
        """
        # don't import Line in the header -> circular import
        from pycam.Geometry.Line import Line
        collisions = []
        for edge, point in ((triangle.e1, triangle.p1),
                (triangle.e2, triangle.p2),
                (triangle.e3, triangle.p3)):
            cp, l = self.intersect_point(edge.dir, point)
            # filter all real collisions
            # We don't want to count vertices double -> thus we only accept
            # a distance that is lower than the length of the edge.
            if (not cp is None) and (-epsilon < l < edge.len - epsilon):
                collisions.append(cp)
            elif (cp is None) and (self.n.dot(edge.dir) == 0):
                cp, dist = self.intersect_point(self.n, point)
                if abs(dist) < epsilon:
                    # the edge is on the plane
                    collisions.append(point)
        if len(collisions) == 3:
            # All points of the triangle are on the plane.
            # We don't return a waterline, as there should be another non-flat
            # triangle with the same waterline.
            return None
        if len(collisions) == 2:
            collision_line = Line(collisions[0], collisions[1])
            # no further calculation, if the line is zero-sized
            if collision_line.len == 0:
                return collision_line
            cross = self.n.cross(collision_line.dir)
            if (cross.dot(triangle.normal) < 0) == bool(not counter_clockwise):
                # anti-clockwise direction -> revert the direction of the line
                collision_line = Line(collision_line.p2, collision_line.p1)
            return collision_line
        elif len(collisions) == 1:
            # only one point is on the plane
            # This waterline (with zero length) should be of no use.
            return None
        else:
            return None

    def get_point_projection(self, point):
        p, dist = self.intersect_point(self.n, point)
        return p

    def get_line_projection(self, line):
        # don't import Line in the header -> circular import
        from pycam.Geometry.Line import Line
        proj_p1 = self.get_point_projection(line.p1)
        proj_p2 = self.get_point_projection(line.p2)
        return Line(proj_p1, proj_p2)

