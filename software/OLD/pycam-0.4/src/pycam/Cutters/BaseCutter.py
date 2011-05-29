# -*- coding: utf-8 -*-
"""
$Id: BaseCutter.py 750 2010-10-12 01:50:09Z sumpfralle $

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


import pycam.Utils.threading
from pycam.Geometry.Point import Point
from pycam.Geometry.utils import number, INFINITE, epsilon
from pycam.Geometry.intersection import intersect_circle_point, \
        intersect_cylinder_point, intersect_cylinder_line
import uuid


class BaseCutter(object):
    id = 0
    vertical = Point(0, 0, -1)

    def __init__(self, radius, location=None, height=None):
        if location is None:
            location = Point(0, 0, 0)
        if height is None:
            height = 10
        radius = number(radius)
        self.height = number(height)
        self.id = BaseCutter.id
        BaseCutter.id += 1
        self.radius = radius
        self.radiussq = radius ** 2
        self.required_distance = 0
        self.distance_radius = self.radius
        self.distance_radiussq = self.distance_radius ** 2
        self.shape = {}
        self.moveto(location)
        self.uuid = None
        self.update_uuid()

    def get_minx(self, start=None):
        if start is None:
            start = self.location
        return start.x - self.distance_radius

    def get_maxx(self, start=None):
        if start is None:
            start = self.location
        return start.x + self.distance_radius

    def get_miny(self, start=None):
        if start is None:
            start = self.location
        return start.y - self.distance_radius

    def get_maxy(self, start=None):
        if start is None:
            start = self.location
        return start.y + self.distance_radius

    def update_uuid(self):
        self.uuid = uuid.uuid4()

    def __repr__(self):
        return "BaseCutter"

    def __cmp__(self, other):
        """ Compare Cutters by shape and size (ignoring the location)
        This function should be overridden by subclasses, if they describe
        cutters with a shape depending on more than just the radius.
        See the ToroidalCutter for an example.
        """
        if self.__class__ == other.__class__:
            return cmp(self.radius, other.radius)
        else:
            # just return a string comparison
            return cmp(str(self), str(other))

    def set_required_distance(self, value):
        if value >= 0:
            self.required_distance = number(value)
            self.distance_radius = self.radius + self.get_required_distance()
            self.distance_radiussq = self.distance_radius * self.distance_radius
            self.update_uuid()

    def get_required_distance(self):
        return self.required_distance

    def moveto(self, location):
        # "moveto" is used for collision detection calculation.
        self.location = location
        for shape, set_pos_func in self.shape.values():
            set_pos_func(location.x, location.y, location.z)

    def intersect(self, direction, triangle, start=None):
        raise NotImplementedError("Inherited class of BaseCutter does not " \
                + "implement the required function 'intersect'.")

    def drop(self, triangle, start=None):
        if start is None:
            start = self.location
        # check bounding box collision
        if self.get_minx(start) > triangle.maxx + epsilon:
            return None
        if self.get_maxx(start) < triangle.minx - epsilon:
            return None
        if self.get_miny(start) > triangle.maxy + epsilon:
            return None
        if self.get_maxy(start) < triangle.miny - epsilon:
            return None

        # check bounding circle collision
        c = triangle.middle
        if (c.x - start.x) ** 2 + (c.y - start.y) ** 2 \
                > (self.distance_radiussq + 2 * self.distance_radius \
                    * triangle.radius + triangle.radiussq) + epsilon:
            return None

        (cl, d, cp) = self.intersect(BaseCutter.vertical, triangle, start=start)
        return cl

    def intersect_circle_triangle(self, direction, triangle, start=None):
        (cl, ccp, cp, d) = self.intersect_circle_plane(direction, triangle,
                start=start)
        if cp and triangle.is_point_inside(cp):
            return (cl, d, cp)
        return (None, INFINITE, None)

    def intersect_circle_vertex(self, direction, point, start=None):
        (cl, ccp, cp, l) = self.intersect_circle_point(direction, point,
                start=start)
        return (cl, l, cp)

    def intersect_circle_edge(self, direction, edge, start=None):
        (cl, ccp, cp, l) = self.intersect_circle_line(direction, edge,
                start=start)
        if cp:
            # check if the contact point is between the endpoints
            m = cp.sub(edge.p1).dot(edge.dir)
            if (m < -epsilon) or (m > edge.len + epsilon):
                return (None, INFINITE, cp)
        return (cl, l, cp)

    def intersect_cylinder_point(self, direction, point, start=None):
        if start is None:
            start = self.location
        (ccp, cp, l) = intersect_cylinder_point(
                start.sub(self.location).add(self.center), self.axis,
                self.distance_radius, self.distance_radiussq, direction, point)
        # offset intersection
        if ccp:
            cl = cp.add(start.sub(ccp))
            return (cl, ccp, cp, l)
        return (None, None, None, INFINITE)

    def intersect_cylinder_vertex(self, direction, point, start=None):
        if start is None:
            start = self.location
        (cl, ccp, cp, l) = self.intersect_cylinder_point(direction, point,
                start=start)
        if ccp and ccp.z < start.sub(self.location).add(self.center).z:
            return (None, INFINITE, None)
        return (cl, l, cp)

    def intersect_cylinder_line(self, direction, edge, start=None):
        if start is None:
            start = self.location
        (ccp, cp, l) = intersect_cylinder_line(
                start.sub(self.location).add(self.center), self.axis,
                self.distance_radius, self.distance_radiussq, direction, edge)
        # offset intersection
        if ccp:
            cl = start.add(cp.sub(ccp))
            return (cl, ccp, cp, l)
        return (None, None, None, INFINITE)

    def intersect_cylinder_edge(self, direction, edge, start=None):
        if start is None:
            start = self.location
        (cl, ccp, cp, l) = self.intersect_cylinder_line(direction, edge,
                start=start)
        if not ccp:
            return (None, INFINITE, None)
        m = cp.sub(edge.p1).dot(edge.dir)
        if (m < -epsilon) or (m > edge.len + epsilon):
            return (None, INFINITE, None)
        if ccp.z < start.sub(self.location).add(self.center).z:
            return (None, INFINITE, None)
        return (cl, l, cp)

