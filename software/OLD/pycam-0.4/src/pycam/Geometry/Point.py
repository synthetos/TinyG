# -*- coding: utf-8 -*-
"""
$Id: Point.py 763 2010-10-13 23:11:28Z sumpfralle $

Copyright 2010 Lars Kruse <devel@sumpfralle.de>
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

from pycam.Geometry.utils import epsilon, sqrt, number

def _is_near(x, y):
    return abs(x - y) < epsilon


class Point:
    id = 0

    def __init__(self, x, y, z):
        self.id = Point.id
        Point.id += 1
        self.x = number(x)
        self.y = number(y)
        self.z = number(z)
        self.reset_cache()

    def __repr__(self):
        return "Point%d<%g,%g,%g>" % (self.id, self.x, self.y, self.z)

    def __cmp__(self, other):
        """ Two points are equal if all dimensions are identical.
        Otherwise the result is based on the individual x/y/z comparisons.
        """
        if self.__class__ == other.__class__:
            if (_is_near(self.x, other.x)) and (_is_near(self.y, other.y)) \
                    and (_is_near(self.z, other.z)):
                return 0
            elif not _is_near(self.x, other.x):
                return cmp(self.x, other.x)
            elif not _is_near(self.y, other.y):
                return cmp(self.y, other.y)
            else:
                return cmp(self.z, other.z)
        else:
            return cmp(str(self), str(other))

    def transform_by_matrix(self, matrix, transformed_list=None, callback=None):
        x = self.x * matrix[0][0] + self.y * matrix[0][1] \
                + self.z * matrix[0][2] + matrix[0][3]
        y = self.x * matrix[1][0] + self.y * matrix[1][1] \
                + self.z * matrix[1][2] + matrix[1][3]
        z = self.x * matrix[2][0] + self.y * matrix[2][1] \
                + self.z * matrix[2][2] + matrix[2][3]
        self.x = x
        self.y = y
        self.z = z
        if callback:
            callback()
        self.reset_cache()

    def reset_cache(self):
        self.normsq = self.dot(self)
        self.norm = sqrt(self.normsq)
        
    def mul(self, c):
        c = number(c)
        return Point(self.x * c, self.y * c, self.z * c)

    def div(self, c):
        c = number(c)
        return Point(self.x / c, self.y / c, self.z / c)

    def add(self, p):
        return Point(self.x + p.x, self.y + p.y, self.z + p.z)

    def sub(self, p):
        return Point(self.x - p.x, self.y - p.y, self.z - p.z)

    def dot(self, p):
        return self.x * p.x + self.y * p.y + self.z * p.z

    def size(self):
        return sqrt(self.dot(self))

    def cross(self, p):
        return Point(self.y * p.z - p.y * self.z, p.x * self.z - self.x * p.z,
                self.x * p.y - p.x * self.y)

    def normalized(self):
        n = self.norm
        if n == 0:
            return None
        else:
            return Point(self.x / n, self.y / n, self.z / n)

    def is_inside(self, minx=None, maxx=None, miny=None, maxy=None, minz=None,
            maxz=None):
        return ((minx is None) or (minx - epsilon <= self.x)) \
                and ((maxx is None) or (self.x <= maxx + epsilon)) \
                and ((miny is None) or (miny - epsilon <= self.y)) \
                and ((maxy is None) or (self.y <= maxy + epsilon)) \
                and ((minz is None) or (minz - epsilon <= self.z)) \
                and ((maxz is None) or (self.z <= maxz + epsilon))

    def get_vector(self):
        return Vector(self.x, self.y, self.z)


class Vector(Point):
    """ The Vector class is similar to the Point class. The only difference
    is that vectors are not shifted during transformations. This feature
    is necessary for normals (e.g. of Triangles or Planes).
    """

    def transform_by_matrix(self, matrix, transformed_list=None, callback=None):
        x = self.x * matrix[0][0] + self.y * matrix[0][1] \
                + self.z * matrix[0][2]
        y = self.x * matrix[1][0] + self.y * matrix[1][1] \
                + self.z * matrix[1][2]
        z = self.x * matrix[2][0] + self.y * matrix[2][1] \
                + self.z * matrix[2][2]
        self.x = x
        self.y = y
        self.z = z
        if callback:
            callback()
        self.reset_cache()

    def __repr__(self):
        return "Vector%d<%g,%g,%g>" % (self.id, self.x, self.y, self.z)

