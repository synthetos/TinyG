# -*- coding: utf-8 -*-
"""
$Id: __init__.py 692 2010-09-24 01:48:54Z sumpfralle $

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

__all__ = ["utils", "Line", "Model", "Path", "Plane", "Point", "Triangle",
           "PolygonExtractor", "TriangleKdtree", "intersection", "kdtree",
           "Matrix", "Polygon"]

from pycam.Geometry.utils import epsilon
import math


def get_bisector(p1, p2, p3, up_vector):
    """ Calculate the bisector between p1, p2 and p3, whereas p2 is the origin
    of the angle.
    """
    d1 = p2.sub(p1).normalized()
    d2 = p2.sub(p3).normalized()
    bisector_dir = d1.add(d2).normalized()
    if bisector_dir is None:
        # the two vectors pointed to opposite directions
        bisector_dir = d1.cross(up_vector).normalized()
    else:
        skel_up_vector = bisector_dir.cross(p2.sub(p1))
        if up_vector.dot(skel_up_vector) < 0:
            # reverse the skeleton vector to point outwards
            bisector_dir = bisector_dir.mul(-1)
    return bisector_dir

def get_angle_pi(p1, p2, p3, up_vector, pi_factor=False):
    """ calculate the angle between three points
    Visualization:
            p3
           /
          /
         /\
        /  \
      p2--------p1
    The result is in a range between 0 and 2*PI.
    """
    d1 = p2.sub(p1).normalized()
    d2 = p2.sub(p3).normalized()
    if (d1 is None) or (d2 is None):
        return 2 * math.pi
    angle = math.acos(d1.dot(d2))
    # check the direction of the points (clockwise/anti)
    # The code is taken from Polygon.get_area
    value = [0, 0, 0]
    for (pa, pb) in ((p1, p2), (p2, p3), (p3, p1)):
        value[0] += pa.y * pb.z - pa.z * pb.y
        value[1] += pa.z * pb.x - pa.x * pb.z
        value[2] += pa.x * pb.y - pa.y * pb.x
    area = up_vector.x * value[0] + up_vector.y * value[1] \
            + up_vector.z * value[2]
    if area > 0:
        # The points are in anti-clockwise order. Thus the angle is greater
        # than 180 degree.
        angle = 2 * math.pi - angle
    if pi_factor:
        # the result is in the range of 0..2
        return angle / math.pi
    else:
        return angle


class TransformableContainer(object):
    """ a base class for geometrical objects containing other elements

    This class is mainly used for simplifying model transformations in a
    consistent way.

    Every subclass _must_ implement a 'next' generator returning (via yield)
    its children.
    Additionally a method 'reset_cache' for any custom re-initialization must
    be provided. This method is called when all children of the object were
    successfully transformed.

    A method 'get_children_count' for calculating the number of children
    (recursively) is necessary for the "callback" parameter of
    "transform_by_matrix".

    Optionally the method 'transform_by_matrix' may be used to perform
    object-specific calculations (e.g. retaining the 'normal' vector of a
    triangle).

    The basic primitives that are part of TransformableContainer _must_
    implement the above 'transform_by_matrix' method. These primitives are
    not required to be a subclass of TransformableContainer.
    """

    def transform_by_matrix(self, matrix, transformed_list=None, callback=None):
        if transformed_list is None:
            transformed_list = []
        # Prevent any kind of loops or double transformations (e.g. Points in
        # multiple containers (Line, Triangle, ...).
        # Use the 'id' builtin to prevent expensive object comparions.
        for item in self.next():
            if isinstance(item, TransformableContainer):
                item.transform_by_matrix(matrix, transformed_list,
                        callback=callback)
            elif not id(item) in transformed_list:
                # non-TransformableContainer do not care to update the
                # 'transformed_list'. Thus we need to do it.
                transformed_list.append(id(item))
                # Don't transmit the 'transformed_list' if the object is
                # not a TransformableContainer. It is not necessary and it
                # is hard to understand on the lowest level (e.g. Point).
                item.transform_by_matrix(matrix, callback=callback)
            # run the callback - e.g. for a progress counter
            if callback and callback():
                # user requesteded abort
                break
        self.reset_cache()

    def __iter__(self):
        return self

    def next(self):
        raise NotImplementedError(("'%s' is a subclass of " \
                + "'TransformableContainer' but it fails to implement the " \
                + "'next' generator") % str(type(self)))

    def get_children_count(self):
        raise NotImplementedError(("'%s' is a subclass of " \
                + "'TransformableContainer' but it fails to implement the " \
                + "'get_children_count' method") % str(type(self)))

    def reset_cache(self):
        raise NotImplementedError(("'%s' is a subclass of " \
                + "'TransformableContainer' but it fails to implement the " \
                + "'reset_cache' method") % str(type(self)))

    def is_completely_inside(self, minx=None, maxx=None, miny=None, maxy=None,
            minz=None, maxz=None):
        return ((minx is None) or (minx - epsilon <= self.minx)) \
                and ((maxx is None) or (self.maxx <= maxx + epsilon)) \
                and ((miny is None) or (miny - epsilon <= self.miny)) \
                and ((maxy is None) or (self.maxy <= maxy + epsilon)) \
                and ((minz is None) or (minz - epsilon <= self.minz)) \
                and ((maxz is None) or (self.maxz <= maxz + epsilon))

    def is_completely_outside(self, minx=None, maxx=None, miny=None, maxy=None,
            minz=None, maxz=None):
        return ((maxx is None) or (maxx + epsilon < self.minx)) \
                or ((minx is None) or (self.maxx < minx - epsilon)) \
                or ((maxy is None) or (maxy + epsilon < self.miny)) \
                or ((miny is None) or (self.maxy < miny - epsilon)) \
                or ((maxz is None) or (maxz + epsilon < self.minz)) \
                or ((minz is None) or (self.maxz < minz - epsilon))

