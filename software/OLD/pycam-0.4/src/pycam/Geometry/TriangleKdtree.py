# -*- coding: utf-8 -*-
"""
$Id: TriangleKdtree.py 629 2010-08-23 16:53:06Z sumpfralle $

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

from pycam.Geometry.kdtree import kdtree, Node

overlaptest = True

def SearchKdtree2d(tree, minx, maxx, miny, maxy):
    if tree.bucket:
        triangles = []
        for n in tree.nodes:
            if not overlaptest:
                triangles.append(n.triangle)
            else:
                if not (n.bound[0] > maxx
                        or n.bound[1] < minx
                        or n.bound[2] > maxy
                        or n.bound[3] < miny):
                    triangles.append(n.triangle)
        return triangles
    else:
        if tree.cutdim == 0:
            if maxx < tree.minval:
                return []
            elif maxx < tree.cutval:
                return SearchKdtree2d(tree.lo, minx, maxx, miny, maxy)
            else:
                return SearchKdtree2d(tree.lo, minx, maxx, miny, maxy) \
                        + SearchKdtree2d(tree.hi, minx, maxx, miny, maxy)
        elif tree.cutdim == 1:
            if minx > tree.maxval:
                return []
            elif minx > tree.cutval:
                return SearchKdtree2d(tree.hi, minx, maxx, miny, maxy)
            else:
                return SearchKdtree2d(tree.lo, minx, maxx, miny, maxy) \
                        + SearchKdtree2d(tree.hi, minx, maxx, miny, maxy)
        elif tree.cutdim == 2:
            if maxy < tree.minval:
                return []
            elif maxy < tree.cutval:
                return SearchKdtree2d(tree.lo, minx, maxx, miny, maxy)
            else:
                return SearchKdtree2d(tree.lo, minx, maxx, miny, maxy) \
                        + SearchKdtree2d(tree.hi, minx, maxx, miny, maxy)
        elif tree.cutdim == 3:
            if miny > tree.maxval:
                return []
            elif miny > tree.cutval:
                return SearchKdtree2d(tree.hi, minx, maxx, miny, maxy)
            else:
                return SearchKdtree2d(tree.lo, minx, maxx, miny, maxy) \
                        + SearchKdtree2d(tree.hi, minx, maxx, miny, maxy)


class TriangleKdtree(kdtree):

    def __init__(self, triangles, cutoff=3, cutoff_distance=1.0):
        nodes = []
        for t in triangles:
            n = Node()
            n.triangle = t
            n.bound = []
            n.bound.append(min(t.p1.x, t.p2.x, t.p3.x))
            n.bound.append(max(t.p1.x, t.p2.x, t.p3.x))
            n.bound.append(min(t.p1.y, t.p2.y, t.p3.y))
            n.bound.append(max(t.p1.y, t.p2.y, t.p3.y))
            nodes.append(n)
        super(TriangleKdtree, self).__init__(nodes, cutoff, cutoff_distance)

    def Search(self, minx, maxx, miny, maxy):
        return SearchKdtree2d(self, minx, maxx, miny, maxy)

