#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
$Id: TriangleKdtreeTest.py 629 2010-08-23 16:53:06Z sumpfralle $

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

from pycam.Geometry.TriangleKdtree import *
from pycam.Geometry.Model import Model
from pycam.Importers.TestModel import TestModel

print "# get model"
testmodel = TestModel()
print "# subdivide"
model = testmodel.subdivide(5)
print "# build kdtree"
kdtree = BuildKdtree2d(model.triangles(), 2, 0.1)
#print "#kdtree=",kdtree

x = 2
y = 2
r = 0.1

minx = x-r
miny = y-r
maxx = x+r
maxy = y+r


print "# query kdtree"
ResetKdtree2dStats(False)
tests = SearchKdtree2d(kdtree, minx, maxx, miny, maxy)
print "# query kdtree"
ResetKdtree2dStats(True)
hits = SearchKdtree2d(kdtree, minx, maxx, miny, maxy)
#print "# hits=%d / tests=%d" % GetKdtree2dStats(), "/ triangles=%d" % len(model.triangles())
print "# hits=%d " % len(hits), "/ tests=%d" % len(tests), "/ triangles=%d" % len(model.triangles())

