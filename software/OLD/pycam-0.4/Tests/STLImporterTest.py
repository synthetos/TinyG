#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
$Id: STLImporterTest.py 629 2010-08-23 16:53:06Z sumpfralle $

Copyright 2010 Lode Leroy

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

import sys, time 
sys.path.insert(0,'.')


from pycam.Importers import STLImporter
from pycam.Gui.Visualization import ShowTestScene


if len(sys.argv)>1:
    filename = sys.argv[1]
else:
    filename = "Samples/STL/TestModel.stl"

start = time.clock()
model = STLImporter.ImportModel(filename)
end = time.clock()

print "time=", (end-start)

#ShowTestScene(model)

