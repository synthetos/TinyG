#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
$Id: VisualizationTest.py 629 2010-08-23 16:53:06Z sumpfralle $

Copyright 2008 Lode Leroy

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

from pycam.Gui.Visualization import Visualization
from pycam.Importers.TestModel import TestModel

model = TestModel()

def DrawScene():
    model.to_OpenGL()

Visualization("VisualizationTest", DrawScene)
