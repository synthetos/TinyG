# -*- coding: utf-8 -*-
"""
$Id: __init__.py 629 2010-08-23 16:53:06Z sumpfralle $

Copyright 2008 Lode Leroy
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

__all__ = ["STLImporter", "DXFImporter", "SVGImporter", "TestModel"]

import pycam.Utils.log
import pycam.Importers.DXFImporter
import pycam.Importers.STLImporter
import pycam.Importers.SVGImporter

import os

log = pycam.Utils.log.get_logger()


def detect_file_type(filename):
    failure = (None, None)
    if not os.path.isfile(filename):
        return failure
    else:
        # check all listed importers
        # TODO: this should be done by evaluating the header of the file
        if filename.lower().endswith(".stl"):
            return ("stl", pycam.Importers.STLImporter.ImportModel)
        elif filename.lower().endswith(".dxf"):
            return ("dxf", pycam.Importers.DXFImporter.import_model)
        elif filename.lower().endswith(".svg"):
            return ("svg", pycam.Importers.SVGImporter.import_model)
        else:
            log.error("Importers: Failed to detect the model type of '%s'." \
                    % filename + " Is the file extension (stl/dxf/svg) correct?")
            return failure

