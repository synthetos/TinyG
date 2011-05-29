# -*- coding: utf-8 -*-
"""
$Id: __init__.py 629 2010-08-23 16:53:06Z sumpfralle $

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

__all__ = [ "SphericalCutter", "CylindricalCutter", "ToroidalCutter",
        "BaseCutter" ]

from pycam.Cutters.BaseCutter import BaseCutter
from pycam.Cutters.SphericalCutter import SphericalCutter
from pycam.Cutters.CylindricalCutter import CylindricalCutter
from pycam.Cutters.ToroidalCutter import ToroidalCutter


def get_tool_from_settings(tool_settings, height=None):
    """ get the tool specified by the relevant settings

    The settings must include:
      - "shape": one of "SphericalCutter", "CylindricalCutter" and
        "ToroidalCutter"
      - "radius": the tool radius
    The following settings are optional or shape specific:
      - "torus_radius": necessary for ToroidalCutter

    @type tool_settings: dict
    @value tool_settings: contains the attributes of the tool
    @type height: float
    @value height: the height of the tool
    @rtype: BaseCutter | basestring
    @return: a tool object or an error string
    """
    cuttername = tool_settings["shape"]
    radius = tool_settings["tool_radius"]
    if cuttername == "SphericalCutter":
        return SphericalCutter(radius, height=height)
    elif cuttername == "CylindricalCutter":
        return CylindricalCutter(radius, height=height)
    elif cuttername == "ToroidalCutter":
        toroid = tool_settings["torus_radius"]
        return ToroidalCutter(radius, toroid, height=height)
    else:
        return "Invalid cutter shape: '%s' is not known" % str(cuttername)

