# -*- coding: utf-8 -*-
"""
$Id: __init__.py 722 2010-10-04 11:01:33Z sumpfralle $

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

__all__ = ["simplify_toolpath", "ToolpathList", "Toolpath", "Generator"]

from pycam.Geometry.Point import Point
from pycam.Geometry.utils import number
import pycam.Utils.log
import random
import os

log = pycam.Utils.log.get_logger()


def _check_colinearity(p1, p2, p3):
    v1 = p2.sub(p1).normalized()
    v2 = p3.sub(p2).normalized()
    # compare if the normalized distances between p1-p2 and p2-p3 are equal
    return v1 == v2


def simplify_toolpath(path):
    """ remove multiple points in a line from a toolpath

    If A, B, C and D are on a straight line, then B and C will be removed.
    This reduces memory consumption and avoids a severe slow-down of the machine
    when moving along very small steps.
    The toolpath is simplified _in_place_.
    @value path: a single separate segment of a toolpath
    @type path: pycam.Geometry.Path.Path
    """
    index = 1
    points = path.points
    while index < len(points) - 1:
        if _check_colinearity(points[index-1], points[index], points[index+1]):
            points.pop(index)
            # don't increase the counter - otherwise we skip one point
        else:
            index += 1


class ToolpathList(list):

    def add_toolpath(self, toolpath, name, toolpath_settings):
        self.append(Toolpath(toolpath, name, toolpath_settings))


class Toolpath(object):

    def __init__(self, toolpath, name, toolpath_settings):
        self.toolpath = toolpath
        self.name = name
        self.toolpath_settings = toolpath_settings
        self.visible = True
        self.color = None
        # generate random color
        self.set_color()

    def get_path(self):
        return self.toolpath

    def get_bounding_box(self):
        box = self.toolpath_settings.get_bounds()
        (minx, miny, minz), (maxx, maxy, maxz) = box.get_bounds()
        return (minx, maxx, miny, maxy, minz, maxz)

    def get_tool_settings(self):
        return self.toolpath_settings.get_tool_settings()

    def get_toolpath_settings(self):
        return self.toolpath_settings

    def get_meta_data(self):
        meta = self.toolpath_settings.get_string()
        start_marker = self.toolpath_settings.META_MARKER_START
        end_marker = self.toolpath_settings.META_MARKER_END
        return os.linesep.join((start_marker, meta, end_marker))

    def set_color(self, color=None):
        if color is None:
            self.color = (random.random(), random.random(), random.random())
        else:
            self.color = color

    def get_machine_time(self, start_position=None, safety_height=0.0):
        """ calculate an estimation of the time required for processing the
        toolpath with the machine

        @value start_position: (optional) the position of the tool before the
                start
        @type start_position: pycam.Geometry.Point.Point
        @rtype: float
        @returns: the machine time used for processing the toolpath in minutes
        """
        settings = self.toolpath_settings
        if start_position is None:
            start_position = Point(0, 0, 0)
        feedrate = number(settings.get_tool_settings()["feedrate"])
        result = {}
        result["time"] = 0
        result["position"] = start_position
        def move(new_pos):
            result["time"] += new_pos.sub(result["position"]).norm / feedrate
            result["position"] = new_pos
        # move to safey height at the starting position
        safety_height = number(safety_height)
        move(Point(start_position.x, start_position.y, safety_height))
        for path in self.get_path():
            # go to safety height (horizontally from the previous x/y location)
            if len(path.points) > 0:
                move(Point(path.points[0].x, path.points[0].y, safety_height))
            # go through all points of the path
            for point in path.points:
                move(point)
            # go to safety height (vertically up from the current x/y location)
            if len(path.points) > 0:
                move(Point(path.points[-1].x, path.points[-1].y, safety_height))
        return result["time"]


class Bounds:

    TYPE_RELATIVE_MARGIN = 0
    TYPE_FIXED_MARGIN = 1
    TYPE_CUSTOM = 2

    def __init__(self, bounds_type=None, bounds_low=None, bounds_high=None,
            reference=None):
        """ create a new Bounds instance

        @value bounds_type: any of TYPE_RELATIVE_MARGIN | TYPE_FIXED_MARGIN |
            TYPE_CUSTOM
        @type bounds_type: int
        @value bounds_low: the lower margin of the boundary compared to the
            reference object (for TYPE_RELATIVE_MARGIN | TYPE_FIXED_MARGIN) or
            the specific boundary values (for TYPE_CUSTOM). Only the lower
            values of the three axes (x, y and z) are given.
        @type bounds_low: (tuple|list) of float
        @value bounds_high: see 'bounds_low'
        @type bounds_high: (tuple|list) of float
        @value reference: optional default reference Bounds instance
        @type reference: Bounds
        """
        self.name = "No name"
        # set type
        self.bounds_type = None
        if bounds_type is None:
            self.set_type(Bounds.TYPE_CUSTOM)
        else:
            self.set_type(bounds_type)
        # store the bounds values
        self.bounds_low = None
        self.bounds_high = None
        if bounds_low is None:
            bounds_low = [0, 0, 0]
        if bounds_high is None:
            bounds_high = [0, 0, 0]
        self.set_bounds(bounds_low, bounds_high)
        self.reference = reference

    def __repr__(self):
        bounds_type_labels = ("relative", "fixed", "custom")
        return "Bounds(%s, %s, %s)" % (bounds_type_labels[self.bounds_type],
                self.bounds_low, self.bounds_high)

    def is_valid(self):
        for index in range(3):
            if self.bounds_low[index] > self.bounds_high[index]:
                return False
        else:
            return True

    def set_reference(self, reference):
        self.reference = reference

    def set_name(self, name):
        self.name = name

    def get_name(self):
        return self.name

    def get_type(self):
        return self.bounds_type

    def set_type(self, bounds_type):
        # complain if an unknown bounds_type value was given
        if not bounds_type in (Bounds.TYPE_RELATIVE_MARGIN,
                Bounds.TYPE_FIXED_MARGIN, Bounds.TYPE_CUSTOM):
            raise ValueError, "failed to create an instance of " \
                    + "pycam.Toolpath.Bounds due to an invalid value of " \
                    + "'bounds_type': %s" % repr(bounds_type)
        else:
            self.bounds_type = bounds_type

    def get_bounds(self):
        return self.bounds_low[:], self.bounds_high[:]

    def set_bounds(self, low=None, high=None):
        if not low is None:
            if len(low) != 3:
                raise ValueError, "lower bounds should be supplied as a " \
                        + "tuple/list of 3 items - but %d were given" % len(low)
            else:
                self.bounds_low = [number(value) for value in low]
        if not high is None:
            if len(high) != 3:
                raise ValueError, "upper bounds should be supplied as a " \
                        + "tuple/list of 3 items - but %d were given" \
                        % len(high)
            else:
                self.bounds_high = [number(value) for value in high]

    def get_absolute_limits(self, reference=None):
        """ calculate the current absolute limits of the Bounds instance

        @value reference: a reference object described by a tuple (or list) of
            three item. These three values describe only the lower boundary of
            this object (for the x, y and z axes). Each item must be a float
            value. This argument is ignored for the boundary type "TYPE_CUSTOM".
        @type reference: (tuple|list) of float
        @returns: a tuple of two lists containg the low and high limits
        @rvalue: tuple(list)
        """
        # use the default reference if none was given
        if reference is None:
            reference = self.reference
        # check if a reference is given (if necessary)
        if self.bounds_type \
                in (Bounds.TYPE_RELATIVE_MARGIN, Bounds.TYPE_FIXED_MARGIN):
            if reference is None:
                raise ValueError, "any non-custom boundary definition " \
                        + "requires an a reference object for caluclating " \
                        + "absolute limits"
            else:
                ref_low, ref_high = reference.get_absolute_limits()
        low = [None] * 3
        high = [None] * 3
        # calculate the absolute limits
        if self.bounds_type == Bounds.TYPE_RELATIVE_MARGIN:
            for index in range(3):
                dim_width = ref_high[index] - ref_low[index]
                low[index] = ref_low[index] \
                        - self.bounds_low[index] * dim_width
                high[index] = ref_high[index] \
                        + self.bounds_high[index] * dim_width
        elif self.bounds_type == Bounds.TYPE_FIXED_MARGIN:
            for index in range(3):
                low[index] = ref_low[index] - self.bounds_low[index]
                high[index] = ref_high[index] + self.bounds_high[index]
        elif self.bounds_type == Bounds.TYPE_CUSTOM:
            for index in range(3):
                low[index] = number(self.bounds_low[index])
                high[index] = number(self.bounds_high[index])
        else:
            # this should not happen
            raise NotImplementedError, "the function 'get_absolute_limits' is" \
                    + " currently not implemented for the bounds_type " \
                    + "'%s'" % str(self.bounds_type)
        return low, high

    def adjust_bounds_to_absolute_limits(self, limits_low, limits_high,
            reference=None):
        """ change the current bounds settings according to some absolute values

        This does not change the type of this bounds instance (e.g. relative).
        @value limits_low: a tuple describing the new lower absolute boundary
        @type limits_low: (tuple|list) of float
        @value limits_high: a tuple describing the new lower absolute boundary
        @type limits_high: (tuple|list) of float
        @value reference: a reference object described by a tuple (or list) of
            three item. These three values describe only the lower boundary of
            this object (for the x, y and z axes). Each item must be a float
            value. This argument is ignored for the boundary type "TYPE_CUSTOM".
        @type reference: (tuple|list) of float
        """
        # use the default reference if none was given
        if reference is None:
            reference = self.reference
        # check if a reference is given (if necessary)
        if self.bounds_type \
                in (Bounds.TYPE_RELATIVE_MARGIN, Bounds.TYPE_FIXED_MARGIN):
            if reference is None:
                raise ValueError, "any non-custom boundary definition " \
                        + "requires an a reference object for caluclating " \
                        + "absolute limits"
            else:
                ref_low, ref_high = reference.get_absolute_limits()
        # calculate the new settings
        if self.bounds_type == Bounds.TYPE_RELATIVE_MARGIN:
            for index in range(3):
                dim_width = ref_high[index] - ref_low[index]
                if dim_width == 0:
                    # We always loose relative margins if the specific dimension
                    # is zero. There is no way to avoid this.
                    message = "Non-zero %s boundary lost during conversion " \
                            + "to relative margins due to zero size " \
                            + "dimension '%s'." % "xyz"[index]
                    # Display warning messages, if we can't reach the requested
                    # absolute dimension.
                    if ref_low[index] != limits_low[index]:
                        log.info(message % "lower")
                    if ref_high[index] != limits_high[index]:
                        log.info(message % "upper")
                    self.bounds_low[index] = 0
                    self.bounds_high[index] = 0
                else:
                    self.bounds_low[index] = \
                            (ref_low[index] - limits_low[index]) / dim_width
                    self.bounds_high[index] = \
                            (limits_high[index] - ref_high[index]) / dim_width
        elif self.bounds_type == Bounds.TYPE_FIXED_MARGIN:
            for index in range(3):
                self.bounds_low[index] = ref_low[index] - limits_low[index]
                self.bounds_high[index] = limits_high[index] - ref_high[index]
        elif self.bounds_type == Bounds.TYPE_CUSTOM:
            for index in range(3):
                self.bounds_low[index] = limits_low[index]
                self.bounds_high[index] = limits_high[index]
        else:
            # this should not happen
            raise NotImplementedError, "the function " \
                    + "'adjust_bounds_to_absolute_limits' is currently not " \
                    + "implemented for the bounds_type '%s'" \
                    % str(self.bounds_type)

