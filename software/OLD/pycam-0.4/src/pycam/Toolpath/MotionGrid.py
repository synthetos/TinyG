# -*- coding: utf-8 -*-
"""
$Id: MotionGrid.py 759 2010-10-13 01:20:50Z sumpfralle $

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

from pycam.Geometry.Point import Point
from pycam.Geometry.utils import epsilon
import math


GRID_DIRECTION_X = 0
GRID_DIRECTION_Y = 1
GRID_DIRECTION_XY = 2

MILLING_STYLE_IGNORE = 0
MILLING_STYLE_CONVENTIONAL = 1
MILLING_STYLE_CLIMB = 2

START_X = 0x1
START_Y = 0x2
START_Z = 0x4

def isiterable(obj):
    try:
        iter(obj)
        return True
    except TypeError:
        return False

def floatrange(start, end, inc=None, steps=None, reverse=False):
    if reverse:
        start, end = end, start
        # 'inc' will be adjusted below anyway
    if abs(start - end) < epsilon:
        yield start
    elif inc is None and steps is None:
        raise ValueError("floatrange: either 'inc' or 'steps' must be provided")
    elif (not steps is None) and (steps < 2):
        raise ValueError("floatrange: 'steps' must be greater than 1")
    else:
        # the input is fine
        # reverse increment, if it does not suit start/end
        if steps is None:
            if ((end - start) > 0) != (inc > 0):
                inc = -inc
            steps = int(math.ceil(float(end - start) / inc) + 1)
        inc = float(end - start) / (steps - 1)
        for index in range(steps):
            yield start + inc * index

def get_fixed_grid_line(start, end, line_pos, z, step_width=None,
        grid_direction=GRID_DIRECTION_X):
    if step_width is None:
        # useful for PushCutter operations
        steps = (start, end)
    elif isiterable(step_width):
        steps = step_width
    else:
        steps = floatrange(start, end, inc=step_width)
    if grid_direction == GRID_DIRECTION_X:
        get_point = lambda pos: Point(pos, line_pos, z)
    else:
        get_point = lambda pos: Point(line_pos, pos, z)
    for pos in steps:
        yield get_point(pos)

def get_fixed_grid_layer(minx, maxx, miny, maxy, z, line_distance,
        step_width=None, grid_direction=GRID_DIRECTION_X,
        milling_style=MILLING_STYLE_IGNORE, start_position=0):
    if grid_direction == GRID_DIRECTION_XY:
        raise ValueError("'get_one_layer_fixed_grid' does not accept XY " \
                + "direction")
    # zigzag is only available if the milling 
    zigzag = (milling_style == MILLING_STYLE_IGNORE)
    # If we happen to start at a position that collides with the milling style,
    # then we need to move to the closest other corner. Here we decide, which
    # would be the best alternative.
    def get_alternative_start_position(start):
        if (maxx - minx) <= (maxy - miny):
            # toggle the X position bit
            return start ^ START_X
        else:
            # toggle the Y position bit
            return start ^ START_Y
    if grid_direction == GRID_DIRECTION_X:
        primary_dir = START_X
        secondary_dir = START_Y
    else:
        primary_dir = START_Y
        secondary_dir = START_X
    # Determine the starting direction (assuming we begin at the lower x/y
    # coordinates.
    if milling_style == MILLING_STYLE_IGNORE:
        # just move forward - milling style is not important
        pass
    elif (milling_style == MILLING_STYLE_CLIMB) == (grid_direction == GRID_DIRECTION_X):
        if bool(start_position & START_X) == bool(start_position & START_Y):
            # we can't start from here - choose an alternative
            start_position = get_alternative_start_position(start_position)
    elif (milling_style == MILLING_STYLE_CONVENTIONAL) == (grid_direction == GRID_DIRECTION_X):
        if bool(start_position & START_X) != bool(start_position & START_Y):
            # we can't start from here - choose an alternative
            start_position = get_alternative_start_position(start_position)
    else:
        raise ValueError("Invalid milling style given: %s" % str(milling_style))
    # sort out the coordinates (primary/secondary)
    if grid_direction == GRID_DIRECTION_X:
        start, end = minx, maxx
        line_start, line_end = miny, maxy
    else:
        start, end = miny, maxy
        line_start, line_end = minx, maxx
    # switch start/end if we move from high to low
    if start_position & primary_dir:
        start, end = end, start
    if start_position & secondary_dir:
        line_start, line_end = line_end, line_start
    # calculate the line positions
    if isiterable(line_distance):
        lines = line_distance
    else:
        lines = floatrange(line_start, line_end, inc=line_distance)
    # at the end of the layer we will be on the other side of the 2nd direction
    end_position = start_position ^ secondary_dir
    # the final position will probably be on the other side (primary)
    if not zigzag:
        end_position ^= primary_dir
    # calculate each line
    def get_lines(start, end, end_position):
        result = []
        for line_pos in lines:
            result.append(get_fixed_grid_line(start, end, line_pos, z,
                    step_width=step_width, grid_direction=grid_direction))
            if zigzag:
                start, end = end, start
                end_position ^= primary_dir
        return result, end_position
    return get_lines(start, end, end_position)

def get_fixed_grid(bounds, layer_distance, line_distance, step_width=None,
        grid_direction=GRID_DIRECTION_X, milling_style=MILLING_STYLE_IGNORE,
        start_position=START_Z):
    """ Calculate the grid positions for toolpath movements
    """
    low, high = bounds.get_absolute_limits()
    if isiterable(layer_distance):
        layers = layer_distance
    elif layer_distance is None:
        # useful for DropCutter
        layers = [low[2]]
    else:
        layers = floatrange(low[2], high[2], inc=layer_distance,
                reverse=bool(start_position & START_Z))
    def get_layers_with_direction(layers):
        for layer in layers:
            if grid_direction != GRID_DIRECTION_Y:
                yield (layer, GRID_DIRECTION_X)
            if grid_direction != GRID_DIRECTION_X:
                yield (layer, GRID_DIRECTION_Y)
    for z, direction in get_layers_with_direction(layers):
        result, start_position = get_fixed_grid_layer(low[0], high[0],
                low[1], high[1], z, line_distance, step_width=step_width,
                grid_direction=direction, milling_style=milling_style,
                start_position=start_position)
        yield result

