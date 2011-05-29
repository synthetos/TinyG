# -*- coding: utf-8 -*-
"""
$Id: PushCutter.py 775 2010-10-14 11:52:18Z sumpfralle $

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

from pycam.Geometry.Point import Point
from pycam.PathGenerators import get_free_paths_ode, get_free_paths_triangles
import pycam.PathProcessors.PathAccumulator
from pycam.Geometry.utils import epsilon, ceil
from pycam.Utils.threading import run_in_parallel
from pycam.Utils import ProgressCounter
import math


# We need to use a global function here - otherwise it does not work with
# the multiprocessing Pool.
def _process_one_line((p1, p2, depth, models, cutter, physics)):
    if physics:
        points = get_free_paths_ode(physics, p1, p2, depth=depth)
    else:
        points = get_free_paths_triangles(models, cutter, p1, p2)
    return points


class PushCutter:

    def __init__(self, cutter, models, path_processor, physics=None):
        self.cutter = cutter
        self.models = models
        self.pa = path_processor
        self.physics = physics
        # check if we use a PolygonExtractor
        self._use_polygon_extractor = hasattr(self.pa, "pe")

    def GenerateToolPath(self, motion_grid, draw_callback=None):
        # calculate the number of steps

        # Transfer the grid (a generator) into a list of lists and count the
        # items.
        grid = []
        num_of_grid_positions = 0
        for layer in motion_grid:
            lines = []
            for line in layer:
                lines.append(list(line))
                num_of_grid_positions += len(lines[-1])
            grid.append(lines)

        num_of_layers = len(grid)

        progress_counter = ProgressCounter(num_of_grid_positions, draw_callback)

        current_layer = 0
        for layer_grid in grid:
            # update the progress bar and check, if we should cancel the process
            if draw_callback and draw_callback(text="PushCutter: processing" \
                        + " layer %d/%d" % (current_layer + 1, num_of_layers)):
                # cancel immediately
                break

            self.pa.new_direction(0)
            self.GenerateToolPathSlice(layer_grid, draw_callback, progress_counter)
            self.pa.end_direction()
            self.pa.finish()

            current_layer += 1

        if self._use_polygon_extractor and (len(self.models) > 1):
            other_models = self.models[1:]
            # TODO: this is complicated and hacky :(
            # we don't use parallelism or ODE (for the sake of simplicity)
            final_pa = pycam.PathProcessors.SimpleCutter()
            for path in self.pa.paths:
                final_pa.new_scanline()
                pairs = []
                for index in range(len(path.points) - 1):
                    pairs.append((path.points[index], path.points[index + 1]))
                for p1, p2 in pairs:
                    free_points = get_free_paths_triangles(other_models,
                            self.cutter, p1, p2)
                    for p in free_points:
                        final_pa.append(p)
                final_pa.end_scanline()
            final_pa.finish()
            return final_pa.paths
        else:
            return self.pa.paths

    def GenerateToolPathSlice(self, layer_grid, draw_callback=None,
            progress_counter=None):
        """ only dx or (exclusive!) dy may be bigger than zero
        """
        # max_deviation_x = dx/accuracy
        accuracy = 20
        max_depth = 20

        # calculate the required number of steps in each direction
        distance = layer_grid[0][-1].sub(layer_grid[0][0]).norm
        step_width = distance / len(layer_grid[0])
        depth = math.log(accuracy * distance / step_width) / math.log(2)
        depth = max(ceil(depth), 4)
        depth = min(depth, max_depth)

        # the ContourCutter pathprocessor does not work with combined models
        if self._use_polygon_extractor:
            models = self.models[:1]
        else:
            models = self.models

        args = []
        for line in layer_grid:
            p1, p2 = line
            args.append((p1, p2, depth, models, self.cutter, self.physics))

        # ODE does not work with multi-threading
        disable_multiprocessing = not self.physics is None
        for points in run_in_parallel(_process_one_line, args,
                disable_multiprocessing=disable_multiprocessing):
            if points:
                self.pa.new_scanline()
                for p in points:
                    self.pa.append(p)
                if draw_callback:
                    draw_callback(tool_position=p, toolpath=self.pa.paths)
                self.pa.end_scanline()
            # update the progress counter
            if progress_counter and progress_counter.increment():
                # quit requested
                break

