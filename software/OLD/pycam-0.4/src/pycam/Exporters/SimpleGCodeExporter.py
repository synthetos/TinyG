# -*- coding: utf-8 -*-
"""
$Id: SimpleGCodeExporter.py 629 2010-08-23 16:53:06Z sumpfralle $

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

from pycam.Exporters.gcode import gcode
import os

# simplistic GCode exporter
# does each run, and moves the tool to the safetyheight in between

class SimpleGCodeExporter:

    def __init__(self, destination, unit, feedrate,
            speed, safety_height=None, tool_id=1, finish_program=False,
            max_skip_safety_distance=None, comment=None):
        self._last_path_point = None
        self._max_skip_safety_distance = max_skip_safety_distance
        if isinstance(destination, basestring):
            # open the file
            self.destination = file(destination,"w")
            self._close_stream_on_exit = True
        else:
            # assume that "destination" is something like a StringIO instance
            # or an open file
            self.destination = destination
            # don't close the stream if we did not open it on our own
            self._close_stream_on_exit = False
        if comment:
            self.add_comment(comment)
        if unit == "mm":
            self.destination.write("G21\n")
        else:
            self.destination.write("G20\n")
        self.gcode = gcode(safetyheight=safety_height, tool_id=tool_id)
        self._finish_program_on_exit = finish_program
        self.destination.write(self.gcode.begin() + "\n")
        self.destination.write("F" + str(feedrate) + "\n")
        self.destination.write("S" + str(speed) + "\n")
        # enable "exact path" mode (prefer accuracy over speed)
        self.destination.write(self.gcode.exactpath())
        self.destination.write(self.gcode.safety() + "\n")

    def close(self):
        gc = self.gcode
        self.destination.write(gc.safety() + "\n")
        if self._finish_program_on_exit:
            self.destination.write(gc.end() + "\n")
        if self._close_stream_on_exit:
            self.destination.close()

    def _check_distance_for_skipping_safety_height(self, new_point):
        if (self._last_path_point is None) \
                or (self._max_skip_safety_distance is None):
            return False
        distance = new_point.sub(self._last_path_point).norm
        return distance <= self._max_skip_safety_distance

    def add_comment(self, comment):
        for line in comment.split(os.linesep):
            self.destination.write(";%s\n" % line)

    def AddPath(self, path):
        gc = self.gcode
        point = path.points[0]
        # first move to the safety height if the distance to the last point
        # does not exceed the given maximum
        if not self._check_distance_for_skipping_safety_height(point):
            # move to safety height at the end of the previous path
            if not self._last_path_point is None:
                self.destination.write(gc.rapid(self._last_path_point.x,
                        self._last_path_point.y, gc.safetyheight) + "\n")
            # move to safety height for the start of the current path
            self.destination.write(gc.rapid(point.x, point.y, gc.safetyheight) \
                    + "\n")
        for point in path.points:
            self.destination.write(gc.cut(point.x, point.y, point.z) + "\n")
        self._last_path_point = point

    def AddPathList(self, pathlist):
        for path in pathlist:
            self.AddPath(path)
        # add the move to safety height to the last path
        if not self._last_path_point is None:
            self.destination.write(self.gcode.rapid(self._last_path_point.x,
                    self._last_path_point.y, self.gcode.safetyheight) + "\n")


def ExportPathList(destination, pathlist, unit, feedrate, speed, **kwargs):
    exporter = SimpleGCodeExporter(destination, unit, feedrate, speed, **kwargs)
    exporter.AddPathList(pathlist)
    exporter.close()

