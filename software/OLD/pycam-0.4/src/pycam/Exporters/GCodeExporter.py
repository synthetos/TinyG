# -*- coding: utf-8 -*-
"""
$Id: GCodeExporter.py 635 2010-08-26 00:15:47Z sumpfralle $

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


DEFAULT_HEADER = ("G40 (disable tool radius compensation)",
                "G49 (disable_tool_length_compensation)",
                "G80 (cancel_modal_motion)",
                "G54 (select_coordinate_system_1)",
                "G90 (use_absolute_coordinates)")

PATH_MODES = {"exact_path": 0, "exact_stop": 1, "continuous": 2}


class GCodeGenerator:

    def __init__(self, destination, metric_units=True, safety_height=0.0,
            toggle_spindle_status=False, max_skip_safety_distance=None,
            header=None, comment=None):
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
        self.safety_height = safety_height
        self.gcode = gcode(safetyheight=self.safety_height)
        self.max_skip_safety_distance = max_skip_safety_distance
        self.toggle_spindle_status = toggle_spindle_status
        self.comment = comment
        self._last_path_point = None
        self._finished = False
        if comment:
            self.add_comment(comment)
        if header is None:
            self.append(DEFAULT_HEADER)
        else:
            self.append(header)
        if metric_units:
            self.append("G21 (metric)")
        else:
            self.append("G20 (imperial)")

    def set_speed(self, feedrate=None, spindle_speed=None):
        if not feedrate is None:
            self.append("F%.4f" % feedrate)
        if not spindle_speed is None:
            self.append("S%.4f" % spindle_speed)

    def set_path_mode(self, mode, motion_tolerance=None,
            naive_cam_tolerance=None):
        result = ""
        if mode == PATH_MODES["exact_path"]:
            result = "G61 (exact path mode)"
        elif mode == PATH_MODES["exact_stop"]:
            result = "G61.1 (exact stop mode)"
        elif mode == PATH_MODES["continuous"]:
            if motion_tolerance is None:
                result = "G64 (continuous mode with maximum speed)"
            elif naive_cam_tolerance is None:
                result = "G64 P%f (continuous mode with tolerance)" \
                        % motion_tolerance
            else:
                result = ("G64 P%f Q%f (continuous mode with tolerance and " \
                        + "cleanup") % (motion_tolerance, naive_cam_tolerance)
        else:
            raise ValueError("GCodeGenerator: invalid path mode (%s)" \
                    % str(mode))
        self.append(result)

    def add_path_list(self, paths, tool_id=None, max_skip_safety_distance=None,
            comment=None):
        if max_skip_safety_distance is None:
            max_skip_safety_distance = self.max_skip_safety_distance
        if not comment is None:
            self.add_comment(comment)
        if not tool_id is None:
            # Move straight up to safety height (avoiding any collisions on the
            # way to the tool changer).
            self.append(self.gcode.safety())
            self.append("T%d M6" % tool_id)
        if self.toggle_spindle_status:
            self.append("M3 (start spindle)")
            self.append(self.gcode.delay(2))
        # move straight up to safety height
        self.append(self.gcode.safety())
        for path in paths:
            self.add_path(path, max_skip_safety_distance=max_skip_safety_distance)
        # go back to safety height
        self.append(self.gcode.safety())
        if self.toggle_spindle_status:
            self.append("M5 (stop spindle)")

    def _check_distance_for_skipping_safety_height(self, new_point,
            max_skip_safety_distance):
        if (self._last_path_point is None) \
                or (max_skip_safety_distance is None):
            return False
        distance = new_point.sub(self._last_path_point).norm
        return distance <= max_skip_safety_distance

    def add_path(self, path, max_skip_safety_distance=None):
        if not path:
            return
        point = path.points[0]
        # first move to the safety height if the distance to the last point
        # does not exceed the given maximum
        if not self._check_distance_for_skipping_safety_height(point,
                max_skip_safety_distance):
            # move to safety height at the end of the previous path
            if not self._last_path_point is None:
                self.append(self.gcode.safety())
            # move to safety height for the start of the current path
            self.append(self.gcode.rapid(point.x, point.y,
                    self.safety_height))
        for point in path.points:
            self.append(self.gcode.cut(point.x, point.y, point.z))
        self._last_path_point = point

    def finish(self):
        self.append(self.gcode.safety())
        self.append("M2 (end program)")
        self._finished = True

    def add_comment(self, comment):
        if isinstance(comment, basestring):
            lines = comment.split(os.linesep)
        else:
            lines = comment
        for line in lines:
            self.append(";%s" % line)

    def append(self, command):
        if self._finished:
            raise TypeError("GCodeGenerator: can't add further commands to a " \
                    + "finished GCodeGenerator instance: %s" % str(command))
        if isinstance(command, basestring):
            command = [command]
        for line in command:
            self.destination.write(line + os.linesep)

