# -*- coding: utf-8 -*-
"""
$ID$

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
from pycam.Geometry.Line import Line
import pycam.Geometry.Model
import pycam.Utils.log

log = pycam.Utils.log.get_logger()


class DXFParser:

    # see http://www.autodesk.com/techpubs/autocad/acad2000/dxf/group_code_value_types_dxf_01.htm
    MAX_CHARS_PER_LINE = 2049

    KEYS = {
        "MARKER": 0,
        "START_X": 10,
        "START_Y": 20,
        "START_Z": 30,
        "END_X": 11,
        "END_Y": 21,
        "END_Z": 31,
        "COLOR": 62,
    }

    def __init__(self, inputstream):
        self.inputstream = inputstream
        self.line_number = 0
        self.lines = []
        self._input_stack = []
        self.parse_content()
        self.optimize_line_order()

    def get_model(self):
        return {"lines": self.lines}

    def optimize_line_order(self):
        groups = []
        current_group = []
        groups.append(current_group)
        remaining_lines = self.lines[:]
        while remaining_lines:
            if not current_group:
                current_group.append(remaining_lines.pop(0))
            else:
                first_line = current_group[0]
                last_line = current_group[-1]
                for line in remaining_lines:
                    if last_line.p2 == line.p1:
                        current_group.append(line)
                        remaining_lines.remove(line)
                        break
                    if first_line.p1 == line.p2:
                        current_group.insert(0, line)
                        remaining_lines.remove(line)
                        break
                else:
                    current_group = []
                    groups.append(current_group)
        def get_distance_between_groups(group1, group2):
            forward = group1[-1].p2.sub(group2[0].p1).norm
            backward = group2[-1].p2.sub(group1[0].p1).norm
            return min(forward, backward)
        remaining_groups = groups[:]
        ordered_groups = []
        while remaining_groups:
            if not ordered_groups:
                ordered_groups.append(remaining_groups.pop(0))
            else:
                current_group = ordered_groups[-1]
                closest_distance = None
                for cmp_group in remaining_groups:
                    cmp_distance = get_distance_between_groups(current_group,
                            cmp_group)
                    if (closest_distance is None) \
                            or (cmp_distance < closest_distance):
                        closest_distance = cmp_distance
                        closest_group = cmp_group
                ordered_groups.append(closest_group)
                remaining_groups.remove(closest_group)
        result = []
        for group in ordered_groups:
            result.extend(group)
        self.lines = result

    def _push_on_stack(self, key, value):
        self._input_stack.append((key, value))

    def _read_key_value(self):
        if self._input_stack:
            return self._input_stack.pop()
        try:
            line1 = self.inputstream.readline(self.MAX_CHARS_PER_LINE).strip()
            line2 = self.inputstream.readline(self.MAX_CHARS_PER_LINE).strip()
        except IOError:
            return None, None
        if not line1 and not line2:
            return None, None
        try:
            line1 = int(line1)
        except ValueError:
            log.warn("DXFImporter: Invalid key in line " \
                    + "%d (int expected): %s" % (self.line_number, line1))
            return None, None
        if line1 in (self.KEYS["START_X"], self.KEYS["START_Y"],
                self.KEYS["START_Z"], self.KEYS["END_X"], self.KEYS["END_Y"],
                self.KEYS["END_Z"]):
            try:
                line2 = float(line2)
            except ValueError:
                log.warn("DXFImporter: Invalid input in line " \
                        + "%d (float expected): %s" % (self.line_number, line2))
                line1 = None
                line2 = None
        elif line1 in (self.KEYS["COLOR"],):
            try:
                line2 = int(line2)
            except ValueError:
                log.warn("DXFImporter: Invalid input in line " \
                        + "%d (float expected): %s" % (self.line_number, line2))
                line1 = None
                line2 = None
        else:
            line2 = line2.upper()
        self.line_number += 2
        return line1, line2

    def parse_content(self):
        key, value = self._read_key_value()
        while (not key is None) \
                and not ((key == self.KEYS["MARKER"]) and (value == "EOF")):
            if key == self.KEYS["MARKER"]:
                if value in ("SECTION", "TABLE", "LAYER", "ENDTAB", "ENDSEC"):
                    # we don't handle these meta-information
                    pass
                elif value == "LINE":
                    self.parse_line()
                else:
                    # not supported
                    log.warn("DXFImporter: Ignored unsupported element in " \
                            + "line %d: %s" % (self.line_number, value))
            key, value = self._read_key_value()

    def parse_line(self):
        start_line = self.line_number
        p1 = [None, None, None]
        p2 = [None, None, None]
        key, value = self._read_key_value()
        while (not key is None) and (key != self.KEYS["MARKER"]):
            if key == self.KEYS["START_X"]:
                p1[0] = value
            elif key == self.KEYS["START_Y"]:
                p1[1] = value
            elif key == self.KEYS["START_Z"]:
                p1[2] = value
            elif key == self.KEYS["END_X"]:
                p2[0] = value
            elif key == self.KEYS["END_Y"]:
                p2[1] = value
            elif key == self.KEYS["END_Z"]:
                p2[2] = value
            else:
                pass
            key, value = self._read_key_value()
        end_line = self.line_number
        # The last lines were not used - they are just the marker for the next
        # item.
        if not key is None:
            self._push_on_stack(key, value)
        if (None in p1) or (None in p2):
            log.warn("DXFImporter: Incomplete LINE definition between line " \
                    + "%d and %d" % (start_line, end_line))
        else:
            line = Line(Point(p1[0], p1[1], p1[2]), Point(p2[0], p2[1], p2[2]))
            if line.len > 0:
                self.lines.append(line)
            else:
                log.warn("DXFImporter: Ignoring zero-length LINE (between " \
                        + "input line %d and %d): %s" % (start_line, end_line,
                        line))

    def check_header(self):
        # TODO: this function is not used?
        # we expect "0" in the first line and "SECTION" in the second one
        key, value = self._read_key_value()
        if (key != self.KEYS["MARKER"]) or (value and (value != "SECTION")):
            log.error("DXFImporter: DXF file header not recognized")
            return None


def import_model(filename, program_locations=None, unit=None):
    try:
        infile = open(filename,"rb")
    except IOError, err_msg:
        log.error("DXFImporter: Failed to read file (%s): %s" \
                % (filename, err_msg))
        return None

    result = DXFParser(infile)

    lines = result.get_model()["lines"]

    if lines:
        model = pycam.Geometry.Model.ContourModel()
        for l in lines:
            model.append(l)
        if unit == "mm":
            # pstoedit uses inch internally - we need to scale
            log.info("DXFImporter: scaling model from inch to mm")
            model.scale(25.4)
        log.info("DXFImporter: Imported DXF model: %d lines / %d polygons" \
                % (len(lines), len(model.get_polygons())))
        return model
    else:
        log.error("DXFImporter: No supported elements found in DXF file!")
        return None

