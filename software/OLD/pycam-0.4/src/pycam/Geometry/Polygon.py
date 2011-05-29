# -*- coding: utf-8 -*-
"""
$Id$

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

from pycam.Geometry.Line import Line
from pycam.Geometry.Point import Point
from pycam.Geometry.Plane import Plane
from pycam.Geometry import TransformableContainer, get_bisector
from pycam.Geometry.utils import number, epsilon
import pycam.Geometry.Matrix as Matrix
import pycam.Utils.log
import math
# import later to avoid circular imports
#from pycam.Geometry.Model import ContourModel


log = pycam.Utils.log.get_logger()


class Polygon(TransformableContainer):

    def __init__(self, plane=None):
        super(Polygon, self).__init__()
        if plane is None:
            # the default plane points upwards along the z axis
            plane = Plane(Point(0, 0, 0), Point(0, 0, 1))
        self.plane = plane
        self._points = []
        self._is_closed = False
        self.maxx = None
        self.minx = None
        self.maxy = None
        self.miny = None
        self.maxz = None
        self.minz = None
        self._lines_cache = None
        self._area_cache = None

    def append(self, line):
        if not self.is_connectable(line):
            raise ValueError("This line does not fit to the polygon")
        elif line.len == 0:
            raise ValueError("A line with zero length may not be part of a " \
                    + "polygon")
        else:
            if not self._points:
                self._points.append(line.p1)
                self._update_limits(line.p1)
                self._points.append(line.p2)
                self._update_limits(line.p2)
            elif self._points[-1] == line.p1:
                # the new Line can be added to the end of the polygon
                if line.dir == line.p1.sub(self._points[-1]).normalized():
                    # Remove the last point, if the previous point combination
                    # is in line with the new Line. This avoids unnecessary
                    # points on straight lines.
                    self._points.pop(-1)
                if line.p2 != self._points[0]:
                    self._points.append(line.p2)
                    self._update_limits(line.p2)
                else:
                    self._is_closed = True
            else:
                # the new Line can be added to the beginning of the polygon
                if line.dir == self._points[0].sub(line.p2).normalized():
                    # Avoid points on straight lines - see above.
                    self._points.pop(0)
                if line.p1 != self._points[-1]:
                    self._points.insert(0, line.p1)
                    self._update_limits(line.p1)
                else:
                    self._is_closed = True

    def __len__(self):
        return len(self._points)

    def __str__(self):
        if self._is_closed:
            status = "closed"
        else:
            status = "open"
        return "Polygon (%s) %s" % (status, [point for point in self._points])

    def reverse_direction(self):
        self._points.reverse()
        self.reset_cache()

    def is_connectable(self, line):
        if self._is_closed:
            return False
        elif not self._points:
            # empty polygons can be connected with any line
            return True
        elif line.p1 == self._points[-1]:
            return True
        elif line.p2 == self._points[0]:
            return True
        else:
            return False

    def next(self):
        for point in self._points:
            yield point
        yield self.plane

    def get_children_count(self):
        return len(self._points) + self.plane.get_children_count()

    def get_area(self):
        """ calculate the area covered by a line group
        Currently this works only for line groups in an xy-plane.
        Returns zero for empty line groups or for open line groups.
        Returns negative values for inner hole.
        """
        if not self._points:
            return 0
        if not self._is_closed:
            return 0
        if self._area_cache is None:
            # calculate the area for the first time
            value = [0, 0, 0]
            # taken from: http://www.wikihow.com/Calculate-the-Area-of-a-Polygon
            # and: http://softsurfer.com/Archive/algorithm_0101/algorithm_0101.htm#3D%20Polygons
            for index in range(len(self._points)):
                p1 = self._points[index]
                p2 = self._points[(index + 1) % len(self._points)]
                value[0] += p1.y * p2.z - p1.z * p2.y
                value[1] += p1.z * p2.x - p1.x * p2.z
                value[2] += p1.x * p2.y - p1.y * p2.x
            result = self.plane.n.x * value[0] + self.plane.n.y * value[1] \
                    + self.plane.n.z * value[2]
            self._area_cache = result / 2
        return self._area_cache

    def get_length(self):
        """ add the length of all lines within the polygon
        """
        return sum(self.get_lengths())

    def get_middle_of_line(self, index):
        if (index >= len(self._points)) \
                or (not self._is_closed and index == len(self._points) - 1):
            return None
        else:
            return self._points[index].add(self._points[(index + 1) % len(self._points)]).div(2)

    def get_lengths(self):
        result = []
        for index in range(len(self._points) - 1):
            result.append(self._points[index + 1].sub(
                    self._points[index]).size())
        if self._is_closed:
            result.append(self._points[0].sub(self._points[-1]).size())
        return result

    def get_max_inside_distance(self):
        """ calculate the maximum distance between two points of the polygon
        """
        if len(self._points) < 2:
            return None
        distance = self._points[1].sub(self._points[0]).norm
        for p1 in self._points:
            for p2 in self._points:
                if p1 is p2:
                    continue
                distance = max(distance, p2.sub(p1).norm)
        return distance

    def is_outer(self):
        return self.get_area() > 0

    def is_polygon_inside(self, polygon):
        inside_counter = 0
        for point in polygon._points:
            if self.is_point_inside(point):
                inside_counter += 1
        return inside_counter == len(polygon._points)

    def is_point_on_outline(self, p):
        for line in self.get_lines():
            if line.is_point_inside(p):
                return True
        return False

    def is_point_inside(self, p):
        """ Test if a given point is inside of the polygon.
        The result is True if the point is on a line (or very close to it).
        """
        # First: check if the point is within the boundary of the polygon.
        if not p.is_inside(self.minx, self.maxx, self.miny, self.maxy,
                self.minz, self.maxz):
            # the point is outside the rectangle boundary
            return False
        # see http://www.alienryderflex.com/polygon/
        # Count the number of intersections of a ray along the x axis through
        # all polygon lines.
        # Odd number -> point is inside
        intersection_count_left = 0
        intersection_count_right = 0
        for index in range(len(self._points)):
            p1 = self._points[index]
            p2 = self._points[(index + 1) % len(self._points)]
            # Only count intersections with lines that are partly below
            # the y level of the point. This solves the problem of intersections
            # through shared vertices or lines that go along the y level of the
            # point.
            if ((p1.y < p.y) and (p.y <= p2.y)) \
                    or ((p2.y < p.y) and (p.y <= p1.y)):
                part_y = (p.y - p1.y) / (p2.y - p1.y)
                intersection_x = p1.x + part_y * (p2.x - p1.x)
                if intersection_x < p.x + epsilon:
                    # count intersections to the left
                    intersection_count_left += 1
                if intersection_x > p.x - epsilon:
                    # count intersections to the right
                    intersection_count_right += 1
        # odd intersection count -> inside
        left_odd = intersection_count_left % 2 == 1
        right_odd = intersection_count_right % 2 == 1
        if left_odd and right_odd:
            # clear decision: we are inside
            return True
        elif not left_odd and not right_odd:
            # clear decision: we are outside
            return False
        else:
            # it seems like we are on the line -> inside
            return True

    def get_lines(self):
        """ Caching is necessary to avoid constant recalculation due to
        the "to_OpenGL" method.
        """
        if (self._lines_cache is None) \
                or (len(self) != len(self._lines_cache)):
            # recalculate the line cache
            lines = []
            for index in range(len(self._points) - 1):
                lines.append(Line(self._points[index], self._points[index + 1]))
            # connect the last point with the first only if the polygon is closed
            if self._is_closed:
                lines.append(Line(self._points[-1], self._points[0]))
            self._lines_cache = lines
        return self._lines_cache[:]

    def to_OpenGL(self):
        for line in self.get_lines():
            line.to_OpenGL()
        return
        offset_polygons = self.get_offset_polygons(0.2)
        for polygon in offset_polygons:
            for line in polygon.get_lines():
                line.to_OpenGL()
        """
        for index, point in enumerate(self._points):
            line = Line(point, point.add(self.get_bisector(index)))
            line.get_length_line(1).to_OpenGL()
        """

    def _update_limits(self, point):
        if self.minx is None:
            self.minx = point.x
            self.maxx = point.x
            self.miny = point.y
            self.maxy = point.y
            self.minz = point.z
            self.maxz = point.z
        else:
            self.minx = min(self.minx, point.x)
            self.maxx = max(self.maxx, point.x)
            self.miny = min(self.miny, point.y)
            self.maxy = max(self.maxy, point.y)
            self.minz = min(self.minz, point.z)
            self.maxz = max(self.maxz, point.z)

    def reset_cache(self):
        self._lines_cache = None
        self._area_cache = None
        self.minx, self.miny, self.minz = None, None, None
        self.maxx, self.maxy, self.maxz = None, None, None
        # update the limit for each line
        for point in self._points:
            self._update_limits(point)

    def get_bisector(self, index):
        p1 = self._points[index - 1]
        p2 = self._points[index]
        p3 = self._points[(index + 1) % len(self._points)]
        return get_bisector(p1, p2, p3, self.plane.n)

    def get_offset_polygons(self, offset):
        def get_shifted_vertex(index, offset):
            p1 = self._points[index]
            p2 = self._points[(index + 1) % len(self._points)]
            cross_offset = p2.sub(p1).cross(self.plane.n).normalized()
            bisector_normalized = self.get_bisector(index)
            factor = cross_offset.dot(bisector_normalized)
            if factor != 0:
                bisector_sized = bisector_normalized.mul(offset / factor)
                return p1.add(bisector_sized)
            else:
                return p2
        def simplify_polygon_intersections(lines):
            new_group = lines[:]
            # remove all non-adjacent intersecting lines (this splits the group)
            if len(new_group) > 0:
                group_starts = []
                index1 = 0
                while index1 < len(new_group):
                    index2 = 0
                    while index2 < len(new_group):
                        index_distance = min(abs(index2 - index1), \
                                abs(len(new_group) - (index2 - index1))) 
                        # skip neighbours
                        if index_distance > 1:
                            line1 = new_group[index1]
                            line2 = new_group[index2]
                            intersection, factor = line1.get_intersection(line2)
                            if intersection and (intersection != line1.p1) \
                                    and (intersection != line1.p2):
                                del new_group[index1]
                                new_group.insert(index1,
                                        Line(line1.p1, intersection))
                                new_group.insert(index1 + 1,
                                        Line(intersection, line1.p2))
                                # Shift all items in "group_starts" by one if
                                # they reference a line whose index changed.
                                for i in range(len(group_starts)):
                                    if group_starts[i] > index1:
                                        group_starts[i] += 1
                                if not index1 + 1 in group_starts:
                                    group_starts.append(index1 + 1)
                                # don't update index2 -> maybe there are other hits
                            elif intersection and (intersection == line1.p1):
                                if not index1 in group_starts:
                                    group_starts.append(index1)
                                index2 += 1
                            else:
                                index2 += 1
                        else:
                            index2 += 1
                    index1 += 1
                # The lines intersect each other
                # We need to split the group.
                if len(group_starts) > 0:
                    group_starts.sort()
                    groups = []
                    last_start = 0
                    for group_start in group_starts:
                        groups.append(new_group[last_start:group_start])
                        last_start = group_start
                    # Add the remaining lines to the first group or as a new
                    # group.
                    if groups[0][0].p1 == new_group[-1].p2:
                        groups[0] = new_group[last_start:] + groups[0]
                    else:
                        groups.append(new_group[last_start:])
                    # try to find open groups that can be combined
                    combined_groups = []
                    for index, current_group in enumerate(groups):
                        # Check if the group is not closed: try to add it to
                        # other non-closed groups.
                        if current_group[0].p1 == current_group[-1].p2:
                            # a closed group
                            combined_groups.append(current_group)
                        else:
                            # the current group is open
                            for other_group in groups[index + 1:]:
                                if other_group[0].p1 != other_group[-1].p2:
                                    # This group is also open - a candidate
                                    # for merging?
                                    if other_group[0].p1 == current_group[-1].p2:
                                        current_group.reverse()
                                        for line in current_group:
                                            other_group.insert(0, line)
                                        break
                                    if other_group[-1].p2 == current_group[0].p1:
                                        other_group.extend(current_group)
                                        break
                            else:
                                # not suitable open group found
                                combined_groups.append(current_group)
                    return combined_groups
                else:
                    # just return one group without intersections
                    return [new_group]
            else:
                return None
        offset = number(offset)
        if offset == 0:
            return [self]
        if offset * 2 >= self.get_max_inside_distance():
            # This offset will not create a valid offset polygon.
            # Sadly there is currently no other way to detect a complete flip of
            # something like a circle.
            return []
        points = []
        for index in range(len(self._points)):
            points.append(get_shifted_vertex(index, offset))
        new_lines = []
        for index in range(len(points)):
            p1 = points[index]
            p2 = points[(index + 1) % len(points)]
            new_lines.append(Line(p1, p2))
        cleaned_line_groups = simplify_polygon_intersections(new_lines)
        if cleaned_line_groups is None:
            return None
        else:
            # remove all groups with a toggled direction
            self_is_outer = self.is_outer()
            groups = []
            for lines in cleaned_line_groups:
                group = Polygon(self.plane)
                for line in lines:
                    group.append(line)
                if group.is_outer() != self_is_outer:
                    # We ignore groups that changed the direction. These
                    # parts of the original group are flipped due to the
                    # offset.
                    continue
                # Remove polygons that should be inside the original,
                # but due to float inaccuracies they are not.
                if ((self.is_outer() and (offset < 0)) \
                        or (not self.is_outer() and (offset > 0))) \
                        and (not self.is_polygon_inside(group)):
                    continue
                groups.append(group)
            # remove all polygons that are within other polygons
            result = []
            for group in groups:
                inside = False
                for group_test in groups:
                    if group_test is group:
                        continue
                    if group_test.is_polygon_inside(group):
                        inside = True
                if not inside:
                    result.append(group)
            return result

    def get_offset_polygons_old(self, offset):
        def get_parallel_line(line, offset):
            if offset == 0:
                return Line(line.p1, line.p2)
            else:
                cross_offset = line.dir.cross(self.plane.n).normalized().mul(offset)
                # Prolong the line at the beginning and at the end - to allow
                # overlaps. Use factor "2" to take care for star-like structure
                # where a complete convex triangle would get cropped (two lines
                # get lost instead of just one). Use the "abs" value to
                # compensate negative offsets.
                in_line = line.dir.mul(2 * abs(offset))
                return Line(line.p1.add(cross_offset).sub(in_line),
                        line.p2.add(cross_offset).add(in_line))
        def do_lines_intersection(l1, l2):
            """ calculate the new intersection between two neighbouring lines
            """
            # TODO: use Line.get_intersection instead of the code below
            if l1.p2 == l2.p1:
                # intersection is already fine
                return
            if (l1.p1 is None) or (l2.p1 is None):
                # one line was already marked as obsolete
                return
            x1, x2, x3, x4 = l2.p1, l2.p2, l1.p1, l1.p2
            a = x2.sub(x1)
            b = x4.sub(x3)
            c = x3.sub(x1)
            # see http://mathworld.wolfram.com/Line-LineIntersection.html (24)
            try:
                factor = c.cross(b).dot(a.cross(b)) / a.cross(b).normsq
            except ZeroDivisionError:
                l2.p1 = None
                return
            if not (0 <= factor < 1):
                # The intersection is always supposed to be within p1 and p2.
                l2.p1 = None
            else:
                intersection = x1.add(a.mul(factor))
                if Line(l1.p1, intersection).dir != l1.dir:
                    # Remove lines that would change their direction due to the
                    # new intersection. These are usually lines that become
                    # obsolete due to a more favourable intersection of the two
                    # neighbouring lines. This appears at small corners.
                    l1.p1 = None
                elif Line(intersection, l2.p2).dir != l2.dir:
                    # see comment above
                    l2.p1 = None
                elif l1.p1 == intersection:
                    # remove invalid lines (zero length)
                    l1.p1 = None
                elif l2.p2 == intersection:
                    # remove invalid lines (zero length)
                    l2.p1 = None
                else:
                    # shorten both lines according to the new intersection
                    l1.p2 = intersection
                    l2.p1 = intersection
        def simplify_polygon_intersections(lines):
            finished = False
            new_group = lines[:]
            while not finished:
                if len(new_group) > 1:
                    # Calculate new intersections for each pair of adjacent
                    # lines.
                    for index in range(len(new_group)):
                        if (index == 0) and (not self._is_closed):
                            # skip the first line if the group is not closed
                            continue
                        # this also works for index==0 (closed groups)
                        l1 = new_group[index - 1]
                        l2 = new_group[index]
                        do_lines_intersection(l1, l2)
                # Remove all lines that were marked as obsolete during
                # intersection calculation.
                clean_group = [line for line in new_group
                        if not line.p1 is None]
                finished = len(new_group) == len(clean_group)
                if (len(clean_group) == 1) and self._is_closed:
                    new_group = []
                    finished = True
                else:
                    new_group = clean_group
            # remove all non-adjacent intersecting lines (this splits the group)
            if len(new_group) > 0:
                group_starts = []
                index1 = 0
                while index1 < len(new_group):
                    index2 = 0
                    while index2 < len(new_group):
                        index_distance = min(abs(index2 - index1), \
                                abs(len(new_group) - (index2 - index1))) 
                        # skip neighbours
                        if index_distance > 1:
                            line1 = new_group[index1]
                            line2 = new_group[index2]
                            intersection, factor = line1.get_intersection(line2)
                            if intersection and (intersection != line1.p1) \
                                    and (intersection != line1.p2):
                                del new_group[index1]
                                new_group.insert(index1,
                                        Line(line1.p1, intersection))
                                new_group.insert(index1 + 1,
                                        Line(intersection, line1.p2))
                                # Shift all items in "group_starts" by one if
                                # they reference a line whose index changed.
                                for i in range(len(group_starts)):
                                    if group_starts[i] > index1:
                                        group_starts[i] += 1
                                if not index1 + 1 in group_starts:
                                    group_starts.append(index1 + 1)
                                # don't update index2 -> maybe there are other hits
                            elif intersection and (intersection == line1.p1):
                                if not index1 in group_starts:
                                    group_starts.append(index1)
                                index2 += 1
                            else:
                                index2 += 1
                        else:
                            index2 += 1
                    index1 += 1
                # The lines intersect each other
                # We need to split the group.
                if len(group_starts) > 0:
                    group_starts.sort()
                    groups = []
                    last_start = 0
                    for group_start in group_starts:
                        groups.append(new_group[last_start:group_start])
                        last_start = group_start
                    # Add the remaining lines to the first group or as a new
                    # group.
                    if groups[0][0].p1 == new_group[-1].p2:
                        groups[0] = new_group[last_start:] + groups[0]
                    else:
                        groups.append(new_group[last_start:])
                    # try to find open groups that can be combined
                    combined_groups = []
                    for index, current_group in enumerate(groups):
                        # Check if the group is not closed: try to add it to
                        # other non-closed groups.
                        if current_group[0].p1 == current_group[-1].p2:
                            # a closed group
                            combined_groups.append(current_group)
                        else:
                            # the current group is open
                            for other_group in groups[index + 1:]:
                                if other_group[0].p1 != other_group[-1].p2:
                                    # This group is also open - a candidate
                                    # for merging?
                                    if other_group[0].p1 == current_group[-1].p2:
                                        current_group.reverse()
                                        for line in current_group:
                                            other_group.insert(0, line)
                                        break
                                    if other_group[-1].p2 == current_group[0].p1:
                                        other_group.extend(current_group)
                                        break
                            else:
                                # not suitable open group found
                                combined_groups.append(current_group)
                    return combined_groups
                else:
                    # just return one group without intersections
                    return [new_group]
            else:
                return None
        new_lines = []
        for line in self.get_lines():
            new_lines.append(get_parallel_line(line, offset))
        cleaned_line_groups = simplify_polygon_intersections(new_lines)
        if cleaned_line_groups is None:
            return None
        else:
            # remove all groups with a toggled direction
            self_is_outer = self.is_outer()
            groups = []
            for lines in cleaned_line_groups:
                group = Polygon(self.plane)
                for line in lines:
                    group.append(line)
                if group.is_outer() == self_is_outer:
                    # We ignore groups that changed the direction. These
                    # parts of the original group are flipped due to the
                    # offset.
                    groups.append(group)
            return groups

    def get_cropped_polygons(self, minx, maxx, miny, maxy, minz, maxz):
        """ crop a line group according to a 3d bounding box

        The result is a list of Polygons, since the bounding box can possibly
        break the original line group into several non-connected pieces.
        """
        new_groups = []
        for line in self.get_lines():
            new_line = None
            if line.is_completely_inside(minx, maxx, miny, maxy, minz, maxz):
                new_line = line
            else:
                cropped_line = line.get_cropped_line(minx, maxx, miny, maxy,
                        minz, maxz)
                if not cropped_line is None:
                    new_line = cropped_line
            # add the new line to one of the line groups
            if not new_line is None:
                # try to find a suitable line group
                for new_group in new_groups:
                    try:
                        new_group.append(new_line)
                        break
                    except ValueError:
                        # the line did not fit to this group (segment is broken)
                        pass
                else:
                    # no suitable group was found - we create a new one
                    new_group = Polygon(self.plane)
                    new_group.append(new_line)
                    new_groups.append(new_group)
        if len(new_groups) > 0:
            return new_groups
        else:
            return None

    def get_plane_projection(self, plane):
        if plane == self.plane:
            return self
        elif plane.n.dot(self.plane.n) == 0:
            log.warn("Polygon projection onto plane: orthogonal projection " \
                    + "is not possible")
            return None
        else:
            result = Polygon(plane)
            for line in self.get_lines():
                p1 = plane.get_point_projection(line.p1)
                p2 = plane.get_point_projection(line.p2)
                result.append(Line(p1, p2))
            # check if the projection would revert the direction of the polygon
            if plane.n.dot(self.plane.n) < 0:
                result.reverse_direction()
            return result

    def is_overlap(self, other):
        for line1 in self.get_lines():
            for line2 in other.get_lines():
                cp, dist = line1.get_intersection(line2)
                if not cp is None:
                    return True
        return False

    def union(self, other):
        """ This "union" of two polygons only works for polygons without
        shared edges. TODO: fix the issues of shared edges!
        """
        # don't import earlier to avoid circular imports
        from pycam.Geometry.Model import ContourModel
        # check if one of the polygons is completely inside of the other
        if self.is_polygon_inside(other):
            return [self]
        if other.is_polygon_inside(self):
            return [other]
        # check if there is any overlap at all
        if not self.is_overlap(other):
            # no changes
            return [self, other]
        contour = ContourModel(self.plane)
        def get_outside_lines(poly1, poly2):
            result = []
            for line in poly1.get_lines():
                collisions = []
                for o_line in poly2.get_lines():
                    cp, dist = o_line.get_intersection(line)
                    if (not cp is None) and (0 < dist < 1):
                        collisions.append((cp, dist))
                # sort the collisions according to the distance
                collisions.append((line.p1, 0))
                collisions.append((line.p2, 1))
                collisions.sort(key=lambda (cp, dist): dist)
                for index in range(len(collisions) - 1):
                    p1 = collisions[index][0]
                    p2 = collisions[index + 1][0]
                    if p1.sub(p2).norm < epsilon:
                        # ignore zero-length lines
                        continue
                    # Use the middle between p1 and p2 to check the
                    # inner/outer state.
                    p_middle = p1.add(p2).div(2)
                    p_inside = poly2.is_point_inside(p_middle) \
                            and not poly2.is_point_on_outline(p_middle)
                    if not p_inside:
                        result.append(Line(p1, p2))
            return result
        outside_lines = []
        outside_lines.extend(get_outside_lines(self, other))
        outside_lines.extend(get_outside_lines(other, self))
        for line in outside_lines:
            contour.append(line)
        # fix potential overlapping at the beginning and end of each polygon
        result = []
        for poly in contour.get_polygons():
            if not poly._is_closed:
                lines = poly.get_lines()
                line1 = lines[-1]
                line2 = lines[0]
                if (line1.dir == line2.dir) \
                        and (line1.is_point_inside(line2.p1)):
                    # remove the last point and define the polygon as closed
                    poly._points.pop(-1)
                    poly._is_closed = True
            result.append(poly)
        return result

