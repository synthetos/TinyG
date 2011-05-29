# -*- coding: utf-8 -*-
"""
$Id: SupportGrid.py 758 2010-10-12 17:00:37Z sumpfralle $

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

from pycam.Geometry.Point import Point, Vector
from pycam.Geometry.Line import Line
from pycam.Geometry.Triangle import Triangle
from pycam.Geometry.Plane import Plane
from pycam.Geometry.Model import Model
from pycam.Geometry.utils import number


def _get_triangles_for_face(pts):
    t1 = Triangle(pts[0], pts[1], pts[2])
    t2 = Triangle(pts[2], pts[3], pts[0])
    return (t1, t2)

def _add_cuboid_to_model(model, start, direction, height, width):
    up = Vector(0, 0, 1).mul(height)
    ortho_dir = direction.cross(up).normalized()
    start1 = start.add(ortho_dir.mul(-width/2))
    start2 = start1.add(up)
    start3 = start2.add(ortho_dir.mul(width))
    start4 = start3.sub(up)
    end1 = start1.add(direction)
    end2 = start2.add(direction)
    end3 = start3.add(direction)
    end4 = start4.add(direction)
    faces = ((start1, start2, start3, start4), (start1, end1, end2, start2),
            (start2, end2, end3, start3), (start3, end3, end4, start4),
            (start4, end4, end1, start1), (end4, end3, end2, end1))
    for face in faces:
        t1, t2 = _get_triangles_for_face(face)
        model.append(t1)
        model.append(t2)

def _add_aligned_cuboid_to_model(minx, maxx, miny, maxy, minz, maxz):
    points = (
            Point(minx, miny, minz),
            Point(maxx, miny, minz),
            Point(maxx, maxy, minz),
            Point(minx, maxy, minz),
            Point(minx, miny, maxz),
            Point(maxx, miny, maxz),
            Point(maxx, maxy, maxz),
            Point(minx, maxy, maxz))
    triangles = []
    # lower face
    triangles.extend(_get_triangles_for_face(
            (points[0], points[1], points[2], points[3])))
    # upper face
    triangles.extend(_get_triangles_for_face(
            (points[7], points[6], points[5], points[4])))
    # front face
    triangles.extend(_get_triangles_for_face(
            (points[0], points[4], points[5], points[1])))
    # back face
    triangles.extend(_get_triangles_for_face(
            (points[2], points[6], points[7], points[3])))
    # right face
    triangles.extend(_get_triangles_for_face(
            (points[1], points[5], points[6], points[2])))
    # left face
    triangles.extend(_get_triangles_for_face(
            (points[3], points[7], points[4], points[0])))
    # add all triangles to the model
    model = Model()
    for t in triangles:
        model.append(t)
    return model

def get_support_grid_locations(minx, maxx, miny, maxy, dist_x, dist_y,
        offset_x=0.0, offset_y=0.0, adjustments_x=None, adjustments_y=None):
    def get_lines(center, dist, min_value, max_value):
        """ generate a list of positions starting from the middle going up and
        and down
        """
        if dist > 0:
            lines = [center]
            current = center
            while current - dist > min_value:
                current -= dist
                lines.insert(0, current)
            current = center
            while current + dist < max_value:
                current += dist
                lines.append(current)
        else:
            lines = []
        # remove lines that are out of range (e.g. due to a huge offset)
        lines = [line for line in lines if min_value < line < max_value]
        return lines
    # convert all inputs to the type defined in "number"
    dist_x = number(dist_x)
    dist_y = number(dist_y)
    offset_x = number(offset_x)
    offset_y = number(offset_y)
    center_x = (maxx + minx) / 2 + offset_x
    center_y = (maxy + miny) / 2 + offset_y
    lines_x = get_lines(center_x, dist_x, minx, maxx)
    lines_y = get_lines(center_y, dist_y, miny, maxy)
    if adjustments_x:
        for index in range(min(len(lines_x), len(adjustments_x))):
            lines_x[index] += number(adjustments_x[index])
    if adjustments_y:
        for index in range(min(len(lines_y), len(adjustments_y))):
            lines_y[index] += number(adjustments_y[index])
    return lines_x, lines_y

def get_support_grid(minx, maxx, miny, maxy, z_plane, dist_x, dist_y, thickness,
        height, offset_x=0.0, offset_y=0.0, adjustments_x=None,
        adjustments_y=None):
    lines_x, lines_y = get_support_grid_locations(minx, maxx, miny, maxy, dist_x,
            dist_y, offset_x, offset_y, adjustments_x, adjustments_y)
    # create all x grid lines
    grid_model = Model()
    # convert all inputs to "number"
    thickness = number(thickness)
    height = number(height)
    # helper variables
    thick_half = thickness / 2
    length_extension = max(thickness, height)
    for line_x in lines_x:
        # we make the grid slightly longer (by thickness) than necessary
        grid_model += _add_aligned_cuboid_to_model(line_x - thick_half,
                line_x + thick_half, miny - length_extension,
                maxy + length_extension, z_plane, z_plane + height)
    for line_y in lines_y:
        # we make the grid slightly longer (by thickness) than necessary
        grid_model += _add_aligned_cuboid_to_model(minx - length_extension,
                maxx + length_extension, line_y - thick_half,
                line_y + thick_half, z_plane, z_plane + height)
    return grid_model

def get_support_distributed(model, z_plane, average_distance,
        min_bridges_per_polygon, thickness, height, length):
    def is_near_list(point_list, point, distance):
        for p in point_list:
            if p.sub(point).norm <= distance:
                return True
        return False
    if (average_distance == 0) or (length == 0) or (thickness == 0) \
            or (height == 0):
        return
    result = Model()
    if hasattr(model, "get_polygons"):
        polygons = model.get_polygons()
    else:
        # TODO: Solid models are not supported, yet - we need to get the
        # maximum outline of the model.
        polygons = model.get_waterline_polygons(Plane(Point(0, 0, z_plane),
                Vector(0, 0, 1)))
    bridge_positions = []
    # minimum required distance between two bridge start points
    avoid_distance = 1.5 * (abs(length) + thickness)
    for polygon in polygons:
        # no grid for _small_ inner polygons
        # TODO: calculate a reasonable factor (see below)
        if not polygon.is_outer() \
                and (abs(polygon.get_area()) < 25000 * thickness ** 2):
            continue
        lines = polygon.get_lines()
        poly_lengths = polygon.get_lengths()
        num_of_bridges = max(min_bridges_per_polygon,
                int(round(sum(poly_lengths) / average_distance)))
        real_average_distance = sum(poly_lengths) / num_of_bridges
        max_line_index = poly_lengths.index(max(poly_lengths))
        positions = []
        current_line_index = max_line_index
        distance_processed = poly_lengths[current_line_index] / 2
        positions.append(current_line_index)
        while len(positions) < num_of_bridges:
            current_line_index += 1
            current_line_index %= len(poly_lengths)
            # skip lines that are not at least twice as long as the grid width
            while (distance_processed + poly_lengths[current_line_index] \
                    < real_average_distance):
                distance_processed += poly_lengths[current_line_index]
                current_line_index += 1
                current_line_index %= len(poly_lengths)
            positions.append(current_line_index)
            distance_processed += poly_lengths[current_line_index]
            distance_processed %= real_average_distance
        for line_index in positions:
            position = polygon.get_middle_of_line(line_index)
            # skip bridges that are close to another existing bridge
            if is_near_list(bridge_positions, position, avoid_distance):
                line = polygon.get_lines()[line_index]
                # calculate two alternative points on the same line
                position1 = position.add(line.p1).div(2)
                position2 = position.add(line.p2).div(2)
                if is_near_list(bridge_positions, position1, avoid_distance):
                    if is_near_list(bridge_positions, position2,
                            avoid_distance):
                        # no valid alternative - we skip this bridge
                        continue
                    else:
                        # position2 is OK
                        position = position2
                else:
                    # position1 is OK
                    position = position1
            # append the original position (ignoring z_plane)
            bridge_positions.append(position)
            # move the point to z_plane
            position = Point(position.x, position.y, z_plane)
            bridge_dir = lines[line_index].dir.cross(
                    polygon.plane.n).normalized().mul(length)
            _add_cuboid_to_model(result, position, bridge_dir, height, thickness)
    return result

