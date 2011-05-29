# -*- coding: utf-8 -*-
"""
$Id: ODEBlocks.py 756 2010-10-12 16:56:59Z sumpfralle $

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

import pycam.Cutters
from pycam.Geometry.Point import Point
import ode

try:
    import OpenGL.GL as GL
    GL_enabled = True
except ImportError:
    GL_enabled = False


class ODEBlocks:

    def __init__(self, tool_settings, (minx, maxx, miny, maxy, minz, maxz),
            x_steps=None, y_steps=None):
        self.cutter = pycam.Cutters.get_tool_from_settings(tool_settings)
        # we don't want to use the "material allowance" distance
        self.cutter.set_required_distance(0)
        if x_steps is None:
            x_steps = 10
        if y_steps is None:
            y_steps = 10
        self.x_steps = x_steps
        self.y_steps = y_steps
        self.x_offset = minx
        self.y_offset = miny
        self.z_offset = minz
        self.x_width = maxx - minx
        self.y_width = maxy - miny
        self.z_width = maxz - minz
        if self.z_width <= 0:
            # Use a default height of "1" if the model is flat.
            # Half of the height is above and half below the plane.
            self.z_width = 1.0
            self.z_offset = minz - self.z_width / 2.0
        self.world = ode.World()
        self.space = ode.Space()
        self.boxes = []
        z_pos = self.z_offset + 0.5 * self.z_width
        self.x_step_width = self.x_width / (self.x_steps - 1)
        self.y_step_width = self.y_width / (self.y_steps - 1)
        for x_pos in [self.x_offset + (index + 0.5) / x_steps * self.x_width \
                for index in range(x_steps)]:
            for y_pos in [self.y_offset + (index + 0.5) / y_steps \
                    * self.y_width for index in range(y_steps)]:
                body = ode.Body(self.world)
                box = ode.GeomBox(self.space, (self.x_step_width,
                        self.y_step_width, self.z_width))
                box.setBody(body)
                box.setPosition((x_pos, y_pos, z_pos))
                box.position = Point(x_pos, y_pos, z_pos)
                self.boxes.append(box)

    def process_cutter_movement(self, location_start, location_end):
        # TODO: fix this workaround in the cutters shape defintions (or in ODE?)
        # for now we may only move from low x/y values to higher x/y values
        if (location_start.x > location_end.x) \
                or (location_start.y > location_end.y):
            swap = location_start
            location_start = location_end
            location_end = swap
        cutter_body = ode.Body(self.world)
        cutter_shape, cutter_position_func = self.cutter.get_shape("ODE")
        self.space.add(cutter_shape)
        cutter_shape.space = self.space
        cutter_shape.setBody(cutter_body)
        cutter_position_func(location_start.x, location_start.y,
                location_start.z)
        cutter_shape.extend_shape(location_end.x - location_start.x,
                location_end.y - location_start.y,
                location_end.z - location_start.z)
        aabb = cutter_shape.getAABB()
        cutter_height = aabb[5] - aabb[4]
        # add a ray along the drill to work around an ODE bug in v0.11.1
        # http://sourceforge.net/tracker/index.php?func=detail&aid=2973876&group_id=24884&atid=382799
        currx, curry, currz = cutter_shape.getPosition()
        ray = ode.GeomRay(self.space, cutter_height)
        ray.set((currx, curry, aabb[5]), (0.0, 0.0, -1.0))
        # check for collisions
        all_cutter_shapes = [cutter_shape] + cutter_shape.children + [ray]
        def check_collision(item):
            for one_cutter_shape in all_cutter_shapes:
                if ode.collide(item, one_cutter_shape):
                    return True
            return False
        for index in range(len(self.boxes)):
            if check_collision(self.boxes[index]):
                self.shrink_box_avoiding_collision(index, check_collision)

    def shrink_box_avoiding_collision(self, box_index, collision_func):
        box = self.boxes[box_index]
        aabb = box.getAABB()
        end_height, start_height = aabb[-2:]
        height_half = (start_height - end_height) / 2.0
        x_pos = box.position.x
        y_pos = box.position.y
        new_z = end_height
        box.setPosition((x_pos, y_pos, end_height - height_half))
        loops_left = 12
        upper_limit = start_height
        lower_limit = end_height
        if collision_func(box):
            # the cutter goes down to zero (end_height) - we can skip the rest
            loops_left = 0
        while loops_left > 0:
            new_z = (upper_limit + lower_limit) / 2.0
            box.setPosition((x_pos, y_pos, new_z - height_half))
            if collision_func(box):
                upper_limit = new_z
            else:
                lower_limit = new_z
            loops_left -= 1
        del self.boxes[box_index]
        # The height should never be zero - otherwise ODE will throw a
        # "bNormalizationResult" assertion.
        new_height = max(new_z - end_height, 0.1)
        z_pos = new_z - new_height / 2.0
        new_box = ode.GeomBox(self.space, (aabb[1] - aabb[0], aabb[3] - aabb[2],
                new_height))
        new_box.position = Point(x_pos, y_pos, z_pos)
        new_box.setBody(box.getBody())
        new_box.setPosition((x_pos, y_pos, z_pos))
        self.boxes.insert(box_index, new_box)

    def get_height_field(self):
        """ returns a two-dimensional height field of the current "landscape"
        """
        result = []
        heights = []
        for box in self.boxes:
            aabb = box.getAABB()
            heights.append(Point((aabb[1] + aabb[0]) / 2.0,
                    (aabb[3] + aabb[2]) / 2.0, aabb[5]))
        for column in range(self.x_steps):
            result.append(heights[
                    column * self.y_steps : (column + 1) * self.y_steps])
        return result

    def _normal(self, z0, z1, z2):
        nx = self.x_steps / self.x_width
        ny = self.y_steps / self.y_width
        nz = 1.0 / (self.z_width)
        return (-ny * (z1 - z0) * nz / nx, -nx * (z2 - z1) * nz / ny,
                nx * ny / nz * 100)
        
    def to_OpenGL(self):
        if not GL_enabled:
            return
        height_field = self.get_height_field()
        def get_box_height_points(x, y):
            """ Get the positions and heights of the the four top corners of a
            height box.

            The result is a tuple of four Points (pycam.Geometry.Point) in the
            following order:
             - left below
             - right below
             - right above
             - left above
            ("above": greater x value; "right": greater y value)
            The height of each corner point is calculated as the average of the
            four neighbouring boxes. Thus a set of 3x3 adjacent boxes is used
            for calculating the heights of the four corners.
            """
            points = []
            # Go through a set of box index combinations (sharing a common
            # corner). The "offsets" tuple is used for indicating the relative
            # position of each corner.
            for offsets, index_list in (
                ((-1, -1), ((x - 1, y - 1), (x, y - 1), (x, y), (x - 1, y))),
                ((+1, -1), ((x, y - 1), (x, y), (x + 1, y), (x + 1, y - 1))),
                ((+1, +1), ((x, y), (x + 1, y), (x + 1, y + 1), (x, y + 1))),
                ((-1, +1), ((x - 1, y), (x, y), (x, y + 1), (x - 1, y + 1)))):
                divisor = 0
                height_sum = 0
                x_positions = []
                y_positions = []
                for ix, iy in index_list:
                    if (0 <= ix < len(height_field)) \
                            and (0 <= iy < len(height_field[ix])):
                        point = height_field[ix][iy]
                        height_sum += point.z
                        x_positions.append(point.x)
                        y_positions.append(point.y)
                        divisor += 1
                # Use the middle between the x positions of two adjacent boxes,
                # _if_ there is a neighbour attached to that corner.
                if (min(x_positions) < height_field[x][y].x) \
                        or (max(x_positions) > height_field[x][y].x):
                    x_value = (min(x_positions) + max(x_positions)) / 2.0
                else:
                    # There is no adjacent box in x direction. Use the step size
                    # to calculate the x value of this edge.
                    x_value = height_field[x][y].x \
                            + offsets[0] * self.x_step_width / 2.0
                # same as above for y instead of x
                if (min(y_positions) < height_field[x][y].y) \
                        or (max(y_positions) > height_field[x][y].y):
                    y_value = (min(y_positions) + max(y_positions)) / 2.0
                else:
                    y_value = height_field[x][y].y \
                            + offsets[1] * self.y_step_width / 2.0
                # Create a Point instance describing the position and the
                # average height.
                points.append(Point(x_value, y_value, height_sum / divisor))
            return points
        # draw the surface
        GL.glBegin(GL.GL_QUADS)
        for x in xrange(self.x_steps):
            for y in xrange(self.y_steps):
                # Get the positions and heights of the four corners surrounding
                # the current box.
                points_around = get_box_height_points(x, y)
                # Calculate the "normal" of polygon. We picked up three random
                # points of this quadrilateral.
                n = self._normal(points_around[1].z, points_around[2].z,
                        points_around[3].z)
                GL.glNormal3f(n[0], n[1], n[2])
                for point in points_around:
                    GL.glVertex3f(point.x, point.y, point.z)
                # go through the conditions for an edge box and use the
                # appropriate corners for the side faces of the material
                for condition, i1, i2 in ((x == 0, 3, 0), (y == 0, 0, 1),
                        (x == self.x_steps - 1, 1, 2),
                        (y == self.y_steps - 1, 2, 3)):
                    # check if this point belongs to an edge of the material
                    if condition:
                        n = self._normal(points_around[1].z, points_around[2].z,
                                points_around[3].z)
                        GL.glNormal3f(n[0], n[1], n[2])
                        GL.glVertex3f(points_around[i1].x, points_around[i1].y,
                                self.z_offset)
                        GL.glVertex3f(points_around[i1].x, points_around[i1].y,
                                points_around[i1].z)
                        GL.glVertex3f(points_around[i2].x, points_around[i2].y,
                                points_around[i2].z)
                        GL.glVertex3f(points_around[i2].x, points_around[i2].y,
                                self.z_offset)
        GL.glEnd()

