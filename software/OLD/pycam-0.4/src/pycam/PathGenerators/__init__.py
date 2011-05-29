# -*- coding: utf-8 -*-
"""
$Id: __init__.py 770 2010-10-14 10:36:24Z sumpfralle $

Copyright 2010 Lars Kruse <devel@sumpfralle.de>
Copyright 2008 Lode Leroy

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

__all__ = ["DropCutter", "PushCutter", "EngraveCutter", "ContourFollow"]

from pycam.Geometry.utils import INFINITE, epsilon, sqrt
from pycam.Geometry.Point import Point
import pycam.Utils.threading


class Hit:
    def __init__(self, cl, cp, t, d, direction):
        self.cl = cl
        self.cp = cp
        self.t = t
        self.d = d
        self.dir = direction
        self.z = -INFINITE

    def __repr__(self):
        return "%s - %s - %s - %s" % (self.d, self.cl, self.dir, self.cp)


def get_free_paths_triangles(models, cutter, p1, p2, return_triangles=False):
    if len(models) == 1:
        # only one model is left - just continue
        model = models[0]
    else:
        # multiple models were given - process them in layers
        result = get_free_paths_triangles(models[:1], cutter, p1, p2,
                return_triangles)
        # group the result into pairs of two points (start/end)
        point_pairs = []
        while result:
            pair1 = result.pop(0)
            pair2 = result.pop(0)
            point_pairs.append((pair1, pair2))
        all_results = []
        for pair in point_pairs:
            one_result = get_free_paths_triangles(models[1:], cutter, pair[0],
                    pair[1], return_triangles)
            all_results.extend(one_result)
        return all_results

    backward = p1.sub(p2).normalized()
    forward = p2.sub(p1).normalized()
    xyz_dist = p2.sub(p1).norm

    minx = min(p1.x, p2.x)
    maxx = max(p1.x, p2.x)
    miny = min(p1.y, p2.y)
    maxy = max(p1.y, p2.y)
    minz = min(p1.z, p2.z)

    # find all hits along scan line
    hits = []

    triangles = model.triangles(minx - cutter.distance_radius,
            miny - cutter.distance_radius, minz, maxx + cutter.distance_radius,
            maxy + cutter.distance_radius, INFINITE)

    for t in triangles:
        (cl1, d1, cp1) = cutter.intersect(backward, t, start=p1)
        if cl1:
            hits.append(Hit(cl1, cp1, t, -d1, backward))
        (cl2, d2, cp2) = cutter.intersect(forward, t, start=p1)
        if cl2:
            hits.append(Hit(cl2, cp2, t, d2, forward))

    # sort along the scan direction
    hits.sort(key=lambda h: h.d)

    count = 0
    points = []
    for h in hits:
        if h.dir == forward:
            if count == 0:
                if -epsilon <= h.d <= xyz_dist + epsilon:
                    if len(points) == 0:
                        points.append((p1, None, None))
                    points.append((h.cl, h.t, h.cp))
            count += 1
        else:
            if count == 1:
                if -epsilon <= h.d <= xyz_dist + epsilon:
                    points.append((h.cl, h.t, h.cp))
            count -= 1

    if len(points) % 2 == 1:
        points.append((p2, None, None))

    if len(points) == 0:
        # check if the path is completely free or if we are inside of the model
        inside_counter = 0
        for h in hits:
            if -epsilon <= h.d:
                # we reached the outer limit of the model
                break
            if h.dir == forward:
                inside_counter += 1
            else:
                inside_counter -= 1
        if inside_counter <= 0:
            # we are not inside of the model
            points.append((p1, None, None))
            points.append((p2, None, None))

    if return_triangles:
        return points
    else:
        # return only the cutter locations (without triangles)
        return [cl for (cl, t, cp) in points]


def get_free_paths_ode(physics, p1, p2, depth=8):
    """ Recursive function for splitting a line (usually along x or y) into
    small pieces to gather connected paths for the PushCutter.
    Strategy: check if the whole line is free (without collisions). Do a
    recursive call (for the first and second half), if there was a
    collision.

    Usually either minx/maxx or miny/maxy should be equal, unless you want
    to do a diagonal cut.
    @param minx: lower limit of x
    @type minx: float
    @param maxx: upper limit of x; should equal minx for a cut along the x axis
    @type maxx: float
    @param miny: lower limit of y
    @type miny: float
    @param maxy: upper limit of y; should equal miny for a cut along the y axis
    @type maxy: float
    @param z: the fixed z level
    @type z: float
    @param depth: number of splits to be calculated via recursive calls; the
        accuracy can be calculated as (maxx-minx)/(2^depth)
    @type depth: int
    @returns: a list of points that describe the tool path of the PushCutter;
        each pair of points defines a collision-free path
    @rtype: list(pycam.Geometry.Point.Point)
    """
    points = []
    # "resize" the drill along the while x/y range and check for a collision
    physics.extend_drill(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)
    physics.set_drill_position((p1.x, p1.y, p1.z))
    if physics.check_collision():
        # collision detected
        if depth > 0:
            middle_x = (p1.x + p2.x) / 2
            middle_y = (p1.y + p2.y) / 2
            middle_z = (p1.z + p2.z) / 2
            p_middle = Point(middle_x, middle_y, middle_z)
            group1 = get_free_paths_ode(physics, p1, p_middle, depth - 1)
            group2 = get_free_paths_ode(physics, p_middle, p2, depth - 1)
            if group1 and group2 and (group1[-1] == group2[0]):
                # The last pair of the first group ends where the first pair of
                # the second group starts.
                # We will combine them into a single pair.
                points.extend(group1[:-1])
                points.extend(group2[1:])
            else:
                # the two groups are not connected - just add both
                points.extend(group1)
                points.extend(group2)
        else:
            # no points to be added
            pass
    else:
        # no collision - the line is free
        points.append(p1)
        points.append(p2)
    physics.reset_drill()
    return points

def get_max_height_ode(physics, x, y, minz, maxz):
    low, high = minz, maxz
    trip_start = 20
    safe_z = None
    # check if the full step-down would be ok
    physics.set_drill_position((x, y, minz))
    if physics.check_collision():
        # there is an object between z1 and z0 - we need more=None loops
        trips = trip_start
    else:
        # No need for further collision detection - we can go down the whole
        # range z1..z0.
        trips = 0
        safe_z = minz
    while trips > 0:
        current_z = (low + high) / 2
        physics.set_drill_position((x, y, current_z))
        if physics.check_collision():
            low = current_z
        else:
            high = current_z
            safe_z = current_z
        trips -= 1
    if safe_z is None:
        # no safe position was found - let's check the upper bound
        physics.set_drill_position((x, y, maxz))
        if physics.check_collision():
            # the object fills the whole range of z0..z1 -> no safe height
            pass
        else:
            # at least the upper bound is collision free
            safe_z = maxz
    if safe_z is None:
        return []
    else:
        return [Point(x, y, safe_z)]

def get_max_height_triangles(model, cutter, x, y, minz, maxz, last_pos=None):
    result = []
    if last_pos is None:
        last_pos = {}
    for key in ("triangle", "cut"):
        if not key in last_pos:
            last_pos[key] = None
    p = Point(x, y, maxz)
    height_max = None
    cut_max = None
    triangle_max = None
    box_x_min = cutter.get_minx(p)
    box_x_max = cutter.get_maxx(p)
    box_y_min = cutter.get_miny(p)
    box_y_max = cutter.get_maxy(p)
    box_z_min = minz
    box_z_max = maxz
    triangles = model.triangles(box_x_min, box_y_min, box_z_min, box_x_max,
            box_y_max, box_z_max)
    for t in triangles:
        cut = cutter.drop(t, start=p)
        if cut and ((height_max is None) or (cut.z > height_max)):
            height_max = cut.z
            cut_max = cut
            triangle_max = t
    # don't do a complete boundary check for the height
    # this avoids zero-cuts for models that exceed the bounding box height
    if not cut_max or cut_max.z < minz + epsilon:
        cut_max = Point(x, y, minz)
    if last_pos["cut"] and \
            ((triangle_max and not last_pos["triangle"]) \
            or (last_pos["triangle"] and not triangle_max)):
        if minz - epsilon <= last_pos["cut"].z <= maxz + epsilon:
            result.append(Point(last_pos["cut"].x, last_pos["cut"].y,
                    cut_max.z))
        else:
            result.append(Point(cut_max.x, cut_max.y, last_pos["cut"].z))
    elif (triangle_max and last_pos["triangle"] and last_pos["cut"] and \
            cut_max) and (triangle_max != last_pos["triangle"]):
        # TODO: check if this path is ever in use (e.g. "intersect_lines" is not
        # defined)
        nl = range(3)
        nl[0] = -getattr(last_pos["triangle"].normal, order[0])
        nl[2] = last_pos["triangle"].normal.z
        nm = range(3)
        nm[0] = -getattr(triangle_max.normal, order[0])
        nm[2] = triangle_max.normal.z
        last = range(3)
        last[0] = getattr(last_pos["cut"], order[0])
        last[2] = last_pos["cut"].z
        mx = range(3)
        mx[0] = getattr(cut_max, order[0])
        mx[2] = cut_max.z
        c = range(3)
        (c[0], c[2]) = intersect_lines(last[0], last[2], nl[0], nl[2], mx[0],
                mx[2], nm[0], nm[2])
        if c[0] and last[0] < c[0] < mx[0] and (c[2] > last[2] or c[2] > mx[2]):
            c[1] = getattr(last_pos["cut"], order[1])
            if (c[2] < minz - 10) or (c[2] > maxz + 10):
                print "^", "%sl=%s" % (order[0], last[0]), \
                        ", %sl=%s" % ("z", last[2]), \
                        ", n%sl=%s" % (order[0], nl[0]), \
                        ", n%sl=%s" % ("z", nl[2]), \
                        ", %s=%s" % (order[0].upper(), c[0]), \
                        ", %s=%s" % ("z".upper(), c[2]), \
                        ", %sm=%s" % (order[0], mx[0]), \
                        ", %sm=%s" % ("z", mx[2]), \
                        ", n%sm=%s" % (order[0], nm[0]), \
                        ", n%sm=%s" % ("z", nm[2])

            else:
                if order[0] == "x":
                    result.append(Point(c[0], c[1], c[2]))
                else:
                    result.append(Point(c[1], c[0], c[2]))
    result.append(cut_max)

    last_pos["cut"] = cut_max
    last_pos["triangle"] = triangle_max
    return result

