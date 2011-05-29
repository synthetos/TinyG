# -*- coding: utf-8 -*-
"""
$Id: Model.py 758 2010-10-12 17:00:37Z sumpfralle $

Copyright 2008-2010 Lode Leroy
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

import pycam.Exporters.STLExporter
from pycam.Geometry.Triangle import Triangle
from pycam.Geometry.Line import Line
from pycam.Geometry.Plane import Plane
from pycam.Geometry.Polygon import Polygon
from pycam.Geometry.Point import Point, Vector
from pycam.Geometry.TriangleKdtree import TriangleKdtree
from pycam.Geometry.Matrix import TRANSFORMATIONS
from pycam.Toolpath import Bounds
from pycam.Geometry.utils import INFINITE
from pycam.Geometry import TransformableContainer
from pycam.Utils import ProgressCounter
import pycam.Utils.log
import uuid

log = pycam.Utils.log.get_logger()


class BaseModel(TransformableContainer):
    id = 0

    def __init__(self):
        self.id = BaseModel.id
        BaseModel.id += 1
        self._item_groups = []
        self.name = "model%d" % self.id
        self.minx = None
        self.miny = None
        self.minz = None
        self.maxx = None
        self.maxy = None
        self.maxz = None
        # derived classes should override this
        self._export_function = None

    def __add__(self, other_model):
        """ combine two models """
        result = self.__class__()
        for item in self.next():
            result.append(item)
        for item in other_model.next():
            result.append(item)
        return result

    def next(self):
        for item_group in self._item_groups:
            for item in item_group:
                if isinstance(item, list):
                    for subitem in item:
                        yield subitem
                else:
                    yield item

    def get_children_count(self):
        result = 0
        for item_group in self._item_groups:
            for item in item_group:
                result += 1
                if hasattr(item, "get_children_count"):
                    result += item.get_children_count()
        return result

    def to_OpenGL(self):
        for item in self.next():
            # ignore invisble things like the normal of a ContourModel
            if hasattr(item, "to_OpenGL"):
                item.to_OpenGL()

    def is_export_supported(self):
        return not self._export_function is None

    def export(self, comment=None):
        if self.is_export_supported():
            return self._export_function(self, comment=comment)
        else:
            raise NotImplementedError(("This type of model (%s) does not " \
                    + "support the 'export' function.") % str(type(self)))

    def _update_limits(self, item):
        # ignore items without limit attributes (e.g. the normal of a ContourModel)
        if hasattr(item, "minx"):
            if self.minx is None:
                self.minx = item.minx
                self.miny = item.miny
                self.minz = item.minz
                self.maxx = item.maxx
                self.maxy = item.maxy
                self.maxz = item.maxz
            else:
                self.minx = min(self.minx, item.minx)
                self.miny = min(self.miny, item.miny)
                self.minz = min(self.minz, item.minz)
                self.maxx = max(self.maxx, item.maxx)
                self.maxy = max(self.maxy, item.maxy)
                self.maxz = max(self.maxz, item.maxz)

    def append(self, item):
        self._update_limits(item)

    def maxsize(self):
        return max(abs(self.maxx), abs(self.minx), abs(self.maxy),
                abs(self.miny), abs(self.maxz), abs(self.minz))

    def subdivide(self, depth):
        model = self.__class__()
        for item in self.next():
            for s in item.subdivide(depth):
                model.append(s)
        return model

    def reset_cache(self):
        self.minx = None
        self.miny = None
        self.minz = None
        self.maxx = None
        self.maxy = None
        self.maxz = None
        for item in self.next():
            self._update_limits(item)

    def _get_progress_callback(self, update_callback):
        if update_callback:
            return ProgressCounter(self.get_children_count(),
                    update_callback=update_callback).increment
        else:
            return None

    def transform_by_template(self, direction="normal", callback=None):
        if direction in TRANSFORMATIONS.keys():
            self.transform_by_matrix(TRANSFORMATIONS[direction],
                    callback=self._get_progress_callback(callback))

    def shift(self, shift_x, shift_y, shift_z, callback=None):
        matrix = ((1, 0, 0, shift_x), (0, 1, 0, shift_y), (0, 0, 1, shift_z))
        self.transform_by_matrix(matrix,
                callback=self._get_progress_callback(callback))
        
    def scale(self, scale_x, scale_y=None, scale_z=None, callback=None):
        if scale_y is None:
            scale_y = scale_x
        if scale_z is None:
            scale_z = scale_x
        matrix = ((scale_x, 0, 0, 0), (0, scale_y, 0, 0), (0, 0, scale_z, 0))
        self.transform_by_matrix(matrix,
                callback=self._get_progress_callback(callback))

    def get_bounds(self):
        return Bounds(Bounds.TYPE_CUSTOM, (self.minx, self.miny, self.minz),
                (self.maxx, self.maxy, self.maxz))


class Model(BaseModel):

    def __init__(self, use_kdtree=True):
        super(Model, self).__init__()
        self._triangles = []
        self._item_groups.append(self._triangles)
        self._export_function = pycam.Exporters.STLExporter.STLExporter
        # marker for state of kdtree and uuid
        self._dirty = True
        # enable/disable kdtree
        self._use_kdtree = use_kdtree
        self._t_kdtree = None
        self.__uuid = None
        
    @property
    def uuid(self):
        if (self.__uuid is None) or self._dirty:
            self._update_caches()
        return self.__uuid

    def append(self, item):
        super(Model, self).append(item)
        if isinstance(item, Triangle):
            self._triangles.append(item)
            # we assume, that the kdtree needs to be rebuilt again
            self._dirty = True

    def reset_cache(self):
        super(Model, self).reset_cache()
        # the triangle kdtree needs to be reset after transforming the model
        self._update_caches()

    def _update_caches(self):
        if self._use_kdtree:
            self._t_kdtree = TriangleKdtree(self.triangles())
        self.__uuid = str(uuid.uuid4())
        # the kdtree is up-to-date again
        self._dirty = False

    def triangles(self, minx=-INFINITE, miny=-INFINITE, minz=-INFINITE,
            maxx=+INFINITE, maxy=+INFINITE, maxz=+INFINITE):
        if (minx == miny == minz == -INFINITE) \
                and (maxx == maxy == maxz == +INFINITE):
            return self._triangles
        if self._use_kdtree:
            # update the kdtree, if new triangles were added meanwhile
            if self._dirty:
                self._update_caches()
            return self._t_kdtree.Search(minx, maxx, miny, maxy)
        return self._triangles

    def get_waterline_polygons(self, plane):
        collision_lines = []
        for t in self._triangles:
            collision_line = plane.intersect_triangle(t, counter_clockwise=True)
            if not collision_line is None:
                collision_lines.append(collision_line)
        # combine these lines into polygons
        contour = ContourModel(plane=plane)
        for line in collision_lines:
            contour.append(line)
        log.debug("Waterline: %f - %d - %s" % (plane.p.z,
                len(contour.get_polygons()),
                [len(p.get_lines()) for p in contour.get_polygons()]))
        return contour.get_polygons()


class ContourModel(BaseModel):

    def __init__(self, plane=None):
        super(ContourModel, self).__init__()
        self.name = "contourmodel%d" % self.id
        if plane is None:
            # the default plane points upwards along the z axis
            plane = Plane(Point(0, 0, 0), Point(0, 0, 1))
        self._plane = plane
        self._line_groups = []
        self._item_groups.append(self._line_groups)
        # there is always just one plane
        self._plane_groups = [self._plane]
        self._item_groups.append(self._plane_groups)
        self._cached_offset_models = {}

    def reset_cache(self):
        super(ContourModel, self).reset_cache()
        # reset the offset model cache
        self._cached_offset_models = {}

    def _merge_polygon_if_possible(self, other_polygon):
        """ Check if the given 'other_polygon' can be connected to another
        polygon of the the current model. Both polygons are merged if possible.
        This function should be called after any "append" event, if the lines to
        be added are given in a random order (e.g. by the "waterline" function).
        """
        connector1 = other_polygon.get_lines()[0]
        connector2 = other_polygon.get_lines()[-1]
        # filter all polygons that can be combined with 'other_polygon'
        connectables = []
        for lg in self._line_groups:
            if lg is other_polygon:
                continue
            if lg.is_connectable(connector1) or lg.is_connectable(connector2):
                connectables.append(lg)
        # merge 'other_polygon' with all other connectable polygons
        for polygon in connectables:
            if other_polygon.is_connectable(polygon.get_lines()[0]):
                for line in polygon.get_lines():
                    other_polygon.append(line)
                self._line_groups.remove(polygon)
            elif other_polygon.is_connectable(polygon.get_lines()[-1]):
                lines = polygon.get_lines()
                lines.reverse()
                for line in lines:
                    other_polygon.append(line)
                self._line_groups.remove(polygon)
            else:
                log.debug("merge_polygon_if_possible: ambiguous combinations " \
                        + "(%s - %s)" % (other_polygon, connectables))

    def append(self, item, unify_overlaps=False):
        super(ContourModel, self).append(item)
        if isinstance(item, Line):
            for line_group in self._line_groups:
                if line_group.is_connectable(item):
                    line_group.append(item)
                    self._merge_polygon_if_possible(line_group)
                    break
            else:
                # add a single line as part of a new group
                new_line_group = Polygon(plane=self._plane)
                new_line_group.append(item)
                self._line_groups.append(new_line_group)
        elif isinstance(item, Polygon):
            if not unify_overlaps or (len(self._line_groups) == 0):
                self._line_groups.append(item)
            else:
                # go through all polygons and check if they can be combined
                is_outer = item.is_outer()
                new_queue = [item]
                processed_polygons = []
                queue = self.get_polygons()
                while len(queue) > 0:
                    polygon = queue.pop()
                    if polygon.is_outer() != is_outer:
                        processed_polygons.append(polygon)
                    else:
                        processed = []
                        while len(new_queue) > 0:
                            new = new_queue.pop()
                            if new.is_polygon_inside(polygon):
                                # "polygon" is obsoleted by "new"
                                processed.extend(new_queue)
                                break
                            elif polygon.is_polygon_inside(new):
                                # "new" is obsoleted by "polygon"
                                continue
                            elif not new.is_overlap(polygon):
                                processed.append(new)
                                continue
                            else:
                                union = polygon.union(new)
                                if union:
                                    for p in union:
                                        if p.is_outer() == is_outer:
                                            new_queue.append(p)
                                        else:
                                            processed_polygons.append(p)
                                else:
                                    processed.append(new)
                                break
                        else:
                            processed_polygons.append(polygon)
                        new_queue = processed
                while len(self._line_groups) > 0:
                    self._line_groups.pop()
                print "Processed polygons: %s" % str([len(p.get_lines()) for p in processed_polygons])
                print "New queue: %s" % str([len(p.get_lines()) for p in new_queue])
                for processed_polygon in processed_polygons + new_queue:
                    self._line_groups.append(processed_polygon)
            self.reset_cache()
        else:
            # ignore any non-supported items (they are probably handled by a
            # parent class)
            pass

    def get_num_of_lines(self):
        return sum([len(group) for group in self._line_groups])

    def get_polygons(self):
        return self._line_groups

    def reverse_directions(self, callback=None):
        for polygon in self._line_groups:
            polygon.reverse_direction()
            if callback and callback():
                return None
        self.reset_cache()

    def get_cropped_model(self, minx, maxx, miny, maxy, minz, maxz):
        new_line_groups = []
        for group in self._line_groups:
            new_groups = group.get_cropped_polygons(minx, maxx, miny, maxy,
                    minz, maxz)
            if not new_groups is None:
                new_line_groups.extend(new_groups)
        if len(new_line_groups) > 0:
            result = ContourModel(plane=self._plane)
            for group in new_line_groups:
                result.append(group)
            return result
        else:
            return None

    def get_offset_model(self, offset, callback=None):
        """ calculate a contour model that surrounds the current model with
        a given offset.
        This is mainly useful for engravings that should not proceed _on_ the
        lines but besides these.
        @value offset: shifting distance; positive values enlarge the model
        @type offset: float
        @value callback: function to call after finishing a single line.
            It should return True if the user interrupted the operation.
        @type callback: callable
        @returns: the new shifted model
        @rtype: pycam.Geometry.Model.Model
        """
        # use a cached offset model if it exists
        if offset in self._cached_offset_models:
            return self._cached_offset_models[offset]
        result = ContourModel(plane=self._plane)
        for group in self._line_groups:
            new_groups = group.get_offset_polygons(offset)
            if not new_groups is None:
                for new_group in new_groups:
                    result.append(new_group)
            if callback and callback():
                return None
        # cache the result
        self._cached_offset_models[offset] = result
        return result

    def check_for_collisions(self, callback=None):
        """ check if lines in different line groups of this model collide

        Returns a pycam.Geometry.Point.Point instance in case of an
        intersection.
        Returns None if the optional "callback" returns True (e.g. the user
        interrupted the operation).
        Otherwise it returns False if no intersections were found.
        """
        def check_bounds_of_groups(g1, g2):
            if (g1.minx <= g2.minx <= g1.maxx) \
                    or (g1.minx <= g2.maxx <= g1.maxx) \
                    or (g2.minx <= g1.minx <= g2.maxx) \
                    or (g2.minx <= g1.maxx <= g2.maxx):
                # the x boundaries overlap
                if (g1.miny <= g2.miny <= g1.maxy) \
                        or (g1.miny <= g2.maxy <= g1.maxy) \
                        or (g2.miny <= g1.miny <= g2.maxy) \
                        or (g2.miny <= g1.maxy <= g2.maxy):
                    # also the y boundaries overlap
                    if (g1.minz <= g2.minz <= g1.maxz) \
                            or (g1.minz <= g2.maxz <= g1.maxz) \
                            or (g2.minz <= g1.minz <= g2.maxz) \
                            or (g2.minz <= g1.maxz <= g2.maxz):
                        # z overlaps as well
                        return True
            return False
        # check each pair of line groups for intersections
        for index, group1 in enumerate(self._line_groups[:-1]):
            for group2 in self._line_groups[index+1:]:
                # check if both groups overlap - otherwise skip this pair
                if check_bounds_of_groups(group1, group2):
                    # check each pair of lines for intersections
                    for line1 in group1.get_lines():
                        for line2 in group2.get_lines():
                            intersection, factor = line1.get_intersection(line2)
                            if intersection:
                                # return just the place of intersection
                                return intersection
            # update the progress visualization and quit if requested
            if callback and callback():
                return None
        return False

