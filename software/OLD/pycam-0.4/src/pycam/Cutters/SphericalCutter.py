# -*- coding: utf-8 -*-
"""
$Id: SphericalCutter.py 732 2010-10-07 00:13:10Z sumpfralle $

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

from pycam.Geometry import Matrix
from pycam.Geometry.Point import Point, Vector
from pycam.Geometry.utils import INFINITE, epsilon, sqrt
from pycam.Geometry.intersection import intersect_sphere_plane, \
        intersect_sphere_point, intersect_sphere_line
from pycam.Cutters.BaseCutter import BaseCutter


try:
    import OpenGL.GL as GL
    import OpenGL.GLU as GLU
    GL_enabled = True
except ImportError:
    GL_enabled = False


class SphericalCutter(BaseCutter):

    def __init__(self, radius, **kwargs):
        BaseCutter.__init__(self, radius, **kwargs)
        self.axis = Vector(0, 0, 1)

    def __repr__(self):
        return "SphericalCutter<%s,%s>" % (self.location, self.radius)

    def get_shape(self, engine="ODE"):
        if engine == "ODE":
            import ode
            import pycam.Physics.ode_physics as ode_physics
            additional_distance = self.get_required_distance()
            radius = self.radius + additional_distance
            center_height = self.height / 2 + radius - additional_distance
            geom = ode.GeomTransform(None)
            geom_drill = ode.GeomCapsule(None, radius, self.height)
            geom_drill.setPosition((0, 0, center_height))
            geom.setGeom(geom_drill)
            geom.children = []
            def reset_shape():
                geom.children = []
            def set_position(x, y, z):
                geom.setPosition((x, y, z))
            def extend_shape(diff_x, diff_y, diff_z):
                reset_shape()
                # see http://mathworld.wolfram.com/RotationMatrix.html
                hypotenuse = sqrt(diff_x * diff_x + diff_y * diff_y)
                # Some paths contain two identical points (e.g. a "touch" of the
                # PushCutter). We don't need any extension for these.
                if hypotenuse == 0:
                    return
                cosinus = diff_x / hypotenuse
                sinus = diff_y / hypotenuse
                # create the cyclinder at the other end
                geom_end_transform = ode.GeomTransform(geom.space)
                geom_end_transform.setBody(geom.getBody())
                geom_end = ode.GeomCapsule(None, radius, self.height)
                geom_end.setPosition((diff_x, diff_y, diff_z + center_height))
                geom_end_transform.setGeom(geom_end)
                # create the block that connects the two cylinders at the end
                rot_matrix_box = (cosinus, sinus, 0.0, -sinus, cosinus, 0.0,
                        0.0, 0.0, 1.0)
                geom_connect_transform = ode.GeomTransform(geom.space)
                geom_connect_transform.setBody(geom.getBody())
                geom_connect = ode_physics.get_parallelepiped_geom((
                        Point(-hypotenuse / 2, radius, -diff_z / 2),
                        Point(hypotenuse / 2, radius, diff_z / 2),
                        Point(hypotenuse / 2, -radius, diff_z / 2),
                        Point(-hypotenuse / 2, -radius, -diff_z / 2)),
                        (Point(-hypotenuse / 2, radius,
                            self.height - diff_z / 2),
                        Point(hypotenuse / 2, radius,
                            self.height + diff_z / 2),
                        Point(hypotenuse / 2, -radius,
                            self.height + diff_z / 2),
                        Point(-hypotenuse / 2, -radius,
                            self.height - diff_z / 2)))
                geom_connect.setRotation(rot_matrix_box)
                geom_connect.setPosition((hypotenuse / 2, 0, radius))
                geom_connect_transform.setGeom(geom_connect)
                # Create a cylinder, that connects the two half spheres at the
                # lower end of both drills.
                geom_cyl_transform = ode.GeomTransform(geom.space)
                geom_cyl_transform.setBody(geom.getBody())
                hypotenuse_3d = Matrix.get_length((diff_x, diff_y, diff_z))
                geom_cyl = ode.GeomCylinder(None, radius, hypotenuse_3d)
                # rotate cylinder vector
                cyl_original_vector = (0, 0, hypotenuse_3d)
                cyl_destination_vector = (diff_x, diff_y, diff_z)
                geom_cyl.setRotation(Matrix.get_rotation_matrix_from_to(
                        cyl_original_vector, cyl_destination_vector))
                # The rotation is around the center - thus we ignore negative
                # diff values.
                geom_cyl.setPosition((abs(diff_x / 2), abs(diff_y / 2),
                        radius - additional_distance))
                geom_cyl_transform.setGeom(geom_cyl)
                # sort the geoms in order of collision probability
                geom.children.extend([geom_connect_transform,
                        geom_cyl_transform, geom_end_transform])
            geom.extend_shape = extend_shape
            geom.reset_shape = reset_shape
            self.shape[engine] = (geom, set_position)
            return self.shape[engine]

    def to_OpenGL(self):
        if not GL_enabled:
            return
        GL.glPushMatrix()
        GL.glTranslate(self.center.x, self.center.y, self.center.z)
        if not hasattr(self, "_sphere"):
            self._sphere = GLU.gluNewQuadric()
        GLU.gluSphere(self._sphere, self.radius, 10, 10)
        if not hasattr(self, "_cylinder"):
            self._cylinder = GLU.gluNewQuadric()
        GLU.gluCylinder(self._cylinder, self.radius, self.radius, self.height,
                10, 10)
        GL.glPopMatrix()

    def moveto(self, location, **kwargs):
        BaseCutter.moveto(self, location, **kwargs)
        self.center = Point(location.x, location.y, location.z + self.radius)

    def intersect_sphere_plane(self, direction, triangle, start=None):
        if start is None:
            start = self.location
        (ccp, cp, d) = intersect_sphere_plane(
                start.sub(self.location).add(self.center), self.distance_radius,
                direction, triangle)
        # offset intersection
        if ccp:
            cl = cp.add(start.sub(ccp))
            return (cl, ccp, cp, d)
        return (None, None, None, INFINITE)

    def intersect_sphere_triangle(self, direction, triangle, start=None):
        (cl, ccp, cp, d) = self.intersect_sphere_plane(direction, triangle,
                start=start)
        if cp and triangle.is_point_inside(cp):
            return (cl, d, cp)
        return (None, INFINITE, None)

    def intersect_sphere_point(self, direction, point, start=None):
        if start is None:
            start = self.location
        (ccp, cp, l) = intersect_sphere_point(
                start.sub(self.location).add(self.center), self.distance_radius,
                self.distance_radiussq, direction, point)
        # offset intersection
        cl = None
        if cp:
            cl = start.add(direction.mul(l))
        return (cl, ccp, cp, l)

    def intersect_sphere_vertex(self, direction, point, start=None):
        (cl, ccp, cp, l) = self.intersect_sphere_point(direction, point,
                start=start)
        return (cl, l, cp)

    def intersect_sphere_line(self, direction, edge, start=None):
        if start is None:
            start = self.location
        (ccp, cp, l) = intersect_sphere_line(
                start.sub(self.location).add(self.center), self.distance_radius,
                self.distance_radiussq, direction, edge)
        # offset intersection
        if ccp:
            cl = cp.sub(ccp.sub(start))
            return (cl, ccp, cp, l)
        return (None, None, None, INFINITE)

    def intersect_sphere_edge(self, direction, edge, start=None):
        (cl, ccp, cp, l) = self.intersect_sphere_line(direction, edge,
                start=start)
        if cp:
            # check if the contact point is between the endpoints
            d = edge.p2.sub(edge.p1)
            m = cp.sub(edge.p1).dot(d)
            if (m < -epsilon) or (m > d.normsq + epsilon):
                return (None, INFINITE, None)
        return (cl, l, cp)

    def intersect_point(self, direction, point, start=None):
        # TODO: probably obsolete?
        return self.intersect_sphere_point(direction, point, start=start)

    def intersect(self, direction, triangle, start=None):
        (cl_t, d_t, cp_t) = self.intersect_sphere_triangle(direction, triangle,
                start=start)
        d = INFINITE
        cl = None
        cp = None
        if d_t < d:
            d = d_t
            cl = cl_t
            cp = cp_t
        if cl and (direction.x == 0) and (direction.y == 0):
            return (cl, d, cp)
        (cl_e1, d_e1, cp_e1) = self.intersect_sphere_edge(direction,
                triangle.e1, start=start)
        (cl_e2, d_e2, cp_e2) = self.intersect_sphere_edge(direction,
                triangle.e2, start=start)
        (cl_e3, d_e3, cp_e3) = self.intersect_sphere_edge(direction,
                triangle.e3, start=start)
        if d_e1 < d:
            d = d_e1
            cl = cl_e1
            cp = cp_e1
        if d_e2 < d:
            d = d_e2
            cl = cl_e2
            cp = cp_e2
        if d_e3 < d:
            d = d_e3
            cl = cl_e3
            cp = cp_e3
        if cl and (direction.x == 0) and (direction.y == 0):
            return (cl, d, cp)
        (cl_p1, d_p1, cp_p1) = self.intersect_sphere_vertex(direction,
                triangle.p1, start=start)
        (cl_p2, d_p2, cp_p2) = self.intersect_sphere_vertex(direction,
                triangle.p2, start=start)
        (cl_p3, d_p3, cp_p3) = self.intersect_sphere_vertex(direction,
                triangle.p3, start=start)
        if d_p1 < d:
            d = d_p1
            cl = cl_p1
            cp = cp_p1
        if d_p2 < d:
            d = d_p2
            cl = cl_p2
            cp = cp_p2
        if d_p3 < d:
            d = d_p3
            cl = cl_p3
            cp = cp_p3
        if cl and (direction.x == 0) and (direction.y == 0):
            return (cl, d, cp)
        if (direction.x != 0) or (direction.y != 0):
            (cl_p1, d_p1, cp_p1) = self.intersect_cylinder_vertex(direction,
                    triangle.p1, start=start)
            (cl_p2, d_p2, cp_p2) = self.intersect_cylinder_vertex(direction,
                    triangle.p2, start=start)
            (cl_p3, d_p3, cp_p3) = self.intersect_cylinder_vertex(direction,
                    triangle.p3, start=start)
            if d_p1 < d:
                d = d_p1
                cl = cl_p1
                cp = cp_p1
            if d_p2 < d:
                d = d_p2
                cl = cl_p2
                cp = cp_p2
            if d_p3 < d:
                d = d_p3
                cl = cl_p3
                cp = cp_p3
            (cl_e1, d_e1, cp_e1) = self.intersect_cylinder_edge(direction,
                    triangle.e1, start=start)
            (cl_e2, d_e2, cp_e2) = self.intersect_cylinder_edge(direction,
                    triangle.e2, start=start)
            (cl_e3, d_e3, cp_e3) = self.intersect_cylinder_edge(direction,
                    triangle.e3, start=start)
            if d_e1 < d:
                d = d_e1
                cl = cl_e1
                cp = cp_e1
            if d_e2 < d:
                d = d_e2
                cl = cl_e2
                cp = cp_e2
            if d_e3 < d:
                d = d_e3
                cl = cl_e3
                cp = cp_e3
        return (cl, d, cp)

