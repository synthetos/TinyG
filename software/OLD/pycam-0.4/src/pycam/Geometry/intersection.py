# -*- coding: utf-8 -*-
"""
$Id: intersection.py 751 2010-10-12 12:29:20Z sumpfralle $

Copyright 2008-2010 Lode Leroy

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


#import pycam.Geometry
from pycam.Utils.polynomials import poly4_roots
from pycam.Geometry.utils import INFINITE, sqrt, epsilon
from pycam.Geometry.Plane import Plane
from pycam.Geometry.Line import Line
from pycam.Geometry.Point import Point

def isNear(a, b):
    return abs(a - b) < epsilon

def isZero(a):
    return isNear(a, 0)

def intersect_lines(xl, zl, nxl, nzl, xm, zm, nxm, nzm):
    X = None
    Z = None
    try:
        if isZero(nzl) and isZero(nzm):
            pass
        elif isZero(nzl) or isZero(nxl):
            X = xl
            Z = zm + (xm - xl) * nxm / nzm
            return (X, Z)
        elif isZero(nzm) or isZero(nxm):
            X = xm
            Z = zl - (xm - xl) * nxl / nzl
            return (X, Z)
        else:
            X = (zl - zm +(xm * nxm / nzm - xl * nxl / nzl)) \
                    / (nxm / nzm - nxl / nzl)
            if X and xl < X and X < xm:
                Z = zl + (X -xl) * nxl / nzl
                return (X, Z)
    except ZeroDivisionError:
        pass
    return (None, None)

def intersect_cylinder_point(center, axis, radius, radiussq, direction, point):
    # take a plane along direction and axis
    n = direction.cross(axis).normalized()
    # distance of the point to this plane
    d = n.dot(point) - n.dot(center)
    if abs(d) > radius - epsilon:
        return (None, None, INFINITE)
    # ccl is on cylinder
    d2 = sqrt(radiussq-d*d)
    ccl = center.add(n.mul(d)).add(direction.mul(d2))
    # take plane through ccl and axis
    plane = Plane(ccl, direction)
    # intersect point with plane
    (ccp, l) = plane.intersect_point(direction, point)
    return (ccp, point, -l)

def intersect_cylinder_line(center, axis, radius, radiussq, direction, edge):
    d = edge.dir
    # take a plane throught the line and along the cylinder axis (1)
    n = d.cross(axis)
    if n.norm == 0:
        # no contact point, but should check here if cylinder *always*
        # intersects line...
        return (None, None, INFINITE)
    n = n.normalized()
    # the contact line between the cylinder and this plane (1)
    # is where the surface normal is perpendicular to the plane
    # so line := ccl + \lambda * axis
    if n.dot(direction) < 0:
        ccl = center.sub(n.mul(radius))
    else:
        ccl = center.add(n.mul(radius))
    # now extrude the contact line along the direction, this is a plane (2)
    n2 = direction.cross(axis)
    if n2.norm == 0:
        # no contact point, but should check here if cylinder *always*
        # intersects line...
        return (None, None, INFINITE)
    n2 = n2.normalized()
    plane1 = Plane(ccl, n2)
    # intersect this plane with the line, this gives us the contact point
    (cp, l) = plane1.intersect_point(d, edge.p1)
    if not cp:
        return (None, None, INFINITE)
    # now take a plane through the contact line and perpendicular to the
    # direction (3)
    plane2 = Plane(ccl, direction)
    # the intersection of this plane (3) with the line through the contact point
    # gives us the cutter contact point
    (ccp, l) = plane2.intersect_point(direction, cp)
    cp = ccp.add(direction.mul(-l))
    return (ccp, cp, -l)

def intersect_circle_plane(center, radius, direction, triangle):
    # let n be the normal to the plane
    n = triangle.normal
    if n.dot(direction) == 0:
        return (None, None, INFINITE)
    # project onto z=0
    n2 = Point(n.x, n.y, 0)
    if n2.norm == 0:
        (cp, d) = triangle.plane.intersect_point(direction, center)
        ccp = cp.sub(direction.mul(d))
        return (ccp, cp, d)
    n2 = n2.normalized()
    # the cutter contact point is on the circle, where the surface normal is n
    ccp = center.add(n2.mul(-radius))
    # intersect the plane with a line through the contact point
    (cp, d) = triangle.plane.intersect_point(direction, ccp)
    return (ccp, cp, d)

def intersect_circle_point(center, axis, radius, radiussq, direction, point):
    # take a plane through the base
    plane = Plane(center, axis)
    # intersect with line gives ccp
    (ccp, l) = plane.intersect_point(direction, point)
    # check if inside circle
    if ccp and (center.sub(ccp).normsq < radiussq - epsilon):
        return (ccp, point, -l)
    return (None, None, INFINITE)

def intersect_circle_line(center, axis, radius, radiussq, direction, edge):
    # make a plane by sliding the line along the direction (1)
    d = edge.dir
    if d.dot(axis) == 0:
        if direction.dot(axis) == 0:
            return (None, None, INFINITE)
        plane = Plane(center, axis)
        (p1, l) = plane.intersect_point(direction, edge.p1)
        (p2, l) = plane.intersect_point(direction, edge.p2)
        pc = Line(p1, p2).closest_point(center)
        d_sq = pc.sub(center).normsq
        if d_sq > radiussq - epsilon:
            return (None, None, INFINITE)
        a = sqrt(radiussq - d_sq)
        d1 = p1.sub(pc).dot(d)
        d2 = p2.sub(pc).dot(d)
        ccp = None
        cp = None
        if abs(d1) < a - epsilon:
            ccp = p1
            cp = p1.sub(direction.mul(l))
        elif abs(d2) < a - epsilon:
            ccp = p2
            cp = p2.sub(direction.mul(l))
        elif ((d1 < -a + epsilon) and (d2 > a - epsilon)) \
                or ((d2 < -a + epsilon) and (d1 > a - epsilon)):
            ccp = pc
            cp = pc.sub(direction.mul(l))
        return (ccp, cp, -l)
    n = d.cross(direction)
    if n.norm == 0:
        # no contact point, but should check here if circle *always* intersects
        # line...
        return (None, None, INFINITE)
    n = n.normalized()
    # take a plane through the base
    plane = Plane(center, axis)
    # intersect base with line
    (lp, l) = plane.intersect_point(d, edge.p1)
    if not lp:
        return (None, None, INFINITE)
    # intersection of 2 planes: lp + \lambda v
    v = axis.cross(n)
    if v.norm == 0:
        return (None, None, INFINITE)
    v = v.normalized()
    # take plane through intersection line and parallel to axis
    n2 = v.cross(axis)
    if n2.norm == 0:
        return (None, None, INFINITE)
    n2 = n2.normalized()
    # distance from center to this plane
    dist = n2.dot(center) - n2.dot(lp)
    distsq = dist * dist
    if distsq > radiussq - epsilon:
        return (None, None, INFINITE)
    # must be on circle
    dist2 = sqrt(radiussq - distsq)
    if d.dot(axis) < 0:
        dist2 = -dist2
    ccp = center.sub(n2.mul(dist)).sub(v.mul(dist2))
    plane = Plane(edge.p1, d.cross(direction).cross(d))
    (cp, l) = plane.intersect_point(direction, ccp)
    return (ccp, cp, l)

def intersect_sphere_plane(center, radius, direction, triangle):
    # let n be the normal to the plane
    n = triangle.normal
    if n.dot(direction) == 0:
        return (None, None, INFINITE)
    # the cutter contact point is on the sphere, where the surface normal is n
    if n.dot(direction) < 0:
        ccp = center.sub(n.mul(radius))
    else:
        ccp = center.add(n.mul(radius))
    # intersect the plane with a line through the contact point
    (cp, d) = triangle.plane.intersect_point(direction, ccp)
    return (ccp, cp, d)

def intersect_sphere_point(center, radius, radiussq, direction, point):
    # line equation
    # (1) x = p_0 + \lambda * d
    # sphere equation
    # (2) (x-x_0)^2 = R^2
    # (1) in (2) gives a quadratic in \lambda
    p0_x0 = center.sub(point)
    a = direction.normsq
    b = 2 * p0_x0.dot(direction)
    c = p0_x0.normsq - radiussq
    d = b * b - 4 * a * c
    if d < 0:
        return (None, None, INFINITE)
    if a < 0:
        l = (-b + sqrt(d)) / (2 * a)
    else:
        l = (-b - sqrt(d)) / (2 * a)
    # cutter contact point
    ccp = point.add(direction.mul(-l))
    return (ccp, point, l)

def intersect_sphere_line(center, radius, radiussq, direction, edge):
    # make a plane by sliding the line along the direction (1)
    d = edge.dir
    n = d.cross(direction)
    if n.norm == 0:
        # no contact point, but should check here if sphere *always* intersects
        # line...
        return (None, None, INFINITE)
    n = n.normalized()

    # calculate the distance from the sphere center to the plane
    dist = - center.dot(n) + edge.p1.dot(n)
    if abs(dist) > radius - epsilon:
        return (None, None, INFINITE)
    # this gives us the intersection circle on the sphere

    # now take a plane through the edge and perpendicular to the direction (2)
    # find the center on the circle closest to this plane

    # which means the other component is perpendicular to this plane (2)
    n2 = n.cross(d).normalized()

    # the contact point is on a big circle through the sphere...
    dist2 = sqrt(radiussq - dist * dist)

    # ... and it's on the plane (1)
    ccp = center.add(n.mul(dist)).add(n2.mul(dist2))

    # now intersect a line through this point with the plane (2)
    plane = Plane(edge.p1, n2)
    (cp, l) = plane.intersect_point(direction, ccp)
    return (ccp, cp, l)

def intersect_torus_plane(center, axis, majorradius, minorradius, direction,
        triangle):
    # take normal to the plane
    n = triangle.normal
    if n.dot(direction) == 0:
        return (None, None, INFINITE)
    if n.dot(axis) == 1:
        return (None, None, INFINITE)
    # find place on torus where surface normal is n
    b = n.mul(-1)
    z = axis
    a = b.sub(z.mul(z.dot(b)))
    a_sq = a.normsq
    if a_sq <= 0:
        return (None, None, INFINITE)
    a = a.div(sqrt(a_sq))
    ccp = center.add(a.mul(majorradius)).add(b.mul(minorradius))
    # find intersection with plane
    (cp, l) = triangle.plane.intersect_point(direction, ccp)
    return (ccp, cp, l)

def intersect_torus_point(center, axis, majorradius, minorradius, majorradiussq,
        minorradiussq, direction, point):
    dist = 0
    if (direction.x == 0) and (direction.y == 0):
        # drop
        minlsq = (majorradius - minorradius) ** 2
        maxlsq = (majorradius + minorradius) ** 2
        l_sq = (point.x-center.x) ** 2 + (point.y - center.y) ** 2
        if (l_sq < minlsq + epsilon) or (l_sq > maxlsq - epsilon):
            return (None, None, INFINITE)
        l = sqrt(l_sq)
        z_sq = minorradiussq - (majorradius - l) ** 2
        if z_sq < 0:
            return (None, None, INFINITE)
        z = sqrt(z_sq)
        ccp = Point(point.x, point.y, center.z - z)
        dist = ccp.z - point.z
    elif direction.z == 0:
        # push
        z = point.z - center.z
        if abs(z) > minorradius - epsilon:
            return (None, None, INFINITE)
        l = majorradius + sqrt(minorradiussq - z * z)
        n = axis.cross(direction)
        d = n.dot(point) - n.dot(center)
        if abs(d) > l - epsilon:
            return (None, None, INFINITE)
        a = sqrt(l * l - d * d)
        ccp = center.add(n.mul(d).add(direction.mul(a)))
        ccp.z = point.z
        dist = point.sub(ccp).dot(direction)
    else:
        # general case
        x = point.sub(center)
        v = direction.mul(-1)
        x_x = x.dot(x)
        x_v = x.dot(v)
        x1 = Point(x.x, x.y, 0)
        v1 = Point(v.x, v.y, 0)
        x1_x1 = x1.dot(x1)
        x1_v1 = x1.dot(v1)
        v1_v1 = v1.dot(v1)
        R2 = majorradiussq
        r2 = minorradiussq
        a = 1.0
        b = 4 * x_v
        c = 2 * (x_x + 2 * x_v ** 2 + (R2 - r2) - 2 * R2 * v1_v1)
        d = 4 * (x_x * x_v + x_v * (R2 - r2) - 2 * R2 * x1_v1)
        e = (x_x) ** 2 + 2 * x_x * (R2 - r2) + (R2 - r2) ** 2 - 4 * R2 * x1_x1
        r = poly4_roots(a, b, c, d, e)
        if not r:
            return (None, None, INFINITE)
        else:
            l = min(r)
        ccp = point.add(direction.mul(-l))
        dist = l
    return (ccp, point, dist)

