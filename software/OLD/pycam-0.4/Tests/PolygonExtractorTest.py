#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
$Id: PolygonExtractorTest.py 629 2010-08-23 16:53:06Z sumpfralle $

Copyright 2009 Lode Leroy

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

import sys
sys.path.insert(0,'.')

from pycam.Geometry.PolygonExtractor import *
import sys

print ""

col = [ 0xff0000, 0x00ff00, 0x0000ff, 0x00ffff, 0xff00ff, 0xffff00,
        0x007f3f, 0x3f007f, 0x7f3f00, 0x3f7f00, 0x003f7f, 0x7f003f ]
def r(x):
    return ((col[(x)%12]>>16)&0xff)
def g(x):
    return ((col[(x)%12]>> 8)&0xff)
def b(x):
    return ((col[(x)%12]>> 0)&0xff)

def test_image(image, image2=None):
    pe = PolygonExtractor(policy=PolygonExtractor.CONTOUR)
    h = len(image)
    w = len(image[0])
    for dir in [1, 0, 1]:
        if dir == 1 and image2:
            image = image2
        pe.new_direction(dir)
        imax = 0
        jmax = 0
        if dir==0:
            imax=w
            jmax=h
        else:
            imax=h
            jmax=w

        for j in range(0,jmax):
            pe.new_scanline()
            x=0
            y=0
            if (dir==0):
                x = 0
                y = j
            else:
                x = j
                y = 0
            c = image[y][x]
            for i in range(0,imax):
                if dir==0:
                    x = i
                else:
                    y = i

                if image[y][x] != c:
                    if dir==0:
                        pe.append(Point(x-0.5,y, 0))
                    else:
                        pe.append(Point(x,y-0.5, 0))
                    c = image[y][x]

            pe.end_scanline()

        pe.end_direction()

    pe.finish()

#    w = int(w*1.5)
#    h = int(h*2)

    fg = 0xae
    bg = 0xfe

    for dir in [0,1,2]:
        screen = []
        for i in range(0,h):
            screen.append([])
            for j in range(0,w):
                screen[i].append([bg,bg,bg])

        path_list = None
        if dir == 0:
            path_list = pe.hor_path_list
        elif dir == 1:
            path_list = pe.ver_path_list
        else:
            path_list = pe.merge_path_list

        if not path_list:
            continue

        for curr_path in path_list:
            for point in curr_path.points:
                x = (int)(point.x)
                y = (int)(point.y)

                screen[y][x][0] = r(curr_path.id);
                screen[y][x][1] = g(curr_path.id);
                screen[y][x][2] = b(curr_path.id);

                if dir==0:
                    for i in range(x+1, w):
                        if screen[y][i][0] == bg:
                            screen[y][i][0] = fg
                            screen[y][i][1] = fg
                            screen[y][i][2] = fg
                        elif screen[y][i][0] == fg:
                            screen[y][i][0] = bg
                            screen[y][i][1] = bg
                            screen[y][i][2] = bg
                else:
                    for i in range(y+1,h):
                        if screen[i][x][0] == bg:
                            screen[i][x][0] = fg
                            screen[i][x][1] = fg
                            screen[i][x][2] = fg
                        elif screen[i][x][0] == fg:
                            screen[i][x][0] = bg
                            screen[i][x][1] = bg
                            screen[i][x][2] = bg

        l = "+"
        for i in range(0, w):
            l += "-"

        l += "+"
        print l
        for j in range(0, h):
            l = "|"
            for i in range(0, w):
                if (screen[j][i][0] == bg):
                    l += " "
                elif (screen[j][i][0] == fg):
                    l += "."
                else:
                    l += "*"
            l += "|"
            print l

        l = "+"
        for i in range(0,w):
            l += "-"

        l += "+"
        print l

        filename = None
        if dir==0:
            filename = "polygon_h.svg"
        elif dir==1:
            filename = "polygon_v.svg"
        else:
            filename = "polygon.svg"

        f = open(filename,"w")
        f.write("<?xml version='1.0'?>\n")
        f.write("<svg")
        f.write(" xmlns='http://www.w3.org/2000/svg'")
        f.write(" width='%f'" % (10+(w)*30))
        f.write(" height='%f'" % (10+(h)*30))
        f.write(">\n")
        f.write("<g transform='translate(10,10)'>\n")
        f.write("<g transform='scale(20)'>\n")

        for winding in range(0,10):
            f.write("<g ")
            if (pe.policy == PolygonExtractor.CONTOUR) and (winding&1)==0:
                f.write(" fill='#ffffff'")
            else:
                f.write(" fill='#e0e0e0'")
            f.write(" stroke-width='0.1'")
            f.write(">\n")
	    s = ""
            for path in path_list:
                if path.winding!=winding:
                    continue
                f.write("<path")
		f.write(" stroke='#%02x%02x%02x'" %(r(path.id),g(path.id),b(path.id)))
                f.write(" d='")
                first = True
                for point in path.points:
                    x = point.x
                    y = point.y;
                    s += "<text x='%g' y='%g' font-size='0.3' fill='#000000'>%d</text>\n" % (x, y, point.id)
                    if first:
                        first = False
                        f.write("M ")
                    else:
                        f.write(" L ")

                    f.write("%g %g" % (x,y))


                f.write(" z'/>\n")
                f.write(s)
            f.write("</g>\n")


        f.write("<g font-size='0.3' fill='#000000' opacity='0.5'>\n")
        for j in range(0,h):
            for i in range(0,w):
		c = image[j][i]
		if c != ' ':
		    f.write("<text x='%g' y='%g'>%c</text>\n" % (i*1.0,j*1.0,c))
        f.write("</g>\n")

        f.write("</g>\n")
        f.write("</g>\n")
        f.write("</svg>\n")

if __name__ == "__main__":
  test = 0
  if len(sys.argv)>1:
      test = int(sys.argv[1])

  if (test==0):
    image = [
      "                                               ",
      "                  **                           ",
      "                 ****                          ",
      "                ******                         ",
      "                ******                         ",
      "                 ****                          ",
      "                  **                           ",
      "                                               ",
    ]
    test_image(image)

  if (test==1):
    image = [
      "                                               ",
      "               ******                          ",
      "              *** *****                        ",
      "              **   *****                       ",
      "                    ***                        ",
      "                                               ",
    ]
    test_image(image)

  if (test==2):
    image = [
      "                                               ",
      "              ***                              ",
      "             *****   **                        ",
      "              ***** ***                        ",
      "                 *****                         ",
      "                                               ",
    ]
    test_image(image)

  if (test==3):
    image = [
      "                                               ",
      "                *******                        ",
      "               *********                       ",
      "              *** *** ***                      ",
      "              **  ***  **                      ",
      "                                               ",
      "                                               ",
    ]
    test_image(image)

  if (test==4):
    image = [
      "                                               ",
      "              **  ***  **                      ",
      "              *** *** ***                      ",
      "               *********                       ",
      "                *******                        ",
      "                                               ",
      "                                               ",
    ]
    test_image(image)

  if (test==5):
    image = [
      "                                               ",
      "                *******                        ",
      "               *********                       ",
      "              *** *** ***                      ",
      "              **  ***  **                      ",
      "              *** *** ***                      ",
      "               *********                       ",
      "                *******                        ",
      "                                               ",
      "                                               ",
    ]
    test_image(image)

  if (test==6):
    image = [
      "                                               ",
      "                *******                        ",
      "               *********                       ",
      "              *** *** ***                      ",
      "               *********                       ",
      "                *******                        ",
      "                                               ",
      "                                               ",
    ]
    test_image(image)

  if (test==7):
    image = [
      "                                               ",
      "              ***********                      ",
      "             *****   *****                     ",
      "             **  *****  **                     ",
      "              ***********                      ",
      "                                               ",
      "                                               ",
    ]
    test_image(image)

  if (test==8):
    image = [
      "                                               ",
      "              ***********                      ",
      "             **         **                     ",
      "            **   *****   **                    ",
      "            **  *** ***  **                    ",
      "            **  **   **  **                    ",
      "            **  *** ***  **                    ",
      "            **   *****   **                    ",
      "             **         **                     ",
      "              ***********                      ",
      "                                               ",
      "                                               ",
    ]
    test_image(image)

  if (test==9):
    image = [
      "                                               ",
      "                                               ",
      "              ***********                      ",
      "              ***********                      ",
      "              ***********                      ",
      "              ***********                      ",
      "                                               ",
      "                                               ",
      "                  ***                          ",
      "                 *****                         ",
      "               *********                       ",
      "              ***********                      ",
      "                                               ",
      "                                               ",
    ]
    image2 = [
      "                                               ",
      "                                               ",
      "              ***********                      ",
      "              ***********                      ",
      "              ***********                      ",
      "              ***********                      ",
      "                                               ",
      "                   *                           ",
      "                  ***                          ",
      "                 *****                         ",
      "               *********                       ",
      "              ***********                      ",
      "                                               ",
      "                                               ",
    ]
    test_image(image, image2 )


  if (test==10):
    image = [
      "                                                 ",
      "            ********************                 ",
      "         ****************************            ",
      "       ************        ************          ",
      "      *****   ********  *********   *******      ",
      "      **************      *****      ******      ",
      "         ********************     *******        ",
      "               ***********************           ",
      "                                                 "
    ]
    test_image(image)

  if (test==11):
    image = [
      "                                                         ",
      "     **********************************************      ",
      "  ******   ***   ***   ***   ***   ***   ***   *******   ",
      "  *****     *     *     *     *     *     *     ******   ",
      "  ******   ***   ***   ***   ***   ***   ***   *******   ",
      "     **********************************************      ",
      "                                                         "
    ]
    test_image(image)



