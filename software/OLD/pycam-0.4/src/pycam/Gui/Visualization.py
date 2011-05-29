# -*- coding: utf-8 -*-
"""
$Id: Visualization.py 629 2010-08-23 16:53:06Z sumpfralle $

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

from pycam.Geometry.utils import sqrt

import math
import sys

import OpenGL.GL as GL
import OpenGL.GLU as GLU
import OpenGL.GLUT as GLUT

from OpenGL.constant import Constant
GLUT_WHEEL_UP = Constant('GLUT_WHEEL_UP', 3)
GLUT_WHEEL_DOWN = Constant('GLUT_WHEEL_DOWN', 4)


_DrawCurrentSceneFunc = None
_KeyHandlerFunc = None

# Some api in the chain is translating the keystrokes to this octal string
# so instead of saying: ESCAPE = 27, we use the following.
ESCAPE = '\033'

# Number of the glut window.
window = 0

# Rotations for cube.
xrot = 110
yrot = 180
zrot = 250
scale = 0.5
xdist = 0
ydist = -1.0
zdist = -8.0

texture_num = 2
light = 1
shade_model = GL.GL_FLAT
polygon_mode = GL.GL_FILL
width = 320
height = 200

# A general OpenGL initialization function.  Sets all of the initial parameters.
def InitGL(Width, Height):
    # We call this right after our OpenGL window is created.
    global width, height
    width = Width
    height = Height

    # This Will Clear The Background Color To Black
    GL.glClearColor(0.0, 0.0, 0.0, 0.0)
    # Enables Clearing Of The Depth Buffer
    GL.glClearDepth(1.0)
    # The Type Of Depth Test To Do
    GL.glDepthFunc(GL.GL_LESS)
    # Enables Depth Testing
    GL.glEnable(GL.GL_DEPTH_TEST)
    # Enables Smooth Color Shading
#    GL.glShadeModel(GL.GL_SMOOTH)
    # Enables Flat Color Shading
#    GL.glShadeModel(GL.GL_FLAT)
    GL.glShadeModel(shade_model)

    GL.glMatrixMode(GL.GL_PROJECTION)
    # Reset The Projection Matrix
    GL.glLoadIdentity()

    # Calculate The Aspect Ratio Of The Window
    GLU.gluPerspective(60.0, float(Width)/float(Height), 0.1, 100.0)

    # Setup The Ambient Light
    GL.glLightfv(GL.GL_LIGHT0, GL.GL_AMBIENT, (0.5, 0.5, 0.5, 1.0))
    # Setup The Diffuse Light
    GL.glLightfv(GL.GL_LIGHT0, GL.GL_DIFFUSE, (1.0, 1.0, 1.0, 1.0))
    # Position The Light
    GL.glLightfv(GL.GL_LIGHT0, GL.GL_POSITION, (-10.0, 0.0, 0.0, 1.0))
    # Enable Light One
    GL.glEnable(GL.GL_LIGHT0)

    GL.glMatrixMode(GL.GL_MODELVIEW)
    GL.glMaterial(GL.GL_FRONT_AND_BACK, GL.GL_SPECULAR, (0.1, 0.1, 0.1, 1.0))
#    GL.glMaterial(GL.GL_FRONT_AND_BACK, GL.GL_SHININESS, (0.5))

    GL.glPolygonMode(GL.GL_FRONT_AND_BACK, polygon_mode)

def ReSizeGLScene(Width, Height):
    # Prevent A Divide By Zero If The Window Is Too Small
    if Height == 0:
        Height = 1

    global width, height
    width = Width
    height = Height

    # Reset The Current Viewport And Perspective Transformation
    GL.glViewport(0, 0, Width, Height)
    GL.glMatrixMode(GL.GL_PROJECTION)
    GL.glLoadIdentity()
    GLU.gluPerspective(60.0, float(Width)/float(Height), 0.1, 100.0)
    GL.glMatrixMode(GL.GL_MODELVIEW)

# The main drawing function.
def DrawGLScene():
    global xrot, yrot, zrot, scale, xdist, ydist, zdist, light

    # Clear The Screen And The Depth Buffer
    GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
    GL.glLoadIdentity()					# Reset The View
    GL.glTranslatef(xdist, ydist, zdist)			# Move Into The Screen

    GL.glRotatef(xrot, 1.0, 0.0, 0.0)			# Rotate The Cube On It's X Axis
    GL.glRotatef(yrot, 0.0, 1.0, 0.0)			# Rotate The Cube On It's Y Axis
    GL.glRotatef(zrot, 0.0, 0.0, 1.0)			# Rotate The Cube On It's Z Axis
    GL.glScalef(scale, scale, scale)
    if light:
        GL.glEnable(GL.GL_LIGHTING)
    else:
        GL.glDisable(GL.GL_LIGHTING)

    if _DrawCurrentSceneFunc:
        _DrawCurrentSceneFunc()

    # Since this is double buffered, swap the buffers to display what just got
    # drawn.
    GLUT.glutSwapBuffers()

# The function called whenever a key is pressed
def keyPressed(key, x, y):
    global light, polygon_mode, shade_model
    global xrot, yrot, zrot
    global _KeyHandlerFunc

    key = key.upper()
    if (key == ESCAPE) or (key == 'Q'):
        # If escape is pressed, kill everything.
        sys.exit()
    elif key == 'S':
        light = not light
    elif key == '=':
        print "rot=<%g,%g,%g>" % (xrot, yrot, zrot)
    elif key == 'I':
        xrot = 110
        yrot = 180
        zrot = 250
    elif key == 'T': # top
        xrot = 0
        yrot = 0
        zrot = 0
    elif key == 'F': # front
        xrot = -90
        yrot = 0
        zrot = 0
    elif key == 'R': # right
        xrot = -90
        yrot = 0
        zrot = -90
    elif key == 'L': # left
        xrot = -90
        yrot = 0
        zrot = +90
    elif key == 'M':
        if shade_model == GL.GL_SMOOTH:
            shade_model = GL.GL_FLAT
        else:
            shade_model = GL.GL_SMOOTH
        GL.glShadeModel(shade_model)
    elif key == 'P':
        if polygon_mode == GL.GL_FILL:
            polygon_mode = GL.GL_LINE
        else:
            polygon_mode = GL.GL_FILL
        GL.glPolygonMode(GL.GL_FRONT_AND_BACK, polygon_mode)
    elif _KeyHandlerFunc:
        _KeyHandlerFunc(key, x, y)

class mouseState:
    button = None
    state = None
    x = 0
    y = 0

def mousePressed(button, state, x, y):
    global xrot, yrot, zrot, xdist, ydist, zdist, scale
    if button == GLUT_WHEEL_DOWN:
        scale *= 1.1
    elif button == GLUT_WHEEL_UP:
        scale /= 1.1

    mouseState.button = button
    mouseState.state = state
    mouseState.x = float(x)
    mouseState.y = float(y)

def mouseMoved(x, y):
    global xrot, yrot, zrot, xdist, ydist, zdist, scale
    global width, height
    x = float(x)
    y = float(y)
    a1 = math.atan2(mouseState.y-height/2.0, mouseState.x-width/2.0)
    r1 = sqrt((mouseState.y - height / 2.0) ** 2 \
            + (mouseState.x - width / 2.0) ** 2)
    a2 = math.atan2(y-height/2.0, x-width/2.0)
    r2 = sqrt((y - height / 2.0) ** 2 + (x - width / 2.0) ** 2)
    if (mouseState.button == GLUT.GLUT_LEFT_BUTTON) \
            or (mouseState.button == GLUT.GLUT_RIGHT_BUTTON):
        a3 = math.acos(mouseState.x/width-0.5)
        a4 = math.acos(x/width-0.5)
        zrot = zrot - (a4-a3)*180/math.pi*2
    if mouseState.button == GLUT.GLUT_RIGHT_BUTTON:
        a3 = math.acos(mouseState.y/height-0.5)
        a4 = math.acos(y/height-0.5)
        if x > width / 2.0:
            yrot = yrot + (a4-a3)*180/math.pi*2
        else:
            yrot = yrot - (a4-a3)*180/math.pi*2
    if mouseState.button == GLUT.GLUT_LEFT_BUTTON:
        a3 = math.acos(mouseState.y/width-0.5)
        a4 = math.acos(y/width-0.5)
        xrot = xrot - (a4-a3)*180/math.pi*2
    mouseState.x = x
    mouseState.y = y

def Visualization(title, drawScene=DrawGLScene, width=320, height=200,
        handleKey=None):
    global window, _DrawCurrentSceneFunc, _KeyHandlerFunc
    GLUT.glutInit(sys.argv)

    _DrawCurrentSceneFunc = drawScene

    if handleKey:
        _KeyHandlerFunc = handleKey

    # Select type of Display mode:
    #  Double buffer
    #  RGBA color
    # Alpha components supported
    # Depth buffer
    GLUT.glutInitDisplayMode(GLUT.GLUT_RGBA | GLUT.GLUT_DOUBLE \
            | GLUT.GLUT_DEPTH)

    # get a 640 x 480 window
    GLUT.glutInitWindowSize(640, 480)

    # the window starts at the upper left corner of the screen
    GLUT.glutInitWindowPosition(0, 0)

    # Okay, like the C version we retain the window id to use when closing, but
    # for those of you new to Python (like myself), remember this assignment
    # would make the variable local and not global if it weren't for the global
    # declaration at the start of main.
    window = GLUT.glutCreateWindow(title)

    # Register the drawing function with glut, BUT in Python land, at least
    # using PyOpenGL, we need to set the function pointer and invoke a function
    # to actually register the callback, otherwise it would be very much like
    # the C version of the code.
    GLUT.glutDisplayFunc(DrawGLScene)

    # Uncomment this line to get full screen.
    # GLUT.glutFullScreen()

    # When we are doing nothing, redraw the scene.
    GLUT.glutIdleFunc(DrawGLScene)

    # Register the function called when our window is resized.
    GLUT.glutReshapeFunc(ReSizeGLScene)

    # Register the function called when the keyboard is pressed.
    GLUT.glutKeyboardFunc(keyPressed)

    # Register the function called when the mouse is pressed.
    GLUT.glutMouseFunc(mousePressed)

    # Register the function called when the mouse is pressed.
    GLUT.glutMotionFunc(mouseMoved)

    # Initialize our window.
    InitGL(640, 480)

    # Start Event Processing Engine
    GLUT.glutMainLoop()


test_model = None
test_cutter = None
test_pathlist = None

def DrawTestScene():
    global test_model, test_cutter, test_pathlist
    if test_model:
        GL.glColor4f(1, 0.5, 0.5, 0.1)
        test_model.to_OpenGL()
    if test_cutter:
        GL.glColor3f(0.5, 0.5, 0.5)
        test_cutter.to_OpenGL()
    if test_pathlist:
        for path in test_pathlist:
            GL.glColor3f(0.5, 0.5, 1)
            GL.glBegin(GL.GL_LINE_STRIP)
            for point in path.points:
                GL.glVertex3f(point.x, point.y, point.z)
#                GL.glVertex3f(point.x, point.y, point.z+1)
            GL.glEnd()

def ShowTestScene(model=None, cutter=None, pathlist=None):
    global test_model, test_cutter, test_pathlist
    test_model = model
    test_cutter = cutter
    test_pathlist = pathlist
    Visualization("TestScene", DrawTestScene)
