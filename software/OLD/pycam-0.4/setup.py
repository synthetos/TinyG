#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
$Id: setup.py 793 2010-10-18 07:54:19Z sumpfralle $

Copyright 2010 Lars Kruse <devel@sumpfralle.de>
Copyright 2010 Arthur Magill

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

from distutils.core import setup
import distutils.sysconfig
import glob
import os.path
import sys
# add the local pycam source directory to the PYTHONPATH
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))
from pycam import VERSION

setup(
    name="pycam",
    version=VERSION,
    license="GPL v3",
    description="Open Source CAM - Toolpath Generation for 3-Axis CNC machining",
    author="Lode Leroy",
    #author_email="",
    provides=["pycam"],
    requires=["ode", "gtk", "gtk.gtkgl", "OpenGL"],
    url="http://sourceforge.net/projects/pycam",
    download_url="http://sourceforge.net/projects/pycam/files",
    keywords=["3-axis", "cnc", "cam", "toolpath", "machining", "g-code"],
    long_description="""IMPORTANT NOTE: Please read the list of requirements:
http://sourceforge.net/apps/mediawiki/pycam/index.php?title=Requirements
Basically you will need Python, GTK and OpenGL.
""",
    # full list of classifiers at:
    #   http://pypi.python.org/pypi?:action=list_classifiers
    classifiers=[
        "Programming Language :: Python",
        "Programming Language :: Python :: 2",
        "Development Status :: 3 - Alpha",
        "License :: OSI Approved :: GNU General Public License (GPL)",
        "Topic :: Scientific/Engineering",
        "Environment :: Win32 (MS Windows)",
        "Environment :: X11 Applications",
        "Intended Audience :: Manufacturing",
        "Operating System :: Microsoft :: Windows",
        "Operating System :: POSIX",
    ],
    package_dir={'': 'src'},
    packages=[
        "pycam",
        "pycam.Cutters",
        "pycam.Geometry",
        "pycam.Importers",
        "pycam.PathProcessors",
        "pycam.Utils",
        "pycam.Exporters",
        "pycam.Gui",
        "pycam.PathGenerators",
        "pycam.Simulation",
        "pycam.Toolpath",
        "pycam.Physics",
    ],
    scripts = ['pycam', 'pycam_win32_postinstall.py'],
    data_files=[("share/pycam/doc", [
            "COPYING.TXT",
            "technical_details.txt",
            "INSTALL.TXT",
            "LICENSE.TXT",
            "README.TXT",
            "Changelog",
            "release_info.txt"]),
        ("share/pycam/ui", [
            os.path.join("share", "gtk-interface", "pycam-project.ui"),
            os.path.join("share", "gtk-interface", "menubar.xml"),
            os.path.join("share", "gtk-interface", "logo_gui.png"),
            ]),
        ("share/pycam", [os.path.join("share", "pycam.ico")]),
        ("share/pycam/samples", glob.glob(os.path.join("samples", "*"))),
    ],
)

# vim: tabstop=4 expandtab shiftwidth=4 softtabstop=4
