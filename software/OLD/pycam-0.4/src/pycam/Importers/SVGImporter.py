# -*- coding: utf-8 -*-
"""
$Id: SVGImporter.py 740 2010-10-11 03:44:23Z sumpfralle $

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

import pycam.Importers.DXFImporter
import tempfile
import subprocess
import os

log = pycam.Utils.log.get_logger()

def convert_svg2eps(svg_filename, eps_filename, location=None):
    if location is None:
        location = "inkscape"
    try:
        process = subprocess.Popen(stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                args = [location, "--export-eps", eps_filename, svg_filename])
    except OSError, err_msg:
        log.error("SVGImporter: failed to execute 'inkscape' (%s): %s" \
                % (location, err_msg))
        return False
    returncode = process.wait()
    if returncode == 0:
        return True
    else:
        log.warn(("SVGImporter: failed to convert SVG file (%s) to EPS file " \
                + "(%s): %s") % (svg_filename, eps_filename,
                process.stderr.read()))
        return False

def convert_eps2dxf(eps_filename, dxf_filename, location=None):
    if location is None:
        location = "pstoedit"
    try:
        process = subprocess.Popen(stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                args = [location,
                        "-dt",
                        "-f", "dxf:-polyaslines",
                        eps_filename, dxf_filename])
    except OSError, err_msg:
        log.error("SVGImporter: failed to execute 'pstoedit' (%s): %s" \
                % (location, err_msg))
        return False
    returncode = process.wait()
    if returncode == 0:
        return True
    elif returncode == -11:
        log.warn(("SVGImporter: maybe there was a problem with the " \
                + "conversion from EPS (%s) to DXF\n Users of Ubuntu 'lucid' should install " \
                + "the package 'libpstoedit0c2a' from the 'maverick' " \
                + "repository to avoid this warning.") % str(eps_filename))
        return True
    else:
        log.warn(("SVGImporter: failed to convert EPS file (%s) to DXF file " \
                + "(%s): %s") % (eps_filename, dxf_filename,
                process.stderr.read()))
        return False

def import_model(filename, program_locations=None, unit=None):
    if not os.path.isfile(filename):
        log.error("SVGImporter: file (%s) does not exist" % filename)
        return None

    if program_locations and "inkscape" in program_locations:
        inkscape_path = program_locations["inkscape"]
    else:
        inkscape_path = None

    if program_locations and "pstoedit" in program_locations:
        pstoedit_path = program_locations["pstoedit"]
    else:
        pstoedit_path = None

    # the "right" way would be:
    # inkscape --print='| pstoedit -dt -f dxf:-polyaslines - -' input.svg
    # Sadly a bug in v0.47 breaks this: https://bugs.launchpad.net/inkscape/+bug/511361

    # convert svg to eps via inkscape
    eps_file_handle, eps_file_name = tempfile.mkstemp(suffix=".eps")
    os.close(eps_file_handle)
    success = convert_svg2eps(filename, eps_file_name, location=inkscape_path)
    def remove_temp_file(filename):
        if os.path.isfile(filename):
            try:
                os.remove(filename)
            except OSError, err_msg:
                log.warn("SVGImporter: failed to remove temporary file " \
                        + "(%s): %s" % (filename, err_msg))
    # remove the temporary file
    if not success:
        remove_temp_file(eps_file_name)
        return None
    log.info("Successfully converted SVG file to EPS file")

    # convert eps to dxf via pstoedit
    dxf_file_handle, dxf_file_name = tempfile.mkstemp(suffix=".dxf")
    os.close(dxf_file_handle)
    success = convert_eps2dxf(eps_file_name, dxf_file_name, location=pstoedit_path)
    # we don't need the eps file anymore
    remove_temp_file(eps_file_name)
    if not success:
        result = None
    else:
        log.info("Successfully converted EPS file to DXF file")
        result = pycam.Importers.DXFImporter.import_model(dxf_file_name, unit=unit)
    # always remove the dxf file
    remove_temp_file(dxf_file_name)
    return result

