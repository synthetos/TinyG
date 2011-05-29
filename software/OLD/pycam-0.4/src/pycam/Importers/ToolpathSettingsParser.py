# -*- coding: utf-8 -*-
"""
$Id: ToolpathSettingsParser.py 629 2010-08-23 16:53:06Z sumpfralle $

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

import pycam.Gui.Settings
import pycam.Gui.Project
import pycam.Utils.log
import re
import os
import sys

COMMENT_CHARACTERS = r";#"
REGEX_META_KEYWORDS = r"[%s]?%s (.*): (.*)$" % (COMMENT_CHARACTERS,
        pycam.Gui.Project.ProjectGui.META_DATA_PREFIX)
REGEX_SETTINGS_START = r"[%s]?%s$" % (COMMENT_CHARACTERS,
        pycam.Gui.Settings.ToolpathSettings.META_MARKER_START)
REGEX_SETTINGS_END = r"[%s]?%s$" % (COMMENT_CHARACTERS,
        pycam.Gui.Settings.ToolpathSettings.META_MARKER_END)

log = pycam.Utils.log.get_logger()


def parse_toolpath_settings(filename):
    """ parse potential PyCAM settings from a given file

    This is mainly useful to retrieve task settings from a GCode file.
    @value filename: the name of the file to be read
    @type filename: str
    @returns: a dictionary (of all setting names and values) and the content
            of the 'comment' section (as a single string)
    @rtype: tuple(dict, str)
    """
    keywords = {}
    in_meta_zone = False
    meta_content = []
    if filename == "-":
        # read from stdin, if the input filename is "-"
        infile = sys.stdin
        close_file = False
    else:
        # open the file
        try:
            infile = open(filename,"r")
        except IOError, err_msg:
            log.warn("ToolpathSettingsParser: Failed to read file (%s): %s" % \
                    (filename, err_msg))
            return None
        close_file = True
    for line in infile.readlines():
        match = re.match(REGEX_META_KEYWORDS, line)
        if match:
            keywords[match.groups()[0]] = match.groups()[1].strip()
        if in_meta_zone:
            if re.match(REGEX_SETTINGS_END, line):
                in_meta_zone = False
            else:
                if line and line[0] in COMMENT_CHARACTERS:
                    meta_content[-1].append(line[1:].strip())
        else:
            if re.match(REGEX_SETTINGS_START, line):
                in_meta_zone = True
                meta_content.append([])
    if close_file:
        # only close the file if it was opened before (e.g. not stdin)
        infile.close()
    return keywords, [os.linesep.join(one_block) for one_block in meta_content]

if __name__ == "__main__":
    # for testing: output the parsed content of the given file (first argument)
    print "\n#################\n".join(parse_toolpath_settings(sys.argv[1])[1])

