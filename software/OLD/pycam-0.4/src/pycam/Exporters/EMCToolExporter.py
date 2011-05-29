# -*- coding: utf-8 -*-
"""
$Id: EMCToolExporter.py 629 2010-08-23 16:53:06Z sumpfralle $

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

import os

class EMCToolExporter:

    def __init__(self, tools):
        self.tools = tools

    def get_tool_definition_string(self):
        result = []
        #result.append(self.HEADER_ROW)
        for index in range(len(self.tools)):
            tool = self.tools[index]
            # use an arbitrary length
            tool_length = tool["tool_radius"] * 10
            line = "T%d P%d D%f Z-%f ;%s" % (index + 1, index + 1,
                    2 * tool["tool_radius"], tool_length, tool["name"])
            result.append(line)
        # add the dummy line for the "last" tool
        result.append("T99999 P99999 Z+0.100000 ;dummy tool")
        return os.linesep.join(result)

