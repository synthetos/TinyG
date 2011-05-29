# -*- coding: utf-8 -*-
"""
$Id: utils.py 764 2010-10-13 23:20:15Z sumpfralle $

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


import decimal
import math

INFINITE = 100000
epsilon = 0.00001

# use the "decimal" module for fixed precision numbers (only for debugging)
_use_precision = False


# the lambda functions below are more efficient than function definitions

if _use_precision:
    ceil = lambda value: int((value + number(1).next_minus()) // 1)
else:
    ceil = lambda value: int(math.ceil(value))

# return "0" for "-epsilon < value < 0" (to work around floating inaccuracies)
# otherwise: return the sqrt function of the current type (could even raise exceptions)
if _use_precision:
    sqrt = lambda value: (((value < -epsilon) or (value > 0)) and value.sqrt()) or 0
else:
    sqrt = lambda value: (((value < -epsilon) or (value > 0)) and math.sqrt(value)) or 0

if _use_precision:
    number = lambda value: decimal.Decimal(str(value))
else:
    number = lambda value: float(value)

