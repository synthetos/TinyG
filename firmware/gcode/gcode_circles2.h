/*
 * gcode_circles2.h - data file containing gcode
 */

// circles test #2
const char PROGMEM gcode_file[] = "\
G20	(inches mode)\n\
G40 G17\n\
T1 M06\n\
M3\n\
S1000\n\
G90 G0 X0 Y0 Z0\n\
G92 G0 X0 Y0 Z0\n\
F25 (F40)\n\
G00 X0.000 Y0.000 Z0.800\n\
G00 X0.000 Y0.000 Z0.100\n\
G01 X0.000 Y0.000 Z-0.15\n\
X3.000 Y0.000\n\
X3.000 Y3.000\n\
X0.000 Y3.000\n\
X0.000 Y0.000\n\
X0.000 Y0.000 Z0.400\n\
X2.875 Y1.500\n\
X2.875 Y1.500 Z0.100\n\
G01 X2.875 Y1.500 Z-0.15\n\
G03 X2.875 Y1.500 I-1.375 J0.000\n\
G00 X2.875 Y1.500 Z0.400\n\
G00 X2.000 Y1.500\n\
G00 X2.000 Y1.500 Z0.100\n\
G01 X2.000 Y1.500 Z-0.15\n\
G03 X2.000 Y1.500 I-0.500 J0.000\n\
G00 X2.000 Y1.500 Z1.600\n\
m5\n\
M30";

