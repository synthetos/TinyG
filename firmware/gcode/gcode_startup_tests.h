/*
 * gcode_startup_tests.h - data file containing assorted tests
 */

const char PROGMEM startup_tests[] = "\
G00 G17 G21 G40 G49 G80 G90 (initialize model)\n\
G92 X0 Y0 Z0 (zero system)\n\
G00 x20\n\
y20\n\
z20\n\
a20\n\
G00 x0 y0 z0 a0\n\
G01 f100 x7\n\
y7\n\
z7\n\
a7\n\
G00 x0 y0 z0 a0\n\
G92 x0 y0";
