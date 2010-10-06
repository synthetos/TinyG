/*
 * data_gcode_tests.h - data file containing assorted tests
 */

const char PROGMEM system_test[] = "\
G00 G17 G21 G40 G49 G80 G90 (initialize model)\n\
T1 M6 (set tool)\n\
G92 X0 Y0 Z0 (zero system)\n\
S5000 M03 (set spindle)\n\
G01 F200 x10 (medium)\n\
y11\n\
z13\n\
x0 y0 z0\n\
G01 F350 x20 (fast-ish)\n\
y23\n\
z27\n\
x0 y0 z0\n\
G01 F500 x20 (fast)\n\
y23\n\
z27\n\
x0 y0 z0\n\
G01 F800 x20 (too fast)\n\
y23\n\
z27\n\
x0 y0 z0\n\
g4 p0.500\n\
G0 F300 x20 (seek)\n\
y23\n\
z27\n\
x0 y0 z0\n\
g2 f100 x10 y10 i5 j5\n\
g3 f100 x0 y0 i5 j5\n\
g2 f200 x10 y10 i5 j5\n\
g3 f200 x0 y0 i5 j5\n\
g2 f300 x10 y10 i5 j5\n\
g3 f300 x0 y0 i5 j5\n\
g2 f800 x10 y10 i5 j5\n\
g3 f800 x0 y0 i5 j5\n\
g0 x0 y0 z0";

/*
const char PROGMEM system_test[] = "\
G00 G17 G21 G40 G49 G80 G90 (initialize model)\n\
T1 M6 (set tool)\n\
G92 X0 Y0 Z0 (zero system)\n\
S5000 M03 (set spindle)\n\
G01 F20 x2 (very slow)\n\
y3\n\
z4\n\
x0 y0 z0\n\
G01 F100 x10 (slow)\n\
y11\n\
z13\n\
x0 y0 z0\n\
G01 F200 x20 (medium)\n\
y23\n\
z27\n\
x0 y0 z0\n\
G01 F350 x20 (fast-ish)\n\
y23\n\
z27\n\
x0 y0 z0\n\
G01 F500 x20 (fast)\n\
y23\n\
z27\n\
x0 y0 z0\n\
G01 F800 x20 (too fast)\n\
y23\n\
z27\n\
x0 y0 z0\n\
g4 p0.500\n\
G0 F300 x20 (seek)\n\
y23\n\
z27\n\
x0 y0 z0\n\
g2 f100 x10 y10 i5 j5\n\
g3 f100 x0 y0 i5 j5\n\
g2 f200 x10 y10 i5 j5\n\
g3 f200 x0 y0 i5 j5\n\
g2 f300 x10 y10 i5 j5\n\
g3 f300 x0 y0 i5 j5\n\
g2 f800 x10 y10 i5 j5\n\
g3 f800 x0 y0 i5 j5\n\
g0 x0 y0 z0";
*/
// G1 F600 x21.45 y76.0982\n\		// test max feed rate error

const char PROGMEM square_test1[] = "\
g1 f333 x1 y0\n\
y1\n\
x0\n\
y0";

const char PROGMEM square_test2[] = "\
g1 f333 x2 y0\n\
y2\n\
x0\n\
y0";

//g1 f333 x10 y0
const char PROGMEM square_test10[] = "\
g1 f333 x10 y0\n\
y10\n\
x0\n\
y0";

const char PROGMEM circle_test10[] = "\
g1 f333 x0 y5\n\
g3 f333 x10 y5 i0 j5\n\
g3 x5 y10 i-5 j0\n\
g3 x0 y5 i0 j-5\n\
g3 x5 y0 i5 j0\n\
g1 f333 x10 y0";

const char PROGMEM square_circle_test10[] = "\
g1 f333 x10 y0\n\
x10 y10\n\
x0 y10\n\
x0 y0\n\
x5\n\
g3 x10 y5 i0 j5\n\
g3 x5 y10 i-5 j0\n\
g3 x0 y5 i0 j-5\n\
g3 x5 y0 i5 j0\n\
g1 f333 x10 y0";

const char PROGMEM square_circle_test100[] = "\
g1 f333 x100 y0\n\
x100 y100\n\
x0 y100\n\
x0 y0\n\
x50\n\
g3 x100 y50 i0 j50\n\
g3 x50 y100 i-50 j0\n\
g3 x0 y50 i0 j-50\n\
g3 x50 y0 i50 j0";

// radius arc test - full circle
const char PROGMEM radius_arc_test1[] = "\
g1 f300 x1 y1\n\
g2 f320 x101 y101 r50\n";

// radius arc test - full circle
const char PROGMEM radius_arc_test2[] = "\
g2\n";


const char PROGMEM dwell_test1[] = "\
g0 x10\n\
g4 p1\n\
g0 x20\n\
g4 p1\n\
g0 x10\n\
g4 p1\n\
g0 x00\n\
g4 p1\n\
g0 x10\n\
g4 p1\n\
g0 x20\n\
g4 p1\n\
g0 x10\n\
g4 p1\n\
g0 x0\n\
g4 p1\n\
g0 x10\n\
g4 p1\n\
g0 x20\n\
g4 p1\n\
g0 x10\n\
g4 p1\n\
g0 x0";

const char PROGMEM dwell_test2[] = "\
g0 x1\n\
g4 p1\n\
g0 x2\n\
g4 p1\n\
g0 x1\n\
g4 p1\n\
g0 x0\n\
g4 p1\n\
g0 x1\n\
g4 p1\n\
g0 x2\n\
g4 p1\n\
g0 x1\n\
g4 p1\n\
g0 x0\n\
g4 p1\n\
g0 x1\n\
g4 p1\n\
g0 x2\n\
g4 p1\n\
g0 x1\n\
g4 p1\n\
g0 x0";

const char PROGMEM dwell_testMax[] = "\
g4 p200.1";

const char PROGMEM g0_test1[] = "\
g0 x10 y20 z30\n\
g0 x0 y21 z-34.2";

const char PROGMEM g0_test2[] = "\
g0 x10 y20 z40\n\
g0 x0 y0 z0\n\
g0 x10 y20 z40\n\
g0 x0 y0 z0";

// 2 shorts and a long. Good for testing and simulation
const char PROGMEM g0_test3[] = "\
g1 f300 x1 y2 z3\n\
g1 x2 y4 z6\n\
g1 x40 y-67 z-12";


