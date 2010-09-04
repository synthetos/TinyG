/*
 * data_gcode_asst.h - data file containing assorted tests
 */

//const char PROGMEM g0_test1[] = "g0 x10 y20 z30";

const char PROGMEM parser_test1[] = "\
N1 G00 G17 G21 G40 G49 G80 G90\n\
N2 T1 M6\n\
N3 G92 X0 Y0 Z0\n\
N20 S5000 M03 \n\
N25 G00 F30.0\n\
N30 X0.076 Y0.341";

const char PROGMEM square_test1[] = "\
g1 f333 x0 y0\n\
x1 y0\n\
x1 y1\n\
x0 y1\n\
x0 y0\n\
x1 y0\n\
x1 y1\n\
x0 y1\n\
x0 y0\n\
x1 y0\n\
x1 y1\n\
x0 y1\n\
x0 y0\n\
x1 y0\n\
x1 y1\n\
x0 y1\n\
x0 y0";

const char PROGMEM square_test2[] = "\
g1 f333 x0 y0\n\
x2 y0\n\
x2 y2\n\
x0 y2\n\
x0 y0";

//g1 f333 x10 y0
const char PROGMEM square_test10[] = "\
g1 f333 x-10 y-5\n\
x10 y10\n\
x0 y10\n\
x0 y0\n\
x5 y5";

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


const char PROGMEM spiral_test5[] = "\
g1 f300 x.5 y0\n\
g2x1Y0I0J0\n\
g2x1.5Y0I0J0\n\
g2x2Y0I0J0\n\
g2x2.5Y0I0J0\n\
g2x3Y0I0J0\n\
g2x3.5Y0I0J0\n\
g2x4Y0I0J0\n\
g2x4.5Y0I0J0\n\
g2x5Y0I0J0";

const char PROGMEM spiral_test50a[] = "\
g1 f333 x0 y0\n\
g3 x50 y50 i0 j50";

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

// radius arc test - full circle
const char PROGMEM radius_arc_test1[] = "\
g1 f300 x1 y1\n\
g2 f320 x101 y101 r50\n";

// radius arc test - full circle
const char PROGMEM radius_arc_test2[] = "\
g2\n";


