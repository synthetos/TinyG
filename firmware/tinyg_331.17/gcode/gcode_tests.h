/*
 * data_gcode_tests.h - data file containing assorted tests
 */

/* trajectory planner cases */
const char PROGMEM gcode_file[] = "\
G00 G17 G21 G40 G49 G80 G90 (initialize model)\n\
T1 M6 (set tool)\n\
G92 X0 Y0 Z0 (zero system)\n\
S5000 M03 (set spindle)\n\
(jerk set to 25000000)\n\
G01 F500 x20 (acceleration)\n\
G01 F200 y20 (deceleration)\n\
G01 F300 x0 (acceleration)\n\
G0 y0 (acceleration to cruise rate)\n\
G02 F400 x20 y20 i10 j10\n\
G02 F300 x0 y0 i10 j10\n\
G0 x10 y10";

/*
G01 F500 x80 (acceleration)\n\
G01 F200 x82.1 (deceleration too-short)\n\
G01 F100 x10 (deceleration continuous short)\n\
G01 F500 y10 (exact path short)\n\
G01 F500 y-10 (exact stop short)\n\
g0 x0 y0 z0\n\
G02 F400 x10 y10 i5 j5\n\
G02 F300 x0 y0 i5 j5\n\
G02 F200 x10 y10 i5 j5\n\
g0 x0 y0 z0";

G64 (continuous mode)\n\


G01 F100 x10 (from stop)\n\
g0 x13 y-10 z11\n\

g0 x10 y-10 z0\n\
g0 x0 y0 z0";

g0 x0 y0 z0";
*/

/* straight feed tests */
const char PROGMEM straight_feed_test[] = "\
G0 G17 G21 G40 G49 G80 G90 (initialize model)\n\
g92x0y0z0a0\n\
(-----G0 tests-----)\n\
G0 x50 y50 z50 a50 (G0 all axes)\n\
x0 y0 z0 a0 (G0 all axes - return)\n\
G4 P1\n\
g1 f400 x50 (test axes one at a time)\n\
y50\n\
z50\n\
a50\n\
()\n\
(-----G1 tests-----)\n\
()\n\
g92x0y0z0a0\n\
G4 P1\n\
G1 F100 x10 y10 z10 a10 (F100 all axes)\n\
g92x0y0z0a0\n\
G4 P1\n\
G1 F200 x20 y20 z20 a20 (F200 all axes)\n\
g92x0y0z0a0\n\
G4 P1\n\
G1 F400 x40 y40 z40 a40 (F400 all axes)\n\
g92x0y0z0a0\n\
G4 P1\n\
G1 F600 x50 y50 z50 a50 (F600 all axes)\n\
g92x0y0z0a0\n\
G4 P1\n\
G1 F800 x50 y50 z50 a50 (F800 all axes)\n\
g92x0y0z0a0\n\
G4 P1\n\
G1 F10000 x50 y50 z50 a50 (max all axes)\n\
g92x0y0z0a0\n\
()\n\
(-----Arc tests-----)\n\
()\n\
g2 f100 x10 y10 i5 j5\n\
g3 f100 x0 y0 i5 j5\n\
g2 f200 x10 y10 i5 j5\n\
g3 f200 x0 y0 i5 j5\n\
g2 f300 x10 y10 i5 j5\n\
g3 f300 x0 y0 i5 j5\n\
g2 f800 x10 y10 i5 j5\n\
g3 f800 x0 y0 i5 j5\n\
g0 x0 y0 z0";

const char PROGMEM arc_feed_test[] = "\
G0 G17 G21 G40 G49 G80 G90 (initialize model)\n\
g92x0y0z0a0\n\
(-----Arc tests-----)\n\
()\n\
g2 f100 x10 y10 i5 j5\n\
g3 f100 x0 y0 i5 j5\n\
g2 f200 x10 y10 i5 j5\n\
g3 f200 x0 y0 i5 j5\n\
g2 f300 x10 y10 i5 j5\n\
g3 f300 x0 y0 i5 j5\n\
g2 f800 x10 y10 i5 j5\n\
g3 f800 x0 y0 i5 j5\n\
g0 x0 y0 z0";


const char PROGMEM straight_feed_test2[] = "\
G0 G17 G21 G40 G49 G80 G90 (initialize model)\n\
g92x0y0z0a0\n\
g0 f400 x50y50z50 (linear axes below rate limit)\n\
g92x0y0z0a0\n\
g0 f4000 x50y50z50 (linear axes rate limited)\n\
g92x0y0z0a0\n\
g0 f400 a10 (A axis below rate limit)\n\
g92x0y0z0a0\n\
G0 f400 a1000 (A axis rate limited)\n\
g92x0y0z0a0\n\
G0 F400 x50 (all axes below rate limit)\n\
y50\n\
z50\n\
a50\n\
g92x0y0z0a0\n\
G0 F400 x50 (A axis is rate limited)\n\
y50\n\
z50\n\
a3600\n\
g92x0y0z0a0\n\
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


const char PROGMEM system_test01[] = "\
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

const char PROGMEM system_test01a[] = "\
G00 G17 G21 G40 G49 G80 G90 (initialize model)\n\
T1 M6 (set tool)\n\
G92 X0 Y0 Z0 (zero system)\n\
S5000 M03 (set spindle)\n\
G01 F350 x1.0(fast-ish)\n\
y0.8\n\
z0.2\n\
x0 y0 z0\n\
G01 F350 x0.8(fast-ish)\n\
y0.5\n\
z0.1\n\
x0 y0 z0\n\
G01 F500 x2.0 (fast)\n\
y2.3\n\
z2.7\n\
x0 y0 z0\n\
G01 F800 x2.0 (too fast)\n\
y2.3\n\
z2.7\n\
x0 y0 z0\n\
g4 p0.500\n\
G0 F300 x2.0 (seek)\n\
y2.3\n\
z2.7\n\
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

const char PROGMEM system_test02[] = "\
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

const char PROGMEM system_test03[] = "\
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
x0 y0 z0\n";

//G00 x10 (fast)\n

const char PROGMEM system_test04[] = "\
G00 G17 G21 G40 G49 G80 G90 (initialize model)\n\
T1 M6 (set tool)\n\
G92 X0 Y0 Z0 (zero system)\n\
S5000 M03 (set spindle)\n\
G61\n\
G01 F400 x10 (fast)\n\
y10\n\
z10\n\
x-10\n\
y-10\n\
z-10\n\
x6\n\
y6\n\
z6\n\
x-6\n\
y-6\n\
z-6\n\
x4\n\
y4\n\
z4\n\
x-4\n\
y-4\n\
z-4\n\
x2\n\
y2\n\
z2\n\
x-2\n\
y-2\n\
z-2\n\
x1\n\
y1\n\
z1\n\
x-1\n\
y-1\n\
z-1\n\
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


