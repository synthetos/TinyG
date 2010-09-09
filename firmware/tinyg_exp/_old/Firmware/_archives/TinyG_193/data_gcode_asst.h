/*
  data_gcode_contraptor_circles.h - data file containing gcode

( Made using CamBam - http://www.cambam.co.uk )
( e-circles 4/10/2010 1:23:46 AM )
( T0 : 0.0 )
G21 
G90 
G64 
G40
G92 X0 Y0 Z0 (set zero)
G0 Z1.5
( T0 : 0.0 )
T0 M6
( Engrave1 )
G17
M3 S0
G0 X17.6075 Y35.6797
G1 F100.0 Z-0.5
G1 F320.0 X21.4068 Y35.2654
G2 X20.1819 Y32.7363 I-9.0526 J2.8233
G2 X18.0773 Y30.7072 I-6.54 J4.6773
G2 X15.1243 Y29.4444 I-4.7414 J7.0037
G2 X11.8677 Y29.0857 I-2.9605 J11.9147
G2 X7.7803 Y29.6697 I-0.3853 J11.899
G2 X4.31 Y31.6621 I2.4791 J8.3368
G2 X2.1243 Y35.0552 I6.0574 J6.3024
G2 X1.532 Y38.9227 I12.7433 J3.9306
G2 X2.1286 Y42.9079 I14.0281 J-0.063
G2 X4.3508 Y46.4175 I8.5166 J-2.9342
G2 X7.6794 Y48.45 I6.1647 J-6.3539
G2 X11.6635 Y49.084 I3.6279 J-9.9636
G2 X15.5393 Y48.4587 I0.3433 J-10.1968
G2 X18.7718 Y46.4716 I-2.8213 J-8.2124
G2 X20.9465 Y43.0285 I-6.1748 J-6.3083
G2 X21.5294 Y39.1209 I-13.2192 J-3.9692
G2 X21.509 Y38.2561 I-32.37 J0.3319
G1 X5.3313
G3 X5.8549 Y35.6831 I9.9322 J0.6816
G3 X7.3535 Y33.4277 I5.7532 J2.1971
G3 X11.8881 Y31.7522 I4.14 J4.2305
G3 X15.3402 Y32.689 I0.3404 J5.5742
G3 X16.7206 Y34.0389 I-2.9329 J4.3799
G3 X17.6075 Y35.6797 I-7.0816 J4.888
G0 Z1.5
*/

//const char PROGMEM g0_test1[] = "g0 x10 y20 z30";

const char PROGMEM square_test1[] = "\
g1 f333 x1 y0\n\
x1 y1\n\
x0 y1\n\
x0 y0\n\
x1 y1";


//g1 f333 x10 y0
const char PROGMEM square_test10[] = "\
g1 f333 x-10 y-5\n\
x10 y10\n\
x0 y10\n\
x0 y0\n\
x5 y5";

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

// contracptor circle test
const char PROGMEM contraptor_circle[] = "\
G21\n\
G90 \n\
G0 Z1.5 \n\
G17 \n\
M3 S0 \n\
G0 X17.6075 Y35.6797 \n\
G1 F100.0 Z-0.5 \n\
G1 F200.0 X21.4068 Y35.2654 \n\
G2 X20.1819 Y32.7363 I-9.0526 J2.8233 \n\
G2 X18.0773 Y30.7072 I-6.54 J4.6773 \n\
G2 X15.1243 Y29.4444 I-4.7414 J7.0037 \n\
G2 X11.8677 Y29.0857 I-2.9605 J11.9147 \n\
G2 X7.7803 Y29.6697 I-0.3853 J11.899 \n\
G2 X4.31 Y31.6621 I2.4791 J8.3368 \n\
G2 X2.1243 Y35.0552 I6.0574 J6.3024 \n\
G2 X1.532 Y38.9227 I12.7433 J3.9306 \n\
G2 X2.1286 Y42.9079 I14.0281 J-0.063 \n\
G2 X4.3508 Y46.4175 I8.5166 J-2.9342 \n\
G2 X7.6794 Y48.45 I6.1647 J-6.3539 \n\
G2 X11.6635 Y49.084 I3.6279 J-9.9636 \n\
G2 X15.5393 Y48.4587 I0.3433 J-10.1968 \n\
G2 X18.7718 Y46.4716 I-2.8213 J-8.2124 \n\
G2 X20.9465 Y43.0285 I-6.1748 J-6.3083 \n\
G2 X21.5294 Y39.1209 I-13.2192 J-3.9692 \n\
G2 X21.509 Y38.2561 I-32.37 J0.3319 \n\
G1 X5.3313 \n\
G3 X5.8549 Y35.6831 I9.9322 J0.6816 \n\
G3 X7.3535 Y33.4277 I5.7532 J2.1971 \n\
G3 X11.8881 Y31.7522 I4.14 J4.2305 \n\
G3 X15.3402 Y32.689 I0.3404 J5.5742 \n\
G3 X16.7206 Y34.0389 I-2.9329 J4.3799 \n\
G3 X17.6075 Y35.6797 I-7.0816 J4.888 \n\
G0 Z1.5";
