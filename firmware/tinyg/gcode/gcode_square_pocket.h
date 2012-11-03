/*
 */
const char PROGMEM gcode_file[] = "\
N1 G20 G90 G40\n\
N2 (pocket 1)\n\
N3 G0 Z0.25\n\
N4 G17\n\
N5 G1 X1 F55\n\
N6 G1 Z0 F15\n\
N7 G1 Z-0.0625 F15\n\
N8 G1 Y1\n\
N9 G1 X0.9375\n\
N10 G1 Y0\n\
N11 G1 X0.875\n\
N12 G1 Y1\n\
N13 G1 X0.8125\n\
N14 G1 Y0\n\
N15 G1 X0.8125\n\
N16 G1 Y1\n\
N17 G1 X0.6875\n\
N18 G1 Y0\n\
N19 G1 X0.625\n\
N20 G1 Y1\n\
N21 G1 X0.5625\n\
N22 G1 Y0\n\
N23 G1 X0.5\n\
N24 G1 Y1\n\
N25 G1 X0.4375\n\
N26 G1 Y0\n\
N27 G1 X0.375\n\
N28 G1 Y1\n\
N29 G1 X0.3125\n\
N30 G1 Y0\n\
N31 G1 X0.25\n\
N32 G1 Y1\n\
N33 G1 X0.1875\n\
N34 G1 Y0\n\
N35 G1 X0.125\n\
N36 G1 Y1\n\
N37 G1 X0.0625\n\
N38 G1 Y0\n\
N39 G1 X0\n\
N40 G1 Y1\n\
N41 G0 Z0.25\n\
N42 G0 X0 Y0\n\
N43 M5\n\
N15 M30";
