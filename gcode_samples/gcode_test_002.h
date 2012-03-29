/*
 */
const char PROGMEM gcode_file[] = "\
N1 G21 G90 G64 G40\n\
N2 G17\n\
N3 T1 M6\n\
N4 M3 S1000\n\
N5 G92 X0 Y0 Z0\n\
N6 G1 F500 X20\n\
N7 G1 Y2\n\
N8 G1 X0\n\
N9 G1 Y4\n\
N10 G1 X20\n\
N11 G1 Y6\n\
N12 G1 X0\n\
N13 G1 Y8\n\
N14 G1 X20\n\
N15 G1 Y10\n\
N16 G1 X0\n\
N17 G1 Y12\n\
N18 G1 X20\n\
N19 G1 Y14\n\
N20 G1 X0\n\
N21 G1 Y16\n\
N22 G1 X20\n\
N23 G1 Y18\n\
N24 G1 X0\n\
N25 G1 Y20\n\
N26 G1 X0 Y0\n\
N15 M30";

