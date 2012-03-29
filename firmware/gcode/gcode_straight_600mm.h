/* 
 * gcode_straight_600mm.h - data file containing gcode
 */

/*
const char PROGMEM gcode_file[] = "\
N1 T1M6\n\
N2 G17\n\
N3 G21 (mm)\n\
N4 G92X0 Y0 Z0\n\
N5 F800.0\n\
N6 G1 x0 y0\n\
N7 G1 x0.3 y0.3\n\
N8 G1 x0.6 y0.6\n\
N9 G1 x0.9 y0.9\n\
N10 G1 x1.2 y1.2\n\
/N11 G1 x1.5 y1.5\n\
/N12 G1 x1.8 y1.8\n\
/N13 G1 x2.1 y2.1\n\
/N14 G1 x2.4 y2.4\n\
/N15 G1 x2.7 y2.7\n\
/N16 G1 x3.0 y3.0\n\
/N17 G1 x3.3 y3.3\n\
/N18 G1 x3.6 y3.6\n\
/N19 G1 x3.9 y3.9\n\
/N20 G1 x4.2 y4.2\n\
/N21 G1 x4.5 y4.5\n\
/N22 G1 x4.8 y4.8\n\
/N23 G1 x5.1 y5.1\n\
/N24 G1 x5.4 y5.4\n\
/N25 G1 x5.7 y5.7\n\
/N26 G1 x6.0 y6.0";
*/


const char PROGMEM gcode_file[] = "\
N1 T1M6\n\
N2 G17\n\
N3 G21 (mm)\n\
N4 G92X0 Y0 Z0\n\
N5 F800.0\n\
N6 G1 x0 y0\n\
N7 G1 x0.3 y0.3\n\
N8 G1 x0.6 y0.6\n\
N9 G1 x0.9 y0.9\n\
N10 G1 x1.2 y1.2\n\
N11 G1 x1.5 y1.5\n\
N12 G1 x1.8 y1.8\n\
N13 G1 x2.1 y2.1\n\
N14 G1 x2.4 y2.4\n\
N15 G1 x2.7 y2.7\n\
N16 G1 x3.0 y3.0\n\
N17 G1 x3.3 y3.3\n\
N18 G1 x3.6 y3.6\n\
N19 G1 x3.9 y3.9\n\
N20 G1 x4.2 y4.2\n\
N21 G1 x4.5 y4.5\n\
N22 G1 x4.8 y4.8\n\
N23 G1 x5.1 y5.1\n\
N24 G1 x5.4 y5.4\n\
N25 G1 x5.7 y5.7\n\
N26 G1 x6.0 y6.0\n\
N27 G1 x6.3 y6.3\n\
N28 G1 x6.6 y6.6\n\
N29 G1 x6.9 y6.9\n\
N30 G1 x7.2 y7.2\n\
N31 G1 x7.5 y7.5\n\
N32 G1 x7.8 y7.8\n\
N33 G1 x8.1 y8.1\n\
N34 G1 x8.4 y8.4\n\
N35 G1 x8.7 y8.7\n\
N36 G1 x9.0 y9.0\n\
N37 G1 x9.3 y9.3\n\
N38 G1 x9.6 y9.6\n\
N39 G1 x9.9 y9.9\n\
N40 G1 x10.2 y10.2\n\
N41 G1 x10.5 y10.5\n\
N42 G1 x10.8 y10.8\n\
N43 G1 x11.1 y11.1\n\
N44 G1 x11.4 y11.4\n\
N45 G1 x11.7 y11.7\n\
N46 G1 x12.0 y12.0\n\
N47 G1 x12.3 y12.3\n\
N48 G1 x12.6 y12.6\n\
N49 G1 x12.9 y12.9\n\
N50 G1 x13.2 y13.2\n\
N51 G1 x13.5 y13.5\n\
N52 G1 x13.8 y13.8\n\
N53 G1 x14.1 y14.1\n\
N54 G1 x14.4 y14.4\n\
N55 G1 x14.7 y14.7\n\
N56 G1 x15.0 y15.0\n\
N57 G1 x15.3 y15.3\n\
N58 G1 x15.6 y15.6\n\
N59 G1 x15.9 y15.9\n\
N60 G1 x16.2 y16.2\n\
N61 G1 x16.5 y16.5\n\
N62 G1 x16.8 y16.8\n\
N63 G1 x17.1 y17.1\n\
N64 G1 x17.4 y17.4\n\
N65 G1 x17.7 y17.7\n\
N66 G1 x18.0 y18.0\n\
N67 G1 x18.3 y18.3\n\
N68 G1 x18.6 y18.6\n\
N69 G1 x18.9 y18.9\n\
N70 G1 x19.2 y19.2\n\
N71 G1 x19.5 y19.5\n\
N72 G1 x19.8 y19.8\n\
N73 G1 x20.1 y20.1\n\
N74 G1 x20.4 y20.4\n\
N75 G1 x20.7 y20.7\n\
N76 G1 x21.0 y21.0\n\
N77 G1 x21.3 y21.3\n\
N78 G1 x21.6 y21.6\n\
N79 G1 x21.9 y21.9\n\
N80 G1 x22.2 y22.2\n\
N81 G1 x22.5 y22.5\n\
N82 G1 x22.8 y22.8\n\
N83 G1 x23.1 y23.1\n\
N84 G1 x23.4 y23.4\n\
N85 G1 x23.7 y23.7\n\
N86 G1 x24.0 y24.0\n\
N87 G1 x24.3 y24.3\n\
N88 G1 x24.6 y24.6\n\
N89 G1 x24.9 y24.9\n\
N90 G1 x25.2 y25.2\n\
N91 G1 x25.5 y25.5\n\
N92 G1 x25.8 y25.8\n\
N93 G1 x26.1 y26.1\n\
N94 G1 x26.4 y26.4\n\
N95 G1 x26.7 y26.7\n\
N96 G1 x27.0 y27.0\n\
N97 G1 x27.3 y27.3\n\
N98 G1 x27.6 y27.6\n\
N99 G1 x27.9 y27.9\n\
N100 G1 x28.2 y28.2\n\
N101 G1 x28.5 y28.5\n\
N102 G1 x28.8 y28.8\n\
N103 G1 x29.1 y29.1\n\
N104 G1 x29.4 y29.4\n\
N105 G1 x29.7 y29.7\n\
N106 G1 x0 y0";

