/* 
 * test_004_arcs.h 
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char PROGMEM test_arcs[] = "\
/(msg**** Arc Test [v1] ****)\n\
N1 g00g17g21g40g49g80g90\n\
N10 g0x0y0\n\
N20 f500\n\
N30 g2x10y-10i10 (CW 270 degrees)\n\
N40 g0x0y0\n\
N50 g3x10y-10i10 (CCW 90 degrees)\n\
N60 g0x0y0\n\
N80 g2x20y0i10 (CW 180 degrees)\n\
N70 g0x0y0\n\
N80 g3x20y0i10 (CCW 180 degrees)\n\
N90 g0x0y0\n\
N99 F1200\n\
N100 g2x0y0i20 (CW 360 degrees)\n\
N110 g2x0y0i20 (CW 360 again)\n\
N110 g2x0y0i20 (CW 360 again)\n\
N110 g2x0y0i20 (CW 360 again)\n\
N110 g2x0y0i20 (CW 360 again)\n\
N110 g2x0y0i20 (CW 360 again)\n\
N110 g2x0y0i20 (CW 360 again)\n\
N120 f700 (msgChange feed rate while in G2 motion mode)\n\
N130 g2x0y0i20 (CW 360 3rd time)\n\
N140 g2x0y0i20 (CW 360 4th time)\n\
N150 f500\n\
N160 g3x0y0z40i20 (CCW 360 degrees with linear travel)\n\
N170 g3x0y0z0i20 (CCW 360 again)\n\
N180 f700 (msg****Change feed rate while in G3 motion mode****)\n\
N190 g3x0y0i20 (CCW 360 3rd time)\n\
N200 g3x0y0i20 (CCW 360 4th time)\n\
N210 (msg****G18 Eval test****)\n\
N220 G1X30.707Z50.727F500\n\
N230 G18 G02X67.738Z23.617R25F250\n\
N240 g80\n\
N250 g0x0y0\n\
N270 m30";

/*
g92x0y0\n\
g55\n\
N250 g54\n\

*/
