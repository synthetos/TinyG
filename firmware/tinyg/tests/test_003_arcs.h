/* 
 * test_003_arcs.h 
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char PROGMEM test_arcs[] = "\
(MSG**** Arc Test [v1] ****)\n\
g00g17g21g40g49g80g90\n\
g92x0y0\n\
f500\n\
g2x10y-10i10 (CW 270 degrees)\n\
g0x0y0\n\
g3x10y-10i10 (CCW 90 degrees)\n\
g0x0y0\n\
g2x20y0i10 (CW 180 degrees)\n\
g0x0y0\n\
g3x20y0i10 (CCW 180 degrees)\n\
g0x0y0\n\
g2x0y0i20 (CW 360 degrees)\n\
g2x0y0i20 (CW 360 again)\n\
f700 (change feed rate while in G2 motion mode)\n\
g2x0y0i20 (CW 360 3rd time)\n\
g2x0y0i20 (CW 360 4th time)\n\
f500\n\
g3x0y0i20 (CCW 360 degrees)\n\
g3x0y0i20 (CCW 360 again)\n\
f700 (change feed rate while in G3 motion mode)\n\
g3x0y0i20 (CCW 360 3rd time)\n\
g3x0y0i20 (CCW 360 4th time)\n\
g80\n\
m30";
