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
g2x0y0i20\n\
g3x0y0i20\n\
g80\n\
m30";

