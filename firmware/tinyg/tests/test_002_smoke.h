/* 
 * test_002_smoke.h 
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char PROGMEM test_smoke[] = "\
(MSG**** Smoke Test [v1] ****)\n\
G00 G17 G21 G40 G49 G80 G90\n\
g55\n\
g0x0y0\n\
G00 x20\n\
x0\n\
y20\n\
y0\n\
z-20\n\
z0\n\
a20\n\
a0\n\
G00 x20 y20 z0 a20\n\
G00 x0 y0 z0 a0\n\
G01 f200 x7\n\
y7\n\
z-7\n\
a3\n\
g54\n\
g0x0y0\n\
m30";
