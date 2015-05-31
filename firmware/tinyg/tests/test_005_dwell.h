/* 
 * test_005_dwell.h 
 *
 *	Tests a 1 second dwell
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char test_dwell[] PROGMEM = "\
(MSG**** Dwell Test [v1] ****)\n\
g00g17g21g90\n\
g55\n\
g0x0y0\n\
f500\n\
g0x10\n\
g4p1\n\
g0x20\n\
g4p1\n\
g0x10\n\
g4p1\n\
g0x00\n\
g4p1\n\
y5\n\
g54\n\
g0x0y0\n\
m3\n\
g4p1\n\
g0x10y10\n\
m5\n\
g4p1\n\
g0x20y20\n\
m4\n\
g4p1\n\
g0x30y30\n\
m8\n\
g4p1\n\
g0x40y40\n\
m9\n\
g4p1\n\
g0x50y50\n\
g0x0y0\n\
m30";
