/* 
 * test_004_dwell.h 
 *
 *	Tests a 1 second dwell
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char PROGMEM test_dwell[] = "\
(MSG**** Dwell Test [v1] ****)\n\
g00g17g21g90\n\
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
m30";
