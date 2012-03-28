/* 
 * tinyg_dwell_test.h 
 *
 *	Tests a 2.5 second dwell
 *
 * Notes:
 *	  -	The character array should be the same name as the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char PROGMEM tinyg_dwell_test[] = "\
(MSG**** Dwell Test - Version 001 ****)\n\
g00g17g21g90\n\
f500\n\
g0x20\n\
g4p2.5\n\
x0\n\
m30";
