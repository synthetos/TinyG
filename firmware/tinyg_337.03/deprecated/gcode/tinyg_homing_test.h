/* 
 * tinyg_homing_test.h 
 *
 * Notes:
 *	  -	The character array should be the same name as the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char PROGMEM tinyg_homing_test[] = "\
(MSG**** Homing Test - Version 001 ****)\n\
g28.1x0y0\n\
m30";
