/* 
 * test_002_homing.h 
 *
 *	- Performs homing cycle in X, Y and Z
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char test_homing[] PROGMEM = "\
(MSG**** Homing Test [v1] ****)\n\
g28.2x0y0z0\n\
m30";
