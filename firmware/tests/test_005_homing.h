/* 
 * test_005_homing.h 
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char PROGMEM test_homing[] = "\
(MSG**** Homing Test [v1] ****)\n\
g28.1x0y0z0\n\
m30";
