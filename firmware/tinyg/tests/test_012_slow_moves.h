/* 
 * test_012_slow_moves.h 
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
$si=3000\n\
g1y5\n\
x0\n\
y0";
 */

const char PROGMEM test_slow_moves[] = "\
(MSG**** Test accuracy of slow moves [v1] ****)\n\
g1z-5f22";
