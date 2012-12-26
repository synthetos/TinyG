/* 
 * test_011_small_moves.h 
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char PROGMEM test_small_moves[] = "\
(MSG**** Test very short moves [v1] ****)\n\
$si=3000\n\
g55\n\
g0x0y0\n\
g4p0.5\n\
g1x0.1f1000\n\
x0\n\
x0.1\n\
x0\n\
x0.1\n\
x0\n\
x0.1\n\
x0\n\
x0.1\n\
x0\n\
x0.1\n\
x0\n\
x0.1\n\
x0\n\
x0.1\n\
x0\n\
x0.1\n\
x0\n\
x0.1\n\
x0\n\
x0.1\n\
x0\n\
x0.1\n\
x0\n\
x0.1\n\
x0\n\
x0.1\n\
x0\n\
x0.1\n\
g4p0.5\n\
g54\n\
g0x0y0\n\
m30";
