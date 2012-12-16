/* 
 * test_013_coordinate_offsets.h 
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char PROGMEM test_coordinate_offsets[] = "\
(MSG**** Coordinate offsets test [v1] ****)\n\
g00g17g21g40g49g80g90\n\
g92x0y0z0\n\
f600\n\
(MSG**** test G92 offsets****)\n\
g1x30\n\
g92x0\n\
g1x20\n\
g1x0y0\n\
$sr\n\
g92x0\n\
$sr\n\
g4p1\n\
(MSG**** test G54 offsets****)\n\
g10L2p1x50y50\n\
g0x0y0\n\
$sr\n\
m30";
