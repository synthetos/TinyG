/*
 * test_013_coordinate_offsets.h
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char test_coordinate_offsets[] PROGMEM = "\
(MSG**** Coordinate offsets test [v1] ****)\n\
g00g17g21g40g49g80g90\n\
g54\n\
g92x0y0z0\n\
f600\n\
(MSG**** test G92 offsets****)\n\
(msgStep 1: Move from 0,0 to 30,0 mm in positive X direction)\n\
(msgStep 2: Reset origin to 0.0 using G92)\n\
(msgStep 3: Move to 20,0. Should move 20 mm in positive X direction and status report should start from 0,0)\n\
(msgStep 4: Move to 0.0   Should move 20 mm in negative X direction and status report should start from 20,0)\n\
g1x30\n\
g92x0\n\
g1x20\n\
g1x0y0\n\
$sr\n\
g92x0\n\
g4p0.5\n\
(MSG**** test G55 offsets****)\n\
(msgStep 1: Set Coordinate system 2 [g55] to offset of 50,50)\n\
(msgStep 2: Select G55 coordinate system)\n\
(msgStep 3: Move to 0,0. Head should move diagonally in +X and +Y and status report should start from 0,0)\n\
g10L2p2x50y50\n\
g55\n\
g0x0y0\n\
g4p0.5\n\
(MSG**** test G53 absolute overrides ****)\n\
(msgStep 1: Run a square in G55 system)\n\
(msgStep 2: Run a square w/G53 override)\n\
g0x0y0\n\
g0x20y0\n\
g0x20y20\n\
g0x0y20\n\
g0x0y0\n\
g4p0.5\n\
g53g0x0y0\n\
g53g0x20y0\n\
g53g0x20y20\n\
g53g0x0y20\n\
g53g0x0y0\n\
g0x0y0\n\
g4p0.5\n\
(MSG**** test Return to home ****)\n\
(msgStep 1: Return to home using G28)\n\
(msgStep 2: Reset to G54 coord system)\n\
g28\n\
g54\n\
m30";
