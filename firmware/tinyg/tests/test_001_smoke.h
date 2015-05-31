/*
 * test_001_smoke.h
 *
 * This test checks basic functionality:
 *	- motor 1 CW at full speed ~3 seconds
 *	- motor 1 CCW at full speed ~3 seconds
 *	- motor 2 CW at full speed ~3 seconds
 *	- motor 2 CCW at full speed ~3 seconds
 *	- motor 3 CW at full speed ~3 seconds
 *	- motor 3 CCW at full speed ~3 seconds
 *	- motor 4 CW at full speed ~3 seconds
 *	- motor 4 CCW at full speed ~3 seconds
 *	- all motors CW at full speed ~3 seconds
 *	- all motors CCW at full speed ~3 seconds
 *	- all motors CW at medium speed ~3 seconds
 *	- all motors CCW at medium speed ~3 seconds
 *	- all motors CW at slow speed ~3 seconds
 *	- all motors CCW at slow speed ~3 seconds
 *	- light LEDs 1,2 and 4 in sequence for about 1 second each:
 *	- short finishing move
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char test_smoke[] PROGMEM = "\
(MSG**** Smoke Test [v1] ****)\n\
G00 G17 G21 G40 G49 G80 G90\n\
m3g4p1\n\
m5g4p1\n\
m4g4p1\n\
m3g4p1\n\
m5g4p1\n\
m7g4p1\n\
m9g4p1\n\
m8g4p1\n\
m9\n\
g0x0y0z0\n\
g00 x20\n\
x0\n\
y20\n\
y0\n\
z20\n\
z0\n\
a20\n\
a0\n\
G00 x20 y20 z20 a20\n\
G00 x0 y0 z0 a0\n\
G01 f30 x2 y2 z2 a2\n\
x0 y0 z0 a0\n\
g0x1\n\
g0x0\n\
m2";

/*
G01 f200 x10 y10 z10 a10\n\
x0 y0 z0 a0\n\
*/
/*
G02 f10000 x0 y0 z40 i27 j27\n\
G03 f10000 x0 y0 z0 i27 j27\n\
g0x0y0z0\n\
*/
