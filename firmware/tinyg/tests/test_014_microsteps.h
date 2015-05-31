/*
 * test_014_coordinate_offsets.h
 *
 * Tests movement in 4 axes with all microatep settings. All moves should be the same length and time
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char test_microsteps[] PROGMEM = "\
(MSG**** Microstep Test [v1] ****)\n\
G00 G17 G21 G40 G49 G80 G90\n\
g0x0y0z0\n\
{\"1mi\":1}\n\
{\"2mi\":1}\n\
{\"3mi\":1}\n\
{\"4mi\":1}\n\
g0 x10\n\
{\"1mi\":2}\n\
x0\n\
{\"1mi\":4}\n\
x10\n\
{\"1mi\":8}\n\
x0\n\
g0 y10\n\
{\"2mi\":2}\n\
y0\n\
{\"2mi\":4}\n\
y10\n\
{\"2mi\":8}\n\
y0\n\
g0 z10\n\
{\"3mi\":2}\n\
z0\n\
{\"3mi\":4}\n\
z10\n\
{\"3mi\":8}\n\
z0\n\
g0 a10\n\
{\"4mi\":2}\n\
a0\n\
{\"4mi\":4}\n\
a10\n\
{\"4mi\":8}\n\
a0\n\
m2";

