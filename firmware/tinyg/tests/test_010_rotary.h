/* 
 * test_010_rotary.h 
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char test_rotary[] PROGMEM = "\
(MSG**** Rotary Axis Motion Test [v1] ****)\n\
g00g17g21g40g49g80g90\n\
g0x0y0z0a0b0c0\n\
g55\n\
g0x0y0\n\
f36000\n\
g0a360\n\
b360\n\
c360\n\
a0\n\
b0\n\
c0\n\
g1a1440\n\
b1440\n\
c1440\n\
a0\n\
b0\n\
c0\n\
g1a1440b720c360\n\
x0y0z0a0b0c0\n\
g1x60a1440b720c360\n\
x0y0z0a0b0c0\n\
g1x60y50a1440b720c360\n\
x0y0z0a0b0c0\n\
g1x60y50x25a1440b720c360\n\
x0y0z0a0b0c0\n\
g54\n\
g0x0y0\n\
m30";
