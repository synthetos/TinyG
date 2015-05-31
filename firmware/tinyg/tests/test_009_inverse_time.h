/* 
 * test_009_inverse_time.h 
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char test_inverse_time[] PROGMEM = "\
(MSG**** Inverse Time Motion Test [v1] ****)\n\
g00g17g21g40g49g80g90\n\
g55\n\
g0x0y0\n\
g93\n\
f0.01\n\
g1x10\n\
y10\n\
z10\n\
a10\n\
x0\n\
y0\n\
z0\n\
a0\n\
g1x10y10z-10a10\n\
g0x0y0z0a0\n\
f0.1\n\
g2x10y-10z-20i10\n\
g0x0y0z0\n\
g3x10y-10z-20i10\n\
g0x0y0z0\n\
g2x20y0z-20i10\n\
g0x0y0z0\n\
g3x20y0i10\n\
g0x0y0z0\n\
g2x0y0z-30i10\n\
g3x0y0z0i10\n\
g0x0y0z0\n\
g2x0y0i20 (CW 360 degrees)\n\
g3x0y0i20 (CCW 360 degrees)\n\
g94\n\
g54\n\
g0x0y0\n\
m30";


/*
g93\n\
f0.01\n\
g1x10\n\
y10\n\
z10\n\
z10\n\
x0\n\
y0\n\
z0\n\
a0\n\
g1x10y10z10a10\n\
g0x0y0z0a0\n\
f0.1\n\
g2x10y-10z20i10\n\
g0x0y0z0\n\
g3x10y-10z20i10\n\
g0x0y0z0\n\
g2x20y0z20i10\n\
g0x0y0z0\n\
g3x20y0i10\n\
g0x0y0z0\n\
g2x0y0z30i10\n\
g3x0y0z0i10\n\
g94\n\
m30";
*/
