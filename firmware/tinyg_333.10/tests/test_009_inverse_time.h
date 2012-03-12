/* 
 * test_009_inverse_time.h 
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char PROGMEM test_inverse_time[] = "\
(MSG**** Inverse Time Motion Test [v1] ****)\n\
g00g17g21g40g49g80g90\n\
g94\n\
f0.1\n\
(MSGsquares)\n\
g0x20\n\
y20\n\
x0\n\
y0\n\
g1x10\n\
y10\n\
x0\n\
y0\n\
(MSGcircles)\n\
g2x10y-10i10\n\
g0x0y0\n\
g3x10y-10i10\n\
g0x0y0\n\
g2x20y0i10\n\
g0x0y0\n\
g3x20y0i10\n\
g0x0y0\n\
g2x0y0i10\n\
g3x0y0i10\n\
m30";
