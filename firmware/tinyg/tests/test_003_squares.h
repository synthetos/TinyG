/* 
 * test_003_squares.h 
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char test_squares[] PROGMEM = "\
(MSG**** Squares Motion Test [v1] ****)\n\
g00g17g21g40g49g80g90\n\
g0x0y0\n\
g0x20 (traverse a 20mm square in X and Y)\n\
y20\n\
x0\n\
y0\n\
g1x10f500 (draw a 10mm square in X and Y at F500)\n\
y10\n\
x0\n\
y0\n\
g0z20 (traverse a 20mm square in X and Z)\n\
x20\n\
z0\n\
x0\n\
g1x20y20z20f500 (feed a 20mm diagonal in X, Y and Z)\n\
x0y0z0\n\
x40y40z40 (traverse a 40mm cube in X, Y and Z)\n\
x40y0z0\n\
x0y40z40\n\
x40y40z0\n\
x0y0z40\n\
x0y40z0\n\
x40y0z40\n\
x0y0z0\n\
g80\n\
g0x0y0\n\
g0x1\n\
g0x0\n\
m30";

/*
g0x40y40z-40 (traverse a 40mm cube in X, Y and Z)\n\
x40y0z0\n\
x0y40z-40\n\
x40y40z0\n\
x0y40z-40\n\
x0y40z0\n\
x40y0z-40\n\
x0y0z0\n\
g80\n\
m30";

x0y0z0\n\
x40y0z-40\n\
x0y40z0\n\
x40y40z-40\n\
x40y0z0\n\
x0y0z-40\n\
x40y40z0\n\
x0y40z-40\n\
x0y0z0\n\

*/
