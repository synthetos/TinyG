/* 
 * test_007_Mcodes.h 
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 *
 * Turn the bits on and off in sequence so you can see the LEDs light in a chain 
 */
const char test_Mcodes[] PROGMEM = "\
(MSG**** Mcodes test [v1] ****)\n\
g55\n\
g0x0y0\n\
m3\n\
g4p1\n\
m5\n\
g4p1\n\
m4\n\
g4p1\n\
m5\n\
g4p1\n\
m7\n\
g4p1\n\
m9\n\
g4p1\n\
m8\n\
g4p1\n\
m9\n\
f500\n\
g0x20\n\
y20\n\
m3\n\
x0\n\
m4\n\
y0\n\
m5\n\
g1x10\n\
m7\n\
y10\n\
m8\n\
x0\n\
m9\n\
y0\n\
g2x10y-10i10\n\
m3\n\
g0x0y0\n\
m4\n\
g3x10y-10i10\n\
m5\n\
g0x0y0\n\
m7\n\
g2x20y0i10\n\
m8\n\
g0x0y0\n\
m9\n\
g3x20y0i10\n\
g0x0y0\n\
g2x0y0i10\n\
g3x0y0i10\n\
g54\n\
g0x0y0\n\
m30";
