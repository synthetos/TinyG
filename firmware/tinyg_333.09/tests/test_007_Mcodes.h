/* 
 * test_007_Mcodes.h 
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char PROGMEM test_Mcodes[] = "\
(MSG**** Mcodes test [v1] ****)\n\
m3\n\
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
m9\n";

/*
m3\n\
g4p0.01\n\
m4\n\
g4p0.01\n\
m5\n\
g4p0.01\n\
m7\n\
g4p0.01\n\
m9\n\
g4p0.01\n\
m8\n\
g4p0.01\n\
m9\n";

*/
/*
m3\n\
g4p1\n\
g0x10\n\
m4\n\
g4p1\n\
g0x20\n\
m5\n\
g4p1\n\
g0x30\n\
m7\n\
g4p1\n\
g0x40\n\
m9\n\
g4p1\n\
g0x50\n\
m8\n\
g4p1\n\
g0x60\n\
m9\n\
g4p1";

*/
