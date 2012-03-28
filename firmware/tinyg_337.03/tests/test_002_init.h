/* 
 * test_002_init.h 
 *
 * Notes:
 *	  -	The character array should be derived from the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char PROGMEM test_init[] = "\
(MSG**** Iniitalization Test [v1] ****)\n\
g00g17g21g40g49g80g90\n\
m30";
