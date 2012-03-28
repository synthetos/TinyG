/* 
 * tinyg_init_test.h 
 *
 * Notes:
 *	  -	The character array should be the same name as the filename (by convention)
 *	  - Comments are not allowed in the char array, but gcode comments are OK e.g. (g0 test)
 */
const char PROGMEM tinyg_init_test[] = "\
(MSG**** Iniitalization Test - Version 001 ****)\n\
g00g17g21g40g49g80g90\n\
m30";
