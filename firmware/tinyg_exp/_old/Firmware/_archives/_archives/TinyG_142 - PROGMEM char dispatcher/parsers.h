/*
  tg_parsers.h - tinyg top level and common parsers
  Copyright (c) 2010 Alden S. Hart, Jr.
*/
#ifndef parsers_h
#define parsers_h

void tg_init();
void tg_prompt();
void top_parser();					// collect and dispatch serial input

/*
 * Defines used by TG and the parsers
 */

#define TG_CONTROL_MODE 0			// control mode only. No otyher modes active
#define TG_CONFIG_MODE 1			// configuration mode active
#define TG_FILE_MODE 2				// file mode - read from a file
#define TG_GCODE_MODE 3				// gcode mode active
#define TG_DIRECT_DRIVE_MODE 4		// direct drive motor mode active
#define TG_IPA_MODE 5				// International Phonetic Alphabet mode

#endif
