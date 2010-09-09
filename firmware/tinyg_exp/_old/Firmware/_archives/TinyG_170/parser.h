/*
  tg_parsers.h - tinyg top level and common parsers
  Copyright (c) 2010 Alden S. Hart, Jr.
*/
#ifndef parsers_h
#define parsers_h

/*
 * Global Scope Functions
 */

void tg_init(void);
void tg_prompt(void);
void tg_set_source(uint8_t src);
void tg_controller(void);
int top_parser(char * buf);
int top_signal(uint8_t sig);
void tg_select_file_mode();

/*
 * Defines used by TG and the parsers
 */

enum tgMode {						// used in place of a series of #defines
		TG_CONTROL_MODE,			// control mode only. No other modes active
		TG_CONFIG_MODE,				// configuration mode active
		TG_FILE_MODE,				// file mode - read from a file
		TG_DIRECT_DRIVE_MODE,		// direct drive motor mode active
		TG_GCODE_MODE,				// gcode mode active
		TG_IPA_MODE,				// International Phonetic Alphabet mode
};

enum tgSource {
		TG_NULL,					// no source selected
		TG_USB,						// USB device is line source
		TG_AUX,						// AUX device is line source (Arduino)
		TG_NET,						// network is line source (RS-485)
		TG_PGM						// lines rea from program memory file
};

#endif
