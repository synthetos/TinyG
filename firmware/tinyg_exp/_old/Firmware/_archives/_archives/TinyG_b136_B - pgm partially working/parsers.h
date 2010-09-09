/*
  tg_parsers.h - tinyg top level and common parsers
  Copyright (c) 2010 Alden S. Hart, Jr.
*/
#ifndef parsers_h
#define parsers_h

//void tg_read_line(char *textline);  	// generic character line reader
//void tg_read_line_P(char *pgmstring);  	// program memory line reader

void tg_init();
char *tg_normalize_gcode(char *block);	// normalize a block of gcode in place
void tg_process();						// execute blocks from the serial port
void tg_print_gcstatus (uint8_t status_code);	// Error handling display and mop up

void run_gcode_from_rom(void);

#endif
