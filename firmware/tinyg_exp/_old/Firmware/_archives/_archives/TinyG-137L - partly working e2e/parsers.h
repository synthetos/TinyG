/*
  tg_parsers.h - tinyg top level and common parsers
  Copyright (c) 2010 Alden S. Hart, Jr.
*/
#ifndef parsers_h
#define parsers_h

void tg_init();
void tg_process();						// execute blocks from the serial port
char *tg_normalize_gcode_block(char *block);	// normalize a block of gcode in place
void tg_print_gcstatus (uint8_t status_code);	// error handling display and mop up

#endif
