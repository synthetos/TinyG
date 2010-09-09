/*
  tg_parsers.h - tinyg top level and common parsers
  Copyright (c) 2010 Alden S. Hart, Jr.
*/
#ifndef parsers_h
#define parsers_h

#define PROMPT "\r\n>>>"		// let the client know we are ready for a new command
//#define EXECUTION_MARKER '~'	// char to acknowledge that the execution has started

void tg_init();
void tg_read_line(char *textline);  	// generic character line reader
void tg_read_line_P(char *pgmstring);  	// program memory line reader

void tg_process();			// read command lines from the serial port and execute 
							// as they come in. Blocks until serial buffer is emptied

void tg_print_gcstatus (uint8_t status_code);	// Error handling display and mop up

void run_gcode_from_rom(void);

#endif
