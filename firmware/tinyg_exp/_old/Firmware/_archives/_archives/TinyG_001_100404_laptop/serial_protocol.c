/*
  serial_protocol.c - the serial protocol master control unit
  Part of Grbl

  Copyright (c) 2009 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
  TinyG Notes
  Modified Grbl to support Xmega family processors
  Modifications Copyright (c) 2010 Alden S. Hart, Jr.

  Added semicolon as line completion character to support Arduino serial monitor
	in sp_process()
		if((c == '\n') || (c == ';')) {  	// Line is complete. Then execute!

  If you are running screen (under terminal) in OSX you will want to do this first:
	in terminal, enter: "defaults write com.apple.Terminal TermCapString xterm"
						"export TERM=xterm"
	(ref: http://atomized.org/2006/05/fixing-backspace-in-screen-in-terminal-on-os-x/)


*/

#include <avr/io.h>
#include <math.h>
#include <avr/pgmspace.h>
#include "xmega_support.h"
#include "serial_protocol.h"
#include "gcode.h"
#include "wiring_serial.h"
#include "config.h"
#include "nuts_bolts.h"


#define LINE_BUFFER_SIZE RX_BUFFER_SIZE+1
char textline[LINE_BUFFER_SIZE];
uint8_t char_counter = 0;

void prompt() 
{
	printPgmString(PSTR("TinyG>> "));
}

void sp_init() 
{
	beginSerial(USB_BAUD_RATE);
	printPgmString(PSTR("\r\nTinyG [TEST MODE] - Version "));
	printPgmString(PSTR(TINYG_VERSION));
	printPgmString(PSTR("\r\n"));
	textline[0] = 0;		// initialize line buffer
	prompt();
}

/* sp_process() - process serial prototol */

void sp_process()
{
	char c;

	while((c = serialRead()) != 0x04) {
		if (TRUE) {							// echo mode
			printByte(c);
		}			
		if((c == '\r') || (c == '\n') || (c == ';')) {  // Line complete. Execute!
			textline[char_counter] = 0;
			printPgmString(PSTR("\r\n EXEC>> "));
			printString(textline);
			printPgmString(PSTR("\r\n"));
			gc_execute_line(textline);
			char_counter = 0;
      		prompt();
		} else if ((c == 0x08) || (c == 0x7F)) {  // backspace or delete
			textline[--char_counter] = 0;
		} else if (c <= ' ') { 				// Throw away whitespace & control chars
		} else if (c >= 'a' && c <= 'z') {	// Convert lowercase to uppercase
			textline[char_counter++] = c-'a'+'A';
		} else {
			textline[char_counter++] = c;
		}
	}
}
