/*
  serial_protocol.h - the serial protocol master control unit
  Part of Grbl

  Copyright (c) 2009 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify it under the terms
  of the GNU General Public License as published by the Free Software Foundation, 
  either version 3 of the License, or (at your option) any later version.

  Grbl is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
  PURPOSE. See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License along with Grbl.  
  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef serial_h
#define serial_h

#define PROMPT "\r\n>>>"		// let the client know we are ready for a new command
#define EXECUTION_MARKER '~'	// char to acknowledge that the execution has started

void sp_init();
void sp_process();			// read command lines from the serial port and execute 
							// as they come in. Blocks until serial buffer is emptied
void run_gcode_from_rom(void);
void sp_print_gcstatus (uint8_t status_code);	// Error handling display and mop up

#endif
