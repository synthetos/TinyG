/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
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

// NOTES by Alden Hart

	In order to link you must use libm.a otherwise the floating point will fail.
	ref: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=80040&start=0
	Right click a source file in the AVR GCC window and select "Edit Configuration Options"
	Select "Libraries"
	Move libm.a from the left pane to the right pane

	It's useful to add the following line to keep <util/dleay.h> from complaining:
	#define F_CPU 1000000UL			// **** - alden 3/30/10 ****

*/

#include <avr/io.h>
#include <avr/sleep.h>
#define F_CPU 1000000UL			// **** added - alden 3/30/10 ****
#include <util/delay.h>
#include "stepper.h"
#include "spindle_control.h"
#include "motion_control.h"
#include "gcode.h"
#include "serial_protocol.h"

#include "config.h"
#include "wiring_serial.h"

int main(void)
{
  beginSerial(BAUD_RATE);
  config_init();
  st_init(); // initialize the stepper subsystem
  mc_init(); // initialize motion control subsystem
  spindle_init(); // initialize spindle controller
  gc_init(); // initialize gcode-parser
  sp_init(); // initialize the serial protocol
  
  DDRD |= (1<<3)|(1<<4)|(1<<5);
  
  for(;;){
    sleep_mode();
    sp_process(); // process the serial protocol
  }
  return 0;   /* never reached */
}
