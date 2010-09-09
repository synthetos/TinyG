/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
  Part of Grbl

  Copyright (c) 2009 Simen Svale Skogsrud

  // NOTES by Alden Hart

---	In order to link you must use libm.a otherwise the floating point will fail.
	ref: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=80040&start=0
	Right click a source file in the AVR GCC window and select "Edit Configuration Options"
	Select "Libraries"
	Move libm.a from the left pane to the right pane

---	It's useful to add the following line to keep <util/delay.h> from complaining:
	#define F_CPU 1000000UL			// **** - alden 3/30/10 ****

--- The EEPROM issue. It's totally different in xmegas
	ref: http://old.nabble.com/xmega-support-td21322852.html
		 http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=84542&start=0

	Need to:
	- leave eeprom.h in place with it's 4 function prototypes
	- add eeprom_driver.c &.h to the project
	- remove eeprom.c from the project 
		- write a compatibility file: eeprom_xmega.c implementing those 4 functions
		- use eeprom_driver.c as the base routines
	- alternately wait for AVR GCC to get memory mapped eeprom IO working (i.e. address 0x1000)

---	Stuff to do:
	- change config.h #defines to split out dir and step bits across multiple ports. 
		- will have ramifications in other parts of the code, I'm sure

*/

#include <avr/io.h>
#include <avr/sleep.h>
#define F_CPU 32000000UL			// **** added
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
  
///  DDRD |= (1<<3)|(1<<4)|(1<<5);
  PORTD.DIR |= (1<<3)|(1<<4)|(1<<5);

  for(;;){
    sleep_mode();
    sp_process(); // process the serial protocol
  }
  return 0;   /* never reached */
}
