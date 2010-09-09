/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
  Part of Grbl
  Copyright (c) 2009 Simen Svale Skogsrud
*/
/* TinyG Notes:
   Modified Grbl to support Xmega family processors
   Modifications Copyright (c) 2010 Alden S. Hart, Jr.

!!!	To compile and link you must use libm.a otherwise the floating point will fail.
	In AVRstudio select Project / Configuration Options
	Select Libraries
	Move libm.a from the left pane to the right pane
	ref: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=80040&start=0

	When asked to browse for stdlib files, go to: C:\WinAVR-20100110\avr\lib\avrxmega6
	When asked to browse for include files go to: C:\WinAVR-20100110\avr\include

	Configure project - 32000000 Hz processor, and also set 32.0000 Mhz in debug configs

*/

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include "xmega_support.h"	// must precede <util/delay> and app defines
#include <util/delay.h>
#include "stepper.h"
#include "spindle_control.h"
#include "motion_control.h"
#include "encoder.h"
#include "gcode.h"
#include "serial_protocol.h"
#include "config.h"
#include "wiring_serial.h"
#include "pmic_driver.h"
#include "debug.h"

int main(void) 
{
	/* These inits are order dependent */
	cli();
	xmega_init();				// xmega setup
	config_init();				// get config record from eeprom
	config_test();
	st_init(); 					// stepper subsystem
	mc_init();					// motion control subsystem
	spindle_init();				// spindle controller
	en_init();					// encoders
	gc_init();					// gcode-parser
	sp_init();					// serial protocol

	PMIC_SetVectorLocationToApplication();  // as opposed to boot rom
//	PMIC_EnableLowLevel();		// nothing at this level
	PMIC_EnableMediumLevel(); 	// enable serial IO
	PMIC_EnableHighLevel();		// enable stepper timers
	sei();						// enable global interrupts

	for(;;){
#if real						// real mode for real men with real hardware
		sleep_mode();
		sp_process(); 			// process the serial protocol
		st_execute_line();		// run next motor move
#else
		run_gcode_from_rom(); 	// serial emulation mode - read for ROM
#endif
	}
	return 0;   /* never reached */
}
