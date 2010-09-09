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

#include "xmega_support.h"		// put this first as it has the F_CPU value
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "xmega_interrupts.h"
#include "xmega_io.h"

//#include "stepper.h"
//#include "spindle_control.h"
//#include "motion_control.h"
//#include "encoder.h"
//#include "gcode.h"
//#include "serial_protocol.h"
//#include "config.h"
//#include "wiring_serial.h"
//#include "debug.h"

int main(void) 
{
	uint8_t fdusb;				// USB device file descriptor
	char rdbuf[RX_BUFSIZE];

	/* These inits are order dependent */
	cli();
	xmega_init();				// xmega setup
	xio_init();					// xmega io subsystem
//	config_init();				// get config record from eeprom
//	config_test();
//	st_init(); 					// stepper subsystem
//	mc_init();					// motion control subsystem
//	spindle_init();				// spindle controller
//	en_init();					// encoders
//	gc_init();					// gcode-parser
//	sp_init();					// serial protocol

	PMIC_SetVectorLocationToApplication();  // as opposed to boot rom
//	PMIC_EnableLowLevel();		// nothing at this level
	PMIC_EnableMediumLevel(); 	// enable serial IO
	PMIC_EnableHighLevel();		// enable stepper timers
	sei();						// enable global interrupts

	fdusb = open(DEV_USB, IO_RDWR | IO_ECHO | IO_BAUD_115200);
	write(fdusb, "USB test started\r\n", NUL_MODE);

	for(;;){
//		sleep_mode();
		read(fdusb, rdbuf, LINE_MODE);
		write(fdusb, "USB test line\r\n", NUL_MODE);
		write(fdusb, rdbuf, NUL_MODE);
//		_delay_us(1000);	 		// spin loop
		_delay_ms(50); 	 			// spin loop

//		sp_process(); 			// process the serial protocol
//		st_execute_line();		// run next motor move
//		run_gcode_from_rom(); 	// serial emulation mode - read for ROM
	}
	return 0;   /* never reached */
}
