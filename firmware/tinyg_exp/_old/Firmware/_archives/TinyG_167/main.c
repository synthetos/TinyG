/*
  main.c - TinyG - An embedded machine controller with rs274/ngc (g-code) support
  Copyright (c) 2010 Alden S. Hart Jr.

  The modules adapted from Simen Svale Skogsrud's Grbl code retain his copyright.
  Modules completely re-written carry new copyrights.

  < insert GPL language here >

****> SEE TOTO LIST AT END OF THIS FILE <****

---- In order to succecssfully compile and link you must do this... ----

	Device should have already been selected to be atxmega256a3. If not:
		In AVRstudio select Project / Configuration Options
		In main window select device atxmega256a3

	Configure clock frequency (optional, but recommended)
		In Project / Configuration Options main window:
		Frequency should be 32000000 		(32 Mhz)
	  also may want set 32.0000 Mhz in Simulator2 configs:
		Go into debug mode
		In Debug / AVR Simulator 2 Options
		Set clock frequency to 32 Mhz.

	Add libm.a (math lib) otherwise the floating point will fail.
		In AVRstudio select Project / Configuration Options
		Select Libraries
		Move libm.a from the left pane to the right pane
		ref: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=80040&start=0

	Add floating point formatting code to the linker string (for printf %f to work)
		In AVRstudio select Project / Configuration Options
		Select Custom Options
		In the left pane (Custom Compilation Options) Select [Linker Options] 
		Add the following lines to the right pane (is now linker options)
			-Wl,-u,vfprintf				(thats: W"ell" not W"one")
			-lprintf_flt
			-lm
		ref: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=92299&start=0
		ref: http://www.cs.mun.ca/~paul/cs4723/material/atmel/avr-libc-user-manual-1.6.5/group__avr__stdio.html#ga3b98c0d17b35642c0f3e4649092b9f1

  	An annoying avr20100110 bug: 
  		If you are running WinAVR-20100110 you may be asked to locate libraries or 
		include files that were known to a previous avr-gcc version. 

		When asked to browse for stdlib files, go to: 	 C:\WinAVR-20100110\avr\lib\avrxmega6
									(or is it: C:\WinAVR-20100110\lib\gcc\avr\4.3.3\avrxmega6

		When asked to browse for include files go to: C:\WinAVR-20100110\avr\include

---- Using "screen" on OSX to drive it ----

  Procedure to use the USB port from mac OSX:
	- Install the FTDI virtual serial port driver
	- Find your tty device in /dev directory, e.g.
		/dev/tty.usbserial-A700eUQo
	- Invoke screen using your tty device at 115200 baud. From terminal prompt, e.g:
		screen /dev/tty.usbserial-A700eUQo 115200

  If you are running screen (under terminal) in OSX you may want to do this first:
	in terminal, enter: "defaults write com.apple.Terminal TermCapString xterm"
						"export TERM=xterm"
  (ref: http://atomized.org/2006/05/fixing-backspace-in-screen-in-terminal-on-os-x/)

---- Coding conventions ----

  Adopted the following xmega and C variable naming conventions
  (See AVR1000: Getting Started Writing C-code for XMEGA [doc8075.pdf] )

	varname_bm		- single bit mask, e.g. 0x40 aka (1<<4)
	varname_bp		- single bit position, e.g. 4 for the above example
	varname_gm		- group bit mask, e.g. 0x0F
	varname_gc		- group configuration, e.g. 0x0A is 2 bits in the above _gm
	varname_ptr		- indicates a pointer. (but NOT array indexes)
	varname_idx		- indicates an array index (if not simply called i or j)
	varname_vect	- interrupt or other vectors

  These conventions are used for internal variables but may be relaxed for old 
  UNIX vars and DEFINES that don't follow these conventions.
*/

#include <stdio.h>			// only needed by fgets2 in xio.h
#include <avr/interrupt.h>

#include "xmega_init.h"
#include "xmega_interrupts.h"
#include "xio.h"
#include "xio_usb.h"		// TEMP for fake ISR

#include "parser.h"
#include "config.h"
#include "stepper.h"
#include "motion_control.h"
#include "spindle_control.h"
#include "encoder.h"
#include "gcode.h"


int main(void) 
{
	/* These inits are order dependent */
	cli();
	xmega_init();				// xmega setup
	xio_init();					// xmega io subsystem

	cfg_init();					// get config record from eeprom
//	cfg_test();					// we don't need no stinkin test
	st_init(); 					// stepper subsystem
	mc_init();					// motion control subsystem
	spindle_init();				// spindle controller
	en_init();					// encoders
	gc_init();					// gcode-parser
	tg_init();					// tinyg parsers

	PMIC_SetVectorLocationToApplication();  // as opposed to boot rom
	PMIC_EnableLowLevel();		// enable TX interrupts
	PMIC_EnableMediumLevel(); 	// enable RX interrupts
	PMIC_EnableHighLevel();		// enable stepper timer interrupts
	sei();						// enable global interrupts

// ability to pre-load the USB input buffer (stdin) with some characters
//	xio_usb_fake_RX_ISR('g');
//	xio_usb_fake_RX_ISR('2');
//	xio_usb_fake_RX_ISR(CTRL_C);
//	xio_usb_fake_RX_ISR('1');
	xio_usb_fake_RX_ISR('?');
	xio_usb_fake_RX_ISR('\n');

	for(;;){
		xio_usb_readln();
//		top_parser();			// get next line to process
//		st_execute_line();		// run next motion 
	}
}

/*

To Do:

Gcode interpreter
	- implement a BLOCK_DELETE function and SWITCH in cgode interpreter
	- implement a PROGRAM_STOP function and SWITCH to hit with ^c
	- learn to ignore line numbers (N's)

*/
