/*
 *	TinyG_test001 - general purpose test routine for TinyG board
 *
 *	Written by: Alden Hart
 *	Revision: 07/27/10
 *
 *	This program tests the following:
 *		- 3.3v supply is working and powering the CPU
 *		- The CPU is alive and running on the internal 32 MHz clock
 *		- The output port LEDS are working
 *
 *	Additionally tests:
 *		- USB port is sending and receiving (at 115,200 baud)
 *		- step motors at 1000 steps per second
 *
 *
---- In order to compile and link in AVRstudio you must do this... ----

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

---- Using "screen" on Mac OSX to drive it ----

  Procedure to use the USB port from mac OSX:
	- Install the FTDI virtual serial port driver for your OS.
	- Find your tty device in /dev directory, e.g.
		/dev/tty.usbserial-A700eUQo
	- Invoke screen using your tty device at 115200 baud. From terminal prompt, e.g:
		screen /dev/tty.usbserial-A700eUQo 115200

  If you are running screen (under terminal) in OSX you may want to do this first:
	in terminal, enter: "defaults write com.apple.Terminal TermCapString xterm"
						"export TERM=xterm"
  (ref: http://atomized.org/2006/05/fixing-backspace-in-screen-in-terminal-on-os-x/)
*/

#include <stdio.h>			// only needed by fgets2 in xio.h
#include <avr/interrupt.h>

#include "xmega_init.h"
#include "xmega_interrupts.h"
#include "xio.h"
#include "xio_usb.h"
#include "xio_pgm.h"

#include "tinyg.h"
#include "config.h"
#include "stepper.h"
#include "encoder.h"
#include "hardware.h"

#include <util/delay.h>			// must follow hardware.h (F_CPU setting)

int main(void) 
{
	/* These inits are order dependent */
	cli();
	xmega_init();				// xmega setup
	xio_init();					// xmega io subsystem

	cfg_init();					// get config record from eeprom
	st_init(); 					// stepper subsystem

	PMIC_SetVectorLocationToApplication();  // as opposed to boot rom
	PMIC_EnableLowLevel();		// enable TX interrupts
	PMIC_EnableMediumLevel(); 	// enable RX interrupts
	PMIC_EnableHighLevel();		// enable stepper timer interrupts
	sei();						// enable global interrupts

	for(;;){
		en_write(0x00);
		_delay_ms(250);
		en_write(0xFF);
		_delay_ms(250);
	}
}
