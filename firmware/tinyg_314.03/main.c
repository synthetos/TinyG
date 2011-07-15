/*
 * main.c - TinyG - An embedded rs274/ngc CNC controller
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */
/* See tinyg_docs.txt or general notes on the program. 
 * The comments below are about getting the project up and running 
 */
/*--- AVRstudio4 compile and link instructions and help ----

	Install WinAVR and AVRStudio 4 (if not already installed):
		run WinAVR-20100110-install.exe
		run AvrStudio4Setup.exe
		run AVRStudio4.18SP1.exe (last one before they went to Studio 5)

	Device should have already been selected to be atxmega192a3 or 
		atxmega256a3. If not:
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

	AVRstudio Custom Compilations Options
		In AVRstudio select Project / Configuration Options
		Select Custom Options
		The right pane should read:
			-Wall
			-gdwarf-2
			-std=gnu99
			-DF_CPU=32000000UL
			-O0		(or typically Os)
			-funsigned-char
			-funsigned-bitfields
			-fpack-struct
			-fshort-enums

	Add floating point formatting code to the linker string (for printf %f to work)
		In AVRstudio select Project / Configuration Options
		Select Custom Options
		In the left pane (Custom Compilation Options) Select [Linker Options] 
		Add the following lines to the right pane (is now linker options)
			-Wl,-u,vfprintf				(Wl --->thats: W"lower-case ell" not W"one")
			-lprintf_flt
			-lm
		ref: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=92299&start=0
		ref: http://www.cs.mun.ca/~paul/cs4723/material/atmel/avr-libc-user-manual-1.6.5/group__avr__stdio.html#ga3b98c0d17b35642c0f3e4649092b9f1

  	An annoying avr20100110 bug: 
  		If you are running WinAVR-20100110 you may be asked to locate 
		libraries or include files that were known to a previous avr-gcc 
		version. 

		When asked to browse for stdlib files, go to: 	 
			C:\WinAVR-20100110\avr\lib\avrxmega6
			(or is it: C:\WinAVR-20100110\lib\gcc\avr\4.3.3\avrxmega6

		When asked to browse for include files go to: 
			C:\WinAVR-20100110\avr\include
*/
/* ---- AVRstudio5 compile and link instructions and help ----

	Note: I gave up on AVRStudio5 until it's more stable. These notes were
		  compiled using the beta releases and may be dated.

	Settings in Project / TinyG Properties
		Build: generate .hex files (others are useful)

		Toolchain:
			General: unsigned defaults
			Symbols: -DF_CPU=32000000UL						  [see Note 1]
			Optimization: -Os, -fpack-struct, -fshort-enums
			Warnings: -Wall
			Miscellaneous: -gdwarf-2 -std=gnu99

		  Linker: -Wl,-lm -Wl,-u,vfprintf -mmcu=atxmega256a3 [see Note 2]
			Libraries:	libm.a
						libprintf_flt.a
			Miscellaneous: -Wl,-u,vfprintf 
			
		  Assembler (should set automatically) [see Note 2]
			-Wa,-gdwarf2 -x assembler-with-cpp -c -Wall -gdwarf-2 -std=gnu99
			-DF_CPU=32000000UL -O0 -funsigned-char -funsigned-bitfields 
			-fpack-struct -fshort-enums  -mmcu=atxmega256a3 

		Device: atxmega192a3 or atxmega256a3
		
		Debugging: AVR Simulator

	Note 1: You may also want to set the clock F to 32 Mhz in the 
			Processor window when you first start the Simulator (debugger)
			
	Note 2: You must add floating point formatting code to the linker string
			for printf %f to work.
			 -Wl,-u,vfprintf  (Wl ->thats: W"lower-case ell" not W"one")
			 -lprintf_flt	  (or add libprintf_flt.a to the libs)
			 -lm
 ref: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=92299&start=0
 ref: http://www.cs.mun.ca/~paul/cs4723/material/atmel/avr-libc-user-manual-1.6.5/group__avr__stdio.html#ga3b98c0d17b35642c0f3e4649092b9f1

*/
/*---- FLASH Programming notes ----

	If you want to preserve the EEPROM contents (config settings) during 
	reprogramming you must tell the programmer not to erase EEPROM during 
	programming. Erasing EEPROM is the default setting for AVRstudio.
*/
/*---- Using "screen" on Mac OSX to drive it ----

  Procedure to use the USB port from mac OSX:
	- Install the FTDI virtual serial port driver for your OS.
	- Find your tty device in /dev directory, e.g.
		/dev/tty.usbserial-A700eUQo
	- Invoke screen using your tty device at 115200 baud. 
	  	e.g: user$  screen /dev/tty.usbserial-A700eUQo 115200

  If you are running screen (under terminal) in OSX you may want to do 
  	this first (except it doesn't seem to have any effect when I try it):
	in terminal, enter: "defaults write com.apple.Terminal TermCapString xterm"
						"export TERM=xterm"
  (ref: http://atomized.org/2006/05/fixing-backspace-in-screen-in-terminal-on-os-x/)
*/
/*---- Coding conventions ----

  Adopted the following xmega C variable naming conventions
  (See AVR1000: Getting Started Writing C-code for XMEGA [doc8075.pdf] )

	varname_bm		- single bit mask, e.g. 0x40 aka (1<<4)
	varname_bp		- single bit position, e.g. 4 for the above example
	varname_gm		- group bit mask, e.g. 0x0F
	varname_gc		- group configuration, e.g. 0x0A is 2 bits in above _gm

  These conventions are used for internal variables but may be relaxed for
  old UNIX vars and DEFINES that don't follow these conventions.
*/
/*---- Notes on comments ----

. This code is possibly over-commented. I do this to remind 
  myself in 6 months on what I was thinking when I wrote the code. 
*/

/**************************************************************************
---- ToDo now ----
  General
  	- Add a Z kill
	- Add some keyboard jog commands

	- test cleaned up modes
	- test Gcode defaults
	- put pause and resume together, trap ^ or something else for resume
  Stability / efficiency
	- Fix bug in ! that leads to freeze during prompt: ==Tin(dead)
  Config
	- make reciprocal seek/feed rates work
	- chase down per-axis radius settings (as opposed to global a_radius)
  Serial IO and Networking
  	- Add a config for CRLF mode (other serial configs?)
	- Install some default behaviors for RS485

---- ToDo next ----
	- verify direction CW and CCW are correct throughout
	- add BC rotary axes
	- add some sort of slaved mode that stops short of slaved UVW axes
	- add M code extensions and RS485 commands
	- add individual help commands for each setting

---- ToDo later ----
	- anti-backlash code
	- [config] add profile versions
	- implement RS485 to Makerbot extruder
	- implement RS485 packet protocol
	- finish direct drive commands
	- add a PWM channel controllable like an axis (ARM only?)
	- add cornering (tightness splining algorithm)
	- build an optimized EEPROM byte writer

---- Done ----
	- Encapsulate TRAPS to prevent memory allocation if not used
*/
/* Version Notes (sparse)
---Version 311.05 - Pristine simulator state that causes a race condition 
	in the motor buffer queue where the second motor buffer to be dequeued
	during st_load_move() is actually dequeued after it has been allocated 
	("queued"), but before it has actually been populated by the 
	mq_queue_line() routine. This needs a state or some kind of mutex to 
	prevent this condition from occurring. Note: this condition only
	occurs when compiling with -O0 in AVRStudio4
 */

#include <stdio.h>				// precursor for xio.h
#include <avr/pgmspace.h>		// precursor for xio.h
#include <avr/interrupt.h>

#include "system.h"
#include "xmega_interrupts.h"
#include "xmega_rtc.h"
#include "xio.h"
//#include "xmega_init.h"
//#include "xmega_eeprom.h"

#include "tinyg.h"
#include "controller.h"
#include "config.h"
#include "stepper.h"
#include "limit_switches.h"
#include "motor_queue.h"
#include "planner.h"
#include "spindle.h"
#include "direct_drive.h"
#include "encoder.h"
#include "gcode.h"
#include "network.h"

// local function prototypes (global prototypes are in tinyg.h)
static void _tg_unit_tests(void);
static void _tg_debug_init(void);	// inits for the debug system

/*
 * Init structure
 *
 *	System startup proceeds through the following levels:
 *
 *	  tg_system_init() 			- called first (on reset) and only once
 *	  tg_application_init()		- typically only called at startup
 *	  tg_unit_tests() 			- called at startup only if unit tests enabled
 *	  tg_application_startup()	- called last; may be called again at any point
 *
 * 	The first three are managed in main.c
 *
 *	tg_application_startup() is provided by controller.c. It is used for 
 *	application starts and restarts (like for limit switches). It manages 
 *	power-on actions like homing cycles and any pre-loaded commands to the 
 *	input buffer.
 */

void tg_system_init(void)
{
	cli();					// These inits are order dependent:
	_tg_debug_init();		// (0) inits for the debug system
	hw_init();				// (1) hardware setup
	xio_init();				// (2) xmega io subsystem
	tg_init(STD_INPUT);		// (3) tinyg controller (arg is std devices)
	cfg_init();				// (4) get config record from eeprom (reqs xio)
	sig_init();				// (5) signal flags
	rtc_init();				// (6) real time counter
	sei();					// enable global interrupts
}

void tg_application_init(void) 
{
	cli();					// disable global interrupts
	st_init(); 				// stepper subsystem
	ls_init();				// limit switches
	mq_init();				// motor queues (buffers)
	mp_init();				// motion planning subsystem
	sp_init();				// spindle controller
	en_init();				// encoders
	gc_init();				// gcode-parser
	dd_init();				// direct drive commands

	PMIC_SetVectorLocationToApplication();  // as opposed to boot ROM
	PMIC_EnableLowLevel();	// enable TX interrupts
	PMIC_EnableMediumLevel();//enable RX interrupts
	PMIC_EnableHighLevel();	// enable stepper timer interrupts
	sei();					// enable global interrupts

	tg_alive();				// (LAST) announce app is online
}

static void _tg_unit_tests(void)
{
  #ifdef __UNIT_TESTS
//	xio_tests();			// IO subsystem
//	EEPROM_tests();			// EEPROM tests
//	cfg_unit_tests();		// config tests
//	mp_unit_tests();		// planner tests
//	mq_unit_tests();		// motor queue / stepper tests
  #endif
}

/*
 * MAIN
 */

int main(void)
{
	tg_system_init();
	tg_application_init();
	_tg_unit_tests();
	tg_application_startup();

#ifdef __STANDALONE_MODE
	for(;;){
		tg_controller();// this mode executes gcode blocks received via USB
	}
#endif

#ifdef __MASTER_MODE
	for(;;){
		tg_repeater();	// this mode receives on USB and repeats to RS485
	}
#endif

#ifdef __SLAVE_MODE
	for(;;){
		tg_receiver();	// this mode executes gcode blocks received via RS485
	}
#endif
}

void _tg_debug_init(void)	// inits for the debug system
{
#ifdef __dbCONFIG
	dbCONFIG = TRUE;
#else
	dbCONFIG = FALSE;
#endif

}

