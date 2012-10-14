/*
 * main.c - TinyG - An embedded rs274/ngc CNC controller
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for details. You should have received a copy of the GNU General Public 
 * License along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* See tinyg_docs.txt for general notes. 
 * See tinyg_docs_developers.txt for coding details, and how to set up this project in AVRstudio
 * See also: http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Developer-Info:
 * 	(the trailing ':' is required!)
 *
 *---- Notes on comments ----
.* Yes, this code is probably over-commented. I do this to remind 
 * myself in 6 months on what I was thinking - however unlikely
 */

/**************************************************************************/

#include <stdio.h>				// precursor for xio.h
#include <avr/pgmspace.h>		// precursor for xio.h
#include <avr/interrupt.h>

#include "xmega/xmega_interrupts.h"
//#include "xmega/xmega_eeprom.h"	// uncomment for unit tests
#include "xmega/xmega_rtc.h"
#include "xio/xio.h"

#include "tinyg.h"				// #1 There are some dependencies
#include "system.h"
#include "util.h"				// #2
#include "config.h"				// #3
#include "controller.h"
#include "canonical_machine.h"
#include "json_parser.h"
#include "gcode_parser.h"
#include "report.h"
#include "planner.h"
#include "stepper.h"
#include "spindle.h"
#include "network.h"
#include "gpio.h"
#include "test.h"
#include "pwm.h"

// local function prototypes (global prototypes are in tinyg.h)
#ifdef __DEBUG
static void _tg_debug_init(void);
#else
#define _tg_debug_init()
#endif

static void _tg_unit_tests(void);

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

void tg_system_reset(void)
{
	cli();					// These inits are order dependent:
	_tg_debug_init();		// (0) inits for the debug system
	sys_init();				// (1) system hardware setup
	xio_init();				// (2) xmega io subsystem
	tg_init(STD_INPUT);		// (3) tinyg controller (arg is std devices)

	sig_init();				// (4) signal flags
	rtc_init();				// (5) real time counter
	st_init(); 				// (6) stepper subsystem (must run before gp_init())
	pwm_init();				// (8) pulse width modulation drivers
	js_init();				// (9) JSON parser & etc.

	PMIC_EnableMediumLevel();// enable TX interrupts for init reporting 
	sei();					// enable global interrupts
	cfg_init();				// (10) get config record from eeprom (reqs xio)
	gpio_init();			// (7) switches and parallel IO
}

void tg_application_reset(void) 
{
	cli();					// disable global interrupts
	st_reset(); 			// reset stepper subsystem
	mp_init();				// motion planning subsystem
	cm_init();				// canonical machine
	gc_init();				// gcode-parser

	PMIC_SetVectorLocationToApplication();  // as opposed to boot ROM
	PMIC_EnableHighLevel();	// all levels are used, so don't bother to abstract them
	PMIC_EnableMediumLevel();
	PMIC_EnableLowLevel();
	sei();					// enable global interrupts
	tg_print_system_ready();// (LAST) announce system is ready
}

static void _tg_unit_tests(void) // uncomment __UNITS... line in .h file to enable unit tests
{
	XIO_UNITS;				// conditional unit tests for xio sub-system
//	EEPROM_UNITS;			// if you want this you must include the .h file in this file
	CONFIG_UNITS;
	JSON_UNITS;
	GPIO_UNITS;
	REPORT_UNITS;
	PLANNER_UNITS;
	PWM_UNITS;
}

/*
 * MAIN
 */

int main(void)
{
	tg_system_reset();
	tg_application_reset();
	_tg_unit_tests();
	tg_application_startup();

#ifdef __STANDALONE_MODE
	for(;;){ tg_controller();}	// this mode executes gcode blocks received via USB
#endif

#ifdef __MASTER_MODE
	for(;;){ tg_repeater();}	// this mode receives on USB and repeats to RS485
	}
#endif

#ifdef __SLAVE_MODE
	for(;;){ tg_receiver();}	// this mode executes gcode blocks received via RS485
#endif
}

#ifdef __DEBUG
void _tg_debug_init(void)	// inits for the debug system
{
#ifdef dbCONFIG_DEBUG_ENABLED
	dbCONFIG_DEBUG_ENABLED = TRUE;
#else
	dbCONFIG_DEBUG_ENABLED = FALSE;
#endif
}
#endif
