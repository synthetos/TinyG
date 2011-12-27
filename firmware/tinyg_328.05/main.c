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

#include "system.h"
#include "xmega_interrupts.h"
#include "xmega_rtc.h"
#include "xio.h"

#include "tinyg.h"
#include "config.h"
#include "controller.h"
#include "gcode_parser.h"
#include "stepper.h"
#include "planner.h"
#include "spindle.h"
#include "network.h"
#include "gpio.h"
#include "util.h"

// local function prototypes (global prototypes are in tinyg.h)
#ifdef __DEBUG
static void _tg_debug_init(void);
#else
#define _tg_debug_init()
#endif

#ifdef __UNIT_TESTS
static void _tg_unit_tests(void);
#else
#define _tg_unit_tests()
#endif

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
	sw_init();				// limit & homing switches
	mp_init();				// motion planning subsystem
	sp_init();				// spindle controller
	en_init();				// GPIO port
	gc_init();				// gcode-parser

	PMIC_SetVectorLocationToApplication();  // as opposed to boot ROM
	PMIC_EnableHighLevel();	// all levels are used, so don't bother to abstract them
	PMIC_EnableMediumLevel();
	PMIC_EnableLowLevel();
	sei();					// enable global interrupts
	tg_alive();				// (LAST) announce app is online
}

#ifdef __UNIT_TESTS			// uncomment __UNIT_TESTS in util.h and uncomment what you need
static void _tg_unit_tests(void)
{
//	xio_tests();			// IO subsystem
//	EEPROM_tests();			// EEPROM tests
//	cfg_unit_tests();		// config tests
//	mp_unit_tests();		// planner tests
//	mq_unit_tests();		// motor queue / stepper tests
}
#endif

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
		tg_controller();	// this mode executes gcode blocks received via USB
	}
#endif

#ifdef __MASTER_MODE
	for(;;){
		tg_repeater();		// this mode receives on USB and repeats to RS485
	}
#endif

#ifdef __SLAVE_MODE
	for(;;){
		tg_receiver();		// this mode executes gcode blocks received via RS485
	}
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

