/*
 * main.c - TinyG - An embedded rs274/ngc CNC controller
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* See github.coom/Synthetos/tinyg for code and docs on the wiki 
 */

#include <avr/interrupt.h>

#include "tinyg.h"				// #1 There are some dependencies
#include "config.h"				// #2
#include "hardware.h"
#include "util.h"
#include "controller.h"
#include "canonical_machine.h"
#include "json_parser.h"
#include "gcode_parser.h"
#include "report.h"
#include "planner.h"
#include "stepper.h"
#include "spindle.h"
#include "network.h"
#include "switch.h"
#include "gpio.h"
#include "test.h"
#include "pwm.h"

#include "xio/xio.h"
#include "xmega/xmega_interrupts.h"
#include "xmega/xmega_rtc.h"
//#include "xmega/xmega_eeprom.h"	// uncomment for unit tests

/**** Random global variables ****/
char status_message[STATUS_MESSAGE_LEN];// allocate string for global use
stat_t status_code;						// allocate a variable for this macro

static void _unit_tests(void);

/*
 * Inits and MAIN
 */

int main(void)
{
	// There are a lot of dependencies in the order of these inits.
	// Don't change the ordering unless you understand this.
	// Inits can assume that all memory has been zeroed by either 
	// a hardware reset or a watchdog timer reset.

	cli();

	// system and drivers
	hardware_init();				// system hardware setup 			- must be first
	rtc_init();						// real time counter
	xio_init();						// xmega io subsystem
	stepper_init(); 				// stepper subsystem 				- must precede gpio_init()
	switch_init();					// switches 
//	gpio_init();					// parallel IO
	pwm_init();						// pulse width modulation drivers	- must follow gpio_init()

	// application structures
	controller_init(STD_IN, STD_OUT, STD_ERR);// must be first app init; reqs xio_init()
	config_init();					// config records from eeprom 		- must be next app init
	net_init();						// reset std devices if required	- must follow config_init()
	planner_init();					// motion planning subsystem
	canonical_machine_init();		// canonical machine				- must follow config_init()
	sp_init();						// spindle PWM and variables

	// now bring up the interrupts and get started
	PMIC_SetVectorLocationToApplication();// as opposed to boot ROM
	PMIC_EnableHighLevel();			// all levels are used, so don't bother to abstract them
	PMIC_EnableMediumLevel();
	PMIC_EnableLowLevel();
	sei();							// enable global interrupts
	rpt_print_system_ready_message();// (LAST) announce system is ready

	_unit_tests();					// run any unit tests that are enabled
	tg_canned_startup();			// run any pre-loaded commands
	
	while (true) {
		controller_run(); 
	}
}

/*
 * _unit_tests() - uncomment __UNITS... line in .h files to enable unit tests
 */

static void _unit_tests(void) 
{
#ifdef __UNIT_TESTS
	XIO_UNITS;				// conditional unit tests for xio sub-system
//	EEPROM_UNITS;			// if you want this you must include the .h file in this file
	CONFIG_UNITS;
	JSON_UNITS;
	GPIO_UNITS;
	REPORT_UNITS;
	PLANNER_UNITS;
	PWM_UNITS;
#endif
}
