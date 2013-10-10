/*
 * main.c - TinyG - An embedded rs274/ngc CNC controller
 * This file is part of the TinyG project.
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart, Jr.
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
/* See github.com/Synthetos/tinyg for code and docs on the wiki 
 */

#include <avr/interrupt.h>

#include "tinyg.h"				// #1 There are some dependencies
#include "config.h"				// #2
#include "hardware.h"
#include "controller.h"
#include "canonical_machine.h"
#include "report.h"
#include "planner.h"
#include "stepper.h"
#include "network.h"
#include "switch.h"
#include "test.h"
#include "pwm.h"

#include "xio/xio.h"
#include "xmega/xmega_interrupts.h"
//#include "xmega/xmega_rtc.h"		// included via hardware.h
//#include "xmega/xmega_eeprom.h"	// uncomment for unit tests

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

void _init() __attribute__ ((weak));
void _init() {;}

#ifdef __cplusplus
}
#endif // __cplusplus

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

	// hardware and low-level drivers
	hardware_init();				// system hardware setup 			- must be first
	rtc_init();						// real time counter
	xio_init();						// xmega io subsystem
	stepper_init(); 				// stepper subsystem 				- must precede gpio_init()
	switch_init();					// switches 
//	gpio_init();					// parallel IO
	pwm_init();						// pulse width modulation drivers	- must follow gpio_init()

	// application sub-systems
	controller_init(STD_IN, STD_OUT, STD_ERR);// must be first app init; reqs xio_init()
	config_init();					// config records from eeprom 		- must be next app init
	network_init();					// reset std devices if required	- must follow config_init()
	planner_init();					// motion planning subsystem
	canonical_machine_init();		// canonical machine				- must follow config_init()

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

/**** Status Messages ***************************************************************
 * get_status_message() - return the status message
 *
 * See tinyg.h for status codes. These strings must align with the status codes in tinyg.h
 * The number of elements in the indexing array must match the # of strings
 *
 * Reference for putting display strings and string arrays in AVR program memory:
 * http://www.cs.mun.ca/~paul/cs4723/material/atmel/avr-libc-user-manual-1.6.5/pgmspace.html
 */

stat_t status_code;						// allocate a variable for this macro
char shared_buf[STATUS_MESSAGE_LEN];	// allocate string for global use

static const char stat_00[] PROGMEM = "OK";
static const char stat_01[] PROGMEM = "Error";
static const char stat_02[] PROGMEM = "Eagain";
static const char stat_03[] PROGMEM = "Noop";
static const char stat_04[] PROGMEM = "Complete";
static const char stat_05[] PROGMEM = "Terminated";
static const char stat_06[] PROGMEM = "Hard reset";
static const char stat_07[] PROGMEM = "End of line";
static const char stat_08[] PROGMEM = "End of file";
static const char stat_09[] PROGMEM = "File not open";
static const char stat_10[] PROGMEM = "Max file size exceeded";
static const char stat_11[] PROGMEM = "No such device";
static const char stat_12[] PROGMEM = "Buffer empty";
static const char stat_13[] PROGMEM = "Buffer full";
static const char stat_14[] PROGMEM = "Buffer full - fatal";
static const char stat_15[] PROGMEM = "Initializing";
static const char stat_16[] PROGMEM = "Entering boot loader";
static const char stat_17[] PROGMEM = "Function is stubbed";
static const char stat_18[] PROGMEM = "18";
static const char stat_19[] PROGMEM = "19";

static const char stat_20[] PROGMEM = "Internal error";
static const char stat_21[] PROGMEM = "Internal range error";
static const char stat_22[] PROGMEM = "Floating point error";
static const char stat_23[] PROGMEM = "Divide by zero";
static const char stat_24[] PROGMEM = "Invalid Address";
static const char stat_25[] PROGMEM = "Read-only address";
static const char stat_26[] PROGMEM = "Initialization failure";
static const char stat_27[] PROGMEM = "System alarm - shutting down";
static const char stat_28[] PROGMEM = "Memory fault or corruption";
static const char stat_29[] PROGMEM = "29";
static const char stat_30[] PROGMEM = "30";
static const char stat_31[] PROGMEM = "31";
static const char stat_32[] PROGMEM = "32";
static const char stat_33[] PROGMEM = "33";
static const char stat_34[] PROGMEM = "34";
static const char stat_35[] PROGMEM = "35";
static const char stat_36[] PROGMEM = "36";
static const char stat_37[] PROGMEM = "37";
static const char stat_38[] PROGMEM = "38";
static const char stat_39[] PROGMEM = "39";

static const char stat_40[] PROGMEM = "Unrecognized command";
static const char stat_41[] PROGMEM = "Expected command letter";
static const char stat_42[] PROGMEM = "Bad number format";
static const char stat_43[] PROGMEM = "Input exceeds max length";
static const char stat_44[] PROGMEM = "Input value too small";
static const char stat_45[] PROGMEM = "Input value too large";
static const char stat_46[] PROGMEM = "Input value range error";
static const char stat_47[] PROGMEM = "Input value unsupported";
static const char stat_48[] PROGMEM = "JSON syntax error";
static const char stat_49[] PROGMEM = "JSON input has too many pairs";	// current longest message: 30 chars
static const char stat_50[] PROGMEM = "JSON output too long";
static const char stat_51[] PROGMEM = "Out of buffer space";
static const char stat_52[] PROGMEM = "Config rejected during cycle";
static const char stat_53[] PROGMEM = "53";
static const char stat_54[] PROGMEM = "54";
static const char stat_55[] PROGMEM = "55";
static const char stat_56[] PROGMEM = "56";
static const char stat_57[] PROGMEM = "57";
static const char stat_58[] PROGMEM = "58";
static const char stat_59[] PROGMEM = "59";

static const char stat_60[] PROGMEM = "Move less than minimum length";
static const char stat_61[] PROGMEM = "Move less than minimum time";
static const char stat_62[] PROGMEM = "Gcode block skipped";
static const char stat_63[] PROGMEM = "Gcode input error";
static const char stat_64[] PROGMEM = "Gcode feedrate error";
static const char stat_65[] PROGMEM = "Gcode axis word missing";
static const char stat_66[] PROGMEM = "Gcode modal group violation";
static const char stat_67[] PROGMEM = "Homing cycle failed";
static const char stat_68[] PROGMEM = "Max travel exceeded";
static const char stat_69[] PROGMEM = "Max spindle speed exceeded";
static const char stat_70[] PROGMEM = "Arc specification error";
static const char stat_71[] PROGMEM = "Soft limit exceeded";
static const char stat_72[] PROGMEM = "Command not accepted";
static const char stat_73[] PROGMEM = "Probing cycle failed";
static const char stat_74[] PROGMEM = "74";
static const char stat_75[] PROGMEM = "75";
static const char stat_76[] PROGMEM = "76";
static const char stat_77[] PROGMEM = "77";
static const char stat_78[] PROGMEM = "78";
static const char stat_79[] PROGMEM = "79";
static const char stat_80[] PROGMEM = "80";
static const char stat_81[] PROGMEM = "81";
static const char stat_82[] PROGMEM = "82";
static const char stat_83[] PROGMEM = "83";
static const char stat_84[] PROGMEM = "84";
static const char stat_85[] PROGMEM = "85";
static const char stat_86[] PROGMEM = "86";
static const char stat_87[] PROGMEM = "87";
static const char stat_88[] PROGMEM = "88";
static const char stat_89[] PROGMEM = "89";
static const char stat_90[] PROGMEM = "90";
static const char stat_91[] PROGMEM = "91";
static const char stat_92[] PROGMEM = "92";
static const char stat_93[] PROGMEM = "93";
static const char stat_94[] PROGMEM = "94";
static const char stat_95[] PROGMEM = "95";
static const char stat_96[] PROGMEM = "96";
static const char stat_97[] PROGMEM = "97";
static const char stat_98[] PROGMEM = "98";
static const char stat_99[] PROGMEM = "99";

static const char stat_100[] PROGMEM = "Generic assertion failure";
static const char stat_101[] PROGMEM = "Generic exception report";
static const char stat_102[] PROGMEM = "Memory fault detected";
static const char stat_103[] PROGMEM = "Stack overflow detected";
static const char stat_104[] PROGMEM = "Controller assertion failure";
static const char stat_105[] PROGMEM = "Canonical machine assertion failure";
static const char stat_106[] PROGMEM = "Planner assertion failure";
static const char stat_107[] PROGMEM = "Stepper assertion failure";
static const char stat_108[] PROGMEM = "Extended IO assertion failure";

static const char *const stat_msg[] PROGMEM = {
	stat_00, stat_01, stat_02, stat_03, stat_04, stat_05, stat_06, stat_07, stat_08, stat_09,
	stat_10, stat_11, stat_12, stat_13, stat_14, stat_15, stat_16, stat_17, stat_18, stat_19,
	stat_20, stat_21, stat_22, stat_23, stat_24, stat_25, stat_26, stat_27, stat_28, stat_29,
	stat_30, stat_31, stat_32, stat_33, stat_34, stat_35, stat_36, stat_37, stat_38, stat_39,
	stat_40, stat_41, stat_42, stat_43, stat_44, stat_45, stat_46, stat_47, stat_48, stat_49,
	stat_50, stat_51, stat_52, stat_53, stat_54, stat_55, stat_56, stat_57, stat_58, stat_59,
	stat_60, stat_61, stat_62, stat_63, stat_64, stat_65, stat_66, stat_67, stat_68, stat_69,
	stat_70, stat_71, stat_72, stat_73, stat_74, stat_75, stat_76, stat_77, stat_78, stat_79, 
	stat_80, stat_81, stat_82, stat_83, stat_84, stat_85, stat_86, stat_87, stat_88, stat_89,
	stat_90, stat_91, stat_92, stat_93, stat_94, stat_95, stat_96, stat_97, stat_98, stat_99,
	stat_100, stat_101, stat_102, stat_103, stat_104, stat_105, stat_106, stat_107, stat_108
};

char *get_status_message(stat_t status)
{
	return ((char *)GET_TEXT_ITEM(stat_msg, status));
}

/*******************************************************************************
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
