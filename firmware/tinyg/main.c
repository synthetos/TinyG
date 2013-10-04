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
#include "util.h"

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

static const char PROGMEM stat_00[] = "OK";
static const char PROGMEM stat_01[] = "Error";
static const char PROGMEM stat_02[] = "Eagain";
static const char PROGMEM stat_03[] = "Noop";
static const char PROGMEM stat_04[] = "Complete";
static const char PROGMEM stat_05[] = "Terminated";
static const char PROGMEM stat_06[] = "Hard reset";
static const char PROGMEM stat_07[] = "End of line";
static const char PROGMEM stat_08[] = "End of file";
static const char PROGMEM stat_09[] = "File not open";
static const char PROGMEM stat_10[] = "Max file size exceeded";
static const char PROGMEM stat_11[] = "No such device";
static const char PROGMEM stat_12[] = "Buffer empty";
static const char PROGMEM stat_13[] = "Buffer full";
static const char PROGMEM stat_14[] = "Buffer full - fatal";
static const char PROGMEM stat_15[] = "Initializing";
static const char PROGMEM stat_16[] = "Entering boot loader";
static const char PROGMEM stat_17[] = "Function is stubbed";
static const char PROGMEM stat_18[] = "18";
static const char PROGMEM stat_19[] = "19";

static const char PROGMEM stat_20[] = "Internal error";
static const char PROGMEM stat_21[] = "Internal range error";
static const char PROGMEM stat_22[] = "Floating point error";
static const char PROGMEM stat_23[] = "Divide by zero";
static const char PROGMEM stat_24[] = "Invalid Address";
static const char PROGMEM stat_25[] = "Read-only address";
static const char PROGMEM stat_26[] = "Initialization failure";
static const char PROGMEM stat_27[] = "System alarm - shutting down";
static const char PROGMEM stat_28[] = "Memory fault or corruption";
static const char PROGMEM stat_29[] = "29";
static const char PROGMEM stat_30[] = "30";
static const char PROGMEM stat_31[] = "31";
static const char PROGMEM stat_32[] = "32";
static const char PROGMEM stat_33[] = "33";
static const char PROGMEM stat_34[] = "34";
static const char PROGMEM stat_35[] = "35";
static const char PROGMEM stat_36[] = "36";
static const char PROGMEM stat_37[] = "37";
static const char PROGMEM stat_38[] = "38";
static const char PROGMEM stat_39[] = "39";

static const char PROGMEM stat_40[] = "Unrecognized command";
static const char PROGMEM stat_41[] = "Expected command letter";
static const char PROGMEM stat_42[] = "Bad number format";
static const char PROGMEM stat_43[] = "Input exceeds max length";
static const char PROGMEM stat_44[] = "Input value too small";
static const char PROGMEM stat_45[] = "Input value too large";
static const char PROGMEM stat_46[] = "Input value range error";
static const char PROGMEM stat_47[] = "Input value unsupported";
static const char PROGMEM stat_48[] = "JSON syntax error";
static const char PROGMEM stat_49[] = "JSON input has too many pairs";	// current longest message: 30 chars
static const char PROGMEM stat_50[] = "JSON output too long";
static const char PROGMEM stat_51[] = "Out of buffer space";
static const char PROGMEM stat_52[] = "Config rejected during cycle";
static const char PROGMEM stat_53[] = "53";
static const char PROGMEM stat_54[] = "54";
static const char PROGMEM stat_55[] = "55";
static const char PROGMEM stat_56[] = "56";
static const char PROGMEM stat_57[] = "57";
static const char PROGMEM stat_58[] = "58";
static const char PROGMEM stat_59[] = "59";

static const char PROGMEM stat_60[] = "Move less than minimum length";
static const char PROGMEM stat_61[] = "Move less than minimum time";
static const char PROGMEM stat_62[] = "Gcode block skipped";
static const char PROGMEM stat_63[] = "Gcode input error";
static const char PROGMEM stat_64[] = "Gcode feedrate error";
static const char PROGMEM stat_65[] = "Gcode axis word missing";
static const char PROGMEM stat_66[] = "Gcode modal group violation";
static const char PROGMEM stat_67[] = "Homing cycle failed";
static const char PROGMEM stat_68[] = "Max travel exceeded";
static const char PROGMEM stat_69[] = "Max spindle speed exceeded";
static const char PROGMEM stat_70[] = "Arc specification error";
static const char PROGMEM stat_71[] = "Soft limit exceeded";
static const char PROGMEM stat_72[] = "Command not accepted";
static const char PROGMEM stat_73[] = "Probing cycle failed";
static const char PROGMEM stat_74[] = "74";
static const char PROGMEM stat_75[] = "75";
static const char PROGMEM stat_76[] = "76";
static const char PROGMEM stat_77[] = "77";
static const char PROGMEM stat_78[] = "78";
static const char PROGMEM stat_79[] = "79";
static const char PROGMEM stat_80[] = "80";
static const char PROGMEM stat_81[] = "81";
static const char PROGMEM stat_82[] = "82";
static const char PROGMEM stat_83[] = "83";
static const char PROGMEM stat_84[] = "84";
static const char PROGMEM stat_85[] = "85";
static const char PROGMEM stat_86[] = "86";
static const char PROGMEM stat_87[] = "87";
static const char PROGMEM stat_88[] = "88";
static const char PROGMEM stat_89[] = "89";
static const char PROGMEM stat_90[] = "90";
static const char PROGMEM stat_91[] = "91";
static const char PROGMEM stat_92[] = "92";
static const char PROGMEM stat_93[] = "93";
static const char PROGMEM stat_94[] = "94";
static const char PROGMEM stat_95[] = "95";
static const char PROGMEM stat_96[] = "96";
static const char PROGMEM stat_97[] = "97";
static const char PROGMEM stat_98[] = "98";
static const char PROGMEM stat_99[] = "99";

static const char PROGMEM stat_100[] = "Generic assertion failure";
static const char PROGMEM stat_101[] = "Generic exception report";
static const char PROGMEM stat_102[] = "Memory fault detected";
static const char PROGMEM stat_103[] = "Stack overflow detected";
static const char PROGMEM stat_104[] = "Controller assertion failure";
static const char PROGMEM stat_105[] = "Canonical machine assertion failure";
static const char PROGMEM stat_106[] = "Planner assertion failure";
static const char PROGMEM stat_107[] = "Stepper assertion failure";
static const char PROGMEM stat_108[] = "Extended IO assertion failure";

static const char PROGMEM *stat_msg[] = {
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
