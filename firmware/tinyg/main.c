/*
 * main.c - TinyG - An embedded rs274/ngc CNC controller
 * This file is part of the TinyG project.
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart, Jr.
 * Copyright (c) 2013 - 2015 Robert Giseburt
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

#include "tinyg.h"					// #1 There are some dependencies
#include "config.h"					// #2
#include "hardware.h"
#include "persistence.h"
#include "controller.h"
#include "canonical_machine.h"
#include "report.h"
#include "planner.h"
#include "stepper.h"
#include "encoder.h"
#include "network.h"
#include "switch.h"
#include "test.h"
#include "pwm.h"
#include "xio.h"

#ifdef __AVR
#include <avr/interrupt.h>
#include "xmega/xmega_interrupts.h"
#endif // __AVR

#ifdef __ARM
#include "MotateTimers.h"
using Motate::delay;

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

void _init() __attribute__ ((weak));
void _init() {;}

void __libc_init_array(void);

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // __ARM

/******************** Application Code ************************/

#ifdef __ARM
const Motate::USBSettings_t Motate::USBSettings = {
	/*gVendorID         = */ 0x1d50,
	/*gProductID        = */ 0x606d,
	/*gProductVersion   = */ TINYG_FIRMWARE_VERSION,
	/*gAttributes       = */ kUSBConfigAttributeSelfPowered,
	/*gPowerConsumption = */ 500
};
	/*gProductVersion   = */ //0.1,

Motate::USBDevice< Motate::USBCDC > usb;
//Motate::USBDevice< Motate::USBCDC, Motate::USBCDC > usb;

typeof usb._mixin_0_type::Serial &SerialUSB = usb._mixin_0_type::Serial;
//typeof usb._mixin_1_type::Serial &SerialUSB1 = usb._mixin_1_type::Serial;

MOTATE_SET_USB_VENDOR_STRING( {'S' ,'y', 'n', 't', 'h', 'e', 't', 'o', 's'} )
MOTATE_SET_USB_PRODUCT_STRING( {'T', 'i', 'n', 'y', 'G', ' ', 'v', '2'} )
MOTATE_SET_USB_SERIAL_NUMBER_STRING( {'0','0','1'} )

Motate::SPI<kSocket4_SPISlaveSelectPinNumber> spi;
#endif

/*
 * _system_init()
 */

void _system_init(void)
{
#ifdef __ARM
	SystemInit();

	// Disable watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;

	// Initialize C library
	__libc_init_array();

	usb.attach();					// USB setup
	delay(1000);
#endif
}

/*
 * _application_init()
 */

static void _application_init(void)
{
	// There are a lot of dependencies in the order of these inits.
	// Don't change the ordering unless you understand this.

	cli();

	// do these first
	hardware_init();				// system hardware setup 			- must be first
	persistence_init();				// set up EEPROM or other NVM		- must be second
	rtc_init();						// real time counter
	xio_init();						// eXtended IO subsystem

	// do these next
	stepper_init(); 				// stepper subsystem 				- must precede gpio_init()
	encoder_init();					// virtual encoders
	switch_init();					// switches
//	gpio_init();					// parallel IO
	pwm_init();						// pulse width modulation drivers	- must follow gpio_init()

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
}

/*
 * main()
 */

int main(void)
{
	// system initialization
	_system_init();

	// TinyG application setup
	_application_init();
	run_canned_startup();			// run any pre-loaded commands

	// main loop
	for (;;) {
		controller_run( );			// single pass through the controller
	}
	return 0;
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

stat_t status_code;						// allocate a variable for the ritorno macro
char global_string_buf[MESSAGE_LEN];	// allocate a string for global message use

//#ifdef __TEXT_MODE

/*** Status message strings ***/

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
static const char stat_28[] PROGMEM = "Failed to get planner buffer";
static const char stat_29[] PROGMEM = "Generic exception report";

static const char stat_30[] PROGMEM = "Move time is infinite";
static const char stat_31[] PROGMEM = "Move time is NAN";
static const char stat_32[] PROGMEM = "Float is infinite";
static const char stat_33[] PROGMEM = "Float is NAN";
static const char stat_34[] PROGMEM = "Persistence error";
static const char stat_35[] PROGMEM = "Bad status report setting";
static const char stat_36[] PROGMEM = "36";
static const char stat_37[] PROGMEM = "37";
static const char stat_38[] PROGMEM = "38";
static const char stat_39[] PROGMEM = "39";

static const char stat_40[] PROGMEM = "40";
static const char stat_41[] PROGMEM = "41";
static const char stat_42[] PROGMEM = "42";
static const char stat_43[] PROGMEM = "43";
static const char stat_44[] PROGMEM = "44";
static const char stat_45[] PROGMEM = "45";
static const char stat_46[] PROGMEM = "46";
static const char stat_47[] PROGMEM = "47";
static const char stat_48[] PROGMEM = "48";
static const char stat_49[] PROGMEM = "49";
static const char stat_50[] PROGMEM = "50";
static const char stat_51[] PROGMEM = "51";
static const char stat_52[] PROGMEM = "52";
static const char stat_53[] PROGMEM = "53";
static const char stat_54[] PROGMEM = "54";
static const char stat_55[] PROGMEM = "55";
static const char stat_56[] PROGMEM = "56";
static const char stat_57[] PROGMEM = "57";
static const char stat_58[] PROGMEM = "58";
static const char stat_59[] PROGMEM = "59";
static const char stat_60[] PROGMEM = "60";
static const char stat_61[] PROGMEM = "61";
static const char stat_62[] PROGMEM = "62";
static const char stat_63[] PROGMEM = "63";
static const char stat_64[] PROGMEM = "64";
static const char stat_65[] PROGMEM = "65";
static const char stat_66[] PROGMEM = "66";
static const char stat_67[] PROGMEM = "67";
static const char stat_68[] PROGMEM = "68";
static const char stat_69[] PROGMEM = "69";
static const char stat_70[] PROGMEM = "70";
static const char stat_71[] PROGMEM = "71";
static const char stat_72[] PROGMEM = "72";
static const char stat_73[] PROGMEM = "73";
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

static const char stat_90[] PROGMEM = "Config sub-system assertion failure";
static const char stat_91[] PROGMEM = "IO sub-system assertion failure";
static const char stat_92[] PROGMEM = "Encoder assertion failure";
static const char stat_93[] PROGMEM = "Stepper assertion failure";
static const char stat_94[] PROGMEM = "Planner assertion failure";
static const char stat_95[] PROGMEM = "Canonical machine assertion failure";
static const char stat_96[] PROGMEM = "Controller assertion failure";
static const char stat_97[] PROGMEM = "Stack overflow detected";
static const char stat_98[] PROGMEM = "Memory fault detected";
static const char stat_99[] PROGMEM = "Generic assertion failure";

static const char stat_100[] PROGMEM = "Unrecognized command or config name";
static const char stat_101[] PROGMEM = "Invalid or malformed command";
static const char stat_102[] PROGMEM = "Bad number format";
static const char stat_103[] PROGMEM = "Unsupported number or JSON type";
static const char stat_104[] PROGMEM = "Parameter is read-only";
static const char stat_105[] PROGMEM = "Parameter cannot be read";
static const char stat_106[] PROGMEM = "Command not accepted at this time";
static const char stat_107[] PROGMEM = "Input exceeds max length";
static const char stat_108[] PROGMEM = "Input less than minimum value";
static const char stat_109[] PROGMEM = "Input exceeds maximum value";

static const char stat_110[] PROGMEM = "Input value range error";
static const char stat_111[] PROGMEM = "JSON syntax error";
static const char stat_112[] PROGMEM = "JSON input has too many pairs";
static const char stat_113[] PROGMEM = "JSON string too long";
static const char stat_114[] PROGMEM = "114";
static const char stat_115[] PROGMEM = "115";
static const char stat_116[] PROGMEM = "116";
static const char stat_117[] PROGMEM = "117";
static const char stat_118[] PROGMEM = "118";
static const char stat_119[] PROGMEM = "119";

static const char stat_120[] PROGMEM = "120";
static const char stat_121[] PROGMEM = "121";
static const char stat_122[] PROGMEM = "122";
static const char stat_123[] PROGMEM = "123";
static const char stat_124[] PROGMEM = "124";
static const char stat_125[] PROGMEM = "125";
static const char stat_126[] PROGMEM = "126";
static const char stat_127[] PROGMEM = "127";
static const char stat_128[] PROGMEM = "128";
static const char stat_129[] PROGMEM = "129";

static const char stat_130[] PROGMEM = "Generic Gcode input error";
static const char stat_131[] PROGMEM = "Gcode command unsupported";
static const char stat_132[] PROGMEM = "M code unsupported";
static const char stat_133[] PROGMEM = "Gcode modal group violation";
static const char stat_134[] PROGMEM = "Axis word missing";
static const char stat_135[] PROGMEM = "Axis cannot be present";
static const char stat_136[] PROGMEM = "Axis is invalid for this command";
static const char stat_137[] PROGMEM = "Axis is disabled";
static const char stat_138[] PROGMEM = "Axis target position is missing";
static const char stat_139[] PROGMEM = "Axis target position is invalid";

static const char stat_140[] PROGMEM = "Selected plane is missing";
static const char stat_141[] PROGMEM = "Selected plane is invalid";
static const char stat_142[] PROGMEM = "Feedrate not specified";
static const char stat_143[] PROGMEM = "Inverse time mode cannot be used with this command";
static const char stat_144[] PROGMEM = "Rotary axes cannot be used with this command";
static const char stat_145[] PROGMEM = "G0 or G1 must be active for G53";
static const char stat_146[] PROGMEM = "Requested velocity exceeds limits";
static const char stat_147[] PROGMEM = "Cutter compensation cannot be enabled";
static const char stat_148[] PROGMEM = "Programmed point same as current point";
static const char stat_149[] PROGMEM = "Spindle speed below minimum";

static const char stat_150[] PROGMEM = "Spindle speed exceeded maximum";
static const char stat_151[] PROGMEM = "Spindle S word is missing";
static const char stat_152[] PROGMEM = "Spindle S word is invalid";
static const char stat_153[] PROGMEM = "Spindle must be off for this command";
static const char stat_154[] PROGMEM = "Spindle must be turning for this command";
static const char stat_155[] PROGMEM = "Arc specification error";
static const char stat_156[] PROGMEM = "Arc specification error - missing axis(es)";
static const char stat_157[] PROGMEM = "Arc specification error - missing offset(s)";
static const char stat_158[] PROGMEM = "Arc specification error - radius arc out of tolerance";	// current longest message: 56 chard
static const char stat_159[] PROGMEM = "Arc specification error - endpoint is starting point";

static const char stat_160[] PROGMEM = "P word is missing";
static const char stat_161[] PROGMEM = "P word is invalid";
static const char stat_162[] PROGMEM = "P word is zero";
static const char stat_163[] PROGMEM = "P word is negative";
static const char stat_164[] PROGMEM = "P word is not an integer";
static const char stat_165[] PROGMEM = "P word is not a valid tool number";
static const char stat_166[] PROGMEM = "D word is missing";
static const char stat_167[] PROGMEM = "D word is invalid";
static const char stat_168[] PROGMEM = "E word is missing";
static const char stat_169[] PROGMEM = "E word is invalid";

static const char stat_170[] PROGMEM = "H word is missing";
static const char stat_171[] PROGMEM = "H word is invalid";
static const char stat_172[] PROGMEM = "L word is missing";
static const char stat_173[] PROGMEM = "L word is invalid";
static const char stat_174[] PROGMEM = "Q word is missing";
static const char stat_175[] PROGMEM = "Q word is invalid";
static const char stat_176[] PROGMEM = "R word is missing";
static const char stat_177[] PROGMEM = "R word is invalid";
static const char stat_178[] PROGMEM = "T word is missing";
static const char stat_179[] PROGMEM = "T word is invalid";

static const char stat_180[] PROGMEM = "180";
static const char stat_181[] PROGMEM = "181";
static const char stat_182[] PROGMEM = "182";
static const char stat_183[] PROGMEM = "183";
static const char stat_184[] PROGMEM = "184";
static const char stat_185[] PROGMEM = "185";
static const char stat_186[] PROGMEM = "186";
static const char stat_187[] PROGMEM = "187";
static const char stat_188[] PROGMEM = "188";
static const char stat_189[] PROGMEM = "189";

static const char stat_190[] PROGMEM = "190";
static const char stat_191[] PROGMEM = "191";
static const char stat_192[] PROGMEM = "192";
static const char stat_193[] PROGMEM = "193";
static const char stat_194[] PROGMEM = "194";
static const char stat_195[] PROGMEM = "195";
static const char stat_196[] PROGMEM = "196";
static const char stat_197[] PROGMEM = "197";
static const char stat_198[] PROGMEM = "198";
static const char stat_199[] PROGMEM = "199";

static const char stat_200[] PROGMEM = "Generic TinyG error";
static const char stat_201[] PROGMEM = "Move less than minimum length";
static const char stat_202[] PROGMEM = "Move less than minimum time";
static const char stat_203[] PROGMEM = "Machine is alarmed - Command not processed";	// current longest message 43 chars (including NUL)
static const char stat_204[] PROGMEM = "Limit switch hit - Shutdown occurred";
static const char stat_205[] PROGMEM = "Trapezoid planner failed to converge";
static const char stat_206[] PROGMEM = "206";
static const char stat_207[] PROGMEM = "207";
static const char stat_208[] PROGMEM = "208";
static const char stat_209[] PROGMEM = "209";

static const char stat_210[] PROGMEM = "210";
static const char stat_211[] PROGMEM = "211";
static const char stat_212[] PROGMEM = "212";
static const char stat_213[] PROGMEM = "213";
static const char stat_214[] PROGMEM = "214";
static const char stat_215[] PROGMEM = "215";
static const char stat_216[] PROGMEM = "216";
static const char stat_217[] PROGMEM = "217";
static const char stat_218[] PROGMEM = "218";
static const char stat_219[] PROGMEM = "219";

static const char stat_220[] PROGMEM = "Soft limit exceeded";
static const char stat_221[] PROGMEM = "Soft limit exceeded - X min";
static const char stat_222[] PROGMEM = "Soft limit exceeded - X max";
static const char stat_223[] PROGMEM = "Soft limit exceeded - Y min";
static const char stat_224[] PROGMEM = "Soft limit exceeded - Y max";
static const char stat_225[] PROGMEM = "Soft limit exceeded - Z min";
static const char stat_226[] PROGMEM = "Soft limit exceeded - Z max";
static const char stat_227[] PROGMEM = "Soft limit exceeded - A min";
static const char stat_228[] PROGMEM = "Soft limit exceeded - A max";
static const char stat_229[] PROGMEM = "Soft limit exceeded - B min";
static const char stat_230[] PROGMEM = "Soft limit exceeded - B max";
static const char stat_231[] PROGMEM = "Soft limit exceeded - C min";
static const char stat_232[] PROGMEM = "Soft limit exceeded - C max";
static const char stat_233[] PROGMEM = "233";
static const char stat_234[] PROGMEM = "234";
static const char stat_235[] PROGMEM = "235";
static const char stat_236[] PROGMEM = "236";
static const char stat_237[] PROGMEM = "237";
static const char stat_238[] PROGMEM = "238";
static const char stat_239[] PROGMEM = "239";

static const char stat_240[] PROGMEM = "Homing cycle failed";
static const char stat_241[] PROGMEM = "Homing Error - Bad or no axis specified";
static const char stat_242[] PROGMEM = "Homing Error - Search velocity is zero";
static const char stat_243[] PROGMEM = "Homing Error - Latch velocity is zero";
static const char stat_244[] PROGMEM = "Homing Error - Travel min & max are the same";
static const char stat_245[] PROGMEM = "Homing Error - Negative latch backoff";
static const char stat_246[] PROGMEM = "Homing Error - Homing switches misconfigured";
static const char stat_247[] PROGMEM = "247";
static const char stat_248[] PROGMEM = "248";
static const char stat_249[] PROGMEM = "249";

static const char stat_250[] PROGMEM = "Probe cycle failed";
static const char stat_251[] PROGMEM = "Probe endpoint is starting point";
static const char stat_252[] PROGMEM = "Jogging cycle failed";

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
	stat_100, stat_101, stat_102, stat_103, stat_104, stat_105, stat_106, stat_107, stat_108, stat_109,
	stat_110, stat_111, stat_112, stat_113, stat_114, stat_115, stat_116, stat_117, stat_118, stat_119,
	stat_120, stat_121, stat_122, stat_123, stat_124, stat_125, stat_126, stat_127, stat_128, stat_129,
	stat_130, stat_131, stat_132, stat_133, stat_134, stat_135, stat_136, stat_137, stat_138, stat_139,
	stat_140, stat_141, stat_142, stat_143, stat_144, stat_145, stat_146, stat_147, stat_148, stat_149,
	stat_150, stat_151, stat_152, stat_153, stat_154, stat_155, stat_156, stat_157, stat_158, stat_159,
	stat_160, stat_161, stat_162, stat_163, stat_164, stat_165, stat_166, stat_167, stat_168, stat_169,
	stat_170, stat_171, stat_172, stat_173, stat_174, stat_175, stat_176, stat_177, stat_178, stat_179,
	stat_180, stat_181, stat_182, stat_183, stat_184, stat_185, stat_186, stat_187, stat_188, stat_189,
	stat_190, stat_191, stat_192, stat_193, stat_194, stat_195, stat_196, stat_197, stat_198, stat_199,
	stat_200, stat_201, stat_202, stat_203, stat_204, stat_205, stat_206, stat_207, stat_208, stat_209,
	stat_210, stat_211, stat_212, stat_213, stat_214, stat_215, stat_216, stat_217, stat_218, stat_219,
	stat_220, stat_221, stat_222, stat_223, stat_224, stat_225, stat_226, stat_227, stat_228, stat_229,
	stat_230, stat_231, stat_232, stat_233, stat_234, stat_235, stat_236, stat_237, stat_238, stat_239,
	stat_240, stat_241, stat_242, stat_243, stat_244, stat_245, stat_246, stat_247, stat_248, stat_249,
	stat_250, stat_251, stat_252
};

char *get_status_message(stat_t status)
{
	return ((char *)GET_TEXT_ITEM(stat_msg, status));
}
