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
#include "spindle.h"
//#include "coolant.h"
#include "switch.h"     // #include "gpio.h"
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
/************************** to here ***************************/

void* __dso_handle = nullptr;

#endif // __ARM

/******************** System Globals *************************/

stat_t status_code;						    // allocate a variable for the ritorno macro
char global_string_buf[GLOBAL_STRING_LEN];	// allocate a string for global message use
static char _status_message[STATUS_MESSAGE_LEN];

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
 * application_inits
 *
 * There are a lot of dependencies in the order of these inits.
 * Don't change the ordering unless you understand this.
 */

static void _application_init_services(void)
{
    hardware_init();				// FIRST:  system hardware setup
    persistence_init();				// SECOND: set up EEPROM or other NVM
    xio_init();						// THIRD:  extended io subsystem
#ifdef __AVR
    rtc_init();						// real time counter
#endif
}

static void _application_init_machine(void)
{
    cm.machine_state = MACHINE_INITIALIZING;

	stepper_init(); 				// stepper subsystem - must precede gpio_init()
	encoder_init();					// virtual encoders
	switch_init();					// switches
//    gpio_init();                    // inputs and outputs
	pwm_init();						// pulse width modulation drivers - must follow gpio_init()
	planner_init();					// motion planning subsystem
	canonical_machine_init();		// canonical machine
}

static void _application_init_startup(void)
{
    // start the application
	controller_init(STD_IN, STD_OUT, STD_ERR);  // FIRST:  depends on xio_init()
	config_init();					            // SECOND: read config records from eeprom
	canonical_machine_reset();                  // depends on config_init()
    spindle_init();                             // depends on config_init()
    spindle_reset();

#ifdef __AVR
	// now bring up the interrupts and get started
	PMIC_SetVectorLocationToApplication();      // as opposed to boot ROM
	PMIC_EnableHighLevel();			            // all levels are used, so don't bother to abstract them
	PMIC_EnableMediumLevel();
	PMIC_EnableLowLevel();
	sei();							            // enable global interrupts
#endif

    // Note: system-ready message generated by controller.c 
}

/*
 * main()
 */

int main(void)
{
#ifdef __AVR
//    cli();
#endif

	// system initialization
	_system_init();

	// TinyG application setup
	_application_init_services();
	_application_init_machine();
	_application_init_startup();
	run_canned_startup();			// run any pre-loaded commands

	// main loop
	for (;;) {
		controller_run( );			// single pass through the controller
	}
	return 0;
}


/*
 * get_status_message() - support for status messages.
 */

char *get_status_message(stat_t status)
{
#ifdef __ARM
    return (&stat_msg[status]); // simple
#endif
#ifdef __AVR    // not so simple
    strncpy_P(_status_message, (char *)pgm_read_word(&stat_msg[status]), STATUS_MESSAGE_LEN);
    return (_status_message);
#endif
}
