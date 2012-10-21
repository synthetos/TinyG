/*
 * Tinyg_tc.c - TinyG temperature controller device
 * Part of TinyG project
 * Based on Kinen Motion Control System 
 *
 * Copyright (c) 2012 Alden S. Hart Jr.
 *
 * The Kinen Motion Control System is licensed under the OSHW 1.0 license
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>				// for memset
#include <avr/io.h>
#include <avr/interrupt.h>

#include "kinen_core.h"
#include "tinyg_tc.h"

// statics and assorted 

static void _controller(void);
static uint8_t device_array[DEVICE_ADDRESS_MAX];

static struct DeviceSingleton {
	uint32_t drvconf;			// word for loading TMC262
} dev;


/****************************************************************************
 * main
 *
 *	Device and Kinen initialization
 *	Main loop handler
 */
int main(void)
{
	cli();						// initializations
	kinen_init();				// do this first
	device_init();
	sei(); 						// enable interrupts

//	device_unit_tests();		// comment out the unit tests for production

	while (true) {				// go to the controller loop and never return
		_controller();
	}
	return (false);				// never returns
}

/*
 * Dispatch loop
 *
 *	The dispatch loop is a set of pre-registered callbacks that (in effect) 
 *	provide rudimentry multi-threading. Functions are organized from highest
 *	priority to lowest priority. Each called function must return a status code
 *	(see kinen_core.h). If SC_EAGAIN (02) is returned the loop restarts at the
 *	start of the list. For any other status code exceution continues down the list
 */

#define	DISPATCH(func) if (func == SC_EAGAIN) return; 
static void _controller()
{
	DISPATCH(kinen_callback());	// intercept low-level communication events
	DISPATCH(rtc_callback());	// real-time clock handler
//	DISPATCH(_dispatch());		// read and execute next command
}

/**** Device Functions ****
 * device_init()  - initialize device
 * device_led_on()
 * device_led_off()
 */
void device_init(void)
{
	DDRB = PORTB_DIR;			// initialize all ports for proper IO
	DDRC = PORTC_DIR;
	DDRD = PORTD_DIR;

	// initialize the chip
	device_led_on();		// put on the red light (Roxanne)
}

void device_led_on(void) 
{
	LED_PORT &= ~(LED_PIN);
}

void device_led_off(void) 
{
	LED_PORT |= LED_PIN;
}

/**** RTC - Real Time Clock Functions ****
 * rtc_init() 	  - initialize RTC timers and data
 * rtc_callback() - run RTC from dispatch loop
 */
void rtc_init(void)
{
	
}

uint8_t rtc_callback(void)
{
	return (SC_NOOP);
}

/**** PWM - Pulse Width Modulation Functions ****
 * pwm_init() 	  - initialize RTC timers and data
 */
void pwm_init(void)
{
	
}


/****************************************************************************
 *
 * Kinen Callback functions - mandatory
 *
 *	These functions are called from Kinen drivers and must be implemented 
 *	for any Kinen device
 *
 *	device_reset() 		- reset device in response tro Kinen reset command
 *	device_read_byte() 	- read a byte from Kinen channel into device structs
 *	device_write_byte() - write a byte from device to Kinen channel
 */

void device_reset(void)
{
	return;
}

uint8_t device_read_byte(uint8_t addr, uint8_t *data)
{
	addr -= KINEN_COMMON_MAX;
	if (addr >= DEVICE_ADDRESS_MAX) return (SC_INVALID_ADDRESS);
	*data = device_array[addr];
	return (SC_OK);
}

uint8_t device_write_byte(uint8_t addr, uint8_t data)
{
	addr -= KINEN_COMMON_MAX;
	if (addr >= DEVICE_ADDRESS_MAX) return (SC_INVALID_ADDRESS);
	// There are no checks in here for read-only locations
	// Assumes all locations are writable.
	device_array[addr] = data;
	return (SC_OK);
}


//###########################################################################
//##### UNIT TESTS ##########################################################
//###########################################################################

#ifdef __UNIT_TEST_DEVICE

void device_unit_tests()
{

// success cases

// exception cases

}

#endif // __UNIT_TEST_DEVICE

