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

// static functions 
static void _controller(void);
static uint8_t _idle_task(void);

// static data
static struct DeviceSingleton {
	double temperature_reading;
	double temperature_set_point;

	uint8_t rtc_flag;			// true = the timer interrupt fired
	uint8_t rtc_100ms_count;	// 100ms down counter
	uint8_t rtc_1sec_count;		// 1 second down counter

} dev;

static uint8_t device_array[DEVICE_ADDRESS_MAX];


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
	device_init();				// handles all the device inits
	sei(); 						// enable interrupts

	device_unit_tests();		// uncomment __UNIT_TEST_DEVICE to enable unit tests

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
	DISPATCH(pid_controller());	// main controller task
	DISPATCH(_idle_task());
}

static uint8_t _idle_task() { return (SC_NOOP);}

/**** PID Controller Functions *************************/
/*
 * pid_controller()
 */

uint8_t pid_controller()
{
	uint16_t int_temp = adc_read(ADC_CHANNEL);

	dev.temperature_set_point = 500;
	dev.temperature_reading = (double)int_temp;

	if (dev.temperature_reading > dev.temperature_set_point) {
		led_on();
	} else {
		led_off();
	}
	return (SC_OK);
}

/**** Device Init ****
 */
void device_init(void)
{
	DDRB = PORTB_DIR;			// initialize all ports for proper IO function
	DDRC = PORTC_DIR;
	DDRD = PORTD_DIR;

	rtc_init();
	pwm_init();
	adc_init();
	led_on();					// put on the red light (Roxanne)
}

/**** ADC - Analog to Digital Converter for thermocouple reader ****/
/*
 * adc_init() - initialize ADC. See tinyg_tc.h for settings used
 */
void adc_init(void)
{
	ADMUX = ADC_REFS;					// setup ADC Vref
	ADCSRA = ADC_ENABLE | ADC_PRESCALE;	// Enable ADC (bit 7)
}

double adc_read(uint8_t channel)
{
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);// set the channel
	ADCSRA |= ADC_START_CONVERSION;
	while (ADCSRA & (1<<ADSC));			// this takes about 100 uSec
	return (ADC);
}

/**** PWM - Pulse Width Modulation Functions ****/
/*
 * pwm_init() - initialize RTC timers and data
 *
 * 	Configure timer 2 for extruder heater PWM
 *	Mode: 8 bit Fast PWM Fast w/OCR2A setting PWM freq (TOP value)
 *		  and OCR2B setting the duty cycle as a fraction of OCR2A seeting
 */
void pwm_init(void)
{
	TCCR2A  = 0b10100000;		// OC2A non-inverted mode, OC2B non-inverted mode
	TCCR2A |= 0b00000011;		// Waveform generation set to MODE 7 - here...
	TCCR2B  = 0b00001000;		// ...continued here
	TCCR2B |= PWM_PRESCALE_SET;	// set clock and prescaler
	TIMSK1  = 0b00000000; 		// disable PWM interrupts
	OCR2A = 0;					// clear PWM frequency (TOP value)
	OCR2B = 0;					// clear PWM duty cycle as % of TOP value
}

/*
 * pwm_set_freq() - set PWM channel frequency
 *
 *	At current settings the range is from about 500 Hz to about 6000 Hz  
 */

uint8_t pwm_set_freq(double freq)
{
	double f_test = F_CPU / PWM_PRESCALE / freq;
	if (f_test < PWM_MIN_RES) { OCR2A = PWM_MIN_RES;} 
	else if (f_test >= PWM_MAX_RES) { OCR2A = PWM_MAX_RES;} 
	else { OCR2A = (uint8_t)f_test;}
	return (SC_OK);
}

/*
 * pwm_set_duty() - set PWM channel duty cycle 
 *
 *	Setting duty cycle between 0 and 100 enables PWM channel
 *	Setting duty cycle to 0 disables the PWM channel with output low
 *	Setting duty cycle to 100 disables the PWM channel with output high
 *
 *	The frequency must have been set previously
 */

uint8_t pwm_set_duty(double duty)
{
	if (duty < 0)   { return (SC_INPUT_VALUE_TOO_SMALL);}
	if (duty > 100) { return (SC_INPUT_VALUE_TOO_LARGE);}
	OCR2B = (uint8_t)OCR2A * (duty / 100);
	return (SC_OK);
}

/**** RTC - Real Time Clock Functions ****
 * rtc_init() 	  - initialize RTC timers and data
 * ISR()		  - RTC interrupt routine 
 * rtc_callback() - run RTC from dispatch loop
 * rtc_10ms()	  - tasks that run every 10 ms
 * rtc_100ms()	  - tasks that run every 100 ms
 * rtc_1sec()	  - tasks that run every 100 ms
 */
void rtc_init(void)
{
	TCCR0A = 0x00;				// normal mode, no compare values
	TCCR0B = 0x05;				// normal mode, internal clock / 1024 ~= 7800 Hz
	TCNT0 = (256 - RTC_10MS_COUNT);	// set timer for approx 10 ms overflow
	TIMSK0 = (1<<TOIE0);		// enable overflow interrupts
	dev.rtc_100ms_count = 10;
	dev.rtc_1sec_count = 10;	
}

ISR(TIMER0_OVF_vect)
{
	TCNT0 = (256 - RTC_10MS_COUNT);	// reset timer for approx 10 ms overflow
	dev.rtc_flag = true;
}

uint8_t rtc_callback(void)
{
	if (dev.rtc_flag == false) { return (SC_NOOP);}
	dev.rtc_flag = false;

	rtc_10ms();

	if (--dev.rtc_100ms_count != 0) { return (SC_OK);}
	dev.rtc_100ms_count = 10;
	rtc_100ms();

	if (--dev.rtc_1sec_count != 0) { return (SC_OK);}
	dev.rtc_1sec_count = 10;
	rtc_1sec();

	return (SC_OK);
}

void rtc_10ms(void)
{
	return;
}

void rtc_100ms(void)
{
	return;
}

void rtc_1sec(void)
{
//	led_toggle();
	return;
}

/**** LED Functions ****
 * led_on()
 * led_off()
 * led_toggle()
 */

void led_on(void) 
{
	LED_PORT &= ~(LED_PIN);
}

void led_off(void) 
{
	LED_PORT |= LED_PIN;
}

void led_toggle(void) 
{
	if (LED_PORT && LED_PIN) {
		led_on();
	} else {
		led_off();
	}
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

// PWM tests
	
	pwm_set_freq(50000);
	pwm_set_freq(10000);
	pwm_set_freq(5000);
	pwm_set_freq(2500);
	pwm_set_freq(1000);
	pwm_set_freq(500);
	pwm_set_freq(250);
	pwm_set_freq(100);

	pwm_set_freq(1000);
	pwm_set_duty(1000);
	pwm_set_duty(100);
	pwm_set_duty(99);
	pwm_set_duty(75);
/*
	pwm_set_duty(50);
	pwm_set_duty(20);
	pwm_set_duty(10);
	pwm_set_duty(5);
	pwm_set_duty(2);
	pwm_set_duty(1);
	pwm_set_duty(0.1);

	pwm_set_freq(5000);
	pwm_set_duty(1000);
	pwm_set_duty(100);
	pwm_set_duty(99);
	pwm_set_duty(75);
	pwm_set_duty(50);
	pwm_set_duty(20);
	pwm_set_duty(10);
	pwm_set_duty(5);
	pwm_set_duty(2);
	pwm_set_duty(1);
	pwm_set_duty(0.1);
*/
// exception cases

}

#endif // __UNIT_TEST_DEVICE

