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
#include <math.h>

#include "kinen_core.h"
#include "tinyg_tc.h"

// static functions 
static void _controller(void);
static double _sensor_sample(uint8_t adc_channel, uint8_t new_period);

// static data
static struct Device {
	// tick counter variables
	uint8_t tick_flag;			// true = the timer interrupt fired
	uint8_t tick_100ms_count;	// 100ms down counter
	uint8_t tick_1sec_count;		// 1 second down counter

	// pwm variables
	double pwm_freq;			// save it for stopping and starting PWM
} device;

static struct Heater {
	uint8_t state;				// heater state
	uint8_t code;				// heater code (more information about heater state)
	double temperature;			// current heater temperature
	double setpoint;			// set point for regulation
	double regulation_timer;	// time taken so far to get out of ambinet and to to regulation (seconds)
	double ambient_timeout;		// timeout beyond which regulation has failed (seconds)
	double regulation_timeout;	// timeout beyond which regulation has failed (seconds)
	double ambient_temperature;	// temperature below which it's ambient temperature (heater failed)
	double overheat_temperature;// overheat temperature (cutoff temperature)
} heater;

static struct PIDstruct {		// PID controller itself
	uint8_t state;				// PID state (actually very simple)
	uint8_t code;				// PID code (more information about PID state)
	double temperature;			// current PID temperature
	double setpoint;			// temperature set point
	double error;				// current error term
	double prev_error;			// error term from previous pass
	double integral;			// integral term
	double derivative;			// derivative term
	double dt;					// pid time constant
	double Kp;					// proportional gain
	double Ki;					// integral gain 
	double Kd;					// derivative gain
	double max;					// saturation filter
	double min;
} pid;

static struct TemperatureSensor {
	uint8_t state;				// sensor state
	uint8_t code;				// sensor return code (more information about state)
	uint8_t samples_per_reading;// number of samples to take per reading
	uint8_t samples;			// number of samples taken. Set to 0 to start a reading 
	uint8_t retries;			// number of retries on sampling errors or for shutdown 
	double temperature;			// high confidence temperature reading
	double previous_temp;		// previous temperature for sampling
	double accumulator;			// accumulated temperature reading during sampling (divide by samples)
	double variance;			// range threshold for max allowable change between samples (reject outliers)
	double disconnect_temperature;// false temperature reading indicating thermocouple is disconnected
	double no_power_temperature;// false temperature reading indicating there's no power to thermocouple amplifier
} sensor;

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

	DEVICE_UNITS;				// uncomment __UNIT_TEST_DEVICE to enable unit tests

	heater_on(200);

	while (true) {				// go to the controller loop and never return
		_controller();
	}
	return (false);				// never returns
}


/*
 * Device Init 
 */
void device_init(void)
{
	DDRB = PORTB_DIR;			// initialize all ports for proper IO function
	DDRC = PORTC_DIR;
	DDRD = PORTD_DIR;

	tick_init();
	pwm_init();
	adc_init();
	led_on();					// put on the red light [Sting, 1978]
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
	DISPATCH(kinen_callback());		// intercept low-level communication events
	DISPATCH(tick_callback());		// regular interval timer clock handler (ticks)
//	DISPATCH(heater_fast_loop());	// fast response heater conditions like shutdown
}

/**** Heater Functions ****/
/*
 * heater_on()
 * heater_off()
 * heater_callback() - handle 100ms ticks
 */

void heater_on(double setpoint)
{ 
	// no action if heater is already on
	if ((heater.state == HEATER_HEATING) || (heater.state == HEATER_AT_TARGET)) {
		return;
	}

	// initialize heater and start PID
	memset(&heater, 0, sizeof(struct Heater));
	heater.setpoint = setpoint;
	heater.ambient_timeout = HEATER_AMBIENT_TIMEOUT;
	heater.regulation_timeout = HEATER_REGULATION_TIMEOUT;
	heater.ambient_temperature = HEATER_AMBIENT_TEMPERATURE;
	heater.overheat_temperature = HEATER_OVERHEAT_TEMPERATURE;
	sensor_on();
	pid_on(heater.setpoint, sensor.temperature);
	pwm_set_freq(PWM_FREQUENCY);
	pwm_set_duty(0);			// turn it off
	heater.state = HEATER_HEATING;
}

void heater_off() 
{
	pid_off();
	heater.state = HEATER_OFF;
}

void heater_callback()
{
	// These are the no-op cases
	if ((heater.state == HEATER_OFF) || (heater.state == HEATER_SHUTDOWN)) { return;}

	// Get the current temp and start another reading
	sensor_start_temperature_reading();
	if (sensor_get_state() != SENSOR_HAS_DATA) { return;}
	heater.temperature = sensor_get_temperature();
	double duty_cycle = pid_calc(heater.setpoint, heater.temperature);
	pwm_set_duty(duty_cycle);
	
	// handle HEATER exceptions
	if (heater.state == HEATER_HEATING) {
		heater.regulation_timer += HEATER_TICK_SECONDS;

		if ((heater.temperature < heater.ambient_temperature) &&
			(heater.regulation_timer > heater.ambient_timeout)) {
			heater.state = HEATER_SHUTDOWN;
			heater.code = HEATER_AMBIENT_TIMED_OUT;
			pid_off();
			return;
		}

		if ((heater.temperature < heater.setpoint) &&
			(heater.regulation_timer > heater.regulation_timeout)) {
			heater.state = HEATER_SHUTDOWN;
			heater.code = HEATER_REGULATION_TIMED_OUT;
			pid_off();
			return;
		}
	}
}

/**** Heater PID Functions ****/
/*
 * pid_off()  - turn pid off.
 * pid_on()	  - turn pid on. Initializez and set setpoint
 * pid_calc() - derived from: http://www.embeddedheaven.com/pid-control-algorithm-c-language.htm
 */

void pid_off() 
{ 
	memset(&pid, 0, sizeof(struct PIDstruct));
}

void pid_on(double setpoint, double temperature) 
{
	memset(&pid, 0, sizeof(struct PIDstruct));
	pid.setpoint = setpoint;
	pid.dt = PID_DT;
	pid.Kp = PID_Kp;
	pid.Ki = PID_Ki;
	pid.Kd = PID_Kd;
	pid.max = PID_MAX_OUTPUT;		// saturation filter max value
	pid.min = PID_MIN_OUTPUT;		// saturation filter min value
	pid.state = PID_ON;
}

double pid_calc(double setpoint,double temperature)
{
	if (pid.state == PID_OFF) { return (0);}

	pid.error = setpoint - temperature;	// current error term

	if(fabs(pid.error) > PID_EPSILON) {	// stop integration if error term is too small
		pid.integral = pid.integral + pid.error * pid.dt;
	}
	pid.derivative = (pid.error - pid.prev_error) / pid.dt;
	double output = pid.Kp * pid.error + pid.Ki * pid.integral + pid.Kd * pid.derivative;

	if(output > pid.max) { 		// run saturation Filter
		output = pid.max;
	} else if(output < pid.min) {
		output = pid.min;
	}
	pid.prev_error = pid.error;		// update error term
	return output;
}

/**** Temperature Sensor and Functions ****/
/*
 * sensor_on()	 			- initialize temperature sensor and start it running
 * sensor_off()	 			- turn temperature sensor off
 * sensor_get_temperature()	- return latest temperature reading or LESS _THAN_ZERO
 * sensor_get_state()		- return current sensor state
 * sensor_get_code()		- return latest sensor code
 * sensor_callback() 		- perform sensor sampling
 */

void sensor_on()
{
	memset(&sensor, 0, sizeof(struct TemperatureSensor));
	sensor.samples_per_reading = SENSOR_SAMPLES_PER_READING;
	sensor.temperature = ABSOLUTE_ZERO;
	sensor.retries = SENSOR_RETRIES;
	sensor.variance = SENSOR_VARIANCE_RANGE;
	sensor.disconnect_temperature = SENSOR_DISCONNECTED_TEMPERATURE;
	sensor.no_power_temperature = SENSOR_NO_POWER_TEMPERATURE;
	sensor.state = SENSOR_HAS_NO_DATA;
}

void sensor_off()
{
	sensor.state = SENSOR_OFF;
}

double sensor_get_temperature() { 
	if (sensor.state == SENSOR_HAS_DATA) { 
		return (sensor.temperature);
	} else {
		return (SURFACE_OF_THE_SUN);	// a value that should say "Shut me off! Now!"
	}
}

uint8_t sensor_get_state() { return (sensor.state);}
uint8_t sensor_get_code() { return (sensor.code);}
void sensor_start_temperature_reading() { sensor.samples = 0; }

/*
 * sensor_callback() - perform tick-timer sensor functions (10ms loop)
 *
 *	The sensor_callback() is called on 10ms ticks. It collects N samples in a 
 *	sampling period before updating the sensor.temperature. Since the heater
 *	runs on 100ms ticks there can be a max of 10 samples in a period.
 *	(The ticks are synchronized so you can actually get 10, not just 9) 
 *
 *	The heater must initate a sample cycle by calling sensor_start_sample()
 */

void sensor_callback()
{
	// don't execute the callback if the sensor is uninitialized or shut down
	if ((sensor.state == SENSOR_OFF) || (sensor.state == SENSOR_SHUTDOWN)) { 
		return;
	}

	// take a temperature sample
	uint8_t new_period = false;
	if (sensor.samples == 0) {
		sensor.accumulator = 0;
		new_period = true;
	}
	double temperature = _sensor_sample(ADC_CHANNEL, new_period);
	if (temperature > SURFACE_OF_THE_SUN) {
		sensor.state = SENSOR_SHUTDOWN;
		sensor.code = SENSOR_BAD_READINGS;
		return;
	}
	sensor.accumulator += temperature;

	// return if still in the sampling period
	if ((++sensor.samples) < sensor.samples_per_reading) { 
		return;
	}

	// record the temperature 
	sensor.temperature = sensor.accumulator / sensor.samples;

	// process the completed reading for exception cases
	if (sensor.temperature > SENSOR_DISCONNECTED_TEMPERATURE) {
		sensor.state = SENSOR_HAS_NO_DATA;
		sensor.code = SENSOR_DISCONNECTED;
	} else if (sensor.temperature < SENSOR_NO_POWER_TEMPERATURE) {
		sensor.state = SENSOR_HAS_NO_DATA;
		sensor.code = SENSOR_NO_POWER;
	} else {
		sensor.state = SENSOR_HAS_DATA;
	}
}

/*
 * _sensor_sample() - take a sample and reject samples showing excessive variance
 *
 *	Returns temperature sample if within variance bounds
 *	Returns ABSOLUTE_ZERO if it cannot get a sample within variance
 *	Retries sampling if variance is exceeded - reject spurious readings
 *	To start a new sampling period set 'new_period' true
 *
 * Temperature calculation math
 *
 *	This setup is using B&K TP-29 K-type test probe (Mouser part #615-TP29, $9.50 ea) 
 *	coupled to an Analog Devices AD597 (available from Digikey)
 *
 *	This combination is very linear between 100 - 300 deg-C outputting 7.4 mV per degree
 *	The ADC uses a 5v reference (the 1st major source of error), and 10 bit conversion
 *
 *	The sample value returned by the ADC is computed by ADCvalue = (1024 / Vref)
 *	The temperature derived from this is:
 *
 *		y = mx + b
 *		temp = adc_value * slope + offset
 *
 *		slope = (adc2 - adc1) / (temp2 - temp1)
 *		slope = 0.686645508							// from measurements
 *
 *		b = temp - (adc_value * slope)
 *		b = -4.062500								// from measurements
 *
 *		temp = (adc_value * 1.456355556) - -120.7135972
 */

//#define SAMPLE(a) (((double)adc_read(a) * SENSOR_SLOPE) + SENSOR_OFFSET)
#define SAMPLE(a) (((double)200 * SENSOR_SLOPE) + SENSOR_OFFSET)	// useful for testing the math

static double _sensor_sample(uint8_t adc_channel, uint8_t new_period)
{
	double sample = SAMPLE(adc_channel);

	if (new_period == true) {
		sensor.previous_temp = sample;
		return (sample);
	}
	for (uint8_t i=sensor.retries; i>0; --i) {
		if (fabs(sample - sensor.previous_temp) < sensor.variance) { // sample is within variance range
			sensor.previous_temp = sample;
			return (sample);
		}
		sample = SAMPLE(adc_channel);	// if outside variance range take another sample
	}
	// exit if all variance tests failed. Return a value that should cause the heater to shut down
	return (HOTTER_THAN_THE_SUN);
}


/**** ADC - Analog to Digital Converter for thermocouple reader ****/
/*
 * adc_init() - initialize ADC. See tinyg_tc.h for settings used
 * adc_read() - returns the raw ADC reading. See __sensor_sample notes for explanation
 */
void adc_init(void)
{
	ADMUX  = (ADC_REFS | ADC_CHANNEL);	 // setup ADC Vref and channel 0
	ADCSRA = (ADC_ENABLE | ADC_PRESCALE);// Enable ADC (bit 7) & set prescaler
}

uint16_t adc_read(uint8_t channel)
{
	ADMUX &= 0xF0;						// clobber the channel
	ADMUX |= 0x0F & channel;			// set the channel

	ADCSRA |= ADC_START_CONVERSION;		// start the conversion
	while (ADCSRA && (1<<ADIF) == 0);	// wait about 100 uSec
	ADCSRA |= (1<<ADIF);				// clear the conversion flag
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
	TCCR2A  = PWM_INVERTED;		// alternative is PWM_NON_INVERTED
	TCCR2A |= 0b00000011;		// Waveform generation set to MODE 7 - here...
	TCCR2B  = 0b00001000;		// ...continued here
	TCCR2B |= PWM_PRESCALE_SET;	// set clock and prescaler
	TIMSK1 = 0; 				// disable PWM interrupts
	OCR2A = 0;					// clear PWM frequency (TOP value)
	OCR2B = 0;					// clear PWM duty cycle as % of TOP value
	device.pwm_freq = 0;
}

/*
 * pwm_set_freq() - set PWM channel frequency
 *
 *	At current settings the range is from about 500 Hz to about 6000 Hz  
 */

uint8_t pwm_set_freq(double freq)
{
	device.pwm_freq = F_CPU / PWM_PRESCALE / freq;
	if (device.pwm_freq < PWM_MIN_RES) { 
		OCR2A = PWM_MIN_RES;
	} else if (device.pwm_freq >= PWM_MAX_RES) { 
		OCR2A = PWM_MAX_RES;
	} else { 
		OCR2A = (uint8_t)device.pwm_freq;
	}
	return (SC_OK);
}

/*
 * pwm_set_duty() - set PWM channel duty cycle 
 *
 *	Setting duty cycle between 0 and 100 enables PWM channel
 *	Setting duty cycle to 0 disables the PWM channel with output low
 *	Setting duty cycle to 100 disables the PWM channel with output high
 *
 *	The frequency must have been set previously.
 *
 *	Since I can't seem to get the output pin to work in non-inverted mode
 *	it's done in software in this routine.
 */

uint8_t pwm_set_duty(double duty)
{
	if (duty <= 0) { 
		OCR2B = 255;
	} else if (duty > 100) { 
		OCR2B = 0;
	} else {
		OCR2B = (uint8_t)(OCR2A * (1-(duty/100)));
	}
	OCR2A = (uint8_t)device.pwm_freq;
	return (SC_OK);
}

/**** Tick - Tick tock - Regular Interval Timer Clock Functions ****
 * tick_init() 	  - initialize RIT timers and data
 * RIT ISR()	  - RIT interrupt routine 
 * tick_callback() - run RIT from dispatch loop
 * tick_10ms()	  - tasks that run every 10 ms
 * tick_100ms()	  - tasks that run every 100 ms
 * tick_1sec()	  - tasks that run every 100 ms
 */
void tick_init(void)
{
	TCCR0A = 0x00;				// normal mode, no compare values
	TCCR0B = 0x05;				// normal mode, internal clock / 1024 ~= 7800 Hz
	TCNT0 = (256 - TICK_10MS_COUNT);	// set timer for approx 10 ms overflow
	TIMSK0 = (1<<TOIE0);		// enable overflow interrupts
	device.tick_100ms_count = 10;
	device.tick_1sec_count = 10;	
}

ISR(TIMER0_OVF_vect)
{
	TCNT0 = (256 - TICK_10MS_COUNT);	// reset timer for approx 10 ms overflow
	device.tick_flag = true;
}

uint8_t tick_callback(void)
{
	if (device.tick_flag == false) { return (SC_NOOP);}
	device.tick_flag = false;

	tick_10ms();

	if (--device.tick_100ms_count != 0) { return (SC_OK);}
	device.tick_100ms_count = 10;
	tick_100ms();

	if (--device.tick_1sec_count != 0) { return (SC_OK);}
	device.tick_1sec_count = 10;
	tick_1sec();

	return (SC_OK);
}

void tick_10ms(void)
{
	sensor_callback();			// run the temperature sensor every 10 ms.
}

void tick_100ms(void)
{
	heater_callback();			// run the heater controller every 100 ms.
}

void tick_1sec(void)
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
 *	at the device level for any Kinen device
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

