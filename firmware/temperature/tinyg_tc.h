/*
 * tinyg_tc.h - TinyG temperature controller - Kinen device
 * Part of TinyG project
 *
 * Copyright (c) 2012 Alden S. Hart Jr.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* Special thanks to Adam Mayer and the Replicator project for heater guidance
 */
#ifndef tinyg_tc_h
#define tinyg_tc_h

// Device function prototypes

#define __UNIT_TEST_TC	// uncomment to enable unit tests

void device_init(void);

void heater_init(void);
void heater_on(double setpoint);
void heater_off(uint8_t state, uint8_t code);
void heater_callback(void);

void sensor_init(void);
void sensor_on(void);
void sensor_off(void);
void sensor_callback(void);
double sensor_get_temperature(void);
uint8_t sensor_get_state(void);
uint8_t sensor_get_code(void);
void sensor_start_temperature_reading(void);

void pid_init();
void pid_reset();
double pid_calculate(double setpoint,double temperature);

void adc_init(void);
uint16_t adc_read(uint8_t channel);

void pwm_init(void);
void pwm_on(double freq, double duty);
void pwm_off(void);
uint8_t pwm_set_freq(double freq);
uint8_t pwm_set_duty(double duty);

void tick_init(void);
uint8_t tick_callback(void);
void tick_10ms(void);
void tick_100ms(void);
void tick_1sec(void);

void led_on(void);
void led_off(void);
void led_toggle(void);

void device_reset(void);
uint8_t device_read_byte(uint8_t addr, uint8_t *data);
uint8_t device_write_byte(uint8_t addr, uint8_t data);

// Device configuration 

#define DEVICE_WAIT_TIME 10		// 10 = 100 uSeconds

#define DEVICE_TYPE 	 DEVICE_TYPE_TEMPERATURE_CONTROLLER
#define DEVICE_ID_HI	 0x00
#define DEVICE_ID_LO	 0x01
#define DEVICE_REV_MAJOR 0x00
#define DEVICE_REV_MINOR 0x01
#define DEVICE_UUID_1	 0x00	// UUID = 0 means there is no UUID
#define DEVICE_UUID_2	 0x00	// UUID = 0 means there is no UUID
#define DEVICE_UUID_3	 0x00	// UUID = 0 means there is no UUID


/**** Heater default parameters ***/

#define HEATER_AMBIENT_TEMPERATURE 40	// detect heater not heating if readings stay below this temp
#define HEATER_OVERHEAT_TEMPERATURE 300	// heater is above max temperature if over this temp. Should shut down
#define HEATER_AMBIENT_TIMEOUT 90		// time to allow heater to heat above ambinet temperature (seconds)
#define HEATER_REGULATION_TIMEOUT 300	// time to allow heater to come to temp (seconds)
#define HEATER_TARGET_CHECK_COUNT 5		// number of successive readings before declaring AT_TEMPERATURE
#define HEATER_TICK_SECONDS 0.1			// 100 ms

enum tcHeaterState {					// heater state machine
	HEATER_OFF = 0,						// heater turned OFF or never turned on - transitions to HEATING
	HEATER_SHUTDOWN,					// heater has been shut down - transitions to HEATING
	HEATER_HEATING,						// heating up from OFF or SHUTDOWN - transitions to AT_TAGRGET or SHUTDOWN
	HEATER_AT_TARGET					// at setpoint and in temperature regulation - transitions to OFF or SHUTDOWN
};

enum tcHeaterCode {
	HEATER_OK = 0,						// heater is OK - no errors reported
	HEATER_AMBIENT_TIMED_OUT,			// heater failed to get past ambient temperature
	HEATER_REGULATION_TIMED_OUT,		// heater heated but failed to achieve regulation before timeout
	HEATER_OVERHEATED,					// heater exceeded maximum temperature cutoff value
};

/**** PID default parameters ***/

#define PID_DT HEATER_TICK_SECONDS		// time constant for computation
#define PID_EPSILON 0.01				// error term precision
#define PID_MAX_OUTPUT 100				// saturation filter max
#define PID_MIN_OUTPUT 0				// saturation filter min

//#define PID_Kp 0.1					// proportional
//#define PID_Ki 0.005					// integral
//#define PID_Kd 0.01					// derivative

#define PID_Kp 0.5						// proportional
#define PID_Ki 0.005						// integral
#define PID_Kd 0.01						// derivative

enum tcPIDState {						// PID state machine
	PID_OFF = 0,						// PID is off
	PID_ON
};

/**** Sensor default parameters ***/

#define SENSOR_SAMPLES_PER_READING 8	// number of sensor samples to take for each reading period
#define SENSOR_RETRIES 4				// number of sequential sensor errors before rejecting sample or shutting down
#define SENSOR_VARIANCE_RANGE 20		// reject sample if termperature is GT or LT previous sample by this amount 
#define SENSOR_NO_POWER_TEMPERATURE 5	// detect thermocouple amplifier disconnected if readings stay below this temp
#define SENSOR_DISCONNECTED_TEMPERATURE 400	// sensor is DISCONNECTED if over this temp (works w/ both 5v and 3v refs)
#define SENSOR_TICK_SECONDS 0.01		// 10 ms

#define SENSOR_SLOPE 0.686645508		// emperically determined for AD597 & B&K TP-29 K-type test probe
#define SENSOR_OFFSET -4.062500			// emperically determined

#define SURFACE_OF_THE_SUN 5505			// termperature at the surface of the sun in Celcius
#define HOTTER_THAN_THE_SUN 10000		// a temperature that is hotter than the surface of the sun
#define ABSOLUTE_ZERO -273.15			// Celcius
#define LESS_THAN_ZERO -274				// a value the thermocouple sensor cannot output

enum tcSensorState {					// sensor state machine
										// sensor values should only be trusted for HAS_DATA
	SENSOR_OFF = 0,						// sensor is uninitialized (initial state)
	SENSOR_SHUTDOWN,					// sensor is shut down and signalling heater to do the same
	SENSOR_HAS_NO_DATA,					// sensor has been initialized but there is no data
	SENSOR_STALE_DATA,					// sensor data is stale (time constant stuff)
	SENSOR_HAS_DATA						// sensor has valid data (completed a sampling period)
};

enum tcSensorCode {						// success and failure codes. Any failure should cause heater shutdown
	SENSOR_IDLE = 0,					// sensor is OK - no errors reported
	SENSOR_IS_READING,					// sensor is in reading period
	SENSOR_READING_COMPLETE,			// reading is complete.
	SENSOR_READING_FAILED_NO_POWER,		// detected lack of power to thermocouple amplifier
	SENSOR_READING_FAILED_DISCONNECTED,	// thermocouple detected as disconnected
	SENSOR_READING_FAILED_BAD_READINGS	// too many number of bad readings
};

// Lower-level device mappings and constants (for atmega328P)

#define SPI_PORT	PORTB		// on-board SPI peripheral
#define SPI_SCK		(1<<PINB5)	// SPI clock line
#define SPI_MISO	(1<<PINB4)	// SPI MISO line
#define SPI_MOSI	(1<<PINB3)	// SPI MOSI line
#define SPI_SS		(1<<PINB2)	// SPI slave select

#define PWM_PORT	PORTD		// Pulse width modulation port
#define PWM_OUTB	(1<<PIND3)	// 0C2B timer output bit
#define PWM_TIMER	TCNT2		// Pulse width modulation timer
#define PWM_F_CPU	F_CPU		// 8 Mhz, nominally (internal RC oscillator)
#define PWM_PRESCALE 64			// corresponds to TCCR2B |= 0b00000100;
#define PWM_PRESCALE_SET 4		// 2=8x, 3=32x, 4=64x, 5=128x, 6=256x
#define PWM_MIN_RES 20			// minimum allowable resolution (20 = 5% duty cycle resolution)
#define PWM_MAX_RES 255			// maximum supported resolution
#define PWM_F_MAX	(F_CPU / PWM_PRESCALE / PWM_MIN_RES)
#define PWM_F_MIN	(F_CPU / PWM_PRESCALE / 256)
#define PWM_FREQUENCY 1000		// set PWM operating frequency
#define PWM_NON_INVERTED 0xC0	// OC2A non-inverted mode, OC2B non-inverted mode
#define PWM_INVERTED 0xF0		// OC2A inverted mode, OC2B inverted mode

#define ADC_PORT	PORTC		// Analog to digital converter channels
#define ADC_CHANNEL 0			// ADC channel 0 / single-ended in this application (write to ADMUX)
#define ADC_REFS	0b01000000	// AVcc external 5v reference (write to ADMUX)
#define ADC_ENABLE	(1<<ADEN)	// write this to ADCSRA to enable the ADC
#define ADC_START_CONVERSION (1<<ADSC) // write to ADCSRA to start conversion
#define ADC_PRESCALE 6			// 6=64x which is ~125KHz at 8Mhz clock
#define ADC_PRECISION 1024		// change this if you go to 8 bit precision
#define ADC_VREF 5.00			// change this if the circuit changes. 3v would be about optimal

#define TICK_TIMER	TCNT0		// Tickclock timer
#define TICK_10MS_COUNT 78		// gets 8 Mhz/1024 close to 100 Hz.
#define TICK_TCCRxA	TCCR0A		// map the registers into the selected timer

#define LED_PORT	PORTD		// LED port
#define LED_PIN		(1<<PIND2)	// LED indicator

// Atmega328P data direction defines: 0=input pin, 1=output pin
// These defines therefore only specify output pins

#define PORTB_DIR	(SPI_MISO)	// setup for on-board SPI to work
#define PORTC_DIR	(0)			// no output bits on C
#define PORTD_DIR	(LED_PIN | PWM_OUTB) // LED and PWM out

// Device configiuration and communication registers

// enumerations for the device configuration array
enum deviceRegisters {
	DEVICE_TEMP_STATE = 0,		// temperature regulation state. See TEMP_STATE enumerations
	DEVICE_TEMP_SET_HI,			// temperature set point hi - Celcius 
	DEVICE_TEMP_SET_LO,			// temperature set point lo - Celcius
	DEVICE_TEMP_SET_FRACTION,	// temperature set point fractions - Celcius (you wish)
	DEVICE_TEMP_HI,				// temperature reading hi - Celcius 
	DEVICE_TEMP_LO,				// temperature reading lo - Celcius
	DEVICE_TEMP_FRACTION,		// temperature reading fractions - Celcius (you wish)
	DEVICE_PWM_FREQ_HI,			// device pulse width modulation frequency - hi
	DEVICE_PWM_FREQ_LO,			// device pulse width modulation frequency - lo
	DEVICE_PWM_DUTY_CYCLE,		// device pulse width modulation duty cycle - integer part
	DEVICE_PWM_DUTY_CYCLE_FRACTION,// device pulse width modulation duty cycle - fractional part

	DEVICE_ADDRESS_MAX			// MUST BE LAST
};

// provide labels for all array elements
#define device_temp_state device_array[DEVICE_TEMP_STATE]
#define device_temp_set_hi device_array[DEVICE_TEMP_SET_HI]
#define device_temp_set_lo device_array[DEVICE_TEMP_SET_LO]
#define device_temp_set_fraction device_array[DEVICE_TEMP_SET_FRACTION]
#define device_temp_hi device_array[DEVICE_TEMP_HI]
#define device_temp_lo device_array[DEVICE_TEMP_LO]
#define device_temp_fraction device_array[DEVICE_TEMP_FRACTION]
#define device_pwm_freq_hi device_array[DEVICE_PWM_FREQ_HI]
#define device_pwm_freq_lo device_array[DEVICE_PWM_FREQ_LO]
#define device_pwm_freq_fraction device_array[DEVICE_PWM_FREQ_FRACTION]
#define device_pwm_cuty_cycle device_array[DEVICE_PWM_DUTY_CYCLE]


#ifdef __UNIT_TEST_TC
void device_unit_tests(void);
#define	UNIT_TESTS device_unit_tests();
#else
#define	UNIT_TESTS
#endif // __UNIT_TEST_TC


/**** TMC262 specific stuff from here on out ****/

/* Used by tmc262 driver
#define SPI2_PORT 	PORTD		// Secondary SPI is a bit banger 
#define SPI2_CLK	(1<<PIND7)	// Secondary SPI SCK line
#define SPI2_MISO	(1<<PIND6)	// Secondary SPI SDO line
#define SPI2_MOSI	(1<<PIND5)	// Secondary SPI SDI line
#define SPI2_SS		(1<<PIND4)	// Secondary SPI CSN slave select
*/

#endif
