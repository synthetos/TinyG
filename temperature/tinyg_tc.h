/*
 * tinyg_tc.h - TinyG temperature controller - Kinen device
 * Part of Kinen project
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

//#define __UNIT_TEST_TC	// uncomment to enable unit tests
//#define __TEST

void device_init(void);
void device_reset(void);
uint8_t device_read_byte(uint8_t addr, uint8_t *data);
uint8_t device_write_byte(uint8_t addr, uint8_t data);

void heater_init(void);
void heater_on(double setpoint);
void heater_off(uint8_t state, uint8_t code);
void heater_callback(void);

void sensor_init(void);
void sensor_on(void);
void sensor_off(void);
void sensor_start_reading(void);
uint8_t sensor_get_state(void);
uint8_t sensor_get_code(void);
double sensor_get_temperature(void);
void sensor_callback(void);

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

/**** Device configuration for Kinen (+++this needs to change) ****/

#define DEVICE_WAIT_TIME 10				// 10 = 100 uSeconds

#define DEVICE_TYPE 	 DEVICE_TYPE_TEMPERATURE_CONTROLLER
#define DEVICE_ID_HI	 0x00
#define DEVICE_ID_LO	 0x01
#define DEVICE_REV_MAJOR 0x00
#define DEVICE_REV_MINOR 0x01
#define DEVICE_UUID_1	 0x00			// UUID = 0 means there is no UUID
#define DEVICE_UUID_2	 0x00			// UUID = 0 means there is no UUID
#define DEVICE_UUID_3	 0x00			// UUID = 0 means there is no UUID

/**** Heater default parameters ***/

#define HEATER_TICK_SECONDS 0.1			// 100 ms
#define HEATER_AMBIENT_TEMPERATURE 40	// detect heater not heating if readings stay below this temp
#define HEATER_OVERHEAT_TEMPERATURE 300	// heater is above max temperature if over this temp. Should shut down
#define HEATER_AMBIENT_TIMEOUT 90		// time to allow heater to heat above ambinet temperature (seconds)
#define HEATER_REGULATION_TIMEOUT 300	// time to allow heater to come to temp (seconds)
#define HEATER_REGULATION_COUNT 10		// number of successive readings before declaring AT_TARGET

enum tcHeaterState {					// heater state machine
	HEATER_OFF = 0,						// heater turned OFF or never turned on - transitions to HEATING
	HEATER_SHUTDOWN,					// heater has been shut down - transitions to HEATING
	HEATER_HEATING,						// heating up from OFF or SHUTDOWN - transitions to REGULATED or SHUTDOWN
	HEATER_REGULATED					// heater is at setpoint and in regulation - transitions to OFF or SHUTDOWN
};

enum tcHeaterCode {
	HEATER_OK = 0,						// heater is OK - no errors reported
	HEATER_AMBIENT_TIMED_OUT,			// heater failed to get past ambient temperature
	HEATER_REGULATION_TIMED_OUT,		// heater heated but failed to achieve regulation before timeout
	HEATER_OVERHEATED,					// heater exceeded maximum temperature cutoff value
	HEATER_SENSOR_ERROR					// heater encountered a fatal sensor error
};

/**** PID default parameters ***/

#define PID_DT HEATER_TICK_SECONDS		// time constant for computation
#define PID_EPSILON 0.01				// error term precision
#define PID_MAX_OUTPUT 100				// saturation filter max PWM percent
#define PID_MIN_OUTPUT 0				// saturation filter min PWM percent

										// starting values from example code
//#define PID_Kp 0.1					// proportional gain term
//#define PID_Ki 0.005					// integral gain term
//#define PID_Kd 0.01					// derivative gain term

#define PID_Kp 0.5						// proportional gain term
#define PID_Ki 0.001					// integral gain term
#define PID_Kd 0.01						// derivative gain term

enum tcPIDState {						// PID state machine
	PID_OFF = 0,						// PID is off
	PID_ON
};

/**** Sensor default parameters ***/

#define SENSOR_SAMPLES 9				// number of sensor samples to take for each reading period
#define SENSOR_SAMPLE_VARIANCE_MAX 1.25	// number of standard deviations from mean to reject a sample
#define SENSOR_READING_VARIANCE_MAX 20	// reject entire reading if std_dev exceeds this amount
#define SENSOR_NO_POWER_TEMPERATURE -2	// detect thermocouple amplifier disconnected if readings stay below this temp
#define SENSOR_DISCONNECTED_TEMPERATURE 400	// sensor is DISCONNECTED if over this temp (works w/ both 5v and 3v refs)
#define SENSOR_TICK_SECONDS 0.01		// 10 ms

#define SENSOR_SLOPE 0.489616568		// derived from AD597 chart between 80 deg-C and 300 deg-C
#define SENSOR_OFFSET -0.419325433		// derived from AD597 chart between 80 deg-C and 300 deg-C

#define SURFACE_OF_THE_SUN 5505			// termperature at the surface of the sun in Celcius
#define HOTTER_THAN_THE_SUN 10000		// a temperature that is hotter than the surface of the sun
#define ABSOLUTE_ZERO -273.15			// Celcius
#define LESS_THAN_ZERO -274				// Celcius - a value the thermocouple sensor cannot output

enum tcSensorState {					// main state machine
	SENSOR_OFF = 0,						// sensor is off or uninitialized
	SENSOR_NO_DATA,						// interim state before first reading is complete
	SENSOR_ERROR,						// a sensor error occurred. Don't use the data
	SENSOR_HAS_DATA						// sensor has valid data
};

enum tcSensorCode {						// success and failure codes
	SENSOR_IDLE = 0,					// sensor is idling
	SENSOR_TAKING_READING,				// sensor is taking samples for a reading
	SENSOR_BAD_READINGS,				// ERROR: too many number of bad readings
	SENSOR_DISCONNECTED,				// ERROR: thermocouple detected as disconnected
	SENSOR_NO_POWER						// ERROR: detected lack of power to thermocouple amplifier
};

/**** Lower-level device mappings and constants (for atmega328P) ****/

#define SPI_PORT		PORTB			// on-board SPI peripheral is hard-wired to PORTB
#define SPI_SCK			(1<<PINB5)		// SPI clock line
#define SPI_MISO		(1<<PINB4)		// SPI MISO line
#define SPI_MOSI		(1<<PINB3)		// SPI MOSI line
#define SPI_SS			(1<<PINB2)		// SPI slave select

#define PWM_PORT		PORTD			// Pulse width modulation port
#define PWM_OUTB		(1<<PIND3)		// 0C2B timer output bit
#define PWM_TIMER		TCNT2			// Pulse width modulation timer
#define PWM_NONINVERTED	0xC0			// OC2A non-inverted mode, OC2B non-inverted mode
#define PWM_INVERTED 	0xF0			// OC2A inverted mode, OC2B inverted mode
#define PWM_PRESCALE 	64				// corresponds to TCCR2B |= 0b00000100;
#define PWM_PRESCALE_SET 4				// 2=8x, 3=32x, 4=64x, 5=128x, 6=256x
#define PWM_MIN_RES 	20				// minimum allowable resolution (20 = 5% duty cycle resolution)
#define PWM_MAX_RES 	255				// maximum supported resolution
#define PWM_F_CPU		F_CPU			// 8 Mhz, nominally (internal RC oscillator)
#define PWM_F_MAX		(F_CPU / PWM_PRESCALE / PWM_MIN_RES)
#define PWM_F_MIN		(F_CPU / PWM_PRESCALE / 256)
#define PWM_FREQUENCY 	1000			// set PWM operating frequency

#define ADC_PORT		PORTC			// Analog to digital converter channels
#define ADC_CHANNEL 	0				// ADC channel 0 / single-ended in this application (write to ADMUX)
#define ADC_REFS		0b01000000		// AVcc external 5v reference (write to ADMUX)
#define ADC_ENABLE		(1<<ADEN)		// write this to ADCSRA to enable the ADC
#define ADC_START_CONVERSION (1<<ADSC)	// write to ADCSRA to start conversion
#define ADC_PRESCALE 	6				// 6=64x which is ~125KHz at 8Mhz clock
#define ADC_PRECISION 	1024			// change this if you go to 8 bit precision
#define ADC_VREF 		5.00			// change this if the circuit changes. 3v would be about optimal

#define TICK_TIMER		TCNT0			// Tickclock timer
#define TICK_10MS_COUNT 78				// gets 8 Mhz/1024 close to 100 Hz.
#define TICK_TCCRxA		TCCR0A			// map the registers into the selected timer

#define LED_PORT		PORTD			// LED port
#define LED_PIN			(1<<PIND2)		// LED indicator

// some defs left over from defining a secondary SPI channel
//#define SPI2_PORT 	PORTD			// Secondary SPI is a bit banger 
//#define SPI2_CLK		(1<<PIND7)		// Secondary SPI SCK line
//#define SPI2_MISO		(1<<PIND6)		// Secondary SPI SDO line
//#define SPI2_MOSI		(1<<PIND5)		// Secondary SPI SDI line
//#define SPI2_SS		(1<<PIND4)		// Secondary SPI CSN slave select

// Atmega328P data direction defines: 0=input pin, 1=output pin
// These defines therefore only specify output pins

#define PORTB_DIR	(SPI_MISO)			// setup for on-board SPI to work
#define PORTC_DIR	(0)					// no output bits on C
#define PORTD_DIR	(LED_PIN | PWM_OUTB)// set LED and PWM bits as outputs

// Device configuration and communication registers

// enumerations for the device configuration array
enum deviceRegisters {
	DEVICE_TEMP_STATE = 0,				// temperature regulation state. See TEMP_STATE enumerations
	DEVICE_TEMP_SET_HI,					// temperature set point hi - Celcius 
	DEVICE_TEMP_SET_LO,					// temperature set point lo - Celcius
	DEVICE_TEMP_SET_FRACTION,			// temperature set point fractions - Celcius (you wish)
	DEVICE_TEMP_HI,						// temperature reading hi - Celcius 
	DEVICE_TEMP_LO,						// temperature reading lo - Celcius
	DEVICE_TEMP_FRACTION,				// temperature reading fractions - Celcius (you wish)
	DEVICE_PWM_FREQ_HI,					// device pulse width modulation frequency - hi
	DEVICE_PWM_FREQ_LO,					// device pulse width modulation frequency - lo
	DEVICE_PWM_DUTY_CYCLE,				// device pulse width modulation duty cycle - integer part
	DEVICE_PWM_DUTY_CYCLE_FRACTION,		// device pulse width modulation duty cycle - fractional part

	DEVICE_ADDRESS_MAX					// MUST BE LAST
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


/**** Data structures ****/
// I prefer these to be static in the main file itself but the scope needs to 
// be made global to allow the report.c functions to get at the variables

struct DeviceStruct {					// hardware devices that are part of the chip
	uint8_t tick_flag;			// true = the timer interrupt fired
	uint8_t tick_100ms_count;	// 100ms down counter
	uint8_t tick_1sec_count;	// 1 second down counter
	double pwm_freq;			// save it for stopping and starting PWM
	uint8_t array[DEVICE_ADDRESS_MAX]; // byte array for Kinen communications
};
typedef struct DeviceStruct Device;

struct HeaterStruct {
	uint8_t state;				// heater state
	uint8_t code;				// heater code (more information about heater state)
	uint8_t led_toggler;
	int8_t readout;
	uint8_t regulation_count;	// number of successive readings before heater is declared in regulation
	double temperature;			// current heater temperature
	double setpoint;			// set point for regulation
	double regulation_timer;	// time taken so far to get out of ambinet and to to regulation (seconds)
	double ambient_timeout;		// timeout beyond which regulation has failed (seconds)
	double regulation_timeout;	// timeout beyond which regulation has failed (seconds)
	double ambient_temperature;	// temperature below which it's ambient temperature (heater failed)
	double overheat_temperature;// overheat temperature (cutoff temperature)
};
typedef struct HeaterStruct Heater;

struct PIDstruct {				// PID controller itself
	uint8_t state;				// PID state (actually very simple)
	uint8_t code;				// PID code (more information about PID state)
	double output;				// also used for anti-windup on integral term
	double output_max;			// saturation filter max
	double output_min;			// saturation filter min
	double error;				// current error term
	double prev_error;			// error term from previous pass
	double integral;			// integral term
	double derivative;			// derivative term
	double dt;					// pid time constant
	double Kp;					// proportional gain
	double Ki;					// integral gain 
	double Kd;					// derivative gain
	// for test only
	double temperature;			// current PID temperature
	double setpoint;			// temperature set point
};
typedef struct PIDstruct PID;

struct SensorStruct {
	uint8_t state;				// sensor state
	uint8_t code;				// sensor return code (more information about state)
	uint8_t sample_idx;			// index into sample array
	double temperature;			// high confidence temperature reading
	double std_dev;				// standard deviation of sample array
	double sample_variance_max;	// sample deviation above which to reject a sample
	double reading_variance_max;// standard deviation to reject the entire reading
	double disconnect_temperature;	// bogus temperature indicates thermocouple is disconnected
	double no_power_temperature;	// bogus temperature indicates no power to thermocouple amplifier
	double sample[SENSOR_SAMPLES];	// array of sensor samples in a reading
	double test;
};
typedef struct SensorStruct Sensor;

// structure allocations
Device device;					// Device is always a singleton (there is only one device)
Heater heater;					// allocate one heater...
PID pid;						// ...with one PID channel...
Sensor sensor;					// ...and one sensor channel

#ifdef __UNIT_TEST_TC
void device_unit_tests(void);
#define	UNIT_TESTS device_unit_tests();
#else
#define	UNIT_TESTS
#endif // __UNIT_TEST_TC

#endif
