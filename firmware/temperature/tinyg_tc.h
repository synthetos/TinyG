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

#ifndef tinyg_tc_h
#define tinyg_tc_h

// Device function prototypes

#define __UNIT_TEST_DEVICE	// uncomment to enable unit tests

void device_init(void);

uint8_t pid_controller(void);

void adc_init(void);
double adc_read(uint8_t channel);

void pwm_init(void);
uint8_t pwm_set_freq(double freq);
uint8_t pwm_set_duty(double duty);

void rtc_init(void);
uint8_t rtc_callback(void);
void rtc_10ms(void);
void rtc_100ms(void);
void rtc_1sec(void);

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

// Device mappings and constants (for atmega328P)

#define SPI_PORT	PORTB		// on-board SPI peripheral
#define SPI_SCK		(1<<PINB5)	// SPI clock line
#define SPI_MISO	(1<<PINB4)	// SPI MISO line
#define SPI_MOSI	(1<<PINB3)	// SPI MOSI line
#define SPI_SS		(1<<PINB2)	// SPI slave select

#define PWM_PORT	PORTD		// Pulse width modulation port
#define PWM_TIMER	TCNT2		// Pulse width modulation timer
#define PWM_OUTB	(1<<PIND3)	// 0C2B timer output bit
#define PWM_F_CPU	F_CPU		// 8 Mhz, nominally (internal RC oscillator)
#define PWM_PRESCALE 64			// corresponds to TCCR2B |= 0b00000100;
#define PWM_PRESCALE_SET 4		// 2=8x, 3=32x, 4=64x, 5=128x, 6=256x
#define PWM_MIN_RES 20			// minimum allowable resolution (20 = 5% duty cycle resolution)
#define PWM_MAX_RES 255			// maximum supported resolution
#define PWM_F_MAX	(F_CPU / PWM_PRESCALE / PWM_MIN_RES)
#define PWM_F_MIN	(F_CPU / PWM_PRESCALE / 256)

#define ADC_PORT	PORTC		// Analog to digital converter channels
#define ADC_CHANNEL 0b00000000	// ADC channel 0 / single-ended in this application (write to ADMUX)
#define ADC_REFS	0b01000000	// AVcc external 5v reference (write to ADMUX)
#define ADC_ENABLE	0b10000000	// write this to ADCSRA to enable the ADC
#define ADC_START_CONVERSION 0b01000000 // write to ADCSRA to start conversion
#define ADC_PRESCALE 6			// 6=64x which is ~125KHz at 8Mhz clock

#define RTC_TIMER	TCNT0		// Real time clock timer
#define RTC_10MS_COUNT 78		// gets 8 Mhz/1024 close to 100 Hz.
#define RTC_TCCRxA	TCCR0A		// map the registers into the selected timer

#define LED_PORT	PORTD		// LED port
#define LED_PIN		(1<<PIND2)	// LED indicator

// Atmega328P data direction defines: 0=input pin, 1=output pin
// These defines therefore only specify output pins

#define PORTB_DIR	(SPI_MISO)	// setup for on-board SPI to work
#define PORTC_DIR	(0)			// no out put bits on C
//#define PORTC_DIR	(ADC_CHAN0)	// used for ACD only
#define PORTD_DIR	(LED_PIN | PWM_OUTB)


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


#ifdef __UNIT_TEST_DEVICE
void device_unit_tests(void);
#define	DEVICE_UNITS device_unit_tests();
#else
#define	DEVICE_UNITS
#endif // __UNIT_TEST_DEVICE


/**** TMC262 specific stuff from here on out ****/

/* Used by tmc262 driver
#define SPI2_PORT 	PORTD		// Secondary SPI is a bit banger 
#define SPI2_CLK	(1<<PIND7)	// Secondary SPI SCK line
#define SPI2_MISO	(1<<PIND6)	// Secondary SPI SDO line
#define SPI2_MOSI	(1<<PIND5)	// Secondary SPI SDI line
#define SPI2_SS		(1<<PIND4)	// Secondary SPI CSN slave select
*/

#endif
