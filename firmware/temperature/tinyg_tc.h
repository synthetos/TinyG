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

#define STEP_ENN	(1<<PINB0)	// Stepper enable line
#define STEP_DIR	(1<<PIND3)	// Stepper direction line
#define STEP_STEP	(1<<PIND2)	// Stepper step line

#define JUMPER_1	(1<<PINC2)	// Jumper position 1 
#define JUMPER_2	(1<<PINC3)	// Jumper position 2
#define JUMPER_3	(1<<PINC4)	// Jumper position 3
#define JUMPER_4	(1<<PINC5)	// Jumper position 4
#define JUMPER_5	(1<<PIND0)	// Jumper position 5 (atmega328P RX line)
#define JUMPER_6	(1<<PIND1)	// Jumper position 6 (atmega328P RX line)
#define JUMPER_SS	(1<<PINB2)	// Primary slave select brought out to jumper 
*/

/*								// These are fixed by the chip. DO NOT CHANGE
#define DRVCONF_ADDR  0x07		// DRVCONF register address
#define DRVCTRL_ADDR  0x00		// DRVCTRL register address - for Step/Dir mode (SDOFF=0)
#define CHOPCONF_ADDR 0x04		// CHOPCONF register address
#define SMARTEN_ADDR  0x05		// SMARTEN register address
#define SGCSCONF_ADDR 0x06		// SGCSCONF register address

// tmc262 configuration initialization values

// (Note: mind your b's and x's )
// DRVCONF settings - driver config (page 24)
#define INIT262_TST 	0b0		// [0]	0=normal operation (not test mode)
#define INIT262_SLPH 	0b00	// [00] slope control, high side
#define INIT262_SLPL 	0b00	// [00] slope control, low side
#define INIT262_DISS2G	0b1		// [1] 	0=enable short to ground protection
#define INIT262_TS2G 	0b00	// [00]	00=slowest short to ground detection time
#define INIT262_SDOFF 	0b0		// [0]	0=enable step/dir operation
#define INIT262_VSENSE 	0b1		// [1]	0=full sense resistor voltage scaling (1=1/2)
#define INIT262_RDSEL 	0b10	// [10]	readout stallguard level & coolstep level
								// [00]	readout microstep value
								// [01]	readout stallguard level

// DRVCTRL settings - for Step/Dir mode (SDOFF=0) (page 20)
#define INIT262_INTPOL 	0x0		// [0]	1=enable step pulse multiplication by 16
#define INIT262_DEDGE 	0x0		// [0]	1=both rising and falling pulse edges active
#define INIT262_MRES 	0x05	// 		1000 = full step microstep resolution
								//		0111 = half step microstep resolution
								//		0110 = quarter step microstep resolution
								//		0101 = eighth step microstep resolution
								//		0100 = 16th step microstep resolution
								//		0011 = 32nd step microstep resolution
								//		0010 = 64th step microstep resolution

// CHOPCONF settings - chopper configuration (page 21)
#define INIT262_TBL 	0b11	// [11]	  blanking time in clock periods (best at 11)	
#define INIT262_CHM 	0b0		// [0]	  0=spread cycle chopper mode
#define INIT262_RNDTF 	0b1		// [1]	  1=randomize TOFF time
#define INIT262_HDEC 	0b01	// [01]	  hysteresis decrement interval
#define INIT262_HEND 	0x03	// [0011] hysteresis end value
#define INIT262_HSTRT 	0x00	// [000]  hysteresis start value
#define INIT262_TOFF 	0x01	// [0001] Off time, 0000 = MOSFET disable

// SMARTEN settings - coolstep control register (page 22)
#define INIT262_SEIMIN	0b0		// [0]	  minimum coolstep current
#define INIT262_SEDN	0b00	// [00]	  current decrement speed
#define INIT262_SEMAX	0x0F	// [1111] upper coolstep threshold
#define INIT262_SEUP	0b00	// [00]	  current increment size
#define INIT262_SEMIN	0x00	// [0000] lower coolstep threshold; 0000=disabled

// SGCSCONF settings - stallguard control register (page 23)
#define INIT262_SFILT	0b1		// [1]		1=filter (more accurate, but slower response)
#define INIT262_SGT		0x0F	// [0x0F]	stallguard threshold (decimal -12 to +63)
#define INIT262_CS		0x1F	// [11111]	current scale, 11111 = maximum



// INITIALIZATION TEST PATTERNS FOR PACKING FUNCTION UNIT TESTS

// DRVCONF settings - driver config (page 24)
#define INIT262_TST 	0b1		// [0]	0=normal operation (not test mode)
#define INIT262_SLPH 	0b11	// [00] slope control, high side
#define INIT262_SLPL 	0b11	// [00] slope control, low side
#define INIT262_DISS2G	0b1		// [1] 	0=enable short to ground protection
#define INIT262_TS2G 	0b11	// [00]	00=slowest short to ground detection time
#define INIT262_SDOFF 	0b1		// [0]	0=enable step/dir operation
#define INIT262_VSENSE 	0b1		// [1]	0=full sense resistor voltage scaling (1=1/2)
#define INIT262_RDSEL 	0b11	// [10]	readout stallguard level & coolstep level
								// [00]	readout microstep value
								// [01]	readout stallguard level

// DRVCTRL settings - for Step/Dir mode (SDOFF=0) (page 20)
#define INIT262_INTPOL 	0x1		// [0]	1=enable step pulse multiplication by 16
#define INIT262_DEDGE 	0x1		// [0]	1=both rising and falling pulse edges active
#define INIT262_MRES 	0x0F	// 		1000 = full step microstep resolution
								//		0111 = half step microstep resolution
								//		0110 = quarter step microstep resolution
								//		0101 = eighth step microstep resolution
								//		0100 = 16th step microstep resolution
								//		0011 = 32nd step microstep resolution
								//		0010 = 64th step microstep resolution

// CHOPCONF settings - chopper configuration (page 21)
#define INIT262_TBL 	0b11	// [11]	  blanking time in clock periods (best at 11)	
#define INIT262_CHM 	0b1		// [0]	  0=spread cycle chopper mode
#define INIT262_RNDTF 	0b1		// [1]	  1=randomize TOFF time
#define INIT262_HDEC 	0b11	// [01]	  hysteresis decrement interval
#define INIT262_HEND 	0x0F	// [0011] hysteresis end value
#define INIT262_HSTRT 	0x07	// [000]  hysteresis start value
#define INIT262_TOFF 	0x0F	// [0001] Off time, 0000 = MOSFET disable

// SMARTEN settings - coolstep control register (page 22)
#define INIT262_SEIMIN	0b1		// [0]	  minimum coolstep current
#define INIT262_SEDN	0b11	// [00]	  current decrement speed
#define INIT262_SEMAX	0x0F	// [1111] upper coolstep threshold
#define INIT262_SEUP	0b11	// [00]	  current increment size
#define INIT262_SEMIN	0x0F	// [0000] lower coolstep threshold; 0000=disabled

// SGCSCONF settings - stallguard control register (page 23)
#define INIT262_SFILT	0b1		// [1]	   1=filter (more accurate, but slower response)
#define INIT262_SGT		0x7F	// []	   stallguard threshold (decimal -12 to +63)
#define INIT262_CS		0x1F	// [11111] current scale, 11111 = maximum

*/

#endif
