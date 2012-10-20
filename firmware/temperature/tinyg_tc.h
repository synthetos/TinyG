/*
 * tmc262.h - Trinamic TMC262 device definitions and settings
 * Part of Sharkfin project
 *
 * Copyright (c) 2012 Alden S. Hart Jr.
 *
 * Open Controller Bus (OCB) is licensed under the OSHW 1.0 license
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef tmc262_h
#define tmc262_h

// Device configuration 

#define DEVICE_WAIT_TIME 10		// 10 = 100 uSeconds

#define DEVICE_TYPE 	 KINEN_DEVICE_TYPE_STEPPER_CONTROLLER
#define DEVICE_ID_HI	 0x00
#define DEVICE_ID_LO	 0x01
#define DEVICE_REV_MAJOR 0x00
#define DEVICE_REV_MINOR 0x01
#define DEVICE_UUID_1	 0x00	// UUID = 0 means there is no UUID
#define DEVICE_UUID_2	 0x00	// UUID = 0 means there is no UUID
#define DEVICE_UUID_3	 0x00	// UUID = 0 means there is no UUID

// atmega328P Port mapping

#define SPI_PORT	PORTB		// Primary SPI is the on-board SPI peripheral
#define SPI_CLK		(1<<PINB5)	// Primary SPI clock line
#define SPI_MISO	(1<<PINB4)	// Primary SPI MISO line
#define SPI_MOSI	(1<<PINB3)	// Primary SPI MOSI line
#define SPI_SS		(1<<PINB2)	// Primary SPI slave select

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

#define LED_PORT	PORTC		// LED port
#define LED_PIN		(1<<PINC0)	// LED indicator
#define DEV_ACD 	ACD7		// current set ACD line

// Atmega328P data direction: 0=input pin, 1=output pin
// These defines therefore only specify output pins
#define PORTB_DIR	(SPI_MISO)	// setup for on-board SPI to work
#define PORTC_DIR	(LED_PIN)	// All inputs except LED
#define PORTD_DIR	(SPI2_CLK | SPI2_MOSI | SPI2_SS)

// function prototypes

void device_init(void);
void device_reset(void);
uint8_t device_read_byte(uint8_t addr, uint8_t *data);
uint8_t device_write_byte(uint8_t addr, uint8_t data);
void device_led_on(void);
void device_led_off(void);
void device_unit_tests(void);

/**** TMC262 specific stuff from here on out ****/

								// These are fixed by the chip. DO NOT CHANGE
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

// enumerations for the config array
enum tmc262Registers {
	TMC262_TST = 0,
	TMC262_SLPH,
	TMC262_SLPL,
	TMC262_DISS2G,
	TMC262_TS2G,
	TMC262_SDOFF,
	TMC262_VSENSE,
	TMC262_RDSEL,
	TMC262_INTPOL,
	TMC262_DEDGE,
	TMC262_MRES,
	TMC262_TBL,
	TMC262_CHM,
	TMC262_RNDTF,
	TMC262_HDEC,
	TMC262_HEND,
	TMC262_HSTRT,
	TMC262_TOFF,
	TMC262_SEIMIN,
	TMC262_SEDN,
	TMC262_SEMAX,
	TMC262_SEUP,
	TMC262_SEMIN,
	TMC262_SFILT,
	TMC262_SGT,
	TMC262_CS,
	TMC262_ADDRESS_MAX
};

// provide labels for all array elements
#define tmc262_tst tmc262_array[TMC262_TST]
#define tmc262_slph tmc262_array[TMC262_SLPH]
#define tmc262_slpl tmc262_array[TMC262_SLPL]
#define tmc262_diss2g tmc262_array[TMC262_DISS2G]
#define tmc262_ts2g tmc262_array[TMC262_TS2G]
#define tmc262_sdoff tmc262_array[TMC262_SDOFF]
#define tmc262_vsense tmc262_array[TMC262_VSENSE]
#define tmc262_rdsel tmc262_array[TMC262_RDSEL]
#define tmc262_intpol tmc262_array[TMC262_INTPOL]
#define tmc262_dedge tmc262_array[TMC262_DEDGE]
#define tmc262_mres tmc262_array[TMC262_MRES]
#define tmc262_tbl tmc262_array[TMC262_TBL]
#define tmc262_chm tmc262_array[TMC262_CHM]
#define tmc262_rndtf tmc262_array[TMC262_RNDTF]
#define tmc262_hdec tmc262_array[TMC262_HDEC]
#define tmc262_hend tmc262_array[TMC262_HEND]
#define tmc262_hstrt tmc262_array[TMC262_HSTRT]
#define tmc262_toff tmc262_array[TMC262_TOFF]
#define tmc262_seimin tmc262_array[TMC262_SEIMIN]
#define tmc262_sedn tmc262_array[TMC262_SEDN]
#define tmc262_semax tmc262_array[TMC262_SEMAX]
#define tmc262_seup tmc262_array[TMC262_SEUP]
#define tmc262_semin tmc262_array[TMC262_SEMIN]
#define tmc262_sfilt tmc262_array[TMC262_SFILT]
#define tmc262_sgt tmc262_array[TMC262_SGT]
#define tmc262_cs tmc262_array[TMC262_CS]


/* INITIALIZATION TEST PATTERNS FOR PACKING FUNCTION UNIT TESTS

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
