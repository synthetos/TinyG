/*
 * tmc262.c - Trinamic TMC262 device driver
 * Part of Sharkfin project
 * Based on Open Controller Bus 
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

#include <stdio.h>
#include <stdbool.h>
#include <string.h>				// for memset
#include <avr/io.h>

#include "ocb.h"
#include "tmc262.h"

// local data structures 

static uint8_t tmc262_array[TMC262_ADDRESS_MAX];

static struct DeviceSingleton {
	uint32_t drvconf;			// word for loading TMC262
	uint32_t drvctrl;			// word for loading TMC262
	uint32_t chopconf;			// word for loading TMC262
	uint32_t smarten;			// word for loading TMC262
	uint32_t sgcsconf;			// word for loading TMC262
} dev;

// local functions (statics)
static uint32_t _pack_drvconf(void);
static uint32_t _pack_drvctrl(void);
static uint32_t _pack_chopconf(void);
static uint32_t _pack_smarten(void);
static uint32_t _pack_sgcsconf(void);
static void _tmc262_init(void);
static void _tmc262_xmit(uint32_t word);

/*** Begin Code ***/

void device_init(void)
{
	DDRB = PORTB_DIR;		// initialize all ports for proper IO
	DDRC = PORTC_DIR;
	DDRD = PORTD_DIR;

	// initialize tmc262 parameter array
	memset(&tmc262_array,0,sizeof(tmc262_array));
	tmc262_tst = INIT262_TST;
	tmc262_slph = INIT262_SLPH;
	tmc262_slpl = INIT262_SLPL;
	tmc262_diss2g = INIT262_DISS2G;
	tmc262_ts2g = INIT262_TS2G;
	tmc262_sdoff = INIT262_SDOFF;
	tmc262_vsense = INIT262_VSENSE;
	tmc262_rdsel = INIT262_RDSEL;
	tmc262_intpol = INIT262_INTPOL;
	tmc262_dedge = INIT262_DEDGE;
	tmc262_mres = INIT262_MRES;
	tmc262_tbl = INIT262_TBL;
	tmc262_chm = INIT262_CHM;
	tmc262_rndtf = INIT262_RNDTF;
	tmc262_hdec = INIT262_HDEC;
	tmc262_hend = INIT262_HEND;
	tmc262_hstrt = INIT262_HSTRT;
	tmc262_toff = INIT262_TOFF;
	tmc262_seimin = INIT262_SEIMIN;
	tmc262_sedn = INIT262_SEDN;
	tmc262_semax = INIT262_SEMAX;
	tmc262_seup = INIT262_SEUP;
	tmc262_semin = INIT262_SEMIN;
	tmc262_sfilt = INIT262_SFILT;
	tmc262_sgt = INIT262_SGT;
	tmc262_cs = INIT262_CS;

	// initialize the chip
	_tmc262_init();			// initialize TMC262 chip
	device_led_on();		// put on the red light (Roxanne)
}

void device_reset(void)
{
	return;
}

uint8_t device_read_byte(uint8_t addr, uint8_t *data)
{
	addr -= OCB_COMMON_MAX;
	if (addr >= TMC262_ADDRESS_MAX) return (OCB_SC_INVALID_ADDRESS);
	*data = tmc262_array[addr];
	return (OCB_SC_OK);
}

uint8_t device_write_byte(uint8_t addr, uint8_t data)
{
	addr -= OCB_COMMON_MAX;
	if (addr >= TMC262_ADDRESS_MAX) return (OCB_SC_INVALID_ADDRESS);
	// there are no checks in here for read-only. Assumes all locations are writable.
	tmc262_array[addr] = data;
	return (OCB_SC_OK);
}

void device_led_on(void) 
{
	LED_PORT &= ~(LED_PIN);
}

void device_led_off(void) 
{
	LED_PORT |= LED_PIN;
}

/*** Local functions (statics) ***/

static void _tmc262_init(void) 
{
	SPI2_PORT |= SPI2_CLK; 		// set clock idle (hi)
	SPI2_PORT |= SPI2_SS;		// de-assert chip select (hi)

	_tmc262_xmit(_pack_drvconf());
	_tmc262_xmit(_pack_drvctrl());
	_tmc262_xmit(_pack_chopconf());
	_tmc262_xmit(_pack_smarten());
	_tmc262_xmit(_pack_sgcsconf());
}

static void _tmc262_xmit(uint32_t word)
{
	// setup clock and assert chip select
	SPI2_PORT |= SPI2_CLK; 		// set clock idle (hi)
	SPI2_PORT &= ~SPI2_SS;		// assert chip select (active lo)

	for (uint8_t i=0; i<20; i++) {		// TMC262 uses 20 bit words
		SPI2_PORT &= ~SPI2_CLK;			// set clock active (lo)
		if ((word && 0x0800) == true) {	// clock out a 1 if bit 19 is hi
			SPI2_PORT |= SPI2_MOSI;
		} else {
			SPI2_PORT &= ~SPI2_MOSI;
		}
		SPI2_PORT |= SPI2_CLK;	// take data (rising edge of clock)
		word <<= 1;				// shift left one bit
	}
	SPI2_PORT |= SPI2_SS;		// de-assert chip select (hi)
}								// leaves with clock idle (hi)

/*
 * TMC262 register builders - pack 20 bit words from controller parameters
 * 
 * _pack_drvconf()
 * _pack_drvctrl()
 * _pack_chopconf()
 * _pack_smarten()
 * _pack_sgcsconf()
 *
 *	Notice that the sum of the shifts in each function must = 17
 */

static uint32_t _pack_drvconf(void)
{
	uint32_t word = DRVCONF_ADDR;
	word <<= 1;	word |= tmc262_tst;
	word <<= 2;	word |= tmc262_slph;
	word <<= 2;	word |= tmc262_slpl;
	word <<= 2;	word |= tmc262_diss2g; // inserts a zero bit
	word <<= 2;	word |= tmc262_ts2g;
	word <<= 1;	word |= tmc262_sdoff;
	word <<= 1;	word |= tmc262_vsense;
	word <<= 2;	word |= tmc262_rdsel;
	word <<= 4;					// lower 4 bits are zero
	return (word);
}

static uint32_t _pack_drvctrl(void)
{
	uint32_t word = DRVCTRL_ADDR;
	word <<= 8;	word |= tmc262_intpol;
	word <<= 1;	word |= tmc262_dedge;
	word <<= 8;	word |= tmc262_mres;
	return (word);
}

static uint32_t _pack_chopconf(void)
{
	uint32_t word = CHOPCONF_ADDR;
	word <<= 2;	word |= tmc262_tbl;
	word <<= 1;	word |= tmc262_chm;
	word <<= 1;	word |= tmc262_rndtf;
	word <<= 2;	word |= tmc262_hdec;
	word <<= 4;	word |= tmc262_hend;
	word <<= 3;	word |= tmc262_hstrt;
	word <<= 4;	word |= tmc262_toff;
	return (word);
}

static uint32_t _pack_smarten(void)
{
	uint32_t word = SMARTEN_ADDR;
	word <<= 2;	word |= tmc262_seimin;
	word <<= 2;	word |= tmc262_sedn;
	word <<= 5;	word |= tmc262_semax;
	word <<= 3;	word |= tmc262_seup;
	word <<= 5;	word |= tmc262_semin;
	return (word);
}

static uint32_t _pack_sgcsconf(void)
{
	uint32_t word = SGCSCONF_ADDR;
	word <<= 1;	word |= tmc262_sfilt;
	word <<= 8;	word |= tmc262_sgt;
	word <<= 8;	word |= tmc262_cs;
	return (word);
}

/**** Device Unit Tests ****/

void device_unit_tests()
{
// Test the packing functions
	// Set all TMC values to all 1's using the test patterns in tmc262.h
	// The returned values should be:
	dev.drvconf = _pack_drvconf();	// 0x000FF7F0
	dev.drvctrl = _pack_drvctrl();	// 0x0000030F
	dev.chopconf = _pack_chopconf();// 0x0009FFFF
	dev.smarten = _pack_smarten();	// 0x000AEF6F
	dev.sgcsconf = _pack_sgcsconf();// 0x000D7F1F
}
