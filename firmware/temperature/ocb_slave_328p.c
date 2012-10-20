/*
 * ocb_slave_328p.c - Open Controller Bus slave driver for Atmega328P 
 * Part of Open Controller Bus project
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
/* The SPI peripheral is used for the SPI slave using its MOSI, MISO, SCK, SS pins.
 * The slave device takes OCB instructions from the motherboard.
 * A bit-banger master SPI is provided to talk to a downstream device
 */
//#include <ctype.h>
//#include <stdlib.h>
//#include <math.h>
//#include <avr/pgmspace.h>

#include <stdio.h>
#include <string.h>				// for memset
#include <avr/interrupt.h>

#include "ocb.h"
#include "ocb_slave_328p.h"
#include "tmc262.h"				// device file

static struct OCBSlaveSingleton {
	uint8_t	phase;				// byte phasing for RX
	uint8_t addr;				// address received from master
	uint8_t data;				// data received from master or to send to master
} ocb_slave;

static uint8_t _ocb_slave_write_byte(const uint8_t addr, const uint8_t data);

/*
 * ocb_slave_init() - setup atmega SPI peripheral to be the OCB slave 
 */
void ocb_slave_init(void)
{
	PRR |= 0x07;
	DDRB &= ~(1<<DDB4);			// Set MISO output, all others unaffected
	SPCR = (1<<SPIE | 1<<SPE);	// Enable SPI and its interrupt, set MSB first, slave mode
	SPCR = (1<<CPOL | 1<<CPHA);	// Uncomment for mode 3 operation, comment for mode 0
	memset(&ocb_array, 0, sizeof(ocb_array));

	ocb_wait_time = DEVICE_WAIT_TIME;	// setup read-only values
	ocb_device_type = DEVICE_TYPE;
	ocb_device_id_hi = DEVICE_ID_HI;
	ocb_device_id_lo = DEVICE_ID_LO;
	ocb_device_rev_major = DEVICE_REV_MAJOR;
	ocb_device_rev_minor = DEVICE_REV_MINOR;
	ocb_device_uuid_1 = DEVICE_UUID_1;
	ocb_device_uuid_2 = DEVICE_UUID_2;
	ocb_device_uuid_3 = DEVICE_UUID_3;

	ocb_status = OCB_SC_OK;
	SPDR = ocb_status;
	device_init();				// initialize the device last
}

/* 
 * SPI Slave RX Interrupt() - interrupts on byte received
 *
 * Uses a 2 phase state machine to toggle back and forth between ADDR and DATA bytes
 */
ISR(SPI_STC_vect)
{
	// receive address byte
	if (ocb_slave.phase == OCB_ADDR) {
		ocb_slave.phase = OCB_DATA;	// advance phase
		ocb_slave.addr = SPDR;		// read and save the address byte
		if (ocb_command == OCB_WRITE) { // write is simple...
			SPDR = OCB_OK_BYTE;			// already saved addr, now return an OK
		} else {
			if (ocb_slave.addr < OCB_COMMON_MAX) {	// handle OCB address space
				SPDR = ocb_array[ocb_slave.addr];
			} else {								// handle device address space
				if ((ocb_status = device_read_byte(ocb_slave.addr, &ocb_slave.data)) == OCB_SC_OK) {
					SPDR = ocb_slave.data;
				} else {
					SPDR = OCB_ERR_BYTE;
				}
			}
		}

	// receive data byte
	} else {
		ocb_slave.phase = OCB_ADDR;	// advance phase
		ocb_slave.data = SPDR;		// read and save the data byte
		if (ocb_command == OCB_WRITE) {
			if (ocb_slave.addr < OCB_COMMON_MAX) {
				ocb_status = _ocb_slave_write_byte(ocb_slave.addr, ocb_slave.data);
			} else {
				ocb_status = device_write_byte(ocb_slave.addr, ocb_slave.data);
			}
		}
	}
}

/* 
 * _ocb_slave_write_byte() - helper to write byte to an OCB non-device address
 */
static uint8_t _ocb_slave_write_byte(const uint8_t addr, const uint8_t data)
{
	if (addr == OCB_COMMAND) {
		ocb_command = data; 

	} else if (addr == OCB_ADDR_PAGE) { 
		ocb_addr_page = data; 

	} else if (addr == OCB_RESET) {
		ocb_slave_init();
		device_reset();

	} else {
		return (OCB_SC_READ_ONLY_ADDRESS);
	}
	return (OCB_SC_OK);
}

