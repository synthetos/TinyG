/*
 * ki_slave_328p.c - Open Controller Bus slave driver for Atmega328P 
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

#include "kinen.h"
#include "kinen_slave_328p.h"
#include "tmc262.h"				// device file

static struct KinenSlaveSingleton {
	uint8_t	phase;				// byte phasing for RX
	uint8_t addr;				// address received from master
	uint8_t data;				// data received from master or to send to master
} ki_slave;

static uint8_t _ki_slave_write_byte(const uint8_t addr, const uint8_t data);

/*
 * kinen_slave_init() - setup atmega SPI peripheral to be the OCB slave 
 */
void ki_slave_init(void)
{
	PRR |= 0x07;
	DDRB &= ~(1<<DDB4);			// Set MISO output, all others unaffected
	SPCR = (1<<SPIE | 1<<SPE);	// Enable SPI and its interrupt, set MSB first, slave mode
	SPCR = (1<<CPOL | 1<<CPHA);	// Uncomment for mode 3 operation, comment for mode 0
	memset(&ki_array, 0, sizeof(ki_array));

	ki_wait_time = DEVICE_WAIT_TIME;	// setup read-only values
	ki_device_type = DEVICE_TYPE;
	ki_device_id_hi = DEVICE_ID_HI;
	ki_device_id_lo = DEVICE_ID_LO;
	ki_device_rev_major = DEVICE_REV_MAJOR;
	ki_device_rev_minor = DEVICE_REV_MINOR;
	ki_device_uuid_1 = DEVICE_UUID_1;
	ki_device_uuid_2 = DEVICE_UUID_2;
	ki_device_uuid_3 = DEVICE_UUID_3;

	ki_status = KINEN_SC_OK;
	SPDR = ki_status;
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
	if (ki_slave.phase == KINEN_ADDR) {
		ki_slave.phase = KINEN_DATA;	// advance phase
		ki_slave.addr = SPDR;		// read and save the address byte
		if (ki_command == KINEN_WRITE) { // write is simple...
			SPDR = KINEN_OK_BYTE;			// already saved addr, now return an OK
		} else {
			if (ki_slave.addr < KINEN_COMMON_MAX) {	// handle OCB address space
				SPDR = ki_array[ki_slave.addr];
			} else {								// handle device address space
				if ((ki_status = device_read_byte(ki_slave.addr, &ki_slave.data)) == KINEN_SC_OK) {
					SPDR = ki_slave.data;
				} else {
					SPDR = KINEN_ERR_BYTE;
				}
			}
		}

	// receive data byte
	} else {
		ki_slave.phase = KINEN_ADDR;	// advance phase
		ki_slave.data = SPDR;		// read and save the data byte
		if (ki_command == KINEN_WRITE) {
			if (ki_slave.addr < KINEN_COMMON_MAX) {
				ki_status = _ki_slave_write_byte(ki_slave.addr, ki_slave.data);
			} else {
				ki_status = device_write_byte(ki_slave.addr, ki_slave.data);
			}
		}
	}
}

/* 
 * _ki_slave_write_byte() - helper to write byte to an OCB non-device address
 */
static uint8_t _ki_slave_write_byte(const uint8_t addr, const uint8_t data)
{
	if (addr == KINEN_COMMAND) {
		ki_command = data; 

	} else if (addr == KINEN_ADDR_PAGE) { 
		ki_addr_page = data; 

	} else if (addr == KINEN_RESET) {
		ki_slave_init();
		device_reset();

	} else {
		return (KINEN_SC_READ_ONLY_ADDRESS);
	}
	return (KINEN_SC_OK);
}

