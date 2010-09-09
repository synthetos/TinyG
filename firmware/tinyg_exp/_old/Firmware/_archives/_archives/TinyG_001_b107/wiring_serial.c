/*
  wiring_serial.c - serial functions.
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2005-2006 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  $Id: wiring.c 248 2007-02-03 15:36:30Z mellis $
*/

/* 
  TinyG Notes
  Modified Grbl to support Xmega family processors
  Modifications Copyright (c) 2010 Alden S. Hart, Jr.

  Changes:
  - All the device register names are different from the ATmega series
  - ISRs are called differently
  - wiring_serial routines not (sup)ported are:
	- printMode()
	- printNewline()
	- printOctal()
	- printBinary()
	- print()

  - implemented more efficient constructs as per:
	http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1235799875 

  ToDo:
  - put buffers in structs and generalize routines to support multiple serial channels
  - Make beginSerial accept a channel argument
  - make beginSerial() actually use the baud rate in the input arg
*/

#include <math.h>
#include <avr/pgmspace.h>
#include "xmega_support.h"
#include "wiring_private.h"
#include "wiring_serial.h"
#include "config.h"

/*  Define constants and variables for buffering incoming serial data.  
	We're using a circular buffer, in which rx_buffer_head is the index of the
	location to which to write the next incoming character and rx_buffer_tail
	is the index of the location from which to read.
*/

#define RX_BUFFER_MASK (RX_BUFFER_SIZE - 1) // new constant introduced
unsigned char rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_buffer_head = 0;					// changed from type int
uint8_t rx_buffer_tail = 0;					// changed from type int

/* beginSerial() - xmega serial routine
   
   Hijacked this routine to configure PORTC, USARTD1 (PORTC:7=Tx, PORTF:6=Rx)
   "baud" value is ignored and set internally to the routine
 */

void beginSerial(long baud) 
{
	PORTC.DIRCLR = (1<<2); 					// clr PORTC:6 receive pin as input
	PORTC.DIRSET = (1<<3); 					// set PORTC:7 transmit pin as output
	PORTC.OUTSET = (1<<3);					// set TX pin HI as initial state

	USARTC0.BAUDCTRLA = USB_BSEL;
	USARTC0.BAUDCTRLB = USB_BSCALE;
	USARTC0.CTRLB = USART_TXEN_bm | USART_RXEN_bm; // enable tx and rx on USART
	USARTC0.CTRLA = USART_RXCINTLVL_MED_gc;		   // receive interrupt medium level

	_delay_us(100);							// give it a chance to settle before use
}

void beginSerialC1(long baud) 
{
	PORTC.DIRCLR = (1<<6); 					// clr PORTC:6 receive pin as input
	PORTC.DIRSET = (1<<7); 					// set PORTC:7 transmit pin as output
	PORTC.OUTSET = (1<<7);					// set TX pin HI as initial state

//	PORTC.DIRSET = enableDE_bm;				// set PORTC:5 for DE line as output
//	PORTC.OUTCLR = enableDE_bm;        		// clr PORTC:5 (disabled)
//	PORTC.DIRSET = enableRE_bm;				// set PORTC:4 for ~RE line as output
//	PORTC.OUTCLR = enableRE_bm;				// clr PORTC:4 (enabled) 

	USARTC1.BAUDCTRLA = USB_BSEL;
	USARTC1.BAUDCTRLB = USB_BSCALE;
	USARTC1.CTRLB = USART_TXEN_bm | USART_RXEN_bm; // enable tx and rx on USART
	USARTC1.CTRLA = USART_RXCINTLVL_MED_gc;		   // receive interrupt medium level

	_delay_us(100);							// give it a chance to settle before use
}

/* ISR for USB serial RX - receive serial byte from USB port*/

ISR(USARTC0_RXC_vect)
{
	unsigned char c = USARTC0.DATA;
	uint8_t i = (rx_buffer_head + 1) & RX_BUFFER_MASK;

	/*  If we should be storing the received character into the location
		just before the tail (meaning that the head would advance to the
		current location of the tail), we're about to overflow the buffer
		and so we don't write the character or advance the head. */

	if (i != rx_buffer_tail) {
		rx_buffer[rx_buffer_head] = c;
		rx_buffer_head = i;
	}
}

ISR(USARTC1_RXC_vect)
{
	unsigned char c = USARTC1.DATA;
	uint8_t i = (rx_buffer_head + 1) & RX_BUFFER_MASK;

	/*  If we should be storing the received character into the location
		just before the tail (meaning that the head would advance to the
		current location of the tail), we're about to overflow the buffer
		and so we don't write the character or advance the head. */

	if (i != rx_buffer_tail) {
		rx_buffer[rx_buffer_head] = c;
		rx_buffer_head = i;
	}
}

/* writeSerial() - write character to serial port */

void serialWrite(unsigned char c) 
{
//+++++	while(!(USARTC0.STATUS & USART_DREIF_bm)); 	// spin until TX data register is available
	USARTC0.DATA = c;							// write data register
}

/*
void serialWrite(unsigned char c) 
{
	while(!(USARTC1.STATUS & USART_DREIF_bm)); 	// spin until TX data register is available
	USARTC1.DATA = c;							// write data register
}
*/

/* serialAvailable() - optimized for 8 bit operation */

uint8_t serialAvailable() 
{
	return (RX_BUFFER_SIZE + rx_buffer_head - rx_buffer_tail) & RX_BUFFER_MASK;
}

/* serialRead() - optimized for 8 bit architecture 
	
	This routine is modified from the original wiring_serial code in 2 ways:

	- The original routine returns -1 if there is no character to read. 
	  Returning a -1 as a character type seems to cause problems
	  This value has beeen changed to ascii EOT (0x04) which is the closest thing
	  I could not find a "no data here, move along folks" character.

	- The buffer wraparound math as been replaced with masking instead of modulus.
	  This requires that the buffer size be a binary multiple. It's currently 128.
*/

char serialRead() 
{
	/* if the head isn't ahead of the tail, we don't have any characters */
	if (rx_buffer_head == rx_buffer_tail) {
		return 0x04;											// ASCII EOT
	} else {
		char c = rx_buffer[rx_buffer_tail];
//		rx_buffer_tail = (rx_buffer_tail + 1) % RX_BUFFER_SIZE;	// original code
		rx_buffer_tail = (rx_buffer_tail + 1) & RX_BUFFER_MASK;	// optimized code
		return c;
	}
}

void serialFlush()
{
	/*  don't reverse this or there may be problems if the RX interrupt
		occurs after reading the value of rx_buffer_head but before writing
		the value to rx_buffer_tail; the previous value of rx_buffer_head
		may be written to rx_buffer_tail, making it appear as if the buffer
		were full, not empty. */
	rx_buffer_head = rx_buffer_tail;
}

/* printByte has been removed in favor of a macro pointing to serialWrite
void printByte(unsigned char c)
{
	serialWrite(c);
}
*/

void printString(const char *s)
{
	while (*s) {
		printByte(*s++);
	}
}

/*  printPgmString() - print a string stored in PGM-memory */
void printPgmString(const char *s)
{
	unsigned char c;
	while ((c = pgm_read_byte_near(s++))) {
		printByte(c);
	}
}

/* printIntegerInBase() */

void printIntegerInBase(unsigned long n, unsigned long base)
{ 
	unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars. 
	unsigned long i = 0;

	if (n == 0) {
		printByte('0');
		return;
	} 

	while (n > 0) {
		buf[i++] = n % base;
		n /= base;
	}

	for (; i > 0; i--)
		printByte(buf[i - 1] < 10 ?
			'0' + buf[i - 1] :
			'A' + buf[i - 1] - 10);
}

/* printInteger() */

void printInteger(long n)
{
	if (n < 0) {
		printByte('-');
		n = -n;
	}
	printIntegerInBase(n, 10);
}

/* printFloat() */

void printFloat(double n)
{
	double integer_part, fractional_part;
	fractional_part = modf(n, &integer_part);
	printInteger(integer_part);
	printByte('.');
	printInteger(round(fractional_part*1000));
}

/* printHex() */

void printHex(unsigned long n)
{
	printIntegerInBase(n, 16);
}

/* Unsupported wiring_serial routines 
void printMode(int mode)
{
	// do nothing, we only support serial printing, not lcd.
}

void printNewline()
{
	printByte('\n');
}
 
void printOctal(unsigned long n)
{
	printIntegerInBase(n, 8);
}

void printBinary(unsigned long n)
{
	printIntegerInBase(n, 2);
}

 * Including print() adds approximately 1500 bytes to the binary size,
 * so we replace it with the smaller and less-confusing printString(),
 * printInteger(), etc.
void print(const char *format, ...)
{
	char buf[256];
	va_list ap;
	
	va_start(ap, format);
	vsnprintf(buf, 256, format, ap);
	va_end(ap);
	
	printString(buf);
}
*/
