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

#include "wiring_private.h"
#include <math.h>
#include <avr/pgmspace.h>

// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which rx_buffer_head is the index of the
// location to which to write the next incoming character and rx_buffer_tail
// is the index of the location from which to read.
#define RX_BUFFER_SIZE 200

unsigned char rx_buffer[RX_BUFFER_SIZE];

int rx_buffer_head = 0;
int rx_buffer_tail = 0;


/* beginSerial (xmega)
   
   Hijacked this routine to configure PORTC, USARTD1 (PORTC:7=Tx, PORTF:6=Rx)
   "baud" value is ignored and set internally to the routine
 */

void beginSerial(long baud) {
	PORTC.DIRSET = (1<<7); 					// set PORTC:7 transmit pin as output
	PORTC.DIRCLR = (1<<6); 					// clr PORTC:6 receive pin as input

//	PORTC.DIRSET = enableDE_bm;				// set PORTC:5 for DE line as output
//	PORTC.OUTCLR = enableDE_bm;        		// clr PORTC:5 (disabled)
//	PORTC.DIRSET = enableRE_bm;				// set PORTC:4 for ~RE line as output
//	PORTC.OUTCLR = enableRE_bm;				// clr PORTC:4 (enabled) 

//	USARTC1.BAUDCTRLA = 207;				// 9600b  (BSCALE=207,BSEL=0)
//	USARTC1.BAUDCTRLA = 103; 				// 19200b  (BSCALE=103,BSEL=0)
	USARTC1.BAUDCTRLA = 34;  					// 57600b  (BSCALE=34,BSEL=0)
//	USARTC1.BAUDCTRLA = 33; USARTC1.BAUDCTRLB = (-1<<4); 	// 115.2kb (BSCALE=33,BSEL=-1)
//	USARTC1.BAUDCTRLA = 31; USARTC1.BAUDCTRLB = (-2<<4);	// 230.4kb (BSCALE=31,BSEL=-2)
//	USARTC1.BAUDCTRLA = 27; USARTC1.BAUDCTRLB = (-3<<4); 	// 460.8kb (BSCALE=27,BSEL=-3)
//	USARTC1.BAUDCTRLA = 19; USARTC1.BAUDCTRLB = (-4<<4);	// 921.6kb (BSCALE=19,BSEL=-4)
//	USARTC1.BAUDCTRLA = 1; USARTC1.BAUDCTRLB = (1<<4); 		// 500kb (BSCALE=19,BSEL=-4)
//	USARTC1.BAUDCTRLA = 1;   								// 1Mb (BSCALE=1,BSEL=0)

	USARTC1.CTRLB = USART_TXEN_bm | USART_RXEN_bm; // enable tx and rx on USART
}


void serialWrite(uint8_t data) {
	while(!(USARTC1.STATUS & USART_DREIF_bm)); 	// spin until TX data register is available
	USARTC1.DATA = data;							// write data register
}

int serialAvailable()
{
	return (RX_BUFFER_SIZE + rx_buffer_head - rx_buffer_tail) % RX_BUFFER_SIZE;
}

int serialRead()
{
	// if the head isn't ahead of the tail, we don't have any characters
	if (rx_buffer_head == rx_buffer_tail) {
		return -1;
	} else {
		unsigned char c = rx_buffer[rx_buffer_tail];
		rx_buffer_tail = (rx_buffer_tail + 1) % RX_BUFFER_SIZE;
		return c;
	}
}

void serialFlush()
{
	// don't reverse this or there may be problems if the RX interrupt
	// occurs after reading the value of rx_buffer_head but before writing
	// the value to rx_buffer_tail; the previous value of rx_buffer_head
	// may be written to rx_buffer_tail, making it appear as if the buffer
	// were full, not empty.
	rx_buffer_head = rx_buffer_tail;
}

#ifdef USART_RX_vect
SIGNAL(USART_RX_vect)
#else
SIGNAL(SIG_USART_RECV)
#endif
{
	unsigned char c = USARTC1.DATA;		// *********

	int i = (rx_buffer_head + 1) % RX_BUFFER_SIZE;

	// if we should be storing the received character into the location
	// just before the tail (meaning that the head would advance to the
	// current location of the tail), we're about to overflow the buffer
	// and so we don't write the character or advance the head.
	if (i != rx_buffer_tail) {
		rx_buffer[rx_buffer_head] = c;
		rx_buffer_head = i;
	}
}

// void printMode(int mode)
// {
//  // do nothing, we only support serial printing, not lcd.
// }

void printByte(unsigned char c)
{
	serialWrite(c);
}

// void printNewline()
// {
//  printByte('\n');
// }
// 
void printString(const char *s)
{
	while (*s)
		printByte(*s++);
}

// Print a string stored in PGM-memory
void printPgmString(const char *s)
{
  char c;
	while ((c = pgm_read_byte_near(s++)))
		printByte(c);
}

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

void printInteger(long n)
{
	if (n < 0) {
		printByte('-');
		n = -n;
	}

	printIntegerInBase(n, 10);
}

void printFloat(double n)
{
  double integer_part, fractional_part;
  fractional_part = modf(n, &integer_part);
  printInteger(integer_part);
  printByte('.');
  printInteger(round(fractional_part*1000));
}

// void printHex(unsigned long n)
// {
//  printIntegerInBase(n, 16);
// }
// 
// void printOctal(unsigned long n)
// {
//  printIntegerInBase(n, 8);
// }
// 
// void printBinary(unsigned long n)
// {
//  printIntegerInBase(n, 2);
// }

/* Including print() adds approximately 1500 bytes to the binary size,
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
