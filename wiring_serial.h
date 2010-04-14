/*
  Based on wiring.h - Partial implementation of the Wiring API for the ATmega8.
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

  $Id: wiring.h 387 2008-03-08 21:30:00Z mellis $
*/
/* 
  TinyG Notes
  Modified Grbl to support Xmega family processors
  Modifications Copyright (c) 2010 Alden S. Hart, Jr.
*/

#ifndef wiring_h
#define wiring_h

/* The performance mods to wiring require the buffer size to be a binary multiple */
#define RX_BUFFER_SIZE 128			// down from 200 in original code
#define printByte serialWrite		// macro replaces call to printByte

void beginSerial(long);
void serialWrite(unsigned char);
uint8_t serialAvailable(void);
char serialRead(void);
void serialFlush(void);

void printByte(unsigned char c);
void printString(const char *s);
void printPgmString(const char *s);
void printInteger(long n);
void printHex(unsigned long n);
void printIntegerInBase(unsigned long n, unsigned long base);
void printFloat(double n);

//void printMode(int);				// unsupported in TinyG
//void printNewline(void);			// unsupported in TinyG
//void printOctal(unsigned long n);	// unsupported in TinyG
//void printBinary(unsigned long n);// unsupported in TinyG

#endif
