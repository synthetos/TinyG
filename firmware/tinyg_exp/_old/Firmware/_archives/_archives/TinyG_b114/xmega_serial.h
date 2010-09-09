/*
  xmega_serial.h - "file" and serial functions for xmega family
  Modeled after unix fileio 

  Copyright (c) 2010 Alden S. Hart, Jr.


*/

#ifndef xmega_serial_h
#define xmega_serial_h


/* Devices recognized by IO system functions */

// Native devices
#define DEV_NULL	0				// NULL device

#define	DEV_SRAM	1				// string in static RAM
#define DEV_EEPROM	2				// string in EEPROM
#define DEV_PROGMEM 3				// string in program memory (FLASH)

#define DEV_PORTA	4				// Define ports as IO devices
#define DEV_PORTB	5
#define DEV_PORTC	6
#define DEV_PORTD	7
#define DEV_PORTE	8
#define DEV_PORTF	9
#define DEV_PORTG	10				// not implemented on xmega A3s
#define DEV_PORTH	11				// not implemented on xmega A3s
#define DEV_PORTJ	12				// not implemented on xmega A3s
#define DEV_PORTK	13				// not implemented on xmega A3s
#define DEV_PORTL	14				// not implemented on xmega A3s
#define DEV_PORTM	15				// not implemented on xmega A3s
#define DEV_PORTN	16				// not implemented on xmega A3s
#define DEV_PORTP	17				// not implemented on xmega A3s
#define DEV_PORTQ	18				// not implemented on xmega A3s
#define DEV_PORTR	19				// not implemented on xmega A3s

#define DEV_PORTR	20				// special purpose port - programming bits only 

#define DEV_USARTC0	21				// USARTS C0 - F1
#define DEV_USARTC1	22
#define DEV_USARTD0	23
#define DEV_USARTD1	24
#define DEV_USARTE0	25
#define DEV_USARTE1	26
#define DEV_USARTF0	27
#define DEV_USARTF1	28

#define DEV_SPIC	29				// SPI interfaces C - F
#define DEV_SPID	30
#define DEV_SPIE	31
#define DEV_SPIF	32

#define DEV_TWIC	33				// Two Wire interfaces C and E
#define DEV_TWIE	34

#define DEV_IRCOM	35				// IR communications module

// Synthetic devices
#define DEV_CONSOLE		36			// mapped to USB, here for convenience
#define DEV_USB			37			// USB comm and controls packaged
#define DEV_RS485		38			// RS485 comm and controls packaged
#define DEV_ENCODERS	39			// Encoder comm and controls packaged

/* global variables */

int	errno=0;						// global error number





/* The performance mods to wiring require the buffer size to be a binary multiple */
#define RX_BUFFER_SIZE 128			// down from 200 in original code
#define printByte serialWrite		// macro replaces call to printByte

void io_init(void);
int io_open();
int io_close();
int io_read(unit8_t fd, char *buf, int i);	// read a single serial character
int io_write(uint8_t fd, char *buf, int i);
int io_flush(void);
void io_ioctl(uint8_t *fd);

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
