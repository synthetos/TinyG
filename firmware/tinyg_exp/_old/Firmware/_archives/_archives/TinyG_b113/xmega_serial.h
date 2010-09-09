/*
  xmega_serial.h - serial functions for xmega family
  Modeled after *nix serial io
  .
  Copyright (c) 2010 Alden S. Hart, Jr.

#ifndef _SERIALIO_H_

extern int serial_open (char *, int *,  int, speed_t);
extern int serial_close (int);
extern int serial_readchar (int, uint8_t *);
extern int serial_write (int, void *, size_t);
extern int serial_read (int, void *, size_t);

#endif  // _SERIALIO_H_

*/

#ifndef xmega_serial_h
#define xmega_serial_h





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
