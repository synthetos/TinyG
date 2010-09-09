/*
  xio_wiring_compat.h - compatibility with some routines in Arduino's wiring_serial.c
*/

#ifndef xio_wiring_compat_h
#define xio_wiring_compat_h

#define serialRead() getchar()
#define printByte(c) putchar(c)
#define printString(b) printf(b)
#define printPgmString(b) printf_P(b)
void printIntegerInBase(unsigned long n, unsigned long base);
void printInteger(long n);
void printFloat(double n);
void printHex(unsigned long n);

#endif
