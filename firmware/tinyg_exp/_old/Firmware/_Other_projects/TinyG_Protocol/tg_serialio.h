/*
	tg_serialio.h
	TinyG Generic Serial Handler (both RS-485 and USB)
	Written by: Alden Hart
	Revision: 03/20/10

	References: Boston Android code used in this module
*/

#define enableDE_bm (1<<5);		// DE line - active HI
#define enableRE_bm (1<<4);		// RE line - active LO
#define charTime (100);			// character time in microseconds

void initSerialio(void);
void config32MHzClock(void);
void configUsartC1(void);
void usartWriteChar(unsigned char data);
void usartWriteString(const char *txstring);
unsigned char usartReadChar(void);

