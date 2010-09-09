/*
  xmega_init.h - general init and support functions for xmega family
  Copyright (c) 2010 Alden S Hart
*/

#ifndef xmega_support_h
#define xmega_support_h

/* set CPU clock for delays */		// should precede <avr/delay.h>
#ifndef F_CPU
#define F_CPU 32000000UL
#endif

/* function prototypes */
void xmega_init(void);
void config32MHzClock(void);

#endif
