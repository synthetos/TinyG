/*
  xmega_support.h - general support functions for xmega family

  Copyright (c) 2010 Alden S Hart

Notes
---	For F_CPU to be useful this file should precede the following files:
#include "xmega_support.h"		// this file
#include <avr/delay.h>

*/

#ifndef xmega_support_h
#define xmega_support_h

#define FALSE 0
#define TRUE 1

/* set CPU clock for delays */
#ifndef F_CPU
#define F_CPU 32000000UL
#endif

/* function prototypes */
void xmega_init(void);
void config32MHzClock(void);

#endif
