/*
  xmega_support.h - support for Grbl xmega port 
  Part of Grbl xmega port

  Copyright (c) 2010 Alden S Hart

Notes
---	For F_CPU to be useful this file should precede the following files:
#include "xmega_support.h"		// this file
#include <avr/delay.h>
#include "nuts_bolts.h"
#include "wiring_private.h"

*/

#ifndef xmega_support_h
#define xmega_support_h

/* set CPU clock for delays */
#ifndef F_CPU
#define F_CPU 32000000UL
#endif

/* function prototypes */
void xmega_init(void);
void config32MHzClock(void);

#endif  // xmega_support_h
