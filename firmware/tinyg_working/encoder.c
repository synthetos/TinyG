/*
 * encoder.c - encoder interfaces
 * Part of TinyG project
 * Copyright (c) 2010 Alden S. Hart, Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, 
 * either version 3 of the License, or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with TinyG  
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <avr/pgmspace.h>

#include "tinyg.h"
#include "config.h"
#include "stepper.h"
#include "encoder.h"
#include "hardware.h"


void en_init(void) 
{
	return;
}

/*
 * en_write() - write lowest 4 bits of a byte to encoder outpur port
 *
 * This is a hack to hide the fact that we've scattered the encode output
 * bits all over the place becuase we have no more contiguous ports left. 
 */

void en_write(uint8_t b)
{
	if (b && 0x01) { 								// b0 is on A axis
		AXIS(A).port->OUTSET = ENCODER_OUT_BIT_bm;
	} else {
		AXIS(A).port->OUTCLR = ENCODER_OUT_BIT_bm;
	}

	if (b && 0x02) { 								// b1 is on Z axis
		AXIS(Z).port->OUTSET = ENCODER_OUT_BIT_bm;
	} else {
		AXIS(Z).port->OUTCLR = ENCODER_OUT_BIT_bm;
	}

	if (b && 0x04) { 								// b2 is on Y axis
		AXIS(Y).port->OUTSET = ENCODER_OUT_BIT_bm;
	} else {
		AXIS(Y).port->OUTCLR = ENCODER_OUT_BIT_bm;
	}

	if (b && 0x08) { 								// b3 is on X axis
		AXIS(X).port->OUTSET = ENCODER_OUT_BIT_bm;
	} else {
		AXIS(X).port->OUTCLR = ENCODER_OUT_BIT_bm;
	}
}
