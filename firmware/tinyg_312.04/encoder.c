/*
 * encoder.c - encoder interfaces
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2011 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <avr/pgmspace.h>

#include "tinyg.h"
#include "config.h"
#include "stepper.h"
#include "encoder.h"
#include "system.h"

static uint8_t encoder_port_value;

void en_init(void) 
{
	return;
}

/*
 * en_bit_on() - turn bit on
 * en_bit_off() - turn bit on
 */

void en_bit_on(uint8_t b)
{
	if (b & 0x01) {
		DEVICE_PORT_MOTOR_4.OUTSET = ENCODER_OUT_BIT_bm;
	}
	if (b & 0x02) {
		DEVICE_PORT_MOTOR_3.OUTSET = ENCODER_OUT_BIT_bm;
	}
	if (b & 0x04) {
		DEVICE_PORT_MOTOR_2.OUTSET = ENCODER_OUT_BIT_bm;
	}
	if (b & 0x08) {
		DEVICE_PORT_MOTOR_1.OUTSET = ENCODER_OUT_BIT_bm;
	}
}

void en_bit_off(uint8_t b)
{
	if (b & 0x01) {
		DEVICE_PORT_MOTOR_4.OUTCLR = ENCODER_OUT_BIT_bm;
	}
	if (b & 0x02) {
		DEVICE_PORT_MOTOR_3.OUTCLR = ENCODER_OUT_BIT_bm;
	}
	if (b & 0x04) {
		DEVICE_PORT_MOTOR_2.OUTCLR = ENCODER_OUT_BIT_bm;
	}
	if (b & 0x08) {
		DEVICE_PORT_MOTOR_1.OUTCLR = ENCODER_OUT_BIT_bm;
	}
}

/*
 * en_write() - write lowest 4 bits of a byte to encoder output port
 *
 * This is a hack to hide the fact that we've scattered the encode output
 * bits all over the place becuase we have no more contiguous ports left. 
 */

void en_write(uint8_t b)
{
	encoder_port_value = b;

	if (b & 0x01) { // b0 is on MOTOR_4 (A axis)
		DEVICE_PORT_MOTOR_4.OUTSET = ENCODER_OUT_BIT_bm;
	} else {
		DEVICE_PORT_MOTOR_4.OUTCLR = ENCODER_OUT_BIT_bm;
	}

	if (b & 0x02) { // b1 is on MOTOR_3 (Z axis)
		DEVICE_PORT_MOTOR_3.OUTSET = ENCODER_OUT_BIT_bm;
	} else {
		DEVICE_PORT_MOTOR_3.OUTCLR = ENCODER_OUT_BIT_bm;
	}

	if (b & 0x04) { // b2 is on MOTOR_2 (Y axis)
		DEVICE_PORT_MOTOR_2.OUTSET = ENCODER_OUT_BIT_bm;
	} else {
		DEVICE_PORT_MOTOR_2.OUTCLR = ENCODER_OUT_BIT_bm;
	}

	if (b & 0x08) { // b3 is on MOTOR_1 (X axis)
		DEVICE_PORT_MOTOR_1.OUTSET = ENCODER_OUT_BIT_bm;
	} else {
		DEVICE_PORT_MOTOR_1.OUTCLR = ENCODER_OUT_BIT_bm;
	}
}

/*
 * en_toggle() - toggle lowest 4 bits of a byte to encoder output port
 *
 *	Note: doesn't take transitions form bit_on / bit_off into account
 */

void en_toggle(uint8_t b)
{
	encoder_port_value ^= b;	// xor the stored encoder value with b
	en_write(encoder_port_value);
}

