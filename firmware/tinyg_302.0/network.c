/*
 * network.c - tinyg networking protocol
 * Part of TinyG project
 *
 * Copyright (c) 2011 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify
 * it under the terms of the Creative Commons CC-BY-NC license 
 * (Creative Commons Attribution Non-Commercial Share-Alike license)
 * as published by Creative Commons. You should have received a copy 
 * of the Creative Commons CC-BY-NC license along with TinyG.
 * If not see http://creativecommons.org/licenses/
 *
 * TinyG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 */

#include <stdio.h>
//#include <util/delay.h>
//#include <ctype.h>
//#include <avr/pgmspace.h>

#include "xio.h"
#include "encoder.h"
#include "controller.h"
//#include "tinyg.h"

/*
 * Local Scope Functions and Data
 */

/*
 * net_init()
 */

void net_init() 
{
	
}

/* 
 * tg_repeater() - top-level controller.
 */

void tg_repeater()
{
/*	while (TRUE) {
		xio_putc_rs485('A', fdev_rs485);
		en_toggle(1);
		_delay_ms(50);
	}
*/	
	char c;

	en_write(4);
	while (TRUE) {
		c = xio_getc(XIO_DEV_USB);		// blocking read
		xio_putc(XIO_DEV_RS485, c);		// transfer to RS485 port
		en_toggle(1);
	}
}

/* 
 * tg_receiver()
 */

void tg_receiver()
{
	tg_controller();	// this node executes gcode blocks received via RS485

/*	char c;

	en_write(4);
	while (TRUE) {
		c = xio_getc(XIO_DEV_RS485);	// blocking read
		xio_putc(XIO_DEV_USB, c);		// transfer to USB port
		if (c != CR) {
			en_toggle(1);
		}
	}
*/
}
