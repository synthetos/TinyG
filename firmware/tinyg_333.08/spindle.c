/*
 * canonical_spindle.c - canonical machine spindle driver
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2012 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for details. You should have received a copy of the GNU General Public 
 * License along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 */
/* This module is pretty much a direct lift from the grbl project.
 * It has not been exercised or tested.
 */

#include <avr/io.h>
#include "tinyg.h"
#include "gpio.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "spindle.h"
#include "planner.h"
#include "system.h"

/* 
 * sp_init()
 */

void cm_spindle_init()
{
	return;
}

/*
 * cm_spindle_control() -  queue the spindle command to the planner buffer
 */
uint8_t cm_spindle_control(uint8_t spindle_mode)
{
	if (spindle_mode == SPINDLE_CW) {
		mp_queue_mcode(MCODE_SPINDLE_CW);
	} else if (spindle_mode == SPINDLE_CCW) {
		mp_queue_mcode(MCODE_SPINDLE_CCW);
	} else {
		mp_queue_mcode(MCODE_SPINDLE_OFF);	// failsafe operation
	}
	return(TG_OK);
}

/*
 * cm_exec_spindle_control() - execute the spindle command (called from planner)
 */
void cm_exec_spindle_control(uint8_t spindle_mode)
{
	cm_set_spindle_mode(spindle_mode);
 	if (spindle_mode == SPINDLE_CW) {
		gpio_set_bit_on(SPINDLE_BIT);
		gpio_set_bit_off(SPINDLE_DIR);
	} else if (spindle_mode == SPINDLE_CCW) {
		gpio_set_bit_on(SPINDLE_BIT);
		gpio_set_bit_on(SPINDLE_DIR);
	} else {
		gpio_set_bit_off(SPINDLE_BIT);	// failsafe: any error causes stop
	}
}

/*
 * cm_set_spindle_speed() - queue the S parameter to the planner buffer
 */

uint8_t cm_set_spindle_speed(double speed)
{
//	if (speed > gm.max_spindle speed) {
//		return (TG_MAX_SPINDLE_SPEED_EXCEEDED);
//	}
	cm_set_spindle_speed_parameter(speed);
	return (TG_OK);
}

/*
 * cm_exec_spindle_speed() - execute the S command (called from the planner buffer)
 */
void cm_exec_spindle_speed(double speed)
{

}
