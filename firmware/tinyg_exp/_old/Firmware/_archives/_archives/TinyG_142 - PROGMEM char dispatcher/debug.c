#include <stdio.h>
#include "stepper.h"
#include "config.h"
#include "tinyg.h"

#include "xio.h"
#include "xio_wiring_compat.h"

#include "debug.h"

/* NOTE: seme calling examples are provided below */
//	printPgmString(PSTR());
//	st_print_axis(&ax.x, "X");
//	st_print_axis(&ax.y, "Y");
//	st_print_axis(&ax.z, "Z");

#ifdef __DEBUG

void st_print_axis(struct Axis *A, char *label)	// called as st_print_axis(&ax.x, "X");
{
	printString(label);					
	printPgmString(PSTR(" axis:"));
	printPgmString(PSTR(" Count="));	
	printInteger(A->counter);
	printPgmString(PSTR(" Port="));		
	printInteger(A->port->IN);	// reading IN reports state
	printPgmString(PSTR("\r\n"));
}

void st_print_four_ints(long x, long y, long z, long u) 
{
	printPgmString(PSTR("Line: X="));	
	printInteger(x);
	printPgmString(PSTR(" Y="));		
	printInteger(y);
	printPgmString(PSTR(" Z="));		
	printInteger(z);
	printPgmString(PSTR(" uS="));		
	printInteger(u);
	printPgmString(PSTR("\r\n"));
}

void st_print_line(struct Line line) \
{
 	printPgmString(PSTR("\r\n"));
	printPgmString(PSTR("Line X="));	printInteger(line.steps_x);
	printPgmString(PSTR(", Y="));		printInteger(line.steps_y);
	printPgmString(PSTR(", Z="));		printInteger(line.steps_z);
	printPgmString(PSTR(" uS="));		printInteger(line.microseconds);
}

void st_print_exec_line(struct Line line, uint8_t active) \
{
 	printPgmString(PSTR("\r\n"));
	printPgmString(PSTR("Exec X="));	printInteger(line.steps_x);
	printPgmString(PSTR(", Y="));		printInteger(line.steps_y);
	printPgmString(PSTR(", Z="));		printInteger(line.steps_z);
	printPgmString(PSTR(" uS="));		printInteger(line.microseconds);
	printPgmString(PSTR(" Active="));	
//	printInteger(active);
	if (active & 0x01) {
		printPgmString(PSTR("X"));
	}
	if (active & 0x02) {
		printPgmString(PSTR("Y"));
	}
	if (active & 0x04) {
		printPgmString(PSTR("Z"));
	}
	if (active & 0x08) {
		printPgmString(PSTR("A"));
	}
}

void st_print_done_line(char *axis) \
{
	printPgmString(PSTR(" Done["));		printString(axis);	
 	printPgmString(PSTR("]"));
}

void st_print_active(struct Axes *ax) {
	printPgmString(PSTR("ACTIVE= "));
//	printHex(ax->active_axes);
	if (ax->active_axes & 0x01) {
		printPgmString(PSTR("X"));
	}
	if (ax->active_axes & 0x02) {
		printPgmString(PSTR("Y"));
	}
	if (ax->active_axes & 0x04) {
		printPgmString(PSTR("Z"));
	}
	if (ax->active_axes & 0x08) {
		printPgmString(PSTR("A"));
	}
	printPgmString(PSTR("\r\n"));
}

#endif

