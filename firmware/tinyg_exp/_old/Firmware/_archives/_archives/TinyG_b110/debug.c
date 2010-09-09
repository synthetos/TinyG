#include <math.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "xmega_support.h"
#include <util/delay.h>
#include "stepper.h"
#include "config.h"
#include "nuts_bolts.h"

#include "debug.h"

/* NOTE: seme calling examples are provided in comments */

//	printPgmString(PSTR());

//	st_print_axis(&ax.x, "X");					// ++++ DEBUG STATEMENT ++++
//	st_print_axis(&ax.y, "Y");					// ++++ DEBUG STATEMENT ++++
//	st_print_axis(&ax.z, "Z");					// ++++ DEBUG STATEMENT ++++

void st_print_axis(struct Axis *A, char *label)	// called as st_print_axis(&ax.x, "X");
{
	printString(label);					printPgmString(PSTR(" axis:"));
	printPgmString(PSTR(" Count="));	printInteger(A->counter);
	printPgmString(PSTR(" Port="));		printInteger(A->port->IN);	// reading IN reports state
	printPgmString(PSTR("\r\n"));
}

void st_print_four_ints(long x, long y, long z, long u) 
{
	printPgmString(PSTR("Line: X="));	printInteger(x);
	printPgmString(PSTR(" Y="));		printInteger(y);
	printPgmString(PSTR(" Z="));		printInteger(z);
	printPgmString(PSTR(" uS="));		printInteger(u);
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
	printPgmString(PSTR(" Active="));	printInteger(active);
}

void st_print_done_line(char *axis) \
{
	printPgmString(PSTR(" Done["));		printString(axis);	
 	printPgmString(PSTR("]"));
}

void st_print_active(struct Axes *ax) {
	printPgmString(PSTR("ACTIVE= "));	printHex(ax->active_axes);
	printPgmString(PSTR("\r\n"));
}



