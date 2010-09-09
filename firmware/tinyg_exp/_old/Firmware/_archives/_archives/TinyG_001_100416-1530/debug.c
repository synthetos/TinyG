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


void st_print_four_ints(long x, long y, long z, long u) {
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

void st_print_line(struct Line line) {
	printPgmString(PSTR("Line X="));		// ++++ DEBUG CODE
	printInteger(line.steps_x);
	printPgmString(PSTR(", Y="));
	printInteger(line.steps_y);
	printPgmString(PSTR(", Z="));
	printInteger(line.steps_z);
	printPgmString(PSTR(", uS="));
	printInteger(line.microseconds);
//	printPgmString(PSTR(", D="));
//	printHex(line.direction_bits);
//	printPgmString(PSTR(", Steps="));
//	printInteger(line.steps_max);
 	printPgmString(PSTR("\r\n"));
}
