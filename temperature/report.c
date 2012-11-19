/*
 * report.c - contains all reporting statements
 * Part of Kinen project
 *
 * Copyright (c) 2012 Alden S. Hart Jr.
 *
 * The Kinen Motion Control System is licensed under the LGPL license
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <avr/pgmspace.h>

#include "print.h"
#include "report.h"
#include "tinyg_tc.h"

#define MSGLEN 24
char msg[MSGLEN];

/*** Strings and string arrays in program memory ***/

static const char initialized[] PROGMEM = "\nDevice Initialized\n"; 

static const char msg_scode0[] PROGMEM = "";
static const char msg_scode1[] PROGMEM = "  Taking Reading";
static const char msg_scode2[] PROGMEM = "  Bad Reading";
static const char msg_scode3[] PROGMEM = "  Disconnected";
static const char msg_scode4[] PROGMEM = "  No Power";
static PGM_P const msg_scode[] PROGMEM = { msg_scode0, msg_scode1, msg_scode2, msg_scode3, msg_scode4 };

static const char msg_hstate0[] PROGMEM = "  OK";
static const char msg_hstate1[] PROGMEM = "  Shutdown";
static const char msg_hstate2[] PROGMEM = "  Heating";
static const char msg_hstate3[] PROGMEM = "  REGULATED";
static PGM_P const msg_hstate[] PROGMEM = { msg_hstate0, msg_hstate1, msg_hstate2, msg_hstate3 };

/*** Display routines ***/

void rpt_initialized()
{
//	printPgmString((PGM_P)(pgm_read_word(initialized))); 
	printPgmString(PSTR("\nDevice Initialized\n")); 
}

void rpt_readout()
{
	printPgmString(PSTR("Temp:")); printFloat(sensor.temperature);
	printPgmString(PSTR("  PWM:")); printFloat(pid.output);
//	printPgmString(PSTR("  s[0]:")); printFloat(sensor.sample[0]);
	printPgmString(PSTR("  StdDev:")); printFloat(sensor.std_dev);
//	printPgmString(PSTR("  Samples:")); printFloat(sensor.samples);
	printPgmString(PSTR("  Err:")); printFloat(pid.error);
	printPgmString(PSTR("  I:")); printFloat(pid.integral);
//	printPgmString(PSTR("  D:")); printFloat(pid.derivative);
//	printPgmString(PSTR("  Hy:")); printFloat(heater.hysteresis);

	printPgmString((PGM_P)pgm_read_word(&msg_hstate[heater.state]));
//	printPgmString((PGM_P)pgm_read_word(&msg_scode[sensor.code]));

	printPgmString(PSTR("\n")); 

}

