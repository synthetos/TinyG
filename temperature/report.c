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

static const char msg_scode0[] PROGMEM = "Idle";
static const char msg_scode1[] PROGMEM = "Taking Reading";
static const char msg_scode2[] PROGMEM = "Bad Reading";
static const char msg_scode3[] PROGMEM = "Disconnected";
static const char msg_scode4[] PROGMEM = "No Power";
static PGM_P const msg_scode[] PROGMEM = { msg_scode0, msg_scode1, msg_scode2, msg_scode3, msg_scode4};

/*** Display routines ***/

void rpt_initialized()
{
//	printPgmString((PGM_P)(pgm_read_word(initialized))); 
	printPgmString(PSTR("\nDevice Initialized\n")); 
}

void rpt_readout()
{
	printPgmString(PSTR("Temp: ")); printFloat(sensor.temperature);
//	printPgmString(PSTR(" s[0]: ")); printFloat(sensor.sample[0]);				//++++++
	printPgmString(PSTR(" StdDev: ")); printFloat(sensor.std_dev);				//++++++
	printPgmString(PSTR(" Error: ")); printFloat(pid.error);				//++++++
	printPgmString(PSTR(" PWM: ")); printFloat(pid.output);
	printPgmString(PSTR("  "));
	rpt_sensor();
}

void rpt_heater_readout()
{
	printPgmString(PSTR("Temp: "));  printFloat(heater.temperature);
	printPgmString(PSTR("  PID: ")); printFloat(pid.output);
	printPgmString(PSTR("\n")); 
}

void rpt_sensor()
{
	printPgmString((PGM_P)pgm_read_word(&msg_scode[sensor.code]));
	printPgmString(PSTR("\n")); 

//	strncpy_P(msg,(PGM_P)pgm_read_word(&msg_scode[sensor.code]), MSGLEN);
//	printString(msg);

//	printPgmString(PSTR(pgm_read_word(&msg_scode[sensor.code]));
//	printPgmString(&msg_scode[sensor.code]);
}
//	return (msg);
