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

void rpt_initialized()
{
	printPgmString(PSTR("\nDevice Initialized\n")); 
}

void rpt_heater_readout()
{
	if (--heater.readout < 0) {
		heater.readout = 5;
		printPgmString(PSTR("Temp: ")); 
		printFloat(heater.temperature);
		printPgmString(PSTR("  PID: ")); 
		printFloat(pid.output);
		printPgmString(PSTR("\n")); 
	}
}
