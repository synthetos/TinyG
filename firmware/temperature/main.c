/*
 * main.c - Sharkfin-34 embedded stepper motor controller 
 * Part of Sharkfin-34 project
 *
 * Copyright (c) 2012 Alden S Hart Jr.
 *
 * the Kinen Motion Control System is licensed under the OSHW 1.0 license
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#include "kinen.h"
#include "kinen_slave_328p.h"
#include "tmc262.h"

int main(void)
{
	ki_init();					// initializes any OCB master, slave and devices
	sei(); 						// enable interrupts

	// you can comment out the unit tests for production
	device_unit_tests();
	
	// go there and never return
	ki_main_loop(); 			// handle whatever events pop up
	return false;   			// never reached
}
