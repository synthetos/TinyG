/*
 * test.c - tinyg test sets
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2015 Alden S. Hart Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "tinyg.h"			// #1
#include "config.h"			// #2
#include "controller.h"
#include "test.h"
#include "xio.h"

#include "tests/test_001_smoke.h" // test basic functionality

/*
 * run_test() - system tests from FLASH invoked by $test=n command
 *
 * 	By convention the character array containing the test must have the same
 *	name as the file name.
 */
uint8_t run_test(nvObj_t *nv)
{
	switch ((uint8_t)nv->value) {
		case 0: { return (STAT_OK);}
		case 1: { xio_open(XIO_DEV_PGM, PGMFILE(&test_smoke),PGM_FLAGS); break;}
		default: {
			fprintf_P(stderr,PSTR("Test #%d not found\n"),(uint8_t)nv->value);
			return (STAT_ERROR);
		}
	}
	tg_set_primary_source(XIO_DEV_PGM);
	return (STAT_OK);
}

/*
 * run_canned_startup() - run a string on startup
 *
 *	Pre-load the USB RX (input) buffer with some test strings that will be called
 *	on startup. Be mindful of the char limit on the read buffer (RX_BUFFER_SIZE).
 *	It's best to create a test file for really complicated things.
 */

void run_canned_startup()	// uncomment in tinyg.h if you want to run this
{
// Example: Run test 99 - 3 different ways to invoke it:
//	xio_queue_RX_string_usb("$test=99\n");		// run test file (doesn't work if text mode is disabled)
//	xio_queue_RX_string_usb("{\"test\":99}\n");	// run test file
//	xio_queue_RX_string_usb("{test:99}\n");		// run test file
}
