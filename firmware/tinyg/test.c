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
#include "planner.h"
#include "test.h"
#include "util.h"
#include "xio.h"

// regression test files
#ifdef __CANNED_TESTS

#include "tests/test_001_smoke.h" 			// basic functionality
#include "tests/test_002_homing.h"			// G28.1 homing cycles
#include "tests/test_003_squares.h"			// square moves
#include "tests/test_004_arcs.h"			// arc moves
#include "tests/test_005_dwell.h"			// dwells embedded in move sequences
#include "tests/test_006_feedhold.h"		// feedhold - requires manual ! and ~ entry
#include "tests/test_007_Mcodes.h"			// M codes synchronized w/moves (planner queue)
#include "tests/test_008_json.h"			// JSON parser and IO
#include "tests/test_009_inverse_time.h"	// inverse time mode
#include "tests/test_010_rotary.h"			// ABC axes
#include "tests/test_011_small_moves.h"		// small move test
#include "tests/test_012_slow_moves.h"		// slow move test
#include "tests/test_013_coordinate_offsets.h"	// what it says
#include "tests/test_014_microsteps.h"		// test all microstep settings
#include "tests/test_050_mudflap.h"			// mudflap test - entire drawing
#include "tests/test_051_braid.h"			// braid test - partial drawing

#endif

#ifdef __TEST_99
#include "tests/test_099.h"					// diagnostic test file. used to diagnose specific issues
#endif

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
#ifdef __CANNED_TESTS

		case 1: { xio_open(XIO_DEV_PGM, PGMFILE(&test_smoke),PGM_FLAGS); break;}
		case 2: { xio_open(XIO_DEV_PGM, PGMFILE(&test_homing),PGM_FLAGS); break;}
		case 3: { xio_open(XIO_DEV_PGM, PGMFILE(&test_squares),PGM_FLAGS); break;}
		case 4: { xio_open(XIO_DEV_PGM, PGMFILE(&test_arcs),PGM_FLAGS); break;}
		case 5: { xio_open(XIO_DEV_PGM, PGMFILE(&test_dwell),PGM_FLAGS); break;}
		case 6: { xio_open(XIO_DEV_PGM, PGMFILE(&test_feedhold),PGM_FLAGS); break;}
		case 7: { xio_open(XIO_DEV_PGM, PGMFILE(&test_Mcodes),PGM_FLAGS); break;}
		case 8: { xio_open(XIO_DEV_PGM, PGMFILE(&test_json),PGM_FLAGS); break;}
		case 9: { xio_open(XIO_DEV_PGM, PGMFILE(&test_inverse_time),PGM_FLAGS); break;}
		case 10: { xio_open(XIO_DEV_PGM, PGMFILE(&test_rotary),PGM_FLAGS); break;}
		case 11: { xio_open(XIO_DEV_PGM, PGMFILE(&test_small_moves),PGM_FLAGS); break;}
		case 12: { xio_open(XIO_DEV_PGM, PGMFILE(&test_slow_moves),PGM_FLAGS); break;}
		case 13: { xio_open(XIO_DEV_PGM, PGMFILE(&test_coordinate_offsets),PGM_FLAGS); break;}
		case 14: { xio_open(XIO_DEV_PGM, PGMFILE(&test_microsteps),PGM_FLAGS); break;}
		case 50: { xio_open(XIO_DEV_PGM, PGMFILE(&test_mudflap),PGM_FLAGS); break;}
		case 51: { xio_open(XIO_DEV_PGM, PGMFILE(&test_braid),PGM_FLAGS); break;}
#endif
#ifdef __TEST_99
		case 99: { xio_open(XIO_DEV_PGM, PGMFILE(&test_99),PGM_FLAGS); break;}
#endif
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
#ifdef __CANNED_STARTUP

/* Run test 99 */
//	xio_queue_RX_string_usb("$test=99\n");		// run test file (doesn't work if text mode is disabled)
//	xio_queue_RX_string_usb("{\"test\":99}\n");	// run test file
//	xio_queue_RX_string_usb("{test:98}\n");		// run test file
//	xio_queue_RX_string_usb("{test:99}\n");		// run test file

#endif // __CANNED_STARTUP
}
