/*
 * controller.c - tinyg controller and top level parser
 * Part of TinyG project
 *
 * Copyright (c) 2010 - 2013 Alden S. Hart Jr.
 *
 * TinyG is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, 
 * or (at your option) any later version.
 *
 * TinyG is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with TinyG  If not, see <http://www.gnu.org/licenses/>.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
/* See the wiki for module details and additional information:
 *	 http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Developer-Info
 */

#include <ctype.h>				// for parsing
#include <string.h>
#include <avr/pgmspace.h>		// precursor for xio.h
#include <avr/interrupt.h>

#include "tinyg.h"				// #1 unfortunately, there are some dependencies
#include "config.h"				// #2
#include "controller.h"
#include "settings.h"
#include "json_parser.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "plan_arc.h"
#include "planner.h"
#include "stepper.h"
#include "system.h"
#include "gpio.h"
#include "report.h"
#include "util.h"
#include "help.h"
#include "xio/xio.h"
#include "xmega/xmega_rtc.h"
#include "xmega/xmega_init.h"

// local helpers
static void _controller_HSM(void);
static uint8_t _dispatch(void);
static uint8_t _reset_handler(void);
static uint8_t _bootloader_handler(void);
static uint8_t _limit_switch_handler(void);
static uint8_t _shutdown_idler(void);
static uint8_t _system_assertions(void);
static uint8_t _feedhold_handler(void);
static uint8_t _cycle_start_handler(void);
static uint8_t _sync_to_tx_buffer(void);
static uint8_t _sync_to_planner(void);

/*
 * tg_init() - controller init
 */

void tg_init(uint8_t std_in, uint8_t std_out, uint8_t std_err) 
{
	tg.magic_start = MAGICNUM;
	tg.magic_end = MAGICNUM;
	tg.fw_build = TINYG_FIRMWARE_BUILD;
	tg.fw_version = TINYG_FIRMWARE_VERSION;	// NB: HW version is set from EEPROM

	xio_set_stdin(std_in);
	xio_set_stdout(std_out);
	xio_set_stderr(std_err);
	tg.default_src = std_in;
	tg_set_active_source(tg.default_src);	// set initial active source
}

/* 
 * tg_controller() - MAIN LOOP - top-level controller
 *
 * The order of the dispatched tasks is very important. 
 * Tasks are ordered by increasing dependency (blocking hierarchy).
 * Tasks that are dependent on completion of lower-level tasks must be
 * later in the list than the task(s) they are dependent upon. 
 *
 * Tasks must be written as continuations as they will be called repeatedly, 
 * and are called even if they are not currently active. 
 *
 * The DISPATCH macro calls the function and returns to the controller parent 
 * if not finished (TG_EAGAIN), preventing later routines from running 
 * (they remain blocked). Any other condition - OK or ERR - drops through 
 * and runs the next routine in the list.
 *
 * A routine that had no action (i.e. is OFF or idle) should return TG_NOOP
 *
 * Useful reference on state machines:
 * http://johnsantic.com/comp/state.html, "Writing Efficient State Machines in C"
 */

void tg_controller() 
{ 
	while (true) { 
		_controller_HSM();
	}
}

#define	DISPATCH(func) if (func == TG_EAGAIN) return; 
static void _controller_HSM()
{
//----- kernel level ISR handlers ----(flags are set in ISRs)-----------//
											// Order is important:
	DISPATCH(_reset_handler());				// 1. software reset received
	DISPATCH(_bootloader_handler());		// 2. received ESC char to start bootloader
	DISPATCH(_limit_switch_handler());		// 3. limit switch has been thrown
	DISPATCH(_shutdown_idler());			// 4. idle in shutdown state
	DISPATCH(_system_assertions());			// 5. system integrity assertions
	DISPATCH(_feedhold_handler());			// 6. feedhold requested
	DISPATCH(_cycle_start_handler());		// 7. cycle start requested

//----- planner hierarchy for gcode and cycles -------------------------//
	DISPATCH(rpt_status_report_callback());	// conditionally send status report
	DISPATCH(rpt_queue_report_callback());	// conditionally send queue report
	DISPATCH(mp_plan_hold_callback());		// plan a feedhold
	DISPATCH(mp_end_hold_callback());		// end a feedhold
	DISPATCH(ar_arc_callback());			// arc generation runs behind lines
	DISPATCH(cm_homing_callback());			// G28.2 continuation

//----- command readers and parsers ------------------------------------//
	DISPATCH(_sync_to_planner());			// ensure there is at least one free buffer in planning queue
	DISPATCH(_sync_to_tx_buffer());			// sync with TX buffer (pseudo-blocking)
	DISPATCH(cfg_baud_rate_callback());		// perform baud rate update (must be after TX sync)
	DISPATCH(_dispatch());					// read and execute next command
}

/***************************************************************************** 
 * _dispatch() - dispatch line received from active input device
 *
 *	Reads next command line and dispatches to relevant parser or action
 *	Accepts commands if the move queue has room - EAGAINS if it doesn't
 *	Manages cutback to serial input from file devices (EOF)
 *	Also responsible for prompts and for flow control 
 */

static uint8_t _dispatch()
{
	uint8_t status;

	// read input line or return if not a completed line
	// xio_gets() is a non-blocking workalike of fgets()
	if ((status = xio_gets(tg.active_src, tg.in_buf, sizeof(tg.in_buf))) != TG_OK) {
		if (status == TG_EOF) {					// EOF can come from file devices only
			fprintf_P(stderr, PSTR("End of command file\n"));
			tg_reset_source();					// reset to default source
		}
		// Note that TG_EAGAIN, TG_NOOP etc. will just flow through
		return (status);
	}
	cmd_reset_list();
	tg.linelen = strlen(tg.in_buf)+1;
	strncpy(tg.saved_buf, tg.in_buf, SAVED_BUFFER_LEN-1);	// save input buffer for reporting

	// dispatch the new text line
	switch (toupper(tg.in_buf[0])) {

		case NUL: { 							// blank line (just a CR)
			if (cfg.comm_mode != JSON_MODE) {
				tg_text_response(TG_OK, tg.saved_buf);
			}
			break;
		}
		case 'H': { 							// intercept help screens
			cfg.comm_mode = TEXT_MODE;
			print_general_help();
			tg_text_response(TG_OK, tg.in_buf);
			break;
		}
		case '$': case '?':{ 					// text-mode configs
			cfg.comm_mode = TEXT_MODE;
			tg_text_response(cfg_text_parser(tg.in_buf), tg.saved_buf);
			break;
		}
		case '{': { 							// JSON input
			cfg.comm_mode = JSON_MODE;
			js_json_parser(tg.in_buf);
			break;
		}
		default: {								// anything else must be Gcode
			if (cfg.comm_mode == JSON_MODE) {
				strncpy(tg.out_buf, tg.in_buf, INPUT_BUFFER_LEN -8);	// use out_buf as temp
				sprintf(tg.in_buf,"{\"gc\":\"%s\"}\n", tg.out_buf);		// '-8' is used for JSON chars
				js_json_parser(tg.in_buf);
			} else {
				tg_text_response(gc_gcode_parser(tg.in_buf), tg.saved_buf);
			}
		}
	}
	return (TG_OK);
}

/************************************************************************************
 * tg_text_response() - text mode responses
 */
static const char prompt_mm[] PROGMEM = "mm";
static const char prompt_in[] PROGMEM = "inch";
static const char prompt_ok[] PROGMEM = "tinyg [%S] ok> ";
static const char prompt_err[] PROGMEM = "tinyg [%S] err: %s: %s ";

void tg_text_response(const uint8_t status, const char *buf)
{
	if (cfg.text_verbosity == TV_SILENT) return;	// skip all this

	const char *units;								// becomes pointer to progmem string
	if (cm_get_units_mode() != INCHES) { 
		units = (PGM_P)&prompt_mm;
	} else {
		units = (PGM_P)&prompt_in;
	}
	if ((status == TG_OK) || (status == TG_EAGAIN) || (status == TG_NOOP) || (status == TG_ZERO_LENGTH_MOVE)) {
		fprintf_P(stderr, (PGM_P)&prompt_ok, units);
	} else {
		char status_message[STATUS_MESSAGE_LEN];
		fprintf_P(stderr, (PGM_P)prompt_err, units, rpt_get_status_message(status, status_message), buf);
	}
	cmdObj_t *cmd = cmd_body+1;
	if (cmd->token[0] == 'm') {
		fprintf(stderr, *cmd->stringp);
	}
	fprintf(stderr, "\n");
}

/**** Utilities ****
 * _sync_to_tx_buffer() - return eagain if TX queue is backed up
 * _sync_to_planner() - return eagain if planner is not ready for a new command
 * tg_reset_source() - reset source to default input device (see note)
 * tg_set_active_source() - set current input source
 *
 * Note: Once multiple serial devices are supported reset_source() should
 *	be expanded to also set the stdout/stderr console device so the prompt
 *	and other messages are sent to the active device.
 */

static uint8_t _sync_to_tx_buffer()
{
	if ((xio_get_tx_bufcount_usart(ds[XIO_DEV_USB].x) >= XOFF_TX_LO_WATER_MARK)) {
		return (TG_EAGAIN);
	}
	return (TG_OK);
}

static uint8_t _sync_to_planner()
{
	if (mp_get_planner_buffers_available() == 0) { 
		return (TG_EAGAIN);
	}
	return (TG_OK);
}

void tg_reset_source()
{
	tg_set_active_source(tg.default_src);
}

void tg_set_active_source(uint8_t dev)
{
	tg.active_src = dev;				// dev = XIO device #. See xio.h
}

/**** Signal handlers ****
 * _reset_handler()
 * _feedhold_handler()
 * _cycle_start_handler()
 * _bootloader_handler()
 */
static uint8_t _reset_handler(void)
{
	if (sig.sig_reset == false) { return (TG_NOOP);}
//	sig.sig_reset = false;				// why bother?
	tg_reset();							// hard reset - identical to hitting RESET button
	return (TG_EAGAIN);
}

static uint8_t _feedhold_handler(void)
{
	if (sig.sig_feedhold == false) { return (TG_NOOP);}
	sig.sig_feedhold = false;
	cm_feedhold();
	return (TG_EAGAIN);					// best to restart the control loop
}

static uint8_t _cycle_start_handler(void)
{
	if (sig.sig_cycle_start == false) { return (TG_NOOP);}
	sig.sig_cycle_start = false;
	cm_cycle_start();
	return (TG_EAGAIN);					// best to restart the control loop
}

/*
 * _bootloader_handler() - executes a software reset using CCPWrite
 *
 * 	An earlier attempt turned off interrupts, reset the stack pointer and jumped
 *	to the boot region. The assembly code is preserved below as an example.
 *	All the values in the assembly are hard coded because the pre-processor 
 *	doesn't open strings to process #defines.
 *
 *	  RAMEND 	0x05ff					// starting location for stack pointer
 *	  SPL		0x3d					// stack pointer lo
 *	  SPH		0x3e					// stack pointer hi
 *	  BOOTSTRT	0x030000				// start of boot region
 *
	asm("ldi r16, 0xff" "\n\t" \
		"out 0x3d, r16" "\n\t" \
		"ldi r16, 0x5f" "\n\t" \
		"out 0x3e, r16" "\n\t" \
		"jmp 0x030000"  "\n\t" \
		);
 *
 *  Refs:
 *	  https://sites.google.com/site/avrasmintro/
 *	  http://www.stanford.edu/class/ee281/projects/aut2002/yingzong-mouse/media/GCCAVRInlAsmCB.pdf
 */
static uint8_t _bootloader_handler(void)
{
	if (sig.sig_request_bootloader == false) { return (TG_NOOP);}
	cli();
	CCPWrite(&RST.CTRL, RST_SWRST_bm);
	return (TG_EAGAIN);					// never gets here but keeps the compiler happy
}

/*
 * _limit_switch_handler() - shut down system if limit switch fired
 */
static uint8_t _limit_switch_handler(void)
{
	if (cm_get_machine_state() == MACHINE_SHUTDOWN) { return (TG_NOOP);}
	if (gpio_get_limit_thrown() == false) return (TG_NOOP);
//	cm_shutdown(gpio_get_sw_thrown); // unexplained complier warning: passing argument 1 of 'cm_shutdown' makes integer from pointer without a cast
	cm_shutdown(sw.sw_num_thrown);
	return (TG_OK);
}

/* 
 * _shutdown_idler() - revent any further activity form occurring if shut down
 *
 *	This function returns EAGAIN causing the control loop to never advance beyond
 *	this point. It's important that the reset handler is still called so a SW reset
 *	(ctrl-x) can be processed.
 */
#define LED_COUNTER 25000

static uint8_t _shutdown_idler(void)
{
	if (cm_get_machine_state() != MACHINE_SHUTDOWN) { return (TG_OK);}

	if (--tg.led_counter < 0) {
		tg.led_counter = LED_COUNTER;
		if (tg.led_state == 0) {
			gpio_led_on(INDICATOR_LED);
			tg.led_state = 1;
		} else {
			gpio_led_off(INDICATOR_LED);
			tg.led_state = 0;
		}
	}
	return (TG_EAGAIN);	 // EAGAIN prevents any other actions from running
}

/* 
 * _system_assertions() - check memory integrity and other assertions
 */
uint8_t _system_assertions()
{
	uint8_t value = 0;

	if (tg.magic_start		!= MAGICNUM) { value = 1; }		// Note: reported VALue is offset by ALARM_MEMORY_OFFSET
	if (tg.magic_end		!= MAGICNUM) { value = 2; }
	if (cm.magic_start 		!= MAGICNUM) { value = 3; }
	if (cm.magic_end		!= MAGICNUM) { value = 4; }
	if (gm.magic_start		!= MAGICNUM) { value = 5; }
	if (gm.magic_end 		!= MAGICNUM) { value = 6; }
	if (cfg.magic_start		!= MAGICNUM) { value = 7; }
	if (cfg.magic_end		!= MAGICNUM) { value = 8; }
	if (cmdStr.magic_start	!= MAGICNUM) { value = 9; }
	if (cmdStr.magic_end	!= MAGICNUM) { value = 10; }
	if (mb.magic_start		!= MAGICNUM) { value = 11; }
	if (mb.magic_end		!= MAGICNUM) { value = 12; }
	if (mr.magic_start		!= MAGICNUM) { value = 13; }
	if (mr.magic_end		!= MAGICNUM) { value = 14; }
	if (ar.magic_start		!= MAGICNUM) { value = 15; }
	if (ar.magic_end		!= MAGICNUM) { value = 16; }
	if (st_get_st_magic()	!= MAGICNUM) { value = 17; }
	if (st_get_sps_magic()	!= MAGICNUM) { value = 18; }
	if (rtc.magic_end 		!= MAGICNUM) { value = 19; }
	xio_assertions(&value);									// run xio assertions

	if (value == 0) { return (TG_OK);}
	rpt_exception(TG_MEMORY_CORRUPTION, value);
	cm_shutdown(ALARM_MEMORY_OFFSET + value);	
	return (TG_EAGAIN);
}
