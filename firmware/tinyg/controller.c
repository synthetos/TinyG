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

#include <avr/interrupt.h>
#include <avr/wdt.h>			// used for software reset

#include "tinyg.h"				// #1 unfortunately, there are some dependencies
#include "config.h"				// #2
#include "controller.h"
#include "settings.h"
#include "json_parser.h"
#include "text_parser.h"
#include "gcode_parser.h"
#include "canonical_machine.h"
#include "plan_arc.h"
#include "planner.h"
#include "stepper.h"
#include "system.h"
#include "switch.h"
#include "gpio.h"
#include "report.h"
#include "util.h"
#include "help.h"
#include "xio/xio.h"
#include "xmega/xmega_rtc.h"
#include "xmega/xmega_init.h"

// local helpers
static void _controller_HSM(void);
static stat_t _alarm_idler(void);
static stat_t _normal_idler(void);
static stat_t _limit_switch_handler(void);
static stat_t _system_assertions(void);
//static stat_t _cycle_start_handler(void);
static stat_t _sync_to_planner(void);
static stat_t _sync_to_tx_buffer(void);
static stat_t _command_dispatch(void);

// prep for export to other modules:
stat_t hardware_hard_reset_handler(void);
stat_t hardware_bootloader_handler(void);

/***********************************************************************************
 **** CODE *************************************************************************
 ***********************************************************************************/
/*
 * controller_init() - controller init
 */

void controller_init(uint8_t std_in, uint8_t std_out, uint8_t std_err) 
{
	cs.magic_start = MAGICNUM;
	cs.magic_end = MAGICNUM;
	cs.fw_build = TINYG_FIRMWARE_BUILD;
	cs.fw_version = TINYG_FIRMWARE_VERSION;	// NB: HW version is set from EEPROM

	cs.linelen = 0;							// initialize index for read_line()
	cs.state = CONTROLLER_STARTUP;			// ready to run startup lines
	cs.hard_reset_requested = false;
	cs.bootloader_requested = false;

	xio_set_stdin(std_in);
	xio_set_stdout(std_out);
	xio_set_stderr(std_err);
	cs.default_src = std_in;
	tg_set_primary_source(cs.default_src);	// set primary source
}

/* 
 * controller_run() - MAIN LOOP - top-level controller
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
 * if not finished (STAT_EAGAIN), preventing later routines from running 
 * (they remain blocked). Any other condition - OK or ERR - drops through 
 * and runs the next routine in the list.
 *
 * A routine that had no action (i.e. is OFF or idle) should return STAT_NOOP
 */

void controller_run() 
{ 
	while (true) { 
		_controller_HSM();
	}
}

#define	DISPATCH(func) if (func == STAT_EAGAIN) return; 
static void _controller_HSM()
{
//----- ISRs. These should be considered the highest priority scheduler functions ----//
/*	
 *	HI	Stepper DDA pulse generation			// see stepper.h
 *	HI	Stepper load routine SW interrupt		// see stepper.h
 *	HI	Dwell timer counter 					// see stepper.h
 *	MED	GPIO1 switch port - limits / homing		// see gpio.h
 *  MED	Serial RX for USB						// see xio_usart.h
 *  LO	Segment execution SW interrupt			// see stepper.h
 *  LO	Serial TX for USB & RS-485				// see xio_usart.h
 *	LO	Real time clock interrupt				// see xmega_rtc.h
 */
//----- kernel level ISR handlers ----(flags are set in ISRs)-----------//
												// Order is important:
	DISPATCH(hardware_hard_reset_handler());	// 1. handle hard reset requests
	DISPATCH(hardware_bootloader_handler());	// 2. handle requests to enter bootloader
	DISPATCH(_alarm_idler());					// 3. idle in alarm state (shutdown)
//	DISPATCH( poll_switches());					// 4. run a switch polling cycle
	DISPATCH(_limit_switch_handler());			// 5. limit switch has been thrown

	DISPATCH(cm_feedhold_sequencing_callback());// 6a. feedhold state machine runner
	DISPATCH(mp_plan_hold_callback());			// 6b. plan a feedhold from line runtime

//	DISPATCH(_cycle_start_handler());			// 7. cycle start requested
	DISPATCH(_system_assertions());				// 8. system integrity assertions

//----- planner hierarchy for gcode and cycles -------------------------//

	DISPATCH(st_motor_disable_callback());		// stepper motor disable timer
//	DISPATCH(switch_debounce_callback());		// debounce switches
	DISPATCH(rpt_status_report_callback());		// conditionally send status report
	DISPATCH(rpt_queue_report_callback());		// conditionally send queue report
	DISPATCH(ar_arc_callback());				// arc generation runs behind lines
	DISPATCH(cm_homing_callback());				// G28.2 continuation
	DISPATCH(cm_probe_callback());				// G38.2 continuation

//----- command readers and parsers ------------------------------------//

	DISPATCH(_sync_to_planner());				// ensure there is at least one free buffer in planning queue
	DISPATCH(_sync_to_tx_buffer());				// sync with TX buffer (pseudo-blocking)
	DISPATCH(set_baud_callback());				// perform baud rate update (must be after TX sync)
	DISPATCH(_command_dispatch());				// read and execute next command
	DISPATCH(_normal_idler());					// blink LEDs slowly to show everything is OK
}

/***************************************************************************** 
 * _command_dispatch() - dispatch line received from active input device
 *
 *	Reads next command line and dispatches to relevant parser or action
 *	Accepts commands if the move queue has room - EAGAINS if it doesn't
 *	Manages cutback to serial input from file devices (EOF)
 *	Also responsible for prompts and for flow control 
 */

static stat_t _command_dispatch()
{
	uint8_t status;

	// read input line or return if not a completed line
	// xio_gets() is a non-blocking workalike of fgets()
	while (true) {
		if ((status = xio_gets(cs.primary_src, cs.in_buf, sizeof(cs.in_buf))) == STAT_OK) {
			cs.bufp = cs.in_buf;
			break;
		}
		// handle end-of-file from file devices
		if (status == STAT_EOF) {					// EOF can come from file devices only
			if (cfg.comm_mode == TEXT_MODE) {
				fprintf_P(stderr, PSTR("End of command file\n"));
			} else {
				rpt_exception(STAT_EOF, 0);		// not really an exception
			}
			tg_reset_source();					// reset to default source
		}
		return (status);						// Note: STAT_EAGAIN, errors, etc. will drop through
	}
	cs.linelen = strlen(cs.in_buf)+1;					// linelen only tracks primary input
	strncpy(cs.saved_buf, cs.bufp, SAVED_BUFFER_LEN-1);	// save input buffer for reporting

	// dispatch the new text line
	switch (toupper(*cs.bufp)) {				// first char

//		case '!': { cm_request_feedhold(); break; }		// include for diagnostics
//		case '@': { cm_request_queue_flush(); break; }
//		case '~': { cm_request_cycle_start(); break; }

		case NUL: { 							// blank line (just a CR)
			if (cfg.comm_mode != JSON_MODE) {
				tg_text_response(STAT_OK, cs.saved_buf);
			}
			break;
		}
		case 'H': { 							// intercept help screens
			cfg.comm_mode = TEXT_MODE;
			print_general_help();
			tg_text_response(STAT_OK, cs.bufp);
			break;
		}
		case '$': case '?':{ 					// text-mode configs
			cfg.comm_mode = TEXT_MODE;
			tg_text_response(text_parser(cs.bufp), cs.saved_buf);
			break;
		}
		case '{': { 							// JSON input
			cfg.comm_mode = JSON_MODE;
			json_parser(cs.bufp);
			break;
		}
		default: {								// anything else must be Gcode
			if (cfg.comm_mode == JSON_MODE) {
				strncpy(cs.out_buf, cs.bufp, INPUT_BUFFER_LEN -8);	// use out_buf as temp
				sprintf(cs.bufp,"{\"gc\":\"%s\"}\n", cs.out_buf);
				json_parser(cs.bufp);
			} else {
				tg_text_response(gc_gcode_parser(cs.bufp), cs.saved_buf);
			}
		}
	}
	return (STAT_OK);
}

/**** Local Utilities ********************************************************/
/*
 * _alarm_idler() - blink rapidly and prevent further activity from occurring
 * _normal_idler() - blink Indicator LED slowly to show everything is OK
 *
 *	Alarm idler flashes indicator LED rapidly to show everything is not OK. 
 *	Alarm function returns EAGAIN causing the control loop to never advance beyond 
 *	this point. It's important that the reset handler is still called so a SW reset 
 *	(ctrl-x) or bootloader request can be processed.
 */

static stat_t _alarm_idler(void)
{
	if (cm_get_machine_state() != MACHINE_ALARM) { return (STAT_OK);}

	if (SysTickTimer_getValue() > cs.led_timer) {
		cs.led_timer = SysTickTimer_getValue() + LED_ALARM_TIMER;
		IndicatorLed_toggle();
	}
	return (STAT_EAGAIN);	 // EAGAIN prevents any other actions from running
}

static stat_t _normal_idler(  )
{
/*
	if (SysTickTimer_getValue() > cs.led_timer) {
		cs.led_timer = SysTickTimer_getValue() + LED_NORMAL_TIMER;
//		IndicatorLed_toggle();
	}
*/
	return (STAT_OK);
}

/*
 * tg_reset_source() 		 - reset source to default input device (see note)
 * tg_set_primary_source() 	 - set current primary input source
 * tg_set_secondary_source() - set current primary input source
 *
 * Note: Once multiple serial devices are supported reset_source() should
 * be expanded to also set the stdout/stderr console device so the prompt
 * and other messages are sent to the active device.
 */

void tg_reset_source() { tg_set_primary_source(cs.default_src);}
void tg_set_primary_source(uint8_t dev) { cs.primary_src = dev;}
void tg_set_secondary_source(uint8_t dev) { cs.secondary_src = dev;}

/*
 * _sync_to_tx_buffer() - return eagain if TX queue is backed up
 * _sync_to_planner() - return eagain if planner is not ready for a new command
 */
static stat_t _sync_to_tx_buffer()
{
	if ((xio_get_tx_bufcount_usart(ds[XIO_DEV_USB].x) >= XOFF_TX_LO_WATER_MARK)) {
		return (STAT_EAGAIN);
	}
	return (STAT_OK);
}

static stat_t _sync_to_planner()
{
	if (mp_get_planner_buffers_available() < PLANNER_BUFFER_HEADROOM) { // allow up to N planner buffers for this line
		return (STAT_EAGAIN);
	}
	return (STAT_OK);
}

/*
 * _limit_switch_handler() - shut down system if limit switch fired
 */
static stat_t _limit_switch_handler(void)
{
	if (cm_get_machine_state() == MACHINE_ALARM) { return (STAT_NOOP);}
	if (get_limit_switch_thrown() == false) return (STAT_NOOP);
//	cm_alarm(gpio_get_sw_thrown); // unexplained complier warning: passing argument 1 of 'cm_shutdown' makes integer from pointer without a cast
	canonical_machine_alarm(sw.sw_num_thrown);
	return (STAT_OK);
}

/* 
 * _system_assertions() - check memory integrity and other assertions
 */
uint8_t _system_assertions()
{
	uint8_t value = 0;

	if (cs.magic_start				!= MAGICNUM) { value = 1; }		// Note: reported VALue is offset by ALARM_MEMORY_OFFSET
	if (cs.magic_end				!= MAGICNUM) { value = 2; }
	if (cm.magic_start 				!= MAGICNUM) { value = 3; }
	if (cm.magic_end				!= MAGICNUM) { value = 4; }
	if (gm.magic_start				!= MAGICNUM) { value = 5; }
	if (gm.magic_end 				!= MAGICNUM) { value = 6; }
	if (cfg.magic_start				!= MAGICNUM) { value = 7; }
	if (cfg.magic_end				!= MAGICNUM) { value = 8; }
	if (cmdStr.magic_start			!= MAGICNUM) { value = 9; }
	if (cmdStr.magic_end			!= MAGICNUM) { value = 10; }
	if (mb.magic_start				!= MAGICNUM) { value = 11; }
	if (mb.magic_end				!= MAGICNUM) { value = 12; }
	if (mr.magic_start				!= MAGICNUM) { value = 13; }
	if (mr.magic_end				!= MAGICNUM) { value = 14; }
	if (ar.magic_start				!= MAGICNUM) { value = 15; }
	if (ar.magic_end				!= MAGICNUM) { value = 16; }
	if (st_get_stepper_run_magic()	!= MAGICNUM) { value = 17; }
	if (st_get_stepper_prep_magic()	!= MAGICNUM) { value = 18; }
	if (rtc.magic_end 		!= MAGICNUM) { value = 19; }
	xio_assertions(&value);									// run xio assertions

	if (value == 0) { return (STAT_OK);}
	rpt_exception(STAT_MEMORY_FAULT, value);
	canonical_machine_alarm(ALARM_MEMORY_OFFSET + value);	
	return (STAT_EAGAIN);
}

//============================================================================
//=========== MOVE TO xmega_tinyg.c ==========================================
//============================================================================

/**** Hardware Reset Handlers *************************************************
 * hardware_request_hard_reset()
 * hardware_hard_reset()		 - hard reset using watchdog timer
 * hardware_hard_reset_handler() - controller's rest handler
 */
void hardware_request_hard_reset() { cs.hard_reset_requested = true; }

void hardware_hard_reset(void)			// software hard reset using the watchdog timer
{
	wdt_enable(WDTO_15MS);
	while (true);						// loops for about 15ms then resets
}

stat_t hardware_hard_reset_handler(void)
{
	if (cs.hard_reset_requested == false) { return (STAT_NOOP);}
	hardware_hard_reset();				// hard reset - identical to hitting RESET button
	return (STAT_EAGAIN);
}

/**** Bootloader Handlers *****************************************************
 * hardware_request_bootloader()
 * hareware_request_bootloader_handler() - executes a software reset using CCPWrite
 */
void hardware_request_bootloader() { cs.bootloader_requested = true;}

stat_t hardware_bootloader_handler(void)
{
	if (cs.bootloader_requested == false) { return (STAT_NOOP);}
	cli();
	CCPWrite(&RST.CTRL, RST_SWRST_bm);  // fire a software reset
	return (STAT_EAGAIN);					// never gets here but keeps the compiler happy
}


//============================================================================
//=========== MOVE TO text_parser.c ==========================================
//============================================================================

/************************************************************************************
 * tg_text_response() - text mode responses
 */
 /*
static const char prompt_mm[] PROGMEM = "mm";
static const char prompt_in[] PROGMEM = "inch";
static const char prompt_ok[] PROGMEM = "tinyg [%S] ok> ";
static const char prompt_err[] PROGMEM = "tinyg [%S] err: %s: %s ";

void tg_text_response(const uint8_t status, const char *buf)
{
	if (cfg.text_verbosity == TV_SILENT) return;	// skip all this

	const char *units;								// becomes pointer to progmem string
	if (cm_get_model_units_mode() != INCHES) { 
		units = (PGM_P)&prompt_mm;
	} else {
		units = (PGM_P)&prompt_in;
	}
//	if ((status == STAT_OK) || (status == STAT_EAGAIN) || (status == STAT_NOOP) || (status == STAT_ZERO_LENGTH_MOVE)) {
	if ((status == STAT_OK) || (status == STAT_EAGAIN) || (status == STAT_NOOP)) {
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
*/
