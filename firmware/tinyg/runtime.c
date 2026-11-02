/*
 * runtime.c - runtime session family "rt": idle heartbeat and quiescence token (see runtime.h)
 */
#include "tinyg.h"
#include "config.h"
#include "controller.h"
#include "canonical_machine.h"
#include "planner.h"
#include "stepper.h"
#include "runtime.h"
#include "util.h"
#include "xio.h"

#include <stdio.h>
#ifdef __AVR
#include <avr/interrupt.h>
#endif

rtSingleton_t rt;

void rt_init(void)
{
	memset(&rt, 0, sizeof(rt));
}

void rt_qt_invalidate(void)
{
	rt.qt_valid = false;					// one byte: safe from interrupts and the main loop alike
}

/*
 * _machine_quiet() - nothing moving, held, flushing or queued, and no prepared segment
 *
 *	One snapshot with interrupts masked: the stepper's counters are wider than a byte.
 */
static bool _machine_quiet(void)
{
#ifdef __AVR
	uint8_t saved_sreg = SREG;
	cli();
#endif
	bool quiet = (cm.machine_state != MACHINE_CYCLE) && (cm.cycle_state == CYCLE_OFF) &&
				 (cm.motion_state == MOTION_STOP) && (cm.hold_state == FEEDHOLD_OFF) &&
				 (cm.queue_flush_state == FLUSH_OFF) && !cm.end_hold_requested && !cm.job_kill_requested &&
				 (mp_get_planner_buffers_available() == PLANNER_BUFFER_POOL_SIZE) &&
				 !st_runtime_isbusy() && (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_LOADER);
#ifdef __AVR
	SREG = saved_sreg;
#endif
	return (quiet);
}

/*
 * rt_heartbeat_callback() - send {"hb":{...}} after hb seconds of quiet (controller loop)
 *
 *	Quiet: the machine as above, JSON mode, and the USB link idle both ways (no unread or partial input,
 *	nothing waiting to go out). Any activity restarts the interval. A beat that cannot go out whole at once
 *	is skipped, and skipped beats are never made up.
 */
stat_t rt_heartbeat_callback(void)
{
	uint32_t now = SysTickTimer_getValue();
	if ((rt.hb == 0) || (cs.comm_mode != JSON_MODE) || !xio_usb_is_quiet() || !_machine_quiet()) {
		rt.hb_quiet_since = now;
		return (STAT_OK);
	}
	if ((now - rt.hb_quiet_since) < ((uint32_t)rt.hb * RT_TICKS_PER_SECOND)) {
		return (STAT_OK);
	}
	rt.hb_quiet_since = now;

	char qt[RT_QT_LEN+3] = "null";			// the token, quoted, or null
#ifdef __AVR
	uint8_t saved_sreg = SREG;
	cli();
#endif
	if (rt.qt_valid) {
		qt[0] = '"';
		memcpy(&qt[1], rt.qt, RT_QT_LEN);
		qt[RT_QT_LEN+1] = '"';
		qt[RT_QT_LEN+2] = NUL;
	}
#ifdef __AVR
	SREG = saved_sreg;
#endif
	char frame[96];
	snprintf_P(frame, sizeof(frame), PSTR("{\"hb\":{\"stat\":%d,\"hold\":%d,\"up\":%lu,\"qt\":%s}}\n"),
			   (int)cm_get_combined_state(), (int)cm.hold_state, (unsigned long)(now / RT_TICKS_PER_SECOND), qt);
	xio_usb_try_send_line(frame);
	return (STAT_OK);
}

/*
 * rt getters and setters - only for a pure {"rt":...} envelope (json_parser sets rt.allowed):
 * no text-mode, flat or status-report access
 */
stat_t rt_get_hb(nvObj_t *nv)
{
	if (!rt.allowed) { return (STAT_UNRECOGNIZED_NAME); }
	nv->value_int = rt.hb;
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t rt_set_hb(nvObj_t *nv)
{
	if (!rt.allowed) { return (STAT_UNRECOGNIZED_NAME); }
	if ((nv->valuetype != TYPE_INTEGER) || (nv->value_int > RT_HB_MAX)) {	// whole seconds only (typesafe marshalling)
		return (STAT_INPUT_VALUE_RANGE_ERROR);
	}
	rt.hb = (uint8_t)nv->value_int;
	rt.hb_quiet_since = SysTickTimer_getValue();
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t rt_get_qt(nvObj_t *nv)
{
	if (!rt.allowed) { return (STAT_UNRECOGNIZED_NAME); }
	char qt[RT_QT_LEN+1];
#ifdef __AVR
	uint8_t saved_sreg = SREG;
	cli();
#endif
	bool valid = rt.qt_valid;
	memcpy(qt, rt.qt, sizeof(qt));
#ifdef __AVR
	SREG = saved_sreg;
#endif
	if (!valid) {
		nv->valuetype = TYPE_NULL;
		return (STAT_OK);
	}
	nv->valuetype = TYPE_STRING;
	return (nv_copy_string(nv, qt));
}

static bool _is_token(const char *s)
{
	uint8_t n = 0;
	for (; s[n] != NUL; n++) {
		char c = s[n];
		if (!(((c >= '0') && (c <= '9')) || ((c >= 'a') && (c <= 'f')) || ((c >= 'A') && (c <= 'F')))) {
			return (false);
		}
	}
	return (n == RT_QT_LEN);
}

stat_t rt_set_qt(nvObj_t *nv)
{
	if (!rt.allowed) { return (STAT_UNRECOGNIZED_NAME); }
	if ((nv->valuetype != TYPE_STRING) || !_is_token(nv->str)) { return (STAT_INPUT_VALUE_RANGE_ERROR); }
	// Only at quiescence: otherwise a token could be installed over a held or queued job that '~' then runs.
	// Data lines the controller has not run yet (line mode serves control lines first) count as queued.
	if (!_machine_quiet() || xio_rx_data_pending()) { return (STAT_COMMAND_NOT_ACCEPTED); }
#ifdef __AVR
	uint8_t saved_sreg = SREG;
	cli();
#endif
	memcpy(rt.qt, nv->str, RT_QT_LEN);
	rt.qt[RT_QT_LEN] = NUL;
	rt.qt_valid = true;
#ifdef __AVR
	SREG = saved_sreg;
#endif
	return (STAT_OK);
}

stat_t rt_get_up(nvObj_t *nv)
{
	if (!rt.allowed) { return (STAT_UNRECOGNIZED_NAME); }
	nv->value_int = SysTickTimer_getValue() / RT_TICKS_PER_SECOND;	// wraps with the tick counter (~48 days)
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}
