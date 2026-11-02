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
#include "feedlimit.h"

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
 *	Quiet: the machine as above, JSON mode, and the USB link idle both ways: no byte in or out since the
 *	last pass (realtime characters included), no unread or partial input, nothing waiting to go out. Any
 *	activity restarts the interval. A beat that cannot go out whole at once is skipped, and skipped beats
 *	are never made up.
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
	if (rt.check_only) { return (STAT_OK); }
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
	// Check and commit in one critical section, so an interrupt that invalidates (a job kill, a flush)
	// cannot fall between them and be overwritten.
#ifdef __AVR
	uint8_t saved_sreg = SREG;
	cli();
#endif
	stat_t status = STAT_OK;
	if (!_machine_quiet() || xio_rx_data_pending()) {
		status = STAT_COMMAND_NOT_ACCEPTED;
	} else if (!rt.check_only) {
		memcpy(rt.qt, nv->str, RT_QT_LEN);
		rt.qt[RT_QT_LEN] = NUL;
		rt.qt_valid = true;
	}
#ifdef __AVR
	SREG = saved_sreg;
#endif
	return (status);
}

stat_t rt_get_up(nvObj_t *nv)
{
	if (!rt.allowed) { return (STAT_UNRECOGNIZED_NAME); }
	nv->value_int = SysTickTimer_getValue() / RT_TICKS_PER_SECOND;	// wraps with the tick counter (~48 days)
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

/*
 * rt_set_grp() - {"rt":{...}}: check every leaf, then apply them; the first refusal is the reply
 *
 *	The ordinary group setter ignores its children's status. Here a refused leaf changes nothing, and
 *	null leaves are reads.
 */
stat_t rt_set_grp(nvObj_t *nv)
{
	if (!rt.allowed) { return (STAT_UNRECOGNIZED_NAME); }
	for (uint8_t pass = 0; pass < 2; pass++) {
		rt.check_only = (pass == 0);
		nvObj_t *child = nv;
		for (uint8_t i=0; i<NV_MAX_OBJECTS; i++) {
			if (((child = nv_next(child)) == NULL) || (child->valuetype == TYPE_EMPTY) || (child->depth <= nv->depth)) {
				break;
			}
			stat_t status = (child->valuetype == TYPE_NULL) ? ((pass == 0) ? STAT_OK : nv_get(child)) : nv_set(child);
			if (status != STAT_OK) {
				rt.check_only = false;
				return (status);
			}
		}
	}
	rt.check_only = false;
	return (STAT_OK);
}

/*
 * Feed-limit report of the move stepping now (feedlimit.h): flk kind (0 none, 1 feed, 2 rapid), flm the
 * constraint tags of its plan, flp its planned cruise as a percent of its F (G94 linear XYZ feed moves,
 * else null), flf its F in mm/min (G94 feed moves, else null). Nothing runs while stopped or held.
 * One snapshot per line, with the motion and hold state, so every leaf of a reply describes one move.
 */
static flBlock_t fl_snap;

static void _fl_current(flBlock_t *b)
{
	if (!rt.fl_taken) {
#ifdef __AVR
		uint8_t saved_sreg = SREG;					// the loader sets fl_cur and the exec fills the other
		cli();										// slot from their interrupts
#endif
		uint8_t cur = fl_cur;
		if (cur < 2) {
			memcpy(&fl_snap, &fl_slot[cur], sizeof(flBlock_t));
		}
		bool running = (cur < 2) && (cm.motion_state == MOTION_RUN) && (cm.hold_state == FEEDHOLD_OFF);
#ifdef __AVR
		SREG = saved_sreg;
#endif
		if (!running) {
			fl_snap.kind = FL_KIND_NONE;
		} else {									// profile tags from the plan, once per line
			fl_snap.tags |= fl_profile_tags(fl_snap.naiive_move_time, fl_snap.cruise, fl_snap.cruise_vmax);
		}
		rt.fl_taken = true;
	}
	*b = fl_snap;
}

stat_t rt_get_flk(nvObj_t *nv)
{
	if (!rt.allowed) { return (STAT_UNRECOGNIZED_NAME); }
	flBlock_t b;
	_fl_current(&b);
	nv->value_int = b.kind;
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t rt_get_flm(nvObj_t *nv)
{
	if (!rt.allowed) { return (STAT_UNRECOGNIZED_NAME); }
	flBlock_t b;
	_fl_current(&b);
	nv->value_int = (b.kind == FL_KIND_NONE) ? 0 : b.tags;
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t rt_get_flp(nvObj_t *nv)
{
	if (!rt.allowed) { return (STAT_UNRECOGNIZED_NAME); }
	flBlock_t b;
	_fl_current(&b);
	if ((b.kind != FL_KIND_FEED) || !b.pct_ok || !(b.feed > 0)) {
		nv->valuetype = TYPE_NULL;
		return (STAT_OK);
	}
	nv->value_int = (uint32_t)((100 * b.cruise / b.feed) + 0.5);	// not clamped: above 100 shows as is
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t rt_get_flf(nvObj_t *nv)
{
	if (!rt.allowed) { return (STAT_UNRECOGNIZED_NAME); }
	flBlock_t b;
	_fl_current(&b);
	if ((b.kind != FL_KIND_FEED) || !b.feed_ok) {
		nv->valuetype = TYPE_NULL;
		return (STAT_OK);
	}
	nv->value_flt = b.feed;
	nv->valuetype = TYPE_FLOAT;
	return (STAT_OK);
}
