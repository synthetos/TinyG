/*
 * runtime.c - runtime session family "rt": idle heartbeat and quiescence token (see runtime.h)
 */
#include "tinyg.h"
#include "config.h"
#include "controller.h"
#include "canonical_machine.h"
#include "planner.h"
#include "plan_arc.h"
#include "stepper.h"
#include "runtime.h"
#include "override.h"
#include "report.h"
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
    ov_init();
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
 * rt_get_job(), rt_set_job(), rt_job_end() - the host's job flag (feed-limit advisor step 3)
 *
 *	{"rt":{"job":1}} while a host streams a job and has more motion to send, {"rt":{"job":0}} after its last
 *	motion line and before its own deliberate stops, so the normal stop at the end is not a supply gap. RAM
 *	only, written any time (no quiescence: rt control lines can overtake buffered data, which errs towards
 *	fewer gaps). Reset (rt_init), a % flush, a job kill and program end clear it; a feedhold does not.
 */
stat_t rt_get_job(nvObj_t *nv)
{
	if (!rt.allowed) { return (STAT_UNRECOGNIZED_NAME); }
	nv->value_int = rt.job;
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t rt_set_job(nvObj_t *nv)
{
	if (!rt.allowed) { return (STAT_UNRECOGNIZED_NAME); }
	if ((nv->valuetype != TYPE_INTEGER) || (nv->value_int > 1)) { return (STAT_INPUT_VALUE_RANGE_ERROR); }
	if (rt.check_only) { return (STAT_OK); }
	uint8_t job = (uint8_t)nv->value_int;
	if (job != rt.job) {							// a job starts or ends: its motion evidence starts clean
		rt.job = 0;								// first, so the exec and loader stop adding evidence
		fl_job_moved = false;
		fl_underrun = false;
		rt.job = job;
	}
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

/*
 * rt_get_fg(), rt_set_fg() - {"rt":{"fg":1}}: follow every fs with an fg frame (gap and boundary counters,
 *	step 3). Opt-in because a host that does not know fg would read it as a damaged line. RAM only, written any
 *	time; reset clears it, a new fi window does not.
 */
stat_t rt_get_fg(nvObj_t *nv)
{
	if (!rt.allowed) { return (STAT_UNRECOGNIZED_NAME); }
	nv->value_int = rt.fg;
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

stat_t rt_set_fg(nvObj_t *nv)
{
	if (!rt.allowed) { return (STAT_UNRECOGNIZED_NAME); }
	if ((nv->valuetype != TYPE_INTEGER) || (nv->value_int > 1)) { return (STAT_INPUT_VALUE_RANGE_ERROR); }
	if (rt.check_only) { return (STAT_OK); }
	rt.fg = (uint8_t)nv->value_int;
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

void rt_job_end(void)
{
	rt.job = 0;								// one byte each: safe from interrupts and the main loop alike;
	fl_job_moved = false;					// the job's motion evidence ends with it, not at the gap callback's
	fl_underrun = false;					// next look (a new job could start before that)
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

/*
 * Feed-limit push (step 2): {rt:{fi:N}} turns accounting on and opens a window; while a cycle runs the board
 * sends {"fs":{...}} every N seconds and one final fs once the runtime has drained after the cycle, and
 * {"fl":{...}} when the executing move's kind or tags change (at most 4 a second). Like the heartbeat these
 * are not responses: no r, no footer. The loader credits finished segments into fl_pend (feedlimit.h); the
 * totals here take them every ~250 ms; a summary is formatted only when the TX ring is empty, so a busy link
 * costs a flag test. The totals wrap at 2^32 ms (49.7 days of cutting) and 2^32 mm.
 * See .analysis/feed-advisor-design.md.
 */
#define FL_FS_LEN		176						// fs v2: the largest is 165 bytes (166 on the wire)
#define FL_FG_LEN		176						// fg: the largest is 157 bytes (158 on the wire)
#define FL_EVENT_TICKS	(RT_TICKS_PER_SECOND / 4)	// at least this long between two fl events
#define FL_POLL_TICKS	20						// ~20 ms between looks for a changed state
#define FL_RETRY_TICKS	10						// ~10 ms before a frame that did not fit is tried again
#define FL_TICKS_PER_MS	50						// DDA ticks per ms (FREQUENCY_DDA 50 kHz)
#define FL_GAP_PENDING	1						// supply gap classes: input is on the board, not yet run
#define FL_GAP_NONE		2						// no known complete input: the host did not send in time
#define FL_SUPPLY_OK	0						// _fl_supply(): motion or work is queued
#define FL_SUPPLY_PAUSED 3						// a hold or M0/M1: an open gap stops accruing
#define FL_SUPPLY_OFF	4						// no job, accounting off, alarm, shutdown, flush, kill or program end
#define FL_FOLD_TICKS	(RT_TICKS_PER_SECOND / 4)	// pending credits are folded every ~250 ms, link or not; if the
												// main loop is held up long enough for them to wrap (~23.8 h of
												// cutting) the loader flags it and summaries carry "x":1

static struct flAccounting {
	uint8_t fi;									// summary interval, seconds; 0 = off
	uint16_t epoch;								// windows opened since reset (fs "e")
	uint32_t q;									// summaries sent in this window (fs "q")
	uint32_t ms[FL_CAUSES + 1];					// totals, ms: [0..6] causes, [FL_CAUSES] cutting
	uint8_t rem[FL_CAUSES + 1];					// DDA ticks not yet a whole ms
	uint32_t cls_ms[FL_CLASSES];				// below-F ms of command-tagged moves, by class (fs "b", step 3)
	uint8_t cls_rem[FL_CLASSES];
	uint32_t breaks[FL_CLASSES];				// motion breaks at command boundaries while a job ran, by class
	uint32_t mm;								// cutting distance: whole mm
	uint16_t um;								// and micrometres
	uint32_t next_fs, next_try, next_fl, next_fold;	// SysTick times
	bool cycle;									// a cycle was running at the last look
	bool fs_auto;								// a periodic or final summary is due (fi 0 cancels it)
	bool fs_ask;								// a summary was asked for ({rt:{fs:null}}; survives fi 0)
	bool invalid;								// a pending total wrapped: these totals are short (fs "x":1)
	bool fl_known;								// last_k, last_m and last_g are the state last sent
	uint8_t last_k, last_m, last_g;
	uint32_t gaps;								// supply gaps in this window (step 3)
	uint32_t gap_ticks[2];						// their SysTick time by class: [0] input pending, [1] no input
	uint32_t gap_t;								// SysTick time the accruing interval began
	bool gap_open;								// a gap is open (it survives a hold or M0/M1)
	uint8_t gap_cls;							// class accruing now (FL_GAP_*), 0 while not accruing
	bool fg_due;								// the fs just sent owes its fg (opted in); the next fs waits for it
	uint32_t fg_q;								// that fs's q
	uint32_t fg_g[3];							// its payload, frozen with the fs: g (ms pending, ms none, gaps)
	uint32_t fg_b[2 * FL_CLASSES];				// and b ([n, ms] per class)
	bool fg_x;
	bool fg_invalid;							// a gap or boundary counter wrapped (fg "x":1)
} fla;

static void _fl_fold(void)						// take the loader's pending credits into the totals
{
#ifdef __AVR
	uint8_t saved_sreg = SREG;
	cli();
#endif
	uint8_t bank = fl_bank;						// the loader moves to the other bank; this one is ours
	fl_bank = bank ^ 1;
#ifdef __AVR
	SREG = saved_sreg;
#endif
	flPending_t p;
	memcpy(&p, (const void *)&fl_pend[bank], sizeof(p));
	memset((void *)&fl_pend[bank], 0, sizeof(p));
	if (p.wrapped) { fla.invalid = true; fla.fg_invalid = true; }
	for (uint8_t k = 0; k <= FL_CAUSES; k++) {	// whole parts and remainders apart: no 32-bit sum can overflow
		fla.ms[k] += p.ticks[k] / FL_TICKS_PER_MS;
		fla.rem[k] += p.ticks[k] % FL_TICKS_PER_MS;
		if (fla.rem[k] >= FL_TICKS_PER_MS) { fla.rem[k] -= FL_TICKS_PER_MS; fla.ms[k]++; }
	}
	for (uint8_t c = 0; c < FL_CLASSES; c++) {	// command boundaries by class (step 3)
		uint32_t ms = fla.cls_ms[c], n = fla.breaks[c];
		fla.cls_ms[c] += p.cls_ticks[c] / FL_TICKS_PER_MS;
		fla.cls_rem[c] += p.cls_ticks[c] % FL_TICKS_PER_MS;
		if (fla.cls_rem[c] >= FL_TICKS_PER_MS) { fla.cls_rem[c] -= FL_TICKS_PER_MS; fla.cls_ms[c]++; }
		fla.breaks[c] += p.breaks[c];
		if ((fla.cls_ms[c] < ms) || (fla.breaks[c] < n)) { fla.fg_invalid = true; }	// fg's own wrap flag
	}
	fla.mm += p.um / 1000;
	fla.um += p.um % 1000;
	if (fla.um >= 1000) { fla.um -= 1000; fla.mm++; }
}


static uint32_t _fl_tick_ms(uint32_t t)		// SysTick (1/1024 s) to ms, no 32-bit overflow
{
	return ((t >> 10) * 1000UL + (((t & 1023UL) * 1000UL) >> 10));
}

/*
 * _fl_fg_take() - freeze the fg payload with the fs just sent, so the pair describes one moment however long the
 *	link keeps fg waiting. An open gap's running interval is included; a sum that wraps marks x for good.
 */
static void _fl_fg_take(uint32_t now)
{
	for (uint8_t k = 0; k < 2; k++) {
		uint32_t t = fla.gap_ticks[k];
		if (fla.gap_cls == k + 1) {
			uint32_t sum = t + (now - fla.gap_t);
			if (sum < t) { fla.fg_invalid = true; }
			t = sum;
		}
		fla.fg_g[k] = _fl_tick_ms(t);
	}
	fla.fg_g[2] = fla.gaps;
	for (uint8_t c = 0; c < FL_CLASSES; c++) {
		fla.fg_b[2 * c] = fla.breaks[c];
		fla.fg_b[2 * c + 1] = fla.cls_ms[c];
	}
	fla.fg_x = fla.fg_invalid;
}

static bool _fl_send_fs(void)					// fold, format and send a summary; false if the link was busy
{
	if (!xio_usb_tx_ready()) { return (false); }
	_fl_fold();
	char frame[FL_FS_LEN];
	snprintf_P(frame, sizeof(frame),
			   PSTR("{\"fs\":{\"v\":2,\"e\":%u,\"q\":%lu,\"c\":[%lu,%lu,%lu,%lu,%lu,%lu,%lu],\"ct\":%lu,\"cd\":%lu.%03u%s}}\n"),
			   (unsigned)fla.epoch, (unsigned long)(fla.q + 1),
			   (unsigned long)fla.ms[0], (unsigned long)fla.ms[1], (unsigned long)fla.ms[2],
			   (unsigned long)fla.ms[3], (unsigned long)fla.ms[4], (unsigned long)fla.ms[5], (unsigned long)fla.ms[6],
			   (unsigned long)fla.ms[FL_CAUSES], (unsigned long)fla.mm, (unsigned)fla.um, fla.invalid ? ",\"x\":1" : "");
	if (!xio_usb_try_send_frame(frame)) { return (false); }
	fla.q++;
	if (rt.fg) {								// opted in: its fg follows before any other fs
		fla.fg_due = true;
		fla.fg_q = fla.q;
		_fl_fg_take(SysTickTimer_getValue());
	}
	return (true);
}

/*
 * _fl_send_fg() - the gap and boundary counters for the fs just sent (same e and q); false if the link was busy
 *
 *	{"fg":{"e":E,"q":Q,"g":[ms_pending,ms_none,gaps],"b":[[n,ms],[n,ms],[n,ms]],"x":1?}}. g: supply gap time
 *	with input pending and with no known input (an open gap's running interval included) and the gap count; b per
 *	class (spindle/S, coolant, other): motion breaks at command boundaries and below-F ms of the moves tagged by them.
 *	x: one of these counters wrapped (fs has its own x). The payload was frozen with the fs (_fl_fg_take).
 */
static bool _fl_send_fg(void)
{
	if (!xio_usb_tx_ready()) { return (false); }
	char frame[FL_FG_LEN];
	snprintf_P(frame, sizeof(frame),
			   PSTR("{\"fg\":{\"e\":%u,\"q\":%lu,\"g\":[%lu,%lu,%lu],\"b\":[[%lu,%lu],[%lu,%lu],[%lu,%lu]]%s}}\n"),
			   (unsigned)fla.epoch, (unsigned long)fla.fg_q,
			   (unsigned long)fla.fg_g[0], (unsigned long)fla.fg_g[1], (unsigned long)fla.fg_g[2],
			   (unsigned long)fla.fg_b[0], (unsigned long)fla.fg_b[1], (unsigned long)fla.fg_b[2],
			   (unsigned long)fla.fg_b[3], (unsigned long)fla.fg_b[4], (unsigned long)fla.fg_b[5],
			   fla.fg_x ? ",\"x\":1" : "");
	return (xio_usb_try_send_frame(frame));
}

/*
 * _fl_finished() - motion ran since the last final summary and has now played out and been credited
 *
 *	The exec latches fl_activity when it starts a block, so a job that starts and ends between two looks
 *	is not missed; the latch is cleared in the same snapshot that finds the runtime drained.
 */
static bool _fl_finished(void)
{
#ifdef __AVR
	uint8_t saved_sreg = SREG;
	cli();
#endif
	bool finished = fl_activity && !st_runtime_isbusy() && (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_LOADER) &&
					!st_fl_record_pending();
	if (finished) { fl_activity = false; }
#ifdef __AVR
	SREG = saved_sreg;
#endif
	return (finished);
}

/*
 * Supply gaps (step 3). With accounting on and a host job ({rt:{job:1}}), a stop after motion began this job, in
 * which the runtime has drained (nothing stepping or dwelling, nothing prepared or runnable), outside a hold,
 * M0/M1, alarm, shutdown, flush, kill or program end, is a supply gap. Class FL_GAP_PENDING: input is on the
 * board but not yet run (raw RX, a buffered data line, an arc being generated); FL_GAP_NONE: no complete input
 * is known, so the host did not send in time. A class change keeps the gap; a hold or M0/M1 stops the accrual and
 * keeps it open; alarm, shutdown, flush, kill, program end and job end close it and forget the motion. The exec
 * latches fl_underrun when it found nothing to run during a job, so a drain and refill between two looks still
 * counts once. Times are SysTick (1/1024 s, steps of 10): entry and exit are seen up to one main-loop pass late.
 */
static uint8_t _fl_supply(void)
{
	if (!fl_on || !rt.job) { return (FL_SUPPLY_OFF); }
	if ((cm.machine_state == MACHINE_ALARM) || (cm.machine_state == MACHINE_SHUTDOWN) ||
		(cm.machine_state == MACHINE_PANIC) || (cm.machine_state == MACHINE_PROGRAM_END) ||
		(cm.queue_flush_state != FLUSH_OFF) || cm.job_kill_requested) {
		return (FL_SUPPLY_OFF);
	}
	if ((cm.hold_state != FEEDHOLD_OFF) || cm.program_paused) { return (FL_SUPPLY_PAUSED); }
#ifdef __AVR
	uint8_t saved_sreg = SREG;
	cli();
#endif
	bool drained = !st_runtime_isbusy() && (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_LOADER) &&
				   !mp_has_runnable_buffer() && !st_aline_in_flight();
#ifdef __AVR
	SREG = saved_sreg;
#endif
	if (!drained) { return (FL_SUPPLY_OK); }
	if (xio_rx_data_pending() || xio_usb_rx_unread() || (arc.run_state != BLOCK_IDLE)) { return (FL_GAP_PENDING); }
	return (FL_GAP_NONE);
}

static void _fl_gap_close(uint32_t now)		// end the accruing interval (the gap itself may stay open)
{
	if (fla.gap_cls != 0) {
		uint32_t was = fla.gap_ticks[fla.gap_cls - 1];
		fla.gap_ticks[fla.gap_cls - 1] += now - fla.gap_t;
		if (fla.gap_ticks[fla.gap_cls - 1] < was) { fla.fg_invalid = true; }
		fla.gap_cls = 0;
	}
}

static void _fl_gap(uint32_t now)
{
#ifdef __AVR
	uint8_t saved_sreg = SREG;
	cli();
#endif
	bool under = fl_underrun;
	fl_underrun = false;
#ifdef __AVR
	SREG = saved_sreg;
#endif
	uint8_t s = _fl_supply();
	if (s == FL_SUPPLY_OFF) {					// the gap ends; the next one needs motion first
		_fl_gap_close(now);
		fla.gap_open = false;
		fl_job_moved = false;
		return;
	}
	if (!fl_job_moved) { return; }				// waiting for the first move is not a gap
	if (s == FL_SUPPLY_PAUSED) {				// the operator's time, not the host's
		_fl_gap_close(now);
		return;
	}
	if (s == FL_SUPPLY_OK) {
		if (!fla.gap_open && under && (++fla.gaps == 0)) { fla.fg_invalid = true; }	// ran dry and refilled between looks
		_fl_gap_close(now);
		fla.gap_open = false;
		return;
	}
	if (!fla.gap_open) {
		fla.gap_open = true;
		if (++fla.gaps == 0) { fla.fg_invalid = true; }
	}
	if (fla.gap_cls != s) {						// a new interval in the other class, the same gap
		_fl_gap_close(now);
		fla.gap_cls = s;
		fla.gap_t = now;
	}
}

/*
 * _fl_event() - one fl line when the executing move's kind or tags change, or a supply gap opens, changes
 *	class or ends (while starved: "g" the class and "n" the last line run)
 *
 *	The look takes two bytes of the current slot (kind and the exec's all_tags) with interrupts masked; only
 *	on a change are the percent and line inputs taken, again together with kind and tags, so the line
 *	describes one move.
 */
static bool _fl_look(uint8_t *k, uint8_t *m, flBlock_t *b)	// b: also take the line's inputs (on a change)
{
#ifdef __AVR
	uint8_t saved_sreg = SREG;
	cli();
#endif
	uint8_t cur = fl_cur;
	bool running = (cur < 2) && (cm.motion_state == MOTION_RUN) && (cm.hold_state == FEEDHOLD_OFF);
	*k = running ? fl_slot[cur].kind : FL_KIND_NONE;
	*m = running ? fl_slot[cur].all_tags : 0;
	if (running && (b != NULL)) {
		b->pct_ok = fl_slot[cur].pct_ok;
		b->cruise = fl_slot[cur].cruise;
		b->feed = fl_slot[cur].feed;
		b->line = fl_slot[cur].line;
	}
#ifdef __AVR
	SREG = saved_sreg;
#endif
	return (running);
}

static void _fl_event(uint32_t now)
{
	uint8_t k, m;
	_fl_look(&k, &m, NULL);
	uint8_t g = fla.gap_open ? fla.gap_cls : 0;
	if (fla.fl_known && (k == fla.last_k) && (m == fla.last_m) && (g == fla.last_g)) {
		fla.next_fl = now + FL_POLL_TICKS;
		return;
	}
	if (!xio_usb_tx_ready()) {					// tried again with the state current then (coalesced)
		fla.next_fl = now + FL_RETRY_TICKS;
		return;
	}
	flBlock_t b;
	bool running = _fl_look(&k, &m, &b);
	char p[12] = "null", n[12] = "null";
	if (running && (k == FL_KIND_FEED) && b.pct_ok && (b.feed > 0)) {
		snprintf_P(p, sizeof(p), PSTR("%lu"), (unsigned long)((100 * b.cruise / b.feed) + 0.5));
	}
	if (running) {
		snprintf_P(n, sizeof(n), PSTR("%lu"), (unsigned long)b.line);
	} else if (g != 0) {
		snprintf_P(n, sizeof(n), PSTR("%lu"), (unsigned long)mr.gm.linenum);	// the last line run
	}
	char gs[12] = "";								// ,"g":N with any uint8_t N (g is 1 or 2 today)
	if (g != 0) { snprintf_P(gs, sizeof(gs), PSTR(",\"g\":%u"), (unsigned)g); }
	char frame[72];								// the longest fl, with g, is 61 bytes
	snprintf_P(frame, sizeof(frame), PSTR("{\"fl\":{\"k\":%u,\"m\":%u,\"p\":%s,\"n\":%s%s}}\n"), (unsigned)k, (unsigned)m, p, n, gs);
	if (xio_usb_try_send_frame(frame)) {
		fla.last_k = k;
		fla.last_m = m;
		fla.last_g = g;
		fla.fl_known = true;
		fla.next_fl = now + FL_EVENT_TICKS;
	} else {
		fla.next_fl = now + FL_RETRY_TICKS;		// tried again with the state current then (coalesced)
	}
}

stat_t rt_feedlimit_callback(void)
{
	if ((fla.fi == 0) && !fla.fs_ask && !fla.fg_due) { return (STAT_OK); }
	uint32_t now = SysTickTimer_getValue();
	if (fla.fi != 0) {
		bool cycle = (cm.cycle_state != CYCLE_OFF);
		if (cycle && !fla.cycle) {				// a cycle starts: first summary fi seconds in
			fla.next_fs = now + (uint32_t)fla.fi * RT_TICKS_PER_SECOND;
		}
		fla.cycle = cycle;
		if (cycle && ((int32_t)(now - fla.next_fs) >= 0)) {
			fla.fs_auto = true;
			fla.next_try = now;
			fla.next_fs = now + (uint32_t)fla.fi * RT_TICKS_PER_SECOND;
		}
		if (!cycle && _fl_finished()) {			// the final summary
			fla.fs_auto = true;
			fla.next_try = now;
		}
		if ((int32_t)(now - fla.next_fold) >= 0) {
			_fl_fold();
			fla.next_fold = now + FL_FOLD_TICKS;
		}
		_fl_gap(now);
		if ((int32_t)(now - fla.next_fl) >= 0) { _fl_event(now); }
	}
	if (fla.fg_due && !rt.fg) { fla.fg_due = false; }	// the host opted out: nothing owed
	if (fla.fg_due) {							// the fs's fg first: pairs never interleave
		if ((int32_t)(now - fla.next_try) >= 0) {
			if (_fl_send_fg()) {
				fla.fg_due = false;
			}
			fla.next_try = now + FL_RETRY_TICKS;
		}
	} else if ((fla.fs_auto || fla.fs_ask) && ((int32_t)(now - fla.next_try) >= 0)) {
		if (_fl_send_fs()) {
			fla.fs_auto = false;
			fla.fs_ask = false;
		} else {
			fla.next_try = now + FL_RETRY_TICKS;
		}
	}
	return (STAT_OK);
}

stat_t rt_get_fi(nvObj_t *nv)
{
	if (!rt.allowed) { return (STAT_UNRECOGNIZED_NAME); }
	nv->value_int = fla.fi;
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

/*
 * rt_set_fi() - 0 stops crediting (the totals stay for {rt:{fs:null}}); 1..60 sets the summary interval
 *	(longer is clamped and the echo shows the interval applied). Off to on opens a new window with zeroed
 *	totals, and is accepted only at quiescence, checked and committed in one critical section as for qt.
 */
stat_t rt_set_fi(nvObj_t *nv)
{
	if (!rt.allowed) { return (STAT_UNRECOGNIZED_NAME); }
	if (nv->valuetype != TYPE_INTEGER) { return (STAT_INPUT_VALUE_RANGE_ERROR); }
	uint8_t fi = (nv->value_int > RT_FI_MAX) ? RT_FI_MAX : (uint8_t)nv->value_int;
	nv->value_int = fi;
	if ((fi == 0) || (fla.fi != 0)) {			// off, or only a new interval
		if (!rt.check_only) {
			if (fi == 0) {							// queued automatic output is cancelled; an open gap ends here
				fl_on = false;
				fla.fs_auto = false;
				_fl_gap_close(SysTickTimer_getValue());
				fla.gap_open = false;
				fl_job_moved = false;
				fl_underrun = false;				// after fl_on: the loader latches no more
			}
			fla.fi = fi;
		}
		return (STAT_OK);
	}
	if (!rt.check_only) {						// off, so the loader is not crediting: one fold empties both
		_fl_fold();								// banks into the old window's totals, which a rejected
	}											// enable leaves intact
#ifdef __AVR
	uint8_t saved_sreg = SREG;
	cli();
#endif
	stat_t status = STAT_OK;
	if (!_machine_quiet() || xio_rx_data_pending()) {
		status = STAT_COMMAND_NOT_ACCEPTED;
	} else if (!rt.check_only) {
		fl_um_q16 = 0;
		fl_activity = false;
		fl_job_moved = false;
		fl_underrun = false;
		fl_on = true;
	}
#ifdef __AVR
	SREG = saved_sreg;
#endif
	if ((status == STAT_OK) && !rt.check_only) {
		uint16_t epoch = fla.epoch + 1;
		uint32_t now = SysTickTimer_getValue();
		memset(&fla, 0, sizeof(fla));
		fla.epoch = epoch;
		fla.fi = fi;
		// deadlines from now, not 0: the signed comparisons would read 0 as future in half the tick range
		fla.next_fs = fla.next_try = fla.next_fl = fla.next_fold = now;
	}
	return (status);
}

/*
 * rt_get_fs() - push the window's totals as an fs line (after this reply); the value is the q it will carry.
 *	Null before any window was opened. {rt:null} reads it too, and so pushes a summary.
 */
stat_t rt_get_fs(nvObj_t *nv)
{
	if (!rt.allowed) { return (STAT_UNRECOGNIZED_NAME); }
	if (fla.epoch == 0) {
		nv->valuetype = TYPE_NULL;
		return (STAT_OK);
	}
	fla.fs_ask = true;
	fla.next_try = SysTickTimer_getValue();
	nv->value_int = fla.q + 1;
	nv->valuetype = TYPE_INTEGER;
	return (STAT_OK);
}

/* Live requests and the factor accepted by the exec handoff. */
#ifdef __AVR
#pragma GCC push_options
#pragma GCC optimize ("Os")
#endif
static stat_t _ov_result(uint8_t result)
{
    return result==OV_OK ? STAT_OK : result==OV_RANGE ? STAT_INPUT_VALUE_RANGE_ERROR : STAT_COMMAND_NOT_ACCEPTED;
}
// Share the float conversion across the four main-loop getters.
static stat_t _ov_float(nvObj_t *nv,uint16_t q) __attribute__((noinline));
static stat_t _ov_float(nvObj_t *nv,uint16_t q)
{
    nv->value_flt=q/1000.0f;nv->valuetype=TYPE_FLOAT;nv->precision=3;return STAT_OK;
}
stat_t ov_get_mfo(nvObj_t *nv){ovSnapshot_t s;ov_snapshot(&s);return _ov_float(nv,s.feed);}
stat_t ov_get_sso(nvObj_t *nv){ovSnapshot_t s;ov_snapshot(&s);return _ov_float(nv,s.spindle);}
stat_t ov_get_mfc(nvObj_t *nv){ovSnapshot_t s;ov_snapshot(&s);return _ov_float(nv,s.committed_feed);}
stat_t ov_get_ssc(nvObj_t *nv){ovSnapshot_t s;ov_snapshot(&s);return _ov_float(nv,s.committed_spindle);}
stat_t ov_get_state(nvObj_t *nv){ovSnapshot_t s;ov_snapshot(&s);nv->value_int=s.state;nv->valuetype=TYPE_INTEGER;return STAT_OK;}
static stat_t _ov_factor(nvObj_t *nv,uint8_t which)
{
    if(!rt.allowed)return STAT_UNRECOGNIZED_NAME;
    stat_t status=cm_is_alarmed();if(status!=STAT_OK)return status;
    if(nv->valuetype!=TYPE_FLOAT && nv->valuetype!=TYPE_INTEGER)return STAT_INPUT_VALUE_RANGE_ERROR;
    float value=nv->valuetype==TYPE_FLOAT?nv->value_flt:(float)nv->value_int;
    nv->precision=3; // Keep rejected numeric requests readable in the response.
    status=_ov_result(ov_set_factor(which,value,!rt.check_only,true));
    if(status==STAT_OK && !rt.check_only){if(which==OV_FEED)ov_get_mfo(nv);else ov_get_sso(nv);}
    return status;
}
stat_t rt_set_mfo(nvObj_t *nv){return _ov_factor(nv,OV_FEED);}
stat_t rt_set_sso(nvObj_t *nv){return _ov_factor(nv,OV_SPINDLE);}
static stat_t _ov_enable(nvObj_t *nv,uint8_t which)
{
    if(!rt.allowed)return STAT_UNRECOGNIZED_NAME;
    stat_t status=cm_is_alarmed();if(status!=STAT_OK)return status;
    if(nv->valuetype!=TYPE_INTEGER || nv->value_int>1)return STAT_INPUT_VALUE_RANGE_ERROR;
    return _ov_result(ov_set_enable(which,nv->value_int,!rt.check_only,true));
}
stat_t rt_set_mfe(nvObj_t *nv){return _ov_enable(nv,OV_FEED);}
stat_t rt_set_sse(nvObj_t *nv){return _ov_enable(nv,OV_SPINDLE);}
stat_t rt_get_mfe(nvObj_t *nv){ovSnapshot_t s;ov_snapshot(&s);nv->value_int=((s.enables&OV_FEED) && s.program_feed);nv->valuetype=TYPE_INTEGER;return STAT_OK;}
stat_t rt_get_sse(nvObj_t *nv){ovSnapshot_t s;ov_snapshot(&s);nv->value_int=!!(s.enables&OV_SPINDLE);nv->valuetype=TYPE_INTEGER;return STAT_OK;}
stat_t rt_get_ovb(nvObj_t *nv){ovSnapshot_t s;ov_snapshot(&s);nv->value_int=s.armed;nv->valuetype=TYPE_INTEGER;return STAT_OK;}
stat_t rt_set_ovb(nvObj_t *nv)
{
    if(!rt.allowed)return STAT_UNRECOGNIZED_NAME;
    stat_t status=cm_is_alarmed();if(status!=STAT_OK)return status;
    if(nv->valuetype!=TYPE_INTEGER || nv->value_int>1)return STAT_INPUT_VALUE_RANGE_ERROR;
#ifdef __AVR
    uint8_t saved_sreg=SREG;cli();
#endif
    bool quiet=_machine_quiet() && !xio_rx_data_pending() && xio_usb_input_boundary();
    status=_ov_result(ov_arm(nv->value_int,quiet,true,!rt.check_only));
#ifdef __AVR
    SREG=saved_sreg;
#endif
    return status;
}

stat_t ov_get_capped(nvObj_t *nv)
{
    ovSnapshot_t s;ov_snapshot(&s);
    nv->value_int=s.capped;nv->valuetype=TYPE_BOOLEAN;return STAT_OK;
}
stat_t rt_get_ov(nvObj_t *nv)
{
    if(!rt.allowed)return STAT_UNRECOGNIZED_NAME;
    ovSnapshot_t s;ov_snapshot(&s);rt.ov_ask=true;
    nv->value_int=s.revision;nv->valuetype=TYPE_INTEGER;return STAT_OK;
}
stat_t rt_set_ov(nvObj_t *nv)
{
    if(!rt.allowed)return STAT_UNRECOGNIZED_NAME;
    if(nv->valuetype!=TYPE_INTEGER || nv->value_int>1)return STAT_INPUT_VALUE_RANGE_ERROR;
    if(!rt.check_only) {rt.ov_events=nv->value_int;rt.ov_revision=0;rt.ov_sent_at=SysTickTimer_getValue();}
    return STAT_OK;
}
stat_t rt_override_callback(void)
{
    if((!rt.ov_events && !rt.ov_ask) || cs.comm_mode!=JSON_MODE)return STAT_OK;
    uint32_t now=SysTickTimer_getValue();
    uint32_t ms=sr.status_report_interval;
    // Convert even the maximum interval without overflowing a 32-bit product.
    uint32_t ticks=(ms/125)*128+((ms%125)*128)/125;
    if(!rt.ov_ask && rt.ov_revision && (uint32_t)(now-rt.ov_sent_at)<ticks)return STAT_OK;
    ovSnapshot_t s;ov_snapshot(&s);
    if(!rt.ov_ask && s.revision==rt.ov_revision)return STAT_OK;
    char frame[112];
    snprintf_P(frame,sizeof(frame),PSTR("{\"ov\":{\"v\":1,\"q\":%lu,\"mfo\":%.3f,\"mfc\":%.3f,\"mfe\":%u,\"capped\":%s}}\n"),
        (unsigned long)s.revision,(double)(s.feed/1000.0f),(double)(s.committed_feed/1000.0f),
        (unsigned)((s.enables&OV_FEED) && s.program_feed),s.capped?"true":"false");
    if(xio_usb_try_send_frame(frame)) {
        rt.ov_revision=s.revision;rt.ov_ask=false;
        rt.ov_sent_at=now;
    }
    return STAT_OK;
}

#ifdef __AVR
#pragma GCC pop_options
#endif
