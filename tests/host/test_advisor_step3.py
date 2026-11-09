"""Feed-limit advisor step 3 (runtime.c, json_parser.c, canonical_machine.c): real code, stubbed machine.

A. Host job flag: {"rt":{"job":1|0}} says whether a host is streaming a job with more motion to send; only then can
   a stop be a supply gap. RAM only, written any time (no quiescence), read with {"rt":{"job":null}}. Like every rt
   leaf it is refused outside a pure rt envelope. Reset, a % flush, a job kill and program end (M2/M30) clear it.
"""
import re
import unittest

import test_runtime_rt
from test_runtime_rt import PRELUDE, fn, function, src

_build_run = test_runtime_rt.RuntimeFamily.build_run     # the compile-and-run helper, not RuntimeFamily's tests


def job_code():
    h = src("runtime.h")
    defs = "\n".join(l for l in h.splitlines() if l.startswith("#define RT_"))
    struct = h[h.index("typedef struct rtSingleton"):h.index("extern rtSingleton_t rt;")]
    code = PRELUDE + defs + "\n" + struct + "\nrtSingleton_t rt;\nvolatile uint8_t fl_job_moved, fl_underrun;\n"
    for name in ("rt_get_job", "rt_set_job", "rt_job_end"):
        code += fn("runtime.c", name) + "\n"
    code += "#define RT_ENVELOPE_NONE 0\n#define RT_ENVELOPE_PURE 1\n#define RT_ENVELOPE_BAD 2\n"
    js = src("json_parser.c")
    code += function(js, r"static bool _is_key_char\(char c\)") + "\n"
    code += function(js, r"static uint8_t _scan_rt\(const char \*p\)") + "\n"
    return code


JOB_MAIN = r'''
static stat_t set_int(stat_t (*f)(nvObj_t *), uint32_t v) { nvObj_t nv = { .valuetype = TYPE_INTEGER, .value_int = v }; return f(&nv); }
static void job_kill(void) {}
int main(void) {
    static const struct { const char *s; int want; } env[] = {
        {"{\"rt\":{\"job\":1}}", RT_ENVELOPE_PURE}, {"{\"rt\":{\"job\":null}}", RT_ENVELOPE_PURE},
        {"{rt:{job:0}}", RT_ENVELOPE_PURE}, {"{\"rt\":{\"job\":1,\"fi\":2},\"tid\":7}", RT_ENVELOPE_PURE},
        {"{\"rt\":{\"job\":1,\"job\":0}}", RT_ENVELOPE_BAD}, {"{\"rt\":{\"job\":1},\"sv\":1}", RT_ENVELOPE_BAD},
    };
    for (unsigned i = 0; i < sizeof(env) / sizeof(env[0]); i++) {
        int got = _scan_rt(env[i].s);
        if (got != env[i].want) { fprintf(stderr, "_scan_rt(%s) = %d, want %d\n", env[i].s, got, env[i].want); return 1; }
    }
    memset(&rt, 0, sizeof(rt));
    assert(set_int(rt_set_job, 1) == STAT_UNRECOGNIZED_NAME && rt.job == 0);      /* not a pure rt envelope */
    { nvObj_t nv = { .valuetype = TYPE_INTEGER }; assert(rt_get_job(&nv) == STAT_UNRECOGNIZED_NAME); }
    rt.allowed = true;
    assert(set_int(rt_set_job, 2) == STAT_INPUT_VALUE_RANGE_ERROR && rt.job == 0);
    { nvObj_t nv = { .valuetype = TYPE_STRING, .str = "1" }; assert(rt_set_job(&nv) == STAT_INPUT_VALUE_RANGE_ERROR); }
    rt.check_only = true;
    assert(set_int(rt_set_job, 1) == STAT_OK && rt.job == 0);                    /* the group's check pass applies nothing */
    rt.check_only = false;
    assert(set_int(rt_set_job, 1) == STAT_OK && rt.job == 1);
    { nvObj_t nv = { .valuetype = TYPE_NULL }; assert(rt_get_job(&nv) == STAT_OK && nv.value_int == 1 && nv.valuetype == TYPE_INTEGER); }
    assert(set_int(rt_set_job, 1) == STAT_OK && rt.job == 1);                    /* idempotent */
    rt_job_end();
    assert(rt.job == 0);
    assert(set_int(rt_set_job, 1) == STAT_OK && set_int(rt_set_job, 0) == STAT_OK && rt.job == 0);
    /* job motion evidence ends with the job, at once (not when the gap callback next looks) */
    assert(set_int(rt_set_job, 1) == STAT_OK); fl_job_moved = 1; fl_underrun = 1;
    assert(set_int(rt_set_job, 1) == STAT_OK && fl_job_moved && fl_underrun);            /* re-sent mid-job: kept */
    rt_job_end(); assert(!fl_job_moved && !fl_underrun);                                 /* program end, %, kill */
    assert(set_int(rt_set_job, 1) == STAT_OK); fl_job_moved = 1; fl_underrun = 1;
    assert(set_int(rt_set_job, 0) == STAT_OK && !fl_job_moved && !fl_underrun);
    fl_job_moved = 1; fl_underrun = 1;                                                    /* stale from outside a job */
    assert(set_int(rt_set_job, 1) == STAT_OK && !fl_job_moved && !fl_underrun);          /* a new job starts clean */
    rt.check_only = true; fl_job_moved = 1; assert(set_int(rt_set_job, 0) == STAT_OK && fl_job_moved && rt.job == 1); rt.check_only = false;
    return 0;
}
'''


class AdvisorStep3Job(unittest.TestCase):
    build_run = _build_run

    def test_job_flag(self):
        self.build_run(job_code() + JOB_MAIN, "job")

    def test_job_flag_wiring(self):
        cfg = src("config_app.c")
        self.assertTrue(re.search(r'\{ "rt","rtjob",_i0, 0, tx_print_nul, rt_get_job, rt_set_job,', cfg), "no rtjob row in cfgArray")
        h = src("runtime.h")
        for proto in ("stat_t rt_get_job(nvObj_t *nv);", "stat_t rt_set_job(nvObj_t *nv);", "void rt_job_end(void);"):
            self.assertIn(proto, h)
        cm = src("canonical_machine.c")
        for name in (r"void cm_request_queue_flush\((?:void)?\)", r"void cm_request_job_kill\((?:void)?\)",
                     r"static void _exec_program_end\(float \*value, bool \*flags\)(?=\s*\{)"):
            body = function(cm, name)
            self.assertIn("rt_job_end();", body, name)
        # reset: rt_init clears the whole singleton, so the flag starts at 0 after every boot and ^X
        self.assertIn("memset(&rt, 0, sizeof(rt));", fn("runtime.c", "rt_init"))


if __name__ == "__main__":
    unittest.main()


# B. Command boundaries -----------------------------------------------------------------------------------------
B_PRELUDE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define __AVR
uint8_t SREG = 0x80;
static void cli(void) { SREG &= ~0x80; }
#define AXES 6
#define AXIS_X 0
#define PSTR(s) s
#define STAT_BUFFER_FULL_FATAL 251
typedef uint8_t stat_t;
enum { MP_BUFFER_EMPTY = 0, MP_BUFFER_LOADING, MP_BUFFER_QUEUED };
enum { BLOCK_TYPE_NULL = 0, BLOCK_TYPE_ALINE, BLOCK_TYPE_DWELL, BLOCK_TYPE_COMMAND };
enum { BLOCK_IDLE = 0, BLOCK_INITIALIZING, BLOCK_RUNNING };
typedef struct mpBuffer { struct mpBuffer *pv; stat_t (*bf_func)(struct mpBuffer *); void (*cm_func)(float[], bool[]);
    uint8_t buffer_state, block_type, block_state; float value_vector[AXES]; bool axis_flags[AXES];
    uint8_t fl_tags, fl_cls; } mpBuf_t;
'''


def b_code():
    h = src("feedlimit.h")
    code = B_PRELUDE + "\n".join(l for l in h.splitlines() if l.startswith("#define FL_")) + "\n"
    code += h[h.index("typedef struct flPending"):h.index("} flPending_t;") + len("} flPending_t;")] + "\n"
    code += "volatile flPending_t fl_pend[2]; volatile uint8_t fl_bank, fl_on, fl_job_moved;\nstruct { volatile uint8_t job; } rt;\n"
    st = src("stepper.c")
    code += "static uint8_t fl_moved;\n" + function(st, r"static void _fl_command_break\(const mpBuf_t \*bf\)") + "\n"
    code += r'''
static mpBuf_t ring[4], *wr;
static int committed, panics;
static void cm_run_short_move(void) {}
static void cm_panic_P(int s, const char *m) { (void)s; (void)m; panics++; }
static mpBuf_t *mp_get_write_buffer(void) { mpBuf_t *b = wr; memset(b, 0, sizeof(*b)); b->pv = (b == ring) ? &ring[3] : b - 1; return b; }
static void mp_commit_write_buffer(uint8_t t) { (void)t; committed++; }
static stat_t _exec_command(mpBuf_t *bf) { (void)bf; return 0; }
static void cmd(float v[], bool f[]) { (void)v; (void)f; }
'''
    code += function(src("planner.c"), r"void mp_queue_command_class\(void\(\*cm_exec\)\(float\[\], bool\[\]\), float \*value, bool \*flags, uint8_t cls\)") + "\n"
    return code


B_MAIN = r'''
static void load_move(void) { fl_moved = (fl_on && rt.job); }  /* an ALINE segment load (stepper.c) */
static void load_dwell(void) { fl_moved = false; }
static mpBuf_t cmd_buf(uint8_t cls) { mpBuf_t b; memset(&b, 0, sizeof(b)); b.block_type = BLOCK_TYPE_COMMAND; b.fl_cls = cls; return b; }
int main(void) {
    /* breaks: only with accounting on and a host job, and only after a move */
    mpBuf_t s = cmd_buf(FL_CLS_SPINDLE), c = cmd_buf(FL_CLS_COOLANT), o = cmd_buf(FL_CLS_OTHER), bad = cmd_buf(9);
    fl_on = 1; rt.job = 1; fl_job_moved = 1;
    _fl_command_break(&s); assert(fl_pend[0].breaks[FL_CLS_SPINDLE] == 0);            /* no move before it */
    load_move(); _fl_command_break(&s); _fl_command_break(&c);                        /* S then M8: one break */
    assert(fl_pend[0].breaks[FL_CLS_SPINDLE] == 1 && fl_pend[0].breaks[FL_CLS_COOLANT] == 0);
    load_move(); _fl_command_break(&c); assert(fl_pend[0].breaks[FL_CLS_COOLANT] == 1);
    load_move(); load_dwell(); _fl_command_break(&o); assert(fl_pend[0].breaks[FL_CLS_OTHER] == 0);   /* after a dwell */
    load_move(); _fl_command_break(&bad); assert(fl_pend[0].breaks[FL_CLS_OTHER] == 1);  /* unknown class -> other */
    rt.job = 0; load_move(); _fl_command_break(&s); assert(fl_pend[0].breaks[FL_CLS_SPINDLE] == 1);   /* no job */
    rt.job = 1; fl_on = 0; load_move(); _fl_command_break(&s); assert(fl_pend[0].breaks[FL_CLS_SPINDLE] == 1); /* off */
    fl_on = 1; fl_bank = 1; load_move(); _fl_command_break(&s); assert(fl_pend[1].breaks[FL_CLS_SPINDLE] == 1);  /* bank */
    /* a move outside the job, or before this job moved, is not a predecessor */
    rt.job = 0; load_move(); rt.job = 1; _fl_command_break(&s); assert(fl_pend[1].breaks[FL_CLS_SPINDLE] == 1);
    load_move(); fl_job_moved = 0; _fl_command_break(&s); assert(fl_pend[1].breaks[FL_CLS_SPINDLE] == 1);
    fl_job_moved = 1; load_move(); _fl_command_break(&s); assert(fl_pend[1].breaks[FL_CLS_SPINDLE] == 2);

    /* tagging: the queued move before a command gets FL_COMMAND and the command's class */
    float v[AXES] = { 0 }; bool f[AXES] = { 0 };
    memset(ring, 0, sizeof(ring)); wr = &ring[1];
    ring[0].buffer_state = MP_BUFFER_QUEUED; ring[0].block_type = BLOCK_TYPE_ALINE; ring[0].block_state = BLOCK_INITIALIZING;
    mp_queue_command_class(cmd, v, f, FL_CLS_COOLANT);
    assert(ring[1].block_type == BLOCK_TYPE_COMMAND && ring[1].fl_cls == FL_CLS_COOLANT && committed == 1);
    assert((ring[0].fl_tags & FL_COMMAND) && ring[0].fl_cls == FL_CLS_COOLANT && SREG == 0x80);
    ring[0].fl_tags = FL_JUNCTION | FL_COMMAND; ring[0].fl_cls = FL_CLS_SPINDLE; wr = &ring[1];   /* keeps the first class */
    mp_queue_command_class(cmd, v, f, FL_CLS_OTHER);
    assert(ring[0].fl_tags == (FL_JUNCTION | FL_COMMAND) && ring[0].fl_cls == FL_CLS_SPINDLE);
    ring[0].fl_tags = 0; ring[0].block_state = BLOCK_RUNNING; wr = &ring[1];                  /* exec took it: untouched */
    mp_queue_command_class(cmd, v, f, FL_CLS_SPINDLE); assert(ring[0].fl_tags == 0);
    ring[0].block_state = BLOCK_INITIALIZING; ring[0].buffer_state = MP_BUFFER_EMPTY; wr = &ring[1];   /* freed */
    mp_queue_command_class(cmd, v, f, FL_CLS_SPINDLE); assert(ring[0].fl_tags == 0);
    ring[0].buffer_state = MP_BUFFER_QUEUED; ring[0].block_type = BLOCK_TYPE_COMMAND; wr = &ring[1];   /* a command */
    mp_queue_command_class(cmd, v, f, FL_CLS_SPINDLE); assert(ring[0].fl_tags == 0);
    return 0;
}
'''


class AdvisorStep3Commands(unittest.TestCase):
    build_run = _build_run

    def test_breaks_and_tagging(self):
        self.build_run(b_code() + B_MAIN, "commands")

    def test_command_wiring(self):
        sp, cm, pl = src("spindle.c"), src("canonical_machine.c"), src("plan_line.c")
        self.assertEqual(len(re.findall(r"mp_queue_command_class\([^;]*FL_CLS_SPINDLE\)", sp)), 2)   # M3/M4/M5 and S
        self.assertEqual(len(re.findall(r"mp_queue_command_class\([^;]*FL_CLS_COOLANT\)", cm)), 2)   # M7 and M8/M9
        self.assertRegex(pl, r"if \(pv_type == BLOCK_TYPE_COMMAND\) \{[^}]*fl_tags \|= FL_COMMAND;")  # the move after a command
        st = src("stepper.c")
        self.assertRegex(st, r"_fl_command_break\(st_pre\.bf\);[^\n]*\n\s*mp_runtime_command\(st_pre\.bf\);")
        self.assertEqual(st.count("fl_moved = (fl_on && rt.job);"), 1)      # only a job move is a predecessor


# C. Supply gaps: with accounting on and a host job, a stop after motion began in which the runtime drained with
#    no hold, M0/M1, alarm, flush or kill is a supply gap. Class 1: input is on the board (a line in RX, a buffered
#    data line, an arc being generated); class 2: no known complete input. A class change keeps the gap; a hold or
#    M0/M1 stops the accrual and keeps it open; alarm, flush, kill and job end close it. The exec latches an underrun
#    so a drain and refill between two looks still counts. While starved, fl carries "g" and the last line run.
import test_feed_limit_accounting as fla_t

U_MAIN = r'''
int main(void) {
    fl_on = 1; rt.job = 1; st_pre.buffer_state = 2;                   /* the loader finds nothing prepared */
    load_empty(); assert(fl_underrun);
    fl_underrun = 0; cm.hold_state = 4; load_empty(); assert(!fl_underrun); cm.hold_state = 0;     /* hold */
    cm.program_paused = true; load_empty(); assert(!fl_underrun); cm.program_paused = false;      /* M0/M1 */
    rt.job = 0; load_empty(); assert(!fl_underrun); rt.job = 1;                                   /* no job */
    fl_on = 0; load_empty(); assert(!fl_underrun); fl_on = 1;                                     /* off */
    return 0;
}
'''


def u_code():
    st = src("stepper.c")
    load = function(st, r"static void _load_move\(\)")
    return (B_PRELUDE + "#define PREP_BUFFER_OWNED_BY_LOADER 0\n#define FEEDHOLD_OFF 0\n"
            "struct { int buffer_state; } st_pre;\nstruct { int hold_state; bool program_paused; } cm;\n"
            "volatile uint8_t fl_on, fl_underrun;\nstruct { volatile uint8_t job; } rt;\n"
            "static void load_empty(void) {\n" + fla_t.block(load, "if (st_pre.buffer_state != PREP_BUFFER_OWNED_BY_LOADER) {") + "\n}\n")

C_MAIN = r'''
static stat_t set_fi(uint32_t v) { nvObj_t nv = {0}; nv.valuetype = TYPE_INTEGER; nv.value_int = v; return rt_set_fi(&nv); }
static void tick(uint32_t to) { while ((int32_t)(to - systick) > 0) { systick++; assert(rt_feedlimit_callback() == STAT_OK); } }
static void moving(void) { busy = true; fl_job_moved = 1; }
int main(void) {
    rt.allowed = true; systick = 1000; cm.machine_state = MACHINE_READY;
    fl_job_moved = 1; fl_underrun = 1; assert(set_fi(1) == STAT_OK && fl_on && !fl_job_moved && !fl_underrun);   /* a new window starts clean */
    fl_underrun = 1; assert(set_fi(0) == STAT_OK && !fl_underrun); fl_underrun = 1; assert(set_fi(1) == STAT_OK && !fl_underrun);
    rt.job = 1;
    tick(1100); assert(fla.gaps == 0);                                            /* drained before any motion: not a gap */
    moving(); tick(1200); assert(fla.gaps == 0 && !fla.gap_open);
    busy = false; uint32_t s = systick;                                           /* drained, nothing on the board */
    tick(s + 512); assert(fla.gaps == 1 && fla.gap_open && fla.gap_cls == FL_GAP_NONE);
    rx_unread = true; tick(s + 768);                                              /* a line arrives: same gap, class 1 */
    assert(fla.gaps == 1 && fla.gap_cls == FL_GAP_PENDING && fla.gap_ticks[FL_GAP_NONE - 1] == 512);
    rx_unread = false; runnable = true; tick(s + 868);                            /* refilled: the gap ends */
    assert(fla.gaps == 1 && !fla.gap_open && fla.gap_cls == 0 && fla.gap_ticks[FL_GAP_PENDING - 1] == 256);
    runnable = false; in_flight = true; tick(s + 900); assert(fla.gaps == 1);      /* an aline still in flight */
    in_flight = false; st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER; tick(s + 950); assert(fla.gaps == 1);
    st_pre.buffer_state = 0; rx_pending = true; tick(s + 1000);                   /* a buffered data line: class 1 */
    assert(fla.gaps == 2 && fla.gap_cls == FL_GAP_PENDING); rx_pending = false; arc.run_state = 1; tick(s + 1010);
    assert(fla.gaps == 2 && fla.gap_cls == FL_GAP_PENDING); arc.run_state = 0;    /* an arc being generated: class 1 */

    /* a hold or M0/M1 stops the accrual and keeps the gap open */
    tick(s + 1100); uint32_t g = fla.gap_ticks[FL_GAP_NONE - 1];
    cm.hold_state = FEEDHOLD_HOLD; tick(s + 1400); assert(fla.gap_open && fla.gap_cls == 0 && fla.gaps == 2);
    cm.hold_state = FEEDHOLD_OFF; cm.program_paused = true; tick(s + 1700); assert(fla.gap_open && fla.gaps == 2);
    cm.program_paused = false; tick(s + 1800);
    assert(fla.gaps == 2 && fla.gap_cls == FL_GAP_NONE);
    tick(s + 1900); _fl_gap_close(systick); assert(fla.gap_ticks[FL_GAP_NONE - 1] - g < 300);   /* hold and M0 time left out */

    /* alarm, flush, kill and job end close the gap and forget the motion: no gap until motion starts again */
    int (*stops[])[1] = { 0 }; (void)stops;
    for (int k = 0; k < 6; k++) {
        moving(); tick(systick + 5); busy = false; tick(systick + 5); assert(fla.gap_open);
        uint32_t n = fla.gaps;
        switch (k) {
        case 0: cm.machine_state = MACHINE_ALARM; break;     case 1: cm.machine_state = MACHINE_SHUTDOWN; break;
        case 2: cm.queue_flush_state = 1; break;             case 3: cm.job_kill_requested = true; break;
        case 4: cm.machine_state = MACHINE_PROGRAM_END; break; case 5: rt.job = 0; break;
        }
        tick(systick + 5); assert(!fla.gap_open && !fl_job_moved && fla.gap_cls == 0);
        cm.machine_state = MACHINE_READY; cm.queue_flush_state = 0; cm.job_kill_requested = false; rt.job = 1;
        tick(systick + 100); assert(fla.gaps == n);
    }

    /* drained and refilled between two looks: the exec's underrun latch counts it once */
    moving(); tick(systick + 10); uint32_t n = fla.gaps;
    fl_underrun = 1; tick(systick + 1); assert(fla.gaps == n + 1 && !fla.gap_open && !fl_underrun);
    tick(systick + 100); assert(fla.gaps == n + 1);
    busy = false; tick(systick + 10); assert(fla.gaps == n + 2);                  /* a seen gap ... */
    fl_underrun = 1; busy = true; tick(systick + 1); assert(fla.gaps == n + 2);    /* ... latched during it: not twice */
    fl_job_moved = 0; fl_underrun = 1; tick(systick + 1); assert(fla.gaps == n + 2 && !fl_underrun);   /* before motion */

    /* fl while starved: {"fl":{"k":0,"m":0,"p":null,"n":<last line>,"g":1|2}}; refilled: no g */
    nframes = 0; fl_cur = FL_CUR_NONE; cm.motion_state = MOTION_STOP; mr.gm.linenum = 77;
    moving(); tick(systick + 300); busy = false;
    while (!count("\"g\":2")) tick(systick + 1);
    assert(strstr(last(), "{\"fl\":{\"k\":0,\"m\":0,\"p\":null,\"n\":77,\"g\":2}}")); printf("%s", last());
    rx_unread = true; uint32_t t = systick; while (!count("\"g\":1")) tick(systick + 1);
    assert(systick - t >= FL_EVENT_TICKS - 1 && strstr(last(), "\"n\":77,\"g\":1}}"));
    rx_unread = false; busy = true; int had = nframes; while (nframes == had) tick(systick + 1);
    assert(strstr(last(), "{\"fl\":{\"k\":0,\"m\":0,\"p\":null,\"n\":null}}"));

    /* accounting off: no gaps */
    assert(set_fi(0) == STAT_OK); n = fla.gaps; busy = false; tick(systick + 500); assert(fla.gaps == n && !fla.gap_open);
    return 0;
}
'''


class AdvisorStep3Gaps(unittest.TestCase):
    run_c = fla_t.FeedLimitAccounting.run_c
    build_run = _build_run

    def test_supply_gaps(self):
        code = fla_t.COMMON + fla_t.feedlimit_code() + fla_t.LINK + fla_t.runtime_code() + C_MAIN
        out = self.run_c(code, "gaps")
        import json
        self.assertEqual(json.loads(out.splitlines()[0])["fl"], {"k": 0, "m": 0, "p": None, "n": 77, "g": 2})

    def test_underrun_latch(self):
        self.build_run(u_code() + U_MAIN, "underrun")

    def test_gap_wiring(self):
        ex = function(src("plan_exec.c"), r"stat_t mp_exec_move\(\)")
        self.assertNotIn("fl_underrun", ex)       # an empty planner while the last segment steps is not a drain
        stage = function(src("feedlimit.c"), r"void fl_stage_block\(const mpBuf_t \*bf\)")
        self.assertIn("if (rt.job) { fl_job_moved = true; }", stage)
        self.assertIn("fl_job_moved = false;", function(src("runtime.c"), r"stat_t rt_set_fi\(nvObj_t \*nv\)"))
        unread = function(src("xio/xio_usb.c"), r"uint8_t xio_usb_rx_unread\(void\)")
        self.assertNotIn("usb_activity", unread)          # no side effect on the heartbeat's quiet detection
        cb = function(src("runtime.c"), r"stat_t rt_feedlimit_callback\(void\)")
        self.assertLess(cb.index("_fl_gap(now);"), cb.index("_fl_event(now);"))


# D. fs v2 and the fg frame. fs carries "v":2 and a 7th c entry (command boundary) and stays ungated. fg holds the gap
#    and boundary counters: {"fg":{"e":E,"q":Q,"g":[ms_pending,ms_none,gaps],"b":[[n,ms],[n,ms],[n,ms]],"x":1?}}. It is
#    sent only after the host opts in with {"rt":{"fg":1}} (a host that does not know fg would take it for damage),
#    right after each fs with the same e and q; a busy link retries it and holds the next fs, so pairs never
#    interleave. Its "x" flags only the gap and boundary counters. RAM only, cleared by reset, not by a new window.
FG_MAIN = r'''
static stat_t set_fi(uint32_t v) { nvObj_t nv = {0}; nv.valuetype = TYPE_INTEGER; nv.value_int = v; return rt_set_fi(&nv); }
static void tick(uint32_t to) { while ((int32_t)(to - systick) > 0) { systick++; assert(rt_feedlimit_callback() == STAT_OK); } }
static void ask(void) { nvObj_t nv = {0}; assert(rt_get_fs(&nv) == STAT_OK); }
static void settle(void) { tick(systick + 300); }                         /* let a pending fl event go out first */
static const char *last_of(const char *k) { for (int i = nframes - 1; i >= 0; i--) if (strstr(frames[i], k)) return frames[i]; return ""; }
int main(void) {
    rt.allowed = true; systick = 1000; cm.machine_state = MACHINE_READY;
    assert(_fl_tick_ms(1024) == 1000 && _fl_tick_ms(512) == 500 && _fl_tick_ms(4294967295u) == 4194303999u);
    assert(set_fi(1) == STAT_OK);

    /* without the opt-in: fs only, as before */
    settle(); nframes = 0; ask(); tick(systick + 50);
    assert(nframes == 1 && strstr(last(), "{\"fs\":{\"v\":2,") && count("\"fg\"") == 0);

    /* opted in: fs then fg with the same e and q */
    rt.fg = 1; fla.gaps = 3; fla.gap_ticks[0] = 2048; fla.gap_ticks[1] = 1024;
    fla.breaks[FL_CLS_SPINDLE] = 4; fla.cls_ms[FL_CLS_SPINDLE] = 120; fla.breaks[FL_CLS_OTHER] = 1; fla.cls_ms[FL_CLS_OTHER] = 7;
    nframes = 0; ask(); tick(systick + 50);
    assert(nframes == 2 && strstr(frames[0], "{\"fs\":{\"v\":2,\"e\":1,\"q\":2,"));
    assert(!strcmp(frames[1], "{\"fg\":{\"e\":1,\"q\":2,\"g\":[2000,1000,3],\"b\":[[4,120],[0,0],[1,7]]}}\n"));
    printf("%s", frames[1]);

    /* an open gap's accruing interval is included */
    rt.job = 1; fl_job_moved = 1; busy = false; tick(systick + 1);         /* a class-2 gap opens */
    tick(systick + 1024); nframes = 0; ask(); tick(systick + 50);
    assert(nframes == 2 && strstr(frames[1], "\"g\":[2000,") && fla.gaps == 4);
    { uint32_t a, b, n; assert(sscanf(strstr(frames[1], "\"g\":["), "\"g\":[%u,%u,%u]", &a, &b, &n) == 3); assert(b >= 2000 && b < 2200 && n == 4); }
    rt.job = 0; tick(systick + 2); settle();

    /* link busy after the fs: fg is retried and the next fs waits for it */
    nframes = 0; ask(); tx_ready = 1; tick(systick + 1); assert(nframes == 1 && fla.fg_due);
    tx_ready = 0; ask(); tick(systick + 200); assert(nframes == 1);
    tx_ready = 1; while (nframes < 2) tick(systick + 1); assert(strstr(last(), "{\"fg\":") && fla.fs_ask);   /* fg, then */
    while (nframes < 3) tick(systick + 1); assert(strstr(last(), "{\"fs\":"));                               /* the held fs */
    while (nframes < 4) tick(systick + 1); assert(strstr(last(), "{\"fg\":") && nframes == 4);
    { char q1[16], q2[16]; sscanf(strstr(frames[2], "\"q\":"), "\"q\":%15[0-9]", q1); sscanf(strstr(frames[3], "\"q\":"), "\"q\":%15[0-9]", q2);
      assert(!strcmp(q1, q2)); }

    /* opting out drops a pending fg; fi 0 still lets an owed fg out */
    nframes = 0; ask(); tick(systick + 1); assert(fla.fg_due); rt.fg = 0; tick(systick + 50); assert(nframes == 1 && !fla.fg_due);
    rt.fg = 1; nframes = 0; ask(); tick(systick + 1); assert(fla.fg_due); assert(set_fi(0) == STAT_OK);
    tick(systick + 50); assert(nframes == 2 && strstr(last(), "{\"fg\":"));
    assert(set_fi(1) == STAT_OK && rt.fg == 1 && !fla.fg_due);              /* a new window keeps the opt-in */

    /* x: gap or boundary wrap marks fg only; a pending wrap marks both */
    fla.gap_ticks[0] = 4294967000u; fla.gap_cls = FL_GAP_PENDING; fla.gap_t = systick - 1000; _fl_gap_close(systick);
    assert(fla.fg_invalid && !fla.invalid);
    settle(); nframes = 0; ask(); tick(systick + 50); assert(!strstr(frames[0], "\"x\"") && strstr(frames[1], ",\"x\":1}}"));
    assert(set_fi(0) == STAT_OK && set_fi(1) == STAT_OK && !fla.fg_invalid);
    fla.breaks[0] = 4294967295u; fl_pend[fl_bank].breaks[0] = 1; _fl_fold(); assert(fla.fg_invalid && !fla.invalid);
    assert(set_fi(0) == STAT_OK && set_fi(1) == STAT_OK);
    fla.cls_ms[1] = 4294967295u; fl_pend[fl_bank].cls_ticks[1] = 100; _fl_fold(); assert(fla.fg_invalid);
    assert(set_fi(0) == STAT_OK && set_fi(1) == STAT_OK);
    fla.gaps = 4294967295u; rt.job = 1; fl_job_moved = 1; busy = false; tick(systick + 2); assert(fla.fg_invalid);
    rt.job = 0; tick(systick + 2);
    assert(set_fi(0) == STAT_OK && set_fi(1) == STAT_OK);
    fl_pend[fl_bank].wrapped = 1; _fl_fold(); assert(fla.invalid && fla.fg_invalid);

    /* fi 0 freezes an open gap: no accrual while accounting is off, and summaries keep that cutoff */
    assert(set_fi(0) == STAT_OK && set_fi(1) == STAT_OK); settle();
    rt.job = 1; fl_job_moved = 1; busy = false; tick(systick + 1); assert(fla.gap_open && fla.gap_cls == FL_GAP_NONE);
    tick(systick + 1024); assert(set_fi(0) == STAT_OK && !fla.gap_open && fla.gap_cls == 0);
    { uint32_t frozen = fla.gap_ticks[FL_GAP_NONE - 1]; assert(frozen >= 1023 && frozen <= 1025);
      tick(systick + 4096); assert(fla.gap_ticks[FL_GAP_NONE - 1] == frozen);
      nframes = 0; ask(); while (count("\"fg\"") < 1) tick(systick + 1);
      char want[32]; snprintf(want, sizeof(want), "\"g\":[0,%lu,1]", (unsigned long)_fl_tick_ms(frozen)); assert(strstr(last_of("\"fg\""), want)); }
    rt.job = 0;

    /* fg is the snapshot taken with its fs, even if the counters move while it waits for the link */
    assert(set_fi(1) == STAT_OK); settle();
    fla.breaks[FL_CLS_SPINDLE] = 1; nframes = 0; ask(); while (count("\"fs\"") < 1) tick(systick + 1);
    assert(fla.fg_due); tx_ready = 0; fla.breaks[FL_CLS_SPINDLE] = 9; fla.gaps = 50; tick(systick + 100);
    tx_ready = 1; while (count("\"fg\"") < 1) tick(systick + 1);
    assert(strstr(last_of("\"fg\""), "\"g\":[0,0,0],\"b\":[[1,0],"));

    /* an open gap's running interval that wraps the total marks fg x at once */
    assert(set_fi(0) == STAT_OK && set_fi(1) == STAT_OK); settle();
    rt.job = 1; fl_job_moved = 1; busy = false; rx_unread = true; tick(systick + 1); assert(fla.gap_cls == FL_GAP_PENDING);
    fla.gap_ticks[FL_GAP_PENDING - 1] = 4294967295u - 1000; tick(systick + 2048);
    nframes = 0; ask(); while (count("\"fg\"") < 1) tick(systick + 1);
    assert(strstr(last_of("\"fg\""), ",\"x\":1}}") && fla.fg_invalid);
    rx_unread = false; rt.job = 0; tick(systick + 2);

    /* largest frames fit the send limit (each LF counts 2) and their buffers */
    fla.epoch = 65535; fla.q = 4294967294u; for (int k = 0; k <= FL_CAUSES; k++) fla.ms[k] = 4294967295u;
    fla.mm = 4294967295u; fla.um = 999; fla.gaps = 4294967295u;
    for (int k = 0; k < 2; k++) fla.gap_ticks[k] = 4294967295u;
    for (int c = 0; c < FL_CLASSES; c++) { fla.breaks[c] = 4294967295u; fla.cls_ms[c] = 4294967295u; }
    fla.gap_open = false; fla.gap_cls = 0;
    settle(); nframes = 0; ask(); tick(systick + 50); assert(nframes == 2);
    for (int i = 0; i < 2; i++) { assert(strlen(frames[i]) + 1 <= 248); printf("%s", frames[i]); }
    assert(strlen(frames[0]) < FL_FS_LEN && strlen(frames[1]) < FL_FG_LEN);
    return 0;
}
'''

FG_FLAG_MAIN = r'''
static stat_t set_int(stat_t (*f)(nvObj_t *), uint32_t v) { nvObj_t nv = { .valuetype = TYPE_INTEGER, .value_int = v }; return f(&nv); }
static void job_kill(void) {}
int main(void) {
    assert(_scan_rt("{\"rt\":{\"fg\":1}}") == RT_ENVELOPE_PURE && _scan_rt("{\"rt\":{\"fg\":null,\"fi\":2}}") == RT_ENVELOPE_PURE);
    assert(_scan_rt("{\"rt\":{\"fg\":1,\"fg\":0}}") == RT_ENVELOPE_BAD && _scan_rt("{\"rt\":{\"fg\":1},\"sv\":1}") == RT_ENVELOPE_BAD);
    memset(&rt, 0, sizeof(rt));
    assert(set_int(rt_set_fg, 1) == STAT_UNRECOGNIZED_NAME && rt.fg == 0);
    rt.allowed = true;
    assert(set_int(rt_set_fg, 2) == STAT_INPUT_VALUE_RANGE_ERROR && rt.fg == 0);
    rt.check_only = true; assert(set_int(rt_set_fg, 1) == STAT_OK && rt.fg == 0); rt.check_only = false;
    assert(set_int(rt_set_fg, 1) == STAT_OK && rt.fg == 1);
    { nvObj_t nv = { .valuetype = TYPE_NULL }; assert(rt_get_fg(&nv) == STAT_OK && nv.value_int == 1 && nv.valuetype == TYPE_INTEGER); }
    assert(set_int(rt_set_fg, 0) == STAT_OK && rt.fg == 0);
    return 0;
}
'''


class AdvisorStep3Frames(unittest.TestCase):
    run_c = fla_t.FeedLimitAccounting.run_c
    build_run = _build_run

    def test_fs_v2_and_fg(self):
        code = fla_t.COMMON + fla_t.feedlimit_code() + fla_t.LINK + fla_t.runtime_code() + FG_MAIN
        import json
        out = [json.loads(l) for l in self.run_c(code, "fg").splitlines()]
        self.assertEqual(out[0]["fg"], {"e": 1, "q": 2, "g": [2000, 1000, 3], "b": [[4, 120], [0, 0], [1, 7]]})
        self.assertEqual(len(out[1]["fs"]["c"]), 7)
        self.assertEqual(out[2]["fg"]["x"], 1)

    def test_fg_flag(self):
        h = src("runtime.h")
        code = job_code() + fn("runtime.c", "rt_get_fg") + "\n" + fn("runtime.c", "rt_set_fg") + "\n" + FG_FLAG_MAIN
        self.build_run(code, "fgflag")

    def test_fg_wiring(self):
        cfg = src("config_app.c")
        self.assertTrue(re.search(r'\{ "rt","rtfg",_i0, 0, tx_print_nul, rt_get_fg, rt_set_fg,', cfg), "no rtfg row in cfgArray")
        self.assertIn("uint8_t fg;", src("runtime.h"))
        self.assertIn("memset(&rt, 0, sizeof(rt));", fn("runtime.c", "rt_init"))   # reset clears the opt-in
