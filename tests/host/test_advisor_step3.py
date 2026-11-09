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
    code = PRELUDE + defs + "\n" + struct + "\nrtSingleton_t rt;\n"
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
    code += "volatile flPending_t fl_pend[2]; volatile uint8_t fl_bank, fl_on;\nstruct { volatile uint8_t job; } rt;\n"
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
static void load_move(void) { fl_moved = true; }               /* an ALINE segment load (stepper.c) */
static void load_dwell(void) { fl_moved = false; }
static mpBuf_t cmd_buf(uint8_t cls) { mpBuf_t b; memset(&b, 0, sizeof(b)); b.block_type = BLOCK_TYPE_COMMAND; b.fl_cls = cls; return b; }
int main(void) {
    /* breaks: only with accounting on and a host job, and only after a move */
    mpBuf_t s = cmd_buf(FL_CLS_SPINDLE), c = cmd_buf(FL_CLS_COOLANT), o = cmd_buf(FL_CLS_OTHER), bad = cmd_buf(9);
    fl_on = 1; rt.job = 1;
    _fl_command_break(&s); assert(fl_pend[0].breaks[FL_CLS_SPINDLE] == 0);            /* no move before it */
    load_move(); _fl_command_break(&s); _fl_command_break(&c);                        /* S then M8: one break */
    assert(fl_pend[0].breaks[FL_CLS_SPINDLE] == 1 && fl_pend[0].breaks[FL_CLS_COOLANT] == 0);
    load_move(); _fl_command_break(&c); assert(fl_pend[0].breaks[FL_CLS_COOLANT] == 1);
    load_move(); load_dwell(); _fl_command_break(&o); assert(fl_pend[0].breaks[FL_CLS_OTHER] == 0);   /* after a dwell */
    load_move(); _fl_command_break(&bad); assert(fl_pend[0].breaks[FL_CLS_OTHER] == 1);  /* unknown class -> other */
    rt.job = 0; load_move(); _fl_command_break(&s); assert(fl_pend[0].breaks[FL_CLS_SPINDLE] == 1);   /* no job */
    rt.job = 1; fl_on = 0; load_move(); _fl_command_break(&s); assert(fl_pend[0].breaks[FL_CLS_SPINDLE] == 1); /* off */
    fl_on = 1; fl_bank = 1; load_move(); _fl_command_break(&s); assert(fl_pend[1].breaks[FL_CLS_SPINDLE] == 1);  /* bank */

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
        self.assertEqual(st.count("fl_moved = true;"), 1)
