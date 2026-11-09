"""Feed-limit advisor step 3 (runtime.c, json_parser.c, canonical_machine.c): real code, stubbed machine.

A. Host job flag: {"rt":{"job":1|0}} says whether a host is streaming a job with more motion to send; only then can
   a stop be a supply gap. RAM only, written any time (no quiescence), read with {"rt":{"job":null}}. Like every rt
   leaf it is refused outside a pure rt envelope. Reset, a % flush, a job kill and program end (M2/M30) clear it.
"""
import re
import unittest

from test_runtime_rt import PRELUDE, RuntimeFamily, fn, function, src

_build_run = RuntimeFamily.build_run          # reuse the compile-and-run helper, not RuntimeFamily's tests


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

    @unittest.expectedFailure      # step 3 A: removed by the commit that adds the job flag
    def test_job_flag(self):
        self.build_run(job_code() + JOB_MAIN, "job")

    @unittest.expectedFailure      # step 3 A
    def test_job_flag_wiring(self):
        cfg = src("config_app.c")
        self.assertTrue(re.search(r'\{ "rt","rtjob",_i0, 0, tx_print_nul, rt_get_job, rt_set_job,', cfg), "no rtjob row in cfgArray")
        h = src("runtime.h")
        for proto in ("stat_t rt_get_job(nvObj_t *nv);", "stat_t rt_set_job(nvObj_t *nv);", "void rt_job_end(void);"):
            self.assertIn(proto, h)
        cm = src("canonical_machine.c")
        for name in (r"void cm_request_queue_flush\((?:void)?\)", r"void cm_request_job_kill\((?:void)?\)",
                     r"static void _exec_program_end\(float \*value, bool \*flags\)"):
            body = function(cm, name)
            self.assertIn("rt_job_end();", body, name)
        # reset: rt_init clears the whole singleton, so the flag starts at 0 after every boot and ^X
        self.assertIn("memset(&rt, 0, sizeof(rt));", fn("runtime.c", "rt_init"))


if __name__ == "__main__":
    unittest.main()
