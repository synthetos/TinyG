"""Runtime session family "rt" (runtime.c, json_parser.c _scan_rt, xio_usb.c): real code, stubbed machine.

- _scan_rt(): rt alone (with or without tid) is pure; rt with any other top-level key, a token write with
  another rt operation, or any key given twice is refused; a flat rthb or a string that merely contains "rt" is not rt.
- the token's quiescence check and commit are one critical section: a job kill arriving between them is
  delivered afterwards and invalidates it;
- rt getters/setters refuse anything but a pure envelope, and in rt_set_grp's checking pass validate without
  applying; hb is whole seconds 0-60; qt is exactly 32 hex
  digits and is accepted only at quiescence (no motion, hold, flush, queued block, prepared segment, busy
  stepper or pending data line); a read returns null once invalidated.
- rt_heartbeat_callback(): no beat when off, in text mode, with USB traffic or with the machine busy; the
  first beat a full interval after the last activity, then one per interval; any busy pass restarts the
  interval; a skipped beat is not made up; the SysTick wrap is handled; the frame carries stat, hold, up
  and qt (quoted) or null.
- xio_usb_try_send_line(): sends a whole line only with the TX ring empty, TX not paused by XOFF or CTS, and the
  line no longer than XIO_TRY_SEND_MAX; xio_usb_is_quiet() also sees traffic handled between two looks.
- every config row whose GET is its SET's function is classified: an action (ends quiescence) or a plain read;
  status reports and groups populate through nv_get(), so an acting GET there ends quiescence too; the
  activity flag is taken and cleared as one, so a byte between the two is not lost.
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FW = Path(os.environ.get("TINYG_FIRMWARE", ROOT / "firmware/tinyg"))
DEF = r"[^;{}]*\)(?=\s*(?://[^\n]*\s*)?\{)"


def src(path):
    return (FW / path).read_text(encoding="latin-1")


def fn(path, name, ret=r"(?:static )?(?:bool|uint8_t|stat_t|void)"):
    return function(src(path), ret + r"\s+" + name + r"\(" + DEF)


PRELUDE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define __AVR
#define NUL 0
#define PSTR(s) s
#define snprintf_P snprintf
typedef uint8_t stat_t;
#define STAT_OK 0
#define STAT_UNRECOGNIZED_NAME 100
#define STAT_COMMAND_NOT_ACCEPTED 106
#define STAT_INPUT_VALUE_RANGE_ERROR 110
enum { TYPE_NULL = 0, TYPE_BOOLEAN, TYPE_INTEGER, TYPE_STRING, TYPE_FLOAT };
typedef struct { uint8_t valuetype; uint32_t value_int; char *str; char strbuf[64]; } nvObj_t;
static stat_t nv_copy_string(nvObj_t *nv, const char *s) { strcpy(nv->strbuf, s); nv->str = nv->strbuf; return STAT_OK; }
uint8_t SREG = 0x80;
static void cli(void) { SREG &= ~0x80; }
enum { MACHINE_READY = 1, MACHINE_CYCLE = 5 };
enum { CYCLE_OFF = 0, CYCLE_MACHINING };
enum { MOTION_STOP = 0, MOTION_RUN };
enum { FEEDHOLD_OFF = 0, FEEDHOLD_HOLD = 4 };
enum { FLUSH_OFF = 0, FLUSH_REQUESTED };
enum { PREP_BUFFER_OWNED_BY_LOADER = 0, PREP_BUFFER_OWNED_BY_EXEC };
enum { TEXT_MODE = 0, JSON_MODE };
#define PLANNER_BUFFER_POOL_SIZE 32
struct { int machine_state, cycle_state, motion_state, hold_state, queue_flush_state; bool end_hold_requested, job_kill_requested; } cm;
struct { int buffer_state; } st_pre;
struct { int comm_mode; } cs;
static int planner_free = PLANNER_BUFFER_POOL_SIZE, stepper_busy, usb_quiet = 1, data_pending, sent_ok = 1;
static uint32_t ticks;
static char sent[128];
static int sends;
static uint8_t mp_get_planner_buffers_available(void) { return planner_free; }
static uint8_t st_runtime_isbusy(void) { return stepper_busy; }
static uint8_t xio_usb_is_quiet(void) { return usb_quiet; }
static int kill_inject, kill_pending;
static void job_kill(void);
static uint8_t xio_rx_data_pending(void) {
    if (kill_inject) { kill_inject = 0; if (SREG & 0x80) job_kill(); else kill_pending = 1; }   /* an interrupt here */
    return data_pending;
}
static void deliver(void) { if (kill_pending && (SREG & 0x80)) { kill_pending = 0; job_kill(); } }
static uint32_t SysTickTimer_getValue(void) { return ticks; }
static int cm_get_combined_state(void) { return cm.machine_state; }
static uint8_t xio_usb_try_send_line(const char *line) { sends++; strcpy(sent, line); return sent_ok; }
'''


def runtime_code():
    h = src("runtime.h")
    defs = "\n".join(l for l in h.splitlines() if l.startswith("#define RT_"))
    struct = h[h.index("typedef struct rtSingleton"):h.index("extern rtSingleton_t rt;")]
    code = PRELUDE + defs + "\n" + struct + "\nrtSingleton_t rt;\n"
    for name in ("rt_qt_invalidate", "_machine_quiet", "rt_heartbeat_callback", "rt_get_hb", "rt_set_hb",
                 "rt_get_qt", "_is_token", "rt_set_qt", "rt_get_up"):
        code += fn("runtime.c", name) + "\n"
    code += r'''
#define RT_ENVELOPE_NONE 0
#define RT_ENVELOPE_PURE 1
#define RT_ENVELOPE_BAD 2
'''
    js = src("json_parser.c")
    code += function(js, r"static bool _is_key_char\(char c\)") + "\n"
    code += function(js, r"static uint8_t _scan_rt\(const char \*p\)") + "\n"
    return code


MAIN = r'''
static void job_kill(void) { rt.qt_valid = false; cm.job_kill_requested = true; }        /* cm_request_job_kill's effects */
static void idle(void) {
    memset(&cm, 0, sizeof(cm)); cm.machine_state = MACHINE_READY; st_pre.buffer_state = PREP_BUFFER_OWNED_BY_EXEC;
    planner_free = PLANNER_BUFFER_POOL_SIZE; stepper_busy = 0; usb_quiet = 1; data_pending = 0; cs.comm_mode = JSON_MODE;
}
static stat_t set_int(stat_t (*f)(nvObj_t *), uint32_t v) { nvObj_t nv = { .valuetype = TYPE_INTEGER, .value_int = v }; return f(&nv); }
static stat_t set_str(stat_t (*f)(nvObj_t *), const char *s) { nvObj_t nv = { .valuetype = TYPE_STRING }; strcpy(nv.strbuf, s); nv.str = nv.strbuf; return f(&nv); }
static void run_for(uint32_t t1) { while (ticks < t1) { ticks += 7; rt_heartbeat_callback(); } }
#define TOKEN "0123456789abcdef0123456789ABCDEF"

int main(void) {
    /* envelope classification */
    struct { const char *s; int want; } env[] = {
        {"{\"rt\":null}", RT_ENVELOPE_PURE}, {"{rt:{hb:5}}", RT_ENVELOPE_PURE}, {"{\"rt\":{\"qt\":\"" TOKEN "\"},\"tid\":4}", RT_ENVELOPE_PURE},
        {"{\"RT\":{\"hb\":1}}", RT_ENVELOPE_PURE},
        {"{\"rt\":{\"hb\":1},\"sv\":1}", RT_ENVELOPE_BAD}, {"{\"sv\":1,\"rt\":null}", RT_ENVELOPE_BAD},
        {"{\"rt\":{\"hb\":1,\"qt\":\"" TOKEN "\"}}", RT_ENVELOPE_BAD}, {"{\"rt\":{\"qt\":\"x\",\"up\":null}}", RT_ENVELOPE_BAD},
        {"{\"rthb\":1}", RT_ENVELOPE_NONE}, {"{\"gc\":\"G0 (rt)\"}", RT_ENVELOPE_NONE}, {"{\"msg\":\"{\\\"rt\\\":1}\"}", RT_ENVELOPE_NONE},
        {"{\"x\":{\"rt\":1}}", RT_ENVELOPE_NONE}, {"{\"sr\":{\"posx\":true,\"rt\":1}}", RT_ENVELOPE_NONE},
        {"{\"rt\":{\"hb\":[1,2],\"qt\":\"a\"}}", RT_ENVELOPE_BAD}, {"{\"rt\":{\"hb\":1,\"up\":null}}", RT_ENVELOPE_PURE},
        {"{\"rt\":{\"hb\":1,\"hb\":61}}", RT_ENVELOPE_BAD}, {"{\"rt\":{\"up\":null,\"up\":null}}", RT_ENVELOPE_BAD},
        {"{\"rt\":null,\"rt\":{\"hb\":1}}", RT_ENVELOPE_BAD}, {"{rt:{hb:1,HB:2}}", RT_ENVELOPE_BAD},
    };
    for (unsigned i = 0; i < sizeof(env) / sizeof(env[0]); i++) {
        int got = _scan_rt(env[i].s);
        if (got != env[i].want) { fprintf(stderr, "_scan_rt(%s) = %d, want %d\n", env[i].s, got, env[i].want); return 1; }
    }

    /* access only through a pure envelope */
    idle(); memset(&rt, 0, sizeof(rt));
    assert(set_int(rt_set_hb, 1) == STAT_UNRECOGNIZED_NAME && rt.hb == 0);
    { nvObj_t nv = {0}; assert(rt_get_hb(&nv) == STAT_UNRECOGNIZED_NAME); assert(rt_get_qt(&nv) == STAT_UNRECOGNIZED_NAME); }
    assert(set_str(rt_set_qt, TOKEN) == STAT_UNRECOGNIZED_NAME && !rt.qt_valid);
    rt.allowed = true;

    /* hb bounds */
    assert(set_int(rt_set_hb, 61) == STAT_INPUT_VALUE_RANGE_ERROR && rt.hb == 0);
    { nvObj_t nv = { .valuetype = TYPE_STRING, .str = "5" }; assert(rt_set_hb(&nv) == STAT_INPUT_VALUE_RANGE_ERROR); }
    assert(set_int(rt_set_hb, 60) == STAT_OK && rt.hb == 60);
    assert(set_int(rt_set_hb, 0) == STAT_OK && rt.hb == 0);
    rt.check_only = true;                                                       /* rt_set_grp's checking pass */
    assert(set_int(rt_set_hb, 7) == STAT_OK && rt.hb == 0);
    assert(set_int(rt_set_hb, 61) == STAT_INPUT_VALUE_RANGE_ERROR);
    rt.check_only = false;

    /* token validation and quiescence */
    const char *bad[] = { "", "0123", TOKEN "0", "0123456789abcdef0123456789abcdeg", "0123456789abcdef 123456789abcdef" };
    for (unsigned i = 0; i < 5; i++) assert(set_str(rt_set_qt, bad[i]) == STAT_INPUT_VALUE_RANGE_ERROR && !rt.qt_valid);
    struct { const char *why; int *p; int v; } states[] = {
        {"cycle", &cm.cycle_state, CYCLE_MACHINING}, {"motion", &cm.motion_state, MOTION_RUN}, {"hold", &cm.hold_state, FEEDHOLD_HOLD},
        {"flush", &cm.queue_flush_state, FLUSH_REQUESTED}, {"machine cycle", &cm.machine_state, MACHINE_CYCLE},
        {"planner", &planner_free, PLANNER_BUFFER_POOL_SIZE - 1}, {"stepper", &stepper_busy, 1},
        {"prepared segment", &st_pre.buffer_state, PREP_BUFFER_OWNED_BY_LOADER}, {"data line", &data_pending, 1},
    };
    for (unsigned i = 0; i < sizeof(states) / sizeof(states[0]); i++) {
        idle(); *states[i].p = states[i].v;
        if (set_str(rt_set_qt, TOKEN) != STAT_COMMAND_NOT_ACCEPTED || rt.qt_valid) { fprintf(stderr, "token accepted while %s\n", states[i].why); return 1; }
    }
    idle(); cm.end_hold_requested = true; assert(set_str(rt_set_qt, TOKEN) == STAT_COMMAND_NOT_ACCEPTED);
    idle(); cm.job_kill_requested = true; assert(set_str(rt_set_qt, TOKEN) == STAT_COMMAND_NOT_ACCEPTED);
    idle(); rt.check_only = true; assert(set_str(rt_set_qt, TOKEN) == STAT_OK && !rt.qt_valid); rt.check_only = false;
    /* a job kill arriving while the token is checked and committed is held off until after the commit, and then
       invalidates it: masked when rt_set_qt looks at the pending data lines, it is delivered on SREG restore */
    idle(); kill_inject = 1; assert(set_str(rt_set_qt, TOKEN) == STAT_OK); deliver();
    assert(!rt.qt_valid && cm.job_kill_requested);
    idle(); usb_quiet = 0; assert(set_str(rt_set_qt, TOKEN) == STAT_OK && rt.qt_valid);   /* newer input does not block it */
    { nvObj_t nv = {0}; assert(rt_get_qt(&nv) == STAT_OK && nv.valuetype == TYPE_STRING && !strcmp(nv.str, TOKEN)); }
    rt_qt_invalidate();
    { nvObj_t nv = {0}; assert(rt_get_qt(&nv) == STAT_OK && nv.valuetype == TYPE_NULL); }

    /* heartbeat timing */
    idle(); rt.allowed = false; sends = 0; ticks = 0xFFFFF000u;                 /* across the SysTick wrap */
    run_for(0xFFFFF000u + 5000); assert(sends == 0);                             /* off */
    rt.allowed = true; set_int(rt_set_hb, 2); rt.allowed = false;
    uint32_t t0 = ticks;
    while (sends == 0) { ticks += 7; rt_heartbeat_callback(); assert(ticks - t0 < 3000); }
    assert(ticks - t0 >= 2048 && ticks - t0 < 2048 + 8);                         /* a full interval after the set */
    assert(!strncmp(sent, "{\"hb\":{\"stat\":1,\"hold\":0,\"up\":", 29) && strstr(sent, ",\"qt\":null}}\n"));
    uint32_t t1 = ticks; while (sends == 1) { ticks += 7; rt_heartbeat_callback(); }
    assert(ticks - t1 >= 2048 && ticks - t1 < 2048 + 8);                         /* then once per interval */
    /* busy passes restart the interval: none during, the next a full interval after the last busy pass */
    int before = sends; cm.motion_state = MOTION_RUN; uint32_t tb = ticks + 5000; while (ticks < tb) { ticks += 7; rt_heartbeat_callback(); }
    assert(sends == before); cm.motion_state = MOTION_STOP; uint32_t tq = ticks;
    while (sends == before) { ticks += 7; rt_heartbeat_callback(); }
    assert(ticks - tq >= 2048 && ticks - tq < 2048 + 16);
    before = sends; usb_quiet = 0; tb = ticks + 3000; while (ticks < tb) { ticks += 7; rt_heartbeat_callback(); } assert(sends == before); usb_quiet = 1;
    before = sends; cs.comm_mode = TEXT_MODE; tb = ticks + 5000; while (ticks < tb) { ticks += 7; rt_heartbeat_callback(); } assert(sends == before); cs.comm_mode = JSON_MODE;
    /* a beat that cannot go out is skipped, not made up */
    sent_ok = 0; before = sends; tq = ticks; while (sends == before) { ticks += 7; rt_heartbeat_callback(); }
    sent_ok = 1; uint32_t tf = ticks; while (sends == before + 1) { ticks += 7; rt_heartbeat_callback(); }
    assert(ticks - tf >= 2048);
    /* the token goes into the frame, quoted */
    rt.allowed = true; assert(set_str(rt_set_qt, TOKEN) == STAT_OK); rt.allowed = false;
    before = sends; while (sends == before) { ticks += 7; rt_heartbeat_callback(); }
    assert(strstr(sent, ",\"qt\":\"" TOKEN "\"}}\n"));
    puts("rt: envelope, access, token and heartbeat rules hold");
    return 0;
}
'''

SEND = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define NUL 0
#define TX_BUFFER_SIZE 250
#define FC_IN_XOFF 1
#define FLOW_CONTROL_RTS 2
#define USB_CTS_bm 0x10
uint8_t SREG = 0x80;
static void cli(void) { SREG &= ~0x80; }
#define XIO_TRY_SEND_MAX ((TX_BUFFER_SIZE - 2) / 2)
typedef uint8_t buffer_t;
struct { uint8_t IN; } port;
struct { buffer_t tx_buf_head, tx_buf_tail; uint8_t fc_state_tx; typeof(port) *port; } USBu = { 1, 1, 0, &port };
struct { int enable_flow_control; } xio;
static char out[512]; static int n;
static int _putc_usb_raw(const char c) { out[n++] = c; return 0; }
'''


class RuntimeFamily(unittest.TestCase):
    def build_run(self, code, name):
        with tempfile.TemporaryDirectory() as tmp:
            c, exe = Path(tmp) / (name + ".c"), Path(tmp) / name
            c.write_text(code)
            subprocess.run(["cc", "-std=gnu99", "-O1", "-Wall", "-Werror", "-Wno-unused-function", "-Wno-unused-variable",
                            "-fsanitize=address,undefined", str(c), "-o", str(exe)], check=True)
            r = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr + r.stdout)

    def test_runtime_rules(self):
        self.build_run(runtime_code() + MAIN, "rt")

    def test_whole_line_send(self):
        code = SEND + function(src("xio/xio_usb.c"), r"uint8_t xio_usb_try_send_line\(const char \*line\)") + r'''
int main(void) {
    assert(xio_usb_try_send_line("{\"hb\":1}\n") && n == 9 && !memcmp(out, "{\"hb\":1}\n", 9));
    n = 0; USBu.tx_buf_head = 7; assert(!xio_usb_try_send_line("x\n") && n == 0); USBu.tx_buf_head = 1;   /* output queued */
    USBu.fc_state_tx = FC_IN_XOFF; assert(!xio_usb_try_send_line("x\n") && n == 0); USBu.fc_state_tx = 0;
    xio.enable_flow_control = FLOW_CONTROL_RTS; port.IN = USB_CTS_bm; assert(!xio_usb_try_send_line("x\n") && n == 0);
    port.IN = 0; assert(xio_usb_try_send_line("x\n") && n == 2);
    char longline[XIO_TRY_SEND_MAX + 2]; memset(longline, 'a', sizeof(longline) - 1); longline[sizeof(longline) - 1] = 0;
    n = 0; assert(!xio_usb_try_send_line(longline) && n == 0);                    /* longer than an empty ring can take */
    longline[XIO_TRY_SEND_MAX] = 0; assert(xio_usb_try_send_line(longline) && n == XIO_TRY_SEND_MAX);
    return 0;
}
'''
        self.build_run(code, "send")


    def test_acting_gets_are_classified(self):
        """Every row whose GET is its SET's function either ends quiescence (cfg_get_is_action) or is a plain read."""
        table = src("config_app.c")
        rows = re.findall(r'\{\s*"([^"]*)"\s*,\s*"([^"]*)"\s*,[^,]*,[^,]*,\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*,', table)
        same = {(t, g) for _, t, _, g, s in rows if g == s}
        listed = set(re.findall(r"get == (\w+)", function(table, r"bool cfg_get_is_action\(fptrCmd get\)")))
        plain_reads = {"help_config", "_reserved_hold_setting"}
        for token, get in sorted(same):
            self.assertTrue(get in listed or get in plain_reads, f"{token}: GET {get} acts like its SET; classify it")
        self.assertTrue({"st_set_md", "st_set_me", "cm_clr", "cm_alrm", "cm_pnic", "cm_shutd"} <= listed)
        get = function(src("config.c"), r"stat_t nv_get\(nvObj_t \*nv\)")
        self.assertRegex(get, r"cfg_get_is_action\(get\)\)\s*\{\s*rt_qt_invalidate\(\);")

    def test_usb_quiet_sees_finished_traffic(self):
        code = SEND.replace("struct { buffer_t tx_buf_head, tx_buf_tail; uint8_t fc_state_tx; typeof(port) *port; } USBu = { 1, 1, 0, &port };",
                            "struct { buffer_t tx_buf_head, tx_buf_tail, rx_buf_head, rx_buf_tail; uint8_t fc_state_tx; typeof(port) *port; } USBu = { 1, 1, 1, 1, 0, &port };\n"
                            "struct { uint8_t flag_in_line, len; } USB;\nstatic volatile uint8_t usb_activity;")
        code += function(src("xio/xio_usb.c"), r"uint8_t xio_usb_is_quiet\(void\)") + r'''
int main(void) {
    assert(xio_usb_is_quiet());
    usb_activity = 1; assert(!xio_usb_is_quiet()); assert(xio_usb_is_quiet());   /* a byte handled between looks */
    USB.flag_in_line = 1; USB.len = 0; assert(xio_usb_is_quiet());               /* an empty line header is quiet */
    USB.len = 3; assert(!xio_usb_is_quiet()); USB.flag_in_line = 0;
    USBu.rx_buf_head = 5; assert(!xio_usb_is_quiet()); USBu.rx_buf_head = 1;
    USBu.tx_buf_head = 5; assert(!xio_usb_is_quiet());
    return 0;
}
'''
        self.build_run(code, "quiet")


    def test_report_population_ends_quiescence_on_acting_get(self):
        """Status reports and groups fill values through nv_populate_nv_by_index(): an acting GET there (md in a
        report) must end quiescence exactly as a direct GET does."""
        cfg = src("config.c")
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define strcpy_P strcpy
#define NUL 0
typedef uint8_t stat_t;
typedef int16_t index_t;
enum { TYPE_NULL = 0, TYPE_INTEGER = 2, TYPE_FLOAT = 4 };
#define F_NOSTRIP 0x40
#define STAT_OK 0
#define STAT_INTERNAL_RANGE_ERROR 9
typedef struct nvObject { index_t index; uint8_t valuetype; int8_t precision; char group[4]; char token[8]; char *str; uint32_t value_int; } nvObj_t;
typedef uint8_t (*fptrCmd)(nvObj_t *nv);
typedef struct { char group[4]; char token[8]; uint8_t flags; int8_t precision; fptrCmd get; } cfgItem_t;
static int disabled, qt_valid = 1;
static stat_t st_set_md(nvObj_t *nv) { (void)nv; disabled = 4; return STAT_OK; }
static stat_t get_ui8(nvObj_t *nv) { nv->value_int = 7; return STAT_OK; }
static const cfgItem_t cfgArray[] = { { "", "md", F_NOSTRIP, 0, st_set_md }, { "", "stat", F_NOSTRIP, 0, get_ui8 } };
#define GET_TABLE_WORD(a) cfgArray[nv->index].a
static index_t nv_index_max(void) { return 2; }
static uint8_t cfg_is_type(index_t i) { (void)i; return TYPE_INTEGER; }
static bool cfg_has_flag(index_t i, uint8_t m) { return (cfgArray[i].flags & m) != 0; }
static bool cfg_get_is_action(fptrCmd get) { return get == st_set_md; }
static void rt_qt_invalidate(void) { qt_valid = 0; }
''' + function(cfg, r"stat_t nv_get\(nvObj_t \*nv\)") + "\n" + function(cfg, r"void nv_populate_nv_by_index\(nvObj_t \*nv, index_t index\)") + r'''
int main(void) {
    nvObj_t nv = {0};
    nv_populate_nv_by_index(&nv, 1); assert(nv.value_int == 7 && qt_valid);      /* a plain read keeps the token */
    nv_populate_nv_by_index(&nv, 0); assert(disabled == 4 && !qt_valid);          /* md in a report ends it */
    return 0;
}
'''
        self.build_run(code, "populate")
        self.assertEqual(len(re.findall(r"GET_TABLE_WORD\(get\)", cfg)), 1, "every getter call goes through nv_get()")

    def test_activity_take_is_atomic(self):
        """A byte whose interrupt arrives between reading and clearing the activity flag is not lost."""
        quiet = function(src("xio/xio_usb.c"), r"uint8_t xio_usb_is_quiet\(void\)")
        assert quiet.count("usb_activity = false;") == 1
        quiet = quiet.replace("usb_activity = false;", "rx_window(); usb_activity = false;")
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
typedef uint8_t buffer_t;
uint8_t SREG = 0x80;
static void cli(void) { SREG &= ~0x80; }
struct { buffer_t tx_buf_head, tx_buf_tail, rx_buf_head, rx_buf_tail; } USBu = { 1, 1, 1, 1 };
struct { uint8_t flag_in_line, len; } USB;
static volatile uint8_t usb_activity;
static int inject, pending;
static void rx_isr(void) { usb_activity = true; }                 /* the RX interrupt's store, e.g. for an ENQ */
static void rx_window(void) { if (inject) { inject = 0; if (SREG & 0x80) rx_isr(); else pending = 1; } }
''' + quiet + r'''
int main(void) {
    assert(xio_usb_is_quiet());
    inject = 1; (void)xio_usb_is_quiet();
    if (pending && (SREG & 0x80)) { pending = 0; rx_isr(); }       /* a masked interrupt runs on SREG restore */
    assert(!xio_usb_is_quiet());                                   /* the byte was seen, not cleared away */
    return 0;
}
'''
        self.build_run(code, "atomic")


if __name__ == "__main__":
    unittest.main()
