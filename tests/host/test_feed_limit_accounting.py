"""Feed-limit accounting and push (step 2): feedlimit.c fl_stage_block/fl_segment, stepper.c _fl_retire and
its _load_move wiring, runtime.c fi/fs and rt_feedlimit_callback, xio_usb.c xio_usb_try_send_frame.

- Records (exec): eligible only for a G94 linear XYZ feed segment prepared outside a feedhold; below F
  (1% or 2 mm/min short of F) it carries every cause bit of its block, or "other" when the block has none;
  the length carries its sub-micrometre remainder, so many segments sum without drift.
- Retire (loader): a finished segment's record is credited once; a second load call adds nothing; with
  accounting off a record is dropped; all six cause bits add the segment's ticks to all six counters.
- Totals (main loop): ticks fold into ms with the remainder kept between folds, um into mm the same way;
  q counts summaries actually sent; a summary at maximum values is valid JSON and fits the TX ring.
- fi: clamps to 60 and echoes it; off to on opens a new window (epoch + 1, totals zeroed) only at
  quiescence with no pending input; a new interval keeps the totals; 0 stops crediting and keeps them.
- Callback: the first summary comes fi s after a cycle starts; after the cycle ends the final one waits
  until the runtime has drained (not busy, nothing prepared, the last record retired), and is retried until
  the link takes it; fl lines follow changes at most 4 a second, coalesced to the latest state, and report
  the change to no move.
- Frame sender: the room check is exact (each LF counted twice, TX_BUFFER_SIZE - 2 bytes); busy, XOFF or
  CTS refuse it whole.
"""
from pathlib import Path
import json
import os
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FW = Path(os.environ.get("TINYG_FIRMWARE", ROOT / "firmware/tinyg"))


def src(path):
    return (FW / path).read_text(encoding="latin-1")


def block(source, opener):
    """The braced statement starting at `opener`, through its closing brace."""
    i = source.index(opener)
    depth, j = 0, source.index("{", i)
    while True:
        depth += {"{": 1, "}": -1}.get(source[j], 0)
        if depth == 0:
            return source[i:j + 1]
        j += 1


COMMON = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define __AVR
#define PSTR(s) (s)
#define snprintf_P snprintf
typedef uint8_t stat_t;
#define STAT_OK 0
#define STAT_UNRECOGNIZED_NAME 100
#define STAT_INPUT_VALUE_RANGE_ERROR 101
#define STAT_COMMAND_NOT_ACCEPTED 106
enum { TYPE_NULL = 0, TYPE_INTEGER = 2, TYPE_FLOAT = 4 };
enum { MOTION_STOP = 0, MOTION_RUN, MOTION_HOLD };
enum { FEEDHOLD_OFF = 0, FEEDHOLD_SYNC = 1, FEEDHOLD_HOLD = 4 };
enum { CYCLE_OFF = 0, CYCLE_MACHINING = 1 };
enum { MOTION_MODE_STRAIGHT_TRAVERSE = 0, MOTION_MODE_STRAIGHT_FEED = 1 };
enum { INVERSE_TIME_MODE = 0, UNITS_PER_MINUTE_MODE = 1 };
enum { AXIS_X, AXIS_Y, AXIS_Z, AXIS_A, AXIS_B, AXIS_C, AXES };
#define NOM_SEGMENT_TIME (5000.0f / 60000000)
#define PREP_BUFFER_OWNED_BY_LOADER 1
#define RT_TICKS_PER_SECOND 1024UL
#define RT_FI_MAX 60
typedef struct nvObj { uint8_t valuetype; union { float value_flt; uint32_t value_int; }; } nvObj_t;
typedef struct { uint8_t motion_mode, feed_rate_mode; float feed_rate; uint32_t linenum; } GCodeState_t;
typedef struct mpBuffer { float unit[AXES]; float cruise_velocity, cruise_vmax, naiive_move_time;
                          GCodeState_t gm; uint8_t fl_tags; } mpBuf_t;
uint8_t SREG = 0x80;
static void cli(void) { SREG &= ~0x80; }
struct { int motion_state, hold_state, cycle_state; } cm;
struct { bool allowed, check_only; } rt;
struct { uint8_t fl_load, fl_rec; uint32_t fl_um, dda_ticks; int buffer_state; } st_pre;
struct { uint8_t fl_rec; uint32_t fl_um, fl_ticks; } st_run;
'''


def feedlimit_code():
    h, fc = src("feedlimit.h"), src("feedlimit.c")
    code = "\n".join(l for l in h.splitlines() if l.startswith("#define FL_")) + "\n"
    code += h[h.index("typedef struct flBlock"):h.index("} flBlock_t;") + len("} flBlock_t;")] + "\n"
    i = h.index("static inline uint8_t fl_axis_moves(float unit)")
    code += h[i:h.index("\n}", i) + 2] + "\n"
    code += h[h.index("typedef struct flPending"):h.index("} flPending_t;") + len("} flPending_t;")] + "\n"
    last = "static float fl_blk_below;"
    code += fc[fc.index("flBlock_t fl_slot[2];"):fc.index(last) + len(last)] + "\n"
    code += function(fc, r"uint8_t fl_profile_tags\(float naiive_move_time, float cruise, float cruise_vmax\)") + "\n"
    code += function(fc, r"void fl_stage_block\(const mpBuf_t \*bf\)") + "\n"
    code += function(fc, r"void fl_segment\(float velocity, float segment_time\)") + "\n"
    return code


def loader_code():
    st = src("stepper.c")
    load = function(st, r"static void _load_move\(\)")
    code = function(st, r"static void _fl_retire\(void\)") + "\n"
    code += "static void load_entry(void) {\n" + block(load, "if (st_run.fl_rec) {") + "\n}\n"
    code += "static void load_aline(void) {\n" + block(load, "if ((st_run.fl_rec = st_pre.fl_rec)) {") + "\n}\n"
    return code


def runtime_code():
    rtc = src("runtime.c")
    code = rtc[rtc.index("#define FL_FS_LEN"):rtc.index("} fla;") + len("} fla;")] + "\n"
    for sig in (r"static void _fl_fold\(void\)", r"static void _fl_request_fs\(void\)", r"static bool _fl_send_fs\(void\)",
                r"static bool _fl_drained\(void\)", r"static void _fl_event\(uint32_t now\)",
                r"stat_t rt_feedlimit_callback\(void\)", r"stat_t rt_get_fi\(nvObj_t \*nv\)",
                r"stat_t rt_set_fi\(nvObj_t \*nv\)", r"stat_t rt_get_fs\(nvObj_t \*nv\)"):
        code += function(rtc, sig) + "\n"
    return code


LINK = r'''
static uint32_t systick;
uint32_t SysTickTimer_getValue(void) { return systick; }
static bool quiet = true, rx_pending = false, busy = false, rec_pending = false;
static bool _machine_quiet(void) { return quiet; }
bool xio_rx_data_pending(void) { return rx_pending; }
uint8_t st_runtime_isbusy(void) { return busy; }
uint8_t st_fl_record_pending(void) { return rec_pending; }
static int tx_ready = 1;
static char frames[64][200]; static int nframes;
uint8_t xio_usb_tx_ready(void) { return tx_ready; }
uint8_t xio_usb_try_send_frame(const char *line) {
    if (!tx_ready) return 0;
    assert(strlen(line) < 200 && nframes < 64); strcpy(frames[nframes++], line); return 1;
}
static int count(const char *key) { int c = 0; for (int i = 0; i < nframes; i++) c += strstr(frames[i], key) != NULL; return c; }
static const char *last(void) { assert(nframes); return frames[nframes - 1]; }
'''


class FeedLimitAccounting(unittest.TestCase):
    def run_c(self, code, name):
        with tempfile.TemporaryDirectory() as tmp:
            c, exe = Path(tmp) / (name + ".c"), Path(tmp) / name
            c.write_text(code)
            subprocess.run(["cc", "-std=gnu99", "-O1", "-Wall", "-Werror", "-Wno-unused-function", "-Wno-unused-variable",
                            "-fsanitize=address,undefined", str(c), "-o", str(exe)], check=True)
            r = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr + r.stdout)
        return r.stdout

    def test_records_and_retire(self):
        code = COMMON + feedlimit_code() + loader_code() + r'''
static mpBuf_t blk(float feed, float cruise, float vmax, uint8_t tags) {
    mpBuf_t b; memset(&b, 0, sizeof(b));
    b.gm.motion_mode = MOTION_MODE_STRAIGHT_FEED; b.gm.feed_rate_mode = UNITS_PER_MINUTE_MODE; b.gm.feed_rate = feed;
    b.cruise_velocity = cruise; b.cruise_vmax = vmax; b.naiive_move_time = 0.01f; b.unit[AXIS_X] = 1; b.fl_tags = tags;
    return b;
}
static uint8_t seg(float v) { fl_segment(v, 0.005f / 60); return st_pre.fl_rec; }
int main(void) {
    fl_on = 1;
    mpBuf_t b = blk(600, 600, 600, 0); fl_stage_block(&b);              /* reaches F: no tags */
    assert(seg(600) == FL_REC_ELIGIBLE && seg(595) == FL_REC_ELIGIBLE); /* within 1% is at F */
    assert(seg(593) == (FL_REC_ELIGIBLE | FL_REC_OTHER));               /* a ramp: below F, no tag */
    b = blk(600, 400, 400, FL_AXIS_CAP); fl_stage_block(&b);
    assert(seg(400) == (FL_REC_ELIGIBLE | FL_AXIS_CAP));
    b = blk(1200, 300, 1200, FL_JUNCTION); fl_stage_block(&b);          /* a corner, too short to reach F */
    assert(seg(300) == (FL_REC_ELIGIBLE | FL_JUNCTION | FL_SHORT));
    b = blk(600, 600, 600, 0); b.gm.motion_mode = MOTION_MODE_STRAIGHT_TRAVERSE; fl_stage_block(&b);
    assert(seg(100) == 0);                                              /* G0 */
    b = blk(0.5f, 100, 100, 0); b.gm.feed_rate_mode = INVERSE_TIME_MODE; fl_stage_block(&b);
    assert(seg(50) == 0);                                               /* G93 */
    b = blk(600, 600, 600, 0); b.unit[AXIS_C] = 0.1f; fl_stage_block(&b);
    assert(seg(300) == 0);                                              /* rotary */
    b = blk(600, 600, 600, 0); fl_stage_block(&b);
    cm.hold_state = FEEDHOLD_SYNC; assert(seg(300) == 0); cm.hold_state = FEEDHOLD_OFF;   /* deceleration into a hold */
    b.gm.linenum = 77; fl_stage_block(&b); assert(fl_slot[fl_stage].line == 77);

    fl_um_frac = 0; uint64_t sum = 0;                                   /* 33.333... um per segment */
    for (int i = 0; i < 3000; i++) { fl_segment(333.3333f, 0.0001f); sum += st_pre.fl_um; }
    assert(sum >= 99998 && sum <= 100001);

    memset((void *)&fl_pend, 0, sizeof(fl_pend));
    st_pre.fl_rec = FL_REC_ELIGIBLE | FL_JUNCTION; st_pre.fl_um = 50; st_pre.dda_ticks = 250;
    load_aline(); assert(st_run.fl_rec && st_run.fl_ticks == 250);
    load_entry(); load_entry();                                         /* a second LOAD call adds nothing */
    assert(fl_pend.ticks[FL_CAUSES] == 250 && fl_pend.ticks[1] == 250 && fl_pend.ticks[0] == 0 && fl_pend.um == 50);
    st_pre.fl_rec = 0; load_aline(); load_entry(); assert(fl_pend.ticks[FL_CAUSES] == 250);   /* no record: nothing */
    st_pre.fl_rec = FL_REC_ELIGIBLE | 0x3F; st_pre.dda_ticks = 100; load_aline(); load_entry();
    for (int k = 0; k <= FL_CAUSES; k++) assert(fl_pend.ticks[k] == ((k == 1) ? 350 : (k == FL_CAUSES) ? 350 : 100));
    st_pre.fl_rec = FL_REC_ELIGIBLE; load_aline(); fl_on = 0; load_entry();       /* off: dropped */
    assert(fl_pend.ticks[FL_CAUSES] == 350 && st_run.fl_rec == 0);
    return 0;
}
'''
        self.run_c(code, "records")

    def test_window_totals_and_push(self):
        code = COMMON + feedlimit_code() + LINK + runtime_code() + r'''
static stat_t set_fi(uint32_t v, uint32_t *echo) {
    nvObj_t nv = {0}; nv.valuetype = TYPE_INTEGER; nv.value_int = v; stat_t s = rt_set_fi(&nv); if (echo) *echo = nv.value_int; return s;
}
static void tick(uint32_t to) { while (systick < to) { systick++; assert(rt_feedlimit_callback() == STAT_OK); } }
int main(void) {
    rt.allowed = true; uint32_t echo;
    assert(set_fi(200, &echo) == STAT_OK && echo == 60 && fla.fi == 60 && fl_on && fla.epoch == 1);
    fla.ms[FL_CAUSES] = 7;
    assert(set_fi(5, &echo) == STAT_OK && echo == 5 && fla.epoch == 1 && fla.ms[FL_CAUSES] == 7);   /* interval only */
    assert(set_fi(0, &echo) == STAT_OK && !fl_on && fla.fi == 0 && fla.ms[FL_CAUSES] == 7);         /* off keeps totals */
    quiet = false; assert(set_fi(3, NULL) == STAT_COMMAND_NOT_ACCEPTED && fla.fi == 0 && !fl_on); quiet = true;
    rx_pending = true; assert(set_fi(3, NULL) == STAT_COMMAND_NOT_ACCEPTED); rx_pending = false;
    rt.check_only = true; assert(set_fi(3, NULL) == STAT_OK && fla.fi == 0 && !fl_on); rt.check_only = false;
    fl_pend.ticks[FL_CAUSES] = 9; fl_um_frac = 0.5f;
    assert(set_fi(1, NULL) == STAT_OK && fla.epoch == 2 && fla.ms[FL_CAUSES] == 0 && fl_on);         /* a new window */
    assert(fl_pend.ticks[FL_CAUSES] == 0 && fl_um_frac == 0);
    { nvObj_t nv = {0}; nv.valuetype = TYPE_FLOAT; assert(rt_set_fi(&nv) == STAT_INPUT_VALUE_RANGE_ERROR); }
    rt.allowed = false; { nvObj_t nv = {0}; assert(rt_get_fi(&nv) == STAT_UNRECOGNIZED_NAME); } rt.allowed = true;

    fl_pend.ticks[FL_CAUSES] = 49; fl_pend.ticks[1] = 50; fl_pend.um = 999;          /* fold with remainders */
    assert(_fl_send_fs() && fla.q == 1);
    assert(strstr(last(), "\"q\":1,") && strstr(last(), "\"c\":[0,1,0,0,0,0]") && strstr(last(), "\"ct\":0,") && strstr(last(), "\"cd\":0.999}"));
    fl_pend.ticks[FL_CAUSES] = 1; fl_pend.um = 2;
    assert(_fl_send_fs() && strstr(last(), "\"ct\":1,") && strstr(last(), "\"cd\":1.001}") && fla.q == 2);
    tx_ready = 0; fl_pend.ticks[FL_CAUSES] = 5000; assert(!_fl_send_fs() && fla.q == 2 && fl_pend.ticks[FL_CAUSES] == 5000); tx_ready = 1;
    fl_pend.ticks[FL_CAUSES] = 0;
    printf("%s", last());

    /* periodic and final summaries */
    nframes = 0; systick = 100; cm.cycle_state = CYCLE_MACHINING; tick(101);       /* a cycle starts */
    tick(100 + 1024); assert(count("\"fs\"") == 0);
    tick(100 + 1025); assert(count("\"fs\"") == 1);
    cm.cycle_state = CYCLE_OFF; busy = true; tick(3000); assert(count("\"fs\"") == 1);              /* still stepping */
    busy = false; st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER; tick(3100); assert(count("\"fs\"") == 1);
    st_pre.buffer_state = 0; rec_pending = true; tick(3200); assert(count("\"fs\"") == 1);
    rec_pending = false; tx_ready = 0; tick(3300); assert(count("\"fs\"") == 1);    /* drained, link busy: retained */
    tx_ready = 1; tick(3320); assert(count("\"fs\"") == 2); tick(6000); assert(count("\"fs\"") == 2);
    { nvObj_t nv = {0}; assert(rt_get_fs(&nv) == STAT_OK && nv.value_int == fla.q + 1); tick(6020);
      assert(count("\"fs\"") == 3 && fla.q == nv.value_int); }

    /* fl events: rate limit, coalescing, no move */
    nframes = 0; cm.motion_state = MOTION_RUN; cm.hold_state = FEEDHOLD_OFF;
    fl_slot[0] = (flBlock_t){ FL_AXIS_CAP, FL_KIND_FEED, 1, 1, 400, 600, 400, 0.01f, 12 }; fl_cur = 0;
    while (!count("\"fl\"")) tick(systick + 1);                       /* after the last event's rate limit */
    assert(count("\"fl\"") == 1 && strstr(last(), "{\"fl\":{\"k\":1,\"m\":1,\"p\":67,\"n\":12}}"));
    uint32_t t0 = systick;
    fl_slot[0].tags = FL_JUNCTION; fl_slot[0].line = 13; tick(t0 + 100);
    fl_slot[0].tags = FL_EXACT_STOP; fl_slot[0].line = 14; tick(t0 + 200); assert(count("\"fl\"") == 1);
    tick(t0 + 300); assert(count("\"fl\"") == 2 && strstr(last(), "\"m\":4,") && strstr(last(), "\"n\":14}"));   /* latest only */
    tick(t0 + 2000); assert(count("\"fl\"") == 2);                      /* unchanged: nothing more */
    tx_ready = 0; fl_cur = FL_CUR_NONE; tick(t0 + 2400); assert(count("\"fl\"") == 2);
    tx_ready = 1; tick(t0 + 2420); assert(count("\"fl\"") == 3 && strstr(last(), "{\"fl\":{\"k\":0,\"m\":0,\"p\":null,\"n\":null}}"));
    printf("%s", frames[0]);

    /* largest summary */
    fla.epoch = 65535; fla.q = 4294967294u; for (int k = 0; k <= FL_CAUSES; k++) fla.ms[k] = 4294967295u;
    fla.mm = 4294967295u; fla.um = 999; tx_ready = 1; assert(_fl_send_fs());
    assert(strlen(last()) + 1 <= 248); printf("%s", last());
    return 0;
}
'''
        out = self.run_c(code, "push")
        frames = [json.loads(l) for l in out.splitlines()]                 # every frame printed is valid JSON
        self.assertEqual(frames[0]["fs"]["cd"], 1.001)
        self.assertEqual(frames[1]["fl"], {"k": 1, "m": 1, "p": 67, "n": 12})
        self.assertEqual(frames[2]["fs"]["e"], 65535)

    def test_frame_sender(self):
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define NUL 0
#define TX_BUFFER_SIZE 250
#define FC_IN_XOFF 1
#define FLOW_CONTROL_RTS 2
#define USB_CTS_bm 0x10
typedef uint8_t buffer_t;
struct { uint8_t IN; } port;
struct { buffer_t tx_buf_head, tx_buf_tail; uint8_t fc_state_tx; typeof(port) *port; } USBu = { 1, 1, 0, &port };
struct { int enable_flow_control; } xio;
static char out[512]; static int n;
static int _putc_usb_raw(const char c) { out[n++] = c; return 0; }
''' + function(src("xio/xio_usb.c"), r"uint8_t xio_usb_tx_ready\(void\)") + "\n" + \
            function(src("xio/xio_usb.c"), r"uint8_t xio_usb_try_send_frame\(const char \*line\)") + r'''
int main(void) {
    char line[260];
    memset(line, 'a', 246); line[246] = '\n'; line[247] = 0;          /* 247 bytes, needs 248 */
    assert(xio_usb_try_send_frame(line) && n == 247);
    memset(line, 'a', 247); line[247] = '\n'; line[248] = 0;          /* needs 249 */
    n = 0; assert(!xio_usb_try_send_frame(line) && n == 0);
    USBu.tx_buf_head = 7; assert(!xio_usb_try_send_frame("x\n")); USBu.tx_buf_head = 1;
    USBu.fc_state_tx = FC_IN_XOFF; assert(!xio_usb_try_send_frame("x\n")); USBu.fc_state_tx = 0;
    xio.enable_flow_control = FLOW_CONTROL_RTS; port.IN = USB_CTS_bm; assert(!xio_usb_try_send_frame("x\n") && n == 0);
    port.IN = 0; assert(xio_usb_try_send_frame("x\n") && n == 2);
    return 0;
}
'''
        self.run_c(code, "frame")

    def test_wiring(self):
        """Retire after the busy check and before the no-next-move return; install on an ALINE load; reset
        clears both records; the exec writes a record only while on; fi does not clear qt; fi and fs are
        rt leaves; the callback is dispatched."""
        st = src("stepper.c")
        load = function(st, r"static void _load_move\(\)")
        busy, retire, nomove = load.index("st_runtime_isbusy()"), load.index("_fl_retire();"), load.index("PREP_BUFFER_OWNED_BY_LOADER")
        self.assertTrue(busy < retire < nomove)
        self.assertGreater(load.index("st_run.fl_rec = st_pre.fl_rec"), load.index("st_pre.block_type == BLOCK_TYPE_ALINE"))
        reset = function(st, r"void stepper_reset\(\)")
        self.assertIn("st_pre.fl_rec = 0;", reset)
        self.assertIn("st_run.fl_rec = 0;", reset)
        self.assertLess(reset.index("st_run.fl_rec = 0;"), reset.index("SREG = saved_sreg;"))
        seg = function(src("plan_exec.c"), r"static stat_t _exec_aline_segment\(\)")
        self.assertRegex(seg, r"if \(fl_on\) \{ fl_segment\(mr\.segment_velocity, mr\.segment_time\); \} else \{ st_pre\.fl_rec = 0; \}\s*ritorno\(st_prep_line")
        self.assertIn("(set != rt_set_fi)", function(src("config.c"), r"stat_t nv_set\(nvObj_t \*nv\)"))
        self.assertIn('{ "rt","rtfi"', src("config_app.c"))
        self.assertIn('{ "rt","rtfs"', src("config_app.c"))
        self.assertIn("DISPATCH(rt_feedlimit_callback());", src("controller.c"))
        scan = src("json_parser.c")
        self.assertIn("(key[0] == 'f') && (key[1] == 'i')) ? 128", scan)
        self.assertIn("(key[0] == 'f') && (key[1] == 's')) ? 256", scan)


if __name__ == "__main__":
    unittest.main()
