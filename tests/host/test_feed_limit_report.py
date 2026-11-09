"""Feed-limit report leaves (runtime.c rt_get_flk/flm/flp/flf) and the slot hand-off that feeds them
(feedlimit.c fl_stage_block, stepper.c st_prep_line / _load_move).

- One snapshot per line: a segment load between two leaves of one reply must not mix moves; the next line
  sees the new move.
- Stopped, held or no current slot reads no move. flf is null for a rapid and for G93 (its F is not mm/min);
  flp is null unless the move is a G94 linear XYZ feed move.
- flm adds the profile tags (too short to reach cruise, single segment) derived from the staged plan.
- The exec's rotary check takes +0 and -0 as an axis not moving and any other value as moving.
- Hand-off: the exec stages each block in the slot that is not current; st_prep_line() passes the staged
  slot to the loader with the block's first segment only; the loader publishes the slot index and clears
  the marker. Staging never touches the current slot; a block staged but never prepared has its slot reused;
  a dwell or command makes no move current; back-to-back single-segment blocks each become current in turn.
"""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FW = Path(os.environ.get("TINYG_FIRMWARE", ROOT / "firmware/tinyg"))

PRELUDE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define __AVR
typedef uint8_t stat_t;
#define STAT_OK 0
#define STAT_UNRECOGNIZED_NAME 100
enum { TYPE_NULL = 0, TYPE_INTEGER = 2, TYPE_FLOAT = 4 };
enum { MOTION_STOP = 0, MOTION_RUN, MOTION_HOLD };
enum { FEEDHOLD_OFF = 0, FEEDHOLD_HOLD = 4 };
enum { MOTION_MODE_STRAIGHT_TRAVERSE = 0, MOTION_MODE_STRAIGHT_FEED = 1 };
enum { INVERSE_TIME_MODE = 0, UNITS_PER_MINUTE_MODE = 1 };
enum { AXIS_X, AXIS_Y, AXIS_Z, AXIS_A, AXIS_B, AXIS_C, AXES };
#define NOM_SEGMENT_TIME (5000.0f / 60000000)
typedef struct nvObj { uint8_t valuetype; union { float value_flt; uint32_t value_int; }; } nvObj_t;
typedef struct { uint8_t motion_mode, feed_rate_mode; float feed_rate; uint32_t linenum; } GCodeState_t;
typedef struct mpBuffer { float unit[AXES]; float cruise_velocity, cruise_vmax, naiive_move_time;
                          GCodeState_t gm; uint8_t fl_tags, fl_cls; } mpBuf_t;
uint8_t SREG = 0x80;
static void cli(void) { SREG &= ~0x80; }
struct { int motion_state, hold_state; } cm;
struct { bool allowed, fl_taken; } rt;
struct { uint8_t fl_load; } st_pre;
'''


def src(path):
    return (FW / path).read_text(encoding="latin-1")


def block(source, opener):
    """The braced statement starting at `opener` (an `if (...) {` line), through its closing brace."""
    i = source.index(opener)
    depth, j = 0, source.index("{", i)
    while True:
        depth += {"{": 1, "}": -1}.get(source[j], 0)
        if depth == 0:
            return source[i:j + 1]
        j += 1


def firmware_code():
    h, fc, rtc, st = src("feedlimit.h"), src("feedlimit.c"), src("runtime.c"), src("stepper.c")
    code = PRELUDE + "\n".join(l for l in h.splitlines() if l.startswith("#define FL_")) + "\n"
    code += h[h.index("typedef struct flBlock"):h.index("} flBlock_t;") + len("} flBlock_t;")] + "\n"
    i = h.index("static inline uint8_t fl_axis_moves(float unit)")
    code += h[i:h.index("\n}", i) + 2] + "\n"
    code += h[h.index("typedef struct flPending"):h.index("} flPending_t;") + len("} flPending_t;")] + "\n"
    last = "static uint8_t fl_blk_cls;"
    code += fc[fc.index("flBlock_t fl_slot[2];"):fc.index(last) + len(last)] + "\n"
    code += function(fc, r"uint8_t fl_profile_tags\(float naiive_move_time, float cruise, float cruise_vmax\)") + "\n"
    code += function(fc, r"void fl_stage_block\(const mpBuf_t \*bf\)") + "\n"
    code += "static flBlock_t fl_snap;\n"
    code += function(rtc, r"static void _fl_current\(flBlock_t \*b\)") + "\n"
    for leaf in ("flk", "flm", "flp", "flf"):
        code += function(rtc, r"stat_t rt_get_" + leaf + r"\(nvObj_t \*nv\)") + "\n"
    prep = function(st, r"stat_t st_prep_line\(float travel_steps\[\], float following_error\[\], float segment_time\)")
    load = function(st, r"static void _load_move\(\)")
    code += "static void prep_segment(void) {\n" + block(prep, "if (fl_staged != FL_LOAD_SAME) {") + "\n}\n"
    code += "static void load_segment(void) {\n" + block(load, "if (st_pre.fl_load != FL_LOAD_SAME) {") + "\n}\n"
    code += "static void prep_dwell(void) { st_pre.fl_load = FL_LOAD_NONE; }\n"   # as st_prep_dwell (test_wiring)
    return code


class FeedLimitReport(unittest.TestCase):
    def run_c(self, code, name):
        with tempfile.TemporaryDirectory() as tmp:
            c, exe = Path(tmp) / (name + ".c"), Path(tmp) / name
            c.write_text(code)
            subprocess.run(["cc", "-std=gnu99", "-O1", "-Wall", "-Werror", "-Wno-unused-function",
                            "-fsanitize=address,undefined","-fstrict-flex-arrays=3", str(c), "-o", str(exe)], check=True)
            r = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)

    def test_report_leaves(self):
        code = firmware_code() + r'''
static nvObj_t get(stat_t (*f)(nvObj_t *)) { nvObj_t nv = {0}; assert(f(&nv) == STAT_OK); return nv; }
static void show(flBlock_t b) { fl_slot[0] = b; fl_cur = 0; rt.fl_taken = false; }   /* a new line, b current */
static const flBlock_t A = { FL_AXIS_CAP, FL_KIND_FEED, 1, 1, 400, 600, 400, 0.01f };   /* reaches its capped cruise */
static const flBlock_t B = { 0, FL_KIND_FEED, 1, 1, 300, 1200, 1200, 0.001f };           /* too short to reach F */
static const flBlock_t T = { 0, FL_KIND_FEED, 1, 1, 600, 600, 600, 0.00005f };           /* one 3 ms segment */
int main(void) {
    rt.allowed = true; cm.motion_state = MOTION_RUN; cm.hold_state = FEEDHOLD_OFF;
    show(A);
    assert(get(rt_get_flm).value_int == FL_AXIS_CAP);
    fl_slot[1] = B; fl_cur = 1;                                       /* a segment load between two leaves */
    assert(get(rt_get_flp).value_int == 67 && get(rt_get_flf).value_flt == 600);   /* still move A */
    rt.fl_taken = false;                                              /* the next line */
    assert(get(rt_get_flm).value_int == FL_SHORT && get(rt_get_flf).value_flt == 1200 && get(rt_get_flp).value_int == 25);
    show(T);
    assert(get(rt_get_flm).value_int == FL_MIN_SEGMENT);
    flBlock_t a = A; a.cruise = 396.5f; show(a);                      /* within 1% of the capped cruise */
    assert(get(rt_get_flm).value_int == FL_AXIS_CAP);
    a.cruise = 395; show(a);
    assert(get(rt_get_flm).value_int == (FL_AXIS_CAP | FL_SHORT));

    show(A); cm.hold_state = FEEDHOLD_HOLD; rt.fl_taken = false;      /* held: no move */
    assert(get(rt_get_flk).value_int == FL_KIND_NONE && get(rt_get_flm).value_int == 0);
    assert(get(rt_get_flp).valuetype == TYPE_NULL && get(rt_get_flf).valuetype == TYPE_NULL);
    cm.hold_state = FEEDHOLD_OFF; cm.motion_state = MOTION_STOP; rt.fl_taken = false;
    assert(get(rt_get_flk).value_int == FL_KIND_NONE);
    cm.motion_state = MOTION_RUN; fl_cur = FL_CUR_NONE; rt.fl_taken = false;   /* a dwell is current */
    assert(get(rt_get_flk).value_int == FL_KIND_NONE && get(rt_get_flm).value_int == 0);

    show((flBlock_t){ 0, FL_KIND_RAPID, 0, 0, 1000, 600, 1000, 0.01f });              /* G0 */
    assert(get(rt_get_flk).value_int == FL_KIND_RAPID && get(rt_get_flp).valuetype == TYPE_NULL && get(rt_get_flf).valuetype == TYPE_NULL);
    show((flBlock_t){ 0, FL_KIND_FEED, 0, 0, 100, 0.5f, 100, 0.01f });                /* G93: F is inverse time */
    assert(get(rt_get_flk).value_int == FL_KIND_FEED && get(rt_get_flp).valuetype == TYPE_NULL && get(rt_get_flf).valuetype == TYPE_NULL);
    show((flBlock_t){ 0, FL_KIND_FEED, 1, 0, 100, 600, 100, 0.01f });                 /* G94 with a rotary axis */
    assert(get(rt_get_flp).valuetype == TYPE_NULL && get(rt_get_flf).value_flt == 600);

    assert(!fl_axis_moves(0.0f) && !fl_axis_moves(-0.0f));                         /* the exec's rotary check */
    assert(fl_axis_moves(1e-30f) && fl_axis_moves(-0.5f) && fl_axis_moves(1.0f));

    rt.allowed = false; nvObj_t nv = {0}; assert(rt_get_flk(&nv) == STAT_UNRECOGNIZED_NAME);
    return 0;
}
'''
        self.run_c(code, "report")

    def test_slot_hand_off(self):
        code = firmware_code() + r'''
static mpBuf_t blk(float feed) {                                  /* a G94 XYZ feed block, told apart by F */
    mpBuf_t b; memset(&b, 0, sizeof(b));
    b.gm.motion_mode = MOTION_MODE_STRAIGHT_FEED; b.gm.feed_rate_mode = UNITS_PER_MINUTE_MODE;
    b.gm.feed_rate = feed; b.cruise_velocity = b.cruise_vmax = feed; b.naiive_move_time = 0.01f; b.unit[AXIS_X] = 1;
    return b;
}
static float current_feed(void) { assert(fl_cur < 2); return fl_slot[fl_cur].feed; }
int main(void) {
    assert(fl_cur == FL_CUR_NONE);
    mpBuf_t b1 = blk(100), b2 = blk(200), b3 = blk(300), b4 = blk(400), b5 = blk(500);

    fl_stage_block(&b1); prep_segment(); load_segment();           /* first segment of b1 */
    assert(current_feed() == 100 && st_pre.fl_load == FL_LOAD_SAME);
    prep_segment(); assert(st_pre.fl_load == FL_LOAD_SAME);         /* later segments carry no marker */
    load_segment(); assert(current_feed() == 100);

    flBlock_t before = fl_slot[fl_cur];
    fl_stage_block(&b2);                                            /* staging fills the other slot */
    assert(memcmp(&before, &fl_slot[fl_cur], sizeof(before)) == 0 && current_feed() == 100);
    prep_segment(); assert(current_feed() == 100);                  /* prepared, not yet loaded */
    load_segment(); assert(current_feed() == 200);

    fl_stage_block(&b3);                                            /* staged; its first prep fails (not called) */
    uint8_t b2_slot = fl_cur;
    fl_stage_block(&b4);                                            /* b3's slot is reused, b2's left alone */
    assert(fl_cur == b2_slot && current_feed() == 200);
    prep_segment(); load_segment(); assert(current_feed() == 400);

    prep_dwell(); load_segment();                                   /* a dwell: no move current */
    assert(fl_cur == FL_CUR_NONE && st_pre.fl_load == FL_LOAD_SAME);
    fl_stage_block(&b5); prep_segment(); load_segment(); assert(current_feed() == 500);

    for (int i = 0; i < 6; i++) {                                   /* back-to-back single-segment blocks */
        mpBuf_t b = blk(1000 + i);
        flBlock_t cur = fl_slot[fl_cur];
        fl_stage_block(&b);
        assert(memcmp(&cur, &fl_slot[fl_cur], sizeof(cur)) == 0);
        prep_segment(); load_segment();
        assert(current_feed() == 1000 + i);
    }
    mpBuf_t r = blk(0); r.gm.motion_mode = MOTION_MODE_STRAIGHT_TRAVERSE; r.unit[AXIS_A] = -0.0f;
    fl_stage_block(&r); prep_segment(); load_segment();
    assert(fl_slot[fl_cur].kind == FL_KIND_RAPID && !fl_slot[fl_cur].feed_ok && !fl_slot[fl_cur].pct_ok);
    mpBuf_t g = blk(600); g.unit[AXIS_C] = 0.2f;
    fl_stage_block(&g); prep_segment(); load_segment();
    assert(fl_slot[fl_cur].feed_ok && !fl_slot[fl_cur].pct_ok);
    return 0;
}
'''
        self.run_c(code, "handoff")

    def test_wiring(self):
        """The exec stages at block start; command and dwell preps clear the report; the null prep, which
        the loader never takes, leaves the marker alone; the marker moves before ownership does."""
        st, ex = src("stepper.c"), src("plan_exec.c")
        self.assertIn("fl_stage_block(bf);", function(ex, r"stat_t mp_exec_aline\(mpBuf_t \*bf\)"))
        for fn in (r"void st_prep_command\(void \*bf\)", r"void st_prep_dwell\(float microseconds\)"):
            self.assertIn("st_pre.fl_load = FL_LOAD_NONE;", function(st, fn), fn)
        self.assertNotIn("fl_load", function(st, r"void st_prep_null\(\)"))
        prep = function(st, r"stat_t st_prep_line\(float travel_steps\[\], float following_error\[\], float segment_time\)")
        tail = prep[prep.rindex("st_pre.block_type = BLOCK_TYPE_ALINE;"):]
        self.assertLess(tail.index("fl_staged = FL_LOAD_SAME;"), tail.index("st_pre.buffer_state = PREP_BUFFER_OWNED_BY_LOADER;"))
        load = function(st, r"static void _load_move\(\)")
        self.assertLess(load.index("PREP_BUFFER_OWNED_BY_LOADER"), load.index("fl_cur = st_pre.fl_load - 1;"))


if __name__ == "__main__":
    unittest.main()
