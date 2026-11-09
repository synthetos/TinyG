"""N83 (#298): a G10 at rest listens before it saves, so a line behind it is not lost to the EEPROM write.

Each persisted value leaves only high-level interrupts on (AVR1008 workaround, xmega_eeprom.c) and serial RX is medium
level, so a byte that arrives during a write is lost and the line it belongs to raises ALARM 13. A value blocks about
30 ms (7.5 ms a byte), three values about 90 ms, measured on a board.

A host waiting for the reply is silent until it comes, the one time a write cannot lose input, and N01 needs that: a
later write can meet its next line. A host streaming ahead of the replies has the next line in the RX ring or on the
wire. An empty ring cannot tell them apart, so the G10 listens for G10_HOST_WAIT_MS. Silence: save, then reply. A byte:
reply at once and leave the save to cm_deferred_write_callback(), which waits for G10_QUIET_MS without a received byte.

That wait alone never ends for a host polling faster than G10_QUIET_MS, and the offset would be lost at power off with
nothing to show for it. After G10_SAVE_WAIT_MS at rest the save starts just after a byte, inside G10_QUIET_SLOT_MS of
the G10_QUIET_MIN_MS point. Any later in the gap and the next poll lands in the write, which a board showed at 100 ms.
The arc, homing, probe and jog callbacks return EAGAIN ahead of that callback, so a whole cycle can pass without it
running, and short jogs one after another leave it seeing only "ready". A cycle can also pass while the main loop waits
on a full TX ring, or a short line can run inside the exec interrupt during a reply, so no look from the main loop can
prove the rest was continuous. mp_exec_move() sets mb.exec_ran as blocks run, and the callback restarts the patience
when it finds that set. It clears it only with the machine at rest, with the check and the clear under one interrupt
guard: a dwell sets it once, as it starts, so a look during the dwell followed by a main loop stuck past its end must
not use that up. A count would alias: mp_exec_move() runs per segment, so a byte wraps in about 1.3 s.

Real functions from canonical_machine.c against stub serial, clock and NV layers.
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / "firmware/tinyg"

PRELUDE = r'''
#include <assert.h>
#include <stdlib.h>
#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
#include "planner.h"
#include "util.h"
cmSingleton_t cm;
bool FLAGS_NONE[AXES];
static int saves, rx, tx, seen, arrive_in = -1;
static bool listening;
static uint32_t now, last_rx;
mpBufferPool_t mb;
static int USBu;
uint16_t xio_get_rx_bufcount_usart(void *p) { return rx; }
uint16_t xio_get_tx_bufcount_usart(void *p) { return tx; }
uint8_t xio_usb_take_rx_seen(void) { int s = seen; seen = 0; return s; }
uint32_t SysTickTimer_getValue(void) {                   /* in the setter time only passes by reading the clock */
    if (listening) { now++; if ((arrive_in >= 0) && (arrive_in-- == 0)) { seen = 1; } }
    return now;
}
bool st_aline_in_flight(void) { return false; }
static bool runtime_idle = true;
bool mp_runtime_is_idle(void) { return runtime_idle; }
void cm_set_work_offsets(void) {}
float cm_get_active_coord_offset(uint8_t axis) { return 0; }
void mp_queue_command(void(*fn)(float[],bool[]),float *v,bool *f) {}
void mp_set_runtime_work_offset(float *v) {}
static void _exec_g10_offset(float *,bool *);
index_t nv_get_index(const char *group, const char *token) { return (token[2]-'4')*6 + (strchr("xyzabc",token[3])-"xyzabc"); }
stat_t nv_persist(nvObj_t *nv) { saves++; return STAT_OK; }
#define NUL 0
static unsigned char SREG;
#undef cli
#define cli()
'''

EXERCISE = r'''
static float v[AXES] = {1,2,3};
static bool flags[AXES] = {true,true,true};

static void prepare(void) {
    memset(&cm, 0, sizeof(cm)); saves = rx = tx = seen = 0; arrive_in = -1; now += 100000;
    cm.gf.parameter = cm.gf.L_word = true; cm.gm.coord_system = G54;
    cm_deferred_write_callback();                        /* the main loop runs it all the time */
}
static void run_for(uint32_t ms) { for (uint32_t i = 0; i < ms; i++) { now++; cm_deferred_write_callback(); } }
static void skipped(uint32_t ms) {                       /* a cycle the callback never sees: only the block latch shows it */
    now += ms; mb.exec_ran = true;
}
static void segments(int count) { for (int i = 0; i < count; i++) { mb.exec_ran = true; } }   /* one per mp_exec_move() call */
static void g10(int arrive) {                            /* the G10 line came in over the wire; a byte follows or not */
    seen = 1; cm_deferred_write_callback();
    arrive_in = arrive; listening = true;
    assert(cm_set_coord_offsets(1,2,v,flags) == STAT_OK);
    listening = false; arrive_in = -1;
}

int main(void) {
    /* a host waiting for the reply: saved before the setter returns, so before the reply */
    prepare();
    uint32_t start = now;
    g10(-1);
    assert(saves == 36 && !cm.deferred_write_flag);
    assert(now - start >= G10_HOST_WAIT_MS);

    /* the next line is in the ring already: no write and no wait */
    prepare();
    rx = 1; start = now;
    g10(-1);
    assert(saves == 0 && cm.deferred_write_flag);
    assert(now - start < 5);
    rx = 0;

    /* the next line arrives while the G10 listens, with the ring empty when it started */
    prepare();
    g10(G10_HOST_WAIT_MS - 10);
    assert(saves == 0 && cm.deferred_write_flag);

    /* nothing is written until the wire has been quiet */
    run_for(G10_QUIET_MS - 1);
    assert(saves == 0 && cm.deferred_write_flag);
    run_for(1);
    assert(saves == 36 && !cm.deferred_write_flag);

    /* a byte late in the wait starts it again */
    prepare();
    g10(5);
    run_for(G10_QUIET_MS - 5);
    seen = 1; cm_deferred_write_callback();
    run_for(G10_QUIET_MS - 1);
    assert(saves == 0);
    run_for(1);
    assert(saves == 36);

    /* quiet does not end the wait for the serial rings or for a cycle */
    prepare();
    g10(5);
    rx = 1; run_for(G10_QUIET_MS + 1); assert(saves == 0);
    rx = 0; tx = 1; run_for(G10_QUIET_MS + 1); assert(saves == 0);
    tx = 0; cm.cycle_state = CYCLE_MACHINING; run_for(G10_QUIET_MS + 1); assert(saves == 0);
    cm.cycle_state = CYCLE_OFF; run_for(1); assert(saves == 36);

    /* a host polling every 100 ms: the save waits, then starts just after a poll whatever the phase */
    for (int phase = 0; phase < 100; phase += 7) {
        prepare();
        start = now;
        g10(5);
        uint32_t last_poll = now, saved_at = 0;
        for (int ms = 1; ms <= 20000 && !saved_at; ms++) {
            now++;
            if ((ms + phase) % 100 == 0) { seen = 1; last_poll = now; }
            cm_deferred_write_callback();
            if (saves) { saved_at = now; }
        }
        assert(saved_at && saves == 36);
        assert(saved_at - start >= G10_SAVE_WAIT_MS && saved_at - start < G10_SAVE_WAIT_MS + 200);
        assert(saved_at - last_poll >= G10_QUIET_MIN_MS);
        assert(saved_at - last_poll < G10_QUIET_MIN_MS + G10_QUIET_SLOT_MS);
    }

    /* lines 10 ms apart never leave that gap, however long they go on */
    prepare();
    g10(5);
    for (int ms = 1; ms <= 60000; ms++) {
        now++;
        if (ms % 10 == 0) { seen = 1; }
        cm_deferred_write_callback();
    }
    assert(saves == 0 && cm.deferred_write_flag);

    /* time in a cycle does not count: the patience starts at rest */
    prepare();
    g10(5);
    cm.cycle_state = CYCLE_MACHINING; run_for(G10_SAVE_WAIT_MS * 2); assert(saves == 0);
    cm.cycle_state = CYCLE_OFF; seen = 1; cm_deferred_write_callback();
    run_for(G10_QUIET_MS - 1);
    assert(saves == 0);
    run_for(1);
    assert(saves == 36);

    /* an arc, homing, probe or jog cycle returns EAGAIN ahead of this callback, which then does not run at all */
    prepare();
    g10(5);
    skipped(G10_SAVE_WAIT_MS * 2);
    seen = 1;
    for (int ms = 1; ms <= G10_SAVE_WAIT_MS - 200; ms++) {
        now++;
        if (ms % 100 == 0) { seen = 1; }
        cm_deferred_write_callback();
        assert(saves == 0);
    }
    run_for(G10_QUIET_MS);
    assert(saves == 36);

    /* a whole busy interval between two looks, the board at rest at both: a full TX ring, or a line run by the exec */
    prepare();
    g10(5);
    for (int ms = 1; ms <= G10_SAVE_WAIT_MS / 2; ms++) {
        now++;
        if (ms % 100 == 0) { seen = 1; }
        cm_deferred_write_callback();
    }
    assert(saves == 0);
    skipped(G10_SAVE_WAIT_MS);
    seen = 1;
    for (int ms = 1; ms <= G10_SAVE_WAIT_MS - 200; ms++) {
        now++;
        if (ms % 100 == 0) { seen = 1; }
        cm_deferred_write_callback();
        assert(saves == 0);
    }

    /* a dwell counts once. The callback looks during it, then the main loop is stuck on TX past its end */
    prepare();
    g10(5);
    for (int ms = 1; ms <= G10_SAVE_WAIT_MS / 2; ms++) {
        now++;
        if (ms % 100 == 0) { seen = 1; }
        cm_deferred_write_callback();
    }
    assert(saves == 0);
    mb.exec_ran = true; runtime_idle = false;                /* the dwell block runs once */
    run_for(100);
    now += G10_SAVE_WAIT_MS; runtime_idle = true;        /* no calls until after it ended */
    seen = 1;
    for (int ms = 1; ms <= G10_SAVE_WAIT_MS - 200; ms++) {
        now++;
        if (ms % 100 == 0) { seen = 1; }
        cm_deferred_write_callback();
        assert(saves == 0);
    }
    run_for(G10_QUIET_MS);
    assert(saves == 36);

    /* a long line while TX blocks: exactly 256 segments between two looks at rest, about 1.3 s */
    prepare();
    g10(5);
    for (int ms = 1; ms <= G10_SAVE_WAIT_MS - 1000; ms++) {
        now++;
        if (ms % 100 == 0) { seen = 1; }
        cm_deferred_write_callback();
    }
    assert(saves == 0);
    segments(256); now += 1280;
    seen = 1;
    for (int ms = 1; ms <= G10_SAVE_WAIT_MS - 200; ms++) {
        now++;
        if (ms % 100 == 0) { seen = 1; }
        cm_deferred_write_callback();
        assert(saves == 0);
    }

    /* short jogs one after another, polled: the callback sees only "ready", before and after each one */
    prepare();
    g10(5);
    for (int jog = 0; jog < (3 * G10_SAVE_WAIT_MS) / 150; jog++) {
        skipped(100);
        seen = 1;
        for (int ms = 0; ms < 50; ms++) {
            now++;
            cm_deferred_write_callback();
            assert(saves == 0);
        }
    }

    /* a tick count that wraps holds neither wait back for good */
    prepare(); now = 0xFFFFFFE0UL; start = now; cm_deferred_write_callback();
    g10(-1);
    assert(saves == 36);
    prepare(); now = 0xFFFFFFF0UL; cm_deferred_write_callback();
    g10(5);
    run_for(G10_QUIET_MS);
    assert(saves == 36);
    return 0;
}
'''


class G10WaitsForQuiet(unittest.TestCase):
    def test_write_waits_for_a_quiet_wire(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        source = (FW / "canonical_machine.c").read_text()
        code = PRELUDE + function((FW / "xio.c").read_text(), r"uint8_t xio_isbusy\(\)")
        code += function(source, r"bool cm_persistence_busy\(\)\s*(?=\{)")
        for name in ("_host_awaits_reply", "_persist_coord_offsets", "cm_deferred_write_callback", "cm_set_coord_offsets",
                     "_exec_g10_offset"):
            sig = r"(?:static )?(?:void|bool|stat_t) " + name + r"\([^;{}]*\)\s*(?=\{)"
            if re.search(sig, source):
                code += function(source, sig)
        code += EXERCISE
        with tempfile.TemporaryDirectory(prefix="tinyg-n83-") as tmp:
            tmp = Path(tmp)
            (tmp / "avr").mkdir()
            (tmp / "avr/pgmspace.h").write_text("#define PROGMEM\n#define PSTR(s) (s)\n")
            c, exe = tmp / "n83.c", tmp / "n83"
            c.write_text(code)
            subprocess.run([cc, "-std=gnu99", "-O1", "-g", "-fsanitize=address,undefined", "-fstrict-flex-arrays=3",
                            "-D_POSIX_C_SOURCE=200809L", "-w", "-I", str(FW), "-I", str(tmp),
                            str(c), "-lm", "-o", str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


    def test_every_block_latches_that_it_ran(self):
        move = function((FW / "plan_exec.c").read_text(), r"stat_t mp_exec_move\(\)")
        for target in ('mp_exec_aline', 'mp_exec_dwell', 'mp_exec_command'):
            call = move.index('return ' + target + '(bf)')
            self.assertIn('mb.exec_ran = true', move[move.rfind('case ', 0, call):call])


if __name__ == "__main__":
    unittest.main()
