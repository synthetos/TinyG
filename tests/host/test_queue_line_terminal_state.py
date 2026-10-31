"""Exercise the real queue helper with controlled planner/interrupt outcomes.

This checks state preservation, not interrupt latency. The pre-cli injection
models a pending halt accepted after the unlocked terminal-state check, including
the halt handler clearing the planner and cycle. It is not a timing measurement.
"""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(os.environ.get("TINYG_SOURCE", Path(__file__).resolve().parents[2]))


class QueueLineTerminalState(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        source = (ROOT / "firmware/tinyg/canonical_machine.c").read_text()
        code = r'''
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#define __AVR 1
#define STAT_OK 0
#define STAT_MINIMUM_LENGTH_MOVE 1
#define STAT_MINIMUM_TIME_MOVE 2
#define MACHINE_CYCLE 1
#define MACHINE_PROGRAM_STOP 2
#define MACHINE_ALARM 3
#define MACHINE_SHUTDOWN 4
#define MACHINE_PANIC 5
#define CYCLE_OFF 0
#define CYCLE_MACHINING 1
typedef int stat_t;
struct {int gm, machine_state, cycle_state;} cm;
uint8_t SREG;
int injection, terminal, has_buffer, starts, ends;
void fault(void) {
    cm.machine_state=terminal;
    // Shutdown/panic halt the cycle; alarm requests a controlled stop.
    cm.cycle_state=(terminal==MACHINE_ALARM) ? CYCLE_MACHINING : CYCLE_OFF;
}
void fake_cli(void) {
    if (injection==1 && (SREG & 0x80)) {
        fault();has_buffer=0;injection=-1; // a real halt discards queued motion
    }
    SREG &= ~0x80;
}
#define cli() fake_cli()
void cm_cycle_start(void) {
    starts++;cm.machine_state=MACHINE_CYCLE;
    if(cm.cycle_state==CYCLE_OFF) cm.cycle_state=CYCLE_MACHINING;
}
void cm_cycle_end(void) {
    ends++;
    if(cm.cycle_state!=CYCLE_OFF) {
        cm.machine_state=MACHINE_PROGRAM_STOP;cm.cycle_state=CYCLE_OFF;
    }
}
stat_t cm_is_alarmed(void) {return cm.machine_state>=MACHINE_ALARM ? 200 : STAT_OK;}
bool mp_has_runnable_buffer(void) {return has_buffer;}
stat_t mp_aline(void *gm) {
    (void)gm;
    if(injection==0) {fault();return 250;}
    return STAT_OK;
}
'''
        code += function(source, r'static stat_t _queue_line\(\)(?=\s*\{)')
        code += r'''
int main(int argc,char **argv) {
    if(argc!=5) return 2;
    injection=atoi(argv[1]);terminal=atoi(argv[2]);has_buffer=atoi(argv[3]);SREG=atoi(argv[4]);
    cm.machine_state=MACHINE_PROGRAM_STOP;cm.cycle_state=CYCLE_OFF;
    int status=_queue_line();
    printf("%d %d %d %d %u\n",status,cm.machine_state,starts,ends,(unsigned)SREG);
    return 0;
}
'''
        path = Path(cls.tmp.name) / "queue.c"
        path.write_text(code)
        cls.exe = Path(cls.tmp.name) / "queue"
        subprocess.run(["cc", "-std=c99", "-Wall", "-Werror",
                        "-fsanitize=address,undefined", str(path), "-o", str(cls.exe)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_case(self, stage, terminal, queued, sreg=128):
        out = subprocess.check_output([str(self.exe), str(stage), str(terminal),
                                       str(queued), str(sreg)], text=True)
        return tuple(map(int, out.split()))

    def test_terminal_state_entered_by_planner(self):
        for terminal in (3, 4, 5):
            for queued in (0, 1):
                for sreg in (0, 128):
                    with self.subTest(terminal=terminal, queued=queued, sreg=sreg):
                        self.assertEqual(self.run_case(0, terminal, queued, sreg),
                                         (250, terminal, 1, 0, sreg))

    def test_halt_before_cli_clears_queue_and_preserves_terminal_state(self):
        for terminal in (4, 5):
            for queued in (0, 1):
                with self.subTest(terminal=terminal, queued=queued):
                    status, state, starts, ends, sreg = self.run_case(1, terminal, queued)
                    self.assertEqual(state, terminal,
                                     f"terminal state overwritten: starts={starts}, ends={ends}")
                    self.assertEqual(sreg, 128)

    def test_ordinary_cycle_repair_and_interrupt_restoration(self):
        for queued in (0, 1):
            for sreg in (0, 128):
                with self.subTest(queued=queued, sreg=sreg):
                    expected = (0, 1, 2, 0, sreg) if queued else (0, 2, 1, 1, sreg)
                    self.assertEqual(self.run_case(2, 0, queued, sreg), expected)


if __name__ == "__main__":
    unittest.main()
