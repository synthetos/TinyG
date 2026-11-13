"""Exercise actual spindle callbacks with GPIO and planner boundary doubles.

These are logical output-level checks, not startup-glitch or waveform measurements.
"""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = Path(os.environ.get("TINYG_FIRMWARE", ROOT / "firmware/tinyg"))


class SpindlePolarity(unittest.TestCase):
    def test_control_and_setting_interlock(self):
        source = (FIRMWARE / "spindle.c").read_text()
        names = ["spindle_config_init", "spindle_init_outputs", "spindle_set_polarity", "spindle_reset", "_exec_spindle_control"]
        definitions = [function((FIRMWARE / "canonical_machine.c").read_text(),
                                r"bool cm_outputs_inhibited\(\)(?=\s*\{)")]
        for name in names:
            signature = r"(?m)^(?:static )?(?:void|stat_t) " + name + r"\([^;{}\n]*\)\s*"
            match = re.search(signature + r"\{", source)
            self.assertIsNotNone(match, name)
            definitions.append(function(source[match.start():], signature))
        if "} spindle_out;" in source:                                  # outputs as driven, for the readouts
            record = re.search(r"(?m)^static void _record_speed\(float speed\)\s*", source)
            definitions.insert(1, source[source.index("static struct {"):source.index("} spindle_out;") + 14] + "\n" +
                               function(source[record.start():], r"static void _record_speed\(float speed\)\s*"))
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define __AVR 1
#define MACHINE_ALARM 10
#define MACHINE_SHUTDOWN 11
#define MACHINE_PANIC 12
#define FLUSH_OFF 0
#define SPINDLE_OFF 0
#define SPINDLE_CW 1
#define SPINDLE_CCW 2
#define SPINDLE_BIT 8
#define SPINDLE_DIR 4
#define SPINDLE_ENABLE_POLARITY 1
#define SPINDLE_DIR_POLARITY 0
#define MACHINE_INITIALIZING 0
#define PREP_BUFFER_OWNED_BY_LOADER 1
#define MODEL 0
#define PWM_1 0
#define FLAGS_NONE ((bool *)0)
#define STAT_OK 0
#define STAT_INPUT_VALUE_RANGE_ERROR 108
#define STAT_COMMAND_NOT_ACCEPTED 106
#define cli() (SREG &= ~128)
typedef int stat_t;
typedef struct { uint32_t value_int; uint8_t *target; } nvObj_t;
struct { uint8_t enable_polarity, direction_polarity; } spindle_cfg;
struct { int machine_state,queue_flush_state; bool job_kill_requested,limit_requested,shutdown_requested; struct { int spindle_mode; float spindle_speed; } gm; } cm;
struct { int buffer_state; } st_pre;
uint8_t SREG;
static bool queued, running, spindle_configured;
static unsigned pins, writes, pwm_writes;
static void _exec_spindle_control(float *, bool *);
void spindle_reset(void);
void gpio_init_outputs(void) { assert(cm.gm.spindle_mode==SPINDLE_OFF); assert(!!(pins&SPINDLE_BIT)==!spindle_cfg.enable_polarity); }
bool mp_has_runnable_buffer(void) { assert(!(SREG & 128)); return queued; }
bool mp_runtime_is_idle(void) { assert(!(SREG & 128)); return !running; }
stat_t set_01(nvObj_t *nv) { assert(!(SREG & 128)); *nv->target=nv->value_int; return STAT_OK; }
void gpio_set_bit_on(unsigned bit) { pins |= bit; writes++; }
void gpio_set_bit_off(unsigned bit) { pins &= ~bit; writes++; }
void cm_set_spindle_mode(int model, int mode) { cm.gm.spindle_mode=mode; }
float cm_get_spindle_pwm(int mode) { return mode==SPINDLE_OFF ? 0 : .5f; }
void pwm_set_duty(int channel, float phase) { pwm_writes++; }
'''
        code += '\n'.join(definitions)
        code += r'''
int main(void) {
    spindle_config_init();
    assert(spindle_cfg.enable_polarity==1 && spindle_cfg.direction_polarity==0);
    for(int ep=0; ep<2; ep++) for(int dp=0; dp<2; dp++) {
        spindle_cfg.enable_polarity=ep; spindle_cfg.direction_polarity=dp;
        for(int mode=SPINDLE_OFF; mode<=SPINDLE_CCW; mode++) {
            pins=0; float value[]={mode}; _exec_spindle_control(value, FLAGS_NONE);
            assert(!!(pins&SPINDLE_BIT)==(mode==SPINDLE_OFF ? !ep : ep));
            if(mode!=SPINDLE_OFF) assert(!!(pins&SPINDLE_DIR)==(mode==SPINDLE_CW ? dp : !dp));
            assert(cm.gm.spindle_mode==mode);
            spindle_reset();
            assert(cm.gm.spindle_mode==SPINDLE_OFF && !!(pins&SPINDLE_BIT)==!ep);
        }
    }
    // Terminal-state inhibition must win over either electrical polarity.
    for(int state=MACHINE_ALARM; state<=MACHINE_PANIC; state++) {
        cm.machine_state=state;
        for(int ep=0;ep<2;ep++) for(int mode=SPINDLE_OFF;mode<=SPINDLE_CCW;mode++) {
            spindle_cfg.enable_polarity=ep; float value[]={mode};
            _exec_spindle_control(value,FLAGS_NONE);
            assert(cm.gm.spindle_mode==SPINDLE_OFF);
            assert(!!(pins&SPINDLE_BIT)==!ep);
        }
    }
    cm.machine_state=1;
    spindle_init_outputs();
    assert(spindle_configured);
    for(int interrupt_enabled=0; interrupt_enabled<2; interrupt_enabled++) {
        uint8_t saved=interrupt_enabled ? 0x95 : 0x15;
        for(int reason=0; reason<5; reason++) {
            cm.machine_state=1; cm.gm.spindle_mode=reason==1 ? SPINDLE_CW : SPINDLE_OFF;
            queued=reason==2; running=reason==3; st_pre.buffer_state=reason==4;
            spindle_cfg.enable_polarity=1; nvObj_t nv={0,&spindle_cfg.enable_polarity};
            SREG=saved; unsigned before=writes;
            assert(spindle_set_polarity(&nv)==(reason ? STAT_COMMAND_NOT_ACCEPTED : STAT_OK));
            assert(SREG==saved);
            assert(spindle_cfg.enable_polarity==(reason ? 1 : 0));
            if(reason) assert(writes==before); else assert(pins&SPINDLE_BIT);
        }
    }
    // Erased EEPROM and invalid settings must not change defaults or interrupt state.
    spindle_config_init();
    uint32_t invalid[]={2,255,UINT32_MAX};
    for(unsigned i=0;i<3;i++) {
        nvObj_t nv={invalid[i],&spindle_cfg.enable_polarity}; SREG=0x95;
        unsigned before=writes;
        assert(spindle_set_polarity(&nv)==STAT_INPUT_VALUE_RANGE_ERROR);
        assert(spindle_cfg.enable_polarity==1 && SREG==0x95 && writes==before);
    }
    // During boot, model-reset commands may already be queued; persisted polarity still loads.
    cm.machine_state=1; queued=running=true; st_pre.buffer_state=1;
    nvObj_t nv={0,&spindle_cfg.enable_polarity};
    assert(spindle_set_polarity(&nv)==STAT_OK && spindle_cfg.enable_polarity==0);
    assert(pins&SPINDLE_BIT);
    assert(pwm_writes>0);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            source_file = Path(directory) / "check.c"
            source_file.write_text(code)
            executable = Path(directory) / "check"
            subprocess.run(["cc", "-std=c99", "-g", "-fsanitize=address,undefined","-fstrict-flex-arrays=3", str(source_file), "-o", str(executable)], check=True)
            subprocess.run([str(executable)], check=True)

    def test_persistent_indexes_are_unchanged(self):
        """Compare the active AVR table to stock, including conditional motor rows."""
        stock = subprocess.check_output(["git", "show", "origin/edge-0.97:firmware/tinyg/config_app.c"], cwd=ROOT, text=True)
        current = (FIRMWARE / "config_app.c").read_text()
        def tokens(source):
            preprocessed = subprocess.check_output(["avr-gcc", "-mmcu=atxmega192a3", "-DF_CPU=32000000UL", "-I", str(FIRMWARE), "-E", "-P", "-x", "c", "-"], input=source, text=True)
            table = preprocessed.split("const cfgItem_t cfgArray[]", 1)[1]
            return re.findall(r'\{\s*"([^\"]*)"\s*,\s*"([^\"]*)"', table)
        old, new = tokens(stock), tokens(current)
        end = next(i for i, row in enumerate(old) if row[1] == "se29") + 1
        self.assertEqual(new[:end], old[:end])
        self.assertEqual(new[end:end+2], [("sys", "spep"), ("sys", "spdp")])
        self.assertEqual(new[end+2], ("sys", "prbp"))
        self.assertEqual(new[end+3], ("", "fstk"))                                  # E151: not in {sys:n}
        self.assertEqual(new[end+4], ("", "adc3"))
        self.assertEqual(new[end+5:end+8], [("", "net"), ("", "fd"), ("", "ma")])   # E099, not persisted
        self.assertEqual(new[end+8], ("sys", "os"))                                # E020, session only
        self.assertEqual(new[end+9:end+13], [("pwr", f"pwr{m}") for m in range(1, 5)])   # E108, read-only
        self.assertEqual(new[end+13:end+16], [("rt", "rthb"), ("rt", "rtqt"), ("rt", "rtup")])   # runtime session, RAM only
        self.assertEqual(new[end+16:end+20], [("rt", "rtflk"), ("rt", "rtflm"), ("rt", "rtflp"), ("rt", "rtflf")])  # feed-limit report
        self.assertEqual(new[end+20:end+22], [("rt", "rtfi"), ("rt", "rtfs")])     # feed-limit push (step 2)
        self.assertEqual(new[end+22:end+28], [("", "spe"), ("", "spd"), ("", "spc"), ("", "sps"), ("", "cof"), ("", "com")])  # g2core readouts
        groups = list(old[end:])
        groups.insert(groups.index(("", "jid")) + 1, ("", "rt"))                    # runtime session group
        self.assertEqual(new[end+28:end+30], [("rt", "rtjob"), ("rt", "rtfg")])      # host job flag, fg opt-in (step 3), RAM only
        self.assertEqual(new[end+30:], groups)


if __name__ == "__main__":
    unittest.main()
