"""Record the real firmware mount's travel range without debugger sampling during motion."""
from pathlib import Path
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]


class MountExtrema(unittest.TestCase):
    def test_extrema_include_steps_run_and_samples_without_edges(self):
        source = (ROOT / "firmware/tinyg/testhooks.c").read_text()
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define TINYG_TEST_HOOKS
#define MOTORS 4
typedef int stat_t;
#include "testhooks.h"
volatile test_hook_t test_hook;
struct {struct {int16_t steps_run;} en[MOTORS];} en;
static uint8_t SREG=128;
#define cli() (SREG=0)
#define SW_MAX_BIT_bp 7
#define SW_MIN_BIT_bp 6
static int changes;
static void _apply(uint8_t axis,uint8_t bit,uint8_t action){changes++;}
''' + function(source, r'static void _service_mounts\(\)') + r'''
int main(void){
 volatile test_mount_t *m=&test_hook.mount[0];
 m->enabled=1;m->motor=0;m->sense=-1;m->at=-640;m->closed=TEST_HOOK_UNKNOWN;
 _service_mounts();assert(m->min_steps==0&&m->max_steps==0);
 test_hook.steps[0]=-700;en.en[0].steps_run=-4;
 _service_mounts();assert(m->trips==1&&m->trip_steps==-704);
 test_hook.steps[0]=-900;en.en[0].steps_run=-28;
 _service_mounts();assert(m->min_steps==-928&&m->max_steps==0);  // no new edge
 test_hook.steps[0]=-600;en.en[0].steps_run=2;
 _service_mounts();assert(!m->closed&&m->trips==1);
 test_hook.steps[0]=-280;en.en[0].steps_run=0;
 _service_mounts();assert(m->min_steps==-928&&m->max_steps==0&&SREG==128);
 assert(changes==3);
 return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            c, exe = Path(tmp)/"mount.c", Path(tmp)/"mount"
            c.write_text(code)
            subprocess.run(["cc", "-std=gnu99", "-fsanitize=address,undefined", "-fstrict-flex-arrays=3", "-I", str(ROOT/"firmware/tinyg"),
                            str(c), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()
