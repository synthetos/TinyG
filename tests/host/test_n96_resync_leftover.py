"""N96: rewriting the step counter folds in the last segment's steps first (encoder.c en_set_encoder_steps).

The real function and ACCUMULATE_ENCODER from encoder.h run against a stand-in en struct, plain and with the TEST_HOOKS
step count, which nothing may rewrite.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT = Path(__file__).resolve().parents[2]

PRELUDE = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
typedef struct {int8_t step_sign; int16_t steps_run; int32_t encoder_steps;} enEncoder_t;
struct {enEncoder_t en[6];} en;
#ifdef TINYG_TEST_HOOKS
struct {int32_t steps[6];} test_hook;
#endif
'''

MAIN = r'''
int main(void) {
 en.en[0].encoder_steps = 6400; en.en[0].steps_run = 3;     /* the last segment ran 3 steps */
 en_set_encoder_steps(0, 0.0f);                              /* G28.3 X0 */
 assert(en.en[0].encoder_steps == 0 && en.en[0].steps_run == 0);
 en.en[0].steps_run = -2;
 en_set_encoder_steps(0, 1280.4f);
 assert(en.en[0].encoder_steps == 1280 && en.en[0].steps_run == 0);
#ifdef TINYG_TEST_HOOKS
 assert(test_hook.steps[0] == 1);                            /* 3 - 2: the real pulses are still counted */
#endif
 return 0;
}
'''


class ResyncLeftover(unittest.TestCase):
    def build(self, defines):
        tinyg = ROOT / "firmware/tinyg"
        header = (tinyg / "encoder.h").read_text().replace("\r\n", "\n")
        macros = "\n".join(re.findall(r"^#(?:if|ifdef|else|endif|define ACCUMULATE_ENCODER)[^\n]*", header, re.M))
        macros = macros.replace('#include "testhooks.h"', "")
        source = (tinyg / "encoder.c").read_text().replace("\r\n", "\n")
        code = defines + PRELUDE + macros + "\n"
        code += function(source, r"void en_set_encoder_steps\([^;{}]*\)(?=\s*\{)") + "\n" + MAIN
        with tempfile.TemporaryDirectory() as d:
            c, exe = Path(d) / "check.c", Path(d) / "check"
            c.write_text(code)
            subprocess.run(["cc", "-Wall", "-Werror", "-fsanitize=address,undefined", str(c), "-o", str(exe), "-lm"],
                           check=True)
            subprocess.run([str(exe)], check=True)

    def test_resync_drops_the_last_segment_steps(self):
        self.build("")

    def test_test_hooks_still_count_them(self):
        self.build("#define TINYG_TEST_HOOKS\n")


if __name__ == "__main__":
    unittest.main()
