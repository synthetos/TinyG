"""E014 (GitHub #264): the first segment's velocity must be the quintic velocity profile at h/2.

_init_forward_diffs() (plan_exec.c) sets up forward differences for V(t) = A t^5 + B t^4 + C t^3 + Vi with
A = 6(Vt-Vi), B = -15(Vt-Vi), C = 10(Vt-Vi), and seeds mr.segment_velocity with V(h/2). Master 440.x computed
half_Ah_5 from C instead of A (#264), so the first segment ran at the wrong speed. This runs the shipped function,
checks the seed against the closed form, and steps the forward differences the way _exec_aline_head() does
(velocity += fd5; fd5 += fd4; fd4 += fd3; fd3 += fd2; fd2 += fd1) to check every later segment midpoint too.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]

HARNESS = r'''
#include <stdio.h>
struct { float segments, segment_velocity, forward_diff_1, forward_diff_2, forward_diff_3, forward_diff_4,
         forward_diff_5; } mr;
%s
int main(void) {
    const float cases[][3] = {{0, 1000, 20}, {1000, 0, 20}, {200, 1800, 7}, {1500, 300, 1}, {50, 60, 400}};
    for (unsigned c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        float Vi = cases[c][0], Vt = cases[c][1];
        mr.segments = cases[c][2];
        _init_forward_diffs(Vi, Vt);
        printf("case %%u %%g %%g %%g\n", c, Vi, Vt, mr.segments);
        for (int k = 0; k < (int)mr.segments; k++) {
            printf("v %%d %%.6f\n", k, mr.segment_velocity);
            mr.segment_velocity += mr.forward_diff_5; mr.forward_diff_5 += mr.forward_diff_4;
            mr.forward_diff_4 += mr.forward_diff_3; mr.forward_diff_3 += mr.forward_diff_2;
            mr.forward_diff_2 += mr.forward_diff_1;
        }
    }
    return 0;
}
'''


def profile(vi, vt, t):
    d = vt - vi
    return 6 * d * t ** 5 - 15 * d * t ** 4 + 10 * d * t ** 3 + vi


class FirstSegmentVelocity(unittest.TestCase):
    def test_seed_and_midpoints_follow_the_quintic(self):
        cc = shutil.which("cc") or shutil.which("gcc")
        if cc is None:
            self.skipTest("no C compiler")
        src = function((ROOT / "firmware/tinyg/plan_exec.c").read_text(), r"static void _init_forward_diffs\(float Vi, float Vt\)(?=\s*\{)")
        with tempfile.TemporaryDirectory() as tmp:
            c = Path(tmp) / "h.c"
            c.write_text(HARNESS % src.replace("static void", "void", 1))
            exe = Path(tmp) / "h"
            subprocess.run([cc, "-std=gnu99", "-O0", "-fsanitize=undefined", str(c), "-o", str(exe), "-lm"],
                           check=True)
            out = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout
        vi = vt = n = None
        checked = 0
        for line in out.splitlines():
            f = line.split()
            if f[0] == "case":
                vi, vt, n = float(f[2]), float(f[3]), float(f[4])
            else:
                k, v = int(f[1]), float(f[2])
                want = profile(vi, vt, (k + 0.5) / n)
                tol = 1e-3 * max(abs(vi), abs(vt), 1.0)
                self.assertAlmostEqual(v, want, delta=tol, msg=f"Vi={vi} Vt={vt} segments={n} k={k}")
                checked += 1
        self.assertGreater(checked, 400)


if __name__ == "__main__":
    unittest.main()
