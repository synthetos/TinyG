"""N73: the homing clear move must not read a limit switch that does not exist.

_homing_axis_clear (cycle_homing.c) checks the homing switch, then sw.s[hm.limit_switch]; hm.limit_switch is -1 when
the axis has no limit switch on its other end, so the second test read the byte before the switch table. Whatever it
held decided whether a clear move ran, and in which direction. Homing and Limits wiki: the clear move backs off a
switch that is already closed. The search that follows re-reads the switches with this guard (E132).

The real function is compiled with the bounds and address sanitizers: the switch table is a fixed-size array, so an
index of -1 is reported. Cases: no limit switch and the homing switch open (no move), no limit switch and the homing
switch closed (backs off it), a closed limit switch (backs off the other way).
"""
from pathlib import Path
import subprocess, tempfile, unittest
from source_extract import function
ROOT = Path(__file__).resolve().parents[2]
PRE = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
typedef int stat_t;
enum {STAT_OK, STAT_EAGAIN, SW_INACTIVE, SW_ACTIVE};
struct {struct {int state;} s[8];} sw;
struct {stat_t (*func)(int8_t);int8_t homing_switch,limit_switch;float latch_backoff,search_velocity;} hm;
static int moves; static float moved;
static stat_t _homing_axis_move(int8_t a,float d,float v){moves++;moved=d;return STAT_EAGAIN;}
static stat_t _set_homing_func(stat_t (*f)(int8_t)){hm.func=f;return STAT_EAGAIN;}
static stat_t _homing_axis_search(int8_t a){return STAT_OK;}
'''


class HomingLimitIndex(unittest.TestCase):
    def test_no_limit_switch_is_not_read(self):
        src = (ROOT / 'firmware/tinyg/cycle_homing.c').read_text(errors='replace')
        code = PRE + function(src, r'static stat_t _homing_axis_clear\([^;{]*\)(?=[^;{]*\{)') + '\n' + r'''
int main(void){
 hm.latch_backoff=2;hm.search_velocity=100;hm.homing_switch=0;
 hm.limit_switch=-1;sw.s[0].state=SW_INACTIVE;moves=0;
 _homing_axis_clear(0);assert(moves==0 && hm.func==_homing_axis_search);   /* no switch closed, no limit: no move */
 hm.limit_switch=-1;sw.s[0].state=SW_ACTIVE;moves=0;
 _homing_axis_clear(0);assert(moves==1 && moved==2);                       /* homing switch closed: back off it */
 hm.limit_switch=1;sw.s[0].state=SW_INACTIVE;sw.s[1].state=SW_ACTIVE;moves=0;
 _homing_axis_clear(0);assert(moves==1 && moved==-2);                      /* limit switch closed: the other way */
 return 0;
}
'''
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'clear.c'; p.write_text(code); exe = p.with_suffix('')
            subprocess.run(['cc', '-std=c99', '-fsanitize=address,undefined', '-fstrict-flex-arrays=3', '-fno-sanitize-recover=all',
                            str(p), '-o', str(exe)], check=True)
            r = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr[-800:])


if __name__ == '__main__':
    unittest.main()
