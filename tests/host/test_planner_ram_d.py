"""Only modal/motion context crosses the queue; section time and dwell payload are separate."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT/'firmware/tinyg'
HEADERS = '#include <stddef.h>\n#include "tinyg.h"\n#include "config.h"\n#include "canonical_machine.h"\n#include "planner.h"\n'


class QueuedSnapshot(unittest.TestCase):
    @unittest.skipUnless(shutil.which('avr-gcc'), 'AVR compiler required')
    def test_snapshot_drops_only_the_timing_word(self):
        code = HEADERS + r'''
#include <stddef.h>
_Static_assert(sizeof(GCodeState_t) == 68, "retain full model/runtime state");
_Static_assert(sizeof(((mpBuf_t*)0)->gm) == 64, "queue only the common snapshot");
_Static_assert(offsetof(GCodeState_t,move_time) == 64, "copy boundary must precede timing");
_Static_assert(sizeof(((mpBuf_t*)0)->gm.target) == 24, "retain all target coordinates");
_Static_assert(sizeof(((mpBuf_t*)0)->gm.work_offset) == 24, "retain historical offsets");
'''
        with tempfile.TemporaryDirectory() as d:
            src,obj=Path(d)/'layout.c',Path(d)/'layout.o';src.write_text(code)
            subprocess.run(['avr-gcc','-mmcu=atxmega192a3','-DF_CPU=32000000UL','-std=gnu99',
                            '-funsigned-char','-funsigned-bitfields','-fpack-struct','-fshort-enums',
                            '-I'+str(FW),'-c',str(src),'-o',str(obj)],check=True)

    def test_real_snapshot_copy_sites_preserve_context_and_do_not_copy_section_time(self):
        queue = next(l for l in (FW/'plan_line.c').read_text().splitlines() if 'memcpy(&bf->gm,' in l)
        lines=(FW/'plan_exec.c').read_text().splitlines()
        i=next(i for i,l in enumerate(lines) if 'memcpy(&mr.gm,' in l)
        restore=lines[i]
        if 'mr.gm.move_time = 0' in lines[i+1]: restore+='\n'+lines[i+1]
        code=HEADERS+r'''
#include <assert.h>
#include <string.h>
__typeof__(mr) mr;
static void queue_context(mpBuf_t *bf,GCodeState_t *gm_in){
'''+queue+'\n}\nstatic void restore_context(mpBuf_t *bf){\n'+restore+r'''
}
int main(void){
 GCodeState_t model;memset(&model,0x5a,sizeof model);
 model.linenum=731;model.feed_rate=123.5;model.spindle_speed=1500;model.move_time=3.5;
 for(unsigned a=0;a<AXES;a++){model.target[a]=a+0.25;model.work_offset[a]=a-0.5;}
 mpBuf_t block;memset(&block,0,sizeof block);block.fl_tags=0xa5;block.fl_cls=0x5a;
 queue_context(&block,&model);
 assert(block.fl_tags==0xa5&&block.fl_cls==0x5a);
 memset(&mr.gm,0,sizeof mr.gm);restore_context(&block);
 assert(!memcmp(&mr.gm,&model,offsetof(GCodeState_t,move_time)));
 assert(mr.gm.move_time==0); /* the exec sets each section's time, never restores it from a slot */
}
'''
        with tempfile.TemporaryDirectory() as d:
            src,exe=Path(d)/'copy.c',Path(d)/'copy';src.write_text(code)
            subprocess.run(['cc','-std=gnu99','-funsigned-char','-fshort-enums','-fpack-struct',
                            '-fsanitize=address,undefined','-fstrict-flex-arrays=3',
                            '-I'+str(ROOT/'tests/host/planner_startup'),'-I'+str(FW),
                            str(src),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

    def test_dwell_payload_keeps_units_and_queue_completion(self):
        code=HEADERS+r'''
#include <assert.h>
#include <string.h>
static mpBuf_t block;
static float staged;
static unsigned ends,commits;
static bool empty;
void cm_run_short_move(void) {}
stat_t cm_panic_P(const stat_t status,const char *msg){assert(0);return status;}
mpBuf_t *mp_get_write_buffer(void){memset(&block,0,sizeof block);return &block;}
void mp_commit_write_buffer(const uint8_t type){assert(type==BLOCK_TYPE_DWELL);block.block_type=type;commits++;}
uint8_t mp_free_run_buffer(void){return empty;}
void cm_cycle_end(void){ends++;}
void st_prep_dwell(float us){staged=us;}
'''
        source=(FW/'planner.c').read_text()
        code+=function(source,r'stat_t mp_dwell\(')+'\n'+function(source,r'stat_t mp_exec_dwell\(')
        code+=r'''
int main(void){
 const float seconds[]={0,0.125,1,12.5};
 const float micros[]={0,125000,1000000,12500000};
 for(unsigned i=0;i<4;i++){
  empty=(i&1);unsigned before=ends;
  assert(mp_dwell(seconds[i])==STAT_OK);
  assert(block.block_type==BLOCK_TYPE_DWELL&&block.block_state==BLOCK_INITIALIZING);
  assert(block.dwell_seconds==seconds[i]);
  assert(mp_exec_dwell(&block)==STAT_OK&&staged==micros[i]);
  assert(ends==before+empty);
 }
 assert(commits==4);
}
'''
        with tempfile.TemporaryDirectory() as d:
            src,exe=Path(d)/'dwell.c',Path(d)/'dwell';src.write_text(code)
            subprocess.run(['cc','-std=gnu99','-fsanitize=address,undefined','-fstrict-flex-arrays=3',
                            '-I'+str(ROOT/'tests/host/planner_startup'),'-I'+str(FW),
                            str(src),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)


if __name__ == '__main__': unittest.main()
