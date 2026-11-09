"""Block-type dispatch has the same guards and only executes the three runnable types."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function

FW = Path(__file__).resolve().parents[2] / 'firmware/tinyg'


class BlockTypeDispatch(unittest.TestCase):
    def test_dispatch_and_guards_without_per_buffer_function_pointer(self):
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#define STAT_OK 0
#define STAT_NOOP 1
#define STAT_INTERNAL_ERROR 99
#define PSTR(s) (s)
typedef int stat_t;
enum {BLOCK_TYPE_NULL,BLOCK_TYPE_ALINE,BLOCK_TYPE_DWELL,BLOCK_TYPE_COMMAND};
enum {MOTION_STOP,MOTION_RUN,MOTION_HOLD};
enum {FEEDHOLD_OFF,FEEDHOLD_HOLD};
typedef struct mpBuffer {int block_type;stat_t (*bf_func)(struct mpBuffer*);} mpBuf_t;
static struct {bool program_paused;int hold_state,motion_state;} cm;
static struct {bool exec_ran;} mb;
static mpBuf_t block;
static bool startup,busy,aline,queued=true;
static unsigned calls,panics,nulls;
static bool mp_startup_is_pending(void){return startup;}
static bool st_runtime_isbusy(void){return busy;}
static bool st_aline_in_flight(void){return aline;}
static mpBuf_t *mp_get_run_buffer(void){return queued?&block:NULL;}
static void st_prep_null(void){nulls++;}
static void cm_set_motion_state(int v){cm.motion_state=v;}
static stat_t cm_panic_P(int status,const char *s){panics++;return status;}
static stat_t execute(mpBuf_t *b,int type){assert(b==&block&&b->block_type==type&&mb.exec_ran);calls++;return 10+type;}
static stat_t mp_exec_aline(mpBuf_t *b){return execute(b,BLOCK_TYPE_ALINE);}
static stat_t mp_exec_dwell(mpBuf_t *b){return execute(b,BLOCK_TYPE_DWELL);}
static stat_t mp_exec_command(mpBuf_t *b){return execute(b,BLOCK_TYPE_COMMAND);}
'''
        code += function((FW/'plan_exec.c').read_text(),r'stat_t mp_exec_move\(\)')
        code += r'''
int main(void){
 for(int type=0;type<8;type++){
  block.block_type=type;mb.exec_ran=false;unsigned before=calls;
  int result=mp_exec_move();
  if(type>=BLOCK_TYPE_ALINE&&type<=BLOCK_TYPE_COMMAND){assert(result==10+type&&calls==before+1&&mb.exec_ran);}
  else {assert(result==STAT_INTERNAL_ERROR&&calls==before&&!mb.exec_ran);}
 }
 assert(calls==3&&panics==5);
 block.block_type=BLOCK_TYPE_ALINE;mb.exec_ran=false;
 cm.program_paused=true;assert(mp_exec_move()==STAT_NOOP&&!mb.exec_ran);cm.program_paused=false;
 startup=true;assert(mp_exec_move()==STAT_NOOP&&!mb.exec_ran);startup=false;
 cm.hold_state=FEEDHOLD_HOLD;assert(mp_exec_move()==STAT_NOOP&&!mb.exec_ran);cm.hold_state=FEEDHOLD_OFF;
 busy=true;assert(mp_exec_move()==STAT_OK&&!mb.exec_ran&&nulls==1);busy=false;
 queued=false;assert(mp_exec_move()==STAT_NOOP&&!mb.exec_ran&&nulls==2);
 assert(calls==3);
}
'''
        with tempfile.TemporaryDirectory() as d:
            src, exe = Path(d)/'dispatch.c', Path(d)/'dispatch'
            src.write_text(code)
            subprocess.run(['cc','-std=c99','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(src),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)


if __name__ == '__main__': unittest.main()
