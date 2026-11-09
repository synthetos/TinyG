"""Packed command flags retain all six synchronous callback arguments."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function

FW = Path(__file__).resolve().parents[2] / 'firmware/tinyg'


class CommandFlagMask(unittest.TestCase):
    def test_every_command_flag_combination_round_trips(self):
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define AXES 6
#define AXIS_X 0
#define FL_CLS_OTHER 0
#define FL_COMMAND 32
#define STAT_OK 0
#define STAT_BUFFER_FULL_FATAL 251
#define PSTR(s) (s)
typedef int stat_t;
enum {MP_BUFFER_EMPTY,MP_BUFFER_LOADING};
enum {BLOCK_TYPE_NULL,BLOCK_TYPE_ALINE,BLOCK_TYPE_DWELL,BLOCK_TYPE_COMMAND};
enum {BLOCK_IDLE,BLOCK_INITIALIZING,BLOCK_RUNNING};
typedef struct mpBuffer {
 struct mpBuffer *pv; stat_t (*bf_func)(struct mpBuffer *); void (*cm_func)(float[],bool[]);
 unsigned buffer_state,block_type,block_state,fl_tags,fl_cls;
 float value_vector[AXES]; bool axis_flags[AXES]; uint8_t axis_mask;
} mpBuf_t;
static mpBuf_t block,previous;
static unsigned expected,calls,commits,ends;
static bool empty;
static void cm_run_short_move(void) {}
static stat_t cm_panic_P(int status,const char *s){assert(0);return status;}
static mpBuf_t *mp_get_write_buffer(void){memset(&block,0,sizeof block);block.pv=&previous;return &block;}
static void mp_commit_write_buffer(int type){assert(type==BLOCK_TYPE_COMMAND);commits++;}
static bool mp_free_run_buffer(void){return empty;}
static void cm_cycle_end(void){ends++;}
static stat_t _exec_command(mpBuf_t *b){return 0;}
static void callback(float values[],bool flags[]){
 calls++;
 for(unsigned a=0;a<AXES;a++){assert(values[a]==10+a);assert(flags[a]==!!(expected&(1u<<a)));}
 flags[0]=!flags[0]; /* local callback storage, no pointer retained */
}
'''
        source = (FW/'planner.c').read_text()
        code += function(source, r'void mp_queue_command_class\(') + '\n'
        code += function(source, r'stat_t mp_runtime_command\(') + '\n'
        code += r'''
int main(void){
 for(expected=0;expected<64;expected++){
  float values[AXES];bool flags[AXES];
  for(unsigned a=0;a<AXES;a++){values[a]=10+a;flags[a]=!!(expected&(1u<<a));}
  mp_queue_command_class(callback,values,flags,0);
  assert(block.axis_mask==expected);
  memset(flags,0,sizeof flags); /* caller storage may change after queueing */
  empty=(expected&1);unsigned before=ends;
  assert(mp_runtime_command(&block)==STAT_OK);
  assert(ends==before+empty);
 }
 assert(calls==64&&commits==64);
}
'''
        with tempfile.TemporaryDirectory() as d:
            src, exe = Path(d)/'flags.c', Path(d)/'flags'
            src.write_text(code)
            subprocess.run(['cc','-std=c99','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(src),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)


if __name__ == '__main__': unittest.main()
