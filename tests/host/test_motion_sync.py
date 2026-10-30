"""Deterministic M400 completion, reply preservation and runtime/prep drain checks.

Extract the production functions. No serial, ICE, wall-clock or interrupt latency claims.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


def function(source, name):
    match = re.search(r"^(?:static )?(?:stat_t|bool) " + name + r"\([^;]*?\)\s*\{", source, re.M)
    assert match, name
    depth = 1
    tokens = re.compile(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|//[^\n]*|/\*.*?\*/|[{}]', re.S)
    for token in tokens.finditer(source, match.end()):
        depth += (token.group() == '{') - (token.group() == '}')
        if depth == 0:
            return source[match.start():token.end()]
    raise AssertionError(f"unterminated function {name}")


class MotionSync(unittest.TestCase):
    def test_drain_and_response_lifetime(self):
        source = (ROOT / 'firmware/tinyg/controller.c').read_text()
        state = source[source.index('enum { SYNC_OFF'):source.index('} motion_sync;') + len('} motion_sync;')]
        code = r'''
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#define OUTPUT_BUFFER_LEN 512
#define STAT_OK 0
#define STAT_NOOP 3
#define STAT_EAGAIN 2
#define STAT_COMMAND_NOT_ACCEPTED 106
#define NUL 0
#define RX_MODE_CHAR 0
#define PREP_BUFFER_OWNED_BY_LOADER 0
#define __AVR 1
#define PSTR(x) (x)
#define strcpy_P strcpy
#define snprintf_P snprintf
uint8_t SREG=128;
#define cli() (SREG=0)
struct {uint8_t buffer_state;} st_pre={1};
struct {int rx_mode;} xio={1};
struct {uint16_t linelen;} cs={5};
bool queued, ticks;
int alarm, credits=31, replies, text_status;
char sent[600];
bool mp_has_runnable_buffer(void) {assert(SREG==0); return queued;}
bool mp_runtime_is_idle(void) {assert(SREG==0); return !ticks;}
int cm_is_alarmed(void) {return alarm;}
int xio_get_line_buffers_available(void) {return credits;}
void nv_reset_nv_list(int unused) {(void)unused;}
void text_response(int status, char *command) {assert(!strcmp(command,"M400")); text_status=status; replies++;}
int capture(const char *s, FILE *f) {(void)f; strcpy(sent,s); replies++; return 0;}
#define fputs capture
typedef int stat_t;
'''
        code += state
        code += '\n'.join(function(source, name) for name in
                          ('controller_request_motion_sync', 'controller_defer_motion_response', '_sync_to_motion'))
        code += r'''
int main(void) {
    char response[OUTPUT_BUFFER_LEN]="{\"r\":{\"gc\":\"M400\",\"tid\":123},\"f\":[3,0,9]}\n";
    assert(_sync_to_motion()==STAT_NOOP);
    assert(controller_request_motion_sync()==0);
    assert(controller_request_motion_sync()==STAT_COMMAND_NOT_ACCEPTED);
    assert(controller_defer_motion_response(0,response));
    motion_sync.state=SYNC_WAITING;
    memset(response,'X',sizeof(response)); // a control query reuses the output buffer
    assert(!controller_defer_motion_response(0,"control response"));
    queued=true; assert(_sync_to_motion()==STAT_EAGAIN && replies==0 && SREG==128);
    queued=false; ticks=true; assert(_sync_to_motion()==STAT_EAGAIN && replies==0);
    ticks=false; st_pre.buffer_state=PREP_BUFFER_OWNED_BY_LOADER;
    assert(_sync_to_motion()==STAT_EAGAIN && replies==0);
    st_pre.buffer_state=1;
    assert(_sync_to_motion()==0 && replies==1 && motion_sync.state==SYNC_OFF && SREG==128);
    assert(!strcmp(sent,"{\"r\":{\"gc\":\"M400\",\"tid\":123},\"f\":[3,0,31]}\n"));
    assert(_sync_to_motion()==STAT_NOOP && replies==1);
    xio.rx_mode=RX_MODE_CHAR; cs.linelen=65535;
    assert(controller_request_motion_sync()==0);
    assert(controller_defer_motion_response(0,"{\"r\":{\"gc\":\"M400\"},\"f\":[2,0,65535]}\n"));
    motion_sync.state=SYNC_WAITING; cs.linelen=3; xio.rx_mode=1; ticks=true; alarm=203;
    assert(_sync_to_motion()==0 && replies==2 && SREG==128);
    assert(strstr(sent,"[2,203,65535]}")!=NULL);
    alarm=0; ticks=false;
    assert(controller_request_motion_sync()==0);
    assert(controller_defer_motion_response(0,NULL));
    motion_sync.state=SYNC_WAITING;
    assert(_sync_to_motion()==0 && replies==3 && text_status==0);
    assert(controller_request_motion_sync()==0);
    assert(!controller_defer_motion_response(132,"bad request"));
    assert(_sync_to_motion()==STAT_NOOP && replies==3);
    // Silent verbosity still creates a barrier without a deferred reply.
    assert(controller_request_motion_sync()==0);
    motion_sync.state=SYNC_WAITING; ticks=true;
    assert(_sync_to_motion()==STAT_EAGAIN);
    ticks=false; assert(_sync_to_motion()==0 && replies==3);
    // Maximum serializer output plus a longer error/footer stays inside the buffer.
    memset(response,' ',sizeof(response));
    strcpy(response+sizeof(response)-strlen("[3,0,9]}\n")-1,"[3,0,9]}\n");
    assert(controller_request_motion_sync()==0);
    assert(controller_defer_motion_response(0,response));
    motion_sync.state=SYNC_WAITING; alarm=203; credits=255;
    assert(_sync_to_motion()==0 && replies==4 && strstr(sent,"[3,203,255]}")!=NULL);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            c = Path(tmp) / 'sync.c'
            c.write_text(code)
            exe = Path(tmp) / 'sync'
            subprocess.run(['cc', '-std=c99', '-Wall', '-Werror', '-fsanitize=address,undefined',
                            str(c), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()
