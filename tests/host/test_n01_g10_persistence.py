"""N01: idle G10 must finish persistence before its response permits the next line.

NIST RS274NGC v3 3.5.5; TinyG wiki Coordinate-Systems: G10 offsets persist once
machining is done. EEPROM's AVR1008 workaround disables serial interrupts, so
background writes after the response can lose the next stop-and-wait request.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]
FW=ROOT/'firmware/tinyg'

class G10Persistence(unittest.TestCase):
    def test_idle_ack_order_and_serial_busy(self):
        source=(FW/'canonical_machine.c').read_text()
        code=r'''
#include <assert.h>
#include <stdlib.h>
#include "tinyg.h"
#include "config.h"
#include "canonical_machine.h"
#include "planner.h"
#include "util.h"
cmSingleton_t cm;
bool FLAGS_NONE[AXES];
static int saves, rx, tx;
static bool segment;
static int USBu;
uint16_t xio_get_rx_bufcount_usart(void *p) { return rx; }
uint16_t xio_get_tx_bufcount_usart(void *p) { return tx; }
bool st_aline_in_flight(void) { return segment; }
void cm_set_work_offsets(void) {}
float cm_get_active_coord_offset(uint8_t axis) { return 0; }
void mp_queue_command(void(*fn)(float[],bool[]),float *v,bool *f) {}
void mp_set_runtime_work_offset(float *v) {}
static void _exec_g10_offset(float *,bool *);
index_t nv_get_index(const char *group,const char *token) { return (token[2]-'4')*6+(strchr("xyzabc",token[3])-"xyzabc"); }
stat_t nv_persist(nvObj_t *nv) { saves++; return STAT_OK; }
'''
        code+=function((FW/'xio.c').read_text(),r'uint8_t xio_isbusy\(\)')
        # Compile the real AVR callback guard without dragging AVR headers into the host.
        code+='\n#define NUL 0\n'
        names=['_persist_coord_offsets','cm_deferred_write_callback','cm_set_coord_offsets','_exec_g10_offset']
        for name in names:
            sig=r'(?:static )?(?:void|stat_t) '+name+r'\([^;{}]*\)\s*(?=\{)'
            if re.search(sig,source): code+=function(source,sig)
        code+=r'''
static void prepare(void) {
    memset(&cm,0,sizeof(cm)); saves=rx=tx=0; segment=false;
    cm.gf.parameter=cm.gf.L_word=true; cm.gm.coord_system=G54;
}
int main(void) {
    prepare(); assert(!xio_isbusy()); rx=1; assert(xio_isbusy()); rx=0;tx=1;assert(xio_isbusy());
    prepare(); cm.deferred_write_flag=true;rx=1;
    cm_deferred_write_callback();assert(saves==0 && cm.deferred_write_flag);
    rx=0;tx=1;cm_deferred_write_callback();assert(saves==0 && cm.deferred_write_flag);
    tx=0;cm_deferred_write_callback();assert(saves==36 && !cm.deferred_write_flag);
    // The setter returns to the parser before its response is printed. Each idle
    // request must have finished its writes at that boundary, regardless of TX reports.
    prepare();tx=1;
    float v[AXES]={1,2,3};bool flags[AXES]={true,true,true};
    for(int cs=1;cs<=6;cs++) {
        assert(cm_set_coord_offsets(cs,2,v,flags)==STAT_OK);
        assert(saves==cs*36 && !cm.deferred_write_flag);
    }
    // Motion retains deferred persistence; a final physical segment is motion too.
    prepare();cm.cycle_state=CYCLE_MACHINING;
    assert(cm_set_coord_offsets(2,2,v,flags)==STAT_OK);
    assert(!saves && cm.deferred_write_flag);
    prepare();segment=true;
    assert(cm_set_coord_offsets(2,2,v,flags)==STAT_OK);
    assert(!saves && cm.deferred_write_flag);
}
'''
        with tempfile.TemporaryDirectory() as d:
            d=Path(d);(d/'avr').mkdir();(d/'avr/pgmspace.h').write_text('#define PROGMEM\n#define PSTR(s) (s)\n')
            c=d/'g10.c';c.write_text(code);exe=d/'g10'
            subprocess.run(['cc','-std=gnu99','-O1','-g','-fsanitize=address,undefined','-D_POSIX_C_SOURCE=200809L','-I'+str(d),'-I'+str(FW),str(c),'-lm','-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
