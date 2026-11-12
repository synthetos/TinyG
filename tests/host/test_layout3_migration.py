"""Removing output-pause rows preserves settings and report selections across interrupted writes."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from source_extract import function

FW = Path(__file__).resolve().parents[2] / 'firmware/tinyg'


class Layout3Migration(unittest.TestCase):
    def test_actual_avr_table_translation(self):
        import subprocess
        old = subprocess.check_output(['git','show','998c02eb:firmware/tinyg/config_app.c'],cwd=FW,text=True)
        def tokens(source):
            pre = subprocess.check_output(['avr-gcc','-mmcu=atxmega192a3','-DF_CPU=32000000UL',
                '-I',str(FW),'-E','-P','-x','c','-'],input=source,text=True)
            body = pre.rsplit('const cfgItem_t cfgArray[]',1)[1].split('\n};',1)[0]
            return re.findall(r'\{\s*"[^"]*"\s*,\s*"([^"]+)"',body)
        before, after = tokens(old), tokens((FW/'config_app.c').read_text())
        first, stamp = before.index('spph'), before.index('_hld')
        self.assertEqual(after, [t for t in before if t not in ('spph','spde','coph','_hld')])
        for i,t in enumerate(before):
            if t in ('spph','spde','coph','_hld'):continue
            mapped = i if i<first else i-(3 if i<stamp else 4)
            self.assertEqual(after[mapped],t)
        end=after.index('sys')
        self.assertLess(end+4+end-after.index('prbp')+30,510-1)

    def test_every_byte_cut_and_saved_report_indexes(self):
        source = (FW / 'config.c').read_text()
        names = ['_layout2_journal_ready', '_layout2_index', '_migrate_layout2']
        for name in names:
            self.assertRegex(source, r'\b' + name + r'\(')
        code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <setjmp.h>
#define CONFIG_NVM_LAYOUT_MARKER 10003.0
#define STAT_OK 0
#define STAT_CONFIG_ASSERTION_FAILURE 90
#define NO_MATCH -1
#define PSTR(x) (x)
#define CHECKPOINT_A 126
#define NV_STATUS_REPORT_LEN 3
#define LAYOUT2_JOURNAL_STAMP 0xC302A65FUL
#define LAYOUT2_JOURNAL_FLAG (CHECKPOINT_A-1)
typedef int index_t;
typedef int stat_t;
typedef struct {index_t index;union {uint32_t value_int;float value_flt;};} nvObj_t;
static uint32_t ee[128], original[128], expected[128];
static jmp_buf reboot;
static int cut, writes;
static index_t nv_get_index(const char*g,const char*t) {
 if(!strcmp(t,"se00"))return 2;
 if(!strcmp(t,"prbp"))return 7;
 if(!strcmp(t,"rtjob"))return 11;
 return NO_MATCH;
}
static bool nv_index_is_single(index_t i){return i<13;}
static int nv_index_max(void){return 20;}
static stat_t read_persistent_value(nvObj_t*n){n->value_int=ee[n->index];return 0;}
static void write_persistent_byte(index_t i,int b,uint8_t v){
 if(writes++==cut)longjmp(reboot,1);
 ee[i]=(ee[i]&~(255UL<<(8*b)))|((uint32_t)v<<(8*b));
}
static stat_t write_persistent_value(nvObj_t*n){
 for(int b=0;b<4;b++)write_persistent_byte(n->index,b,n->value_int>>(8*b));return 0;
}
static void _write_nvm_layout_marker(nvObj_t*n){n->index=0;n->value_flt=10003;write_persistent_value(n);}
static stat_t cm_panic_P(int s,const char*t){assert(0);return s;}
'''
        for name in names:
            code += function(source, r'(?:static )?(?:bool|index_t|void) '+name+r'\([^;{}]*\)(?=\s*\{)')+'\n'
        code += r'''
static void boot(void){nvObj_t n={0};read_persistent_value(&n);
 if(n.value_flt==10002 || _layout2_journal_ready())_migrate_layout2(&n);
}
int main(void){
 for(int i=0;i<128;i++)original[i]=0xABC00000+i;
 nvObj_t n={0};n.value_flt=10002;original[0]=n.value_int;
 // Old 7..9 are removed. Old 14 is the activation stamp. Old 10..13 and 15..16 survive.
 original[2]=10;original[3]=15;original[4]=7;
 original[7]=1;original[8]=0x40000000;original[9]=1;
 original[LAYOUT2_JOURNAL_FLAG]=0;
 memcpy(ee,original,sizeof ee);cut=-1;writes=0;boot();int total=writes;
 memcpy(expected,ee,sizeof ee);
 assert(ee[2]==7 && ee[3]==11 && ee[4]==(uint32_t)NO_MATCH);
 for(int i=7;i<11;i++)assert(ee[i]==original[i+3]);
 for(int i=11;i<13;i++)assert(ee[i]==original[i+4]);
 assert(ee[1]==original[1] && ee[5]==original[5] && ee[6]==original[6]);
 // Cut every byte, including copying, committing the journal, shifting rows and writing the layout marker.
 for(int c=0;c<=total;c++){
  memcpy(ee,original,sizeof ee);cut=c;writes=0;
  if(!setjmp(reboot))boot();
  cut=-1;writes=0;boot();
  for(int i=0;i<13;i++)assert(ee[i]==expected[i]);
  assert(!_layout2_journal_ready());
 }
 // Status references before, within, and after both removed ranges.
 assert(_layout2_index(6,7,11)==6);
 assert(_layout2_index(7,7,11)==NO_MATCH);
 assert(_layout2_index(9,7,11)==NO_MATCH);
 assert(_layout2_index(14,7,11)==NO_MATCH);
 assert(_layout2_index(16,7,11)==12);
 assert(_layout2_index((uint32_t)NO_MATCH,7,11)==NO_MATCH);
 assert(_layout2_index(65543,7,11)==NO_MATCH);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            c, exe = Path(tmp)/'migration.c', Path(tmp)/'migration'
            c.write_text(code)
            subprocess.run(['cc','-std=c99','-fsanitize=address,undefined',str(c),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
