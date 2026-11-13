"""Real JSON/config/runtime dispatch, including AVR-libc decimal conversion."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import input_harness as h
from source_extract import function

WORDS = ['0.05', '0.050', '5e-2', '0.051', '2.0', '0.049', '0.04999999', '2.001']
EXPECTED = [50, 50, 50, 51, 2000, 1000, 1000, 1000]
STATUSES = [0, 0, 0, 0, 0, 110, 110, 110]


def source(out):
    fixture=(h.FIXTURES/'json_prelude.h').read_text()
    fixture='#include "override.h"\n'+fixture
    fixture=fixture.replace('"sr", "js"};','"sr", "js", "rt", "rtmfo"};')
    fixture=fixture.replace('TYPE_TXTCON, TYPE_INTEGER, TYPE_NULL, TYPE_INTEGER};',
                            'TYPE_TXTCON, TYPE_INTEGER, TYPE_NULL, TYPE_INTEGER, TYPE_NULL, TYPE_FLOAT};')
    fixture=fixture.replace('return !strcmp(t,"1");','return !strcmp(t,"1") || !strcmp(t,"rt");')
    fixture=fixture.replace('static fptrCmd table_get(index_t i) { (void)i;return capture_get; }',
        'stat_t ov_get_mfo(nvObj_t *);\nstatic fptrCmd table_get(index_t i) { return i==11?ov_get_mfo:capture_get; }')
    fixture=fixture.replace('static fptrCmd table_set(index_t i) { return i ? capture_set : set_nul; }',
        '''stat_t rt_set_grp(nvObj_t *);stat_t rt_set_mfo(nvObj_t *);
static uint32_t parsed_bits;
static uint8_t factor_precision;
static stat_t tracked_set(nvObj_t *nv){memcpy(&parsed_bits,&nv->value_flt,4);stat_t status=rt_set_mfo(nv);factor_precision=nv->precision;return status;}
static fptrCmd table_set(index_t i){return i==10?rt_set_grp:i==11?tracked_set:i?capture_set:set_nul;}''')
    fixture=fixture.replace('bool allowed, fl_taken;', 'bool allowed, fl_taken, check_only;')
    runtime=(h.FW/'runtime.c').read_text()
    wrappers=''.join(function(runtime,r'(?:static )?stat_t '+name+r'\(') for name in
                     ('_ov_result','_ov_float','ov_get_mfo','ov_get_sso','_ov_factor','ov_set_mfo','rt_set_mfo','rt_set_grp'))
    fixture=fixture.replace('static stat_t rt_set_grp(nvObj_t *n) { (void)n; return STAT_OK; }',wrappers)
    fixture=fixture.replace('static void reset(void) {','static void reset(void) {ov_init();parsed_bits=0;')
    (out/'json_prelude.h').write_text(fixture)
    original_run=subprocess.run
    def compile_with_override(cmd,**kw):
        if str(cmd[0]).endswith('cc'): cmd=list(cmd)+[str(h.FW/'override.c')]
        return original_run(cmd,**kw)
    with patch.object(h,'FIXTURES',out), patch.object(h.subprocess,'run',compile_with_override):
        h.build('json',out)
    code=(out/'json.c').read_text()
    code=code[:code.index('#include <signal.h>')]
    exercise=function(code,r'static int exercise\(')
    code=code.replace(exercise,'''static int exercise(char *s,unsigned n) {
 reset();cm.machine_state=MACHINE_READY;json_parser(s);return last_status;
}''')
    return code


class OverrideFactorParser(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.out=Path(self.tmp.name)
        self.code=source(self.out)

    def test_native_real_parser(self):
        main='''int main(void){char line[80];ovSnapshot_t state;const char *words[]={'''+','.join('"'+s+'"' for s in WORDS)+'''};
for(unsigned i=0;i<sizeof(words)/sizeof(*words);i++){
snprintf(line,sizeof(line),"{\\"rt\\":{\\"mfo\\":%s}}",words[i]);
int status=exercise(line,strlen(line));ov_snapshot(&state);
if(factor_precision!=3)return 99;
printf("%d %u\\n",status,state.feed);}return 0;}'''
        c=self.out/'factor.c';c.write_text(self.code+main);exe=self.out/'factor'
        subprocess.run(['cc','-std=gnu99','-funsigned-char','-Os','-I'+str(self.out),'-I'+str(h.FW),str(c),str(h.FW/'override.c'),'-lm','-o',str(exe)],check=True)
        rows=[list(map(int,line.split())) for line in subprocess.check_output([str(exe)],text=True).splitlines()]
        self.assertEqual(rows,list(map(list,zip(STATUSES,EXPECTED))))

    def test_avr_libc_real_parser(self):
        if not all(shutil.which(t) for t in ('avr-gcc','avr-nm','pkg-config')):
            self.skipTest('AVR compiler and simavr development library required')
        flags=subprocess.run(['pkg-config','--cflags','--libs','simavr','libelf'],capture_output=True,text=True)
        if flags.returncode:self.skipTest('simavr development library required')
        # No device registers are exercised. ATmega1284p supplies enough SRAM
        # for the real NV/parser structs and runs the same AVR-libc float code.
        main='''volatile uint32_t results[24];volatile uint8_t finished;
int main(void){char line[80];ovSnapshot_t state;const char *words[]={'''+','.join('"'+s+'"' for s in WORDS)+'''};
for(uint8_t i=0;i<8;i++){
sprintf(line,"{\\"rt\\":{\\"mfo\\":%s}}",words[i]);
results[3*i]=exercise(line,strlen(line));ov_snapshot(&state);
results[3*i+1]=state.feed;results[3*i+2]=parsed_bits;}
finished=1;for(;;); }'''
        c=self.out/'factor-avr.c';c.write_text(self.code+main);elf=self.out/'factor.elf'
        subprocess.run(['avr-gcc','-mmcu=atmega1284p','-std=gnu99','-funsigned-char','-Os','-I'+str(self.out),'-I'+str(h.FW),str(c),str(h.FW/'override.c'),'-lm','-o',str(elf)],check=True)
        symbols=subprocess.check_output(['avr-nm',str(elf)],text=True)
        def addr(name):return int(re.search(r'^([0-9a-f]+) [A-Za-z] '+name+'$',symbols,re.M)[1],16)-0x800000
        runner=self.out/'runner.c';exe=self.out/'runner'
        runner.write_text('''#include <stdio.h>
#include <stdlib.h>
#include "sim_avr.h"
#include "sim_elf.h"
int main(int argc,char **argv){elf_firmware_t f={0};elf_read_firmware(argv[1],&f);
avr_t *a=avr_make_mcu_by_name("atmega1284p");avr_init(a);a->frequency=16000000;avr_load_firmware(a,&f);
unsigned done=strtoul(argv[3],0,10);unsigned long steps=0;
while(!a->data[done] && steps++<20000000)avr_run(a);
if(!a->data[done])return 2;
unsigned addr=strtoul(argv[2],0,10);printf("RESULT ");
for(unsigned i=0;i<96;i++)printf("%02x",a->data[addr+i]);puts("");return 0;}''')
        subprocess.run(['cc',str(runner),*flags.stdout.split(),'-o',str(exe)],check=True)
        output=subprocess.check_output([str(exe),str(elf),str(addr('results')),str(addr('finished'))],text=True,timeout=20)
        raw=bytes.fromhex(re.search(r'RESULT ([0-9a-f]+)',output)[1])
        rows=[[int.from_bytes(raw[i+j:i+j+4],'little') for j in (0,4,8)] for i in range(0,96,12)]
        print('AVR parser [status, factor*1000, parsed binary32]:',rows)
        self.assertEqual([row[:2] for row in rows],list(map(list,zip(STATUSES,EXPECTED))))
