"""Public override settings are ordinary RAM-only configuration entries."""
from pathlib import Path
import re
import unittest

FW=Path(__file__).resolve().parents[2]/'firmware/tinyg'

class OverrideNames(unittest.TestCase):
    def test_public_rows_are_readable_in_text_and_never_initialized_or_persisted(self):
        source=(FW/'config_app.c').read_text()
        rows={m[1]:m[2].split(',') for m in re.finditer(
            r'\{\s*""\s*,\s*"([^"]+)"\s*,([^}\n]+)',source)}
        for token in ('mfo','mfoe','m48e','mfc','mcp','mfe'):
            with self.subTest(token=token):
                row=[v.strip() for v in rows[token]]
                self.assertIn(row[0],('_f0','_i0'))
                self.assertNotEqual(row[2],'tx_print_nul')
                if token in ('mfo','mfoe','m48e'):
                    self.assertNotEqual(row[4],'set_nul')
                else:self.assertEqual(row[4],'set_nul')
        # These flags are the actual persistence/default-init contract.
        header=(FW/'config.h').read_text()
        for flag in ('_f0','_i0'):
            definition=re.search(r'#define\s+'+flag+r'\s+([^\n]+)',header)[1]
            self.assertNotIn('F_PERSIST',definition)
            self.assertNotIn('F_INITIALIZE',definition)

    def test_real_flat_text_and_rt_dispatch_never_writes_eeprom(self):
        import subprocess,tempfile
        from test_override_factor_parser import source
        from source_extract import function
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)
            code=source(out)
            code=code.replace('!strcmp(full,names[i]) || !strcmp(t,names[i])','!strcmp(full,names[i]) || (!*g && !strcmp(t,names[i]))')
            code=code.replace('"rt", "rtmfo"};','"rt", "rtmfo", "mfo", "mfoe", "m48e", "mfe", "rtmfe"};')
            code=code.replace('TYPE_NULL, TYPE_FLOAT};','TYPE_NULL, TYPE_FLOAT, TYPE_FLOAT, TYPE_INTEGER, TYPE_INTEGER, TYPE_INTEGER, TYPE_INTEGER};')
            code=code.replace('return i==11?ov_get_mfo:capture_get;',
                              'return i==13?ov_get_mfoe:i==14?ov_get_m48e:(i==15 || i==16)?rt_get_mfe:(i==11 || i==12)?ov_get_mfo:capture_get;')
            code=code.replace('static fptrCmd table_get', 'stat_t ov_get_mfoe(nvObj_t*);stat_t ov_get_m48e(nvObj_t*);stat_t rt_get_mfe(nvObj_t*);stat_t ov_set_mfoe(nvObj_t*);stat_t ov_set_m48e(nvObj_t*);stat_t rt_set_mfe(nvObj_t*);\nstatic fptrCmd table_get')
            code=code.replace('stat_t rt_set_grp(nvObj_t *);stat_t rt_set_mfo(nvObj_t *);',
                              'stat_t rt_set_grp(nvObj_t *);stat_t rt_set_mfo(nvObj_t *);stat_t ov_set_mfo(nvObj_t *);')
            code=code.replace('return i==10?rt_set_grp:i==11?tracked_set:',
                              'return i==13?ov_set_mfoe:i==14?ov_set_m48e:i==15?set_nul:i==16?rt_set_mfe:i==12?ov_set_mfo:i==10?rt_set_grp:i==11?tracked_set:')
            persist=function((FW/'config.c').read_text(),r'stat_t nv_persist\(')
            code=code.replace('stat_t nv_persist(nvObj_t *n) { (void)n;return STAT_OK; }',
                'static unsigned writes;\nbool nv_index_lt_groups(index_t i){return nv_index_is_single(i);}\n'
                'stat_t write_persistent_value(nvObj_t*n){writes++;return STAT_OK;}\n'+persist)
            code+='''
#include <errno.h>
#define memcpy_P memcpy
#define strcpy_P strcpy
#define TEXT_RESPONSE 0
#define JSON_RESPONSE 0
const cfgItem_t cfgArray[17]={{0}};
static volatile uint8_t ov_operator_enables=3;
static uint8_t SREG=128;static void cli(void){SREG=0;}
void sr_run_text_status_report(void){}
void help_general(nvObj_t*n){}
void nv_print_list(stat_t s,uint8_t a,uint8_t b){}
'''
            runtime=(FW/'runtime.c').read_text()
            for name in ('_ov_operator','ov_set_mfoe','ov_set_m48e','ov_get_mfoe','ov_get_m48e','rt_set_mfe','rt_get_mfe','rt_override_reset_job','rt_init'):
                code+=function(runtime,r'(?:static )?(?:stat_t|void) '+name+r'\(')+'\n'
            code+='''
void rt_job_end(void){}
void spindle_reset(void){}
void cm_coolant_off_immediate(void){}
void mp_set_runtime_work_offset(float *offset){}
static void _exec_program_finalize(float *value,bool *flags){}
'''
            canonical=(FW/'canonical_machine.c').read_text()
            for name in ('cm_request_job_kill','_exec_program_end'):
                code+=function(canonical,r'(?m)^(?:static )?void '+name+r'\([^;{}]*\)\s*(?=\{)')+'\n'
            text=(FW/'text_parser.c').read_text()
            for name in ('_text_parser_kernal','text_parser'):
                code+=function(text,r'(?m)^(?:static )?stat_t '+name+r'\([^;{}]*\)\s*(?=\{)')+'\n'
            code+=r'''
static void json(const char*s,int expected){char line[100];strcpy(line,s);json_parser(line);if(last_status!=expected)fprintf(stderr,"input %s: got %d expected %d\n",s,last_status,expected);assert(last_status==expected);}
static void txt(const char*s,int expected){char line[100];strcpy(line,s);assert(text_parser(line)==expected);}
static void factor(unsigned value){ovSnapshot_t s;ov_snapshot(&s);assert(s.feed==value);}
int main(void){
 reset();cm.machine_state=MACHINE_READY;
 json("{\"mfo\":0.05}",0);factor(50);
 json("{\"rt\":{\"mfo\":1.5}}",0);factor(1500);
 txt("$mfo=0.5",0);factor(500);
 txt("$mfo",0);json("{\"mfo\":null}",0);factor(500);
 txt("$mfo=0.049",110);factor(500);
 json("{\"mfo\":2.001}",110);factor(500);
 json("{\"mfo\":2.0}",0);factor(2000);
 ov_set_program_feed(false);txt("$mfo=0.75",0);
 ovSnapshot_t s;ov_snapshot(&s);assert(!s.program_feed);factor(750);
 cm.machine_state=MACHINE_ALARM;
 json("{\"mfo\":null}",0);json("{\"mfo\":0.5}",204);
 txt("$mfo",0);txt("$mfo=0.5",204);factor(750);
 cm.machine_state=MACHINE_READY;
 cm.cycle_state=CYCLE_MACHINING; // non-persistent settings remain writable during a job
 txt("$mfo=0.5",0);json("{\"mfo\":1.5}",0);
 for(unsigned program=0;program<2;program++)for(unsigned master=0;master<2;master++)for(unsigned feed=0;feed<2;feed++){
  ov_set_program_feed(program);
  char line[80];sprintf(line,"$m48e=%u",master);txt(line,0);
  sprintf(line,"{\"mfoe\":%u}",feed);json(line,0);
  nvObj_t n={0};ov_get_m48e(&n);assert(n.value_int==master);
  ov_get_mfoe(&n);assert(n.value_int==feed);
  rt_get_mfe(&n);assert(n.value_int==(program && master && feed));
  ov_snapshot(&s);assert(!!(s.enables&OV_FEED)==(master && feed));
  sprintf(line,"{\"rt\":{\"mfe\":%u}}",!feed);json(line,0);
  ov_get_mfoe(&n);assert(n.value_int==!feed);ov_get_m48e(&n);assert(n.value_int==master);
  rt_get_mfe(&n);assert(n.value_int==(program && master && !feed));
 }
 json("{\"mfoe\":2}",110);txt("$m48e=-1",110);
 json("{\"mfe\":1}",104);
 for(unsigned state=0;state<3;state++){
  cm.machine_state=(int[]){MACHINE_ALARM,MACHINE_SHUTDOWN,MACHINE_PANIC}[state];
  int refused=cm_is_alarmed();assert(refused!=0);
  json("{\"mfoe\":null,\"m48e\":null,\"mfe\":null}",0);
  json("{\"mfoe\":1}",refused);json("{\"m48e\":1}",refused);
  txt("$mfoe",0);txt("$m48e",0);txt("$mfoe=1",refused);txt("$m48e=1",refused);
 }
 // Real kill and queued program-end callbacks reset interface state with the engine.
 for(unsigned master=0;master<2;master++)for(unsigned feed=0;feed<2;feed++)for(unsigned ending=0;ending<4;ending++){
  rt_init();cm.machine_state=MACHINE_READY;nvObj_t n={.valuetype=TYPE_INTEGER,.value_int=master};
  assert(!ov_set_m48e(&n));n.value_int=feed;assert(!ov_set_mfoe(&n));
  ov_set_program_feed(false);ov_set_factor(OV_FEED,.5f,true,true);ov_arm(true,true,true,true);
  if(ending==0)cm_request_job_kill();
  else if(ending==3)rt_init();
  else{float value[6]={MACHINE_PROGRAM_END,1};bool flags[6]={true};_exec_program_end(value,flags);}
  ov_get_m48e(&n);assert(n.value_int==1);ov_get_mfoe(&n);assert(n.value_int==1);
  rt_get_mfe(&n);assert(n.value_int==1);ov_snapshot(&s);
  assert(s.feed==1000 && s.committed_feed==1000 && !s.armed && SREG==128);
 }
 assert(writes==0);
}
'''
            c=out/'names.c';c.write_text(code);exe=out/'names'
            subprocess.run(['cc','-std=gnu99','-funsigned-char','-Os','-I'+str(out),'-I'+str(FW),
                            str(c),str(FW/'override.c'),'-lm','-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
