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
            code=code.replace('"rt", "rtmfo"};','"rt", "rtmfo", "mfo"};')
            code=code.replace('TYPE_NULL, TYPE_FLOAT};','TYPE_NULL, TYPE_FLOAT, TYPE_FLOAT};')
            code=code.replace('return i==11?ov_get_mfo:capture_get;',
                              'return (i==11 || i==12)?ov_get_mfo:capture_get;')
            code=code.replace('stat_t rt_set_grp(nvObj_t *);stat_t rt_set_mfo(nvObj_t *);',
                              'stat_t rt_set_grp(nvObj_t *);stat_t rt_set_mfo(nvObj_t *);stat_t ov_set_mfo(nvObj_t *);')
            code=code.replace('return i==10?rt_set_grp:i==11?tracked_set:',
                              'return i==12?ov_set_mfo:i==10?rt_set_grp:i==11?tracked_set:')
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
const cfgItem_t cfgArray[13]={{0}};
void sr_run_text_status_report(void){}
void help_general(nvObj_t*n){}
void nv_print_list(stat_t s,uint8_t a,uint8_t b){}
'''
            text=(FW/'text_parser.c').read_text()
            for name in ('_text_parser_kernal','text_parser'):
                code+=function(text,r'(?m)^(?:static )?stat_t '+name+r'\([^;{}]*\)\s*(?=\{)')+'\n'
            code+=r'''
static void json(const char*s,int expected){char line[100];strcpy(line,s);json_parser(line);assert(last_status==expected);}
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
 assert(writes==0);
}
'''
            c=out/'names.c';c.write_text(code);exe=out/'names'
            subprocess.run(['cc','-std=gnu99','-funsigned-char','-Os','-I'+str(out),'-I'+str(FW),
                            str(c),str(FW/'override.c'),'-lm','-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
