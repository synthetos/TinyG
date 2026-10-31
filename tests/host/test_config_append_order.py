"""Exercise the real AVR-preprocessed singles predicate with every append order.

The fixtures retain the reviewed v2/optional-stop rows so the test also works
before those branches merge, without depending on unmerged Git refs in CI.
"""
from pathlib import Path
import itertools
import json
import re
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT=Path(__file__).resolve().parents[2]
FW=ROOT/'firmware/tinyg'


class ConfigAppendOrder(unittest.TestCase):
    def test_all_appended_persisted_rows_remain_single(self):
        source=(FW/'config_app.c').read_text()
        fixture=json.loads((Path(__file__).parent/'fixtures/appended_config_rows.json').read_text())
        legacy_tokens={'net','fd','ma'}
        fixture_tokens={'spep','spdp','spph','spde','coph','prbp','fstk','adc3','os'}
        def token(line):
            m=re.match(r'\s*\{\s*"[^"]*"\s*,\s*"([^"]*)"',line)
            return m.group(1) if m else None
        legacy=[line for line in source.splitlines() if token(line) in legacy_tokens]
        self.assertEqual(len(legacy),3)
        clean='\n'.join(line for line in source.splitlines() if token(line) not in legacy_tokens|fixture_tokens)
        marker='// Group lookups - must follow'
        blocks={'legacy':legacy,'v2':fixture['v2'],'optional_stop':fixture['optional_stop']}
        for order in itertools.permutations(blocks):
            with self.subTest(order=order):
                added='\n'.join(row for name in order for row in blocks[name])+'\n'
                combined=clean.replace(marker,added+marker)
                pre=subprocess.check_output(['avr-gcc','-mmcu=atxmega192a3','-DF_CPU=32000000UL',
                    '-I',str(FW),'-E','-P','-x','c','-'],input=combined,text=True)
                table=pre.split('const cfgItem_t cfgArray[]',1)[1].split('\n};',1)[0]
                rows=[row.split(',') for row in re.findall(r'\{(\s*"[^{}]+)\}',table)]
                self.assertGreater(len(rows),300)
                first_group=next(i for i,r in enumerate(rows) if r[1].strip()=='"sys"')
                enum=re.search(r'typedef enum\s*\{[^{}]*\}\s*valueType\s*;',pre).group()
                code='#include <stdbool.h>\n#include <stdint.h>\n#include <stdio.h>\n#include <string.h>\n'+enum
                code+='\ntypedef uint16_t index_t;\ntypedef struct {const char *token;uint8_t flags;} cfgItem_t;\n'
                code+='static const cfgItem_t cfgArray[]={\n'
                code+='\n'.join('{'+r[1]+','+r[2]+'},' for r in rows)+'\n};\n'
                code+=function(pre,r'(?:_Bool|bool) nv_index_is_single\([^;{}]*\)(?=\s*\{)')
                code+='\nint main(void){int failures=0;\n'
                code+=f'for(unsigned i=0;i<sizeof(cfgArray)/sizeof(*cfgArray);i++){{bool expected=i<{first_group};\n'
                code+='if(nv_index_is_single(i)!=expected){fprintf(stderr,"wrong singles classification: %s index=%u\\n",cfgArray[i].token,i);failures++;}\n'
                code+='if((cfgArray[i].flags&0x30)==0x30 && !nv_index_is_single(i)){fprintf(stderr,"boot load excludes persisted row: %s\\n",cfgArray[i].token);failures++;}\n}\nreturn failures?1:0;}\n'
                with tempfile.TemporaryDirectory() as tmp:
                    p=Path(tmp)/'table.c';exe=Path(tmp)/'table';p.write_text(code)
                    subprocess.run(['cc','-std=c99','-Wall','-Werror','-fsanitize=address,undefined',str(p),'-o',str(exe)],check=True)
                    result=subprocess.run([str(exe)],capture_output=True,text=True)
                    self.assertEqual(result.returncode,0,result.stderr)
