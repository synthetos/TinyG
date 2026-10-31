"""Check JSON string escaping and every output-capacity boundary under ASan."""
import json
import pathlib
import subprocess
import tempfile
import unittest
from source_extract import function
ROOT=pathlib.Path(__file__).resolve().parents[2]

class JsonStringEscape(unittest.TestCase):
    def test_ascii_roundtrip_and_capacity(self):
        source=(ROOT/'firmware/tinyg/util.c').read_text()
        append=function(source,r'char \*strcat_string\([^;{}]*\)\s*\n')
        call='strcat_string(dst,src,capacity)' if 'size_t capacity' in append else 'strcat_string(dst,src)'
        code=r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define NUL 0
'''+append+'\nstatic char *append(char *dst,const char *src,size_t capacity) { return '+call+';}\n'+r'''
static unsigned encoded_size(const char *s) {
    unsigned n=3; /* two quotes and NUL */
    while(*s) {unsigned char c=*s++;n+=c<32?6:(c=='"'||c=='\\')?2:1;}
    return n;
}
static void check(const char *src) {
    unsigned need=encoded_size(src);
    for(unsigned cap=0;cap<=need+1;cap++) {
        char *dst=malloc(cap?cap:1);memset(dst,0xa5,cap?cap:1);
        char *end=append(dst,src,cap);
        if(cap<need) assert(end==NULL);
        else {assert(end==dst+need-1);assert(*end==0);}
        if(!cap)assert((unsigned char)dst[0]==0xa5);
        free(dst);
    }
    char dst[1024];assert(append(dst,src,sizeof(dst))!=NULL);puts(dst);
}
int main(void) {
    check("");check("say \"hi\" \\ path");
    for(unsigned c=1;c<128;c++){char src[2]={(char)c,0};check(src);}
    char src[128];for(unsigned i=0;i<127;i++)src[i]=i+1;src[127]=0;check(src);
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            src=pathlib.Path(tmp)/'escape.c';exe=pathlib.Path(tmp)/'escape';src.write_text(code)
            subprocess.run(['cc','-std=c99','-Wall','-Wextra','-Wno-unused-parameter','-fsanitize=address,undefined',str(src),'-o',str(exe)],check=True)
            result=subprocess.run([str(exe)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            expected=['','say "hi" \\ path']+[chr(c) for c in range(1,128)]+[''.join(chr(c) for c in range(1,128))]
            self.assertEqual([json.loads(line) for line in result.stdout.splitlines()],expected)
if __name__=='__main__': unittest.main()
