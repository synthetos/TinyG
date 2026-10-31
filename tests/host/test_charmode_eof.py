"""Run the production character reader across stale EOF and USB handoff."""
import pathlib
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT = pathlib.Path(__file__).resolve().parents[2]
PRELUDE = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint8_t devflags_t;
typedef int stat_t;
enum {DEV_IS_NONE=0,DEV_IS_CTRL=1,DEV_IS_DATA=2,DEV_IS_OVERFLOW=8};
enum {BUFFER_FREE,BUFFER_FULL};
enum {STAT_OK,STAT_EOF,XIO_EAGAIN,XIO_BUFFER_FULL,XIO_LINE_DAMAGED};
#define _FDEV_ERR (-1)
#define NUL 0
#define TEXT_MODE 1
#define RX_BUFFER_POOL_SIZE 1000
#define PSTR(s) (s)
static unsigned reports,reads,resets;
#define printf_P(s) (++reports)
static struct {int comm_mode;} cs;
static struct {int buf_state,primary_src;char *bufp;uint16_t buf_size;} xio;
static int next_status;
static void rpt_exception(int status,const char *msg) {
    assert(status==STAT_EOF);assert(!*msg);reports++;
}
static void controller_reset_source(void) {xio.primary_src=0;resets++;}
static int xio_gets(int source,char *buf,unsigned size) {
    (void)size;reads++;
    if(next_status==STAT_OK) {assert(source==0);strcpy(buf,"G1X1");}
    return next_status; /* immediate EOF leaves the previous bytes intact */
}
static devflags_t _classify_line(const char *buf,uint16_t size) {
    (void)size;return *buf=='{'?DEV_IS_CTRL:DEV_IS_DATA;
}
'''
EXERCISE = r'''
int main(void) {
    char buffer[RX_BUFFER_POOL_SIZE];
    const char *stale[]={"N2519M30","M2","{gc", ""};
    for(unsigned mode=0;mode<2;mode++) for(unsigned n=0;n<4;n++)
    for(unsigned accept=DEV_IS_CTRL;accept<=3;accept++) {
        strcpy(buffer,stale[n]); xio.bufp=buffer;xio.buf_state=BUFFER_FREE;
        xio.primary_src=1;cs.comm_mode=mode;reports=reads=resets=0;
        devflags_t flags=accept;uint16_t size=999;next_status=STAT_EOF;
        assert(_readline_charmode(&flags,&size)==NULL);
        assert(flags==DEV_IS_NONE&&size==0&&xio.buf_state==BUFFER_FREE);
        assert(reports==1&&resets==1&&reads==1&&xio.primary_src==0);
        next_status=XIO_EAGAIN;flags=3;
        assert(_readline_charmode(&flags,&size)==NULL);
        assert(reports==1&&resets==1);
        next_status=STAT_OK;flags=DEV_IS_CTRL;
        assert(_readline_charmode(&flags,&size)==NULL);
        assert(xio.buf_state==BUFFER_FULL);
        flags=DEV_IS_DATA;
        assert(_readline_charmode(&flags,&size)==buffer);
        assert(strcmp(buffer,"G1X1")==0&&size==5&&flags==DEV_IS_DATA);
        assert(reads==3&&xio.buf_state==BUFFER_FREE);
    }
    return 0;
}
'''

class CharModeEOF(unittest.TestCase):
    def test_eof_does_not_dispatch_or_hold_previous_line(self):
        source = (ROOT / 'firmware/tinyg/xio.c').read_text()
        code = PRELUDE + '\n'.join(function(source, r'static char \*'+name+r'\([^;]*?\)\s*\n')
                                  for name in ('_exit_line','_exit_null','_readline_charmode')) + EXERCISE
        with tempfile.TemporaryDirectory() as tmp:
            src = pathlib.Path(tmp)/'eof.c'; exe = pathlib.Path(tmp)/'eof'
            src.write_text(code)
            subprocess.run(['cc','-std=c99','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(src),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

if __name__=='__main__': unittest.main()
