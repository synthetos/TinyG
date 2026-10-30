"""Real line allocator/reader with deterministic USB and program-memory devices."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function
from test_linemode_allocator import PRELUDE

ROOT = Path(__file__).resolve().parents[2]

class ProgramFileLineMode(unittest.TestCase):
    def test_file_dispatch_and_source_handoff(self):
        source=(ROOT/'firmware/tinyg/xio.c').read_text()
        code='#define RX_HEADERS 10\n#define RX_BUFFER_POOL_SIZE 1000\n#define RX_BUFFER_MIN_SIZE 200\n'+PRELUDE
        code+=r'''
#define STAT_OK 0
#define STAT_COMMAND_NOT_ACCEPTED 106
#define STAT_EOF 8
#define XIO_EOF 8
#define XIO_OK 0
#define XIO_EAGAIN 2
#define XIO_BUFFER_FULL 3
#define XIO_DEV_USB 0
#define XIO_DEV_PGM 1
#define RX_MODE_LINE 1
#define TEXT_MODE 0
#define PSTR(x) x
#define printf_P printf
#define strcpy_P strcpy
#define min(a,b) ((a)<(b)?(a):(b))
typedef int stat_t;
typedef struct {bool flag_in_line;unsigned len;} xioDev_t;
xioDev_t ds[2];
struct {int rx_mode,primary_src;} xio;
struct {int comm_mode;} cs;
int pgm_reads,usb_reads,eof_reports;
void controller_reset_source(void) {xio.primary_src=XIO_DEV_USB;}
void rpt_exception(int status,const char *msg) {(void)msg;assert(status==STAT_EOF);eof_reports++;}
int xio_gets_usart(xioDev_t *d,char *buf,int size) {
    (void)buf;(void)size;usb_reads++;d->flag_in_line=true;return XIO_EAGAIN;
}
int xio_gets(int dev,char *buf,int size) {
    if(dev==XIO_DEV_USB)return xio_gets_usart(&ds[0],buf,size);
    pgm_reads++;
    if(pgm_reads==1) {strcpy(buf,"G1 X1");return XIO_OK;}
    if(pgm_reads==2) {strcpy(buf,"G1 X2");return XIO_OK;}
    return XIO_EOF; // fgets at EOF leaves the previous bytes unchanged
}
'''
        for name in ('_get_free_buffer','_get_filling_buffer','_post_buffer','_post_overflow','_next_buffer_to_process','_free_processed_buffer','xio_prepare_pgm_source','_readline_source','_readline_linemode'):
            code+=function(source,r'(?:static )?(?:stat_t|char \*|void)\s*'+name+r'\([^;{]*\)(?=\s*\{)')
        code+=r'''
int main(void) {
    bm.pool_base=pool;bm.pool_top=pool+sizeof(pool);
    bm.used_base=bm.used_top=&bm.buf[0];bm.free_headers=RX_HEADERS;
    bm.requested_size=RX_BUFFER_MIN_SIZE;
    for(unsigned i=0;i<RX_HEADERS;i++) {
        bm.buf[i].bufnum=i;bm.buf[i].bufp=pool;
        bm.buf[i].pv=&bm.buf[(i+RX_HEADERS-1)%RX_HEADERS];
        bm.buf[i].nx=&bm.buf[(i+1)%RX_HEADERS];
    }
    xio.rx_mode=RX_MODE_LINE;cs.comm_mode=1;
    ds[0].flag_in_line=true;ds[0].len=3;
    assert(xio_prepare_pgm_source()==STAT_COMMAND_NOT_ACCEPTED);
    assert(ds[0].flag_in_line && ds[0].len==3);
    ds[0].len=0;
    assert(xio_prepare_pgm_source()==STAT_OK && !ds[0].flag_in_line);
    xio.primary_src=XIO_DEV_PGM;
    assert(xio_prepare_pgm_source()==STAT_COMMAND_NOT_ACCEPTED);
    uint16_t size=200;devflags_t flags=DEV_IS_CTRL;
    assert(_readline_linemode(&flags,&size)==NULL);assert(pgm_reads==1);
    flags=DEV_IS_CTRL;
    assert(_readline_linemode(&flags,&size)==NULL);assert(pgm_reads==1);
    flags=DEV_IS_DATA;
    assert(strcmp(_readline_linemode(&flags,&size),"G1 X1")==0);
    flags=DEV_IS_DATA;
    assert(strcmp(_readline_linemode(&flags,&size),"G1 X2")==0);
    flags=DEV_IS_DATA;
    assert(_readline_linemode(&flags,&size)==NULL);
    assert(xio.primary_src==XIO_DEV_USB && eof_reports==1);
    flags=DEV_IS_DATA;
    assert(_readline_linemode(&flags,&size)==NULL);
    assert(pgm_reads==3 && usb_reads>0 && eof_reports==1);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'reader.c';exe=Path(tmp)/'reader';p.write_text(code)
            subprocess.run(['cc','-std=c99','-Wall','-Werror','-fsanitize=address,undefined',str(p),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
