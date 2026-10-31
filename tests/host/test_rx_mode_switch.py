"""Run both actual readers and the USART line assembler against a simulated RX ring."""
from pathlib import Path
import os
import re
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT=Path(__file__).resolve().parents[2]
FIRMWARE=Path(os.environ.get('TINYG_FIRMWARE', ROOT/'firmware/tinyg'))


class ReceiveModeSwitch(unittest.TestCase):
    def test_buffer_ownership(self):
        source=(FIRMWARE/'xio.c').read_text()
        usart=(FIRMWARE/'xio/xio_usart.c').read_text()
        names=['_init_readline_linemode','_init_readline_charmode','_get_free_buffer',
               '_get_filling_buffer','_post_buffer','_post_overflow','_next_buffer_to_process',
               '_free_processed_buffer','_exit_line','_exit_null','_readline_source',
               '_readline_linemode','_readline_charmode','readline']
        definitions=[function(source, r"static bool _json_space\(char c\)(?=\s*\{)"),
                     function(source, r"static bool _json_wraps_gcode\(const char \*line, const char \*end\)(?=\s*\{)"),
                     function(source, r"static devflags_t _classify_line\(const char \*line, uint16_t size\)(?=\s*\{)")]
        for name in ['_gets_helper','xio_gets_usart']+names:
            text=usart if name in ['_gets_helper','xio_gets_usart'] else source
            signature=r'(?m)^(?:static )?(?:void|int|stat_t|char \*)\s*'+name+r'\([^;{}\n]*\)\s*'
            match=re.search(signature+r'\{',text)
            self.assertIsNotNone(match,name)
            definitions.append(function(text[match.start():],signature))
        code=r'''
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#define NUL 0
#define CR 13
#define LF 10
#define RX_HEADERS 12
#define RX_BUFFER_MIN_SIZE 200
#define RX_BUFFER_POOL_SIZE 1000
#define RX_BUFFER_SIZE 512
#define RX_MODE_CHAR 0
#define RX_MODE_LINE 1
#define XIO_DEV_USB 0
#define XIO_DEV_PGM 2
#define DEV_IS_NONE 0
#define DEV_IS_CTRL 1
#define DEV_IS_DATA 2
#define DEV_IS_BOTH 3
#define DEV_IS_OVERFLOW 8
#define TEXT_MODE 0
#define PSTR(x) x
#define strchr_P strchr
#define strcpy_P strcpy
#define pgm_read_byte(p) (*(p))
#define min(a,b) ((a)<(b)?(a):(b))
#define printf_P printf
#define _FDEV_ERR -1
#define advance_buffer(p,n) do { if(--(p)==0) (p)=(n)-1; } while(0)
enum {XIO_OK=0,XIO_EAGAIN=1,XIO_BUFFER_EMPTY=2,XIO_BUFFER_FULL=3,XIO_EOL=4,STAT_EOF=5,XIO_SIG_OK=6,XIO_SIG_EOL=7,XIO_EOF=8};
enum {BUFFER_FREE,BUFFER_FRAGMENT,BUFFER_FILLING,BUFFER_FULL,BUFFER_PROCESSING};
typedef uint8_t devflags_t;
typedef int stat_t;
typedef struct hdr { uint8_t bufnum; struct hdr *pv,*nx; int state;
  uint8_t flags; uint16_t size; char *bufp; } buf_hdr_t;
typedef struct { buf_hdr_t *used_base,*used_top; uint16_t requested_size;
  uint8_t fragments,free_headers,out_of_ram; char *pool_base,*pool_top;
  buf_hdr_t buf[RX_HEADERS]; } buf_mgr_t;
static buf_mgr_t bm;
struct { char rx_pool[RX_BUFFER_POOL_SIZE]; } bufpool;
struct { uint8_t rx_mode,buf_state,primary_src; uint16_t buf_size; char *bufp; } xio;
static uint8_t active_rx_mode;
struct { int comm_mode; } cs;
typedef struct { unsigned rx_buf_head,rx_buf_tail,rx_buf_count,rx_flush_mark; bool rx_flush_marked; char rx_buf[RX_BUFFER_SIZE]; } xioUsart_t;
typedef struct dev { xioUsart_t *x; bool flag_in_line,flag_echo; int size,signal; uint8_t len;
 char *buf; void (*x_flow)(struct dev*); int (*x_putc)(char,FILE*); } xioDev_t;
xioUsart_t ring;
xioDev_t ds[1];
int xio_gets_usart(xioDev_t *,char *,const int);
int xio_gets(uint8_t src,char *buf,int size) { return xio_gets_usart(&ds[src],buf,size); }
void controller_reset_source(void) {}
void rpt_exception(int s,const char *m) { assert(0); }
void flow(xioDev_t *d) {}
static char *_readline_charmode(devflags_t*,uint16_t*);
static char *_readline_linemode(devflags_t*,uint16_t*);
'''+ '\n'.join(definitions)+r'''
static void setup(int mode) {
    memset(&bm,0,sizeof(bm)); memset(&bufpool,0,sizeof(bufpool));
    memset(&xio,0,sizeof(xio)); memset(&ds,0,sizeof(ds)); memset(&ring,0,sizeof(ring));
    ring.rx_buf_head=ring.rx_buf_tail=1; ds[0].x=&ring; ds[0].x_flow=flow;
    _init_readline_linemode(); _init_readline_charmode();
    active_rx_mode=mode; xio.rx_mode=mode;
    devflags_t flags=DEV_IS_CTRL; uint16_t size=0; assert(readline(&flags,&size)==NULL);
}
static void input(const char *s) {
    while(*s) { advance_buffer(ring.rx_buf_head,RX_BUFFER_SIZE);
        assert(ring.rx_buf_head!=ring.rx_buf_tail); ring.rx_buf[ring.rx_buf_head]=*s++; ring.rx_buf_count++; }
}
static char *read_flags(devflags_t flags) { uint16_t size=0; char *p=readline(&flags,&size); assert(p!=(char*)_FDEV_ERR); if(p) assert(size==strlen(p)+1); else assert(size==0); return p; }
static void expect(const char *text) {
    char *p=read_flags(DEV_IS_BOTH); assert(p); if(strcmp(p,text)) fprintf(stderr,"wanted %s, got %s\n",text,p);
    assert(!strcmp(p,text));
}
int main(void) {
    setup(RX_MODE_LINE);
    input("{rxm:0}\n{fb:null}\n{fv:null}\n"); expect("{rxm:0}");
    xio.rx_mode=RX_MODE_CHAR;
    expect("{fb:null}"); expect("{fv:null}"); assert(!read_flags(DEV_IS_BOTH));
    input("{rxm:1}\n{fb:null}\n"); expect("{rxm:1}"); xio.rx_mode=RX_MODE_LINE;
    expect("{fb:null}"); assert(!read_flags(DEV_IS_BOTH));
    // A partial line lives beyond the current processing line; moving it must use memmove.
    setup(RX_MODE_LINE); input("{rxm:0}\n{fv:"); expect("{rxm:0}");
    assert(ds[0].flag_in_line && ds[0].len==4 && ds[0].buf!=bufpool.rx_pool);
    xio.rx_mode=RX_MODE_CHAR; assert(!read_flags(DEV_IS_BOTH));
    input("null}\n"); expect("{fv:null}"); assert(!read_flags(DEV_IS_BOTH));
    // Posted DATA is retained until the controller asks for DATA, despite a pending mode switch.
    setup(RX_MODE_LINE); input("G1 X1\n{rxm:0}\n{fb:");
    char *p=read_flags(DEV_IS_CTRL); assert(p && !strcmp(p,"{rxm:0}")); xio.rx_mode=RX_MODE_CHAR;
    assert(!read_flags(DEV_IS_CTRL)); expect("G1 X1");
    input("null}\n"); expect("{fb:null}"); assert(!read_flags(DEV_IS_BOTH));
    // An overflow posted in line mode keeps its error flag when the queued mode change drains it.
    setup(RX_MODE_LINE);
    char overlong[212]; memset(overlong,' ',210); overlong[210]='\n'; overlong[211]=0;
    input("{rxm:0}\n"); input(overlong); input("{fb:null}\n");
    expect("{rxm:0}"); xio.rx_mode=RX_MODE_CHAR;
    devflags_t overflow_flags=DEV_IS_BOTH; uint16_t overflow_size=0;
    p=readline(&overflow_flags,&overflow_size);
    assert(p && (overflow_flags & DEV_IS_OVERFLOW) && overflow_size==strlen(p)+1);
    expect("{fb:null}"); assert(!read_flags(DEV_IS_BOTH));
    // Retained DATA and partial lines also survive a character-to-line switch.
    setup(RX_MODE_CHAR); input("G1 X2\n"); assert(!read_flags(DEV_IS_CTRL));
    xio.rx_mode=RX_MODE_LINE; assert(!read_flags(DEV_IS_CTRL)); expect("G1 X2");
    input("{fb:null}\n"); expect("{fb:null}"); assert(!read_flags(DEV_IS_BOTH));
    setup(RX_MODE_CHAR); input("{fv:"); assert(!read_flags(DEV_IS_BOTH)); xio.rx_mode=RX_MODE_LINE;
    input("null}\n"); expect("{fv:null}"); input("{fb:null}\n"); expect("{fb:null}");
    assert(!read_flags(DEV_IS_BOTH));
    // Returned text remains valid until the *next* read, even if rxm changes during parsing.
    setup(RX_MODE_LINE); input("{rxm:0}\n"); p=read_flags(DEV_IS_BOTH); assert(p);
    xio.rx_mode=RX_MODE_CHAR; strcpy(p,"parsed"); assert(!strcmp(p,"parsed"));
    input("{fb:null}\n"); expect("{fb:null}");
    for(int i=0;i<200;i++) {
        input("{rxm:0}\n{fb:null}\n"); expect("{rxm:0}"); xio.rx_mode=0; expect("{fb:null}");
        input("{rxm:1}\n{fv:null}\n"); expect("{rxm:1}"); xio.rx_mode=1; expect("{fv:null}");
        assert(!read_flags(DEV_IS_BOTH));
    }
}
'''
        with tempfile.TemporaryDirectory() as d:
            c=Path(d)/'check.c';c.write_text(code);out=Path(d)/'check'
            subprocess.run(['cc','-std=c99','-g','-fsanitize=address,undefined',str(c),'-o',str(out)],check=True)
            subprocess.run([str(out)],check=True)


if __name__=='__main__': unittest.main()
