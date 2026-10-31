"""Exercise the USART line reader with bytewise input and an oversized caller buffer."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]

class CharLineLimit(unittest.TestCase):
    def test_bounded_receive_and_recovery(self):
        source=(ROOT/'firmware/tinyg/xio/xio_usart.c').read_text()
        code=r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#define NUL 0
#define CR 13
#define LF 10
#define RX_BUFFER_SIZE 64
#define min(a,b) ((a)<(b)?(a):(b))
#define advance_buffer(i,n) ((i)=((i)+1)%(n))
enum {XIO_OK=0,XIO_EAGAIN,XIO_BUFFER_EMPTY,XIO_BUFFER_FULL,XIO_EOL,XIO_SIG_EOL,XIO_SIG_OK,XIO_LINE_DAMAGED};
typedef struct {uint8_t rx_buf_head,rx_buf_tail;unsigned rx_buf_count;uint8_t rx_flush_mark,rx_flush_marked,rx_lost_first,rx_lost_last,rx_lost,rx_lost_after_mark,rx_lost_post_first,rx_lost_open,rx_line_damaged,rx_damage_next;char rx_buf[64];} xioUsart_t;
typedef struct dev {xioUsart_t *x;uint8_t len;int size,signal;char *buf;bool flag_in_line,flag_echo;
 void (*x_flow)(struct dev*);int (*x_putc)(char,FILE*);} xioDev_t;
void flow(xioDev_t *d){(void)d;}
'''
        code+=function(source,r'static int _gets_helper\([^;{}]*\)(?=\s*\{)')
        code+=function(source,r'int xio_gets_usart\([^;{}]*\)(?=\s*\{)')
        code+=r'''
static xioUsart_t rx;static xioDev_t d;static char buf[1000];
static int push(char c){advance_buffer(rx.rx_buf_head,RX_BUFFER_SIZE);rx.rx_buf[rx.rx_buf_head]=c;rx.rx_buf_count++;return xio_gets_usart(&d,buf,sizeof(buf));}
int main(void){
 d.x=&rx;d.x_flow=flow;d.x_putc=fputc;
 unsigned lengths[]={254,255,256,270,512,1200};
 for(unsigned t=0;t<sizeof(lengths)/sizeof(*lengths);t++){
  for(unsigned i=0;i<lengths[t];i++)assert(push(i<10?'A':'B')==XIO_EAGAIN);
  assert(push('\n')==(lengths[t]<255?XIO_OK:XIO_BUFFER_FULL));
  assert(buf[0]=='A');assert(strlen(buf)==254);
  const char *q="{fv:null}";for(const char *p=q;*p;p++)assert(push(*p)==XIO_EAGAIN);
  assert(push('\n')==XIO_OK);assert(!strcmp(buf,q));
 }
}
'''
        # The real reader never echoes here; match its char/FILE callback signature.
        code=code.replace('d.x_putc=fputc;', 'd.x_putc=NULL;')
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'check.c').write_text(code)
            subprocess.run(['cc','-std=c99','-fsanitize=address,undefined',str(p/'check.c'),'-o',str(p/'check')],check=True)
            subprocess.run([str(p/'check')],check=True)
