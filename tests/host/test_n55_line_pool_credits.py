"""Control overtaking DATA must return its credit when its fragment is reclaimed."""
from pathlib import Path
import os,subprocess,tempfile,re
from source_extract import function
ROOT=Path(__file__).resolve().parents[2]


def test_fragment_reclamation_returns_line_credit():
    source=Path(os.environ.get('N55_XIO_SOURCE',ROOT/'firmware/tinyg/xio.c')).read_text()
    code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define RX_HEADERS 12
#define RX_BUFFER_POOL_SIZE 1000
#define RX_BUFFER_MIN_SIZE 255
#define NUL 0
#define PSTR(s) s
#define strchr_P strchr
#define pgm_read_byte(p) (*(p))
enum {BUFFER_FREE,BUFFER_FRAGMENT,BUFFER_FILLING,BUFFER_FULL,BUFFER_PROCESSING};
enum {DEV_IS_NONE=0,DEV_IS_CTRL=1,DEV_IS_DATA=2,DEV_IS_BOTH=3,DEV_IS_OVERFLOW=8};
typedef uint8_t devflags_t;
typedef struct hdr {uint8_t bufnum;struct hdr *pv,*nx;int state;uint8_t flags;uint16_t size;char *bufp;} buf_hdr_t;
typedef struct {buf_hdr_t *used_base,*used_top;uint16_t requested_size;uint8_t fragments,free_headers,out_of_ram;char *pool_base,*pool_top;buf_hdr_t buf[RX_HEADERS];} buf_mgr_t;
static buf_mgr_t bm;
static struct {char rx_pool[RX_BUFFER_POOL_SIZE];} bufpool;
'''
    for name in ['_json_space','_json_wraps_gcode','_classify_line']:
        if re.search(r'\b'+name+r'\(',source):
            code+=function(source,r'(?:static )?(?:bool|devflags_t) '+name+r'\([^;{}]*\)(?=\s*\{)')+'\n'
    for name,ret in [('_init_readline_linemode','void'),('_get_free_buffer',r'char \*'),
                     ('_get_filling_buffer',r'char \*'),('_post_buffer','void'),
                     ('_next_buffer_to_process',r'char \*'),('_free_processed_buffer','void')]:
        code+=function(source,r'static '+ret+r'\s*'+name+r'\([^;{}]*\)(?=\s*\{)')+'\n'
    code+=function(source,r'uint8_t xio_get_line_buffers_available\(\)(?=\s*\{)')
    code+=r'''
static unsigned actual_free(void) {unsigned n=0;for(int i=0;i<RX_HEADERS;i++)n+=bm.buf[i].state==BUFFER_FREE;return n;}
static void posted(const char *line) {
 char *p=_get_filling_buffer();if(!p)p=_get_free_buffer(RX_BUFFER_MIN_SIZE);
 assert(p);strcpy(p,line);_post_buffer();
}
static void filling(void) {if(!_get_filling_buffer())assert(_get_free_buffer(RX_BUFFER_MIN_SIZE));}
static void take(devflags_t flags,const char *expected) {
 char *p=_next_buffer_to_process(&flags);assert(p&&!strcmp(p,expected));
}
int main(void) {
 const char *ctrl[]={"{\"sr\":null}","{\"qr\":null}","{\"stat\":null,\"hold\":null}",
  "{\"spe\":null,\"sps\":null}","{\"rt\":{\"up\":null}}","{\"rt\":{\"fs\":null}}","{\"rt\":{\"hb\":0}}","$HZ"};
 _init_readline_linemode();filling();
 for(unsigned trial=0;trial<1024;trial++) {
  const char *c=ctrl[trial%(sizeof(ctrl)/sizeof(*ctrl))];
  posted("G4 P0.001");posted(c);posted("G4 P0.002");filling();
  assert(bm.free_headers==actual_free());
  // Planner backpressure holds older DATA while a CTRL is served from the middle.
  take(DEV_IS_CTRL,c);_free_processed_buffer();
  assert(bm.fragments==1 && bm.free_headers==actual_free());
  take(DEV_IS_DATA,"G4 P0.001");_free_processed_buffer();
  assert(bm.fragments==0);
  if(bm.free_headers!=actual_free())fprintf(stderr,"credit leak: reported=%u actual=%u trial=%u\n",bm.free_headers,actual_free(),trial);
  assert(bm.free_headers==actual_free());
  take(DEV_IS_DATA,"G4 P0.002");_free_processed_buffer();
  assert(bm.free_headers==11 && actual_free()==11);
  // An RX query itself occupies one slot, with the following empty FILLING slot.
  posted("{\"rx\":null}");filling();take(DEV_IS_CTRL,"{\"rx\":null}");
  assert(xio_get_line_buffers_available()==10);
  _free_processed_buffer();assert(bm.free_headers==11);
 }
}
'''
    with tempfile.TemporaryDirectory() as d:
        p=Path(d);(p/'check.c').write_text(code)
        subprocess.run(['cc','-std=c99','-g','-fsanitize=address,undefined',str(p/'check.c'),'-o',str(p/'check')],check=True)
        subprocess.run([str(p/'check')],check=True)
