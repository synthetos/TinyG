"""Compile the real line-mode allocator and exercise wraparound without hardware.

Run: python3 tests/host/test_linemode_allocator.py
Requires a host C compiler. The fixture supplies header/storage layout only;
allocation, posting, FIFO selection and reclamation come from firmware/xio.c.
"""
import pathlib
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]


def function(source, name):
    match = re.search(r"(?m)^static [^\n;]*\b" + name + r"\([^\n]*\)\n\{", source)
    if not match:
        raise AssertionError(f"missing function {name}")
    return source[match.start():source.index("\n}", match.end()) + 2]


PRELUDE = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define NUL 0
#define DEV_IS_NONE 0
#define DEV_IS_CTRL 1
#define DEV_IS_DATA 2
#define DEV_IS_OVERFLOW 8
typedef uint8_t devflags_t;
enum {BUFFER_FREE,BUFFER_FRAGMENT,BUFFER_FILLING,BUFFER_FULL,BUFFER_PROCESSING};
typedef struct hdr {uint8_t bufnum;struct hdr *pv,*nx;int state;
    uint8_t flags;uint16_t size;char *bufp;} buf_hdr_t;
typedef struct {buf_hdr_t *used_base,*used_top;uint16_t requested_size;
    uint8_t fragments,free_headers,out_of_ram;char *pool_base,*pool_top;
    buf_hdr_t buf[RX_HEADERS];} buf_mgr_t;
static buf_mgr_t bm;
static char pool[RX_BUFFER_POOL_SIZE];
/* _post_buffer() classifies each line (E103); classification does not matter to the allocator */
static devflags_t _classify_line(const char *line, uint16_t size) {(void)line;(void)size;return DEV_IS_DATA;}
'''

EXERCISE = r'''
static char expected[RX_HEADERS][RX_BUFFER_MIN_SIZE];
static unsigned next_id,consumed,allocations;
static uint32_t rng;
static uint32_t random_value(void) {rng=rng*1664525u+1013904223u;return rng;}
static void init(void) {
    memset(&bm,0,sizeof(bm));memset(pool,0,sizeof(pool));
    bm.pool_base=pool;bm.pool_top=pool+sizeof(pool);
    bm.used_base=bm.used_top=&bm.buf[0];bm.free_headers=RX_HEADERS;
    bm.requested_size=RX_BUFFER_MIN_SIZE;next_id=consumed=0;
    for(unsigned i=0;i<RX_HEADERS;i++) {
        bm.buf[i].bufnum=i;bm.buf[i].bufp=pool;
        bm.buf[i].pv=&bm.buf[(i+RX_HEADERS-1)%RX_HEADERS];
        bm.buf[i].nx=&bm.buf[(i+1)%RX_HEADERS];
    }
}
static void verify_payloads(void) {
    for(unsigned i=0;i<RX_HEADERS;i++) {
        buf_hdr_t *h=&bm.buf[i];
        if(h->state==BUFFER_FULL||h->state==BUFFER_PROCESSING)
            assert(strcmp(h->bufp,expected[i])==0);
    }
}
static bool append(unsigned length) {
    assert(length+1<RX_BUFFER_MIN_SIZE);
    char *p=_get_free_buffer(bm.requested_size);
    if(!p) {verify_payloads();return false;}
    buf_hdr_t *allocated=bm.used_top;
    assert(p>=pool&&p+bm.requested_size<=pool+sizeof(pool));
    assert(allocated->size>=bm.requested_size);
    for(unsigned i=0;i<RX_HEADERS;i++) {
        buf_hdr_t *h=&bm.buf[i];
        if(h==allocated||h->state==BUFFER_FREE)continue;
        if(p<h->bufp+h->size&&h->bufp<p+bm.requested_size) {
            fprintf(stderr,"E094: allocation at %td overlaps queued header %u at %td (line %u)\n",
                    p-pool,i,h->bufp-pool,next_id);
            exit(2);
        }
    }
    memset(p,'A',length);p[length]=0;
    char prefix[24];snprintf(prefix,sizeof(prefix),"N%06u ",next_id++);
    memcpy(p,prefix,strlen(prefix));strcpy(expected[allocated->bufnum],p);
    _post_buffer();allocations++;verify_payloads();return true;
}
static bool consume(void) {
    devflags_t flags=DEV_IS_DATA;char *p=_next_buffer_to_process(&flags);
    if(!p)return false;
    verify_payloads();assert(strtoul(p+1,NULL,10)==consumed++);
    _free_processed_buffer();verify_payloads();return true;
}
int main(void) {
    /* Eleven 72-byte lines, then reclaim three: the next wrap exposes E094. */
    init();bm.requested_size=200;for(int i=0;i<11;i++)assert(append(72));
    for(int i=0;i<3;i++)assert(consume());
    assert(append(72));
    assert(!append(72)); /* there is less than one full receive buffer before used_base */
    while(consume()) {}
        assert(consumed==next_id);
    for(unsigned seed=1;seed<=20;seed++)for(unsigned mode=0;mode<7;mode++) {
        init();rng=seed;
        for(unsigned step=0;step<2000;step++) {
            unsigned lengths[]={12,40,65,72,80,190,60+(random_value()%31)};
            if((random_value()&7)<6) {
                if(!append(lengths[mode]))assert(consume());
            } else consume();
        }
        while(consume()) {}
        assert(consumed==next_id);
    }
    printf("E094: %u allocations; deterministic wrap and 140 FIFO stress scenarios passed\n",allocations);
    return 0;
}
'''


class LineModeAllocatorTests(unittest.TestCase):
    def test_wrapped_allocations_preserve_queued_lines(self):
        compiler = shutil.which("cc")
        if not compiler:
            self.skipTest("host C compiler not installed")
        source = (ROOT / "firmware/tinyg/xio.c").read_text()
        header = (ROOT / "firmware/tinyg/xio.h").read_text()
        defines = ""
        for name in ("RX_HEADERS", "RX_BUFFER_MIN_SIZE", "RX_BUFFER_POOL_SIZE"):
            value = re.search(r"^#define\s+" + name + r"\s+(\d+)", header, re.M).group(1)
            defines += f"#define {name} {value}\n"
        functions = "\n".join(function(source, name) for name in (
            "_get_free_buffer", "_post_buffer", "_next_buffer_to_process", "_free_processed_buffer"))
        with tempfile.TemporaryDirectory(prefix="tinyg-allocator-") as directory:
            path = pathlib.Path(directory)
            (path / "check.c").write_text(defines + PRELUDE + functions + EXERCISE)
            subprocess.run([compiler, "-std=c99", "-O1", "-fsanitize=address,undefined","-fstrict-flex-arrays=3",
                            str(path / "check.c"), "-o", str(path / "check")], check=True)
            result = subprocess.run([str(path / "check")], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            print(result.stdout.strip())


if __name__ == "__main__":
    unittest.main()
