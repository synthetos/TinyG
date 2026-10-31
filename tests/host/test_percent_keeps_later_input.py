"""E146: a host '%' drops what arrived before it and keeps what the host sent after it.

The real _gets_helper()/xio_gets_usart() (xio_usart.c), xio_get_rx_bufcount_usart() and
xio_reset_usb_rx_buffers()/xio_drop_usb_rx_before_mark() (xio_usb.c) run on the firmware's own
ring macro. Bytes before the '%' include a whole line and half of another; the mark set by the RX ISR
must stop the reader at the '%', and the flush must drop only the bytes before it.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function

ROOT = Path(__file__).resolve().parents[2]

PRELUDE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define NUL 0
#define CR 13
#define LF 10
#define UINT8_MAX 255
#define RX_BUFFER_SIZE 64
#define min(a,b) ((a)<(b)?(a):(b))
#define advance_buffer(buf,len) { if ((--(buf)) == 0) buf = len-1;}
enum {XIO_OK=0,XIO_EAGAIN,XIO_BUFFER_EMPTY,XIO_BUFFER_FULL,XIO_EOL,XIO_SIG_EOL,XIO_SIG_OK};
typedef uint8_t buffer_t;
typedef struct {buffer_t rx_buf_tail,rx_buf_head,rx_buf_count,rx_flush_mark;uint8_t rx_flush_marked;char rx_buf[64];} xioUsart_t;
typedef struct dev {xioUsart_t *x;uint8_t len;int size,signal;char *buf;bool flag_in_line,flag_echo;
 void (*x_flow)(struct dev*);int (*x_putc)(char,FILE*);} xioDev_t;
static void flow(xioDev_t *d){(void)d;}
xioUsart_t USBu; xioDev_t USB; volatile bool rx_in_line;
'''

EXERCISE = r'''
static char line[200];
static void rx(const char *s) { for (; *s; s++) { advance_buffer(USBu.rx_buf_head, RX_BUFFER_SIZE); USBu.rx_buf[USBu.rx_buf_head] = *s; } }
static void percent(void) { USBu.rx_flush_mark = USBu.rx_buf_head; USBu.rx_flush_marked = true; }  /* the ISR's honored '%' */
static int getline_(void) { return xio_gets_usart(&USB, line, sizeof(line)); }
int main(void) {
    USB.x = &USBu; USB.x_flow = flow;
    /* honored '%': a whole line and half a line came first, a query follows */
    xio_reset_usb_rx_buffers();
    rx("G1 X5\n{\"fb\":nu"); percent(); rx("{\"stat\":null}\n");
    assert(getline_() == XIO_OK && !strcmp(line, "G1 X5"));       /* before the '%': read normally */
    assert(getline_() != XIO_OK && USB.flag_in_line);              /* stops at the mark mid-line */
    assert(USBu.rx_buf_tail == USBu.rx_flush_mark);
    xio_drop_usb_rx_before_mark();                             /* the flush (interrupts masked) */
    assert(!USBu.rx_flush_marked && !USB.flag_in_line && USB.len == 0);
    assert(USBu.rx_buf_count == xio_get_rx_bufcount_usart(&USBu) && USBu.rx_buf_count == 14);
    assert(getline_() == XIO_OK && !strcmp(line, "{\"stat\":null}"));  /* after the '%': whole */
    /* unread bytes before the mark are dropped too */
    xio_reset_usb_rx_buffers();
    rx("{\"xvm\":1}\n"); percent(); rx("{\"fv\":null}\n");
    xio_drop_usb_rx_before_mark();
    assert(getline_() == XIO_OK && !strcmp(line, "{\"fv\":null}"));
    /* no mark (a '%' line or {qf}): everything goes, as before */
    xio_reset_usb_rx_buffers();
    rx("{\"fv\":null}\n");
    xio_drop_usb_rx_before_mark();
    assert(USBu.rx_buf_head == USBu.rx_buf_tail && getline_() != XIO_OK);
    return 0;
}
'''


class PercentKeepsLaterInput(unittest.TestCase):
    def test_reader_stops_at_mark_and_flush_keeps_later_bytes(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        usart = (ROOT / "firmware/tinyg/xio/xio_usart.c").read_text()
        usb = (ROOT / "firmware/tinyg/xio/xio_usb.c").read_text()
        code = (PRELUDE
                + function(usart, r"buffer_t xio_get_rx_bufcount_usart\([^;{}]*\)(?=\s*\{)")
                + function(usart, r"static int _gets_helper\([^;{}]*\)(?=\s*\{)")
                + function(usart, r"int xio_gets_usart\([^;{}]*\)(?=\s*\{)")
                + function(usb, r"void xio_reset_usb_rx_buffers\([^;{}]*\)(?=\s*\{)")
                + function(usb, r"void xio_drop_usb_rx_before_mark\([^;{}]*\)(?=\s*\{)")
                + EXERCISE)
        with tempfile.TemporaryDirectory(prefix="tinyg-e146-") as tmp:
            c, exe = Path(tmp) / "ring.c", Path(tmp) / "ring"
            c.write_text(code)
            subprocess.run([cc, "-std=c99", "-Wall", "-Wno-unused-function", "-fsanitize=address,undefined",
                            str(c), "-o", str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
