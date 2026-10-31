"""E162: bytes the RX ISR tosses on a full ring must not splice two lines into one that runs.

The real USB RX ISR (xio_usb.c), _gets_helper()/xio_gets_usart() (xio_usart.c),
xio_reset_usb_rx_buffers() and xio_drop_usb_rx_before_mark() (xio_usb.c) run on the firmware's own
ring macro with a 16-byte ring. Every line that loses bytes, or starts right after lost bytes, must
come back XIO_LINE_DAMAGED, reported where the reader meets the loss (a line broken by a loss ends
there, and a loss right after a good line is reported on the next read even with nothing more to
read); lines before and after the loss must come back intact; a '%' flush must
forget losses before its mark and keep the ones after it. A seeded stream with random host bursts
and reader calls checks that no spliced line is ever returned as XIO_OK and that the loss state
always clears once the reader catches up.
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
#define __AVR
#define ISR(name) void name(void)
#define NUL 0
#define CR 13
#define LF 10
#define UINT8_MAX 255
#define XON 0x11
#define XOFF 0x13
#define CHAR_RESET 24
#define CHAR_ENQUIRY 5
#define CHAR_KILL_JOB 4
#define CHAR_FEEDHOLD '!'
#define CHAR_END_HOLD '~'
#define CHAR_QUEUE_FLUSH '%'
#define RX_BUFFER_SIZE 16
#define XOFF_RX_HI_WATER_MARK (RX_BUFFER_SIZE * 0.8)
#define XOFF_RX_LO_WATER_MARK (RX_BUFFER_SIZE * 0.1)
#define CTRLA_RXON_TXON 0x3c
#define USB_RTS_bm 0x08
#define BUFFER_FREE 0
#define min(a,b) ((a)<(b)?(a):(b))
#define advance_buffer(buf,len) { if ((--(buf)) == 0) buf = len-1;}
enum {XIO_OK=0,XIO_EAGAIN,XIO_BUFFER_EMPTY,XIO_BUFFER_FULL,XIO_EOL,XIO_SIG_EOL,XIO_SIG_OK,XIO_LINE_DAMAGED};
enum {FC_IN_XON, FC_IN_XOFF};
enum {FLOW_CONTROL_OFF, FLOW_CONTROL_XON, FLOW_CONTROL_RTS};
typedef uint8_t buffer_t;
struct {uint8_t CTRLA; char DATA;} uart; struct {uint8_t OUTSET, OUTCLR;} port;
typedef struct {uint8_t fc_char_rx, fc_state_rx, fc_state_tx; buffer_t rx_buf_tail,rx_buf_head,rx_buf_count,rx_flush_mark;
                uint8_t rx_flush_marked,rx_lost_first,rx_lost_last,rx_lost,rx_lost_after_mark,rx_lost_post_first,rx_lost_open,rx_line_damaged,rx_gap_report;
                typeof(uart) *usart; typeof(port) *port; char rx_buf[RX_BUFFER_SIZE];} xioUsart_t;
typedef struct dev {xioUsart_t *x;uint8_t len;int size,signal;char *buf;bool flag_in_line,flag_echo,flag_xoff;
 void (*x_flow)(struct dev*);int (*x_putc)(char,FILE*);} xioDev_t;
struct {int enable_flow_control, buf_state;} xio;
struct {bool queue_flush_input;} cm;
xioUsart_t USBu; xioDev_t USB; volatile bool rx_in_line;
uint8_t SREG = 0x80;
static void cli(void) { SREG &= ~0x80; }
static void _init_readline_linemode(void) {}
static void hw_hard_reset(void) { assert(0); }
static void cm_request_feedhold(void) {}
static void cm_request_job_kill(void) {}
static void cm_request_end_hold(void) {}
static void cm_request_queue_flush(void) { cm.queue_flush_input = true; }
static void controller_request_enquiry(void) {}
void xio_fc_usart(xioDev_t *d);
void xio_xoff_usart(xioUsart_t *dx);
'''

EXERCISE = r'''
static char line[64];
static void tx(const char *s) { for (; *s; s++) { uart.DATA = *s; USB_RX_ISR_vect(); } }   /* host bytes through the ISR */
static int rd(void) { return xio_gets_usart(&USB, line, sizeof(line)); }
static void fresh(void) { xio_reset_usb_rx_buffers(); USBu.rx_buf_count = 0; USB.flag_in_line = false; rx_in_line = false; cm.queue_flush_input = false; }
static void expect(int status, const char *text) {
    int got = rd();
    if (got != status || (text && strcmp(line, text))) { fprintf(stderr, "want %d '%s' got %d '%s'\n", status, text ? text : "", got, line); assert(0); }
}
static void drop_at_mark(void) { cli(); xio_drop_usb_rx_before_mark(); SREG |= 0x80; cm.queue_flush_input = false; }

/* the seeded stream: fixed-width lines, random host bursts and reader calls */
static unsigned rng;
static unsigned rnd(void) { rng = rng * 1103515245u + 12345u; return (rng >> 16) & 0x7fff; }
static void stream(unsigned seed, int nlines) {
    char text[600][16]; int sent = 0, pos = 0, expect_next = 0, ok = 0, damaged = 0; bool ended = false;
    for (int k = 0; k < nlines; k++) snprintf(text[k], sizeof(text[k]), "N%04dY%04d\n", k, (k * 7919) % 10000);
    rng = seed; fresh();
    while (sent < nlines || USBu.rx_buf_head != USBu.rx_buf_tail || (USB.flag_in_line && USB.len) || USBu.rx_gap_report) {   /* len: a partial line */
        for (int n = rnd() % 24; n > 0 && sent < nlines; n--) {
            uart.DATA = text[sent][pos++]; USB_RX_ISR_vect();
            if (!text[sent][pos]) { sent++; pos = 0; }
        }
        for (int n = rnd() % 3 + (sent == nlines); n > 0; n--) {
            int got = rd();
            if (got == XIO_EAGAIN) break;
            if (got == XIO_LINE_DAMAGED) { damaged++; continue; }
            assert(got == XIO_OK);
            int k = expect_next;
            while (k < nlines && strncmp(line, text[k], 10)) k++;
            if (k == nlines || strlen(line) != 10) { fprintf(stderr, "seed %u: spliced line '%s' returned OK\n", seed, line); assert(0); }
            expect_next = k + 1; ok++;
        }
        if (sent == nlines && USBu.rx_buf_head == USBu.rx_buf_tail && USB.flag_in_line && USB.len && !ended) {
            tx("\n"); ended = true;                     /* the last line lost its end: the host sends more later */
        }
    }
    assert(!USBu.rx_lost && !USBu.rx_lost_open && !USBu.rx_gap_report);   /* caught up: no loss left to account for */
    assert(ok > 0 && damaged > 0 && ok <= nlines);
}

int main(void) {
    USB.x = &USBu; USB.x_flow = xio_fc_usart; USBu.usart = &uart; USBu.port = &port;
    xio.enable_flow_control = FLOW_CONTROL_OFF;
    /* the ring holds 14 bytes */
    /* a gap inside a line: "G1 Y119." + "G1 X3" must not run as "G1 Y119.G1 X3" */
    fresh();
    tx("G1 X1\nG1 Y119.5\n"); tx("G1 X2\n");             /* "5\n" and "G1 X2\n" tossed */
    assert(USBu.rx_lost);
    expect(XIO_OK, "G1 X1");
    expect(XIO_LINE_DAMAGED, NULL);                       /* "G1 Y119." ends at the loss */
    tx("G1 X3\n");
    expect(XIO_LINE_DAMAGED, NULL);                       /* its tail, "G1 X3" */
    tx("G1 X4\n");
    expect(XIO_OK, "G1 X4");
    assert(!USBu.rx_lost && !USBu.rx_line_damaged);
    /* a gap right after a line end: that line is intact, the next one is not */
    fresh();
    tx("G1 X1\nG1 X222\n"); tx("G1 Y");                   /* "G1 Y" tossed */
    expect(XIO_OK, "G1 X1");
    tx("9\n");
    expect(XIO_OK, "G1 X222");
    expect(XIO_LINE_DAMAGED, NULL);                       /* the loss itself, reported at once */
    expect(XIO_LINE_DAMAGED, NULL);                       /* "9" */
    tx("G1 X5\n");
    expect(XIO_OK, "G1 X5");
    /* several overruns before the reader catches up: every line in the stretch is damaged */
    fresh();
    tx("AAAA\nBBBB\nCCC"); tx("c\n");                     /* first loss inside "CCC" */
    expect(XIO_OK, "AAAA");
    tx("DDDD\nEEEE");                                     /* "EEEE" tossed: last loss right after a line end */
    expect(XIO_OK, "BBBB");
    expect(XIO_LINE_DAMAGED, NULL);                       /* "CCC" ends at the first loss */
    expect(XIO_LINE_DAMAGED, NULL);                       /* "DDDD", inside the stretch */
    tx("FF\nGG\n");
    expect(XIO_LINE_DAMAGED, NULL);                       /* "FF", which lost its "EEEE" */
    expect(XIO_OK, "GG");
    /* an overlong line that also lost bytes is reported as damaged */
    fresh();
    USB.flag_in_line = true; USB.len = 0; USB.buf = line; USB.size = 4; USB.signal = XIO_SIG_OK;
    tx("ABCDEFGHIJKLMN"); tx("O");                        /* tossed */
    int st = XIO_EAGAIN;
    while (st == XIO_EAGAIN && USBu.rx_buf_head != USBu.rx_buf_tail) st = _gets_helper(&USB, &USBu);
    assert(st == XIO_LINE_DAMAGED && !USB.flag_in_line && USBu.rx_buf_head == USBu.rx_buf_tail);
    tx("\n");
    assert(_gets_helper(&USB, &USBu) == XIO_LINE_DAMAGED);  /* the (empty) tail */
    tx("ok\n");
    expect(XIO_OK, "ok");
    /* a reset forgets everything */
    fresh();
    tx("G1 X1\nG1 Y119.5\n");
    fresh();
    tx("G1 X6\n");
    expect(XIO_OK, "G1 X6");
    /* '%' after the loss: the flush drops the damaged input, what follows the '%' is clean */
    fresh();
    tx("G1 X1\nG1 X2\nG1"); tx(" X\n");                   /* " X\n" tossed; the line end clears rx_in_line */
    tx("%");
    assert(USBu.rx_flush_marked);
    drop_at_mark();
    assert(!USBu.rx_lost);
    tx("G1 X7\n");
    expect(XIO_OK, "G1 X7");
    /* loss after the '%': kept across the flush */
    fresh();
    tx("G1 X1\n"); tx("%"); tx("G1 X2\nG1 Y3\n");        /* " Y3\n" tossed */
    drop_at_mark();
    assert(USBu.rx_lost);
    expect(XIO_OK, "G1 X2");
    expect(XIO_LINE_DAMAGED, NULL);                       /* "G1" ends at the loss */
    tx("G1 X9\n");
    expect(XIO_LINE_DAMAGED, NULL);                       /* its tail, "G1 X9" */
    tx("G1 X10\n");
    expect(XIO_OK, "G1 X10");
    /* losses on both sides of the '%': only the one after it counts */
    fresh();
    tx("AAAA\nBBBB\nCCC"); tx("C\n");                     /* first loss before the mark */
    tx("%");
    expect(XIO_OK, "AAAA");
    tx("DD\nEEEEE");                                      /* "EEE" tossed: a loss after the mark */
    drop_at_mark();
    assert(USBu.rx_lost);
    expect(XIO_OK, "DD");
    expect(XIO_LINE_DAMAGED, NULL);                       /* "EE" ends at the loss */
    tx("\nFF\n");
    expect(XIO_LINE_DAMAGED, NULL);                       /* its (empty) tail */
    expect(XIO_OK, "FF");
    assert(!USBu.rx_lost);
    /* a full ring, then '%', then a loss: the loss sits at the mark itself but came after the '%' */
    fresh();
    tx("G1 X1\nG1 X222\n"); tx("%"); tx("G91 ");           /* "G91 " tossed */
    drop_at_mark();
    expect(XIO_LINE_DAMAGED, NULL);                       /* the loss, reported at once */
    tx("G1 Y583\n");
    expect(XIO_LINE_DAMAGED, NULL);                       /* not "G1 Y583" in G91 */
    tx("G1 X5\n");
    expect(XIO_OK, "G1 X5");
    assert(!USBu.rx_lost && !USBu.rx_lost_open);
    /* the same, and the reader frees room before the mark so a later loss lands elsewhere */
    fresh();
    tx("G1 X1\nG1 X222\n"); tx("%"); tx("G91 ");
    expect(XIO_OK, "G1 X1");                              /* before the mark: still read */
    tx("Y5\nG1 X7\n");                                    /* "X7\n" tossed */
    drop_at_mark();
    expect(XIO_LINE_DAMAGED, NULL);                       /* the loss, reported at once */
    expect(XIO_LINE_DAMAGED, NULL);                       /* "Y5", which lost its "G91 " */
    expect(XIO_LINE_DAMAGED, NULL);                       /* "G1 " ends at the last loss */
    tx("Z\n");
    expect(XIO_LINE_DAMAGED, NULL);                       /* its tail, "Z" */
    tx("ok\n");
    expect(XIO_OK, "ok");
    assert(!USBu.rx_lost && !USBu.rx_lost_open);
    /* a second '%' moves the mark: losses between the two are dropped with the input */
    fresh();
    tx("G1 X1\nG1 X222\n"); tx("%"); tx("G91\n"); tx("%");  /* "G91\n" tossed; the second '%' starts a line */
    drop_at_mark();
    tx("G1 X8\n");
    expect(XIO_OK, "G1 X8");
    /* seeded streams */
    for (unsigned seed = 1; seed <= 200; seed++) stream(seed, 400);
    return 0;
}
'''


class RxOverrunDamagedLine(unittest.TestCase):
    def test_lost_bytes_never_splice_a_runnable_line(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        fw = ROOT / "firmware/tinyg"
        usart = (fw / "xio/xio_usart.c").read_text()
        usb = (fw / "xio/xio_usb.c").read_text()
        code = PRELUDE
        for source, signature in ((usart, r"buffer_t xio_get_rx_bufcount_usart\("), (usart, r"void xio_xon_usart\("),
                                  (usart, r"void xio_xoff_usart\("), (usart, r"void xio_fc_usart\("),
                                  (usart, r"static int _gets_helper\("), (usart, r"int xio_gets_usart\("),
                                  (usb, r"ISR\(USB_RX_ISR_vect\)"), (usb, r"void xio_reset_usb_rx_buffers\("),
                                  (usb, r"void xio_drop_usb_rx_before_mark\(")):
            if signature.startswith("ISR"):
                code += function(source, signature) + "\n"
            else:
                code += function(source, signature + r"[^;{}]*\)(?=[^\n{;]*\n?\s*\{)") + "\n"   # a comment may follow
        code += EXERCISE
        with tempfile.TemporaryDirectory(prefix="tinyg-e162-") as tmp:
            c, exe = Path(tmp) / "overrun.c", Path(tmp) / "overrun"
            c.write_text(code)
            subprocess.run([cc, "-std=gnu99", "-Wall", "-Wno-unused-function", "-fsanitize=address,undefined",
                            str(c), "-o", str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
