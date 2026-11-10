"""E146: a host '%' drops what arrived before it and keeps what the host sent after it.

The real _gets_helper()/xio_gets_usart(), xio_get_rx_bufcount_usart(), xio_xon_usart()/xio_fc_usart()
(xio_usart.c), xio_reset_usb_rx_buffers()/xio_drop_usb_rx_before_mark() (xio_usb.c) and
xio_flush_input() (xio.c) run on the firmware's own ring macro. The mark set by the RX ISR must stop
the reader at the '%' even when the ISR fires right at the reader's boundary (ring wrapping), the
flush must drop only the bytes before it, and flow control must stay paused while what is kept is
still above the low-water mark, for XON/XOFF and for RTS.
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
#define NUL 0
#define CR 13
#define LF 10
#define UINT8_MAX 255
#define XON 0x11
#define XOFF 0x13
#define RX_BUFFER_SIZE 64
// Scaled thresholds for this small synthetic ring; N102 covers hardware marks.
#define RTS_RX_HI_WATER_MARK XOFF_RX_HI_WATER_MARK
#define RTS_RX_LO_WATER_MARK XOFF_RX_LO_WATER_MARK
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
struct {uint8_t CTRLA;} uart; struct {uint8_t OUTSET, OUTCLR;} port;
typedef struct {uint8_t fc_char_rx, fc_state_rx; buffer_t rx_buf_tail,rx_buf_head,rx_buf_count,rx_flush_mark;
                uint8_t rx_flush_marked,rx_lost_first,rx_lost_last,rx_lost,rx_lost_after_mark,rx_lost_post_first,rx_lost_open,rx_line_damaged,rx_gap_report; typeof(uart) *usart; typeof(port) *port; char rx_buf[64];} xioUsart_t;
typedef struct dev {xioUsart_t *x;uint8_t len;int size,signal;char *buf;bool flag_in_line,flag_echo;
 void (*x_flow)(struct dev*);int (*x_putc)(char,FILE*);} xioDev_t;
enum {XIO_DEV_USB, XIO_DEV_PGM};
struct {int enable_flow_control, buf_state, primary_src;} xio;
static void controller_reset_source(void) { xio.primary_src=XIO_DEV_USB; }
xioUsart_t USBu; xioDev_t USB; volatile bool rx_in_line; volatile uint8_t rx_quiet_ticks; static volatile uint8_t usb_activity;
#define RX_STALE_LINE_TICKS 103
#define RX_STALE_LINE_MARK 0x1A

uint8_t SREG = 0x80;
static int inject;                                   /* an RX interrupt armed to fire at the next cli() */
static void isr_mark_and_post(void);
static void cli(void) { if ((SREG & 0x80) && inject) { inject = 0; isr_mark_and_post(); } SREG &= ~0x80; }
static unsigned pool_resets;
static void _init_readline_linemode(void) { pool_resets++; }
void xio_fc_usart(xioDev_t *d);
'''

EXERCISE = r'''
static char line[200];
static void rx(const char *s) { for (; *s; s++) { advance_buffer(USBu.rx_buf_head, RX_BUFFER_SIZE); USBu.rx_buf[USBu.rx_buf_head] = *s; } }
static void percent(void) { USBu.rx_flush_mark = USBu.rx_buf_head; USBu.rx_flush_marked = true; }  /* the ISR's honored '%' */
static void isr_mark_and_post(void) { percent(); rx("{\"fb\":null}\n"); }
static int getline_(void) { return xio_gets_usart(&USB, line, sizeof(line)); }
static void start(buffer_t at) { xio_reset_usb_rx_buffers(); USBu.rx_buf_head = USBu.rx_buf_tail = at; }
static bool released(void) { return USBu.fc_state_rx == FC_IN_XON; }
int main(void) {
    USB.x = &USBu; USB.x_flow = xio_fc_usart; USBu.usart = &uart; USBu.port = &port;
    /* N43: flushing a canned program returns input to USB and keeps post-'%' bytes. */
    start(1); xio.primary_src=XIO_DEV_PGM; xio.buf_state=1;
    rx("discard\n"); percent(); rx("{\"stat\":null}\n");
    xio_flush_input();
    assert(xio.primary_src==XIO_DEV_USB && xio.buf_state==BUFFER_FREE && pool_resets);
    assert(getline_()==XIO_OK && !strcmp(line,"{\"stat\":null}"));
    /* a whole line and half a line came before the '%', a query follows */
    start(1);
    rx("G1 X5\n{\"fb\":nu"); percent(); rx("{\"stat\":null}\n");
    assert(getline_() == XIO_OK && !strcmp(line, "G1 X5"));
    assert(getline_() != XIO_OK && USB.flag_in_line);           /* stops at the mark mid-line */
    assert(USBu.rx_buf_tail == USBu.rx_flush_mark);
    xio_drop_usb_rx_before_mark();
    assert(!USBu.rx_flush_marked && !USB.flag_in_line && USB.len == 0);
    assert(USBu.rx_buf_count == xio_get_rx_bufcount_usart(&USBu) && USBu.rx_buf_count == 14);
    assert(getline_() == XIO_OK && !strcmp(line, "{\"stat\":null}"));
    /* the ISR marks a '%' at an empty tail right as the reader starts, and its bytes wrap the ring */
    start(2); inject = 1;
    assert(getline_() != XIO_OK && USBu.rx_buf_tail == 2 && !inject);   /* never steps past the mark */
    assert(SREG & 0x80);
    xio_drop_usb_rx_before_mark();
    assert(getline_() == XIO_OK && !strcmp(line, "{\"fb\":null}"));
    /* unread bytes before the mark are dropped too */
    start(1);
    rx("{\"xvm\":1}\n"); percent(); rx("{\"fv\":null}\n");
    xio_drop_usb_rx_before_mark();
    assert(getline_() == XIO_OK && !strcmp(line, "{\"fv\":null}"));
    /* no mark (a '%' line or {qf}): everything goes, as before */
    start(1); rx("{\"fv\":null}\n");
    xio_drop_usb_rx_before_mark();
    assert(USBu.rx_buf_head == USBu.rx_buf_tail && getline_() != XIO_OK);
    /* flow control after the flush, XON/XOFF and RTS: paused while what is kept is high */
    for (int mode = FLOW_CONTROL_XON; mode <= FLOW_CONTROL_RTS; mode++) {
        xio.enable_flow_control = mode;
        start(1); rx("abc"); percent();
        for (int i = 0; i < 40; i++) rx("x");                   /* 40 kept bytes: above the low mark */
        USBu.fc_state_rx = FC_IN_XOFF; USBu.fc_char_rx = 0; port.OUTCLR = 0;
        xio_flush_input();
        assert(!released() && USBu.fc_char_rx != XON && !(port.OUTCLR & USB_RTS_bm));
        assert(xio_get_rx_bufcount_usart(&USBu) == 40 && (SREG & 0x80));
        while (xio_get_rx_bufcount_usart(&USBu) >= XOFF_RX_LO_WATER_MARK) getline_();
        getline_();                                             /* the reader's flow callback releases it */
        assert(released());
        assert(mode == FLOW_CONTROL_XON ? USBu.fc_char_rx == XON : (port.OUTCLR & USB_RTS_bm));
        /* little kept: released by the flush itself */
        start(1); rx("abc"); percent(); rx("xy");
        USBu.fc_state_rx = FC_IN_XOFF; USBu.fc_char_rx = 0; port.OUTCLR = 0;
        xio_flush_input();
        assert(released());
    }
    return 0;
}
'''


class PercentKeepsLaterInput(unittest.TestCase):
    def test_reader_stops_at_mark_and_flush_keeps_later_bytes(self):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        fw = ROOT / "firmware/tinyg"
        usart = (fw / "xio/xio_usart.c").read_text()
        usb = (fw / "xio/xio_usb.c").read_text()
        xio = (fw / "xio.c").read_text()
        code = PRELUDE
        if "void xio_cancel_pgm_source(void)" in xio:
            code += function(xio, r"void xio_cancel_pgm_source\(void\)") + "\n"
        for source, signature in ((usart, r"buffer_t xio_get_rx_bufcount_usart\("), (usart, r"void xio_xon_usart\("),
                                  (usart, r"void xio_fc_usart\("), (usart, r"static int _gets_helper\("),
                                  (usart, r"int xio_gets_usart\("), (usb, r"void xio_reset_usb_rx_buffers\("),
                                  (usb, r"void xio_drop_usb_rx_before_mark\("), (xio, r"void xio_flush_input\(")):
            code += function(source, signature + r"[^;{}]*\)(?=[^\n{;]*\n?\s*\{)") + "\n"   # a comment may follow
        code += EXERCISE
        with tempfile.TemporaryDirectory(prefix="tinyg-e146-") as tmp:
            c, exe = Path(tmp) / "ring.c", Path(tmp) / "ring"
            c.write_text(code)
            subprocess.run([cc, "-std=gnu99", "-Wall", "-Wno-unused-function", "-fsanitize=address,undefined","-fstrict-flex-arrays=3",
                            str(c), "-o", str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
