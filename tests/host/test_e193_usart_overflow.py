"""E193: frames the USART loses in hardware must not leave an altered line that runs.

The USART receive buffer holds two frames. If the RX interrupt cannot run for longer than that,
the next frame overwrites one still waiting and the USART sets BUFOVF in STATUS, carried with
the frame that follows the lost one (XMEGA A manual 21.7.2, 21.15.2). The USB RX interrupt
(xio_usb.c) read DATA only, so "G1 X1.5" with the '.' lost came back as "G1 X15", XIO_OK. E162
marks bytes the interrupt tosses on a full ring; a hardware loss must be marked the same way.

The real RX interrupt, _gets_helper()/xio_gets_usart() (xio_usart.c) and xio_reset_usb_rx_buffers()
run on the firmware's ring macro with the 250-byte ring. Each case raises BUFOVF with one frame:
inside a line the reader has not reached, inside a line it has read up to the loss, and at the
start of a line after it has read everything. The line with the loss must not come back XIO_OK,
the next line must, and no loss state may be left over. Runs without BUFOVF are the controls.
"""
from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile
import unittest

from source_extract import function
from test_rx_overrun_damaged_line import PRELUDE as RX_PRELUDE

ROOT = Path(__file__).resolve().parents[2]
FW = Path(os.environ.get("TINYG_FIRMWARE", ROOT / "firmware/tinyg"))
DEF = r"[^;{}]*\)(?=\s*(?://[^\n]*\s*)?\{)"


def src(path):
    return (FW / path).read_text(encoding="latin-1")


def fn(path, name):
    return function(src(path), r"(?:static )?(?:inline )?(?:void|bool|int|uint8_t|uint32_t|buffer_t) " + name + r"\(" + DEF)


def rx_code():
    pre = RX_PRELUDE.replace("#define RX_BUFFER_SIZE 16", "#define RX_BUFFER_SIZE 250")
    pre = pre.replace("(RX_BUFFER_SIZE * 0.8)", "(RX_BUFFER_SIZE * 0.3)")
    pre = pre.replace("(RX_BUFFER_SIZE * 0.1)", "(RX_BUFFER_SIZE * 0.15)")
    pre = pre.replace("uint8_t CTRLA; char DATA;", "volatile uint8_t CTRLA, STATUS; char DATA;")
    code = pre + "\n#define USART_BUFOVF_bm 8\n"
    for name in ("xio_get_rx_bufcount_usart", "xio_xon_usart", "xio_xoff_usart", "xio_fc_usart", "_gets_helper", "xio_gets_usart"):
        code += fn("xio/xio_usart.c", name) + "\n"
    usb = src("xio/xio_usb.c")
    isr = function(usb, r"ISR\(USB_RX_ISR_vect\)")
    for name in re.findall(r"static (?:inline )?void (_\w+)\(void\)", usb):   # file helpers the interrupt calls
        if re.search(r"\b" + name + r"\(", isr):
            code += fn("xio/xio_usb.c", name) + "\n"
    code += isr + "\n"
    code += fn("xio/xio_usb.c", "xio_reset_usb_rx_buffers") + "\n"
    return code + r'''
#include <stdlib.h>
static int overrun;
static char line[64];
static void send_byte(char c) { uart.DATA = c; USB_RX_ISR_vect(); }
static void send_text(const char *s) { while (*s) send_byte(*s++); }
static void send_after_loss(char c) {                    /* the frame that follows a lost one carries BUFOVF */
    uart.STATUS = overrun ? USART_BUFOVF_bm : 0; send_byte(c); uart.STATUS = 0;
}
static void setup(void) {
    USB.x = &USBu; USB.x_flow = xio_fc_usart; USBu.usart = &uart; USBu.port = &port;
    xio_reset_usb_rx_buffers(); USBu.rx_buf_count = 0; SREG = 0x80;
}
static int rd(void) { return xio_gets_usart(&USB, line, sizeof(line)); }
/* Read until a line comes back XIO_OK; every other line must be reported damaged. */
static int damaged_before(const char *want) {
    int damaged = 0;
    for (int n = 0; n < 8; n++) {
        int status = rd();
        if (status == XIO_LINE_DAMAGED) { damaged++; continue; }
        if (status != XIO_OK || strcmp(line, want)) {
            fprintf(stderr, "E193: want '%s' as XIO_OK (0), got %d '%s'\n", want, status, line); exit(66);
        }
        return damaged;
    }
    fprintf(stderr, "E193: '%s' never came back\n", want); exit(66);
}
static void caught_up(void) {
    assert(USBu.rx_buf_head == USBu.rx_buf_tail);
    assert(!USBu.rx_lost && !USBu.rx_lost_open && !USBu.rx_line_damaged && !USBu.rx_gap_report);
}
static void report(const char *where, int damaged) {
    if (overrun && !damaged) { fprintf(stderr, "E193: %s: USART lost input returns altered G-code as XIO_OK\n", where); exit(66); }
    if (!overrun && damaged) { fprintf(stderr, "%s: control lost a line\n", where); exit(1); }
}
'''


def case(body):
    return rx_code() + r'''
int main(int argc, char **argv) {
    (void)argv; overrun = (argc == 1); setup();
''' + body + r'''
    caught_up();
    return 0;
}
'''


INSIDE_UNREAD_LINE = r'''
    send_text("G1 X1"); send_after_loss('5'); send_text("\n");       /* "G1 X1.5" lost its '.' */
    send_text("G1 X2\n");
    assert(USBu.rx_buf_count < 20);                                   /* the software ring never filled */
    int damaged = damaged_before(overrun ? "G1 X2" : "G1 X15");
    report("inside an unread line", damaged);
    if (!overrun) assert(damaged_before("G1 X2") == 0);
'''

INSIDE_LINE_BEING_READ = r'''
    send_text("G1 X1");
    assert(rd() == XIO_EAGAIN);                                       /* the reader has taken "G1 X1" */
    send_after_loss('5'); send_text("\n");
    send_text("G1 X2\n");
    int damaged = damaged_before(overrun ? "G1 X2" : "G1 X15");
    report("inside a line being read", damaged);
    if (!overrun) assert(damaged_before("G1 X2") == 0);
'''

AT_LINE_START = r'''
    send_text("G1 X1\n");
    assert(damaged_before("G1 X1") == 0);                             /* read, and the ring is empty */
    send_after_loss('1'); send_text("0 Y5\n");                        /* "N10 Y5" lost its 'N' */
    send_text("G1 X2\n");
    int damaged = damaged_before(overrun ? "G1 X2" : "10 Y5");
    report("at the start of a line", damaged);
    if (!overrun) assert(damaged_before("G1 X2") == 0);
'''


class UsartOverflow(unittest.TestCase):
    def _run(self, name, code):
        cc = shutil.which("cc")
        if not cc:
            self.skipTest("host C compiler not installed")
        with tempfile.TemporaryDirectory(prefix="tinyg-e193-") as d:
            c, exe = Path(d) / (name + ".c"), Path(d) / name
            c.write_text(code)
            subprocess.run([cc, "-std=gnu99", "-O1", "-g", "-Wall", "-Wno-unused-function", "-Wno-unused-variable",
                            "-fsanitize=address,undefined", str(c), "-o", str(exe)], check=True)
            control = subprocess.run([str(exe), "control"], capture_output=True, text=True)
            self.assertEqual(control.returncode, 0, control.stderr)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_loss_inside_an_unread_line(self):
        self._run("unread", case(INSIDE_UNREAD_LINE))

    def test_loss_inside_the_line_being_read(self):
        self._run("reading", case(INSIDE_LINE_BEING_READ))

    def test_loss_at_the_start_of_a_line(self):
        self._run("start", case(AT_LINE_START))


if __name__ == "__main__":
    unittest.main()
