"""E195: a partial line left by a host that went away must not run when the next host connects.

The next host's first line ({sr:null}) is appended to the leftover, and if the leftover ends inside a
comment, "G0 X0.5 (rough" + "{sr:null}" runs G0 X0.5. A '{' that arrives after the line in progress has
been quiet for longer than RX_STALE_LINE_TICKS RTC ticks starts a fresh line instead.

The real RX interrupt, _gets_helper()/xio_gets_usart() (xio_usart.c), xio_reset_usb_rx_buffers(),
xio_drop_usb_rx_before_mark() and the RTC tick callback run on the firmware's ring macro with the 250-byte
ring. Cases: the leftover still unread in the ring; the leftover already taken by the reader; the quiet gap
spent with the leftover and a later line both unread. Controls, which must keep the old behaviour: no gap;
a gap before something other than '{'; a gap with no line in progress; a '{' inside a line after a short
pause; and a flush or buffer reset between the gap's mark and the reader.
"""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

from test_e193_usart_overflow import fn, rx_code, src

TICKS = 100     # RX_STALE_LINE_TICKS: 10 ms RTC ticks


def code():
    c = rx_code()
    usb = src("xio/xio_usb.c")
    if "xio_usb_rtc_callback" in usb:
        c = c.replace("#include <stdlib.h>", fn("xio/xio_usb.c", "xio_usb_rtc_callback") + "\n#include <stdlib.h>", 1)
    else:                                                     # before the fix: nothing counts the quiet time
        c = c.replace("#include <stdlib.h>", "static void xio_usb_rtc_callback(void) {}\n#include <stdlib.h>", 1)
    return c + r'''
static void quiet(int ticks) { while (ticks--) xio_usb_rtc_callback(); }
static void want(const char *where, const char *expect) {
    int status = rd();
    if (status != XIO_OK || strcmp(line, expect)) {
        fprintf(stderr, "E195: %s: want '%s', got %d '%s'\n", where, expect, status, line); exit(66);
    }
}
static void control(const char *where, const char *expect) {
    int status = rd();
    if (status != XIO_OK || strcmp(line, expect)) {
        fprintf(stderr, "control %s: want '%s', got %d '%s'\n", where, expect, status, line); exit(1);
    }
}
int main(void) {
    setup();

    /* the leftover still unread in the ring */
    send_text("G0 X0.5 (rough"); quiet(@T@ + 1); send_text("{sr:null}\n");
    want("leftover unread", "{sr:null}");
    caught_up();

    /* the reader has taken the leftover */
    send_text("G0 X0.5 ;rough");
    assert(rd() == XIO_EAGAIN);
    quiet(@T@ + 1); send_text("{sr:null}\n");
    want("leftover read", "{sr:null}");
    caught_up();

    /* a complete line ahead of the leftover still runs, and so does the new host's line */
    send_text("G4 P0\n"); send_text("G0 X1 (a"); quiet(@T@ + 5); send_text("{sr:null}\n");
    control("line before the leftover", "G4 P0");
    want("after a complete line", "{sr:null}");
    caught_up();

    /* controls: the old behaviour */
    send_text("G0 X0.5 (rough"); quiet(@T@ - 1); send_text("{sr:null}\n");
    control("no gap", "G0 X0.5 (rough{sr:null}");
    send_text("G0 X0.5"); quiet(@T@ + 1); send_text(" Y1\n");
    control("gap before a non-brace", "G0 X0.5 Y1");
    send_text("G4 P0\n"); control("complete line", "G4 P0"); quiet(@T@ + 1); send_text("{sr:null}\n");
    control("gap with no line in progress", "{sr:null}");
    send_text("{\"gc\":\"G4 P0 ("); quiet(10); send_text("{note})\"}\n");
    control("brace after a short pause", "{\"gc\":\"G4 P0 ({note})\"}");
    caught_up();

    /* a flush between the mark and the reader drops the marked line with the rest */
    send_text("G0 X0.5 (rough"); quiet(@T@ + 1); send_text("{sr:null}\n");
    cm.queue_flush_input = false; send_byte('%');
    send_text("G4 P1\n");
    flush_at_mark();
    control("after a flush", "G4 P1");
    caught_up();

    /* a buffer reset leaves no mark behind */
    send_text("G0 X0.5 (rough"); quiet(@T@ + 1); send_text("{sr");
    xio_reset_usb_rx_buffers();
    send_text("G0 X2 (b"); send_text("{c}\n");
    control("after a reset", "G0 X2 (b{c}");
    caught_up();

    puts("E195: stale partial lines are dropped at the next host's '{'");
    return 0;
}
'''.replace("@T@", str(TICKS))


class StalePartialLine(unittest.TestCase):
    def test_stale_line_dropped(self):
        with tempfile.TemporaryDirectory() as tmp:
            c, exe = Path(tmp) / "e195.c", Path(tmp) / "e195"
            c.write_text(code())
            subprocess.run(["cc", "-std=gnu99", "-O1", "-Wall", "-Werror", "-Wno-unused-function", "-Wno-unused-variable",
                            "-fsanitize=address,undefined", str(c), "-o", str(exe)], check=True)
            r = subprocess.run([str(exe)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr + r.stdout)


if __name__ == "__main__":
    unittest.main()
