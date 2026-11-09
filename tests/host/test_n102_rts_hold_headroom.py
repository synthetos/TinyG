"""RTS must leave a bounded sender able to deliver real-time controls during a hold."""
from pathlib import Path
import os
import re
import subprocess
import tempfile
import unittest
from source_extract import function
from test_rx_overrun_damaged_line import PRELUDE

ROOT = Path(os.environ.get("TINYG_SOURCE", Path(__file__).resolve().parents[2]))


class RtsHoldHeadroom(unittest.TestCase):
    def test_bounded_stream_keeps_controls_and_preserves_backpressure(self):
        fw = ROOT / "firmware/tinyg/xio"
        usart, usb = (fw / "xio_usart.c").read_text(), (fw / "xio_usb.c").read_text()
        header = (fw / "xio_usart.h").read_text()
        code = PRELUDE.replace("#define RX_BUFFER_SIZE 16", "#define RX_BUFFER_SIZE 250")
        code = re.sub(r"#define (XOFF|RTS)_RX_(HI|LO)_WATER_MARK[^\n]*", "", code)
        code += "\n" + "\n".join(l for l in header.splitlines() if re.match(r"#define (XOFF|RTS)_RX_(HI|LO)_WATER_MARK\b", l)) + "\n"
        code = code.replace("static void cm_request_end_hold(void) {}", "static unsigned resumes; static void cm_request_end_hold(void) {resumes++;}")
        for name in ("hw_hard_reset", "cm_request_feedhold", "cm_request_job_kill"):
            code = re.sub(r"static void " + name + r"\(void\) \{[^}]*\}",
                          "static unsigned " + name + "_calls; static void " + name + "(void) {" + name + "_calls++;}", code)
        for source, sig in ((usart, r"buffer_t xio_get_rx_bufcount_usart\("),
                            (usart, r"void xio_xon_usart\("), (usart, r"void xio_xoff_usart\("),
                            (usart, r"void xio_fc_usart\("), (usb, r"static void _usb_rx_lost\("),
                            (usb, r"ISR\(USB_RX_ISR_vect\)"), (usb, r"void xio_reset_usb_rx_buffers\(")):
            code += function(source, sig) + "\n"
        code += r'''
static void fresh(unsigned mode) {
    USB.x=&USBu; USB.x_flow=xio_fc_usart; USBu.usart=&uart; USBu.port=&port;
    xio.enable_flow_control=mode; xio_reset_usb_rx_buffers();
    USBu.rx_buf_count=0; USBu.fc_state_rx=FC_IN_XON; USB.flag_xoff=true;
    rx_in_line=false; usb_boot_discard=false; cm.queue_flush_input=false;
    uart.STATUS=0; resumes=0;
}
static bool wire(unsigned char c) {
    if (USBu.fc_state_rx==FC_IN_XOFF) return false; /* FTDI respects the board's RTS */
    uart.DATA=c; USB_RX_ISR_vect(); return true;
}
int main(void) {
    /* The planner is full and held: the main loop consumes no RX bytes. */
    fresh(FLOW_CONTROL_RTS);
    for(unsigned i=0;i<192;i++) assert(wire(i%6==5?'\n':' '));
    assert(wire('!') && cm_request_feedhold_calls==1);
    assert(wire('~') && resumes==1);
    assert(wire('%') && cm.queue_flush_input);
    assert(wire(CHAR_KILL_JOB) && cm_request_job_kill_calls==1);
    assert(wire(CHAR_RESET) && hw_hard_reset_calls==1);
    assert(USBu.rx_buf_count==192 && !USBu.rx_lost);
    /* Every byte was retained, including across a ring wrap. */
    for(unsigned i=0;i<192;i++) {
        advance_buffer(USBu.rx_buf_tail,RX_BUFFER_SIZE);
        assert(USBu.rx_buf[USBu.rx_buf_tail]==(i%6==5?'\n':' '));
    }
    /* An oversized sender still meets backpressure before actual overflow. */
    fresh(FLOW_CONTROL_RTS);
    unsigned kept=0;
    while(wire(' ')) {assert(++kept<248);}
    assert(kept>=224 && kept<=225 && !USBu.rx_lost);
    assert(xio_get_rx_bufcount_usart(&USBu)==225);
    while(xio_get_rx_bufcount_usart(&USBu)>192) advance_buffer(USBu.rx_buf_tail,RX_BUFFER_SIZE);
    xio_fc_usart(&USB); assert(USBu.fc_state_rx==FC_IN_XOFF); /* hysteresis at 192 */
    advance_buffer(USBu.rx_buf_tail,RX_BUFFER_SIZE);
    xio_fc_usart(&USB); assert(USBu.fc_state_rx==FC_IN_XON);
    /* The conservative software flow-control thresholds do not change. */
    fresh(FLOW_CONTROL_XON); kept=0;
    while(wire(' ')) ++kept;
    assert(kept>=75 && kept<=76 && USBu.fc_char_rx==XOFF && !USBu.rx_lost);
    assert(xio_get_rx_bufcount_usart(&USBu)==76);
    while(xio_get_rx_bufcount_usart(&USBu)>36) advance_buffer(USBu.rx_buf_tail,RX_BUFFER_SIZE);
    xio_fc_usart(&USB); assert(USBu.fc_state_rx==FC_IN_XON && USBu.fc_char_rx==XON);
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            c, exe = Path(tmp)/"rts.c", Path(tmp)/"rts"
            c.write_text(code)
            subprocess.run(["cc", "-std=gnu99", "-Wall", "-Wno-unused-function", "-fsanitize=address,undefined",
                            "-fstrict-flex-arrays=3", str(c), "-o", str(exe)], check=True)
            result=subprocess.run([str(exe)],capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)
