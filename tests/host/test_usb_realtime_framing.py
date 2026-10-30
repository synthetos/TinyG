"""Exercise the actual RX ISR against a byte/register fixture; no timing claims."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

ROOT = Path(os.environ.get("TINYG_SOURCE", Path(__file__).resolve().parents[2]))


from source_extract import function


class RealtimeFraming(unittest.TestCase):
    def test_isr_keeps_literals_and_traps_boundary_controls(self):
        source = (ROOT / 'firmware/tinyg/xio/xio_usb.c').read_text()
        code = r'''
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <string.h>
#define ISR(name) void name(void)
#define NUL 0
#define CHAR_RESET 24
#define CHAR_ENQUIRY 5
#define CHAR_FEEDHOLD '!'
#define CHAR_END_HOLD '~'
#define CHAR_QUEUE_FLUSH '%'
#define XOFF 19
#define XON 17
#define FC_IN_XOFF 0
#define FC_IN_XON 1
#define CTRLA_RXON_TXOFF 32
#define RX_BUFFER_SIZE 512
#define XOFF_RX_HI_WATER_MARK 400
#define advance_buffer(i,size) ((i)=((i)+1)%(size))
struct {unsigned char DATA; int CTRLA;} port;
struct {int len; bool flag_in_line, flag_xoff;} USB;
struct {typeof(port) *usart; int rx_buf_head,rx_buf_tail,rx_buf_count,fc_state_tx;
        char rx_buf[RX_BUFFER_SIZE];} USBu={.usart=&port,.rx_buf_head=1,.rx_buf_tail=1};
volatile bool rx_in_line;
int holds,resumes,flushes,resets,enquiries;
void cm_request_feedhold(void) {holds++;}
void cm_request_end_hold(void) {resumes++;}
void cm_request_queue_flush(void) {flushes++;}
void hw_request_hard_reset(void) {resets++;}
void controller_request_enquiry(void) {enquiries++;}
int xio_get_rx_bufcount_usart(void *unused) {(void)unused; return USBu.rx_buf_count;}
void xio_xoff_usart(void *unused) {(void)unused;}
'''
        code += function(source, r'ISR\(USB_RX_ISR_vect\)')
        code += function(source, r'void xio_reset_usb_rx_buffers\(void\)')
        code += r'''
void send_byte(unsigned char c) {port.DATA=c;USB_RX_ISR_vect();}
void send(const char *s) {while(*s) send_byte(*s++);}
void fresh(void) {xio_reset_usb_rx_buffers(); USBu.rx_buf_count=0; holds=resumes=flushes=resets=enquiries=0;}
void expect_data(const char *s) {
    assert((int)strlen(s)==USBu.rx_buf_count);
    for (int i=0;s[i];i++) assert(USBu.rx_buf[i+2]==s[i]);
}
int main(void) {
    const char *lines[]={"G21 (MSG 50% done ! ~)\n", "G1 X1 ; !~%\r", "{\"msg\":\"!~%\"}\n",
                         "G1 X1!~%\n", "(unterminated !~%\n"};
    for(int i=0;i<5;i++) {
        fresh(); send(lines[i]); expect_data(lines[i]);
        assert(!holds && !resumes && !flushes);
        send("!~%"); assert(holds==1 && resumes==1 && flushes==1);
    }
    fresh(); send(" \t!~%"); expect_data(" \t");
    assert(holds==1 && resumes==1 && flushes==1);
    fresh(); send("partial line"); USB.flag_xoff=true;
    send_byte(CHAR_RESET); send_byte(CHAR_ENQUIRY); send_byte(XOFF);
    assert(resets==1 && enquiries==1 && USBu.fc_state_tx==FC_IN_XOFF);
    send_byte(XON); assert(USBu.fc_state_tx==FC_IN_XON);
    expect_data("partial line");
    send("!~%"); assert(!holds && !resumes && !flushes);
    send("\r!\n~%"); assert(holds==1 && resumes==1 && flushes==1);
    fresh(); send("unfinished ("); xio_reset_usb_rx_buffers(); send("!");
    assert(holds==1);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            c = Path(tmp) / 'rx.c'
            c.write_text(code)
            exe = Path(tmp) / 'rx'
            subprocess.run(['cc', '-std=gnu99', '-Wall', '-Werror', '-fsanitize=address,undefined',
                            str(c), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()
