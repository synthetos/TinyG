"""Check immediate reset dispatch and strict startup output without hardware."""
import pathlib
import subprocess
import tempfile
import unittest
from source_extract import function, OVERRIDE_STUBS

ROOT=pathlib.Path(__file__).resolve().parents[2]
PRELUDE=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <setjmp.h>
typedef int stat_t;
#define __AVR 1
#define CHAR_RESET 24
#define ISR(name) void name(void)
#define USB_RX_ISR_vect receive
#define WDTO_15MS 15
#define PSTR(s) (s)
#define NV_HEAD 0
#define JSON_SYNTAX_STRICT 1
static struct {int json_syntax;} js;
#define USART_BUFOVF_bm 8
static struct {uint8_t STATUS; char DATA;} regs;
static void _usb_rx_lost(void) {assert(0);}      /* the USART reports no lost frames here (E193) */
static struct {typeof(regs) *usart;} USBu={&regs};
static int immediate,requested,masked,printed;
static jmp_buf reset;
static void cli(void) {masked=1;}
static void wdt_enable(int timeout) {assert(masked&&timeout==15);immediate++;longjmp(reset,1);}
static void hw_request_hard_reset(void) {requested++;}
static void hw_hard_reset(void);
static void nv_reset_nv_list(const char *key) {assert(key[0]=='s'&&key[1]=='r');}
static void nv_add_object(const char *key) {(void)key;}
static void nv_add_string(const char *key,const char *value) {(void)key;(void)value;}
static const char *pstr2str(const char *s) {return s;}
static void json_print_object(int head) {assert(head==0);assert(js.json_syntax==JSON_SYNTAX_STRICT);printed++;}
'''

class ResetHandshake(unittest.TestCase):
    def test_dispatch_and_startup(self):
        hw=(ROOT/'firmware/tinyg/hardware.c').read_text()
        usb=(ROOT/'firmware/tinyg/xio/xio_usb.c').read_text()
        rpt=(ROOT/'firmware/tinyg/report.c').read_text()
        dispatch=function(usb,r'ISR\(USB_RX_ISR_vect\)[^\n]*\n')
        dispatch=dispatch[:dispatch.index('\n\tif (c == CHAR_FEEDHOLD)')]+ '\n}'
        code=OVERRIDE_STUBS+PRELUDE+"static bool usb_boot_discard; static volatile unsigned char usb_activity, usb_rx_seen, rx_quiet_ticks;\n"+dispatch
        code+=function(hw,r'void hw_hard_reset\(void\)[^\n]*\n')
        code+=function(rpt,r'void _startup_helper\(stat_t status, const char \*msg\)\s*\n')
        code+=r'''
int main(void) {
    regs.DATA=CHAR_RESET;
    if(!setjmp(reset)) {receive();assert(0);}
    assert(immediate==1&&requested==0&&masked);
    regs.DATA='G';receive();assert(immediate==1&&requested==0);
    for(int syntax=0;syntax<=1;syntax++) {
        js.json_syntax=syntax;_startup_helper(0,"SYSTEM READY");
        assert(js.json_syntax==syntax);
    }
    assert(printed==2);return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            src=pathlib.Path(tmp)/'reset.c';exe=pathlib.Path(tmp)/'reset'
            src.write_text(code)
            subprocess.run(['cc','-std=gnu99','-Wall','-Wextra','-Wno-unused-function','-Wno-unused-parameter','-fsanitize=address,undefined','-fstrict-flex-arrays=3',str(src),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

if __name__=='__main__': unittest.main()
