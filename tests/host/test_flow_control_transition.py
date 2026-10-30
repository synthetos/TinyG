"""Exercise mode transitions and RX flow characters with the production functions."""
import pathlib
import subprocess
import tempfile
import unittest
from source_extract import function
ROOT=pathlib.Path(__file__).resolve().parents[2]
PRELUDE=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
typedef int stat_t;
typedef struct {uint32_t value_int;} nvObj_t;
enum {STAT_OK,STAT_INPUT_VALUE_RANGE_ERROR};
enum {FLOW_CONTROL_OFF,FLOW_CONTROL_XON,FLOW_CONTROL_RTS};
enum {FC_IN_XON,FC_IN_XOFF};
#define NUL 0
#define XON 17
#define XOFF 19
#define USB_RTS_bm 1
#define CTRLA_RXON_TXON 7
#define XOFF_RX_HI_WATER_MARK 200
#define XIO_DEV_USB 0
#define XIO_XOFF 1
#define XIO_NOXOFF 2
#define CHAR_RESET 24
#define CHAR_FEEDHOLD '!'
#define CHAR_QUEUE_FLUSH '%'
#define CHAR_END_HOLD '~'
#define CHAR_ENQUIRY 5
#define ISR(name) void name(void)
#define USB_RX_ISR_vect receive
static uint8_t SREG;
static void cli(void) {SREG=0;}
static struct {uint8_t enable_flow_control;} xio;
static volatile bool rx_in_line;
static struct {uint8_t flag_xoff;} USB;
typedef struct {uint8_t CTRLA,DATA;} usart_t;
typedef struct {uint8_t OUTCLR,OUTSET;} port_t;
typedef struct {uint8_t fc_state_rx,fc_state_tx,fc_char_rx;usart_t *usart;port_t *port;} xioUsart_t;
static usart_t uart;
static port_t port;
static xioUsart_t USBu;
static unsigned count,normal;
static unsigned xio_get_rx_bufcount_usart(xioUsart_t *d) {(void)d;return count;}
static int xio_ctrl(int dev,uint32_t flags) {assert(dev==0);USB.flag_xoff=flags==XIO_XOFF;return 0;}
static void hw_request_hard_reset(void) {}
static void hw_hard_reset(void) {}
static void cm_request_feedhold(void) {}
static void cm_request_queue_flush(void) {}
static void cm_request_end_hold(void) {}
static void controller_request_enquiry(void) {}
'''
EXERCISE=r'''
static void init(unsigned old,unsigned tx,unsigned rx,unsigned high,unsigned saved) {
    memset(&uart,0,sizeof(uart));memset(&port,0,sizeof(port));
    USBu=(xioUsart_t){rx,tx,0,&uart,&port};xio.enable_flow_control=old;
    USB.flag_xoff=old!=FLOW_CONTROL_OFF;count=high?201:0;SREG=saved;
}
static void change(unsigned mode) {nvObj_t nv={mode};assert(set_ex(&nv)==STAT_OK);}
int main(void) {
    for(unsigned old=0;old<3;old++)for(unsigned mode=0;mode<3;mode++)
    for(unsigned tx=0;tx<2;tx++)for(unsigned rx=0;rx<2;rx++)
    for(unsigned high=0;high<2;high++)for(unsigned saved=0;saved<=128;saved+=128) {
        init(old,tx,rx,high,saved);change(mode);
        assert(SREG==saved&&xio.enable_flow_control==mode);
        assert(USB.flag_xoff==(mode!=FLOW_CONTROL_OFF));
        if(old==mode) {assert(USBu.fc_state_tx==tx&&USBu.fc_state_rx==rx);continue;}
        assert(USBu.fc_state_tx==FC_IN_XON&&port.OUTCLR==USB_RTS_bm);
        assert(uart.CTRLA==CTRLA_RXON_TXON);
        assert(USBu.fc_state_rx==(mode!=FLOW_CONTROL_OFF&&high?FC_IN_XOFF:FC_IN_XON));
        if(mode==FLOW_CONTROL_RTS&&high) assert(port.OUTSET==USB_RTS_bm);
        if(mode==FLOW_CONTROL_XON&&high) assert(USBu.fc_char_rx==XOFF);
        else if(old==FLOW_CONTROL_XON&&rx==FC_IN_XOFF) assert(USBu.fc_char_rx==XON);
        else assert(USBu.fc_char_rx==NUL);
    }
    init(1,1,1,0,128);change(0);change(2);assert(USBu.fc_char_rx==XON);
    init(1,0,0,0,128);USBu.fc_char_rx=XOFF;change(0);assert(USBu.fc_char_rx==XON);
    init(1,1,1,0,128);nvObj_t invalid={3};assert(set_ex(&invalid)==STAT_INPUT_VALUE_RANGE_ERROR);
    assert(SREG==128&&xio.enable_flow_control==1&&USBu.fc_state_tx==1);
    for(unsigned mode=0;mode<3;mode++) {
        init(mode,0,0,0,128);normal=0;uart.DATA=XOFF;receive();
        assert(USBu.fc_state_tx==(mode==FLOW_CONTROL_XON?FC_IN_XOFF:FC_IN_XON));
        assert(normal==(mode==FLOW_CONTROL_XON?0:1));
    }
    return 0;
}
'''
class FlowControlTransition(unittest.TestCase):
    def test_mode_change_releases_old_flow_and_preserves_new_backpressure(self):
        cfg=(ROOT/'firmware/tinyg/config_app.c').read_text()
        usart=(ROOT/'firmware/tinyg/xio/xio_usart.c').read_text()
        usb=(ROOT/'firmware/tinyg/xio/xio_usb.c').read_text()
        code=PRELUDE+function(usart,r'void xio_xoff_usart\(xioUsart_t \*dx\)\s*\n')
        code+=function(cfg,r'static stat_t _set_comm_helper\(nvObj_t \*nv, uint32_t yes, uint32_t no\)\s*\n')
        code+=function(cfg,r'static stat_t set_ex\(nvObj_t \*nv\)[^;\n]*\n')
        rx=function(usb,r'ISR\(USB_RX_ISR_vect\)[^\n]*\n')
        code+=rx[:rx.index('\n\t// normal character path')]+ '\nnormal++;\n}' +EXERCISE
        with tempfile.TemporaryDirectory() as tmp:
            src=pathlib.Path(tmp)/'flow.c';exe=pathlib.Path(tmp)/'flow';src.write_text(code)
            subprocess.run(['cc','-std=c99','-Wall','-Wextra','-Wno-unused-function','-fsanitize=address,undefined',str(src),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
if __name__=='__main__': unittest.main()
