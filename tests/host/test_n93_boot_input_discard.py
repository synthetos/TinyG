"""Boot's unread USART frames must not become an input-loss alarm after startup.

Run the actual application startup, USB RX ISR and line reader against a two-frame
USART FIFO. Register reads pop frames as on the device; EEPROM/other init work is
stubbed. No timing claim is made for this deterministic boundary test.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

from source_extract import function
from test_rx_overrun_damaged_line import PRELUDE

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / "firmware/tinyg"


class BootInputDiscard(unittest.TestCase):
    @unittest.expectedFailure
    def test_boot_overrun_is_discarded_but_runtime_loss_is_reported(self):
        usb = (FW / "xio/xio_usb.c").read_text()
        usart = (FW / "xio/xio_usart.c").read_text()
        code = PRELUDE.replace(
            "static void cm_request_feedhold(void) {}",
            "static unsigned controls; static void cm_request_feedhold(void) {controls++;}",
        ).replace("static void cm_request_job_kill(void) {}", "static void cm_request_job_kill(void) {controls++;}")
        code = code.replace("static void cm_request_end_hold(void) {}", "static void cm_request_end_hold(void) {controls++;}")
        code = code.replace("cm.queue_flush_input = true;", "controls++; cm.queue_flush_input = true;")
        code += r'''
#define USART_RXCIF_bm 0x80
static bool usb_boot_discard;
static bool loaded;
static unsigned fifo_count, fifo_reads;
static unsigned char fifo_data[2], fifo_status[2];
static unsigned char boot_uart_status(void) {
 return fifo_count ? USART_RXCIF_bm | fifo_status[0] : 0;
}
static unsigned char boot_uart_data(void) {
 assert(fifo_count);fifo_reads++;
 unsigned char c=fifo_data[0];
 fifo_data[0]=fifo_data[1];fifo_status[0]=fifo_status[1];fifo_count--;
 return c;
}
static void push(char c,unsigned char error) {
 assert(fifo_count<2);fifo_data[fifo_count]=c;fifo_status[fifo_count++]=error;
}
#define STD_IN 0
#define STD_OUT 1
#define STD_ERR 2
static void controller_init(int a,int b,int c) {(void)a;(void)b;(void)c;}
static void rt_init(void) {}
static void adc_init(void) {}
static void config_init(void) {loaded=true;}
static void stepper_init_complete(void) {}
static void switch_init(void) {}
static struct {bool configured;} sw;
static void canonical_machine_reset(void) {}
static void spindle_init(void) {}
static void spindle_init_outputs(void) {}
static void PMIC_SetVectorLocationToApplication(void) {assert(loaded);}
static void PMIC_EnableHighLevel(void) {}
static void PMIC_EnableMediumLevel(void) {}
static void PMIC_EnableLowLevel(void) {}
static void sei(void) {SREG|=128;}
'''
        for name in ("xio_get_rx_bufcount_usart", "xio_xon_usart", "xio_xoff_usart", "xio_fc_usart",
                     "_gets_helper", "xio_gets_usart"):
            code += function(usart, r'(?:static )?(?:buffer_t|void|int) ' + name + r'\([^;{}]*\)(?=[^;{}]*\{)') + "\n"
        for name in ("_usb_rx_lost", "xio_reset_usb_rx_buffers", "xio_usb_rtc_callback",
                     "xio_usb_startup_complete"):
            signature = r'(?:static )?void ' + name + r'\([^;{}]*\)(?=[^;{}]*\{)'
            if name not in usb:
                continue  # stock startup has no discard boundary
            fn = function(usb, signature)
            code += fn.replace("USBu.usart->STATUS", "boot_uart_status()") \
                      .replace("USBu.usart->DATA", "boot_uart_data()") + "\n"
        rx = function(usb, r'ISR\(USB_RX_ISR_vect\)')
        code += rx.replace("USBu.usart->STATUS", "boot_uart_status()") \
                  .replace("USBu.usart->DATA", "boot_uart_data()") + "\n"
        code += function((FW / "main.c").read_text(), r'static void _application_init_startup\(void\)(?=\s*\{)') + "\n"
        code += r'''
static char line[64];
static int read_line(void) {return xio_gets_usart(&USB,line,sizeof(line));}
static void send_byte(char c,unsigned char error) {push(c,error);USB_RX_ISR_vect();}
static void send(const char *s) {for(;*s;s++)send_byte(*s,0);}
static void boot(bool early,bool software) {
 memset(&USB,0,sizeof(USB));memset(&USBu,0,sizeof(USBu));
 USB.x=&USBu;USB.x_flow=xio_fc_usart;USBu.usart=&uart;USBu.port=&port;
 USBu.rx_buf_head=USBu.rx_buf_tail=1;
 loaded=false;controls=0;fifo_count=fifo_reads=0;SREG=0;
 usb_boot_discard=false;rx_quiet_ticks=0;usb_rx_seen=usb_activity=false;
 if(early){push('{',USART_BUFOVF_bm);push('"',0);}
 if(software){
  USBu.rx_buf_count=7;USBu.rx_buf_head=8;USB.len=4;USB.flag_in_line=true;
  rx_in_line=true;USBu.rx_lost=USBu.rx_lost_open=USBu.rx_line_damaged=USBu.rx_gap_report=true;
  USBu.rx_flush_marked=USBu.rx_lost_after_mark=true;
 }
 _application_init_startup();
 assert(loaded&&(SREG&128));
 while(fifo_count)USB_RX_ISR_vect(); // IRQs become runnable only AFTER config_init on stock
 assert(read_line()==XIO_EAGAIN);   // stock reports the boot-time loss here
 assert(!USBu.rx_lost&&!USBu.rx_lost_open&&!USBu.rx_gap_report&&!USBu.rx_line_damaged);
 assert(USBu.rx_buf_count==0&&!USBu.rx_flush_marked&&!USBu.rx_lost_after_mark);
}
static void query(void) {
 send("{\"fb\":null}\n");assert(read_line()==XIO_OK);
 assert(!strcmp(line,"{\"fb\":null}"));
}
int main(void) {
 boot(true,false);
 assert(fifo_reads==2); // both the overflow frame and the following queued frame were discarded
 send("X99!~%\004\n");assert(!controls); // never execute a suffix or realtime request from boot traffic
 assert(read_line()==XIO_EAGAIN);query();
 boot(true,true);
 xio_usb_rtc_callback();xio_usb_rtc_callback(); // a clean retry after a receive-idle gap needs no leading LF
 query();assert(!controls);
 boot(false,false);query(); // a first request AFTER a clean startup works immediately
 // Startup is over: genuine hardware and software losses MUST still be reported.
 send_byte('G',USART_BUFOVF_bm);send("1 X9\n");
 assert(read_line()==XIO_LINE_DAMAGED);
 boot(false,false);send("012345678901234567890123456789\n");
 assert(read_line()==XIO_LINE_DAMAGED);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            c, exe = Path(tmp) / "boot.c", Path(tmp) / "boot"
            c.write_text(code)
            subprocess.run(["cc", "-std=gnu99", "-Wall", "-Werror", "-Wno-unused-function",
                            "-Wno-unused-variable", "-fsanitize=address,undefined", "-fstrict-flex-arrays=3",
                            str(c), "-o", str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    unittest.main()
