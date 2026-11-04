"""N28: status reports wait for elapsed time across the 32-bit tick wrap. TinyG configuration wiki SI/SV: si is the automatic-report interval; sv=0 disables reports. Intervals are bounded below half the tick range; ASAP and timed requests retain first-request-wins behaviour."""
from pathlib import Path
import subprocess,tempfile,unittest
from source_extract import function
FW=Path(__file__).resolve().parents[2]/'firmware/tinyg'
class ReportTiming(unittest.TestCase):
 def test_real_setter_request_and_callback(self):
  code=r'''
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#define __AVR 1
#define cli() (SREG &= ~128)
#define STAT_OK 0
#define STAT_NOOP 3
#define STAT_INPUT_VALUE_RANGE_ERROR 110
#define STATUS_REPORT_MIN_MS 100
#define STATUS_REPORT_MAX_MS 2147483647UL
#define SR_OFF 0
#define SR_FILTERED 1
#define SR_VERBOSE 2
#define SR_REQUEST_ASAP 0
#define SR_REQUEST_ASAP_UNFILTERED 1
#define SR_REQUEST_TIMED 2
#define SR_REQUEST_TIMED_UNFILTERED 3
#define TEXT_INLINE_PAIRS 0
#define JSON_OBJECT 0
typedef int stat_t;
typedef struct {uint32_t value_int;} nvObj_t;
static struct {uint32_t status_report_interval,status_report_systick,status_report_delay;int status_report_request,status_report_verbosity;}sr;
static uint8_t SREG=128;
static uint32_t now;
static int reports;
static bool check_mask;
static uint32_t SysTickTimer_getValue(void){if(check_mask)assert(!(SREG&128));return now;}
static void _populate_unfiltered_status_report(char*k){}
static bool _populate_filtered_status_report(void){return true;}
static void nv_print_list(int s,int t,int j){reports++;}
'''
  source=(FW/'report.c').read_text()
  for name in ['sr_set_si','sr_request_status_report','sr_status_report_callback']:
   code+=function(source,r'stat_t '+name+r'\([^;{}]*\)[^;{}]*(?=\{)')+'\n'
  code+=r'''
int main(void){
 nvObj_t n={.value_int=2147483648UL};sr.status_report_interval=250;
 assert(sr_set_si(&n)==110&&sr.status_report_interval==250);
 n.value_int=UINT32_MAX;assert(sr_set_si(&n)==110&&sr.status_report_interval==250);
 n.value_int=0;assert(sr_set_si(&n)==0&&n.value_int==100);
 sr.status_report_verbosity=SR_VERBOSE;
 uint32_t starts[]={100,0x80000010UL,0xfffffff0UL};
 for(int i=0;i<3;i++){
  sr.status_report_request=SR_OFF;now=starts[i];reports=0;
  assert(sr_request_status_report(SR_REQUEST_TIMED)==0);
  now=starts[i]+99;assert(sr_status_report_callback()==STAT_NOOP&&reports==0);
  now=starts[i]+100;assert(sr_status_report_callback()==0&&reports==1);
  assert(sr_status_report_callback()==STAT_NOOP&&reports==1);
 }
 check_mask=true;SREG=128;sr.status_report_request=SR_OFF;now=123;
 sr_request_status_report(SR_REQUEST_ASAP);assert(SREG==128);
 assert(sr_status_report_callback()==0&&SREG==128);
 SREG=0;sr_request_status_report(SR_REQUEST_ASAP_UNFILTERED);assert(SREG==0);
 assert(sr_status_report_callback()==0&&SREG==0);SREG=128;
 n.value_int=2147483647UL;assert(sr_set_si(&n)==0&&SREG==128);
 now=0x80000010UL;sr_request_status_report(SR_REQUEST_TIMED);now+=100;
 assert(sr_status_report_callback()==STAT_NOOP);
 now=0x80000010UL+2147483647UL;assert(sr_status_report_callback()==0);
}
'''
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'check.c';p.write_text(code);exe=p.with_suffix('')
   subprocess.run(['cc','-std=gnu99','-O1','-fsanitize=address,undefined',str(p),'-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
