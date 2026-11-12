"""Host I/O adapter for 449.01. No planner or motion algorithm is replaced.

449 has no runtime/advisor module or host support. JSON/settings replay is explicitly
unsupported here: machine settings are installed by the same host profile as 450.
"""
import re


def prepare(out, fw):
    p = out / 'driver.c'
    s = p.read_text().replace('#include "runtime.h"', '')
    s = re.sub(r'    // Supply the operator.*?\n    }\n', '', s, flags=re.S)
    s = s.replace('if (mb.buffers_available <= before)', 'if (mb.buffers_available <= before && mb.r == &mb.bf[slot])')
    s = s.replace('extern int json_status;', 'int json_status;')
    s += '\nvoid json_parser(char *s){json_status=STAT_FUNCTION_IS_STUBBED;}\nstat_t text_parser(char *s){return STAT_FUNCTION_IS_STUBBED;}\n'
    s += '\nvoid qr_init_queue_report(void){}\nvoid xio_flush_read(void){}\n'
    p.write_text(s)
    (out / 'feedlimit.h').write_text('/* 449 has no advisor. */\n')
