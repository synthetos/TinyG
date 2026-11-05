"""ASan/UBSan harness builders. Only hardware/config execution is stubbed."""
from pathlib import Path
import os
import shutil
import re
import subprocess
from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FW = Path(os.environ.get('TINYG_FIRMWARE', ROOT / 'firmware/tinyg'))
FIXTURES = Path(__file__).with_name('input_cases')


def extract(source, name, filename):
    definition = function(source, r'(?:static )?(?:void|bool|char(?:\s*\*)?|nvObj_t\s*\*|stat_t|uint8_t)\s*' + re.escape(name) + r'\([^;{}]*\)(?=\s*(?://[^\n]*\n\s*)?\{)')
    line = source[:source.index(definition)].count('\n') + 1
    return f'\n#line {line} "{FW / filename}"\n' + definition


def build(kind, folder):
    pre = (FIXTURES / (kind + '_prelude.h')).read_text()
    pre += r'''
static int input_machine_state(void) {
    const char *state=getenv("TINYG_INPUT_STATE");
    if(state && !strcmp(state,"alarm")) return MACHINE_ALARM;
    if(state && !strcmp(state,"shutdown")) return MACHINE_SHUTDOWN;
    if(state && !strcmp(state,"panic")) return MACHINE_PANIC;
    return MACHINE_READY;
}
'''
    util = (FW / 'util.c').read_text()
    if kind == 'gcode':
        source = (FW / 'gcode_parser.c').read_text()
        code = pre + '\n'.join(x for x in source.splitlines() if x.startswith('#define SET_')) + '\n'
        for name in ['_normalize_gcode_block', '_get_next_gcode_word', '_point', '_takes_subcode', '_validate_gcode_block', '_parse_gcode_block']:
            code += extract(source, name, 'gcode_parser.c')
        code += r'''
#line 1 "input_fixture.c"
static int exercise(char *s, unsigned n) {
    memset(&cm,0,sizeof(cm)); executions=0; cm.gm.motion_mode=MOTION_MODE_STRAIGHT_FEED;
    char none=0, *com=&none,*msg=&none; uint8_t del;
    _normalize_gcode_block(s,&com,&msg,&del);

    int status=_parse_gcode_block(s);
    printf("%d %d ",status,executions);
    if(!*s) printf("-");
    for(char *p=s;*p;p++) printf("%02x",(unsigned char)*p);
    printf(" G=%u,%u",cm.gf.motion_mode,cm.gn.motion_mode);
    const char *axes="XYZABC";
    for(unsigned i=0;i<AXES;i++) printf(" %c=%u,%.9g",axes[i],cm.gf.target[i],cm.gn.target[i]);
    printf(" F=%u,%.9g S=%u,%.9g P=%u,%.9g L=%u,%u T=%u,%u N=%u,%u",
        cm.gf.feed_rate,cm.gn.feed_rate,cm.gf.spindle_speed,cm.gn.spindle_speed,
        cm.gf.parameter,cm.gn.parameter,cm.gf.L_word,cm.gn.L_word,
        cm.gf.tool_select,cm.gn.tool_select,cm.gf.linenum,cm.gn.linenum);
    printf(" I=%u,%.9g J=%u,%.9g K=%u,%.9g R=%u,%.9g",
        cm.gf.arc_offset[0],cm.gn.arc_offset[0],cm.gf.arc_offset[1],cm.gn.arc_offset[1],
        cm.gf.arc_offset[2],cm.gn.arc_offset[2],cm.gf.arc_radius,cm.gn.arc_radius);
    char *cursor=s,letter;float value;uint8_t decimals;int word_status;
    while(1) {
        char *before=cursor;
        word_status=_get_next_gcode_word(&cursor,&letter,&value,&decimals);
        if(word_status!=STAT_OK) break;
        assert(cursor>before && cursor<=s+strlen(s));
        printf(" w=%c,%.9g,%u,%ld,%ld",letter,value,decimals,(long)(before-s),(long)(cursor-s));
    }
    printf(" ws=%d\n",word_status);
    return status;
}
'''
    elif kind == 'text':
        source = (FW / 'text_parser.c').read_text()
        code = pre + extract((FW / 'canonical_machine.c').read_text(), 'cm_is_alarmed', 'canonical_machine.c') + extract(source, '_text_parser_kernal', 'text_parser.c') + extract(source, 'text_parser', 'text_parser.c')
        code += r'''
#line 1 "input_fixture.c"
static int exercise(char *s,unsigned n) {
    get_count=sets=persists=0; cm.machine_state=input_machine_state(); int status=text_parser(s);
    assert(persists==sets); assert(!status || !sets);
    printf("%d %u %u %.9g %u\n",status,get_count,sets,obj.value_flt,obj.value_int); return status;
}
'''
    else:
        source = (FW / 'json_parser.c').read_text()
        code = pre + extract((FW / 'canonical_machine.c').read_text(), 'cm_is_alarmed', 'canonical_machine.c')
        config = (FW / 'config.c').read_text()
        code += extract(config,'nv_reset_nv_list','config.c').replace('nv_reset_nv_list(', 'real_reset_nv_list(', 1)
        for name in ['nv_reset_nv','nv_next','nv_prev','nv_next_empty','set_nul','_refused_in_cycle','nv_set','nv_get','nv_copy_string']:
            code += extract(config,name,'config.c')
        for name in ['str_asciify','str_unescape','str2float','str2long']:
            code += extract(util,name,'util.c')
        for name in ['_is_key_char','_scan_tid','_scan_rt','_skip_whitespace','_skip_quote','_value_type','_tokenize_json_string','_next_token','_marshall_nv_pair','_motion_sync_envelope','_json_parser_kernal','json_parser']:
            code += extract(source,name,'json_parser.c')
        code += r'''
#line 1 "input_fixture.c"
static int exercise(char *s,unsigned n) {
    reset(); cm.machine_state=input_machine_state(); json_parser(s); int status=last_status; assert(status>=0);
    assert(nvl.magic_start==0x1357 && nvl.magic_end==0x2468);
    printf("%d %u %u %u %d %.9g %u %u ",status,executed,get_count,sets,
        observed.valuetype,observed.value_flt,observed.value_int,js.tid_present);
    if(observed.str && *observed.str) {
        for(char *p=observed.str;*p;p++) printf("%02x",(unsigned char)*p);
    } else printf("-");
    printf("\n");
    return status;
}
'''
    code += r'''
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>
static void timed_out(int signal_number) {
    (void)signal_number;
    const char msg[]="CASE_TIMEOUT\n";
    (void)write(2,msg,sizeof(msg)-1);_Exit(124);
}
int main(void) {
    signal(SIGALRM,timed_out);
    char hex[32770]; unsigned iteration=0;
    while(fgets(hex,sizeof(hex),stdin)) {
        unsigned n=strcspn(hex,"\r\n")/2;
        /* Line-mode callers own exactly the characters plus their terminating NUL. */
        unsigned pad=1;
        char *s=malloc(n+pad); assert(s); memset(s,0,n+pad);
        for(unsigned i=0;i<n;i++) {unsigned v; assert(sscanf(hex+2*i,"%2x",&v)==1);s[i]=(char)v;}
        struct itimerval timer={0};timer.it_value.tv_usec=200000;
        assert(setitimer(ITIMER_REAL,&timer,NULL)==0);
        exercise(s,n);
        timer.it_value.tv_usec=0;assert(setitimer(ITIMER_REAL,&timer,NULL)==0);
        fflush(stdout); free(s); iteration++;
    }
    return 0;
}
'''
    folder=Path(folder); (folder/'avr').mkdir(exist_ok=True)
    (folder/'avr/pgmspace.h').write_text('#define PROGMEM\n#define PSTR(s) (s)\n#define strcmp_P strcmp\n')
    c=folder/(kind+'.c');exe=folder/kind;c.write_text(code)
    subprocess.run([shutil.which('cc') or 'cc','-std=gnu99','-Wall','-Werror=format','-O1','-g','-funsigned-char','-fsanitize=address,undefined,float-cast-overflow','-fno-sanitize-recover=all','-Wno-macro-redefined','-include','stdio.h','-include','ctype.h','-D_POSIX_C_SOURCE=200809L','-I'+str(folder),'-I'+str(FW),str(c),'-lm','-o',str(exe)],check=True,capture_output=True,text=True,timeout=20)
    return exe


def run(exe, inputs, state="ready"):
    env = dict(os.environ, TINYG_INPUT_STATE=state)
    env["ASAN_OPTIONS"] = "symbolize=0:detect_leaks=0"
    return subprocess.run([str(exe)], input=''.join(x.hex()+'\n' for x in inputs),capture_output=True,text=True,timeout=30,env=env)
