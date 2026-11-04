"""Execute every AVR F_INITIALIZE row's actual setter with its table default.

Hardware register effects are stubbed; validation, numeric setters, motor guard,
unit conversion, and the AVR-preprocessed table/enum values come from firmware.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from source_extract import function

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / 'firmware/tinyg'


class ConfigDefaults(unittest.TestCase):
    def test_every_initialized_default_is_accepted(self):
        args = ['avr-gcc', '-mmcu=atxmega192a3', '-DF_CPU=32000000UL', '-I', str(FW)]
        pre = subprocess.check_output(args + ['-E', '-P', str(FW/'config_app.c')], text=True)
        table = pre.split('const cfgItem_t cfgArray[]', 1)[1].split('\n};', 1)[0]
        rows = [row.split(',') for row in re.findall(r'\{(\s*"[^{}]+)\}', table)]
        self.assertGreater(len(rows), 300)
        enums = '\n'.join(re.findall(r'typedef enum\s*(?:\w+\s*)?\{[^{}]*\}\s*\w+\s*;', pre))
        macros = subprocess.check_output(args + ['-dM', '-E', str(FW/'config_app.c')], text=True)
        needed = ['F_CPU','AXIS_MODE_MAX_LINEAR','AXIS_MODE_MAX_ROTARY','TINYG_HARDWARE_VERSION_MAX',
                  'TINYG_HARDWARE_VERSION','MOTOR_TIMEOUT_SECONDS_MIN','MOTOR_TIMEOUT_SECONDS_MAX',
                  'FREQUENCY_DDA','EPSILON','JERK_MULTIPLIER','STATUS_REPORT_MIN_MS','SW_MODE_MAX_VALUE',
                  'STAT_OK','STAT_PARAMETER_IS_READ_ONLY','STAT_INPUT_VALUE_RANGE_ERROR','STAT_INPUT_EXCEEDS_MAX_VALUE','STAT_INPUT_LESS_THAN_MIN_VALUE','STAT_COMMAND_NOT_ACCEPTED','HW_VERSION_TINYGV8','SW_MODE_PROBE','XIO_DEV_USB','XIO_CRLF','XIO_NOCRLF','XIO_ECHO','XIO_NOECHO','XIO_XOFF','XIO_NOXOFF']
        needed += [f'M{m}_{field}' for m in range(1,7) for field in ('STEP_ANGLE','TRAVEL_PER_REV','MICROSTEPS')]
        defines = '\n'.join(line for line in macros.splitlines() if line.split()[1] in needed)
        code = r'''
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#define MAX_GEOMETRY_COORDINATE 1e18f
#define F_CONVERT 128
#define __AVR 1
#define PSTR(s) (s)
#define MM_PER_INCH 25.4
#define AXES 6
#define MODEL 0
#define _to_millimeters(a) (a)
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define cli() (SREG &= ~128)
'''+enums+'\n'+defines+r'''
typedef uint8_t stat_t;
typedef uint16_t index_t;
typedef struct {index_t index; char token[6],group[4]; union {float value_flt;uint32_t value_int;};uint8_t valuetype,precision;} nvObj_t;
typedef stat_t (*setter_t)(nvObj_t *);
typedef struct {char token[6];uint8_t flags,precision;setter_t set;void *target;float default_value;} row_t;
static row_t cfgArray[512];
static uint32_t storage[512];
static bool cfg_has_flag(index_t i,uint8_t f){return (cfgArray[i].flags & f)!=0;}
#define GET_TABLE_WORD(field) (cfgArray[nv->index].field)
struct {uint32_t dda_ticks_downcount;} st_run;
struct {int buffer_state;} st_pre;
static void st_refresh_move_time(void) {}
struct {float motor_power_timeout;struct {float step_angle,travel_rev,steps_per_unit,units_per_step;uint32_t microsteps;} mot[4];} st_cfg;
struct {int cycle_state;struct {int spindle_mode;} gm;struct {float jerk_max,recip_jerk;} a[6];} cm;
struct {bool configured;uint8_t probe_type;} sw;
static bool spindle_configured;
void spindle_reset(void){}
bool mp_runtime_is_idle(void){return true;}
struct {uint32_t status_report_interval;} sr;
struct {uint8_t json_verbosity;bool echo_json_footer,echo_json_messages,echo_json_configs,echo_json_linenum,echo_json_gcode_block;} js;
struct {uint8_t enable_cr,enable_echo,enable_flow_control;} xio;
static uint8_t SREG;
static bool stepper_configured;
bool mp_has_runnable_buffer(void){return true;}
void stepper_reset(void){}
void _set_hw_microsteps(int motor,int value){assert(motor>=0&&motor<4);}
void nv_add_message_P(const char *s){}
void switch_init(void){}
void _port_bindings(float hv){}
int xio_ctrl(int dev,uint32_t flags){return 0;}
/* E139 set_ex flow-control transition (only its compile surface matters here) */
#ifndef NUL
#define NUL 0
#endif
enum {XON=0x11, XOFF=0x13};
#ifndef ritorno
#define ritorno(a) do { int status_code=(a); if(status_code != STAT_OK) return status_code; } while(0)
#endif
#define USB_RTS_bm 0x10
#define CTRLA_RXON_TXON 0
#define XOFF_RX_HI_WATER_MARK 192
typedef struct {uint8_t OUTCLR;} fc_port_t;
typedef struct {uint8_t CTRLA;} fc_usart_t;
static fc_port_t fc_port; static fc_usart_t fc_usart;
static struct {uint8_t fc_char_rx, fc_state_rx, fc_state_tx; fc_port_t *port; fc_usart_t *usart;} USBu = {0,0,0,&fc_port,&fc_usart};
static uint16_t xio_get_rx_bufcount_usart(void *d){(void)d;return 0;}
static void xio_xoff_usart(void *d){(void)d;}
int cm_get_units_mode(int model){return MILLIMETERS;}
#define _to_millimeters(a) (a) /* defaults are loaded in millimeters */
static uint8_t _get_axis(index_t index){const char *p=strchr("xyzabc",cfgArray[index].token[0]);assert(p);return p-"xyzabc";}
static uint8_t _get_axis_type(index_t index){return _get_axis(index)>=3;}
'''
        selections = {
            'config.c': ['set_nul','set_not','set_ui8','set_u32','set_01','set_012','set_0123','set_data','set_flt'],
            'config_app.c': ['set_flu','_set_comm_helper','set_ec','set_ee','set_ex','set_gco','set_ma'],
            'stepper.c': ['_init_motor_geometry','_motor_geometry_valid','st_runtime_isbusy','_stepper_configuration_busy','_get_motor','_set_motor_steps_per_unit','st_set_sa','st_set_tr','st_set_mi','st_set_pm','st_set_mt'],
            'canonical_machine.c': ['cm_set_am','cm_set_axis_rate','cm_set_axis_jerk','cm_set_xjm','cm_set_xjh','cm_set_ja','cm_set_radius','cm_set_ct','cm_set_coord_value'],
            'hardware.c': ['hw_set_hv'], 'switch.c': ['sw_set_st','sw_set_sw','sw_set_prbp'],
            'spindle.c': ['spindle_set_polarity'],
            'json_parser.c': ['json_set_jv'], 'report.c': ['sr_set_si']}
        pwm_source = (FW/'pwm.c').read_text()
        code += '\n'+'\n'.join(line for line in pwm_source.splitlines()
                                  if re.match(r'#define PWM_(MAX|MIN)_FREQ\b',line))+'\n'
        selections['pwm.c'] = ['_pwm_frequency','pwm_set_frq']
        extracted = []
        for file, names in selections.items():
            source = (FW/file).read_text()
            for name in names:
                definition = function(source, r'(?:static )?(?:stat_t|uint8_t|int8_t|bool|void) '+name+r'\([^;{}]*\)[^\n{]*(?=\s*\{)')
                extracted.append(definition)
        code += '\n'.join(extracted)+'\nint main(void){\n'
        for i, row in enumerate(rows):
            if "0x10" not in row[2]:
                continue
            _, token, flags, precision, _, _, setter, target, default = map(str.strip, row)
            # Use real motor storage for its cached steps-per-unit calculations.
            motor = re.search(r'&st_cfg\.mot\[(.*?)\]\.(step_angle|travel_rev|microsteps)', target)
            target = '&st_cfg.mot[%s].%s' % motor.groups() if motor else '&storage[%d]' % i
            # Ignore non-initialized setters without linking unrelated command execution.
            code += f'if (({flags}) & 0x10) {{\n'
            if '0x10' in flags:
                code += f'cfgArray[{i}]=(row_t){{{token}, {flags}, {precision}, {setter}, {target}, {default}}};\n'
                code += f'nvObj_t nv={{.index={i}}};strcpy(nv.token,{token});\n'
                code += f'nv.value_flt=cfgArray[{i}].default_value;\n'
                code += f'if ((cfgArray[{i}].flags & 0xf)!=TYPE_FLOAT){{nv.value_int=(uint32_t)nv.value_flt;nv.valuetype=TYPE_INTEGER;}}else nv.valuetype=TYPE_FLOAT;\n'
                code += f'stat_t status=cfgArray[{i}].set(&nv);if(status){{fprintf(stderr,"default rejected: %s status=%u\\n",nv.token,status);failures++;}}count++;\n'
            code += '}\n'
        code = code.replace('int main(void){', 'int main(void){int count=0,failures=0;st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;_init_motor_geometry();' )
        code += 'assert(count>100);printf("Checked %d initialized defaults; %d rejected\\n",count,failures);return failures?1:0;}\n'
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory);(path/'check.c').write_text(code)
            subprocess.run(['cc','-Wall','-Werror','-Wno-unused-function','-fsanitize=address,undefined',str(path/'check.c'),'-lm','-o',str(path/'check')],check=True)
            subprocess.run([str(path/'check')],check=True)
