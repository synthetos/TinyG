/* Host-only logical scheduler. Physical execution, clocks, pins and EEPROM are stubbed.
 * Read README.md for the declared machine configuration and coverage limits. */
#include "tinyg.h"
#include "config.h"
#include "controller.h"
#include "canonical_machine.h"
#include "planner.h"
#include "plan_arc.h"
#include "stepper.h"
#include "encoder.h"
#include "report.h"
#include "gcode_parser.h"
#include "util.h"

#include "runtime.h"
#include "xmega/xmega_rtc.h"
uint8_t SREG;
stPrepSingleton_t st_pre;
rtClock_t rtc;
controller_t cs;

stat_t status_code;
stConfig_t st_cfg;
stat_t adc_report(void){return STAT_OK;}
stat_t cm_homing_cycle_start(void){fprintf(stderr,"UNMODELED homing\n");return STAT_FUNCTION_IS_STUBBED;}
stat_t cm_homing_cycle_start_no_set(void){return cm_homing_cycle_start();}
stat_t cm_straight_probe(const float t[],const bool f[]){fprintf(stderr,"UNMODELED probe\n");return STAT_FUNCTION_IS_STUBBED;}
static void finish(void);
stat_t controller_request_motion_sync(void){finish();return STAT_OK;}
void en_set_encoder_steps(uint8_t m,float x){}
void kn_inverse_kinematics(const float travel[],float steps[]){for(int m=0;m<MOTORS;m++)steps[m]=0;}
void gpio_set_bit_on(uint8_t n){}
void gpio_set_bit_off(uint8_t n){}
void mp_exit_hold_state(void){}
static nvObj_t dummy;
nvObj_t *nv_add_message_P(const char *s){return &dummy;}
nvObj_t *nv_add_object(const char *s){return &dummy;}
nvObj_t *nv_add_string(const char *s,const char *v){return &dummy;}
stat_t nv_persist(nvObj_t *nv){return STAT_OK;}
void qr_request_queue_report(int8_t n){}
stat_t rpt_exception(stat_t st,const char *s){fprintf(stderr,"EXCEPTION %u %s\n",st,s?s:"");return st;}
stat_t rpt_exception_P(stat_t st,const char *s){return rpt_exception(st,s);}
stat_t sr_request_status_report(uint8_t x){return STAT_OK;}
bool st_aline_in_flight(void){return false;}
void st_prep_command(void *b){mp_runtime_command((mpBuf_t*)b);}
void st_prep_dwell(float t){}
void st_prep_null(void){}
void st_cancel_dwell(void){}            /* N51: flush/job kill discard a dwell; replays never flush */
void st_request_exec_move(void){}
uint8_t st_runtime_isbusy(void){return false;}
void stepper_reset(void){}
stat_t mp_exec_aline(mpBuf_t *b){return STAT_OK;}

#include "pwm.h"
pwmSingleton_t pwm;
stat_t pwm_set_duty(uint8_t c,float d){return STAT_OK;}
void gpio_init_outputs(void){}
stat_t pwm_set_freq(uint8_t c,float f){return STAT_OK;}

#include "json_parser.h"
#include "text_parser.h"
extern int json_status;
#include <signal.h>
#include <unistd.h>
#include <time.h>
unsigned source_line;
bool host_config_dispatch;
static unsigned blocks, bad, slow;
static bool naive_seen[PLANNER_BUFFER_POOL_SIZE];
static double maximum_ratio;
static unsigned block_line[PLANNER_BUFFER_POOL_SIZE];
static void timeout_line(int sig){fprintf(stderr,"TIMEOUT line=%u\n",source_line);_Exit(124);}
static void check_plan(mpBuf_t *b) {
 if(b->block_type!=BLOCK_TYPE_ALINE)return;
 unsigned n=block_line[b-mb.bf];
 #define FIELD(f) do {if(!isfinite(b->f)||b->f<0){printf("BAD %u %s %.9g\n",n,#f,(double)b->f);bad++;}}while(0)
 FIELD(length);
 FIELD(head_length);
 FIELD(body_length);
 FIELD(tail_length);
 FIELD(entry_velocity);
 FIELD(cruise_velocity);
 FIELD(exit_velocity);
 FIELD(entry_vmax);
 FIELD(cruise_vmax);
 FIELD(exit_vmax);
 FIELD(delta_vmax);
 FIELD(braking_velocity);
 FIELD(jerk);
 FIELD(recip_jerk);
 FIELD(cbrt_jerk);
 FIELD(gm.move_time);
 FIELD(gm.minimum_time);
 if(!isfinite(b->naiive_move_time) && !naive_seen[b-mb.bf]){naive_seen[b-mb.bf]=true;printf("NAIVE_INF %u %.9g %.9g %.9g\n",n,(double)b->naiive_move_time,(double)b->entry_velocity,(double)b->exit_velocity);}
 for(int a=0;a<AXES;a++){if(!isfinite(b->unit[a])||!isfinite(b->gm.target[a])){printf("BAD %u vector%u\n",n,a);bad++;}}
 #undef FIELD
 double t=0;
 if(b->head_length>0)t+=2*b->head_length/(b->entry_velocity+b->cruise_velocity);
 if(b->body_length>0)t+=b->body_length/b->cruise_velocity;
 if(b->tail_length>0)t+=2*b->tail_length/(b->exit_velocity+b->cruise_velocity);
 double linear=0,rotary=0,kin=0;
 for(int a=0;a<AXES;a++){
  double d=(double)b->unit[a]*b->length;
  if(a<3)linear+=d*d;else rotary+=d*d;
  double rate=b->gm.motion_mode==MOTION_MODE_STRAIGHT_TRAVERSE?cm.a[a].velocity_max:cm.a[a].feedrate_max;
  if(rate>0 && fabs(d)/rate>kin)kin=fabs(d)/rate;
 }
 for(int m=0;m<MOTORS;m++){
  int a=st_cfg.mot[m].motor_map;if(a>=AXES || cm.a[a].axis_mode==AXIS_INHIBITED)continue;
  double motor=fabs((double)b->unit[a]*b->length)*st_cfg.mot[m].steps_per_unit/(60.0*50000*0.9);
  if(motor>kin)kin=motor;
 }
 if(b->gm.motion_mode!=MOTION_MODE_STRAIGHT_TRAVERSE){
  double feed=b->gm.feed_rate_mode==INVERSE_TIME_MODE?b->gm.feed_rate:sqrt(linear>0?linear:rotary)/b->gm.feed_rate;
  if(feed>kin)kin=feed;
 }
 if(kin<MIN_SEGMENT_TIME)kin=MIN_SEGMENT_TIME;
 if(b->gm.move_time>10*kin){printf("BASE_SLOW %u %.9g %.9g\n",n,(double)b->gm.move_time,kin);slow++;}
 double expect=kin+4*cbrt(b->length/(2*b->jerk));
 if(!isfinite(t)||t<0){printf("BAD %u duration %.9g\n",n,t);bad++;}
 if(t/expect>maximum_ratio)maximum_ratio=t/expect;
 if(t>10*expect){printf("SLOW %u %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n",n,t,expect,(double)b->length,(double)b->entry_velocity,(double)b->cruise_velocity,(double)b->exit_velocity,(double)b->jerk);slow++;}
}
static void take_one(void)
{
    mpBuf_t *b = mp_get_run_buffer();
    if (!b) return;
    if (b->block_type == BLOCK_TYPE_ALINE) {
        check_plan(b);
        blocks++;
        memcpy(&mr.gm, &b->gm, sizeof(mr.gm));
        memcpy(mr.unit, b->unit, sizeof(mr.unit));
        memcpy(mr.position, b->gm.target, sizeof(mr.position));
        mr.exit_velocity = b->exit_velocity;
        if (mp_free_run_buffer()) cm_cycle_end();
    } else if (b->block_type == BLOCK_TYPE_COMMAND) {
        mp_runtime_command(b);
    } else {
        if (mp_free_run_buffer()) cm_cycle_end();
    }
    // Supply the operator's continuation for M0/M1; no physical wait is simulated.
    if (cm.program_paused) {
        cm.program_paused = false;
        cm.hold_state = FEEDHOLD_OFF;
        cm.motion_state = MOTION_STOP;
    }
}

static void mark_new(void)
{
    for (int i = 0; i < PLANNER_BUFFER_POOL_SIZE; i++) {
        if (mb.bf[i].buffer_state && block_line[i] == 0) block_line[i] = source_line;
    }
}

static void drain_one(void)
{
    int slot = mb.r - mb.bf;
    uint8_t before = mb.buffers_available;
    take_one();
    if (mb.buffers_available <= before) {
        fprintf(stderr, "DRAIN_STALLED\n");
        exit(125);
    }
    block_line[slot] = 0;
    naive_seen[slot] = false;
}

static void room(void)
{
    while (mb.buffers_available < PLANNER_BUFFER_HEADROOM) drain_one();
}

static void finish(void)
{
    while (mp_has_runnable_buffer()) drain_one();
}

void st_refresh_move_time(void);
static void setup(void){
 planner_init();
 cm.default_units_mode=MILLIMETERS;cm.default_coord_system=G54;cm.default_select_plane=CANON_PLANE_XY;cm.default_path_control=PATH_CONTINUOUS;cm.default_distance_mode=ABSOLUTE_MODE;
 cm.chordal_tolerance=.01f;cm.junction_acceleration=100000;
 for(int a=0;a<AXES;a++){cm.a[a].axis_mode=a<4?AXIS_STANDARD:AXIS_DISABLED;cm.a[a].velocity_max=cm.a[a].feedrate_max=a<3?16000:3600;cm_set_axis_jerk(a,500);cm.a[a].junction_dev=.05f;cm.a[a].radius=1;cm.a[a].travel_min=-1;cm.a[a].travel_max=-1;}
 for(int m=0;m<MOTORS;m++){st_cfg.mot[m].motor_map=m;st_cfg.mot[m].steps_per_unit=1280;st_cfg.mot[m].units_per_step=1.0f/1280;}
 st_refresh_move_time();st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;SREG=128;canonical_machine_init();
 char b[]="G21 G90 G17 G94 G92.1 G54";if(gc_gcode_parser(b))abort();finish();char f[]="F600";if(gc_gcode_parser(f))abort();
}
static int detector_selftest(void)
{
    mpBuf_t *b = &mb.bf[0];
    memset(b, 0, sizeof(*b));
    b->block_type = BLOCK_TYPE_ALINE;
    b->length = b->body_length = b->unit[0] = 1;
    b->gm.feed_rate = 600;
    b->gm.motion_mode = MOTION_MODE_STRAIGHT_FEED;
    b->gm.feed_rate_mode = UNITS_PER_MINUTE_MODE;
    b->jerk = 500000000;
    b->cruise_velocity = NAN;
    check_plan(b);
    if (!bad) return 1;
    b->cruise_velocity = 1e-9f;
    check_plan(b);
    return slow ? 0 : 1;
}

int main(int argc, char **argv)
{
    signal(SIGALRM, timeout_line);
    setvbuf(stdout, NULL, _IOLBF, 0);
    setup();
    if (argc > 1 && !strcmp(argv[1], "--detector-selftest")) return detector_selftest();

    char *line = NULL;
    size_t cap = 0;
    ssize_t len;
    clock_t start = clock();
    while ((len = getline(&line, &cap, stdin)) >= 0) {
        source_line++;
        alarm(5);
        while (len && (line[len-1] == '\n' || line[len-1] == '\r')) line[--len] = 0;
        room();
        char *c = line;
        while (*c == ' ' || *c == '\t') c++;
        if (*c == '{') {
            host_config_dispatch = true;
            json_status = -1;
            json_parser(c);
            host_config_dispatch = false;
            printf("JSON %u %d\n", source_line, json_status);
            alarm(0);
            continue;
        }
        if (*c == '$') {
            host_config_dispatch = true;
            stat_t st = text_parser(c);
            host_config_dispatch = false;
            printf("TEXT %u %u\n", source_line, st);
            alarm(0);
            continue;
        }
        stat_t status = gc_gcode_parser(c);
        mark_new();
        while (arc.run_state != BLOCK_IDLE && cm_is_alarmed() == STAT_OK) {
            room();
            stat_t st = cm_arc_callback();
            mark_new();
            if (st != STAT_OK && st != STAT_NOOP && st != STAT_EAGAIN) {
                printf("ARC_STATUS %u %u\n", source_line, st);
                break;
            }
        }
        for (int i = 0; i < PLANNER_BUFFER_POOL_SIZE; i++) {
            if (mb.bf[i].buffer_state) check_plan(&mb.bf[i]);
        }
        printf("LINE %u %u %u %u %.9g %.9g %.9g %.9g\n",
               source_line, status, cm.machine_state, cm.cycle_state,
               (double)cm.gmx.position[0], (double)cm.gmx.position[1],
               (double)cm.gmx.position[2], (double)cm.gmx.position[3]);
        if (cm_is_alarmed() != STAT_OK) printf("STATE %u %u\n", source_line, cm.machine_state);
        alarm(0);
    }
    alarm(5);
    finish();
    alarm(0);
    free(line);
    printf("DONE %u %u %u %u %.6f %.9g\n", source_line, blocks, bad, slow,
           (double)(clock()-start)/CLOCKS_PER_SEC, maximum_ratio);
    return 0;
}

bool xio_rx_data_pending(void){return false;}
bool xio_usb_is_quiet(void){return true;}
bool xio_usb_tx_ready(void){return true;}
bool xio_usb_try_send_frame(const char *s){return true;}
bool xio_usb_try_send_line(const char *s){return true;}
uint8_t st_fl_record_pending(void){return false;}
