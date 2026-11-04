/* Logical scheduler fixture. Real planner/zoid math is linked; step pulses are not simulated. */
#include <assert.h>
#include <avr/interrupt.h>
#include "tinyg.h"
#include "config.h"
#include "controller.h"
#include "canonical_machine.h"
#include "planner.h"
#include "plan_arc.h"
#include "kinematics.h"
#include "stepper.h"
#include "encoder.h"
#include "report.h"
#include "xmega/xmega_rtc.h"

cmSingleton_t cm;
stPrepSingleton_t st_pre;
float st_get_move_time(const float axis_length[]) { return 0; }
rtClock_t rtc;
controller_t cs;

uint8_t harness_path_control = PATH_CONTINUOUS;
uint8_t SREG;
static bool pending_exec, runtime_busy;
static unsigned executed;
static float captured_exit;
stat_t cm_is_alarmed(void) { return cm.machine_state==MACHINE_ALARM; }
void st_prep_null(void) {}

uint8_t cm_get_path_control(const GCodeState_t *gcode_state) { return (harness_path_control); }
void cm_set_motion_state(const cmMotionState motion_state) { cm.motion_state = motion_state; }
void cm_cycle_end(void) {}
void cm_abort_arc(void) {}
stat_t cm_panic_P(const stat_t status, const char *msg_P)
{
	fprintf(stderr, "cm_panic_P(%d)\n", status);
	return (status);
}

void en_set_encoder_steps(uint8_t motor, float steps) {}
void kn_inverse_kinematics(const float travel[], float steps[]) {}

stat_t rpt_exception(stat_t status, const char *msg)
{
	fprintf(stderr, "rpt_exception(%d)\n", status);
	return (status);
}
void qr_request_queue_report(int8_t buffers) {}

uint8_t st_runtime_isbusy(void) { return runtime_busy; }
void st_request_exec_move(void) { pending_exec=true; }
void st_prep_command(void *bf) {}
void st_prep_dwell(float microseconds) {}

stat_t mp_exec_aline(mpBuf_t *bf) {
    bf->replannable=false; executed++; captured_exit=bf->exit_velocity;
    return STAT_OK;
}

void stepper_reset(void) {}
stat_t rpt_exception_P(stat_t status, const char *msg) { return rpt_exception(status, msg); }
char text_item[256];

static void setup(void) {
    memset(&cm,0,sizeof(cm)); memset(&st_pre,0,sizeof(st_pre));
    planner_init(); pending_exec=runtime_busy=false; executed=0; captured_exit=-1;
    cm.machine_state=MACHINE_READY; cm.hold_state=FEEDHOLD_OFF;
    cm.cycle_state=CYCLE_MACHINING; st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;
    for(int a=0;a<AXES;a++) {
        cm.a[a].velocity_max=cm.a[a].feedrate_max=1000;
        cm.a[a].jerk_max=50; cm.a[a].recip_jerk=1.0f/(50*JERK_MULTIPLIER);
        cm.a[a].junction_dev=.05;
    }
    cm.junction_acceleration=100000;
    rtc.sys_ticks=100; SREG=0x95;
}
static void move(float x) {
    GCodeState_t gm={0}; gm.motion_mode=MOTION_MODE_STRAIGHT_FEED;
    gm.feed_rate_mode=UNITS_PER_MINUTE_MODE; gm.feed_rate=300; gm.target[0]=x;
    assert(mp_aline(&gm)==STAT_OK); assert(SREG==0x95);
}
static void exec_pending(void) {
    if(pending_exec) { pending_exec=false; mp_exec_move(); }
}
int main(void) {
    setup(); move(2); exec_pending();
    assert(executed==0); // stock starts and freezes this block at exit_velocity=0
    assert(mp_exec_move()==STAT_NOOP); // even a stray exec interrupt cannot freeze it
    assert(mb.r->replannable && mb.r->buffer_state==MP_BUFFER_QUEUED);
    move(4); exec_pending();
    assert(executed==1 && captured_exit>240); // actual lookahead joins two collinear lines
#ifdef HAS_STARTUP
    setup(); move(2); rtc.sys_ticks+=49; mp_startup_callback(); exec_pending(); assert(!executed);
    rtc.sys_ticks++; mp_startup_callback(); exec_pending(); assert(executed==1 && captured_exit==0);
    setup(); rtc.sys_ticks=UINT32_MAX-20; move(2); rtc.sys_ticks+=49;
    mp_startup_callback(); exec_pending(); assert(!executed);
    rtc.sys_ticks++; mp_startup_callback(); exec_pending(); assert(executed==1 && SREG==0x95);
    setup(); move(2); cm_request_feedhold();
    assert(!mp_startup_is_pending() && cm.hold_state==FEEDHOLD_HOLD && cm.motion_state==MOTION_HOLD);
    rtc.sys_ticks+=100; mp_startup_callback(); assert(!pending_exec && !executed);
    mp_flush_planner(); assert(!mp_startup_is_pending());
    cm.hold_state=FEEDHOLD_OFF; move(4); assert(mp_startup_is_pending());
    mp_flush_planner(); rtc.sys_ticks+=100; mp_startup_callback(); assert(!pending_exec);
    setup(); move(2); cm.machine_state=MACHINE_ALARM; rtc.sys_ticks+=100;
    mp_startup_callback(); assert(!pending_exec && !executed);
    setup(); move(2); assert(mp_get_write_buffer()!=NULL); // a reservation is not a second committed block
    assert(mp_startup_is_pending() && !pending_exec);
    setup(); move(2); assert(mp_dwell(.1f)==STAT_OK); assert(!mp_startup_is_pending() && pending_exec);
    setup(); cm.cycle_state=CYCLE_HOMING; move(2); assert(pending_exec && !mp_startup_is_pending());
    setup(); cm.cycle_state=CYCLE_PROBE; move(2); assert(pending_exec && !mp_startup_is_pending());
    setup(); cm.cycle_state=CYCLE_JOG; move(2); assert(pending_exec && !mp_startup_is_pending());
    setup(); runtime_busy=true; move(2); assert(pending_exec && !mp_startup_is_pending());
    setup(); st_pre.buffer_state=PREP_BUFFER_OWNED_BY_LOADER; move(2);
    assert(pending_exec && !mp_startup_is_pending());
    setup(); assert(mp_dwell(.1f)==STAT_OK); assert(pending_exec && !mp_startup_is_pending());
    SREG=0x15; mp_startup_callback(); assert(SREG==0x15); mp_hold_startup(); assert(SREG==0x15);
#endif
    return 0;
}
