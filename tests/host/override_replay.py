"""Inject a feed factor at an exec step into the real planner, exec and segment prep.

The step replay models completed segments, not AVR interrupt timing or a physical
board. The mailbox injection uses the same override setters as the realtime path.
Section numbers are the planner's HEAD=0, BODY=1 and TAIL=2. No trajectory or
planner fields are changed by the driver.
"""
from pathlib import Path
import argparse
import json
import subprocess

import step_replay


INJECTION = r'''
#include "override.h"
static unsigned long ov_tick, ov_at;
static unsigned ov_factor, ov_sent;
static int ov_section = -1, ov_interleave;
static unsigned ov_interleaved, ov_stopped;
static unsigned long ov_hold_at;
static unsigned ov_schedule, ov_move, ov_move_tick;
void host_checkpoint(unsigned move);
static void host_set_factor(unsigned factor)
{
    if(ov_set_enable(OV_FEED,true,true,true)!=OV_OK ||
       ov_set_factor(OV_FEED,factor/1000.0f,true,true)!=OV_OK)exit(124);
    printf("OVCHANGE %u %u %u\n",ov_move,ov_move_tick,factor);
}
static void host_move_begin(void)
{
    ov_move++;ov_move_tick=0;
    if(ov_schedule)host_set_factor(ov_schedule==1?(ov_move%2?500:1500):ov_factor);
}
void host_run_active(void);
void host_report(void);
void xio_flush_input(void){}
static void host_override_step(void)
{
    ov_tick++;ov_move_tick++;
    if(ov_schedule==1 && (ov_move_tick==100 || ov_move_tick==400 || ov_move_tick==800))
        host_set_factor(ov_move_tick==400?1000:(ov_move_tick==100?1500:500));
    if (ov_hold_at && ov_tick==ov_hold_at) cm_request_feedhold();
    if (!ov_at || ov_sent || ov_tick < ov_at || cm.motion_state != MOTION_RUN ||
        (ov_section >= 0 && (int)mr.section != ov_section)) return;
    if (ov_set_enable(OV_FEED, true, true, true) != OV_OK ||
        ov_set_factor(OV_FEED, ov_factor / 1000.0f, true, true) != OV_OK) {
        fprintf(stderr, "override injection refused\n"); exit(124);
    }
    ovSnapshot_t snapshot; ov_snapshot(&snapshot);
    printf("OVREQ %lu %d %u %lu\n", ov_tick, (int)mr.section,
           (unsigned)snapshot.feed, (unsigned long)snapshot.request);
    ov_sent = 1;
}
static void host_override_sequence(void)
{
    if (ov_interleave < 4 || ov_interleave==8 || !ov_interleaved) return;
    cm_feedhold_sequencing_callback();
    if (cm.hold_state == FEEDHOLD_HOLD || cm_is_alarmed() ||
        (ov_interleave==5 && cm.queue_flush_state==FLUSH_OFF && cm.motion_state==MOTION_STOP)) {
        if (!ov_stopped) {
            ov_stopped=1;
            printf("OVSTOP %lu %d %d\n",ov_tick,(int)cm.hold_state,(int)cm.machine_state);
            if (ov_interleave==4 || ov_interleave==9) cm_request_end_hold();
            else {
                host_run_active(); host_report(); exit(0);
            }
        }
    }
}
static void host_override_trace(void)
{
    printf("OVSEG %lu %d %.9g %.9g %.9g\n", ov_tick, (int)mr.section,
           (double)mr.segment_velocity, (double)mr.segment_time,
           (double)mr.gm.target[0]);
}
void host_override_taken(void)
{
    if (ov_interleave==9 && !ov_hold_at) ov_hold_at=ov_tick+80;
    printf("OVTAKE %lu %.9g\n",ov_tick,(double)mr.cruise_velocity);
}
void host_override_interleave(void)
{
    if (!ov_sent || ov_interleaved++ || !ov_interleave) return;
    if (ov_interleave==1) ov_set_factor(OV_FEED,1.0f,true,true);
    else if (ov_interleave==2) {
        for(int i=0;i<40;i++) {
            ov_tick++;
            st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;
            stat_t yielded = mp_exec_move();
            if(yielded!=STAT_OK && yielded!=STAT_EAGAIN)exit(123);
            host_override_trace();
        }
    } else if (ov_interleave==8) {
        mpBuf_t *old=mb.r;
        unsigned advance_guard=0;
        while(mb.r==old && old->buffer_state!=MP_BUFFER_EMPTY) {
            ov_tick++;st_pre.buffer_state=PREP_BUFFER_OWNED_BY_EXEC;
            stat_t yielded=mp_exec_move();
            if(yielded!=STAT_OK && yielded!=STAT_EAGAIN)exit(123);
            host_override_trace();
            if(++advance_guard>100000)exit(122);
        }
    } else if (ov_interleave==3) {
        ov_cancel();
    } else if (ov_interleave==4) cm_request_feedhold();
    else if (ov_interleave==5) {cm_request_feedhold();cm_request_queue_flush();}
    else if (ov_interleave==6) cm_request_job_kill();
    else if (ov_interleave==7) cm_alarm(STAT_ALARM,"host test");
    printf("OVINTERLEAVE %lu %d\n",ov_tick,ov_interleave);
}
'''


def _replace_once(text, old, new):
    if text.count(old) != 1:
        raise RuntimeError(f"override replay anchor changed: {old!r}")
    return text.replace(old, new)


def build(root, out):
    root, out = Path(root).resolve(), Path(out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    exe = step_replay.build(root, out)
    driver = (out / 'driver.c').read_text()
    driver = _replace_once(driver, 'bool st_aline_in_flight(void){return false;}',
                           'bool st_aline_in_flight(void){return cm.motion_state==MOTION_RUN || cm.motion_state==MOTION_HOLD;}')
    driver = _replace_once(driver, 'static void take_one(void)\n',
                           INJECTION + '\nstatic void take_one(void)\n')
    driver = _replace_once(driver, '            stat_t status=mp_exec_move();',
                           '            host_override_step();\n'
                           '            host_override_sequence();\n'
                           '            if(ov_interleave<3 || ov_interleave>=8 || !ov_interleaved)mp_override_callback();\n'
                           '            stat_t status=mp_exec_move();\n'
                           '            host_override_trace();')
    driver = _replace_once(driver, '        unsigned guard=0;',
                           '        host_move_begin();\n        unsigned guard=0;')
    driver = _replace_once(driver, '            if(++guard>1000000)abort();\n        }',
                           '            if(++guard>1000000)abort();\n        }\n        host_checkpoint(ov_move);')
    driver = _replace_once(driver, '    setup();',
                           '    setup(); ov_init();\n'
                           '    if(argc>2)ov_at=strtoul(argv[2],NULL,10);\n'
                           '    if(argc>3)ov_factor=(unsigned)strtoul(argv[3],NULL,10);\n'
                           '    if(argc>4)ov_section=atoi(argv[4]);\n'
                           '    if(argc>5)ov_interleave=atoi(argv[5]);\n'
                           '    if(argc>6)ov_schedule=atoi(argv[6]);')
    # Release a lone first move as the clock-driven startup callback would.
    driver = _replace_once(driver, 'static void finish(void)\n{\n',
                           'static void finish(void)\n{\n'
                           '    rtc.sys_ticks += 1024; mp_startup_callback();\n')
    (out / 'driver.c').write_text(driver)
    plan = (out/'plan_exec.c').read_text()
    plan = _replace_once(plan, '    // Revalidate after all floating-point planning,',
                         '    extern void host_override_interleave(void); host_override_interleave();\n'
                         '    // Revalidate after all floating-point planning,')
    plan = _replace_once(plan, '    ov_applied_request = ov_body.ticket.request;',
                         '    extern void host_override_taken(void); host_override_taken();\n'
                         '    ov_applied_request = ov_body.ticket.request;')
    (out/'plan_exec.c').write_text(plan)
    # Finish the last loaded segment before observing an endpoint. A real null
    # load folds its counted pulses into encoder_steps; the base replay's null
    # loader is a stub, so do that accounting explicitly here.
    step = (out/'step_engine.c').read_text()
    step += r"""
void host_checkpoint(unsigned move)
{
    host_run_active();
    for(unsigned m=0;m<MOTORS;m++) {
        ACCUMULATE_ENCODER(m);
        unsigned axis=st_cfg.mot[m].motor_map;
        printf("OVEND %u %u %.9g %lld %ld %.9g\n",move,m,
            (double)mr.target_steps[m],(long long)pulses[m],
            (long)en.en[m].encoder_steps,
            (double)(mp_get_runtime_absolute_position(axis)*st_cfg.mot[m].steps_per_unit));
    }
}
"""
    (out/'step_engine.c').write_text(step)
    args = json.loads((out / 'command.json').read_text())
    args[args.index('-lm'):args.index('-lm')] = [
        str(out / (name + '.c')) for name in ('plan_exec', 'kinematics', 'step_engine')]
    args.insert(1, '-DF_CPU=32000000UL')
    result = subprocess.run(args, capture_output=True, text=True)
    (out / 'override_build.log').write_text(result.stdout + result.stderr)
    if result.returncode:
        raise RuntimeError(result.stderr[-4000:])
    return exe


def run(exe, program, at=0, factor=1000, section=-1, batch=28, interleave=0, schedule=False):
    if at < 0 or not 50 <= factor <= 2000 or section not in (-1, 0, 1, 2):
        raise ValueError('invalid injection schedule')
    result = subprocess.run([str(exe), str(batch), str(at), str(factor), str(section), str(interleave), str(int(schedule))],
                            input=program, capture_output=True, text=True, timeout=60)
    if result.returncode:
        raise RuntimeError(f'replay exit {result.returncode}: {result.stderr[-2000:]}')
    rows = {'requests': [], 'segments': [], 'steps': [], 'taken': [], 'interleave': [], 'stops': [], 'ends': [], 'changes': []}
    for line in result.stdout.splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == 'OVREQ':
            rows['requests'].append(tuple(map(int, fields[1:])))
        elif fields[0] == 'OVTAKE':
            rows['taken'].append((int(fields[1]),float(fields[2])))
        elif fields[0] == 'OVINTERLEAVE':
            rows['interleave'].append(tuple(map(int,fields[1:])))
        elif fields[0] == 'OVSTOP':
            rows['stops'].append(tuple(map(int,fields[1:])))
        elif fields[0] == 'OVEND':
            rows['ends'].append((int(fields[1]),int(fields[2]),float(fields[3]),int(fields[4]),int(fields[5]),float(fields[6])))
        elif fields[0] == 'OVCHANGE':
            rows['changes'].append(tuple(map(int,fields[1:])))
        elif fields[0] == 'OVSEG':
            rows['segments'].append((int(fields[1]), int(fields[2]),
                                     *map(float, fields[3:])))
        elif fields[0] == 'STEPS':
            rows['steps'].append((int(fields[1]), float(fields[2]), int(fields[3])))
        elif fields[0] in ('EXEC', 'STATE', 'STUCK') or (fields[0] == 'LINE' and fields[-1] != '0'):
            raise RuntimeError(f'replay rejected the program: {line}')
    if not rows['segments'] or not rows['steps']:
        raise RuntimeError('replay produced no motion')
    return rows


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('program', type=Path)
    parser.add_argument('--source', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--work', type=Path, required=True)
    parser.add_argument('--at', type=int, default=20)
    parser.add_argument('--factor', type=int, default=500, help='Q1000 feed factor')
    parser.add_argument('--section', type=int, choices=(-1, 0, 1, 2), default=1)
    args = parser.parse_args()
    print(json.dumps(run(build(args.source, args.work), args.program.read_text(),
                         args.at, args.factor, args.section), indent=2))
