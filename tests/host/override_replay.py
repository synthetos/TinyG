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
static int ov_section = -1;
static void host_override_step(void)
{
    ov_tick++;
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
static void host_override_trace(void)
{
    printf("OVSEG %lu %d %.9g %.9g %.9g\n", ov_tick, (int)mr.section,
           (double)mr.segment_velocity, (double)mr.segment_time,
           (double)mr.gm.target[0]);
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
    driver = _replace_once(driver, 'static void take_one(void)\n',
                           INJECTION + '\nstatic void take_one(void)\n')
    driver = _replace_once(driver, '            stat_t status=mp_exec_move();',
                           '            host_override_step();\n'
                           '            stat_t status=mp_exec_move();\n'
                           '            host_override_trace();')
    driver = _replace_once(driver, '    setup();',
                           '    setup(); ov_init();\n'
                           '    if(argc>2)ov_at=strtoul(argv[2],NULL,10);\n'
                           '    if(argc>3)ov_factor=(unsigned)strtoul(argv[3],NULL,10);\n'
                           '    if(argc>4)ov_section=atoi(argv[4]);')
    # Release a lone first move as the clock-driven startup callback would.
    driver = _replace_once(driver, 'static void finish(void)\n{\n',
                           'static void finish(void)\n{\n'
                           '    rtc.sys_ticks += 1024; mp_startup_callback();\n')
    (out / 'driver.c').write_text(driver)
    args = json.loads((out / 'command.json').read_text())
    args[args.index('-lm'):args.index('-lm')] = [
        str(out / (name + '.c')) for name in ('plan_exec', 'kinematics', 'step_engine')]
    args.insert(1, '-DF_CPU=32000000UL')
    result = subprocess.run(args, capture_output=True, text=True)
    (out / 'override_build.log').write_text(result.stdout + result.stderr)
    if result.returncode:
        raise RuntimeError(result.stderr[-4000:])
    return exe


def run(exe, program, at=0, factor=1000, section=-1, batch=28):
    if at < 0 or not 50 <= factor <= 2000 or section not in (-1, 0, 1, 2):
        raise ValueError('invalid injection schedule')
    result = subprocess.run([str(exe), str(batch), str(at), str(factor), str(section)],
                            input=program, capture_output=True, text=True, timeout=60)
    if result.returncode:
        raise RuntimeError(f'replay exit {result.returncode}: {result.stderr[-2000:]}')
    rows = {'requests': [], 'segments': [], 'steps': []}
    for line in result.stdout.splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == 'OVREQ':
            rows['requests'].append(tuple(map(int, fields[1:])))
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
